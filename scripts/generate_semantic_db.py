#!/usr/bin/env python3
"""
generate_semantic_db.py — Semantic 3D Database Extractor (Phase 1)

Parses a Gazebo .world file and writes a flat dictionary (JSON) keyed by model
name, giving O(1) lookup of each object's pose and real metric size.

Data sources (nothing is hardcoded):
  * <state> poses (what Gazebo actually restores at load), falling back to the
    <model> <pose>, falling back to zeros.
  * Physical size: bounding box of the model's visual COLLADA (.dae) mesh,
    computed with an exact stdlib parser that applies BOTH the <asset><unit>
    scale AND every <node> transform. (trimesh does not reliably do this:
    bin.dae is in millimetres, mesh.dae stores its 0.001 scale in a node matrix.)

Units / frame: all positions and sizes are metres, angles radians, expressed in
the Gazebo "world" frame (frame_id = "world").

Usage:
    python3 scripts/generate_semantic_db.py --world Our_Structured_Warehouse \
        --output maps/Our_Structured_Warehouse_semantic.json
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

# Models that are not semantic targets (infrastructure / floor)
SKIP_MODELS = {"ground_plane"}

# No warehouse object is taller than this; larger means a unit bug.
MAX_SANE_DIMENSION_M = 50.0

IDENTITY = (1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
def find_project_root():
    """Walk up from this script until a directory containing Sim/ is found."""
    path = os.path.dirname(os.path.abspath(__file__))
    while path != os.path.dirname(path):
        if os.path.isdir(os.path.join(path, "Sim")):
            return path
        path = os.path.dirname(path)
    return None


def resolve_mesh_path(uri_text, sim_dir):
    """Resolve a model:// or file:// URI to an absolute filesystem path."""
    if uri_text.startswith("model://"):
        return os.path.join(sim_dir, "models", uri_text[len("model://"):])
    if uri_text.startswith("file://"):
        return os.path.join(sim_dir, uri_text[len("file://"):])
    return os.path.join(sim_dir, uri_text)


def extract_model_type_from_uri(uri_text):
    """'model://workcell_bin/meshes/bin.dae' -> 'workcell_bin'."""
    return uri_text.replace("model://", "").replace("file://", "").split("/")[0]


# ---------------------------------------------------------------------------
# COLLADA bounding box (stdlib only)
# ---------------------------------------------------------------------------
def _local(tag):
    """Strip the XML namespace from a tag."""
    return tag.rsplit("}", 1)[-1]


def _children(elem, name):
    return [c for c in elem if _local(c.tag) == name]


def _child(elem, name):
    found = _children(elem, name)
    return found[0] if found else None


def _floats(text):
    return [float(v) for v in text.split()]


def _mat_mul(a, b):
    """Multiply two row-major 4x4 matrices (tuples/lists of 16)."""
    return tuple(
        sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4))
        for r in range(4) for c in range(4)
    )


def _mat_apply(m, p):
    x, y, z = p
    return (
        m[0] * x + m[1] * y + m[2] * z + m[3],
        m[4] * x + m[5] * y + m[6] * z + m[7],
        m[8] * x + m[9] * y + m[10] * z + m[11],
    )


def _node_local_matrix(node):
    """Compose a COLLADA node's transform elements, in document order."""
    m = IDENTITY
    for el in node:
        tag = _local(el.tag)
        if tag == "matrix":
            t = tuple(_floats(el.text))
            if len(t) != 16:
                raise ValueError("COLLADA <matrix> must have 16 values")
        elif tag == "translate":
            x, y, z = _floats(el.text)
            t = (1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1)
        elif tag == "scale":
            x, y, z = _floats(el.text)
            t = (x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1)
        elif tag == "rotate":
            ax, ay, az, deg = _floats(el.text)
            n = math.sqrt(ax * ax + ay * ay + az * az)
            ax, ay, az = ax / n, ay / n, az / n
            c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            C = 1 - c
            t = (c + ax * ax * C, ax * ay * C - az * s, ax * az * C + ay * s, 0,
                 ay * ax * C + az * s, c + ay * ay * C, ay * az * C - ax * s, 0,
                 az * ax * C - ay * s, az * ay * C + ax * s, c + az * az * C, 0,
                 0, 0, 0, 1)
        elif tag in ("lookat", "skew"):
            raise ValueError(f"Unsupported COLLADA transform <{tag}>")
        else:
            continue
        m = _mat_mul(m, t)
    return m


def _geometry_positions(root):
    """Map geometry id -> list of (x, y, z) vertex positions (raw units)."""
    geoms = {}
    for geom in root.iter():
        if _local(geom.tag) != "geometry":
            continue
        mesh = _child(geom, "mesh")
        if mesh is None:
            continue
        sources = {}
        for src in _children(mesh, "source"):
            arr = _child(src, "float_array")
            if arr is None or not arr.text:
                continue
            stride = 3
            tc = _child(src, "technique_common")
            acc = _child(tc, "accessor") if tc is not None else None
            if acc is not None:
                stride = int(acc.get("stride", "3"))
            sources[src.get("id")] = (_floats(arr.text), stride)

        points = []
        for verts in _children(mesh, "vertices"):
            for inp in _children(verts, "input"):
                if inp.get("semantic") != "POSITION":
                    continue
                data, stride = sources[inp.get("source").lstrip("#")]
                if stride < 3:
                    raise ValueError("POSITION source stride < 3")
                for i in range(0, len(data) - stride + 1, stride):
                    points.append((data[i], data[i + 1], data[i + 2]))
        geoms[geom.get("id")] = points
    return geoms


def get_mesh_dimensions(dae_path):
    """Return {'width_x','length_y','height_z'} in metres for a .dae file.

    Applies <asset><unit meter=...> and all <node> transforms, and ignores
    non-geometry nodes (cameras, lights).
    """
    root = ET.parse(dae_path).getroot()

    unit_el = None
    asset = _child(root, "asset")
    if asset is not None:
        unit_el = _child(asset, "unit")
    unit = float(unit_el.get("meter", "1.0")) if unit_el is not None else 1.0

    up_el = _child(asset, "up_axis") if asset is not None else None
    up_axis = (up_el.text.strip() if up_el is not None and up_el.text else "Z_UP")
    if up_axis == "X_UP":
        raise ValueError(f"{dae_path}: X_UP is not supported")

    geoms = _geometry_positions(root)

    # Locate the visual scene referenced by <scene>.
    scene_el = _child(root, "scene")
    vs_ref = None
    if scene_el is not None:
        ivs = _child(scene_el, "instance_visual_scene")
        if ivs is not None:
            vs_ref = ivs.get("url", "").lstrip("#")
    lib = _child(root, "library_visual_scenes")
    scenes = _children(lib, "visual_scene") if lib is not None else []
    scene = next((s for s in scenes if s.get("id") == vs_ref), scenes[0] if scenes else None)
    if scene is None:
        raise ValueError(f"{dae_path}: no <visual_scene> found")

    mins = [math.inf] * 3
    maxs = [-math.inf] * 3
    found_geometry = False

    def walk(node, parent_matrix):
        nonlocal found_geometry
        matrix = _mat_mul(parent_matrix, _node_local_matrix(node))
        for inst in _children(node, "instance_geometry"):
            gid = inst.get("url", "").lstrip("#")
            for p in geoms.get(gid, []):
                x, y, z = _mat_apply(matrix, p)
                x, y, z = x * unit, y * unit, z * unit
                if up_axis == "Y_UP":
                    x, y, z = x, -z, y
                for i, v in enumerate((x, y, z)):
                    mins[i] = min(mins[i], v)
                    maxs[i] = max(maxs[i], v)
                found_geometry = True
        if _children(node, "instance_node"):
            raise ValueError(f"{dae_path}: <instance_node> is not supported")
        for child in _children(node, "node"):
            walk(child, matrix)

    for top in _children(scene, "node"):
        walk(top, IDENTITY)

    if not found_geometry:
        raise ValueError(f"{dae_path}: no geometry vertices found")

    dims = {
        "width_x": round(maxs[0] - mins[0], 4),
        "length_y": round(maxs[1] - mins[1], 4),
        "height_z": round(maxs[2] - mins[2], 4),
    }
    for k, v in dims.items():
        if v <= 0 or v > MAX_SANE_DIMENSION_M:
            raise ValueError(
                f"{dae_path}: {k}={v} m is not physically sensible — "
                f"unit conversion failure?"
            )
    return dims


# ---------------------------------------------------------------------------
# World parsing
# ---------------------------------------------------------------------------
def parse_pose(pose_str):
    """'X Y Z Roll Pitch Yaw' -> (x, y, z, yaw)."""
    p = pose_str.split()
    return float(p[0]), float(p[1]), float(p[2]), float(p[5])


def generate_semantic_db(world_name, output_path):
    project_root = find_project_root()
    if project_root is None:
        sys.exit("ERROR: could not find project root (directory containing Sim/).")

    sim_dir = os.path.join(project_root, "Sim")
    world_path = os.path.join(sim_dir, "worlds", world_name, f"{world_name}.world")
    if not os.path.exists(world_path):
        sys.exit(f"ERROR: world file not found: {world_path}")

    print(f"Parsing: {world_path}")
    world = ET.parse(world_path).getroot().find("world")

    # <state> poses override <model> poses at runtime.
    state_poses = {}
    state = world.find("state")
    if state is not None:
        for sm in state.findall("model"):
            pose_el = sm.find("pose")
            if pose_el is not None and pose_el.text:
                state_poses[sm.get("name")] = pose_el.text
        print(f"  <state> block: {len(state_poses)} pose overrides")

    database = {}
    skipped = []

    # Direct children only: './/model' would also pick up the <state> copies.
    for model in world.findall("model"):
        name = model.get("name")
        if name in SKIP_MODELS:
            skipped.append(name)
            continue

        pose_str = state_poses.get(name)
        if pose_str is None:
            pose_el = model.find("pose")
            pose_str = pose_el.text if pose_el is not None and pose_el.text else "0 0 0 0 0 0"
        x, y, z, yaw = parse_pose(pose_str)

        mesh_uri_el = model.find(".//visual/geometry/mesh/uri")
        if mesh_uri_el is None:
            skipped.append(name)  # primitive-only model
            continue

        mesh_uri = mesh_uri_el.text.strip()
        dae_path = resolve_mesh_path(mesh_uri, sim_dir)
        if not os.path.exists(dae_path):
            sys.exit(f"ERROR: mesh for '{name}' not found: {dae_path}")

        dims = get_mesh_dimensions(dae_path)

        database[name] = {
            "source_world": f"{world_name}.world",
            "model_type": extract_model_type_from_uri(mesh_uri),
            "frame_id": "world",
            "position": {"x": round(x, 6), "y": round(y, 6), "z": round(z, 6)},
            "rotation_yaw": round(yaw, 6),
            "physical_dimensions": dims,
            "dimension_source": "visual_dae",
        }
        print(f"  OK {name:24s} pos=({x:7.3f},{y:7.3f},{z:5.2f}) "
              f"dims=({dims['width_x']:.4f} x {dims['length_y']:.4f} x {dims['height_z']:.4f}) m")

    if skipped:
        print(f"  Skipped: {', '.join(skipped)}")

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(database, f, indent=2)
    print(f"\nWrote {len(database)} targets to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate a semantic 3D database from a Gazebo .world file."
    )
    parser.add_argument("--world", default="Our_Structured_Warehouse",
                        help="World folder name inside Sim/worlds/")
    parser.add_argument("--output", default=None,
                        help="Output JSON path (default: maps/<world>_semantic.json)")
    args = parser.parse_args()
    output = args.output or os.path.join("maps", f"{args.world}_semantic.json")
    generate_semantic_db(args.world, output)


if __name__ == "__main__":
    main()
