#!/usr/bin/env python3
"""Generate the CyberScape topological graph from warehouse spatial assets.

Reads:
  Sim/ros_ws/src/warehouse_navigation/maps/semantic.json
  Sim/ros_ws/src/warehouse_navigation/config/waypoints.yaml

Emits:
  backend/data/topo_graph.json

The generator:
  1. Parses rack bounding coordinates from semantic.json, grouped by zone with
     zone-aware orientation (vertical vs horizontal aisles based on rack yaw).
  2. Builds aisle spines parallel to rack orientation within each zone.
  3. Builds cross-aisle thoroughfares perpendicular to spines, connecting all
     spine endpoints so robots can travel between aisles and zones.
  4. Places Passing Junctions (capacity=2) at every spine x cross-aisle
     intersection where clearance >= 1.25 m (2 x PASS_CLEARANCE).
  5. Connects every waypoint (box, dock, home, truck, rack, zone_*) as a spur
     edge to its nearest navigable node.
  6. Assigns normalized undirected edge keys: lock:edge:{min(u,v)}_{max(u,v)}.
"""

from __future__ import annotations

import json
import math
import os
import re
from collections import defaultdict

try:
    import yaml
except ImportError as exc:  # pragma: no cover - dependency should be present
    raise SystemExit("PyYAML is required: pip install pyyaml") from exc

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PASS_CLEARANCE = 0.625  # m; Passing Junction requires 2 x PASS_CLEARANCE = 1.25 m
AISLE_NODE_SPACING = 2.0  # m between adjacent nodes along a spine / thoroughfare
SPUR_COST_FACTOR = 1.2  # spur edges cost slightly more than corridor edges

PROJECT_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(
            os.path.dirname(
                os.path.dirname(
                    os.path.dirname(os.path.abspath(__file__))
                )
            )
        )
    )
)
SEMANTIC_PATH = os.path.join(
    PROJECT_ROOT,
    "Sim",
    "ros_ws",
    "src",
    "warehouse_navigation",
    "maps",
    "semantic.json",
)
WAYPOINTS_PATH = os.path.join(
    PROJECT_ROOT,
    "Sim",
    "ros_ws",
    "src",
    "warehouse_navigation",
    "config",
    "waypoints.yaml",
)
OUTPUT_PATH = os.path.join(PROJECT_ROOT, "backend", "data", "topo_graph.json")

# Zone orientation: rack yaw determines whether aisles run vertical or horizontal.
# yaw 0.0 / 3.14  -> racks vertical (long dim along Y) -> aisles run vertical
# yaw +/-1.57     -> racks horizontal (long dim along X) -> aisles run horizontal
ZONE_ORIENTATION: dict[str, str] = {}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def zone_of_name(name: str) -> str:
    """Return the zone letter (A-D) from any waypoint name, case-insensitive."""
    m = re.match(r"(?:box|rack|dock|home|truck|zone)_([A-Da-d])", name)
    return m.group(1).upper() if m else "?"


def euclid(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def zone_bounds(racks: list[dict], zone: str) -> tuple[float, float, float, float]:
    """Return (x_min, x_max, y_min, y_max) of all racks in a zone."""
    zr = [r for r in racks if r["zone"] == zone]
    if not zr:
        return 0.0, 0.0, 0.0, 0.0
    xs = [r["x"] for r in zr]
    ys = [r["y"] for r in zr]
    ws = [r["w"] for r in zr]
    ls = [r["l"] for r in zr]
    x_min = min(x - w / 2 for x, w in zip(xs, ws))
    x_max = max(x + w / 2 for x, w in zip(xs, ws))
    y_min = min(y - l / 2 for y, l in zip(ys, ls))
    y_max = max(y + l / 2 for y, l in zip(ys, ls))
    return x_min, x_max, y_min, y_max


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def load_semantic(path: str) -> dict:
    with open(path, "r") as fh:
        return json.load(fh)


def load_waypoints(path: str) -> dict:
    with open(path, "r") as fh:
        return yaml.safe_load(fh)


def extract_racks(semantic: dict) -> list[dict]:
    """Return rack records with zone, x, y, width, length, yaw."""
    racks = []
    for name, entry in semantic.items():
        if not name.startswith("rack_"):
            continue
        zone = zone_of_name(name)
        pos = entry["position"]
        dims = entry["physical_dimensions"]
        racks.append(
            {
                "name": name,
                "zone": zone,
                "x": pos["x"],
                "y": pos["y"],
                "w": dims["width_x"],
                "l": dims["length_y"],
                "yaw": entry.get("rotation_yaw", 0.0),
            }
        )
    return racks


def extract_waypoints(waypoints: dict) -> list[dict]:
    """Return all waypoint records (box, dock, home, truck, rack, zone_*)."""
    places = waypoints.get("places", {})
    out = []
    for name, info in places.items():
        kind = info.get("kind", "?")
        out.append(
            {
                "name": name,
                "kind": kind,
                "x": info["x"],
                "y": info["y"],
                "zone": zone_of_name(name),
                "label": info.get("description", name),
            }
        )
    return out


def detect_zone_orientations(racks: list[dict]) -> None:
    """Populate ZONE_ORIENTATION from rack yaw values."""
    for rack in racks:
        zone = rack["zone"]
        yaw = abs(rack["yaw"]) % math.pi
        # yaw near 0 or pi  -> racks vertical  -> aisles run vertical
        # yaw near pi/2     -> racks horizontal -> aisles run horizontal
        if abs(yaw - math.pi / 2) < 0.3:
            ZONE_ORIENTATION[zone] = "horizontal"
        else:
            ZONE_ORIENTATION[zone] = "vertical"


# ---------------------------------------------------------------------------
# Graph construction: spines + cross-aisles + junctions
# ---------------------------------------------------------------------------


def build_zone_graph(
    racks: list[dict], zone: str, node_counter: list[int]
) -> tuple[dict[str, list[dict]], list[dict], list[dict]]:
    """Build spines, cross-aisles, and junctions for one zone.

    Returns (spines, cross_aisles, junctions) where each is a list of node dicts.
    node_counter is a single-element list used as a mutable counter.
    """
    orientation = ZONE_ORIENTATION.get(zone, "vertical")
    x_min, x_max, y_min, y_max = zone_bounds(racks, zone)

    # Pad bounds slightly so cross-aisles extend past the outermost racks
    pad = 1.0
    x_min -= pad
    x_max += pad
    y_min -= pad
    y_max += pad

    spines: list[dict] = []
    cross_aisles: list[dict] = []
    junctions: list[dict] = []

    zr = [r for r in racks if r["zone"] == zone]

    if orientation == "vertical":
        # Racks run vertically; aisles run vertically between rack columns.
        # Group rack columns by x coordinate.
        columns: dict[float, list[dict]] = defaultdict(list)
        for r in zr:
            columns[r["x"]].append(r)
        xs = sorted(columns.keys())

        # Aisle spines in gaps between consecutive rack columns
        for i in range(len(xs) - 1):
            x_left = xs[i]
            x_right = xs[i + 1]
            aisle_x = (x_left + x_right) / 2.0
            aisle_width = x_right - x_left
            if aisle_width < 2 * PASS_CLEARANCE:
                continue

            border_racks = columns[x_left] + columns[x_right]
            span_y_min = min(r["y"] - r["l"] / 2 for r in border_racks)
            span_y_max = max(r["y"] + r["l"] / 2 for r in border_racks)

            spine_id = f"spine_{zone}_v{int(round(aisle_x))}"
            nodes = []
            y = span_y_min + AISLE_NODE_SPACING / 2
            while y <= span_y_max:
                node_id = f"{spine_id}_n{node_counter[0]}"
                node_counter[0] += 1
                nodes.append(
                    {
                        "id": node_id,
                        "x": round(aisle_x, 3),
                        "y": round(y, 3),
                        "capacity": 1,
                        "type": "aisle",
                        "zone": zone,
                    }
                )
                y += AISLE_NODE_SPACING
            spines.append({"id": spine_id, "nodes": nodes, "x": aisle_x, "orientation": "vertical"})

        # Cross-aisles (horizontal) at zone y boundaries
        for cy in (y_min, y_max):
            ca_id = f"cross_{zone}_h{int(round(cy))}"
            ca_nodes = []
            x = x_min + AISLE_NODE_SPACING / 2
            while x <= x_max:
                node_id = f"{ca_id}_n{node_counter[0]}"
                node_counter[0] += 1
                ca_nodes.append(
                    {
                        "id": node_id,
                        "x": round(x, 3),
                        "y": round(cy, 3),
                        "capacity": 1,
                        "type": "aisle",
                        "zone": zone,
                    }
                )
                x += AISLE_NODE_SPACING
            cross_aisles.append({"id": ca_id, "nodes": ca_nodes, "y": cy, "orientation": "horizontal"})

    else:
        # Racks run horizontally; aisles run horizontally between rack rows.
        rows: dict[float, list[dict]] = defaultdict(list)
        for r in zr:
            rows[r["y"]].append(r)
        ys = sorted(rows.keys())

        for i in range(len(ys) - 1):
            y_bottom = ys[i]
            y_top = ys[i + 1]
            aisle_y = (y_bottom + y_top) / 2.0
            aisle_width = y_top - y_bottom
            if aisle_width < 2 * PASS_CLEARANCE:
                continue

            border_racks = rows[y_bottom] + rows[y_top]
            span_x_min = min(r["x"] - r["w"] / 2 for r in border_racks)
            span_x_max = max(r["x"] + r["w"] / 2 for r in border_racks)

            spine_id = f"spine_{zone}_h{int(round(aisle_y))}"
            nodes = []
            x = span_x_min + AISLE_NODE_SPACING / 2
            while x <= span_x_max:
                node_id = f"{spine_id}_n{node_counter[0]}"
                node_counter[0] += 1
                nodes.append(
                    {
                        "id": node_id,
                        "x": round(x, 3),
                        "y": round(aisle_y, 3),
                        "capacity": 1,
                        "type": "aisle",
                        "zone": zone,
                    }
                )
                x += AISLE_NODE_SPACING
            spines.append({"id": spine_id, "nodes": nodes, "y": aisle_y, "orientation": "horizontal"})

        # Cross-aisles (vertical) at zone x boundaries
        for cx in (x_min, x_max):
            ca_id = f"cross_{zone}_v{int(round(cx))}"
            ca_nodes = []
            y = y_min + AISLE_NODE_SPACING / 2
            while y <= y_max:
                node_id = f"{ca_id}_n{node_counter[0]}"
                node_counter[0] += 1
                ca_nodes.append(
                    {
                        "id": node_id,
                        "x": round(cx, 3),
                        "y": round(y, 3),
                        "capacity": 1,
                        "type": "aisle",
                        "zone": zone,
                    }
                )
                y += AISLE_NODE_SPACING
            cross_aisles.append({"id": ca_id, "nodes": ca_nodes, "x": cx, "orientation": "vertical"})

    # Place junctions at every spine x cross-aisle intersection
    for spine in spines:
        for ca in cross_aisles:
            if spine["orientation"] == "vertical" and ca["orientation"] == "horizontal":
                sx, cy = spine["x"], ca["y"]
                junction_id = f"junction_{zone}_{int(round(sx))}_{int(round(cy))}"
                junctions.append(
                    {
                        "id": junction_id,
                        "x": round(sx, 3),
                        "y": round(cy, 3),
                        "capacity": 2,
                        "type": "junction",
                        "zone": zone,
                    }
                )
            elif spine["orientation"] == "horizontal" and ca["orientation"] == "vertical":
                cx, sy = ca["x"], spine["y"]
                junction_id = f"junction_{zone}_{int(round(cx))}_{int(round(sy))}"
                junctions.append(
                    {
                        "id": junction_id,
                        "x": round(cx, 3),
                        "y": round(sy, 3),
                        "capacity": 2,
                        "type": "junction",
                        "zone": zone,
                    }
                )

    return spines, cross_aisles, junctions


def build_spine_edges(spines: list[dict]) -> list[dict]:
    """Create edges between consecutive nodes along each spine."""
    edges = []
    for spine in spines:
        nodes = spine["nodes"]
        for i in range(len(nodes) - 1):
            n1, n2 = nodes[i], nodes[i + 1]
            weight = round(euclid(n1["x"], n1["y"], n2["x"], n2["y"]), 3)
            key = "lock:edge:" + "_".join(sorted([n1["id"], n2["id"]]))
            edges.append(
                {
                    "from": n1["id"],
                    "to": n2["id"],
                    "weight": weight,
                    "key": key,
                    "single_lane": True,
                }
            )
    return edges


def build_cross_aisle_edges(cross_aisles: list[dict]) -> list[dict]:
    """Create edges between consecutive nodes along each cross-aisle."""
    edges = []
    for ca in cross_aisles:
        nodes = ca["nodes"]
        for i in range(len(nodes) - 1):
            n1, n2 = nodes[i], nodes[i + 1]
            weight = round(euclid(n1["x"], n1["y"], n2["x"], n2["y"]), 3)
            key = "lock:edge:" + "_".join(sorted([n1["id"], n2["id"]]))
            edges.append(
                {
                    "from": n1["id"],
                    "to": n2["id"],
                    "weight": weight,
                    "key": key,
                    "single_lane": False,
                }
            )
    return edges


def build_spine_to_cross_edges(
    spines: list[dict], cross_aisles: list[dict], junctions: list[dict]
) -> list[dict]:
    """Connect spines to cross-aisles at every intersection.

    At each spine x cross-aisle intersection:
      1. Connect the nearest spine node to the junction (if present).
      2. Connect the junction to the nearest cross-aisle node.
      3. If no junction exists, connect the nearest spine node directly to the
         nearest cross-aisle node.

    This merges the spine network and cross-aisle network into one connected
    component per zone.
    """
    edges = []
    junction_by_pos = {(j["x"], j["y"]): j["id"] for j in junctions}

    for spine in spines:
        for ca in cross_aisles:
            if spine["orientation"] == "vertical" and ca["orientation"] == "horizontal":
                sx, cy = spine["x"], ca["y"]
                best_spine = min(spine["nodes"], key=lambda n: abs(n["y"] - cy))
                best_ca = min(ca["nodes"], key=lambda n: abs(n["x"] - sx))
                junction_id = junction_by_pos.get((round(sx, 3), round(cy, 3)))

                if junction_id:
                    # Spine node -> junction
                    w1 = round(euclid(best_spine["x"], best_spine["y"], sx, cy), 3)
                    k1 = "lock:edge:" + "_".join(sorted([best_spine["id"], junction_id]))
                    edges.append(
                        {"from": best_spine["id"], "to": junction_id, "weight": w1, "key": k1, "single_lane": True}
                    )
                    # Junction -> cross-aisle node
                    w2 = round(euclid(sx, cy, best_ca["x"], best_ca["y"]), 3)
                    k2 = "lock:edge:" + "_".join(sorted([junction_id, best_ca["id"]]))
                    edges.append(
                        {"from": junction_id, "to": best_ca["id"], "weight": w2, "key": k2, "single_lane": True}
                    )
                else:
                    # Direct spine -> cross-aisle connection
                    w = round(euclid(best_spine["x"], best_spine["y"], best_ca["x"], best_ca["y"]), 3)
                    k = "lock:edge:" + "_".join(sorted([best_spine["id"], best_ca["id"]]))
                    edges.append(
                        {"from": best_spine["id"], "to": best_ca["id"], "weight": w, "key": k, "single_lane": True}
                    )

            elif spine["orientation"] == "horizontal" and ca["orientation"] == "vertical":
                cx, sy = ca["x"], spine["y"]
                best_spine = min(spine["nodes"], key=lambda n: abs(n["x"] - cx))
                best_ca = min(ca["nodes"], key=lambda n: abs(n["y"] - sy))
                junction_id = junction_by_pos.get((round(cx, 3), round(sy, 3)))

                if junction_id:
                    w1 = round(euclid(best_spine["x"], best_spine["y"], cx, sy), 3)
                    k1 = "lock:edge:" + "_".join(sorted([best_spine["id"], junction_id]))
                    edges.append(
                        {"from": best_spine["id"], "to": junction_id, "weight": w1, "key": k1, "single_lane": True}
                    )
                    w2 = round(euclid(cx, sy, best_ca["x"], best_ca["y"]), 3)
                    k2 = "lock:edge:" + "_".join(sorted([junction_id, best_ca["id"]]))
                    edges.append(
                        {"from": junction_id, "to": best_ca["id"], "weight": w2, "key": k2, "single_lane": True}
                    )
                else:
                    w = round(euclid(best_spine["x"], best_spine["y"], best_ca["x"], best_ca["y"]), 3)
                    k = "lock:edge:" + "_".join(sorted([best_spine["id"], best_ca["id"]]))
                    edges.append(
                        {"from": best_spine["id"], "to": best_ca["id"], "weight": w, "key": k, "single_lane": True}
                    )
    return edges


def build_waypoint_spurs(
    waypoints: list[dict],
    spines: list[dict],
    cross_aisles: list[dict],
    junctions: list[dict],
) -> list[dict]:
    """Connect each waypoint as a spur edge to its nearest navigable node."""
    edges = []

    # Collect all navigable corridor nodes (spine + cross-aisle + junction)
    all_nodes = []
    for spine in spines:
        all_nodes.extend(spine["nodes"])
    for ca in cross_aisles:
        all_nodes.extend(ca["nodes"])
    all_nodes.extend(junctions)

    for wp in waypoints:
        best = None
        best_dist = float("inf")
        for n in all_nodes:
            dist = euclid(wp["x"], wp["y"], n["x"], n["y"])
            if dist < best_dist:
                best_dist = dist
                best = n
        if best:
            weight = round(euclid(wp["x"], wp["y"], best["x"], best["y"]) * SPUR_COST_FACTOR, 3)
            key = "lock:edge:" + "_".join(sorted([wp["name"], best["id"]]))
            edges.append(
                {
                    "from": wp["name"],
                    "to": best["id"],
                    "weight": weight,
                    "key": key,
                    "single_lane": True,
                }
            )
    return edges


def build_inter_zone_edges(
    all_spines: dict[str, list[dict]],
    all_cross_aisles: dict[str, list[dict]],
    zones: list[str],
    node_counter: list[int],
) -> tuple[list[dict], list[dict]]:
    """Create edges connecting adjacent zones together.

    Connects zone boundary cross-aisle nodes between neighboring zones so
    robots can travel from one zone to another.
    """
    edges = []
    new_nodes = []

    # Zone adjacency: which zones share a boundary
    # A (bottom-left) <-> B (bottom-right): horizontal connection
    # A (bottom-left) <-> D (top-left): vertical connection
    # B (bottom-right) <-> C (top-right): vertical connection
    # D (top-left) <-> C (top-right): horizontal connection
    adjacency = [
        ("A", "B", "horizontal"),
        ("A", "D", "vertical"),
        ("B", "C", "vertical"),
        ("D", "C", "horizontal"),
    ]

    for z1, z2, direction in adjacency:
        if z1 not in all_cross_aisles or z2 not in all_cross_aisles:
            continue
        c1 = all_cross_aisles[z1]
        c2 = all_cross_aisles[z2]

        if direction == "horizontal":
            # z1 is left of z2; connect rightmost cross-ailse of z1 to leftmost of z2
            # Find cross-aisles near the zone boundary
            ca1 = max(c1, key=lambda c: c.get("x", 0)) if c1 else None
            ca2 = min(c2, key=lambda c: c.get("x", 0)) if c2 else None
        else:
            # z1 is below z2; connect topmost cross-aisle of z1 to bottommost of z2
            ca1 = max(c1, key=lambda c: c.get("y", 0)) if c1 else None
            ca2 = min(c2, key=lambda c: c.get("y", 0)) if c2 else None

        if ca1 is None or ca2 is None:
            continue

        # Pick the node on each cross-aisle closest to the zone boundary
        n1 = ca1["nodes"][0] if ca1["nodes"] else None
        n2 = ca2["nodes"][0] if ca2["nodes"] else None
        if n1 is None or n2 is None:
            continue

        weight = round(euclid(n1["x"], n1["y"], n2["x"], n2["y"]), 3)
        key = "lock:edge:" + "_".join(sorted([n1["id"], n2["id"]]))
        edges.append(
            {
                "from": n1["id"],
                "to": n2["id"],
                "weight": weight,
                "key": key,
                "single_lane": False,
            }
        )

    return edges, new_nodes


# ---------------------------------------------------------------------------
# Graph assembly & output
# ---------------------------------------------------------------------------


def build_graph(racks: list[dict], waypoints: list[dict]) -> dict:
    """Build the full topological graph for all zones."""
    detect_zone_orientations(racks)
    zones = sorted({r["zone"] for r in racks})
    node_counter = [0]

    all_spines: dict[str, list[dict]] = {}
    all_cross_aisles: dict[str, list[dict]] = {}
    all_junctions: list[dict] = []
    all_edges: list[dict] = []
    all_nodes: dict[str, dict] = {}

    # Build per-zone graphs
    for zone in zones:
        zr = [r for r in racks if r["zone"] == zone]
        spines, cross_aisles, junctions = build_zone_graph(zr, zone, node_counter)
        all_spines[zone] = spines
        all_cross_aisles[zone] = cross_aisles
        all_junctions.extend(junctions)

        # Register spine and cross-aisle nodes
        for spine in spines:
            for n in spine["nodes"]:
                all_nodes[n["id"]] = {
                    "x": n["x"],
                    "y": n["y"],
                    "capacity": n["capacity"],
                    "type": n["type"],
                    "zone": n["zone"],
                }
        for ca in cross_aisles:
            for n in ca["nodes"]:
                all_nodes[n["id"]] = {
                    "x": n["x"],
                    "y": n["y"],
                    "capacity": n["capacity"],
                    "type": n["type"],
                    "zone": n["zone"],
                }
        for j in junctions:
            all_nodes[j["id"]] = {
                "x": j["x"],
                "y": j["y"],
                "capacity": j["capacity"],
                "type": j["type"],
                "zone": j["zone"],
            }

        # Intra-zone edges
        all_edges.extend(build_spine_edges(spines))
        all_edges.extend(build_cross_aisle_edges(cross_aisles))
        all_edges.extend(build_spine_to_cross_edges(spines, cross_aisles, junctions))

    # Inter-zone edges
    inter_edges, _ = build_inter_zone_edges(
        all_spines, all_cross_aisles, zones, node_counter
    )
    all_edges.extend(inter_edges)

    # Waypoint spur edges (all types: box, dock, home, truck, rack, zone_*)
    waypoint_edges = build_waypoint_spurs(
        waypoints,
        [s for spines in all_spines.values() for s in spines],
        [ca for cas in all_cross_aisles.values() for ca in cas],
        all_junctions,
    )
    all_edges.extend(waypoint_edges)

    # Register waypoint nodes
    for wp in waypoints:
        all_nodes[wp["name"]] = {
            "x": wp["x"],
            "y": wp["y"],
            "capacity": 1,
            "type": wp["kind"],
            "zone": wp["zone"],
            "label": wp["label"],
        }

    return {"nodes": all_nodes, "edges": all_edges}


def write_graph(graph: dict, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(graph, fh, indent=2, sort_keys=True)
    junction_count = sum(1 for n in graph["nodes"].values() if n["type"] == "junction")
    aisle_count = sum(1 for n in graph["nodes"].values() if n["type"] == "aisle")
    waypoint_count = sum(1 for n in graph["nodes"].values() if n["type"] not in ("aisle", "junction"))
    print(
        f"Wrote {len(graph['nodes'])} nodes "
        f"({aisle_count} aisle, {junction_count} junction, {waypoint_count} waypoint) "
        f"and {len(graph['edges'])} edges to {path}"
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    semantic = load_semantic(SEMANTIC_PATH)
    waypoints_data = load_waypoints(WAYPOINTS_PATH)

    racks = extract_racks(semantic)
    waypoints = extract_waypoints(waypoints_data)

    print(f"Parsed {len(racks)} racks and {len(waypoints)} waypoints")

    graph = build_graph(racks, waypoints)
    print(f"Zone orientations: {dict(ZONE_ORIENTATION)}")
    write_graph(graph, OUTPUT_PATH)


if __name__ == "__main__":
    main()
