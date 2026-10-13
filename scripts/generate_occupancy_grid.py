#!/usr/bin/env python3
"""
generate_occupancy_grid.py — Direct 2D Occupancy Grid Generator

Renders a deterministic 2D occupancy grid map (.pgm + .yaml) directly from the
collision geometry Gazebo simulates, without running SLAM:

  * every <collision> box of every model in the .world, plus the models it
    <include>s (resolved from Sim/models/<name>/model.sdf)
  * model, link and collision poses are composed in 3D (roll/pitch/yaw)
  * a floor box (top at ground level) marks free space; any box that reaches
    into the robot's height band (above the floor, below ROBOT_HEIGHT) is an
    obstacle, drawn with its full footprint

Two maps are written, because the robot's planner and its lidar see different worlds:

  * <prefix>        navigation map: racks are solid, because their bottom shelf sits at wheel
                    height and the robot cannot drive under them. Used by the Nav2 costmaps.
  * <prefix>_scan   scan map: only what the lidar plane (LIDAR_Z) hits, which for racks is the
                    thin posts, since the lidar looks between the shelves. Used by AMCL, which
                    cannot localise against walls the lidar sees through.

Outputs (each as .pgm + .yaml, plus a .png preview of the .pgm):
  - maps/our_structured_warehouse_map
  - maps/our_structured_warehouse_map_scan
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw
import yaml

# Height band (m) in which a collision box blocks the Husky: from just above the
# floor up to the top of the robot (lidar on top at ~0.42 m) plus margin.
FLOOR_TOP = 0.01
ROBOT_HEIGHT = 0.6
# Height of the lidar plane above the floor: laser_link in urdf/husky_lidar.urdf.xacro (0.417 m
# above base_footprint). A box shows up in the scan map if the plane passes through it.
LIDAR_Z = 0.417


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def pose_of(elem):
    """(translation, rotation matrix) of an element's <pose>, identity if absent."""
    p = elem.find("pose")
    v = [float(x) for x in p.text.split()] if p is not None and p.text and p.text.strip() else [0.0] * 6
    return v[:3], rpy_matrix(*v[3:6])


def compose(parent, child):
    (pt, pr), (ct, cr) = parent, child
    t = [pt[i] + sum(pr[i][k] * ct[k] for k in range(3)) for i in range(3)]
    return t, matmul(pr, cr)


def box_corners(pose, size):
    t, r = pose
    corners = []
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            for sz in (-0.5, 0.5):
                local = (sx * size[0], sy * size[1], sz * size[2])
                corners.append([t[i] + sum(r[i][k] * local[k] for k in range(3)) for i in range(3)])
    return corners


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def model_boxes(model, model_pose):
    """World-frame (corners, model name, collision name) of every box collision in a model."""
    boxes = []
    for link in model.findall("link"):
        link_pose = compose(model_pose, pose_of(link))
        for col in link.findall("collision"):
            size = col.find("geometry/box/size")
            if size is None:
                continue
            col_pose = compose(link_pose, pose_of(col))
            boxes.append((box_corners(col_pose, [float(x) for x in size.text.split()]),
                          model.get("name"), col.get("name")))
    return boxes


def world_boxes(world_xml, models_dir):
    world = ET.parse(world_xml).getroot().find("world")
    boxes = []
    for model in world.findall("model"):
        boxes += model_boxes(model, pose_of(model))
    for inc in world.findall("include"):
        name = inc.find("uri").text.replace("model://", "").strip()
        sdf = os.path.join(models_dir, name, "model.sdf")
        if not os.path.exists(sdf):
            print(f"[WARN] {name}: no {sdf}, skipped")
            continue
        model = ET.parse(sdf).getroot().find("model")
        boxes += model_boxes(model, compose(pose_of(inc), pose_of(model)))
    return boxes


def render_map(boxes, hits, output_prefix, label):
    """Draw floors free and every box for which hits(z_min, z_max) is true as occupied; save .pgm/.yaml/.png."""
    # Map specifications (50m x 50m @ 0.05m/pixel)
    origin_x, origin_y, origin_yaw = -15.0, -15.0, 0.0
    resolution = 0.05
    width, height = 1000, 1000

    # Initialize with unknown space (205)
    img = Image.new("L", (width, height), color=205)
    draw = ImageDraw.Draw(img)

    def world_to_pixel(x, y):
        px = int((x - origin_x) / resolution)
        py = height - 1 - int((y - origin_y) / resolution)
        return px, py

    floors, obstacles = [], []
    for corners, model, col in boxes:
        z_min = min(c[2] for c in corners)
        z_max = max(c[2] for c in corners)
        footprint = convex_hull([(c[0], c[1]) for c in corners])
        if z_max <= FLOOR_TOP:
            floors.append(footprint)
        elif hits(z_min, z_max):
            obstacles.append(footprint)

    # 1. Floors are free space (254), 2. obstacles are occupied (0)
    for footprint in floors:
        draw.polygon([world_to_pixel(x, y) for x, y in footprint], fill=254)
    for footprint in obstacles:
        draw.polygon([world_to_pixel(x, y) for x, y in footprint], fill=0)
    print(f"[{label}] {len(floors)} floors, {len(obstacles)} obstacles")

    pgm_path = f"{output_prefix}.pgm"
    img.save(pgm_path)
    img.save(f"{output_prefix}.png")
    print(f"Generated PGM map: {pgm_path} ({os.path.getsize(pgm_path)} bytes)")

    meta = {
        "image": os.path.basename(pgm_path),
        "resolution": resolution,
        "origin": [origin_x, origin_y, origin_yaw],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(f"{output_prefix}.yaml", "w") as f:
        yaml.dump(meta, f, default_flow_style=False)
    print(f"Generated YAML metadata: {output_prefix}.yaml")


def generate_map(world_name="Our_Structured_Warehouse", output_prefix="maps/our_structured_warehouse_map"):
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    os.makedirs(os.path.dirname(os.path.abspath(output_prefix)), exist_ok=True)

    world_xml = os.path.join(project_root, "Sim", "worlds", world_name, f"{world_name}.world")
    boxes = world_boxes(world_xml, os.path.join(project_root, "Sim", "models"))
    print(f"Collision boxes: {len(boxes)}")

    # Navigation map: anything in the robot's height band blocks it
    render_map(boxes, lambda z_min, z_max: z_min < ROBOT_HEIGHT, output_prefix, "navigation")
    # Scan map: only what the lidar plane passes through
    render_map(boxes, lambda z_min, z_max: z_min <= LIDAR_Z <= z_max, f"{output_prefix}_scan", "scan")


def main():
    parser = argparse.ArgumentParser(description="Generate 2D Occupancy Grid map (.pgm + .yaml) directly from world")
    parser.add_argument("--world", default="Our_Structured_Warehouse", help="World folder name in Sim/worlds/")
    parser.add_argument("--output", default="maps/our_structured_warehouse_map", help="Output path prefix")
    args = parser.parse_args()
    generate_map(args.world, args.output)


if __name__ == "__main__":
    main()
