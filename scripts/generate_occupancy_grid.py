#!/usr/bin/env python3
"""
generate_occupancy_grid.py — Direct 2D Occupancy Grid Generator

Renders a deterministic 2D occupancy grid map (.pgm + .yaml) directly from
Gazebo collision geometry (.sdf and .world) without running SLAM.

Outputs:
  - maps/our_structured_warehouse_map.pgm
  - maps/our_structured_warehouse_map.yaml
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw
import yaml


def generate_map(world_name="Our_Structured_Warehouse", output_prefix="maps/our_structured_warehouse_map"):
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    maps_dir = os.path.dirname(os.path.abspath(output_prefix))
    os.makedirs(maps_dir, exist_ok=True)

    pgm_path = f"{output_prefix}.pgm"
    yaml_path = f"{output_prefix}.yaml"

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

    def rotate_point(x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return x * c - y * s, x * s + y * c

    # 1. Fill 4 workcell module floors with free space (254)
    workcells = [
        ("workcell_SW", -0.040107, 0.052292, 0.0),
        ("workcell_SE", 20.0625, 0.052292, math.pi / 2),
        ("workcell_NE", 20.0625, 20.1549, math.pi),
        ("workcell_NW", -0.040107, 20.1549, -math.pi / 2),
    ]

    half_floor = 20.1026 / 2.0
    for name, cx, cy, yaw in workcells:
        corners = [
            (-half_floor, -half_floor),
            (half_floor, -half_floor),
            (half_floor, half_floor),
            (-half_floor, half_floor),
        ]
        poly_px = [world_to_pixel(cx + rx, cy + ry) for lx, ly in corners for rx, ry in [rotate_point(lx, ly, yaw)]]
        draw.polygon(poly_px, fill=254)

    # 2. Draw our_workcell internal collision obstacles (walls, pillars, conveyor frames)
    workcell_sdf = os.path.join(project_root, "Sim", "models", "our_workcell", "model.sdf")
    if os.path.exists(workcell_sdf):
        tree = ET.parse(workcell_sdf)
        root = tree.getroot()
        wc_collisions = []
        for c in root.findall(".//collision"):
            cname = c.get("name")
            if cname == "floor":
                continue
            pose = c.find("pose")
            box = c.find(".//box/size")
            if pose is not None and box is not None:
                p = [float(x) for x in pose.text.split()]
                s = [float(x) for x in box.text.split()]
                wc_collisions.append((cname, p, s))

        for wc_name, wcx, wcy, wc_yaw in workcells:
            for cname, p, s in wc_collisions:
                lx, ly = p[0], p[1]
                sx, sy = s[0], s[1]
                half_sx, half_sy = sx / 2.0, sy / 2.0
                corners = [
                    (-half_sx, -half_sy),
                    (half_sx, -half_sy),
                    (half_sx, half_sy),
                    (-half_sx, half_sy),
                ]
                poly_px = []
                for x_c, y_c in corners:
                    gx, gy = rotate_point(lx + x_c, ly + y_c, wc_yaw)
                    poly_px.append(world_to_pixel(wcx + gx, wcy + gy))
                draw.polygon(poly_px, fill=0)

    # 3. Draw warehouse storage racks and boxes
    world_xml = os.path.join(project_root, "Sim", "worlds", world_name, f"{world_name}.world")
    if os.path.exists(world_xml):
        w_tree = ET.parse(world_xml)
        w_root = w_tree.getroot()

        rack_dims = {
            "warehouse_rack_5lvl_6bay": (0.83, 5.8),
            "warehouse_rack_5lvl_7bay": (0.83, 6.8),
            "warehouse_rack_4lvl_6bay": (0.83, 5.8),
            "warehouse_rack_4lvl_7bay": (0.83, 6.8),
        }

        for inc in w_root.findall(".//include"):
            uri = inc.find("uri")
            pose = inc.find("pose")
            if uri is not None and pose is not None:
                model_name = uri.text.replace("model://", "").strip()
                if model_name in rack_dims:
                    sx, sy = rack_dims[model_name]
                    p = [float(x) for x in pose.text.split()]
                    rx, ry, ryaw = p[0], p[1], p[5]
                    half_sx, half_sy = sx / 2.0, sy / 2.0
                    corners = [
                        (-half_sx, -half_sy),
                        (half_sx, -half_sy),
                        (half_sx, half_sy),
                        (-half_sx, half_sy),
                    ]
                    poly_px = [world_to_pixel(rx + gx, ry + gy) for lx, ly in corners for gx, gy in [rotate_point(lx, ly, ryaw)]]
                    draw.polygon(poly_px, fill=0)
                elif model_name == "delivery_truck":
                    p = [float(x) for x in pose.text.split()]
                    rx, ry, ryaw = p[0], p[1], p[5]
                    half_sx, half_sy = 7.5 / 2.0, 2.42 / 2.0
                    corners = [
                        (-half_sx, -half_sy),
                        (half_sx, -half_sy),
                        (half_sx, half_sy),
                        (-half_sx, half_sy),
                    ]
                    poly_px = [world_to_pixel(rx + gx, ry + gy) for lx, ly in corners for gx, gy in [rotate_point(lx, ly, ryaw)]]
                    draw.polygon(poly_px, fill=0)
                elif model_name == "dock_conveyor":
                    p = [float(x) for x in pose.text.split()]
                    rx, ry, ryaw = p[0], p[1], p[5]
                    half_sx, half_sy = 6.45 / 2.0, 1.06 / 2.0
                    corners = [
                        (-half_sx, -half_sy),
                        (half_sx, -half_sy),
                        (half_sx, half_sy),
                        (-half_sx, half_sy),
                    ]
                    poly_px = [world_to_pixel(rx + gx, ry + gy) for lx, ly in corners for gx, gy in [rotate_point(lx, ly, ryaw)]]
                    draw.polygon(poly_px, fill=0)

        for model in w_root.findall(".//model"):
            mname = model.get("name", "")
            if mname.startswith("box_") or mname.startswith("wall_filler"):
                pose = model.find("pose")
                box_size = model.find(".//geometry/box/size")
                if pose is not None and box_size is not None:
                    p = [float(x) for x in pose.text.split()]
                    s = [float(x) for x in box_size.text.split()]
                    bx, by, byaw = p[0], p[1], p[5]
                    half_sx, half_sy = s[0] / 2.0, s[1] / 2.0
                    corners = [
                        (-half_sx, -half_sy),
                        (half_sx, -half_sy),
                        (half_sx, half_sy),
                        (-half_sx, half_sy),
                    ]
                    poly_px = [world_to_pixel(bx + gx, by + gy) for lx, ly in corners for gx, gy in [rotate_point(lx, ly, byaw)]]
                    draw.polygon(poly_px, fill=0)

    # Save PGM image
    img.save(pgm_path)
    print(f"Generated PGM map: {pgm_path} ({os.path.getsize(pgm_path)} bytes)")

    # Save YAML metadata
    image_rel = os.path.basename(pgm_path)
    meta = {
        "image": image_rel,
        "resolution": resolution,
        "origin": [origin_x, origin_y, origin_yaw],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }

    with open(yaml_path, "w") as f:
        yaml.dump(meta, f, default_flow_style=False)
    print(f"Generated YAML metadata: {yaml_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate 2D Occupancy Grid map (.pgm + .yaml) directly from world")
    parser.add_argument("--world", default="Our_Structured_Warehouse", help="World folder name in Sim/worlds/")
    parser.add_argument("--output", default="maps/our_structured_warehouse_map", help="Output path prefix")
    args = parser.parse_args()
    generate_map(args.world, args.output)


if __name__ == "__main__":
    main()

