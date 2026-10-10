#!/usr/bin/env python3
"""Generate the Nav2 map of the warehouse from the Gazebo world file.

Every obstacle in the world is a collision box, so the map is drawn from their exact
positions instead of being built with SLAM.

Run on the host, from anywhere:
  python3 Sim/ros_ws/src/warehouse_navigation/scripts/generate_map.py
"""
import math
import os
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..', '..'))
WORLD_FILE = os.path.join(SIM_DIR, 'worlds', 'Our_Structured_Warehouse', 'Our_Structured_Warehouse.world')
MODELS_DIR = os.path.join(SIM_DIR, 'models')
MAP_FILE = os.path.join(SCRIPT_DIR, '..', 'maps', 'warehouse')  # writes warehouse.pgm and warehouse.yaml

RESOLUTION = 0.05  # m per pixel, same as slam_toolbox
MARGIN = 1.0       # m of free space around the outermost obstacle
FREE, OCCUPIED = 254, 0  # pixel values Nav2's map_server reads as free / occupied

# Only parts between these heights (m) can touch the Husky; the floor and anything higher are left out.
ROBOT_Z_MIN = 0.05
ROBOT_Z_MAX = 0.6


def parse_pose(text):
    """'x y z roll pitch yaw' -> (x, y, z, yaw). Roll and pitch are always 0 in this world."""
    x, y, z, _roll, _pitch, yaw = (float(v) for v in (text or '0 0 0 0 0 0').split())
    return x, y, z, yaw


def read_world(path):
    """Every model placed in the world: <include> (reused from Sim/models) and inline <model>."""
    world = ET.parse(path).getroot().find('world')
    models = []
    for element in world:
        if element.tag == 'include':
            name = element.findtext('name')
            kind = element.findtext('uri').replace('model://', '')
        elif element.tag == 'model':
            name = element.get('name')
            kind = 'inline'
        else:
            continue  # light, physics, gui, ...
        x, y, z, yaw = parse_pose(element.findtext('pose'))
        models.append({'name': name, 'type': kind, 'x': x, 'y': y, 'z': z, 'yaw': yaw, 'element': element})
    return models


def collision_elements(model):
    """The <collision> elements of a model: inline in the world, or from Sim/models/<type>/model.sdf."""
    if model['type'] == 'inline':
        source = model['element']
    else:
        source = ET.parse(os.path.join(MODELS_DIR, model['type'], 'model.sdf')).getroot().find('model')
    return source.findall('./link/collision')


def collision_boxes(model):
    """Each collision box of a model, placed in world coordinates.

    A box is stored by its centre (x, y), its size along its own axes (sx, sy), its yaw,
    and the heights of its bottom and top (z_min, z_max).
    """
    boxes = []
    cos_m, sin_m = math.cos(model['yaw']), math.sin(model['yaw'])
    for collision in collision_elements(model):
        size = collision.findtext('./geometry/box/size')
        if size is None:
            continue  # not a box: only the ground plane in this world
        sx, sy, sz = (float(v) for v in size.split())
        lx, ly, lz, lyaw = parse_pose(collision.findtext('pose'))  # offset inside the model
        # rotate the offset by the model's yaw, then move it to the model's position
        x = model['x'] + cos_m * lx - sin_m * ly
        y = model['y'] + sin_m * lx + cos_m * ly
        z = model['z'] + lz
        boxes.append({'model': model['name'], 'part': collision.get('name'),
                      'x': x, 'y': y, 'sx': sx, 'sy': sy, 'yaw': model['yaw'] + lyaw,
                      'z_min': z - sz / 2, 'z_max': z + sz / 2,
                      'local': (lx, ly, sx, sy)})
    return boxes


def solid_rack(model, parts):
    """One rectangle covering all shelves and posts of a rack, so Nav2 never plans between the posts."""
    local = [p['local'] for p in parts]  # (x, y, size_x, size_y) inside the rack model
    x_min = min(x - sx / 2 for x, _, sx, _ in local)
    x_max = max(x + sx / 2 for x, _, sx, _ in local)
    y_min = min(y - sy / 2 for _, y, _, sy in local)
    y_max = max(y + sy / 2 for _, y, _, sy in local)
    cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2  # centre inside the rack model
    cos_m, sin_m = math.cos(model['yaw']), math.sin(model['yaw'])
    return {'name': model['name'],
            'x': model['x'] + cos_m * cx - sin_m * cy, 'y': model['y'] + sin_m * cx + cos_m * cy,
            'sx': x_max - x_min, 'sy': y_max - y_min, 'yaw': model['yaw']}


def map_obstacles(models):
    """The rectangles to draw on the map, and how many collision boxes each rule left out."""
    obstacles, skipped = [], {'labelled boxes': 0, 'outside robot height': 0}
    for model in models:
        parts = collision_boxes(model)
        if model['name'].startswith('box_'):
            skipped['labelled boxes'] += len(parts)  # they move; boxes on shelves are inside the solid rack
        elif model['name'].startswith('rack_'):
            obstacles.append(solid_rack(model, parts))
        else:
            for p in parts:
                if p['z_max'] < ROBOT_Z_MIN or p['z_min'] > ROBOT_Z_MAX:
                    skipped['outside robot height'] += 1
                else:
                    obstacles.append({'name': f"{p['model']}/{p['part']}", **{k: p[k] for k in ('x', 'y', 'sx', 'sy', 'yaw')}})
    return obstacles, skipped


def corners(o):
    """The four corners of a rectangle obstacle in world coordinates."""
    c, s = math.cos(o['yaw']), math.sin(o['yaw'])
    return [(o['x'] + c * dx - s * dy, o['y'] + s * dx + c * dy)
            for dx, dy in ((-o['sx'] / 2, -o['sy'] / 2), (o['sx'] / 2, -o['sy'] / 2),
                           (o['sx'] / 2, o['sy'] / 2), (-o['sx'] / 2, o['sy'] / 2))]


def draw_map(obstacles, path):
    """Draw the obstacles into <path>.pgm and write <path>.yaml in Nav2 map_server format."""
    points = [p for o in obstacles for p in corners(o)]
    x0 = min(x for x, _ in points) - MARGIN
    y0 = min(y for _, y in points) - MARGIN
    width = math.ceil((max(x for x, _ in points) + MARGIN - x0) / RESOLUTION)
    height = math.ceil((max(y for _, y in points) + MARGIN - y0) / RESOLUTION)

    def pixel(x, y):  # image rows go down, world y goes up
        return (x - x0) / RESOLUTION, height - (y - y0) / RESOLUTION

    image = Image.new('L', (width, height), FREE)
    draw = ImageDraw.Draw(image)
    for o in obstacles:
        draw.polygon([pixel(x, y) for x, y in corners(o)], fill=OCCUPIED)
    image.save(path + '.pgm')

    with open(path + '.yaml', 'w') as f:
        f.write(f'# Generated by scripts/generate_map.py from {os.path.basename(WORLD_FILE)}; do not edit by hand.\n'
                f'image: {os.path.basename(path)}.pgm\n'
                f'mode: trinary\n'
                f'resolution: {RESOLUTION}\n'
                f'origin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n')
    return width, height, x0, y0


def main():
    models = read_world(WORLD_FILE)
    obstacles, skipped = map_obstacles(models)
    racks = sum(1 for o in obstacles if o['name'].startswith('rack_'))
    print(f'obstacles: {racks} solid racks + {len(obstacles) - racks} others; '
          f'skipped {skipped["labelled boxes"]} labelled boxes, {skipped["outside robot height"]} parts outside robot height')

    path = os.path.abspath(MAP_FILE)
    width, height, x0, y0 = draw_map(obstacles, path)
    print(f'map: {width} x {height} px = {width * RESOLUTION:.1f} x {height * RESOLUTION:.1f} m, '
          f'origin ({x0:.2f}, {y0:.2f})')
    print(f'wrote {path}.pgm and {path}.yaml')


if __name__ == '__main__':
    main()
