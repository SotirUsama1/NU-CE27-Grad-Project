#!/usr/bin/env python3
"""Build the warehouse robots and the Custom_Robot_Warehouse world (Our_Structured_Warehouse plus robots).

picker_robot     case-handling robot: lifting tray with two-stage telescopic side arms that pull a
                 box off a shelf, four storage slots, so it carries up to five boxes.
transport_robot  mobile robot with a powered roller deck at conveyor height, carries two small or
                 heavy boxes or one long box, and rolls them onto the conveyor at S1.

Both are driven by the Custom_Robot plugin (Sim/plugins/build_plugins.sh builds it) through ROS 2;
the topics and commands are listed in the generated warehouse_robots.yaml next to the world.

Run from the repository root:  python3 Sim/tools/build_warehouse_robots.py
Re-running replaces the robot models and the Custom_Robot_Warehouse world; Our_Structured_Warehouse
is only read, so re-run this after changing it.
"""
import math
import re
import shutil
import sys

from PIL import Image, ImageDraw

sys.dont_write_bytecode = True
from build_our_warehouse_storage import (BELT_START_X, BELT_TOP, DOCK_POINTS, DOCK_ZONE, DOOR_Y, LEVEL_SETS,
                                         SIM, WORLD, ZONES, Mesh, fmt, font, new_model_dir, to_world)

ROBOT_WORLD = SIM / 'worlds/Custom_Robot_Warehouse/Custom_Robot_Warehouse.world'

# ------------------------------------------------------------ dimensions
# Picker, base frame at the centre of the footprint on the floor, x forward.
P_LENGTH, P_WIDTH = 1.78, 0.9
TRAY_X = 0.37                  # tray centre
TRAY_ZERO = 0.1                # tray top at the lowest lift position
LIFT_RANGE = 1.9               # tray top reaches 2.0 m, above the top shelf level (1.9 m)
STAGE_TRAVEL = 0.75            # each telescopic stage, arm centre reaches 1.5 m to either side
SLOT_X = -0.53
SLOT_TOPS = (0.3, 0.85, 1.4, 1.95)
MAST_X = -0.16
MAST_TOP = 2.45
ARM_Y = 0.42                   # half length of the arm beams along y
ARM_X = (0.445, 0.465, 0.485)  # arm, stage and rail beams, distance from the tray centre along x
AISLE_HALF = 0.78              # robot centre to shelf front when driving in the middle of an aisle
P_LASER = (0.9, 0, 0.05)

# Transport robot
T_LENGTH, T_WIDTH = 1.3, 0.8
DECK_TOP = BELT_TOP + 0.01     # boxes roll off onto the conveyor belt
DECK = (1.24, 0.78)
T_LASER = (0.67, 0, 0.18)

# name, model, zone whose frame the pose is given in, local x, local y, local yaw
ROBOTS = [
    ('picker_1', 'picker', 'A', 2.5, -1.2, 0.0),         # cross aisle of zone A
    ('picker_2', 'picker', 'B', 2.5, -1.2, 0.0),         # cross aisle of zone B
    ('transport_1', 'transport', DOCK_ZONE, DOCK_POINTS[1][2], DOCK_POINTS[1][3], math.pi),  # S2
    ('transport_2', 'transport', DOCK_ZONE, DOCK_POINTS[2][2], DOCK_POINTS[2][3], math.pi),  # S3
]

WHITE, BLUE, GRAPHITE, STEEL, BLACK = (0.92, 0.93, 0.94), (0.1, 0.33, 0.72), (0.18, 0.19, 0.21), (0.62, 0.64, 0.66), (0.04, 0.04, 0.04)
ORANGE, DARK = (0.95, 0.45, 0.05), (0.27, 0.28, 0.3)


def box_at(m, mat, x0, x1, y0, y1, z0, z1):
    m.box(mat, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0))


def label_texture(path, text, colour):
    img = Image.new('RGB', (512, 128), tuple(int(c * 255) if isinstance(c, float) else c for c in colour))
    dr = ImageDraw.Draw(img)
    dr.text((256, 64), text, font=font(64), fill=(255, 255, 255), anchor='mm')
    img.save(path, quality=92)


def side_labels(m, mat, x, z, w, h, half_width):
    for side in (-1, 1):
        m.plate(mat, (x, side * (half_width + 0.002), z), (-side, 0, 0), (0, 0, 1), w, h)


def wheels(m, xs, half_width, r):
    for x in xs:
        for side in (-1, 1):
            y = side * (half_width - 0.05)
            m.cylinder_y('tyre', x, r, y - 0.03, y + 0.03, r)


# ------------------------------------------------------------------ picker

def build_picker():
    d = new_model_dir('picker_robot', keep=('plugins',))
    label_texture(d / 'materials/textures/label.jpg', 'NU-CE27  PICKER', BLUE)
    hx, hw = P_LENGTH / 2, P_WIDTH / 2
    front_top, rear_top = TRAY_ZERO - 0.025, 0.25

    m = Mesh()
    box_at(m, 'body', MAST_X + 0.03, hx - 0.03, -hw, hw, 0.02, front_top)
    box_at(m, 'body', -hx + 0.03, MAST_X + 0.03, -hw, hw, 0.02, rear_top)
    box_at(m, 'bumper', hx - 0.03, hx, -hw, hw, 0.02, front_top)
    box_at(m, 'bumper', -hx, -hx + 0.03, -hw, hw, 0.02, rear_top)
    box_at(m, 'laser', hx - 0.001, hx + 0.004, -0.08, 0.08, 0.03, 0.07)
    for side in (-1, 1):
        box_at(m, 'stripe', -hx + 0.03, MAST_X + 0.03, side * hw - 0.002, side * hw + 0.002, 0.2, 0.23)
    side_labels(m, 'label', (SLOT_X + (-hx)) / 2 + 0.25, 0.13, 0.5, 0.125, hw)
    wheels(m, (0.0,), hw, 0.07)
    for x in (-hx + 0.15, hx - 0.15):
        for side in (-1, 1):
            m.cylinder_y('tyre', x, 0.025, side * (hw - 0.08) - 0.02, side * (hw - 0.08) + 0.02, 0.025)
    # mast, rear uprights and top frame
    for side in (-1, 1):
        y0, y1 = sorted((side * (hw - 0.04), side * hw))
        box_at(m, 'mast', MAST_X - 0.025, MAST_X + 0.025, y0, y1, front_top, MAST_TOP)
        box_at(m, 'mast', -hx + 0.03, -hx + 0.07, y0, y1, rear_top, MAST_TOP)
        box_at(m, 'mast', -hx + 0.03, MAST_X + 0.025, y0, y1, MAST_TOP - 0.04, MAST_TOP)
    for x0, x1 in ((MAST_X - 0.025, MAST_X + 0.025), (-hx + 0.03, -hx + 0.07)):
        box_at(m, 'mast', x0, x1, -hw, hw, MAST_TOP - 0.04, MAST_TOP)
    for top in SLOT_TOPS:
        box_at(m, 'slot', SLOT_X - 0.31, SLOT_X + 0.31, -hw + 0.04, hw - 0.04, top - 0.02, top)
        for side in (-1, 1):
            box_at(m, 'mast', SLOT_X - 0.31, SLOT_X + 0.31, side * (hw - 0.04) - 0.01, side * (hw - 0.04) + 0.01,
                   top - 0.05, top)
    box_at(m, 'beacon', SLOT_X - 0.05, SLOT_X + 0.05, -0.05, 0.05, MAST_TOP, MAST_TOP + 0.06)
    m.write(d / 'meshes/base.dae', {'body': WHITE, 'bumper': BLACK, 'stripe': BLUE, 'laser': BLACK,
                                    'tyre': BLACK, 'mast': GRAPHITE, 'slot': STEEL, 'beacon': ORANGE,
                                    'label': 'label.jpg'})

    # carriage frame: tray top centre
    m = Mesh()
    box_at(m, 'tray', -0.47, 0.47, -0.41, 0.41, -0.02, 0)
    for side in (-1, 1):
        x = side * ARM_X[2]
        box_at(m, 'rail', x - 0.01, x + 0.01, -ARM_Y, ARM_Y, 0.04, 0.12)
        box_at(m, 'rail', MAST_X - TRAY_X + 0.025, -0.47, side * 0.4 - 0.03, side * 0.4 + 0.03, -0.02, 0.2)
    m.write(d / 'meshes/carriage.dae', {'tray': BLUE, 'rail': GRAPHITE})

    m = Mesh()
    for side in (-1, 1):
        x = side * ARM_X[1]
        box_at(m, 'stage', x - 0.01, x + 0.01, -ARM_Y, ARM_Y, 0.045, 0.115)
    m.write(d / 'meshes/stage.dae', {'stage': STEEL})

    m = Mesh()
    for side in (-1, 1):
        x = side * ARM_X[0]
        box_at(m, 'arm', x - 0.01, x + 0.01, -ARM_Y, ARM_Y, 0.05, 0.11)
        for y in (-0.2, 0.2):
            box_at(m, 'pad', x - side * 0.016, x - side * 0.01, y - 0.05, y + 0.05, 0.055, 0.105)
    m.write(d / 'meshes/arm.dae', {'arm': (0.85, 0.86, 0.88), 'pad': ORANGE})
    write_model_files(d, 'picker_robot', 'Case-handling picker robot: lifting tray with telescopic side arms, '
                                         f'{len(SLOT_TOPS)} storage slots.', robot_sdf('picker_robot', 'picker'))


def picker_links(name):
    hx, hw = P_LENGTH / 2, P_WIDTH / 2
    tray = f'{fmt(TRAY_X)} 0 {fmt(TRAY_ZERO)} 0 0 0'
    mu0 = '<surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>'
    collisions = [('chassis_front', (MAST_X + hx) / 2, 0, (TRAY_ZERO - 0.025) / 2,
                   hx - MAST_X, P_WIDTH, TRAY_ZERO - 0.025),
                  ('chassis_rear', (MAST_X - hx) / 2, 0, 0.125, hx + MAST_X, P_WIDTH, 0.25)]
    for side in (-1, 1):
        collisions.append((f'mast_{side}', MAST_X, side * (hw - 0.02), (0.25 + MAST_TOP) / 2, 0.05, 0.04, MAST_TOP - 0.25))
    col = ''.join(f'''
        <collision name="{n}">
          <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0</pose>
          <geometry><box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box></geometry>{mu0}
        </collision>''' for n, x, y, z, sx, sy, sz in collisions)

    def link(lname, pose, mass, size, mesh, extra=''):
        ix, iy, iz = (mass / 12 * (size[1] ** 2 + size[2] ** 2), mass / 12 * (size[0] ** 2 + size[2] ** 2),
                      mass / 12 * (size[0] ** 2 + size[1] ** 2))
        return f'''
      <link name="{lname}">
        <pose>{pose}</pose>
        <inertial><mass>{mass}</mass><inertia><ixx>{ix:.4g}</ixx><iyy>{iy:.4g}</iyy><izz>{iz:.4g}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>{extra}
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><mesh><uri>model://picker_robot/meshes/{mesh}.dae</uri></mesh></geometry>
        </visual>
      </link>'''

    def prismatic(jname, parent, child, axis, lower, upper):
        return f'''
      <joint name="{jname}" type="prismatic">
        <parent>{parent}</parent><child>{child}</child>
        <axis><xyz>{axis}</xyz><limit><lower>{lower}</lower><upper>{upper}</upper>
          <effort>5000</effort><velocity>1</velocity></limit></axis>
      </joint>'''

    return (link('base_link', '0 0 0 0 0 0', 250, (P_LENGTH, P_WIDTH, 0.4), 'base', col + laser_xml(name, P_LASER))
            + link('carriage', tray, 15, (0.94, 0.82, 0.1), 'carriage')
            + link('stage', tray, 2, (0.94, 0.84, 0.07), 'stage')
            + link('arm', tray, 2, (0.9, 0.84, 0.06), 'arm')
            + prismatic('lift_joint', 'base_link', 'carriage', '0 0 1', 0, LIFT_RANGE)
            + prismatic('reach_joint', 'carriage', 'stage', '0 1 0', -STAGE_TRAVEL, STAGE_TRAVEL)
            + prismatic('reach2_joint', 'stage', 'arm', '0 1 0', -STAGE_TRAVEL, STAGE_TRAVEL))


def picker_plugin():
    return f'''
        <role>picker</role>
        <max_speed>1.2</max_speed>
        <max_turn_rate>1.2</max_turn_rate>
        <acceleration>0.8</acceleration>
        <tray_x>{fmt(TRAY_X)}</tray_x>
        <tray_zero>{fmt(TRAY_ZERO)}</tray_zero>
        <reach_max>{fmt(2 * STAGE_TRAVEL)}</reach_max>
        <slot_x>{fmt(SLOT_X)}</slot_x>
        <slot_heights>{' '.join(fmt(z) for z in SLOT_TOPS)}</slot_heights>
        <aisle_half>{fmt(AISLE_HALF)}</aisle_half>
        <laser_pose>{' '.join(fmt(v) for v in P_LASER)} 0 0 0</laser_pose>'''


# --------------------------------------------------------------- transport

def roller_texture(path):
    img = Image.new('RGB', (512, 128))
    dr = ImageDraw.Draw(img)
    pitch = 512 // 20
    for k in range(20):
        for i in range(pitch):
            shade = int(95 + 90 * math.sin(math.pi * i / pitch)) if i < pitch - 3 else 35
            dr.line([(k * pitch + i, 0), (k * pitch + i, 127)], fill=(shade, shade, shade + 6))
    img.save(path, quality=92)


def build_transport():
    d = new_model_dir('transport_robot', keep=('plugins',))
    tex = d / 'materials/textures'
    roller_texture(tex / 'rollers.jpg')
    label_texture(tex / 'label.jpg', 'NU-CE27  TRANSPORT', (200, 90, 10))
    hx, hw = T_LENGTH / 2, T_WIDTH / 2
    m = Mesh()
    box_at(m, 'body', -hx + 0.03, hx - 0.03, -hw, hw, 0.03, 0.5)
    for s in (-1, 1):
        box_at(m, 'bumper', *sorted((s * hx, s * (hx - 0.03))), -hw, hw, 0.03, 0.22)
        box_at(m, 'light', -hx + 0.03, hx - 0.03, *sorted((s * hw, s * (hw + 0.004))), 0.24, 0.27)
    box_at(m, 'frame', -hx, hx, -hw, hw, 0.5, DECK_TOP - 0.04)
    box_at(m, 'frame', -DECK[0] / 2, DECK[0] / 2, -DECK[1] / 2, DECK[1] / 2, DECK_TOP - 0.04, DECK_TOP - 0.001)
    m.plate('rollers', (0, 0, DECK_TOP), (1, 0, 0), (0, 1, 0), DECK[0], DECK[1])
    box_at(m, 'laser', hx - 0.001, hx + 0.02, -0.07, 0.07, 0.15, 0.21)
    side_labels(m, 'label', 0, 0.38, 0.8, 0.2, hw)
    wheels(m, (-hx + 0.2, 0.0, hx - 0.2), hw, 0.08)
    m.write(d / 'meshes/base.dae', {'body': DARK, 'bumper': BLACK, 'light': ORANGE, 'frame': GRAPHITE,
                                    'rollers': 'rollers.jpg', 'laser': BLACK, 'tyre': BLACK, 'label': 'label.jpg'})
    write_model_files(d, 'transport_robot', 'Transport robot with a powered roller deck at conveyor height.',
                      robot_sdf('transport_robot', 'transport'))


def transport_links(name):
    hx, hw = T_LENGTH / 2, T_WIDTH / 2
    mass = 180
    return f'''
      <link name="base_link">
        <inertial><mass>{mass}</mass><inertia><ixx>{mass / 12 * (T_WIDTH ** 2 + 0.36):.4g}</ixx>
          <iyy>{mass / 12 * (T_LENGTH ** 2 + 0.36):.4g}</iyy><izz>{mass / 12 * (T_LENGTH ** 2 + T_WIDTH ** 2):.4g}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="body">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><box><size>{fmt(T_LENGTH)} {fmt(T_WIDTH)} 0.5</size></box></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>
        </collision>
        <collision name="deck">
          <pose>0 0 {fmt((0.5 + DECK_TOP) / 2)} 0 0 0</pose>
          <geometry><box><size>{fmt(T_LENGTH)} {fmt(T_WIDTH)} {fmt(DECK_TOP - 0.5)}</size></box></geometry>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><mesh><uri>model://transport_robot/meshes/base.dae</uri></mesh></geometry>
        </visual>{laser_xml(name, T_LASER)}
      </link>'''


def transport_plugin():
    return f'''
        <role>transport</role>
        <max_speed>1.5</max_speed>
        <max_turn_rate>1.5</max_turn_rate>
        <acceleration>0.8</acceleration>
        <deck_length>{fmt(DECK[0])}</deck_length>
        <deck_width>{fmt(DECK[1])}</deck_width>
        <deck_top>{fmt(DECK_TOP)}</deck_top>
        <roller_speed>0.4</roller_speed>
        <laser_pose>{' '.join(fmt(v) for v in T_LASER)} 0 0 0</laser_pose>'''


# ------------------------------------------------------------------ common

def laser_xml(name, pose):
    return f'''
        <sensor name="laser" type="gpu_ray">
          <pose>{' '.join(fmt(v) for v in pose)} 0 0 0</pose>
          <update_rate>10</update_rate>
          <ray>
            <scan><horizontal><samples>540</samples><resolution>1</resolution>
              <min_angle>-2.356</min_angle><max_angle>2.356</max_angle></horizontal></scan>
            <range><min>0.1</min><max>20</max><resolution>0.01</resolution></range>
            <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
          </ray>
          <plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
            <ros><namespace>/{name}</namespace><remapping>~/out:=scan</remapping></ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>{name}/laser</frame_name>
          </plugin>
        </sensor>'''


def robot_sdf(model, kind, name=None, pose='0 0 0 0 0 0'):
    name = name or model
    links = picker_links(name) if kind == 'picker' else transport_links(name)
    params = picker_plugin() if kind == 'picker' else transport_plugin()
    return f'''    <model name="{name}">
      <pose>{pose}</pose>{links}
      <plugin name="custom_robot" filename="/workspace/models/{model}/plugins/libcustom_robot.so">
        <ros><namespace>/{name}</namespace></ros>{params}
      </plugin>
    </model>
'''


def write_model_files(d, name, description, sdf):
    (d / 'model.sdf').write_text(f'<?xml version="1.0"?>\n<sdf version="1.6">\n{sdf}</sdf>\n')
    (d / 'model.config').write_text(f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>NU-CE27</name></author>
  <description>{description} Needs Sim/plugins/build_plugins.sh and Gazebo started with ROS 2.</description>
</model>
''')


def robot_pose(zone, lx, ly, yaw):
    _, _, i, j, th, *_ = next(z for z in ZONES if z[0] == zone)
    x, y = to_world(i, j, th, lx, ly)
    return x, y, th + yaw


def update_world():
    ROBOT_WORLD.parent.mkdir(exist_ok=True)
    shutil.copy(WORLD.parent / 'shipping_dock.yaml', ROBOT_WORLD.parent)
    text = WORLD.read_text()
    text = re.sub(r'    <!-- Robots -->\n.*?    <!-- /Robots -->\n', '', text, flags=re.S)
    parts = ['    <!-- Robots -->\n']
    for name, kind, zone, lx, ly, yaw in ROBOTS:
        x, y, th = robot_pose(zone, lx, ly, yaw)
        th = math.atan2(math.sin(th), math.cos(th))
        parts.append(robot_sdf(f'{kind}_robot', kind, name, f'{fmt(x)} {fmt(y)} 0.005 0 0 {fmt(th)}'))
    parts.append('    <!-- /Robots -->\n')
    gui = text.index("    <gui fullscreen='0'>")
    ROBOT_WORLD.write_text(text[:gui] + ''.join(parts) + text[gui:])


def write_robot_info():
    _, _, i, j, th, *_ = next(z for z in ZONES if z[0] == DOCK_ZONE)
    lines = ['# Warehouse robots (generated by Sim/tools/build_warehouse_robots.py). World frame: "map".',
             'robots:']
    for name, kind, zone, lx, ly, yaw in ROBOTS:
        x, y, a = robot_pose(zone, lx, ly, yaw)
        a = math.atan2(math.sin(a), math.cos(a))
        lines.append(f'  {name}: {{type: {kind}, start: {{x: {x:.3f}, y: {y:.3f}, yaw: {a:.3f}}}}}')
    lx, ly = BELT_START_X + T_LENGTH / 2 + 0.03, DOOR_Y
    x, y = to_world(i, j, th, lx, ly)
    a = math.atan2(math.sin(th + math.pi), math.cos(th + math.pi))
    lines += [
        'topics:  # under /<robot name>/',
        '  cmd_vel: geometry_msgs/Twist (differential drive, linear.x and angular.z)',
        '  odom: nav_msgs/Odometry',
        '  pose: geometry_msgs/PoseStamped (world frame "map")',
        '  scan: sensor_msgs/LaserScan (270 degrees, 20 m)',
        '  handler/command: std_msgs/String',
        '  handler/state: std_msgs/String (JSON, latched)',
        'picker:',
        f'  max_speed: 1.2',
        f'  footprint: {{length: {P_LENGTH}, width: {P_WIDTH}}}',
        f'  storage_slots: {len(SLOT_TOPS)}',
        f'  shelf_levels_5: [{", ".join(f"{z + 0.002:.3f}" for z in LEVEL_SETS[5])}]  # zones A and B',
        f'  shelf_levels_4: [{", ".join(f"{z + 0.002:.3f}" for z in LEVEL_SETS[4])}]  # zones C and D',
        '  commands:',
        '    - "lift <z>"                tray top to height z',
        '    - "pick <left|right>"       box beside the tray (drive so it is level with the tray, lift to its shelf)',
        '    - "store [slot]"            tray box into a storage slot (default: first free)',
        '    - "retrieve <slot>"         slot box onto the tray',
        '    - "place <left|right> [y]"  push the tray box out to y m from the centre line (default: shelf)',
        'transport:',
        f'  max_speed: 1.5',
        f'  footprint: {{length: {T_LENGTH}, width: {T_WIDTH}}}',
        f'  deck_top: {DECK_TOP}',
        '  commands:',
        '    - "unload <front|back>"     roll every deck box off that end',
        '  note: boxes placed on the deck are fixed to it automatically',
        f'conveyor_unload_pose: {{x: {x:.3f}, y: {y:.3f}, yaw: {a:.3f}}}  # transport front at the belt start, then "unload front"',
    ]
    (ROBOT_WORLD.parent / 'warehouse_robots.yaml').write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    build_picker()
    build_transport()
    update_world()
    write_robot_info()
    print(f'robots: {", ".join(r[0] for r in ROBOTS)}')
