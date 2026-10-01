#!/usr/bin/env python3
"""Demo: shelf -> picker -> transport robot -> conveyor -> truck, in the dock zone (zone D).

  1. transport_1 drives from its staging point to a hand-off spot next to row 2
  2. picker_1 drives along row 2, lifts to the 0.68 m shelf and picks box_D_015 (a long box)
  3. picker_1 lowers the box to deck height and places it onto transport_1
  4. transport_1 drives to the conveyor and rolls the box onto the belt, which carries it into the truck

Runs inside the simulation container (started by Sim/gzScripts/runCustomRobotWarehouse.sh);
use Sim/gzScripts/runCustomRobotDemo.sh to start it from the host.
Robots are steered with cmd_vel towards waypoints, using their /<robot>/pose topic (no Nav2).
"""
import json
import math
import re
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

WORLD = '/workspace/worlds/Custom_Robot_Warehouse/Custom_Robot_Warehouse.world'
BOX = 'box_D_015'

# zone D workcell frame -> world (same transform as Sim/tools/build_our_warehouse_storage.py)
C0, TILE, TH = (-0.040107, 0.052292), 20.1026, -math.pi / 2


def W(lx, ly):
    c, s = math.cos(TH), math.sin(TH)
    return C0[0] + c * lx - s * ly, C0[1] + TILE + s * lx + c * ly


def WYAW(local_yaw):
    return wrap(local_yaw + TH)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# Layout (zone D frame): row 2 west face boxes at x 1.095, the open dock floor west of x 0.6,
# row 1 at x -1.1 for y < -0.2, conveyor along y -1.3 starting at x -5.0.
PICKER_START = (0.1, 5.0, -math.pi / 2)        # in the free lane beside row 2, facing south
PICKER_CLEAR_Y = 4.0                           # where the picker backs off to after the hand-off
TRAY_X = 0.37                                  # tray ahead of the picker centre
HANDOFF_SIDE = 1.15                            # transport centre beside the picker centre line
UNLOAD = (-5.0 + 0.65 + 0.03, -1.3)            # transport front 3 cm before the belt start
DECK_TOP = 0.64


class Demo:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('warehouse_demo')
        self.pose, self.state, self.pubs, self.cmds = {}, {}, {}, {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for r in ('picker_1', 'transport_1'):
            self.pubs[r] = self.node.create_publisher(Twist, f'/{r}/cmd_vel', 10)
            self.cmds[r] = self.node.create_publisher(String, f'/{r}/handler/command', 10)
            self.node.create_subscription(PoseStamped, f'/{r}/pose', lambda m, r=r: self._pose(r, m), 10)
            self.node.create_subscription(String, f'/{r}/handler/state', lambda m, r=r: self._state(r, m), latched)

    def _pose(self, robot, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose[robot] = (msg.pose.position.x, msg.pose.position.y, yaw)

    def _state(self, robot, msg):
        self.state[robot] = json.loads(msg.data)
        self.state[robot]['_stamp'] = time.monotonic()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_for_robots(self):
        while not all(r in self.pose and r in self.state for r in ('picker_1', 'transport_1')):
            print('  waiting for the robots (is the simulation running?)')
            self.spin(1.0)

    # ------------------------------------------------------------ driving

    def drive(self, robot, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pubs[robot].publish(t)

    def stop(self, robot):
        for _ in range(5):
            self.drive(robot, 0, 0)
            self.spin(0.05)
        self.spin(0.5)

    def turn_to(self, robot, yaw, w_max=0.6):
        while True:
            self.spin(0.05)
            err = wrap(yaw - self.pose[robot][2])
            if abs(err) < 0.01:
                break
            w = max(min(2.0 * err, w_max), -w_max)
            self.drive(robot, 0, math.copysign(max(abs(w), 0.08), w))
        self.stop(robot)

    def go_to(self, robot, x, y, speed, reverse=False, tol=0.02):
        """Drive straight to (x, y): turn to face it, then follow the line with heading correction."""
        px, py, _ = self.pose[robot]
        heading = math.atan2(y - py, x - px) + (math.pi if reverse else 0.0)
        if math.hypot(x - px, y - py) > 0.1:
            self.turn_to(robot, wrap(heading))
        while True:
            self.spin(0.05)
            px, py, yaw = self.pose[robot]
            dx, dy = x - px, y - py
            dist = math.hypot(dx, dy)
            ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
            if reverse:
                ahead = -ahead
            if dist < tol or ahead < 0:
                break
            err = wrap(math.atan2(dy, dx) + (math.pi if reverse else 0.0) - yaw) if dist > 0.15 else 0.0
            v = min(speed, max(0.6 * dist, 0.06))
            self.drive(robot, -v if reverse else v, max(min(1.5 * err, 0.4), -0.4))
        self.stop(robot)

    def go_local(self, robot, lx, ly, speed, reverse=False):
        self.go_to(robot, *W(lx, ly), speed, reverse)

    # ------------------------------------------------------------ handler

    def command(self, robot, text, timeout=60.0):
        sent = time.monotonic()
        self.cmds[robot].publish(String(data=text))
        print(f'  {robot} <- "{text}"')
        while time.monotonic() - sent < timeout:
            self.spin(0.1)
            s = self.state[robot]
            if s['_stamp'] > sent and not s['busy']:
                print(f'  {robot}: {s["result"]}')
                if not s['result'].startswith('ok'):
                    raise SystemExit(f'{robot} failed: {s["result"]}')
                return s
        raise SystemExit(f'{robot}: no answer to "{text}"')


def gz_set(model, x, y, z, yaw=0.0):
    subprocess.run(['gz', 'model', '-m', model, '-x', str(x), '-y', str(y), '-z', str(z),
                    '-R', '0', '-P', '0', '-Y', str(yaw)], check=True, timeout=10)


def gz_pose(model):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True, timeout=10).stdout
    return [float(v) for v in out.split()]


def box_start_pose():
    world = open(WORLD).read()
    m = re.search(rf"<model name='{BOX}'>\s*<pose>([^<]+)</pose>", world)
    return [float(v) for v in m.group(1).split()]


def main():
    d = Demo()
    d.wait_for_robots()

    print('== 0. reset: picker_1 to the dock zone, transport_1 to staging point S2, box back on its shelf')
    bx, by, bz, _, _, byaw = box_start_pose()
    gz_set(BOX, bx, by, bz, byaw)
    gz_set('picker_1', *W(*PICKER_START[:2]), 0.005, WYAW(PICKER_START[2]))
    gz_set('transport_1', *W(-4.2, -3.7), 0.005, WYAW(math.pi))
    d.spin(2.0)
    if d.state['picker_1'].get('tray'):
        d.command('picker_1', 'store')                # free the tray, keep its box on board

    _, box_ly = local_of(bx, by)
    picker_y = box_ly + TRAY_X                       # facing south: tray is TRAY_X south of the centre
    handoff = (PICKER_START[0] - HANDOFF_SIDE, picker_y - TRAY_X)

    print('== 1. transport_1 drives to the hand-off spot')
    d.go_local('transport_1', -4.2, 0.9, 0.8)
    d.go_local('transport_1', handoff[0], 0.9, 0.8)
    d.turn_to('transport_1', WYAW(math.pi / 2))
    d.go_local('transport_1', *handoff, 0.3)

    print(f'== 2. picker_1 drives to {BOX}, lifts to its shelf and picks it')
    d.go_local('picker_1', PICKER_START[0], picker_y, 0.6)
    d.turn_to('picker_1', WYAW(-math.pi / 2))
    d.command('picker_1', f'lift {bz - 0.1:.3f}')      # long box: 0.2 m high, bottom = shelf top
    d.command('picker_1', 'pick left')

    print('== 3. picker_1 places the box onto transport_1')
    d.command('picker_1', f'lift {DECK_TOP}')
    d.command('picker_1', f'place right {HANDOFF_SIDE}')
    d.spin(2.0)
    print(f'  transport_1 deck: {d.state["transport_1"].get("deck")}')
    d.go_local('picker_1', PICKER_START[0], PICKER_CLEAR_Y, 0.6, reverse=True)

    print('== 4. transport_1 drives to the conveyor')
    d.go_local('transport_1', -3.0, handoff[1], 0.8)
    d.go_local('transport_1', -3.0, UNLOAD[1], 0.8)
    d.go_local('transport_1', *UNLOAD, 0.3)
    d.turn_to('transport_1', WYAW(math.pi))

    print('== 5. unload onto the belt, the belt carries the box into the truck')
    d.command('transport_1', 'unload front')
    for _ in range(45):
        d.spin(1.0)
        p = gz_pose(BOX)
        if p and p[1] > 31.0 and abs(p[2] - 0.7) < 0.05:
            break
    p = gz_pose(BOX)
    upright = abs(p[3]) < 0.2 and abs(p[4]) < 0.2
    print(f'  {BOX} at x {p[0]:.2f} y {p[1]:.2f} z {p[2]:.2f} ({"in the truck, upright" if p[1] > 31 and upright else "check it"})')
    d.node.destroy_node()
    rclpy.shutdown()


def local_of(x, y):
    dx, dy = x - C0[0], y - (C0[1] + TILE)
    c, s = math.cos(TH), math.sin(TH)
    return c * dx + s * dy, -s * dx + c * dy


if __name__ == '__main__':
    main()
