#!/usr/bin/env python3
"""Demo: two MiR 250 + UR10e robots put three cartons on the conveyor.

  The six-cup robot (mir_light) carries
    1. box_D_006, 2 kg, set on the empty top shelf of its dock bay
    2. box_D_004, 4 kg, set on a clear dock bay at its own shelf height
       (its real bay is behind the 8 kg carton)
  The twelve-cup robot (mir_ur) carries
    3. box_D_003, 8 kg, from the 0.68 m shelf

  Each carton is backed straight out to the open dock before the robot turns, then
  set on the stopped belt. The belt takes it toward the truck before the next pick.

Start the simulation first, then Sim/gzScripts/runMirUrHeavyDemo.sh.
The original 2 kg conveyor demo is unchanged.
"""
import math
import re
import subprocess

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WORLD = '/workspace/worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world'

# Dock-facing poses. yaw -pi/2, so the face toward the dock is at y + depth/2.
TWO = dict(name='box_D_006', depth=0.40, height=0.30, mass=2.0,
           x=-2.6568, y=21.46, z=1.992, yaw=-math.pi / 2)
FOUR = dict(name='box_D_004', depth=0.40, height=0.20, mass=4.0,
            x=-1.690, y=21.46, z=1.362, yaw=-math.pi / 2)
EIGHT = dict(name='box_D_003', depth=0.78, height=0.45, mass=8.0)

BELT_X, BELT_START_Y, BELT_TOP, BELT_SPEED = -1.3401, 25.155, 0.63, 0.5
CLEAR_Y = 28.0          # belt has carried the carton past the load point

OPEN_Y = 24.40          # open dock. Back up to here before any turn with a carton
STANDOFF = 0.85         # pad distance in front of the base centre while sealing
LOAD = (BELT_X, 24.55, math.pi / 2)
PARK_HEAVY = (-4.5, 23.2, math.pi)  # west dock, clear of the other robot's road
PARK_LIGHT = (2.5, 24.6, math.pi)    # east dock

MOUNT_XYZ, MOUNT_YAW = (0.135, 0.0, 0.79), math.pi / 2 + math.pi
DH_D = (0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655)
DH_A = (0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0)
DH_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
PAD_FACE = 0.08
CUP_STROKE = 0.01
LIFT = 0.06
JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
JOINT_SPEED = 1.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _dh(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


_MOUNT = np.eye(4)
_MOUNT[:3, :3] = [[math.cos(MOUNT_YAW), -math.sin(MOUNT_YAW), 0],
                  [math.sin(MOUNT_YAW), math.cos(MOUNT_YAW), 0],
                  [0, 0, 1]]
_MOUNT[:3, 3] = MOUNT_XYZ


def _tool(q):
    t = _MOUNT
    for i in range(6):
        t = t @ _dh(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
    return t


def fk(q):
    t = _tool(q)
    z = t[:3, 2]
    return t[:3, 3] + PAD_FACE * z, z


def _link_points(q):
    """Base-frame origins after each UR joint. Used to reject poses that fold into the body."""
    t = _MOUNT
    pts = [t[:3, 3].copy()]
    for i in range(6):
        t = t @ _dh(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        pts.append(t[:3, 3].copy())
    return pts


def arm_clear(q):
    """True when the arm stays outside the MiR body instead of collapsing into it.

    A reach down to the belt is allowed: only reject links that sit inside the white
    body volume, not every low elbow that is still out in front of the base.
    """
    for p in _link_points(q)[1:5]:
        if p[2] < 0.55:
            return False
        if abs(p[0]) < 0.40 and abs(p[1]) < 0.38 and p[2] < 0.82:
            return False
    pad, _ = fk(q)
    return pad[2] >= 0.50 and not (pad[2] < 0.90 and pad[0] < 0.30)


def _residual(q, pos, axis):
    t = _tool(q)
    z = t[:3, 2]
    return np.concatenate([t[:3, 3] + PAD_FACE * z - pos, 0.5 * np.cross(z, axis), [0.5 * t[2, 0]]])


def ik(pos, axis, seed, iters=200):
    pos, axis = np.asarray(pos, float), np.asarray(axis, float) / np.linalg.norm(axis)
    q = np.array(seed, float)
    for _ in range(iters):
        r = _residual(q, pos, axis)
        if np.linalg.norm(r) < 1e-5:
            break
        jac = np.empty((7, 6))
        for j in range(6):
            dq = q.copy()
            dq[j] += 1e-6
            jac[:, j] = (_residual(dq, pos, axis) - r) / 1e-6
        q -= np.linalg.solve(jac.T @ jac + 1e-4 * np.eye(6), jac.T @ r)
    t = _tool(q)
    p, z = t[:3, 3] + PAD_FACE * t[:3, 2], t[:3, 2]
    ok = (np.linalg.norm(p - pos) < 1e-3 and z @ axis > 0.999
          and abs(t[2, 0]) < 0.01 and arm_clear(q))
    return q, ok


def solve(pos, axis, current):
    q, ok = ik(pos, axis, current)
    if ok and np.abs(q - current).max() < 1.0:
        return q
    best = None
    seeds = [current] + [[q1, q2, q3, q4, q5, 0.0]
                         for q1 in np.linspace(-math.pi, math.pi, 8, endpoint=False)
                         for q2 in (-2.0, -1.0) for q3 in (1.5, -1.5)
                         for q4 in (-1.5, 0.0) for q5 in (1.57, -1.57)]
    for s in seeds:
        q, ok = ik(pos, axis, s, iters=100)
        if ok:
            q = np.array([qq - 2 * math.pi * round((qq - c) / (2 * math.pi)) for qq, c in zip(q, current)])
            cost = np.abs(q - current).sum()
            if best is None or cost < best[1]:
                best = (q, cost)
    return None if best is None else best[0]


def gz_set(model, x, y, z, yaw=0.0):
    subprocess.run(['gz', 'model', '-m', model, '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)],
                   check=False, timeout=15)


def gz_pose(model):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True, timeout=10).stdout
    return [float(v) for v in out.split()]


def belt(speed):
    subprocess.run(['gz', 'topic', '-p', '/gazebo/default/dock_conveyor/speed', '-m', f'data: "{speed}"'],
                   check=True, timeout=10)


def box_start_pose(name):
    world = open(WORLD).read()
    m = re.search(rf"<model name='{name}'>\s*<pose>([^<]+)</pose>", world)
    return [float(v) for v in m.group(1).split()]


class Robot:
    def __init__(self, node, name, ns):
        self.node, self.name = node, name
        self.pose, self.joints = None, {}
        root = f'/{ns}' if ns else ''
        self.cmd = node.create_publisher(Twist, f'{root}/diff_cont/cmd_vel_unstamped', 10)
        self.traj = node.create_publisher(JointTrajectory, f'{root}/joint_trajectory_controller/joint_trajectory', 10)
        self.valve = node.create_client(SetBool, f'{root}/suction/set_vacuum')
        self.vacuum_kpa, self.part_present = 0.0, False
        node.create_subscription(Odometry, f'/{name}/ground_truth', self._pose, 10)
        node.create_subscription(Float64, f'{root}/suction/vacuum', self._vacuum, 10)
        node.create_subscription(Bool, f'{root}/suction/part_present', self._present, 10)
        for topic in (f'{root}/joint_states', f'{root}/joint_broadcaster/joint_states'):
            node.create_subscription(JointState, topic, self._joints, 10)

    def _vacuum(self, msg):
        self.vacuum_kpa = msg.data

    def _present(self, msg):
        self.part_present = msg.data

    def _pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self):
        while True:
            missing = []
            if self.pose is None:
                missing.append('ground truth')
            if not all(j in self.joints for j in JOINTS):
                missing.append('arm joints')
            if self.now() == 0:
                missing.append('sim clock')
            if not self.valve.service_is_ready():
                missing.append('suction')
            if not missing:
                return
            print(f'  waiting for {self.name}: {", ".join(missing)}')
            self.spin(1.0)

    def drive(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def stop(self):
        for _ in range(3):
            self.drive(0, 0)
            self.spin(0.04)
        self.spin(0.12)

    def turn_to(self, yaw, w_max=0.75):
        while True:
            self.hold_arm()
            self.spin(0.05)
            err = wrap(yaw - self.pose[2])
            if abs(err) < 0.01:
                break
            w = max(min(2.0 * err, w_max), -w_max)
            self.drive(0, math.copysign(max(abs(w), 0.08), w))
        self.stop()
        self.hold_arm()

    def go_to(self, x, y, speed, reverse=False, tol=0.03):
        px, py, _ = self.pose
        heading = math.atan2(y - py, x - px) + (math.pi if reverse else 0.0)
        if math.hypot(x - px, y - py) > 0.1:
            self.turn_to(wrap(heading))
        while True:
            self.hold_arm()
            self.spin(0.05)
            px, py, yaw = self.pose
            dx, dy = x - px, y - py
            dist = math.hypot(dx, dy)
            ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
            if reverse:
                ahead = -ahead
            if dist < tol or ahead < 0:
                break
            err = wrap(math.atan2(dy, dx) + (math.pi if reverse else 0.0) - yaw) if dist > 0.15 else 0.0
            v = min(speed, max(0.6 * dist, 0.05))
            self.drive(-v if reverse else v, max(min(1.5 * err, 0.4), -0.4))
        self.stop()
        self.hold_arm()

    def hold_arm(self):
        q = self.q()
        self.publish([q, q], [0.0, 1.0])

    def creep(self, distance, speed=0.12):
        yaw0 = self.pose[2]
        sx, sy = self.pose[0], self.pose[1]
        while True:
            self.hold_arm()
            self.spin(0.05)
            px, py, yaw = self.pose
            travelled = (px - sx) * math.cos(yaw0) + (py - sy) * math.sin(yaw0)
            left = distance - travelled
            if abs(left) < 0.02:
                break
            err = wrap(yaw0 - yaw)
            v = math.copysign(min(speed, max(0.5 * abs(left), 0.04)), left)
            self.drive(v, max(min(1.0 * err, 0.1), -0.1))
        self.stop()
        self.hold_arm()

    def nudge_to(self, x, y, tol=0.01):
        while True:
            self.hold_arm()
            self.spin(0.05)
            px, py, yaw = self.pose
            ahead = (x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)
            if abs(ahead) < tol:
                break
            self.drive(math.copysign(min(0.1, max(0.5 * abs(ahead), 0.03)), ahead), 0)
        self.stop()

    def q(self):
        return np.array([self.joints[j] for j in JOINTS])

    def publish(self, points, times):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        for p, t in zip(points, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in p]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            msg.points.append(pt)
        self.traj.publish(msg)

    def send(self, points, seconds_each=None):
        prev, t, times = self.q(), 0.0, []
        for p in points:
            t += seconds_each or max(np.abs(np.asarray(p) - prev).max() / JOINT_SPEED, 0.35)
            times.append(t)
            prev = np.asarray(p)
        self.publish(points, times)
        self.spin(t)
        end = self.now() + 2.0
        while np.abs(self.q() - prev).max() > 0.005 and self.now() < end:
            self.spin(0.05)

    def move_joint(self, target):
        self.send([target])

    def move_pad(self, pos, axis, steps=1, seconds=None):
        start, _ = fk(self.q())
        q, points = self.q(), []
        for i in range(1, steps + 1):
            aim = start + (np.asarray(pos) - start) * i / steps
            q = solve(aim, axis, q)
            if q is None:
                raise SystemExit(f'{self.name}: no IK solution for pad at {np.round(aim, 3)}')
            points.append(q)
        self.send(points, None if seconds is None else seconds / steps)

    def vacuum(self, on):
        future = self.valve.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None:
            raise SystemExit(f'{self.name}: suction valve did not answer')
        end = self.now() + 1.0
        while self.part_present != on and self.now() < end:
            self.spin(0.05)
        self.spin(0.15)
        return self.part_present == on

    def to_base(self, x, y, z):
        px, py, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py), z])

    def dir_to_base(self, dx, dy):
        yaw = self.pose[2]
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * dx + s * dy, -s * dx + c * dy, 0.0])


def place_box(spec):
    gz_set(spec['name'], spec['x'], spec['y'], spec['z'], spec['yaw'])


def deliver(r, spec):
    """Seal on the dock face, back straight out, and set the carton on the belt."""
    name, depth, height, mass = spec['name'], spec['depth'], spec['height'], spec['mass']
    print(f'== {r.name} picks {name}: {mass:.0f} kg')
    face = spec['y'] + depth / 2
    # The 8 kg bay sits next to the stairs. Stand farther out and creep in slowly so the
    # base does not wedge between the rack and the handrail.
    standoff = 1.05 if mass >= 8.0 else STANDOFF
    stand_y = face + standoff
    r.move_joint(HOME)
    # Stay west of the stairs, then slide onto the bay centre line in the open dock.
    approach_x = spec['x'] - 0.8 if mass >= 8.0 else spec['x']
    r.go_to(approach_x, OPEN_Y, 0.45)
    r.go_to(spec['x'], OPEN_Y, 0.35)
    r.turn_to(-math.pi / 2)
    r.go_to(spec['x'], stand_y, 0.18 if mass >= 8.0 else 0.28)
    r.turn_to(-math.pi / 2)
    r.nudge_to(spec['x'], stand_y)

    into = r.dir_to_base(0.0, -1.0)
    # Each carton is met at its own centre height. Joint-space move avoids Cartesian
    # paths that fold the arm through the white body.
    shelf = np.array([0.70, 0.0, spec['z']])
    q = solve(shelf, into, r.q())
    if q is None:
        raise SystemExit(f'{r.name}: no IK solution for the shelf at {np.round(shelf, 3)}')
    r.move_joint(q)
    r.move_pad(r.to_base(spec['x'], face + 0.25, spec['z']), into, steps=3, seconds=1.2)
    r.move_pad(r.to_base(spec['x'], face + 0.05, spec['z']), into, steps=4, seconds=2.0)
    live = gz_pose(name)
    face = live[1] + depth / 2
    r.move_pad(r.to_base(live[0], face + CUP_STROKE + 0.001, live[2]), into, steps=3, seconds=1.2)
    if not r.vacuum(True):
        r.vacuum(False)
        raise SystemExit(f'{name} did not seal ({r.vacuum_kpa:.1f} kPa)')
    print(f'  vacuum {r.vacuum_kpa:.1f} kPa, part present')

    r.move_pad(fk(r.q())[0] + np.array([0.0, 0.0, LIFT]), into, steps=2, seconds=1.0)
    back = OPEN_Y - r.pose[1]
    print(f'  reversing {back:.2f} m straight before turning')
    r.creep(-back, speed=0.14 if mass >= 8.0 else 0.18)
    r.turn_to(-math.pi / 2)
    into = r.dir_to_base(0.0, -1.0)
    carry_z = max(1.05, 0.95 + height / 2)
    carry = np.array([0.60, 0.0, carry_z])
    q = solve(carry, into, r.q())
    if q is None:
        raise SystemExit(f'{r.name}: no clear carry pose at {np.round(carry, 3)}')
    r.move_joint(q)
    r.spin(0.2)
    print(f'  held: {r.part_present}, vacuum {r.vacuum_kpa:.1f} kPa, box z {gz_pose(name)[2]:.3f}')
    if not r.part_present:
        r.vacuum(False)
        raise SystemExit(f'{name} dropped while lifting ({mass:.0f} kg)')

    print(f'== {r.name} drives {name} to the conveyor')
    belt(0.0)
    r.go_to(LOAD[0], LOAD[1], 0.40)
    r.turn_to(LOAD[2])
    r.nudge_to(LOAD[0], LOAD[1])
    r.turn_to(LOAD[2])
    if not r.part_present:
        raise SystemExit(f'{name} dropped while driving')

    forward = np.array([1.0, 0.0, 0.0])
    pad_now, _ = fk(r.q())
    offset = r.to_base(*gz_pose(name)[:3]) - pad_now
    drop = r.to_base(BELT_X, BELT_START_Y + depth / 2 + 0.06, BELT_TOP + height / 2 + 0.01)
    drop_pad = drop - offset
    # Keep the pad well out over the belt. A low, close pad is rejected as folded into
    # the body, which aborted the release even though the carton was still held.
    release = np.array([max(float(drop_pad[0]), 0.72), 0.0, max(float(drop_pad[2]), 0.78)])
    high = np.array([release[0], 0.0, max(float(pad_now[2]), 1.15)])
    q = solve(high, forward, r.q())
    if q is None:
        raise SystemExit(f'{r.name}: no clear pose above the belt at {np.round(high, 3)}')
    r.move_joint(q)
    q = solve(release, forward, r.q())
    if q is None:
        raise SystemExit(f'{r.name}: no clear release pose at {np.round(release, 3)}')
    r.move_joint(q)
    if not r.vacuum(False):
        raise SystemExit(f'{name}: the vacuum did not vent ({r.vacuum_kpa:.1f} kPa)')
    # Ease the plate off the face before going home. A pure upward move kicks the carton
    # (especially the long 4 kg). Stay at least 0.58 m ahead of the base so IK stays clear.
    r.spin(0.35)
    back = release - 0.12 * forward + np.array([0.0, 0.0, 0.10])
    back[0] = max(float(back[0]), 0.58)
    q = solve(back, forward, r.q())
    if q is None:
        raise SystemExit(f'{r.name}: no clear retract pose at {np.round(back, 3)}')
    r.move_joint(q)
    r.move_joint(HOME)
    pose = gz_pose(name)
    print(f'  {name} on the belt at x {pose[0]:.2f} y {pose[1]:.2f} z {pose[2]:.2f}')
    belt(BELT_SPEED)
    for _ in range(30):
        r.spin(0.5)
        if gz_pose(name)[1] > CLEAR_Y:
            break
    pose = gz_pose(name)
    print(f'  {name} riding the belt at y {pose[1]:.2f} z {pose[2]:.2f}')


def main():
    rclpy.init()
    node = rclpy.create_node('mir_ur_two_robot_demo', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    light = Robot(node, 'mir_light', 'mir_light')
    heavy = Robot(node, 'mir_ur', '')
    light.wait_ready()
    heavy.wait_ready()

    print('== reset both robots and the three cartons')
    light.vacuum(False)
    heavy.vacuum(False)
    light.move_joint(HOME)
    heavy.move_joint(HOME)
    eight = box_start_pose(EIGHT['name'])
    EIGHT.update(x=eight[0], y=eight[1], z=eight[2], yaw=eight[5])
    gz_set('mir_light', PARK_LIGHT[0], PARK_LIGHT[1], 0.02, PARK_LIGHT[2])
    gz_set('mir_ur', *PARK_HEAVY[:2], 0.02, PARK_HEAVY[2])
    place_box(TWO)
    place_box(FOUR)
    gz_set(EIGHT['name'], EIGHT['x'], EIGHT['y'], EIGHT['z'], EIGHT['yaw'])
    belt(BELT_SPEED)
    light.spin(0.8)

    deliver(light, TWO)
    deliver(light, FOUR)
    print('== small-cup robot parks clear of the dock')
    light.go_to(PARK_LIGHT[0], PARK_LIGHT[1], 0.55)
    light.turn_to(PARK_LIGHT[2])

    deliver(heavy, EIGHT)
    print('== all three cartons have gone onto the conveyor')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
