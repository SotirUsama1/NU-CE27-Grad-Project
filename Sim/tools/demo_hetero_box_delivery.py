#!/usr/bin/env python3
"""Demo: four mobile manipulators put different cartons on the dock conveyor.

  mir_light (6-cup, UR)   -> box_D_006, 2 kg
  jackal   (6-cup, UR5e)  -> box_D_008, 2 kg  (low bay — small arm)
  husky    (12-cup, UR10e)-> box_D_001, 8 kg  (heavy carton)
  mir_ur   (12-cup, UR)   -> box_D_003, 8 kg

One robot at a time. Start ./Sim/gzScripts/runHeteroWarehouseWorld.sh first, then
./Sim/gzScripts/runHeteroBoxDemo.sh.
"""
import math
import re
import subprocess
import threading
import time

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

_GZ_LOCK = threading.Lock()
_PRINT_LOCK = threading.Lock()
_SPIN_LOCK = threading.Lock()
_NODE = None


def spin_ros(timeout_sec=0.02):
    """Single-threaded spin_once (kept locked so helpers stay re-entrant-safe)."""
    if _NODE is None:
        time.sleep(timeout_sec)
        return
    with _SPIN_LOCK:
        rclpy.spin_once(_NODE, timeout_sec=timeout_sec)

WORLD = '/workspace/worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world'

TWO = dict(name='box_D_006', depth=0.40, height=0.30, mass=2.0,
           x=-2.6568, y=21.46, z=1.992, yaw=-math.pi / 2)
# Jackal UR5e: low-mid 2 kg bay (floor bay at 0.25 is below the arm — use 0.78).
TWO_JACKAL = dict(name='box_D_008', depth=0.40, height=0.30, mass=2.0,
                  x=-5.5393, y=21.46, z=0.782, yaw=-math.pi / 2)
# Husky UR10e + large suction: 8 kg carton.
EIGHT_HUSKY = dict(name='box_D_001', depth=0.78, height=0.45, mass=8.0,
                   x=-5.5393, y=21.27, z=0.907, yaw=-math.pi / 2)
EIGHT = dict(name='box_D_003', depth=0.78, height=0.45, mass=8.0)

BELT_X, BELT_START_Y, BELT_TOP, BELT_SPEED = -1.3401, 25.155, 0.63, 0.5
CLEAR_Y = 28.0
OPEN_Y = 24.40
STANDOFF = 0.85
LOAD = (BELT_X, 24.55, math.pi / 2)

PARK = {
    'mir_ur': (-4.5, 23.2, math.pi),
    'mir_light': (2.5, 24.6, math.pi),
    # East of the zone-D faces so Jackal/Husky picks are not blocked.
    'husky': (7.0, 22.5, math.pi / 2),
    'jackal': (4.0, 22.0, math.pi / 2),
}

# Arm base (= UR base frame) relative to each robot's drive base_link.
# Yaw is ur_base (not ur_base_link): Clearpath mounts face -Y in the chassis frame.
MOUNTS = {
    'mir_ur': ((0.135, 0.0, 0.79), math.pi / 2 + math.pi),
    'mir_light': ((0.135, 0.0, 0.79), math.pi / 2 + math.pi),
    'husky': ((0.0812, 0.0, 0.295), -math.pi / 2),
    'jackal': ((0.0, 0.0, 0.234), -math.pi / 2),
}

DH_D = (0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655)
DH_A = (0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0)
DH_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
# UR5e uses shorter upper/forearm; keep UR10e DH for MiR/Husky and a UR5e set for Jackal.
DH_A_UR5E = (0.0, -0.425, -0.39225, 0.0, 0.0, 0.0)
DH_D_UR5E = (0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996)

PAD_FACE = 0.08
CUP_STROKE = 0.01
LIFT = 0.08          # clear the shelf lip; keep small so cups are not sheared
JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
JOINT_SPEED = 0.55   # slow while holding — vacuum lips cannot take fast shear


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _dh(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def _make_mount(xyz, yaw):
    t = np.eye(4)
    t[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0],
                 [math.sin(yaw), math.cos(yaw), 0],
                 [0, 0, 1]]
    t[:3, 3] = xyz
    return t


def log(msg):
    with _PRINT_LOCK:
        print(msg, flush=True)


def _gz_run(args, *, timeout=45.0, retries=4, check=False, capture=False):
    """Run a gz CLI call with retries — Gazebo stalls under parallel picks at low RTF."""
    last = None
    for attempt in range(retries):
        try:
            with _GZ_LOCK:
                return subprocess.run(
                    args, check=check, capture_output=capture, text=True, timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            last = exc
            time.sleep(0.4 * (attempt + 1))
    raise last


def gz_set(model, x, y, z, yaw=0.0):
    _gz_run(['gz', 'model', '-m', model, '-x', str(x), '-y', str(y), '-z', str(z),
             '-Y', str(yaw)], timeout=45.0)


def gz_pose(model):
    out = _gz_run(['gz', 'model', '-m', model, '-p'], timeout=45.0, capture=True).stdout
    return [float(v) for v in out.split()]


def belt(speed):
    _gz_run(['gz', 'topic', '-p', '/gazebo/default/dock_conveyor/speed',
             '-m', f'data: "{speed}"'], timeout=30.0, check=True)


def box_start_pose(name):
    world = open(WORLD).read()
    m = re.search(rf"<model name='{name}'>\s*<pose>([^<]+)</pose>", world)
    return [float(v) for v in m.group(1).split()]


class Robot:
    def __init__(self, node, name, *, cmd_topic, traj_topic, vacuum_ns, gt_topic,
                 joint_topics, mount_xyz, mount_yaw, dh_a=DH_A, dh_d=DH_D):
        self.node, self.name = node, name
        self.pose, self.joints = None, {}
        self._mount = _make_mount(mount_xyz, mount_yaw)
        self._dh_a, self._dh_d = dh_a, dh_d
        self.cmd = node.create_publisher(Twist, cmd_topic, 10)
        self.traj = node.create_publisher(JointTrajectory, traj_topic, 10)
        vac = vacuum_ns.rstrip('/')
        self.valve = node.create_client(SetBool, f'{vac}/suction/set_vacuum' if vac else '/suction/set_vacuum')
        vac_topic = f'{vac}/suction/vacuum' if vac else '/suction/vacuum'
        present_topic = f'{vac}/suction/part_present' if vac else '/suction/part_present'
        self.vacuum_kpa, self.part_present = 0.0, False
        node.create_subscription(Odometry, gt_topic, self._pose, 10)
        node.create_subscription(Float64, vac_topic, self._vacuum, 10)
        node.create_subscription(Bool, present_topic, self._present, 10)
        for topic in joint_topics:
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
            spin_ros(0.02)

    def wait_ready(self, timeout_wall=180.0):
        t0 = time.time()
        last_print = -1.0
        while time.time() - t0 < timeout_wall:
            spin_ros(0.05)
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
                log(f'  {self.name} ready')
                return
            if time.time() - last_print >= 2.0:
                log(f'  waiting for {self.name}: {", ".join(missing)}')
                last_print = time.time()
        raise SystemExit(
            f'{self.name} not ready after {timeout_wall:.0f}s ({", ".join(missing)})')

    def drive(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)
        return t

    def hold_cmd(self, twist, sim_dt=0.05):
        """Re-publish cmd while waiting for sim time so diff-drive does not time out at low RTF."""
        end = self.now() + sim_dt
        while self.now() < end:
            self.cmd.publish(twist)
            spin_ros(0.02)

    def stop(self):
        zero = self.drive(0, 0)
        for _ in range(3):
            self.hold_cmd(zero, 0.04)
        self.spin(0.12)

    def turn_to(self, yaw, w_max=0.75):
        while True:
            self.hold_arm()
            err = wrap(yaw - self.pose[2])
            if abs(err) < 0.01:
                break
            w = max(min(2.0 * err, w_max), -w_max)
            self.hold_cmd(self.drive(0, math.copysign(max(abs(w), 0.08), w)))
        self.stop()
        self.hold_arm()

    def go_to(self, x, y, speed, reverse=False, tol=0.03):
        px, py, _ = self.pose
        heading = math.atan2(y - py, x - px) + (math.pi if reverse else 0.0)
        if math.hypot(x - px, y - py) > 0.1:
            self.turn_to(wrap(heading))
        while True:
            self.hold_arm()
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
            self.hold_cmd(self.drive(-v if reverse else v, max(min(1.5 * err, 0.4), -0.4)))
        self.stop()
        self.hold_arm()

    def hold_arm(self):
        # Single setpoint — spamming dense trajectories makes the arms shake.
        q = self.q()
        self.publish([q], [0.3])

    def creep(self, distance, speed=0.12):
        yaw0 = self.pose[2]
        sx, sy = self.pose[0], self.pose[1]
        while True:
            self.hold_arm()
            px, py, yaw = self.pose
            travelled = (px - sx) * math.cos(yaw0) + (py - sy) * math.sin(yaw0)
            left = distance - travelled
            if abs(left) < 0.02:
                break
            err = wrap(yaw0 - yaw)
            v = math.copysign(min(speed, max(0.5 * abs(left), 0.04)), left)
            self.hold_cmd(self.drive(v, max(min(1.0 * err, 0.1), -0.1)))
        self.stop()
        self.hold_arm()

    def nudge_to(self, x, y, tol=0.01):
        while True:
            self.hold_arm()
            px, py, yaw = self.pose
            ahead = (x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)
            if abs(ahead) < tol:
                break
            self.hold_cmd(self.drive(
                math.copysign(min(0.1, max(0.5 * abs(ahead), 0.03)), ahead), 0))
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

    def send(self, points, seconds_each=None, wait=True):
        prev, t, times = self.q(), 0.0, []
        for p in points:
            t += seconds_each or max(np.abs(np.asarray(p) - prev).max() / JOINT_SPEED, 0.45)
            times.append(t)
            prev = np.asarray(p)
        self.publish(points, times)
        if not wait:
            return
        self.spin(t)
        # Tight tolerance — 0.02 left the pad short of the face (0 kPa seals).
        end = self.now() + 3.0
        while np.abs(self.q() - prev).max() > 0.005 and self.now() < end:
            self.spin(0.05)

    def move_joint(self, target, wait=True):
        self.send([target], wait=wait)

    def _tool(self, q):
        t = self._mount
        for i in range(6):
            t = t @ _dh(q[i], self._dh_d[i], self._dh_a[i], DH_ALPHA[i])
        return t

    def fk(self, q):
        t = self._tool(q)
        z = t[:3, 2]
        return t[:3, 3] + PAD_FACE * z, z

    def _link_points(self, q):
        t = self._mount
        pts = [t[:3, 3].copy()]
        for i in range(6):
            t = t @ _dh(q[i], self._dh_d[i], self._dh_a[i], DH_ALPHA[i])
            pts.append(t[:3, 3].copy())
        return pts

    def arm_clear(self, q):
        deck_z = self._mount[2, 3]
        for p in self._link_points(q)[1:5]:
            if p[2] < deck_z - 0.15:
                return False
            if abs(p[0]) < 0.40 and abs(p[1]) < 0.38 and p[2] < deck_z + 0.05:
                return False
        pad, _ = self.fk(q)
        return pad[2] >= deck_z - 0.2 and not (pad[2] < deck_z + 0.15 and pad[0] < 0.30)

    def _residual(self, q, pos, axis):
        t = self._tool(q)
        z = t[:3, 2]
        return np.concatenate([t[:3, 3] + PAD_FACE * z - pos, 0.5 * np.cross(z, axis), [0.5 * t[2, 0]]])

    def ik(self, pos, axis, seed, iters=200):
        pos, axis = np.asarray(pos, float), np.asarray(axis, float) / np.linalg.norm(axis)
        q = np.array(seed, float)
        for _ in range(iters):
            r = self._residual(q, pos, axis)
            if np.linalg.norm(r) < 1e-5:
                break
            jac = np.empty((7, 6))
            for j in range(6):
                dq = q.copy()
                dq[j] += 1e-6
                jac[:, j] = (self._residual(dq, pos, axis) - r) / 1e-6
            q -= np.linalg.solve(jac.T @ jac + 1e-4 * np.eye(6), jac.T @ r)
        t = self._tool(q)
        p, z = t[:3, 3] + PAD_FACE * t[:3, 2], t[:3, 2]
        ok = (np.linalg.norm(p - pos) < 1e-3 and z @ axis > 0.999
              and abs(t[2, 0]) < 0.01 and self.arm_clear(q))
        return q, ok

    def solve(self, pos, axis, current):
        q, ok = self.ik(pos, axis, current)
        if ok and np.abs(q - current).max() < 1.0:
            return q
        best = None
        seeds = [current] + [[q1, q2, q3, q4, q5, 0.0]
                             for q1 in np.linspace(-math.pi, math.pi, 8, endpoint=False)
                             for q2 in (-2.0, -1.0) for q3 in (1.5, -1.5)
                             for q4 in (-1.5, 0.0) for q5 in (1.57, -1.57)]
        for s in seeds:
            q, ok = self.ik(pos, axis, s, iters=100)
            if ok:
                q = np.array([qq - 2 * math.pi * round((qq - c) / (2 * math.pi))
                              for qq, c in zip(q, current)])
                cost = np.abs(q - current).sum()
                if best is None or cost < best[1]:
                    best = (q, cost)
        return None if best is None else best[0]

    def move_pad(self, pos, axis, steps=1, seconds=None):
        start, _ = self.fk(self.q())
        q, points = self.q(), []
        for i in range(1, steps + 1):
            aim = start + (np.asarray(pos) - start) * i / steps
            q = self.solve(aim, axis, q)
            if q is None:
                raise RuntimeError(f'{self.name}: no IK solution for pad at {np.round(aim, 3)}')
            points.append(q)
        self.send(points, None if seconds is None else seconds / steps)

    def vacuum(self, on):
        future = self.valve.call_async(SetBool.Request(data=on))
        t0 = time.time()
        while not future.done() and time.time() - t0 < 10.0:
            spin_ros(0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{self.name}: suction valve did not answer')
        end = self.now() + 2.5
        while self.part_present != on and self.now() < end:
            self.spin(0.05)
        self.spin(0.2)
        return self.part_present == on

    def to_base(self, x, y, z):
        px, py, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py), z])

    def dir_to_base(self, dx, dy):
        yaw = self.pose[2]
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * dx + s * dy, -s * dx + c * dy, 0.0])


def wake_box(name):
    """Gazebo sleeps idle cartons (link enabled=false); vacuum_gripper cannot seal on them."""
    pose = gz_pose(name)
    yaw = pose[5] if len(pose) > 5 else 0.0
    gz_set(name, pose[0], pose[1], pose[2] + 0.001, yaw)
    gz_set(name, pose[0], pose[1], pose[2], yaw)


def place_box(spec):
    gz_set(spec['name'], spec['x'], spec['y'], spec['z'] + 0.001, spec['yaw'])
    gz_set(spec['name'], spec['x'], spec['y'], spec['z'], spec['yaw'])


def stop_all(robots):
    for r in robots:
        r.stop()


def park_robot(r, *, drive_mir=True):
    """Fold the arm and clear the aisle. MiR only drives when drive_mir=True."""
    r.stop()
    try:
        r.vacuum(False)
    except Exception:
        pass
    r.move_joint(HOME, wait=False)
    park = PARK[r.name]
    if r.name in ('jackal', 'husky'):
        gz_set(r.name, park[0], park[1], 0.12 if r.name == 'jackal' else 0.15, park[2])
        r.spin(0.4)
        r.stop()
        r.move_joint(HOME)
    elif drive_mir:
        r.go_to(park[0], park[1], 0.55)
        r.turn_to(park[2])
        r.stop()
        r.move_joint(HOME)
    else:
        r.spin(0.3)
        r.move_joint(HOME)
        r.stop()


def assert_holding(r, name, mass, where):
    r.spin(0.15)
    log(f'  {r.name}@{where}: held={r.part_present}, vac={r.vacuum_kpa:.1f} kPa, '
        f'box z={gz_pose(name)[2]:.3f}')
    if not r.part_present or r.vacuum_kpa < 12.0:
        r.vacuum(False)
        raise RuntimeError(f'{name} dropped {where} ({mass:.0f} kg)')


def deliver(r, spec):
    name, depth, height, mass = spec['name'], spec['depth'], spec['height'], spec['mass']
    log(f'== {r.name} picks {name}: {mass:.0f} kg')
    face = spec['y'] + depth / 2
    # UR5e max reach ~0.85 m — stand closer than the MiR/UR10e heavy standoff.
    if r.name == 'jackal':
        standoff = 0.72
    elif mass >= 8.0:
        standoff = 1.05
    else:
        standoff = STANDOFF
    stand_y = face + standoff
    r.stop()
    r.move_joint(HOME)
    # Clearpath: teleport empty. MiR: drive (gz_set ignored under ros2_control).
    if r.name in ('jackal', 'husky'):
        gz_set(r.name, spec['x'], stand_y, 0.12 if r.name == 'jackal' else 0.15,
               -math.pi / 2)
        r.spin(1.0)
        r.stop()
    else:
        approach_x = spec['x'] - 0.8 if mass >= 8.0 else spec['x']
        r.go_to(approach_x, OPEN_Y, 0.45)
        r.go_to(spec['x'], OPEN_Y, 0.35)
        r.turn_to(-math.pi / 2)
        r.go_to(spec['x'], stand_y, 0.18 if mass >= 8.0 else 0.28)
        r.turn_to(-math.pi / 2)
        r.nudge_to(spec['x'], stand_y)
        r.stop()

    place_box(spec)
    wake_box(name)
    r.spin(0.5)

    into = r.dir_to_base(0.0, -1.0)
    fwd = 0.45 if r.name == 'jackal' else 0.70
    shelf = np.array([fwd, 0.0, spec['z']])
    q = r.solve(shelf, into, r.q())
    if q is None:
        raise RuntimeError(f'{r.name}: no IK solution for the shelf at {np.round(shelf, 3)}')
    r.move_joint(q)
    r.move_pad(r.to_base(spec['x'], face + 0.25, spec['z']), into, steps=3, seconds=1.5)
    r.move_pad(r.to_base(spec['x'], face + 0.05, spec['z']), into, steps=4, seconds=2.5)

    sealed = False
    for press in (CUP_STROKE + 0.001, 0.005, 0.001, -0.002, -0.005):
        live = gz_pose(name)
        face = live[1] + depth / 2
        r.move_pad(r.to_base(live[0], face + press, live[2]), into, steps=3, seconds=1.5)
        r.spin(0.8)
        if r.vacuum(True):
            sealed = True
            break
        r.vacuum(False)
        wake_box(name)
    if not sealed:
        raise RuntimeError(f'{name} did not seal ({r.vacuum_kpa:.1f} kPa)')
    log(f'  {r.name}: vacuum {r.vacuum_kpa:.1f} kPa, part present')
    r.spin(1.2)

    # Lift only — do NOT Cartesian-drag heavy cartons sideways (peels the lips).
    # The base reverse pulls the carton out of the bay with the arm locked.
    pad, _ = r.fk(r.q())
    r.move_pad(pad + np.array([0.0, 0.0, LIFT]), into, steps=3, seconds=2.5)
    assert_holding(r, name, mass, 'after lift')

    back = OPEN_Y - r.pose[1]
    log(f'  {r.name}: reversing {back:.2f} m (arm held still)')
    r.creep(-back, speed=0.06 if mass >= 8.0 else 0.08)
    r.stop()
    assert_holding(r, name, mass, 'after reverse')

    log(f'== {r.name} drives {name} to the conveyor')
    belt(0.0)
    # Never teleport while holding — gz_set on the base leaves the carton behind.
    r.go_to(LOAD[0], OPEN_Y, 0.22 if r.name in ('jackal', 'husky') else 0.30)
    assert_holding(r, name, mass, 'at open aisle')
    r.go_to(LOAD[0], LOAD[1], 0.16 if r.name in ('jackal', 'husky') else 0.25)
    r.turn_to(LOAD[2])
    r.nudge_to(LOAD[0], LOAD[1])
    r.turn_to(LOAD[2])
    r.stop()
    assert_holding(r, name, mass, 'at conveyor')

    # Place with Cartesian pad moves only — joint-space swings drop the carton.
    log(f'== {r.name} places {name} on the conveyor')
    forward = np.array([1.0, 0.0, 0.0])
    box = gz_pose(name)
    pad_now, _ = r.fk(r.q())
    # Keep the same pad↔box offset while we lower onto the belt.
    offset = r.to_base(box[0], box[1], box[2]) - pad_now
    target_box = np.array([BELT_X, BELT_START_Y + depth / 2 + 0.04,
                           BELT_TOP + height / 2 + 0.02])
    release_pad = r.to_base(*target_box) - offset
    if r.name == 'jackal':
        release_pad = np.array([
            float(np.clip(release_pad[0], 0.45, 0.65)),
            float(np.clip(release_pad[1], -0.15, 0.15)),
            float(np.clip(release_pad[2], 0.70, 0.95)),
        ])
    else:
        release_pad = np.array([
            max(float(release_pad[0]), 0.70),
            float(np.clip(release_pad[1], -0.20, 0.20)),
            max(float(release_pad[2]), 0.78),
        ])
    # Hover above the belt, then lower onto it.
    hover = release_pad + np.array([0.0, 0.0, 0.12])
    r.move_pad(hover, forward, steps=5, seconds=3.5)
    assert_holding(r, name, mass, 'above belt')
    r.move_pad(release_pad, forward, steps=4, seconds=3.0)
    assert_holding(r, name, mass, 'on belt')
    if not r.vacuum(False):
        raise RuntimeError(f'{name}: vacuum did not vent ({r.vacuum_kpa:.1f} kPa)')
    r.spin(0.4)
    # Ease the plate off, then fold.
    r.move_pad(release_pad + np.array([-0.10, 0.0, 0.08]), forward, steps=3, seconds=2.0)
    r.move_joint(HOME)
    pose = gz_pose(name)
    log(f'  {name} on the belt at x {pose[0]:.2f} y {pose[1]:.2f} z {pose[2]:.2f}')
    if pose[2] < BELT_TOP - 0.05:
        raise RuntimeError(f'{name} missed the belt (z={pose[2]:.2f})')
    belt(BELT_SPEED)
    for _ in range(40):
        r.spin(0.5)
        if gz_pose(name)[1] > CLEAR_Y:
            break
    pose = gz_pose(name)
    log(f'  {name} riding the belt at y {pose[1]:.2f} z {pose[2]:.2f}')

    park_robot(r)
    log(f'== {r.name} done')

def make_robots(node):
    mir_xyz, mir_yaw = MOUNTS['mir_ur']
    husky_xyz, husky_yaw = MOUNTS['husky']
    jackal_xyz, jackal_yaw = MOUNTS['jackal']
    return [
        Robot(node, 'mir_light',
              cmd_topic='/mir_light/diff_cont/cmd_vel_unstamped',
              traj_topic='/mir_light/joint_trajectory_controller/joint_trajectory',
              vacuum_ns='/mir_light', gt_topic='/mir_light/ground_truth',
              joint_topics=('/mir_light/joint_states', '/mir_light/joint_broadcaster/joint_states'),
              mount_xyz=mir_xyz, mount_yaw=mir_yaw),
        Robot(node, 'jackal',
              cmd_topic='/jackal/jackal_velocity_controller/cmd_vel_unstamped',
              traj_topic='/jackal/joint_trajectory_controller/joint_trajectory',
              vacuum_ns='/jackal', gt_topic='/jackal/ground_truth',
              joint_topics=('/jackal/joint_states',),
              mount_xyz=jackal_xyz, mount_yaw=jackal_yaw,
              dh_a=DH_A_UR5E, dh_d=DH_D_UR5E),
        Robot(node, 'husky',
              cmd_topic='/husky_velocity_controller/cmd_vel_unstamped',
              traj_topic='/joint_trajectory_controller/joint_trajectory',
              vacuum_ns='', gt_topic='/husky/ground_truth',
              joint_topics=('/joint_states',),
              mount_xyz=husky_xyz, mount_yaw=husky_yaw),
        Robot(node, 'mir_ur',
              cmd_topic='/mir_ur/diff_cont/cmd_vel_unstamped',
              traj_topic='/mir_ur/joint_trajectory_controller/joint_trajectory',
              vacuum_ns='/mir_ur', gt_topic='/mir_ur/ground_truth',
              joint_topics=('/mir_ur/joint_states', '/mir_ur/joint_broadcaster/joint_states'),
              mount_xyz=mir_xyz, mount_yaw=mir_yaw),
    ]


def main():
    global _NODE
    rclpy.init()
    node = rclpy.create_node('hetero_box_delivery_demo', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    _NODE = node

    light, jackal, husky, heavy = make_robots(node)
    for r in (light, jackal, husky, heavy):
        r.wait_ready()

    log('== reset robots and cartons')
    for r in (light, jackal, husky, heavy):
        try:
            r.vacuum(False)
        except Exception as exc:
            log(f'  warn: {r.name} vacuum off failed: {exc}')
        r.move_joint(HOME, wait=False)
    light.spin(2.0)  # one shared wait for all four HOME trajectories

    eight = box_start_pose(EIGHT['name'])
    EIGHT.update(x=eight[0], y=eight[1], z=eight[2], yaw=eight[5])
    for name, pose in PARK.items():
        gz_set(name, pose[0], pose[1], 0.05 if name in ('husky', 'jackal') else 0.02, pose[2])
    place_box(TWO)
    place_box(TWO_JACKAL)
    place_box(EIGHT_HUSKY)
    place_box(dict(name=EIGHT['name'], x=EIGHT['x'], y=EIGHT['y'], z=EIGHT['z'],
                   yaw=EIGHT['yaw']))
    belt(BELT_SPEED)
    robots = (light, jackal, husky, heavy)
    stop_all(robots)
    # Full park once. Later turns only freeze idle robots (no random driving).
    for r in robots:
        park_robot(r, drive_mir=True)
    log('== reset done, running one robot at a time')
    light.spin(0.5)

    jobs = (
        (light, TWO),
        (jackal, TWO_JACKAL),
        (husky, EIGHT_HUSKY),
        (heavy, EIGHT),
    )
    errors = []
    for r, spec in jobs:
        idle = [other for other, _ in jobs if other is not r]
        stop_all(idle)
        for other in idle:
            # Keep Clearpath out of the aisle; do not re-drive parked MiRs.
            park_robot(other, drive_mir=False)
        stop_all(idle)
        log(f'-- turn: {r.name} (others frozen)')
        try:
            deliver(r, spec)
        except Exception as exc:
            errors.append(f'{r.name}: {exc}')
            log(f'!! {r.name} failed: {exc}')
            try:
                r.stop()
                park_robot(r, drive_mir=True)
            except Exception:
                pass

    if errors:
        raise SystemExit('demo failed:\n  ' + '\n  '.join(errors))
    log('== all four cartons have gone onto the conveyor')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
