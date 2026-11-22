#!/usr/bin/env python3
"""Demo: the MiR 250 + UR10e takes a box from a shelf to the LOAD station and onto the conveyor.

  1. drive from the dock corridor to row 1 of zone D and face box_D_006 (2 kg, 0.68 m shelf)
  2. put the suction cup on the box's front face and switch the vacuum on; it holds the box by
     suction force and friction, and the vacuum switch confirms the grip
  3. pull the box out, hold it in front of the robot and drive to the LOAD station (S1)
  4. stop the belt, lower the box onto it, release it and restart the belt: the box rides into the truck

Runs inside the simulation container (started by Sim/gzScripts/runMirUrWarehouse.sh);
use Sim/gzScripts/runMirUrDemo.sh to start it from the host. The demo resets the robot and the
box first, so it can be run again. The robot is steered with cmd_vel towards waypoints using its
ground-truth pose (no Nav2); the arm uses a numeric IK on the UR10e kinematics (no MoveIt).
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
ROBOT = 'mir_ur'
BOX = 'box_D_006'
BOX_DEPTH, BOX_HEIGHT = 0.4, 0.3

BELT_X, BELT_START_Y, BELT_TOP, BELT_SPEED = -1.3401, 25.155, 0.63, 0.3
TRUCK_Y = 31.0

# Robot poses (x, y, yaw) on the open dock floor north of row 1 (rack front at y 21.675).
START = (0.5, 23.0, math.pi)
PICK_Y = 22.70        # robot centre while picking: front edge 0.6 m from the rack
TURN_Y = 23.0         # room to turn with the box held 0.95 m ahead of the centre
LOAD = (BELT_X, 24.55, math.pi / 2)   # S1 LOAD, front edge 0.2 m before the belt start

# Box held in front of the robot (base_footprint frame) while driving: bottom 0.85 m, above
# the robot top (0.79 m) and the belt rails (0.73 m).
CARRY_BOX = (0.75, 0.0, 1.0)
LIFT = 0.06           # off the shelf before pulling out (0.28 m free above the box)
READY_PAD = (0.55, 0.0, 1.2)   # pad in base_footprint before approaching a shelf

# Arm mount on base_footprint (mir_250_v1.urdf.xacro: ur_base_link, then UR's base -> DH base)
# and the UR10e DH parameters (ur_description config/ur10e/default_kinematics.yaml).
MOUNT_XYZ, MOUNT_YAW = (0.135, 0.0, 0.79), math.pi / 2 + math.pi
DH_D = (0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655)
DH_A = (0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0)
DH_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
PAD_FACE = 0.08       # gripper plate face in front of ur_tool0 (mir_ur.urdf.xacro)
CUP_STROKE = 0.01     # the bellows cups stand this far in front of the plate face
JOINTS = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
JOINT_SPEED = 0.5     # rad/s for the slowest-moving trajectories


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ------------------------------------------------------------------ arm kinematics

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
    """Pad face position and approach axis (tool z) in base_footprint."""
    t = _tool(q)
    z = t[:3, 2]
    return t[:3, 3] + PAD_FACE * z, z


def _residual(q, pos, axis):
    t = _tool(q)
    z = t[:3, 2]
    # The plate's long side (tool x) stays horizontal so the cups fit the low box faces.
    return np.concatenate([t[:3, 3] + PAD_FACE * z - pos, 0.5 * np.cross(z, axis), [0.5 * t[2, 0]]])


def ik(pos, axis, seed, iters=200):
    """Damped least squares on position, approach axis and a horizontal plate."""
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
    ok = np.linalg.norm(p - pos) < 1e-3 and z @ axis > 0.999 and abs(t[2, 0]) < 0.01
    return q, ok


def ik_best(pos, axis, current):
    """IK from the current joints, else from many seeds; the solution closest to the current joints."""
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
    if best is None:
        raise SystemExit(f'no IK solution for pad at {np.round(pos, 3)} pointing {np.round(axis, 2)}')
    return best[0]


# ------------------------------------------------------------------ gazebo helpers

def gz_set(model, x, y, z, yaw=0.0):
    subprocess.run(['gz', 'model', '-m', model, '-x', str(x), '-y', str(y), '-z', str(z),
                    '-R', '0', '-P', '0', '-Y', str(yaw)], check=True, timeout=10)


def gz_pose(model):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True, timeout=10).stdout
    return [float(v) for v in out.split()]


def belt(speed):
    subprocess.run(['gz', 'topic', '-p', '/gazebo/default/dock_conveyor/speed', '-m', f'data: "{speed}"'],
                   check=True, timeout=10)


def box_start_pose():
    world = open(WORLD).read()
    m = re.search(rf"<model name='{BOX}'>\s*<pose>([^<]+)</pose>", world)
    return [float(v) for v in m.group(1).split()]


# ------------------------------------------------------------------ robot

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('mir_ur_demo', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pose, self.joints = None, {}
        self.cmd = self.node.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.traj = self.node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.valve = self.node.create_client(SetBool, '/suction/set_vacuum')
        self.vacuum_kpa, self.part_present = 0.0, False
        self.node.create_subscription(Odometry, f'/{ROBOT}/ground_truth', self._pose, 10)
        self.node.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.node.create_subscription(Float64, '/suction/vacuum', self._vacuum, 10)
        self.node.create_subscription(Bool, '/suction/part_present', self._present, 10)

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
        """Simulation time (the simulation may run slower than real time)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self):
        while (self.pose is None or not all(j in self.joints for j in JOINTS) or self.now() == 0
               or not self.valve.service_is_ready()):
            print('  waiting for the robot (is the simulation running?)')
            self.spin(1.0)

    # driving -------------------------------------------------------

    def drive(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def stop(self):
        for _ in range(5):
            self.drive(0, 0)
            self.spin(0.05)
        self.spin(0.5)

    def turn_to(self, yaw, w_max=0.5):
        while True:
            self.spin(0.05)
            err = wrap(yaw - self.pose[2])
            if abs(err) < 0.01:
                break
            w = max(min(2.0 * err, w_max), -w_max)
            self.drive(0, math.copysign(max(abs(w), 0.08), w))
        self.stop()

    def go_to(self, x, y, speed, reverse=False, tol=0.02):
        """Drive straight to (x, y): turn to face it, then follow the line with heading correction."""
        px, py, _ = self.pose
        heading = math.atan2(y - py, x - px) + (math.pi if reverse else 0.0)
        if math.hypot(x - px, y - py) > 0.1:
            self.turn_to(wrap(heading))
        while True:
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

    def nudge_to(self, x, y, tol=0.01):
        """Remove the remaining along-heading error by driving straight forward or back."""
        while True:
            self.spin(0.05)
            px, py, yaw = self.pose
            ahead = (x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)
            if abs(ahead) < tol:
                break
            self.drive(math.copysign(min(0.1, max(0.5 * abs(ahead), 0.03)), ahead), 0)
        self.stop()

    # arm -----------------------------------------------------------

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
        """Joint trajectory through `points`; waits until it has finished."""
        prev, t, times = self.q(), 0.0, []
        for p in points:
            t += seconds_each or max(np.abs(np.asarray(p) - prev).max() / JOINT_SPEED, 1.0)
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
        """Pad face along a straight line to `pos` (base_footprint), approach axis `axis`."""
        start, _ = fk(self.q())
        q, points = self.q(), []
        for i in range(1, steps + 1):
            q = ik_best(start + (np.asarray(pos) - start) * i / steps, axis, q)
            points.append(q)
        self.send(points, None if seconds is None else seconds / steps)

    def vacuum(self, on):
        """Switch the vacuum valve; waits for the vacuum switch to confirm grip or release."""
        future = self.valve.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None:
            raise SystemExit('/suction/set_vacuum did not answer')
        end = self.now() + 1.5
        while self.part_present != on and self.now() < end:
            self.spin(0.05)
        self.spin(0.3)
        return self.part_present == on

    def to_base(self, x, y, z):
        """World point -> base_footprint (the robot must be standing still)."""
        px, py, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py), z])

    def dir_to_base(self, dx, dy):
        yaw = self.pose[2]
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * dx + s * dy, -s * dx + c * dy, 0.0])


def main():
    r = Robot()
    r.wait_ready()

    print(f'== 0. reset: robot to the dock corridor, {BOX} back on its shelf, belt running')
    r.vacuum(False)
    r.move_joint(HOME)
    gz_set(ROBOT, START[0], START[1], 0.02, START[2])
    bx, by, bz, _, _, byaw = box_start_pose()
    gz_set(BOX, bx, by, bz, byaw)
    belt(BELT_SPEED)
    r.spin(2.0)

    face_y = by + BOX_DEPTH / 2          # the box faces the dock (+y)
    print(f'== 1. drive to row 1 and face {BOX} ({bz - BOX_HEIGHT / 2:.2f} m shelf)')
    r.go_to(bx, START[1], 0.5)
    r.turn_to(-math.pi / 2)
    r.go_to(bx, PICK_Y, 0.2)
    r.turn_to(-math.pi / 2)

    print('== 2. suction pad onto the box, lift it and pull it out')
    into = r.dir_to_base(0.0, -1.0)
    # A joint-space swing straight to the box brushes boxes on low shelves, so swing to a ready
    # pose well short of the rack first and go on in a straight line.
    r.move_pad(READY_PAD, into)
    r.move_pad(r.to_base(bx, face_y + 0.35, bz), into, steps=6)
    r.move_pad(r.to_base(bx, face_y + 0.05, bz), into, steps=6, seconds=3.0)
    bx, by, bz = gz_pose(BOX)[:3]
    face_y = by + BOX_DEPTH / 2
    # Stop with the cup lips just short of the face; the vacuum pulls the box onto the cups.
    r.move_pad(r.to_base(bx, face_y + CUP_STROKE + 0.001, bz), into, steps=4, seconds=2.0)
    if not r.vacuum(True):
        r.vacuum(False)
        raise SystemExit(f'no vacuum on {BOX} ({r.vacuum_kpa:.1f} kPa): the cup did not seal')
    print(f'  vacuum {r.vacuum_kpa:.1f} kPa, part present')
    lifted = fk(r.q())[0] + np.array([0.0, 0.0, LIFT])
    r.move_pad(lifted, into, steps=3, seconds=1.5)
    r.move_pad(lifted - 0.45 * np.asarray(into), into, steps=8, seconds=4.0)
    carry_pad = np.array(CARRY_BOX) - np.array([BOX_DEPTH / 2, 0.0, 0.0])
    forward = np.array([1.0, 0.0, 0.0])
    r.move_pad(carry_pad, forward, steps=8, seconds=3.0)
    if not r.part_present:
        raise SystemExit(f'{BOX} dropped while pulling it out (z {gz_pose(BOX)[2]:.2f})')

    print('== 3. stop the belt and drive to the LOAD station with the box')
    r.go_to(bx, TURN_Y, 0.2, reverse=True)
    belt(0.0)                            # the belt would also push the held box
    r.go_to(LOAD[0], LOAD[1], 0.3)
    r.turn_to(LOAD[2])
    r.nudge_to(LOAD[0], LOAD[1])
    r.turn_to(LOAD[2])
    print('  at LOAD: x {:.2f} y {:.2f} yaw {:.2f}, box z {:.2f}, vacuum {:.1f} kPa'.format(
        *r.pose, gz_pose(BOX)[2], r.vacuum_kpa))
    if not r.part_present:
        raise SystemExit(f'{BOX} dropped while driving')

    print('== 4. put the box on the stopped belt, release it and restart the belt')
    # The box can sag or creep on the cup, so place by where the box really is.
    pad_now, _ = fk(r.q())
    offset = r.to_base(*gz_pose(BOX)[:3]) - pad_now
    drop = r.to_base(BELT_X, BELT_START_Y + BOX_DEPTH / 2 + 0.06, BELT_TOP + BOX_HEIGHT / 2 + 0.01)
    drop_pad = drop - offset
    r.move_pad([drop_pad[0], drop_pad[1], pad_now[2]], forward, steps=4, seconds=2.0)
    r.move_pad(drop_pad, forward, steps=6, seconds=3.0)
    if not r.vacuum(False):
        raise SystemExit(f'the vacuum did not vent ({r.vacuum_kpa:.1f} kPa)')
    r.move_pad(drop_pad + np.array([0.0, 0.0, 0.2]), forward, steps=3, seconds=1.5)
    r.move_joint(HOME)
    p = gz_pose(BOX)
    print(f'  {BOX} on the belt at x {p[0]:.2f} y {p[1]:.2f} z {p[2]:.2f}')
    belt(BELT_SPEED)

    for _ in range(40):
        r.spin(1.0)
        p = gz_pose(BOX)
        if p[1] > TRUCK_Y:
            break
    upright = abs(p[3]) < 0.2 and abs(p[4]) < 0.2
    in_truck = p[1] > TRUCK_Y and upright
    print(f'  {BOX} at x {p[0]:.2f} y {p[1]:.2f} z {p[2]:.2f} ({"in the truck, upright" if in_truck else "check it"})')
    r.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
