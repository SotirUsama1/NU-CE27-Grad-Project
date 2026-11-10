#!/usr/bin/env python3
"""Move a robot's arm (urdf/ur_arm.urdf.xacro) to a named pose, through its arm_controller.

  ros2 run warehouse_husky arm_pose.py travel                  (husky1)
  ros2 run warehouse_husky arm_pose.py --robot jackal1 tuck
  ros2 run warehouse_husky arm_pose.py --list

Poses (joint angles in rad: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3):
  travel  folded over the robot, inside its outline and above the lidar: for driving
  tuck    bent up close to its base: the arm turns in this pose
  zero    stretched out forward (all joints 0): how the arm spawns in Gazebo

Every move goes in three steps: tuck without turning, turn while tucked, then into the pose.
So a stretched-out arm never sweeps sideways across racks or other robots. (Turning the arm while it
stands fully upright made Gazebo's physics blow up with the UR10e; tucked it is fine.)
The launch files run `arm_pose.py travel` once the arm controller has started.
"""
import argparse
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
          'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']
POSES = {
    'travel': [3.14, -3.14, 3.0, 3.28, 0.0, 0.0],
    'tuck': [3.14, -0.8, 2.6, -2.6, 0.0, 0.0],
    'zero': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}
STEP_S = 3.0   # seconds per step of a move


def current_pan(node, timeout):
    """The arm's shoulder_pan angle now, from joint_states (None if it does not come)."""
    got = {}
    sub = node.create_subscription(JointState, 'joint_states', lambda m: got.setdefault('m', m), 10)
    end = node.get_clock().now().nanoseconds + timeout * 1e9
    while 'm' not in got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    m = got.get('m')
    return m.position[m.name.index(JOINTS[0])] if m is not None and JOINTS[0] in m.name else None


def move(robot, pose, timeout=60.0):
    """Send the arm of `robot` to `pose` (tuck, turn, pose); True if it got there."""
    node = rclpy.create_node('arm_pose', namespace=robot)
    client = ActionClient(node, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
    try:
        if not client.wait_for_server(timeout_sec=timeout):
            print(f'{robot}: no arm_controller (is the robot started with an arm?)', file=sys.stderr)
            return False
        pan = current_pan(node, 10.0)
        if pan is None:
            print(f'{robot}: no joint_states from the arm', file=sys.stderr)
            return False
        target = POSES[pose]
        tuck = POSES['tuck'][1:]
        steps = [[pan] + tuck, [target[0]] + tuck, target]   # tuck without turning, turn, into the pose
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for i, positions in enumerate(steps, start=1):
            t = STEP_S * i
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=positions, time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=timeout)
        handle = sent.result()
        if handle is None or not handle.accepted:
            print(f'{robot}: arm_controller refused the move', file=sys.stderr)
            return False
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(node, done, timeout_sec=timeout)
        result = done.result()
        ok = result is not None and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        print(f'{robot}: arm {"reached" if ok else "did not reach"} {pose}')
        return ok
    finally:
        node.destroy_node()


def main():
    ap = argparse.ArgumentParser(description='Move a robot arm to a named pose')
    ap.add_argument('pose', nargs='?', choices=sorted(POSES))
    ap.add_argument('--robot', default='husky1', help='namespace of the robot (default husky1)')
    ap.add_argument('--list', action='store_true', help='print the poses')
    args, _ = ap.parse_known_args()   # ignore the --ros-args a launch file adds
    if args.list or not args.pose:
        for name, q in POSES.items():
            print(f'  {name:7s} {q}')
        return 0
    rclpy.init()
    try:
        return 0 if move(args.robot, args.pose) else 1
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
