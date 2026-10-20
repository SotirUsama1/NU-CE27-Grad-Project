#!/usr/bin/env python3
"""Endurance & Stress Benchmark Suite for Nav2 (MPPI vs RPP).

Executes a sequence of high-difficulty warehouse navigation challenges across
narrow aisles, tight dead-ends, pallet staging bottlenecks, and cross-warehouse
transits over continuous endurance laps (1 to 2 hours).

Usage (inside container):
  python3 /main/scripts/run_stress_benchmark.py --laps 8 --controller mppi
  python3 /main/scripts/run_stress_benchmark.py --duration-mins 90 --controller rpp
"""
import argparse
import json
import math
from pathlib import Path
import time
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_from_msg(p):
    return [p.position.x, p.position.y, yaw_from_quat(p.orientation)]


def compute_errors(actual, target):
    dist = math.hypot(actual[0] - target[0], actual[1] - target[1])
    diff_yaw = math.atan2(math.sin(actual[2] - target[2]), math.cos(actual[2] - target[2]))
    return [dist, abs(diff_yaw)]


# High-difficulty warehouse waypoints designed to stress local planners
STRESS_WAYPOINTS = [
    {
        "name": "W01_Aisle1_Ingress",
        "desc": "90-degree intersection turn from open staging into narrow Aisle 1",
        "pose": [2.46, 3.5, 1.5708],
        "timeout": 120.0
    },
    {
        "name": "W02_Aisle1_DeepPocket",
        "desc": "Deep rack pocket navigation to dead-end of Aisle 1",
        "pose": [2.46, 7.8, 1.5708],
        "timeout": 150.0
    },
    {
        "name": "W03_North_Header_Transit",
        "desc": "Tight 90-degree turn out of Aisle 1 across northern crossway",
        "pose": [3.66, 8.2, 0.0],
        "timeout": 120.0
    },
    {
        "name": "W04_Aisle2_South_Run",
        "desc": "Full-length 14-meter narrow corridor run down Aisle 2",
        "pose": [3.66, -5.5, -1.5708],
        "timeout": 180.0
    },
    {
        "name": "W05_South_Header_Cross",
        "desc": "Southern cross-aisle transfer to Aisle 3 entry",
        "pose": [6.06, -6.5, 0.0],
        "timeout": 120.0
    },
    {
        "name": "W06_Aisle3_North_Transit",
        "desc": "Full northbound transit along eastern rack aisle 3",
        "pose": [6.06, 6.5, 1.5708],
        "timeout": 180.0
    },
    {
        "name": "W07_Pallet_Staging_Bottleneck",
        "desc": "High-risk squeeze between Zone A pallet stacks and rack A2_1",
        "pose": [0.4, 2.0, 3.1415],
        "timeout": 150.0
    },
    {
        "name": "W08_Workcell_Western_Slalom",
        "desc": "High-speed slalom past western machinery columns",
        "pose": [-6.5, 4.5, 1.5708],
        "timeout": 150.0
    },
    {
        "name": "W09_Loading_Dock_Approach",
        "desc": "Tight precision docking approach at southern loading wall",
        "pose": [-7.5, -4.5, -1.5708],
        "timeout": 150.0
    },
    {
        "name": "W10_Home_Return",
        "desc": "Return to default staging home position",
        "pose": [-1.5, 2.0, 0.0],
        "timeout": 120.0
    }
]


class StressValidator(Node):
    def __init__(self, output_dir, robot_type='husky'):
        super().__init__('stress_validator', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.robot_type = robot_type
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.latest = {}
        self.truth = None
        self.truth_twist = None
        self.feedback = {}
        self.received = {}
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        for topic, msg_type in [('/scan', LaserScan), ('/odometry/filtered', Odometry)]:
            self.create_subscription(msg_type, topic, lambda msg, t=topic: self.receive(t, msg), qos_profile_sensor_data)
        for topic in ['/model_states', '/gazebo/model_states']:
            self.create_subscription(ModelStates, topic, self.model_states, qos_profile_sensor_data)

    def receive(self, topic, msg):
        self.latest[topic] = msg
        self.received[topic] = time.monotonic()

    def model_states(self, msg):
        for name in [self.robot_type, 'husky', 'jackal']:
            if name in msg.name:
                i = msg.name.index(name)
                self.truth = pose_from_msg(msg.pose[i])
                t = msg.twist[i]
                self.truth_twist = [math.hypot(t.linear.x, t.linear.y), t.angular.z]
                self.received['truth'] = time.monotonic()
                break

    def await_future(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('ROS operation timed out')
        return future.result()

    def wait_ready(self, timeout=45):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.truth is not None and '/scan' in self.latest and '/odometry/filtered' in self.latest:
                try:
                    if self.buffer.can_transform('map', 'base_link', Time()) and self.action.wait_for_server(timeout_sec=0.5):
                        return True
                except TransformException:
                    pass
        return False

    def navigate_to(self, target, timeout=150.0):
        if not self.action.wait_for_server(timeout_sec=10.0):
            return {"pass": False, "error": "Action server /navigate_to_pose not available"}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target[0])
        goal.pose.pose.position.y = float(target[1])
        goal.pose.pose.orientation.z = math.sin(target[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(target[2] / 2.0)

        handle_future = self.action.send_goal_async(goal, feedback_callback=self.on_feedback)
        handle = self.await_future(handle_future, 15)
        if not handle.accepted:
            return {"pass": False, "error": "Goal was rejected by Nav2"}

        result_future = handle.get_result_async()
        start_time = time.monotonic()
        timed_out = False
        recoveries = 0

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - start_time
            if elapsed >= timeout:
                timed_out = True
                try:
                    self.await_future(handle.cancel_goal_async(), 10)
                except Exception:
                    pass
                break

        elapsed_total = time.monotonic() - start_time
        result = result_future.result() if result_future.done() else None
        status = result.status if result else GoalStatus.STATUS_ABORTED

        # Settle
        stop_start = time.monotonic()
        while time.monotonic() - stop_start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        gt_error = compute_errors(self.truth, target) if self.truth else [999, 999]
        recov = self.feedback.get('recoveries', 0)
        passed = (status == GoalStatus.STATUS_SUCCEEDED) and not timed_out and (gt_error[0] <= 0.35)

        return {
            "pass": passed,
            "status": int(status),
            "timed_out": timed_out,
            "elapsed_s": round(elapsed_total, 2),
            "recoveries": recov,
            "gt_error_xy": round(gt_error[0], 3),
            "gt_error_yaw": round(gt_error[1], 3),
            "final_truth": [round(v, 3) for v in self.truth] if self.truth else None
        }

    def on_feedback(self, msg):
        self.feedback = {
            'distance_remaining': msg.feedback.distance_remaining,
            'recoveries': msg.feedback.number_of_recoveries
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, default=8, help='Number of full warehouse circuits (default: 8)')
    parser.add_argument('--duration-mins', type=float, default=None, help='Target endurance duration in minutes')
    parser.add_argument('--controller', type=str, default='mppi', choices=['mppi', 'rpp', 'dwb'])
    parser.add_argument('--robot', type=str, default='husky', choices=['husky', 'jackal'])
    parser.add_argument('--output-dir', type=str, default='/main/ros_ws/log/nav2_benchmark/stress_results')
    args = parser.parse_args()

    rclpy.init()
    validator = StressValidator(args.output_dir, args.robot)
    print("=" * 80)
    print(f"Starting Nav2 Stress Benchmark Suite ({args.controller.upper()} on {args.robot.upper()})")
    print(f"Planned laps: {args.laps} | Target endurance: {args.duration_mins or 'N/A'} mins")
    print("=" * 80)

    print("Waiting for sensors and transforms...")
    if not validator.wait_ready(30):
        print("ERROR: Sensors, ground truth, or TF transforms not ready within 30s!", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    print("System READY. Beginning endurance circuit...\n")
    start_all = time.monotonic()
    all_results = []
    lap = 1

    while True:
        if args.duration_mins and (time.monotonic() - start_all) / 60.0 >= args.duration_mins:
            print(f"Target duration of {args.duration_mins} minutes reached!")
            break
        if not args.duration_mins and lap > args.laps:
            break

        print(f"\n>>> BEGINNING LAP {lap} (Elapsed: {(time.monotonic() - start_all)/60.0:.1f} mins) <<<")
        lap_passed = 0
        lap_recoveries = 0

        for wp in STRESS_WAYPOINTS:
            wp_name = wp['name']
            target = wp['pose']
            timeout = wp['timeout']
            print(f"  -> Leg: {wp_name} ({wp['desc']}) Target: {target}")

            res = validator.navigate_to(target, timeout=timeout)
            res['lap'] = lap
            res['waypoint'] = wp_name
            all_results.append(res)

            status_str = "PASS" if res['pass'] else "FAIL"
            print(f"     [{status_str}] Elapsed: {res['elapsed_s']}s | Recoveries: {res['recoveries']} | GT Err: {res['gt_error_xy']}m, {res['gt_error_yaw']}rad")
            if res['pass']:
                lap_passed += 1
            lap_recoveries += res['recoveries']

        print(f">>> FINISHED LAP {lap}: {lap_passed}/{len(STRESS_WAYPOINTS)} Legs Passed | {lap_recoveries} Total Recoveries <<<")
        lap += 1

    total_time_m = (time.monotonic() - start_all) / 60.0
    passed_legs = sum(1 for r in all_results if r['pass'])
    total_legs = len(all_results)
    total_recov = sum(r['recoveries'] for r in all_results)

    # Save summary report
    summary = {
        "controller": args.controller,
        "robot": args.robot,
        "total_wall_time_mins": round(total_time_m, 2),
        "total_laps": lap - 1,
        "total_legs": total_legs,
        "passed_legs": passed_legs,
        "pass_rate_pct": round((passed_legs / total_legs * 100.0) if total_legs else 0, 1),
        "total_recoveries": total_recov,
        "results": all_results
    }

    report_path = Path(args.output_dir) / f"stress_{args.controller}_{args.robot}_{int(time.time())}.json"
    report_path.write_text(json.dumps(summary, indent=2))
    print("\n" + "=" * 80)
    print(f"BENCHMARK COMPLETE in {total_time_m:.1f} minutes!")
    print(f"Legs Passed: {passed_legs}/{total_legs} ({summary['pass_rate_pct']}%)")
    print(f"Total Recoveries: {total_recov}")
    print(f"Detailed JSON report saved to: {report_path}")
    print("=" * 80)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

