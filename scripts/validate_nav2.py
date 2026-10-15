#!/usr/bin/env python3
"""Measure warehouse Nav2 in simulation; run inside the project's ROS container.

Requires the test world's gazebo_ros_state plugin for independent ground truth.
Examples:
  python3 /main/scripts/validate_nav2.py --output /tmp/check --check
  python3 /main/scripts/validate_nav2.py --output /tmp/route --goal=-0.5,2,0
  python3 /main/scripts/validate_nav2.py --output /tmp/cancel --goal=2.5,4,1.57 --cancel-after 3
Each invocation writes a JSON result and JSONL trajectory. Deadlines use wall time,
so a frozen simulator cannot leave the test waiting forever.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def pose(p):
    return [p.position.x, p.position.y, yaw(p.orientation)]


def errors(a, b):
    return [math.hypot(a[0]-b[0], a[1]-b[1]),
            abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2])))]


class Validator(Node):
    def __init__(self, output):
        super().__init__('nav2_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.latest, self.counts, self.received = {}, {}, {}
        self.truth, self.truth_twist, self.feedback = None, None, {}
        self.samples = []
        self.output = Path(output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.trace = self.output.with_suffix('.jsonl').open('w')
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.active_handle = None
        for topic, msg_type in [('/scan', LaserScan), ('/odometry/filtered', Odometry),
                                ('/cmd_vel', Twist), ('/husky_velocity_controller/cmd_vel_unstamped', Twist)]:
            self.create_subscription(msg_type, topic, lambda msg, t=topic: self.receive(t, msg), qos_profile_sensor_data)
        for topic in ['/model_states', '/gazebo/model_states']:
            self.create_subscription(ModelStates, topic, self.model_states, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ['/map', '/map_nav']:
            self.create_subscription(OccupancyGrid, topic, lambda msg, t=topic: self.receive(t, msg), map_qos)
        self.create_timer(0.2, self.sample)

    def receive(self, topic, msg):
        self.latest[topic] = msg
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.received[topic] = time.monotonic()

    def model_states(self, msg):
        if 'husky' in msg.name:
            i = msg.name.index('husky')
            self.truth = pose(msg.pose[i])
            t = msg.twist[i]
            self.truth_twist = [math.hypot(t.linear.x, t.linear.y), t.angular.z]
            self.received['truth'] = time.monotonic()

    def sample(self):
        s = {'sim_time': self.get_clock().now().nanoseconds / 1e9, 'wall_time': time.time()}
        if self.truth is not None:
            s.update(truth=self.truth, truth_twist=self.truth_twist,
                     truth_age=time.monotonic()-self.received['truth'])
        try:
            t = self.buffer.lookup_transform('map', 'base_link', Time())
            p, q = t.transform.translation, t.transform.rotation
            s['localized'] = [p.x, p.y, yaw(q)]
            s['tf_age'] = s['sim_time'] - (t.header.stamp.sec + t.header.stamp.nanosec / 1e9)
            if self.truth is not None:
                s['localization_error'] = errors(s['localized'], self.truth)
        except TransformException:
            pass
        for topic in ['/cmd_vel', '/husky_velocity_controller/cmd_vel_unstamped']:
            if topic in self.latest:
                t = self.latest[topic]
                s[topic] = [t.linear.x, t.angular.z]
                s[topic+'_age'] = time.monotonic()-self.received[topic]
        self.samples.append(s)
        self.trace.write(json.dumps(s)+'\n')
        self.trace.flush()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def await_future(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('ROS operation exceeded wall-clock deadline')
        return future.result()

    def warmup(self):
        end = time.monotonic() + 30
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.truth is not None and '/scan' in self.latest
                    and '/odometry/filtered' in self.latest and '/map_nav' in self.latest
                    and self.buffer.can_transform('map', 'base_link', Time())
                    and len(self.samples) >= 5):
                return
        raise TimeoutError('Sensors, ground truth, clock, or map transform not ready within 30 seconds')

    def check(self):
        self.warmup()
        self.spin_for(5)
        states = {}
        for name in ['map_server', 'map_server_nav', 'amcl', 'planner_server',
                     'controller_server', 'bt_navigator', 'behavior_server', 'velocity_smoother']:
            c = self.create_client(GetState, '/'+name+'/get_state')
            if c.wait_for_service(timeout_sec=3):
                states[name] = self.await_future(c.call_async(GetState.Request()), 5).current_state.label
            else:
                states[name] = 'unavailable'
            self.destroy_client(c)
        transforms = {}
        for parent, child in [('map','odom'), ('odom','base_link'), ('base_link','laser_link')]:
            transforms[parent+' -> '+child] = self.buffer.can_transform(parent, child, Time())
        scan = self.latest.get('/scan')
        maps = {k: {'width': self.latest[k].info.width, 'height': self.latest[k].info.height,
                    'resolution': self.latest[k].info.resolution}
                for k in ['/map', '/map_nav'] if k in self.latest}
        fresh = {k: time.monotonic()-self.received.get(k, 0) < 2
                 for k in ['/scan', '/odometry/filtered', 'truth']}
        clock_advancing = len(self.samples) > 2 and self.samples[-1]['sim_time'] > self.samples[0]['sim_time']
        return {'test': 'preflight', 'pass': all(v == 'active' for v in states.values())
                and all(transforms.values()) and all(fresh.values()) and len(maps) == 2 and clock_advancing,
                'lifecycle': states, 'transforms': transforms, 'counts': self.counts,
                'fresh_data': fresh, 'clock_advancing': clock_advancing, 'maps': maps,
                'scan_beams': len(scan.ranges) if scan else 0,
                'last_sample': self.samples[-1] if self.samples else None}

    def feedback_cb(self, message):
        f = message.feedback
        self.feedback = {'distance_remaining': f.distance_remaining, 'recoveries': f.number_of_recoveries}

    def goal(self, target, timeout, cancel_after):
        self.warmup()
        if not self.action.wait_for_server(timeout_sec=10):
            raise RuntimeError('NavigateToPose action server unavailable')
        if self.truth is None:
            raise RuntimeError('Independent Gazebo ground truth unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z = math.sin(target[2]/2)
        goal.pose.pose.orientation.w = math.cos(target[2]/2)
        handle = self.await_future(self.action.send_goal_async(goal, feedback_callback=self.feedback_cb), 10)
        if not handle.accepted:
            raise RuntimeError('Navigation goal rejected')
        self.active_handle = handle
        result_future = handle.get_result_async()
        start, first_sample, last_print = time.monotonic(), len(self.samples), 0
        canceled, timed_out, cancel_accepted = False, False, False
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic()-start
            if elapsed-last_print >= 10:
                print(json.dumps({'elapsed_s': round(elapsed, 1), 'target': target, **self.feedback,
                                  'truth': self.truth}), flush=True)
                last_print = elapsed
            if not canceled and ((cancel_after is not None and elapsed >= cancel_after) or elapsed >= timeout):
                timed_out = elapsed >= timeout
                response = self.await_future(handle.cancel_goal_async(), 10)
                cancel_accepted = bool(response.goals_canceling)
                canceled = True
                cancel_time = time.monotonic()
            if canceled and time.monotonic()-cancel_time > 15:
                raise TimeoutError('Navigation did not terminate after cancellation')
        result = result_future.result()
        self.active_handle = None
        stop_start = time.monotonic()
        stop_sim = self.get_clock().now().nanoseconds / 1e9
        stable_since = None
        # Physics may run slower than wall time. Require a sustained stop in
        # simulation time, with a wall deadline if physics or messages freeze.
        while time.monotonic() - stop_start < 15:
            rclpy.spin_once(self, timeout_sec=0.1)
            now_sim = self.get_clock().now().nanoseconds / 1e9
            stopped_now = (self.truth_twist is not None and abs(self.truth_twist[0]) < 0.03
                           and abs(self.truth_twist[1]) < 0.05
                           and time.monotonic()-self.received.get('truth', 0) < 2)
            if stopped_now:
                if stable_since is None:
                    stable_since = now_sim
                if now_sim-stable_since >= 0.5:
                    break
            else:
                stable_since = None
        self.sample()
        last = self.samples[-1]
        estimated_error = errors(last['localized'], target) if 'localized' in last else None
        truth_error = errors(self.truth, target)
        motion_samples = self.samples[first_sample:]
        expected = GoalStatus.STATUS_CANCELED if cancel_after is not None else GoalStatus.STATUS_SUCCEEDED
        stopped = abs(self.truth_twist[0]) < 0.03 and abs(self.truth_twist[1]) < 0.05
        passed = result.status == expected and stopped and not timed_out
        if cancel_after is None:
            passed = (passed and estimated_error is not None and estimated_error[0] <= 0.25
                      and estimated_error[1] <= 0.25 and truth_error[0] <= 0.25
                      and truth_error[1] <= 0.25 and last.get('truth_age', 999) < 2
                      and abs(last.get('tf_age', 999)) < 0.5)
        return {'test': 'cancel' if cancel_after is not None else 'navigation', 'pass': passed,
                'target': target, 'status': result.status, 'elapsed_wall_s': time.monotonic()-start,
                'feedback': self.feedback, 'timed_out': timed_out, 'cancel_accepted': cancel_accepted,
                'stopped': stopped, 'settle_sim_s': now_sim-stop_sim,
                'estimated_goal_error_m_rad': estimated_error,
                'ground_truth_goal_error_m_rad': truth_error, 'last_sample': last,
                'max_localization_error_m': max((s.get('localization_error', [0])[0] for s in motion_samples), default=None),
                'max_tf_age_s': max((s.get('tf_age', 0) for s in motion_samples), default=None)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, help='Output filename prefix')
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--goal', help='x,y,yaw in map frame, radians')
    p.add_argument('--timeout', type=float, default=180)
    p.add_argument('--cancel-after', type=float)
    args = p.parse_args()
    rclpy.init()
    node = Validator(args.output)
    try:
        result = node.check() if args.check else node.goal([float(v) for v in args.goal.split(',')], args.timeout, args.cancel_after)
    except Exception as exc:
        result = {'pass': False, 'error': str(exc)}
        if node.active_handle is not None:
            try:
                node.await_future(node.active_handle.cancel_goal_async(), 5)
            except Exception:
                pass
    finally:
        node.trace.close()
    node.output.with_suffix('.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
