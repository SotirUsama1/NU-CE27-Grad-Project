#!/usr/bin/env python3
"""Drive the Husky to the furthest point of the warehouse it can reach.

1. Reads the map (/map) and where the robot is (AMCL: map -> base_link).
2. Finds every cell the robot's centre can be in without touching anything, and which of those it
   can reach from where it stands (flood fill).
3. Among the reachable cells with room to turn on the spot, picks the one furthest from the robot
   (straight-line distance).
4. Sends it to Nav2 as a goal (navigate_to_pose) and reports progress until the robot arrives.

Needs navigation.launch.py running. Then:
  ros2 run warehouse_navigation go_far.py
  ros2 run warehouse_navigation go_far.py --ros-args -p dry_run:=true    (only print the point)
"""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

ROBOT_HALF_WIDTH = 0.345   # m, base_link to the side of the Husky


def near_obstacle(occupied, radius_cells):
    """Cells with an occupied cell within radius_cells (a disk around every obstacle)."""
    out = occupied.copy()
    h, w = occupied.shape
    r = int(math.ceil(radius_cells))
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if (dx or dy) and dx * dx + dy * dy <= radius_cells * radius_cells:
                # out[y, x] |= occupied[y + dy, x + dx], for the part of the grid both cover
                out[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)] |= \
                    occupied[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)]
    return out


def reachable_from(free, start):
    """Cells of `free` connected to `start` (row, col) through free cells, 8 neighbours."""
    reached = np.zeros_like(free)
    reached[start] = True
    frontier = reached.copy()
    while frontier.any():
        grown = frontier.copy()
        grown[1:, :] |= frontier[:-1, :]
        grown[:-1, :] |= frontier[1:, :]
        grown[:, 1:] |= grown[:, :-1].copy()
        grown[:, :-1] |= grown[:, 1:].copy()
        frontier = grown & free & ~reached
        reached |= frontier
    return reached


class GoFar(Node):
    def __init__(self):
        super().__init__('go_far')
        # The robot's centre must stay this far from obstacles to pass: half width (0.345 m) plus
        # the planner's margin (footprint_padding of the global costmap, 0.28 m)
        self.declare_parameter('pass_clearance', 0.625)
        # The goal needs this much room all round: the Husky's corners reach 0.61 m from its centre
        # when it turns on the spot, plus a margin for localization error
        self.declare_parameter('goal_clearance', 0.75)
        self.declare_parameter('dry_run', False)

        self.map = None
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_map(self, msg):
        self.map = msg

    def robot_pose(self):
        t = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def wait_for_inputs(self):
        self.get_logger().info('Waiting for the map and the robot position (AMCL)...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.map is not None and self.tf_buffer.can_transform('map', 'base_link', Time()):
                return self.robot_pose()

    def find_furthest(self, rx, ry):
        info = self.map.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        occupied = (grid >= 65) | (grid < 0)   # unknown cells count as occupied

        passable = ~near_obstacle(occupied, self.get_parameter('pass_clearance').value / res)
        roomy = ~near_obstacle(occupied, self.get_parameter('goal_clearance').value / res)

        start = (int((ry - oy) / res), int((rx - ox) / res))
        if not passable[start]:
            # the robot may stand closer to something than pass_clearance: start from the nearest passable cell
            rows, cols = np.nonzero(passable)
            k = np.argmin((rows - start[0]) ** 2 + (cols - start[1]) ** 2)
            start = (rows[k], cols[k])
        reached = reachable_from(passable, start)

        rows, cols = np.nonzero(reached & roomy)
        if len(rows) == 0:
            return None
        xs, ys = ox + (cols + 0.5) * res, oy + (rows + 0.5) * res
        k = int(np.argmax((xs - rx) ** 2 + (ys - ry) ** 2))
        self.get_logger().info(
            f'{reached.sum() * res * res:.0f} m2 of floor reachable, {len(rows) * res * res:.0f} m2 of it '
            'roomy enough for the goal')
        return float(xs[k]), float(ys[k])

    def go(self, gx, gy, yaw):
        if not self.nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose is not available: is navigation.launch.py running?')
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

        last_log = [self.get_clock().now()]

        def on_feedback(msg):
            fb = msg.feedback
            now = self.get_clock().now()
            if now - last_log[0] > Duration(seconds=5.0):
                last_log[0] = now
                self.get_logger().info(
                    f'{fb.distance_remaining:5.1f} m to go, {fb.navigation_time.sec} s driving, '
                    f'{fb.number_of_recoveries} recoveries')

        sent = self.nav.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Navigation failed (status {result.result().status})')
            return False
        x, y, _ = self.robot_pose()
        self.get_logger().info(f'Arrived at ({x:.2f}, {y:.2f}), {math.dist((x, y), (gx, gy)):.2f} m from the goal')
        return True


def main():
    rclpy.init()
    node = GoFar()
    try:
        rx, ry, _ = node.wait_for_inputs()
        far = node.find_furthest(rx, ry)
        if far is None:
            node.get_logger().error('No reachable goal with enough room found')
            return 1
        gx, gy = far
        # arrive facing away from the start, the way the robot is already heading
        yaw = math.atan2(gy - ry, gx - rx)
        node.get_logger().info(
            f'Robot at ({rx:.2f}, {ry:.2f}); furthest reachable point ({gx:.2f}, {gy:.2f}), '
            f'{math.dist((rx, ry), (gx, gy)):.1f} m away in a straight line')
        if node.get_parameter('dry_run').value:
            return 0
        return 0 if node.go(gx, gy, yaw) else 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
