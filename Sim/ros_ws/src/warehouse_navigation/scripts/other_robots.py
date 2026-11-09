#!/usr/bin/env python3
"""Let this robot's costmaps see the whole body of every other robot.

Why: the lidar sits on top of the Husky, 0.36 m above the floor, and its beam passes over another Husky's
body (about 0.3 m high). It only hits the other robot's 10 cm lidar puck, so Nav2 took a 1 x 0.7 m robot for a
thin pole and drove close enough to touch it.

What: this node takes this robot's real scan (scan) and shortens every beam that would hit another robot's
outline, as if the lidar could see the other bodies, and publishes the result on scan_costmap. The costmaps
use scan_costmap (warehouse_nav/nav2_launch.py); AMCL keeps the real scan, because other robots are not on
the map.

  * Other robots are found by their /<name>/tf topics, so a new robot (a second Husky, a Jackal) needs no setup.
  * A robot's outline is its own costmap footprint (/<name>/local_costmap/published_footprint), so every robot
    type gets its real size.
  * A robot's position is its map -> base_link from its own TF tree (/<name>/tf), fresh at the TF rate.
  * A robot whose position is older than STALE_S (navigation stopped) is ignored.

Started by navigation.launch.py, one per robot, in the robot's namespace:
  ros2 run warehouse_navigation other_robots.py --ros-args -r __ns:=/husky1 -r /tf:=tf -r /tf_static:=tf_static
"""
import math
import re

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

STALE_S = 2.0          # ignore a robot whose map -> base_link is older than this (s, sim time)
DISCOVER_S = 2.0       # look for new robots this often (s)
TF_QOS = QoSProfile(depth=100)
TF_STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)


def pose_2d(transform):
    """(x, y, yaw) of a geometry_msgs Transform."""
    q = transform.rotation
    return (transform.translation.x, transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def place(points, x, y, yaw):
    """Points (N x 2) given in a frame at (x, y, yaw), expressed in the parent frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + (x, y)


def express_in(points, x, y, yaw):
    """Points (N x 2) of the parent frame, expressed in a frame at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (points - (x, y)) @ np.array([[c, -s], [s, c]])


class OtherRobot:
    """Another robot: its own TF tree and its outline."""

    def __init__(self, node, name):
        self.node = node
        self.name = name
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.outline = None   # N x 2, in its base_link
        node.create_subscription(TFMessage, f'/{name}/tf', lambda m: self._on_tf(m, False), TF_QOS)
        node.create_subscription(TFMessage, f'/{name}/tf_static', lambda m: self._on_tf(m, True), TF_STATIC_QOS)
        node.create_subscription(PolygonStamped, f'/{name}/local_costmap/published_footprint',
                                 self._on_footprint, 5)

    def _on_tf(self, msg, static):
        for t in msg.transforms:
            if static:
                self.tf.set_transform_static(t, 'other_robots')
            else:
                self.tf.set_transform(t, 'other_robots')

    def _on_footprint(self, msg):
        # The footprint is published in that robot's odom frame; keep it in its base_link, where it does not move
        try:
            t = self.tf.lookup_transform('base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        pts = np.array([[p.x, p.y] for p in msg.polygon.points])
        if len(pts) >= 3:
            self.outline = place(pts, *pose_2d(t.transform))

    def outline_in_map(self):
        """Its outline in the map frame, or None if unknown or out of date."""
        if self.outline is None:
            return None
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        age = (self.node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        if age > STALE_S:
            return None
        return place(self.outline, *pose_2d(t.transform))


class OtherRobotsScan(Node):
    def __init__(self):
        super().__init__('other_robots',
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.me = self.get_namespace().strip('/')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)   # /tf, remapped by the launch file to this robot's tree
        self.others = {}
        self.pub = self.create_publisher(LaserScan, 'scan_costmap', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(DISCOVER_S, self.discover)
        self.discover()
        self.get_logger().info(f'{self.me}: publishing scan_costmap (scan plus the outlines of other robots)')

    def discover(self):
        for topic, _ in self.get_topic_names_and_types():
            m = re.fullmatch(r'/([A-Za-z0-9_]+)/tf', topic)
            if m and m.group(1) != self.me and m.group(1) not in self.others:
                self.others[m.group(1)] = OtherRobot(self, m.group(1))
                self.get_logger().info(f'{self.me}: now seeing robot {m.group(1)}')

    def on_scan(self, scan):
        ranges = np.array(scan.ranges, dtype=float)
        outlines = [o for o in (r.outline_in_map() for r in self.others.values()) if o is not None]
        if outlines:
            try:
                laser = pose_2d(self.tf.lookup_transform('map', scan.header.frame_id, Time()).transform)
            except Exception:
                laser = None
            if laser is not None:
                hit = self.ray_cast(scan, [express_in(o, *laser) for o in outlines])
                ranges = np.minimum(ranges, np.maximum(hit, scan.range_min))
        out = LaserScan()
        out.header, out.angle_min, out.angle_max = scan.header, scan.angle_min, scan.angle_max
        out.angle_increment, out.time_increment, out.scan_time = scan.angle_increment, scan.time_increment, scan.scan_time
        out.range_min, out.range_max, out.intensities = scan.range_min, scan.range_max, scan.intensities
        out.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(out)

    @staticmethod
    def ray_cast(scan, outlines):
        """Distance along each beam to the nearest outline edge (inf where a beam hits none). Outlines in the
        laser frame. An outline the laser is inside of (robots overlapping) is skipped."""
        n = len(scan.ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        d = np.stack([np.cos(angles), np.sin(angles)], axis=1)                    # B x 2 beam directions
        best = np.full(n, np.inf)
        for poly in outlines:
            if inside(poly):
                continue
            p, e = poly, np.roll(poly, -1, axis=0) - poly                          # M edges: p + u * e
            cross = lambda a, b: a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]     # noqa: E731
            denom = cross(d[:, None, :], e[None, :, :])                             # B x M
            with np.errstate(divide='ignore', invalid='ignore'):
                t = cross(p[None, :, :], e[None, :, :]) / denom                     # distance along the beam
                u = cross(p[None, :, :], d[:, None, :]) / denom                     # position along the edge
            ok = (np.abs(denom) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
            best = np.minimum(best, np.where(ok, t, np.inf).min(axis=1))
        return best


def inside(poly):
    """True if the origin is inside the polygon (N x 2)."""
    x, y = poly[:, 0], poly[:, 1]
    xn, yn = np.roll(x, -1), np.roll(y, -1)
    crosses = ((y > 0) != (yn > 0)) & (0 < (xn - x) * (0 - y) / np.where(yn == y, 1e-12, yn - y) + x)
    return bool(np.count_nonzero(crosses) % 2)


def main():
    rclpy.init()
    node = OtherRobotsScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
