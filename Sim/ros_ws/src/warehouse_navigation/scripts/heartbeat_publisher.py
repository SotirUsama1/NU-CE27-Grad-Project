#!/usr/bin/env python3
"""Publish the robot's simulation-time heartbeat at 1 Hz.

Phase 5: one `std_msgs/msg/Header` per second on /{namespace}/heartbeat:
  * ``stamp``    = the current SIMULATION timestamp (Gazebo /clock, via
                   use_sim_time) - the orchestrator's liveness monitor checks
                   timeouts against this, never the wall clock, so a slow
                   real-time factor cannot produce false "robot is dead"
                   positives;
  * ``frame_id`` = the topological node ID nearest to the robot's AMCL pose
                   (nearest node of backend/data/topo_graph.json; empty string
                   until a pose arrives or while the graph file is missing).

The AMCL pose (x, y, yaw) itself is published by AMCL on /{namespace}/amcl_pose
(PoseWithCovarianceStamped); the orchestrator subscribes to that topic in
parallel (rosbridge_client.py) for continuous pose and 0.5 m junction-arrival
detection. The heartbeat only adds the simulation timestamp and the grounded
node ID - exactly what the Header can carry.

The node subscribes to the relative topic amcl_pose and publishes the relative
topic heartbeat: run it under the robot's namespace (namespace:=husky1) so both
resolve to /husky1/amcl_pose and /husky1/heartbeat.

The topological graph is loaded once at start-up from (in order):
  1. the ``topo_graph_path`` ROS parameter (absolute path; set by
     multi_robot.launch.py),
  2. the CYBERSCAPE_TOPO_GRAPH environment variable,
  3. <project root>/backend/data/topo_graph.json, found relative to this
     script (with colcon --symlink-install, __file__ is the source copy).

Usage (inside the container, after multi_robot.launch.py or manually):
  ros2 run warehouse_navigation heartbeat_publisher.py --ros-args -r __ns:=/husky1
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

DEFAULT_RATE = 1.0
UNKNOWN_NODE = ""


def yaw_from_quaternion(q) -> float:
    """Yaw (rad) of a geometry_msgs quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def default_topo_graph_path() -> str:
    """<project root>/backend/data/topo_graph.json, from this script's location.

    Walks up from scripts/heartbeat_publisher.py until a directory holding
    backend/data/topo_graph.json is found (the project root is five levels
    above this script; with colcon --symlink-install, __file__ resolves to the
    source copy, so this works from the install space too). Empty string when
    no ancestor has the file - the launch file's topo_graph_path argument or
    CYBERSCAPE_TOPO_GRAPH is the way to point at it in that case.
    """
    directory = os.path.dirname(os.path.realpath(__file__))
    for _ in range(8):
        candidate = os.path.join(directory, "backend", "data", "topo_graph.json")
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent
    return ""


def load_topo_nodes(path: str) -> list[tuple[str, float, float]]:
    """[(node_id, x, y)] of the topological graph, or [] if the file is missing."""
    try:
        with open(path, "r") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return []
    nodes = data.get("nodes", {})
    out = []
    for node_id, node in nodes.items():
        try:
            out.append((str(node_id), float(node["x"]), float(node["y"])))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def nearest_node(nodes: list[tuple[str, float, float]], x: float, y: float) -> str:
    """Node ID of the graph node closest to (x, y) by straight-line distance."""
    best, best_d = UNKNOWN_NODE, None
    for node_id, nx, ny in nodes:
        d = (nx - x) ** 2 + (ny - y) ** 2
        if best_d is None or d < best_d:
            best, best_d = node_id, d
    return best


class HeartbeatPublisher(Node):
    """1 Hz std_msgs/Header heartbeat with sim time and nearest topo node."""

    def __init__(self):
        super().__init__("heartbeat_publisher")
        self.declare_parameter("topo_graph_path", "")
        self.declare_parameter("rate", DEFAULT_RATE)
        self.declare_parameter("pose_topic", "amcl_pose")
        self.declare_parameter("heartbeat_topic", "heartbeat")

        path = str(self.get_parameter("topo_graph_path").value) or ""
        if not path:
            path = os.environ.get("CYBERSCAPE_TOPO_GRAPH", "") or default_topo_graph_path()
        self.topo_nodes = load_topo_nodes(path)
        if not self.topo_nodes:
            self.get_logger().warning(
                f"topological graph not loaded from {path!r}: heartbeat frame_id stays "
                "empty until the graph file exists (generate_topo_graph.py)"
            )
        else:
            self.get_logger().info(f"nearest-node source: {path} ({len(self.topo_nodes)} nodes)")

        self.pose = None  # (x, y, yaw) of the last AMCL pose
        # AMCL latches amcl_pose (transient local); a volatile subscriber is
        # compatible with that and with a plain publisher either way.
        self.create_subscription(
            PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value),
            self._on_pose, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.pub = self.create_publisher(Header, str(self.get_parameter("heartbeat_topic").value), 10)
        rate = float(self.get_parameter("rate").value) or DEFAULT_RATE
        self.create_timer(1.0 / rate, self._publish_heartbeat)

    # ---------------------------------------------------------------- inputs
    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q))

    # ---------------------------------------------------------------- output
    def _publish_heartbeat(self) -> None:
        header = Header()
        # Sim time: use_sim_time makes the node clock follow Gazebo /clock.
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = (
            nearest_node(self.topo_nodes, self.pose[0], self.pose[1]) if self.pose else UNKNOWN_NODE
        )
        self.pub.publish(header)


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
