import os
from pathlib import Path
from threading import Thread

import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .api_server import create_app
from .coordinate_utils import quaternion_to_yaw_deg, world_to_target
from .memory_core import ThreadSafeMemoryCore


class WorldStateNode(Node):
    def __init__(self):
        super().__init__("semantic_world_manager")
        self.declare_parameter("semantic_db_path", "")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("world_to_map_transform", [0.0, 0.0, 0.0])
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("api_host", "0.0.0.0")
        self.declare_parameter("api_port", 8000)

        self.target_frame = self.get_parameter("target_frame").value
        self.transform = tuple(self.get_parameter("world_to_map_transform").value)
        if len(self.transform) != 3:
            raise ValueError("world_to_map_transform must be [x, y, yaw_deg]")

        self.memory = ThreadSafeMemoryCore(self.target_frame, self.transform)
        semantic_db_path = self._resolve_semantic_db_path(
            self.get_parameter("semantic_db_path").value
        )
        self.get_logger().info(f"Loading semantic database from {semantic_db_path}")
        self.memory.seed_from_file(semantic_db_path)

        topic = self.get_parameter("model_states_topic").value
        self.create_subscription(ModelStates, topic, self._on_model_states, 10)
        self.api_host = self.get_parameter("api_host").value
        self.api_port = self.get_parameter("api_port").value

    def _resolve_semantic_db_path(self, configured_path):
        filename = "Our_Structured_Warehouse_semantic.json"
        candidates = []
        if configured_path:
            candidates.append(Path(configured_path))
        candidates.append(Path("/main/maps") / filename)
        workspace_root = os.environ.get("WORKSPACE_ROOT")
        if workspace_root:
            candidates.append(Path(workspace_root) / "maps" / filename)
        try:
            package_share = Path(get_package_share_directory("semantic_world_manager"))
            candidates.append(package_share / "maps" / filename)
        except (LookupError, RuntimeError):
            pass

        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
        searched = ", ".join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(f"Semantic database not found; searched: {searched}")

    def _on_model_states(self, message):
        self.memory.mark_telemetry_received()
        for entity_id, pose in zip(message.name, message.pose):
            if not self.memory.has_entity(entity_id):
                continue
            orientation = pose.orientation
            yaw_deg = quaternion_to_yaw_deg(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            loc, target_yaw = world_to_target(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                yaw_deg,
                self.transform,
            )
            self.memory.update_entity_pose(entity_id, loc, target_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = WorldStateNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        uvicorn.run(create_app(node.memory), host=node.api_host, port=node.api_port)
    finally:
        executor.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
