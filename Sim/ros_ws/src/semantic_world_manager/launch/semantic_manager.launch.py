import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("semantic_world_manager")
    params_file = os.path.join(package_share, "config", "params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "semantic_db_path",
                default_value="/main/maps/Our_Structured_Warehouse_semantic.json",
                description="Path to the Phase 1 semantic JSON database",
            ),
            DeclareLaunchArgument("target_frame", default_value="map"),
            DeclareLaunchArgument("api_port", default_value="8000"),
            Node(
                package="semantic_world_manager",
                executable="world_state_node",
                name="semantic_world_manager",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "semantic_db_path": LaunchConfiguration("semantic_db_path"),
                        "target_frame": LaunchConfiguration("target_frame"),
                        "api_port": ParameterValue(
                            LaunchConfiguration("api_port"), value_type=int
                        ),
                    },
                ],
            ),
        ]
    )
