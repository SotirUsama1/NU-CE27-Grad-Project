"""Show the warehouse map in RViz with the Husky and its lidar on top.

Loads maps/warehouse.yaml with Nav2's map_server and places the robot on the map with a fixed
map -> odom transform at its spawn pose (no localization yet; Nav2's AMCL replaces this later).

Start the Husky first (warehouse_husky husky.launch.py), then:
  ros2 launch warehouse_navigation view_map.launch.py
  ros2 launch warehouse_navigation view_map.launch.py x:=2.0 y:=1.0 yaw:=1.57   (same as the spawn pose)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_navigation')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': os.path.join(pkg_share, 'maps', 'warehouse.yaml'), 'use_sim_time': True}],
        output='screen',
    )
    # map_server is a lifecycle node: it only publishes /map once configured and activated
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'node_names': ['map_server'], 'autostart': True, 'use_sim_time': True}],
        output='screen',
    )

    # odom starts where the robot spawned, so map -> odom is the spawn pose
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', LaunchConfiguration('x'), '--y', LaunchConfiguration('y'), '--yaw', LaunchConfiguration('yaw'),
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn x (same as husky.launch.py)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw (rad)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        map_server,
        lifecycle_manager,
        map_to_odom,
        rviz,
    ])
