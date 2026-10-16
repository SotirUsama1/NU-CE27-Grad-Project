"""Localize the Husky on the warehouse map with AMCL.

Loads maps/warehouse.yaml with map_server (/map, racks solid: what the planner uses and RViz shows)
and maps/warehouse_lidar.yaml with map_server_lidar (/map_lidar, racks post by post: what the lidar
sees). AMCL matches the lidar scan against /map_lidar and publishes map -> odom, correcting the
wheel odometry drift. Replaces view_map.launch.py:
do not run both, they would both publish map -> odom.

Start the Husky first (warehouse_husky husky.launch.py), then:
  ros2 launch warehouse_navigation localization.launch.py
  ros2 launch warehouse_navigation localization.launch.py x:=2.0 y:=1.0 yaw:=1.57   (same as the spawn pose)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_navigation')
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # map_server, amcl and their lifecycle manager run in one process (a component container).
    # As separate processes, the manager sometimes asked a server to start before ROS had finished
    # connecting them; the server's reply was lost and the start-up hung. Inside one process they
    # are connected at once.
    container = ComposableNodeContainer(
        name='localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',   # each node gets its own thread
        # the settings file is also given to the whole process: the costmaps are extra nodes created
        # inside the planner and controller, and only process-wide settings reach them
        parameters=[params],
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[params, {'yaml_filename': os.path.join(pkg_share, 'maps', 'warehouse.yaml')}],
            ),
            # The same warehouse as the lidar sees it (rack posts one by one), for AMCL only
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server_lidar',
                parameters=[params, {'yaml_filename': os.path.join(pkg_share, 'maps', 'warehouse_lidar.yaml')}],
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[params, {
                    'initial_pose.x': ParameterValue(LaunchConfiguration('x'), value_type=float),
                    'initial_pose.y': ParameterValue(LaunchConfiguration('y'), value_type=float),
                    'initial_pose.yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
                }],
            ),
            # map_server and amcl are lifecycle nodes: they only start working once configured and activated
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'node_names': ['map_server', 'map_server_lidar', 'amcl'], 'autostart': True, 'use_sim_time': True}],
            ),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn x (same as husky.launch.py)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw (rad)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        container,
        rviz,
    ])
