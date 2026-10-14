"""Localize the Husky on the warehouse map with AMCL.

Loads maps/warehouse.yaml with map_server and runs AMCL, which matches the lidar scan against
the map and publishes map -> odom, correcting the wheel odometry drift. Replaces view_map.launch.py:
do not run both, they would both publish map -> odom.

Start the Husky first (warehouse_husky husky.launch.py), then:
  ros2 launch warehouse_navigation localization.launch.py
  ros2 launch warehouse_navigation localization.launch.py x:=2.0 y:=1.0 yaw:=1.57   (same as the spawn pose)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_navigation')
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[params, {'yaml_filename': os.path.join(pkg_share, 'maps', 'warehouse.yaml')}],
        output='screen',
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[params, {
            'initial_pose.x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'initial_pose.y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'initial_pose.yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
        }],
        output='screen',
    )
    # map_server and amcl are lifecycle nodes: they only start working once configured and activated
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'node_names': ['map_server', 'amcl'], 'autostart': True, 'use_sim_time': True}],
        output='screen',
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
        map_server,
        amcl,
        # started a few seconds later: if it asks the servers to switch on before ROS has finished
        # discovering them, their reply can be lost and it waits forever
        TimerAction(period=3.0, actions=[lifecycle_manager]),
        rviz,
    ])
