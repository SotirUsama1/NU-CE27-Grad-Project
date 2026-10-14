"""Show Nav2's global and local costmaps for the Husky, on top of localization.

Starts localization.launch.py (map_server, AMCL, RViz) and the two costmaps. Nav2 runs each costmap
inside a server: the global one in planner_server, the local one in controller_server. Here those servers
only host the costmaps; they plan and drive nothing until they get a goal, which comes in a later stage.

Start the Husky first (warehouse_husky husky.launch.py), then:
  ros2 launch warehouse_navigation costmaps.launch.py
  ros2 launch warehouse_navigation costmaps.launch.py x:=2.0 y:=1.0 yaw:=1.57   (same as the spawn pose)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_navigation')
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'localization.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in ('x', 'y', 'yaw', 'rviz')}.items(),
    )

    # planner_server creates /global_costmap/global_costmap, controller_server /local_costmap/local_costmap;
    # both read their settings from nav2_params.yaml
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params, {'use_sim_time': True}],
        output='screen',
    )
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        # placeholder path follower until the controller stage; it only runs when given a path
        parameters=[params, {'use_sim_time': True, 'controller_plugins': ['FollowPath'],
                             'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        output='screen',
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmaps',
        parameters=[{'node_names': ['planner_server', 'controller_server'], 'autostart': True, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn x (same as husky.launch.py)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw (rad)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        localization,
        planner,
        controller,
        # started a few seconds later: if it asks the servers to switch on before ROS has finished
        # discovering them, their reply can be lost and it waits forever
        TimerAction(period=3.0, actions=[lifecycle_manager]),
    ])
