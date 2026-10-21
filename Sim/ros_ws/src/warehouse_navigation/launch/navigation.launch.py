"""Full Nav2 navigation for the Husky in the warehouse: one command, then click a goal in RViz.

Starts localization.launch.py (map_server, AMCL, RViz) and the navigation servers:
  planner_server     global costmap + planner (NavFn): plans the route
  controller_server  local costmap + controller (MPPI): drives along the route on /cmd_vel_nav
  behavior_server    recovery moves (spin, back up, wait) for when the robot is stuck
  bt_navigator       takes a goal and runs the behavior tree that uses the three servers above

Start the Husky first (warehouse_husky husky.launch.py), then:
  ros2 launch warehouse_navigation navigation.launch.py
  ros2 launch warehouse_navigation navigation.launch.py x:=2.0 y:=1.0 yaw:=1.57   (same as the spawn pose)

Give a goal with RViz's "2D Goal Pose" button (click the spot, drag for the direction), or:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
    "{pose: {header: {frame_id: map}, pose: {position: {x: 28.0, y: 25.6}, orientation: {w: 1.0}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_navigation')
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'localization.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in ('x', 'y', 'yaw', 'rviz')}.items(),
    )

    # The servers and their lifecycle manager run in one process, for the same reason as in
    # localization.launch.py. All of them read their settings from nav2_params.yaml.
    container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        # the settings file is also given to the whole process: the costmaps and the navigator's
        # helper nodes are extra nodes created inside the servers, and only process-wide settings reach them
        parameters=[params],
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params, {'use_sim_time': True}],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params, {'use_sim_time': True}],
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[params, {'use_sim_time': True}],
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[params, {'use_sim_time': True}],
            ),
            # Switches the servers on in this order; the navigator last, once everything it calls is ready
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator'],
                             'autostart': True, 'use_sim_time': True}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn x (same as husky.launch.py)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw (rad)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        localization,
        container,
    ])
