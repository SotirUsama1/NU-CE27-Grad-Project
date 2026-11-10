"""Localize the Husky on the warehouse map with AMCL.

Loads maps/warehouse.yaml with map_server (/map, racks solid: what the planner uses and RViz shows)
and maps/warehouse_lidar.yaml with map_server_lidar (/map_lidar, racks post by post: what the lidar
sees). AMCL matches the lidar scan against /map_lidar and publishes map -> odom, correcting the
wheel odometry drift. Replaces view_map.launch.py:
do not run both, they would both publish map -> odom.

Everything runs under the robot's namespace (argument namespace, default husky1: /husky1/amcl,
/husky1/map...) and uses the robot's own TF tree (/husky1/tf), like the robot itself
(warehouse_husky husky.launch.py). RViz shows this robot, plus the pose and route of one other robot
(argument other_namespace): both robots use the same map frame, so they appear on the same map.

Start the Husky first (warehouse_husky husky.launch.py), then:
  ros2 launch warehouse_navigation localization.launch.py
  ros2 launch warehouse_navigation localization.launch.py x:=2.0 y:=1.0 yaw:=1.57   (same as the spawn pose)
  ros2 launch warehouse_navigation localization.launch.py namespace:=husky2 other_namespace:=husky1 x:=... y:=...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterValue
from nav2_common.launch import ReplaceString
from warehouse_nav.nav2_launch import TF_REMAPS, nav2_params


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_navigation')
    namespace = LaunchConfiguration('namespace')
    params = nav2_params(pkg_share, namespace, LaunchConfiguration('robot_type'))

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
        remappings=TF_REMAPS,
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[params, {'yaml_filename': os.path.join(pkg_share, 'maps', 'warehouse.yaml')}],
                remappings=TF_REMAPS,
            ),
            # The same warehouse as the lidar sees it (rack posts one by one), for AMCL only
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server_lidar',
                parameters=[params, {'yaml_filename': os.path.join(pkg_share, 'maps', 'warehouse_lidar.yaml')}],
                remappings=TF_REMAPS,
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
                remappings=TF_REMAPS,
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

    # navigation.rviz is a template: <robot_namespace> is this robot, <other_namespace> and
    # <third_namespace> the robots whose pose and route are also shown
    rviz_config = ReplaceString(
        source_file=os.path.join(pkg_share, 'rviz', 'navigation.rviz'),
        replacements={'<robot_namespace>': ['/', namespace],
                      '<other_namespace>': ['/', LaunchConfiguration('other_namespace')],
                      '<third_namespace>': ['/', LaunchConfiguration('third_namespace')]})
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        remappings=TF_REMAPS,   # inside the group below: this robot's TF tree
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # Fast DDS settings for every process started here (warehouse_husky config/fastdds_profile.xml explains
    # why): without them, the answer to "load this node" was sometimes lost and start-up hung
    fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(get_package_share_directory('warehouse_husky'), 'config', 'fastdds_profile.xml'))

    return LaunchDescription([
        fastdds_profile,
        DeclareLaunchArgument('x', default_value='-1.5', description='Robot spawn x (same as husky.launch.py)'),
        DeclareLaunchArgument('y', default_value='2.0', description='Robot spawn y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw (rad)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('robot_type', default_value='husky',
                              description='husky or jackal: sets the robot outline in the costmaps'),
        DeclareLaunchArgument('namespace', default_value='husky1', description='Robot to localize (its namespace)'),
        DeclareLaunchArgument('other_namespace', default_value='husky2',
                              description='Other robot whose pose and route RViz also shows'),
        DeclareLaunchArgument('third_namespace', default_value='jackal1',
                              description='Third robot whose pose and route RViz shows (purple)'),
        GroupAction([PushRosNamespace(namespace), container, rviz]),   # all of it under the robot's namespace
    ])
