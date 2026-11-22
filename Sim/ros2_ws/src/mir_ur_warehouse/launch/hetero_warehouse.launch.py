"""Heterogeneous warehouse: Our_Structured_Warehouse + two namespaced MiR/UR suction robots.

Both MiRs use /mir_ur and /mir_light so Husky (or Jackal) can keep /controller_manager.
Vacuum topics:
  ros2 service call /mir_ur/suction/set_vacuum std_srvs/srv/SetBool "{data: true}"
  ros2 service call /mir_light/suction/set_vacuum std_srvs/srv/SetBool "{data: true}"

Usage:
  ros2 launch mir_ur_warehouse hetero_warehouse.launch.py
  # then spawn Husky into the same Gazebo:
  ros2 launch warehouse_husky husky.launch.py start_gazebo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    use_sim_time = {'use_sim_time': True}
    world_default = (
        '/workspace/worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world')

    args = [
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('ur_type', default_value='ur10e'),
        DeclareLaunchArgument('x', default_value='-4.5'),
        DeclareLaunchArgument('y', default_value='23.2'),
        DeclareLaunchArgument('yaw', default_value='3.1416'),
        DeclareLaunchArgument('light_x', default_value='2.5'),
        DeclareLaunchArgument('light_y', default_value='24.6'),
        DeclareLaunchArgument('light_yaw', default_value='3.1416'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
    ]

    xacro_base = [
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('mir_ur_warehouse'), 'urdf', 'mir_ur.urdf.xacro']),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        ' joint_limit_params:=', PathJoinSubstitution([
            FindPackageShare('mir_ur_warehouse'), 'config',
            [LaunchConfiguration('ur_type'), '_joint_limits.yaml']]),
    ]

    # Twelve-cup. Empty MiR prefix (wheels stay left_wheel_joint); ROS namespace is mir_ur.
    # frame_prefix on RSP avoids TF clashes with Husky's base_link / odom.
    heavy_description = ParameterValue(Command(xacro_base + [
        ' gripper:=large ros_namespace:=mir_ur',
        ' robot_param_node:=/mir_ur/robot_state_publisher gt_namespace:=/mir_ur',
        ' controllers:=', PathJoinSubstitution([
            FindPackageShare('mir_ur_warehouse'), 'config', 'controllers_mir_ur.yaml']),
    ]), value_type=str)

    light_description = ParameterValue(Command(xacro_base + [
        ' prefix:=light_ gripper:=small ros_namespace:=mir_light',
        ' robot_param_node:=/mir_light/robot_state_publisher gt_namespace:=/mir_light',
        ' controllers:=', PathJoinSubstitution([
            FindPackageShare('mir_ur_warehouse'), 'config', 'controllers_light.yaml']),
    ]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_gazebo')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'verbose': 'true'}.items())

    heavy_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        name='robot_state_publisher', namespace='mir_ur',
        parameters=[{'robot_description': heavy_description, 'frame_prefix': 'mir_'},
                    use_sim_time])

    light_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        name='robot_state_publisher', namespace='mir_light',
        parameters=[{'robot_description': light_description, 'frame_prefix': 'light_'},
                    use_sim_time])

    spawn_heavy = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'mir_ur', '-topic', '/mir_ur/robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.05', '-Y', LaunchConfiguration('yaw')])

    spawn_light = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'mir_light', '-topic', '/mir_light/robot_description',
                   '-x', LaunchConfiguration('light_x'),
                   '-y', LaunchConfiguration('light_y'),
                   '-z', '0.05',
                   '-Y', LaunchConfiguration('light_yaw')])

    def spawner(name, manager):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', manager,
                               '--controller-manager-timeout', '90'])

    heavy_arm = spawner('joint_trajectory_controller', '/mir_ur/controller_manager')
    heavy_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_heavy,
        on_exit=[spawner('joint_broadcaster', '/mir_ur/controller_manager'),
                 spawner('diff_cont', '/mir_ur/controller_manager'),
                 heavy_arm]))
    spawn_light_after_arm = RegisterEventHandler(OnProcessExit(
        target_action=heavy_arm, on_exit=[spawn_light]))
    light_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_light,
        on_exit=[spawner('joint_broadcaster', '/mir_light/controller_manager'),
                 spawner('diff_cont', '/mir_light/controller_manager'),
                 spawner('joint_trajectory_controller', '/mir_light/controller_manager')]))

    laser_merger = Node(
        package='ira_laser_tools', executable='laserscan_multi_merger',
        name='mir_laser_scan_merger', output='screen', namespace='mir_ur',
        parameters=[{'laserscan_topics': 'b_scan f_scan',
                     'destination_frame': 'mir_virtual_laser_link',
                     'scan_destination_topic': 'scan',
                     'cloud_destination_topic': 'scan_cloud',
                     'min_height': -0.25,
                     'max_completion_time': 0.05,
                     'max_merge_time_diff': 0.005,
                     'best_effort': False}, use_sim_time])

    return LaunchDescription(args + [
        gazebo, heavy_state_publisher, light_state_publisher,
        spawn_heavy, heavy_controllers, spawn_light_after_arm, light_controllers,
        laser_merger])
