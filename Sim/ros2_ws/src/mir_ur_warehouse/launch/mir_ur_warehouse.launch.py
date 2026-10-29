import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    use_sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'world',
            default_value='/workspace/worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world'),
        DeclareLaunchArgument('ur_type', default_value='ur10e', description='ur5e, ur10e or ur16e'),
        # Open dock, west side. The six-cup robot spawns on the east side so the two
        # bases and their arms are not in the same spot.
        DeclareLaunchArgument('x', default_value='-4.5'),
        DeclareLaunchArgument('y', default_value='23.2'),
        DeclareLaunchArgument('yaw', default_value='3.1416'),
        DeclareLaunchArgument('gui', default_value='true'),
    ]

    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('mir_ur_warehouse'), 'urdf', 'mir_ur.urdf.xacro']),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        # mir_description names the arm joints ur_*; only its ur5e limits file was renamed to match.
        ' joint_limit_params:=', PathJoinSubstitution([
            FindPackageShare('mir_ur_warehouse'), 'config',
            [LaunchConfiguration('ur_type'), '_joint_limits.yaml']]),
    ]
    # Twelve-cup gripper. Topics stay on the root so the original conveyor demo is unchanged.
    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)
    # Six-cup gripper. Its own namespace, so the two arms do not share a controller manager.
    light_description = ParameterValue(Command(xacro_cmd + [
        ' prefix:=light_ gripper:=small ros_namespace:=mir_light',
        ' robot_param_node:=/mir_light/robot_state_publisher gt_namespace:=/mir_light',
        ' controllers:=', PathJoinSubstitution([
            FindPackageShare('mir_ur_warehouse'), 'config', 'controllers_light.yaml']),
    ]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'verbose': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        parameters=[{'robot_description': robot_description}, use_sim_time])

    light_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        name='robot_state_publisher', namespace='mir_light',
        parameters=[{'robot_description': light_description, 'frame_prefix': 'light_'}, use_sim_time])

    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'mir_ur', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.05', '-Y', LaunchConfiguration('yaw')])

    # East side of the same dock, about 7 m from the twelve-cup robot.
    spawn_light = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'mir_light', '-topic', '/mir_light/robot_description',
                   '-x', '2.5', '-y', '24.6', '-z', '0.05', '-Y', '3.1416'])

    def spawner(name, manager):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', manager,
                               '--controller-manager-timeout', '90'])

    root_arm = spawner('joint_trajectory_controller', '/controller_manager')
    # The second model is inserted only after the first arm controller is active. Spawning
    # both at once hangs inside Gazebo, and loading the second controller manager while the
    # first is still starting drops the first arm's parameters.
    controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[spawner('joint_broadcaster', '/controller_manager'),
                 spawner('diff_cont', '/controller_manager'),
                 root_arm]))
    spawn_light_after_arm = RegisterEventHandler(OnProcessExit(
        target_action=root_arm, on_exit=[spawn_light]))

    light_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_light,
        on_exit=[spawner('joint_broadcaster', '/mir_light/controller_manager'),
                 spawner('diff_cont', '/mir_light/controller_manager'),
                 spawner('joint_trajectory_controller', '/mir_light/controller_manager')]))

    laser_merger = Node(
        package='ira_laser_tools', executable='laserscan_multi_merger',
        name='mir_laser_scan_merger', output='screen',
        parameters=[{'laserscan_topics': 'b_scan f_scan',
                     'destination_frame': 'virtual_laser_link',
                     'scan_destination_topic': 'scan',
                     'cloud_destination_topic': 'scan_cloud',
                     'min_height': -0.25,
                     'max_completion_time': 0.05,
                     'max_merge_time_diff': 0.005,
                     'best_effort': False}, use_sim_time])

    return LaunchDescription(args + [
        gazebo, robot_state_publisher, light_state_publisher,
        spawn_robot, controllers, spawn_light_after_arm, light_controllers, laser_merger])
