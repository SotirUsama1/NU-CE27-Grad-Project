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
        # Open dock floor in zone D, where the pick-to-conveyor demo starts.
        DeclareLaunchArgument('x', default_value='0.5'),
        DeclareLaunchArgument('y', default_value='23.0'),
        DeclareLaunchArgument('yaw', default_value='3.1416'),
        DeclareLaunchArgument('gui', default_value='true'),
    ]

    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('mir_ur_warehouse'), 'urdf', 'mir_ur.urdf.xacro']),
        ' ur_type:=', LaunchConfiguration('ur_type'),
        # mir_description names the arm joints ur_*; only its ur5e limits file was renamed to match.
        ' joint_limit_params:=', PathJoinSubstitution([
            FindPackageShare('mir_ur_warehouse'), 'config',
            [LaunchConfiguration('ur_type'), '_joint_limits.yaml']]),
    ]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'verbose': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        parameters=[{'robot_description': robot_description}, use_sim_time])

    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'mir_ur', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.05', '-Y', LaunchConfiguration('yaw')])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60'])

    controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[spawner('joint_broadcaster'), spawner('diff_cont'),
                 spawner('joint_trajectory_controller')]))

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

    return LaunchDescription(args + [gazebo, robot_state_publisher, spawn_robot, controllers, laser_merger])
