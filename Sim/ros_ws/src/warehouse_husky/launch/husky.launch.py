"""Husky + UR10e + large suction in Gazebo Classic.

Compared with husky_gazebo/launch/gazebo.launch.py it:
  * mounts UR10e + 12-cup suction via mir_ur_warehouse/urdf/husky_ur_suction.urdf.xacro
  * adds a 2D lidar (included in that extras file)
  * publishes odom -> base_link from an EKF (config/ekf.yaml)
  * runs every node on simulation time
  * routes Nav2 (/cmd_vel_nav) through twist_mux, below keyboard teleop (/cmd_vel)

Usage (inside the container):
  ros2 launch warehouse_husky husky.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world
  # Join an already-running hetero Gazebo (MiRs already spawned):
  ros2 launch warehouse_husky husky.launch.py start_gazebo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, EnvironmentVariable, FindExecutable,
                                  LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('husky_description')
    pkg_share = get_package_share_directory('warehouse_husky')
    mir_share = get_package_share_directory('mir_ur_warehouse')
    control_config = os.path.join(pkg_share, 'config', 'control.yaml')
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    arm_extras = os.path.join(mir_share, 'urdf', 'husky_ur_suction.urdf.xacro')

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description_share, 'urdf', 'husky.urdf.xacro'),
        ' is_sim:=true',
        ' urdf_extras:=', arm_extras,
        ' gazebo_controllers:=', control_config,
    ]), value_type=str)

    gz_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', [
        EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ':/usr/share/gazebo-11/models/:',
        os.path.dirname(description_share),
    ])

    start_gz = LaunchConfiguration('start_gazebo')
    gui = LaunchConfiguration('gui')

    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world_path')],
        output='screen',
        condition=IfCondition(start_gz),
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'], output='screen',
        condition=IfCondition(PythonExpression([
            "'", start_gz, "' == 'true' and '", gui, "' == 'true'"
        ])),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=['-entity', 'husky', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.15', '-Y', LaunchConfiguration('yaw')],
        output='screen',
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
    spawn_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen',
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/husky_velocity_controller/cmd_vel_unstamped')],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_path', default_value='',
                              description='World file, relative to Sim/worlds or absolute; empty world if unset'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument(
            'start_gazebo', default_value='true',
            description='Set false to spawn into an already-running Gazebo (hetero world)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        spawn_joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(target_action=spawn_joint_state_broadcaster,
                                           on_exit=[spawn_velocity_controller])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_velocity_controller,
                                           on_exit=[spawn_arm_controller])),
        twist_mux,
        ekf,
    ])
