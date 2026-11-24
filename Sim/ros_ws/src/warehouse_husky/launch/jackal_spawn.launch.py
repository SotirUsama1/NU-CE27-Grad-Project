"""Spawn Jackal + UR5e + small suction into an already-running Gazebo.

Uses mir_ur_warehouse/urdf/jackal_ur_suction.urdf.xacro as JACKAL_URDF_EXTRAS,
namespaced /jackal/controller_manager, bright chassis materials, and file://
mesh URIs so gzclient always finds Clearpath meshes.

Usage (inside hetero container, Gazebo already running):
  ros2 launch warehouse_husky jackal_spawn.launch.py
  ros2 launch warehouse_husky jackal_spawn.launch.py x:=0.0 y:=8.0 yaw:=0.0
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_robot_description(context, *args, **kwargs):
    jackal_share = get_package_share_directory('jackal_description')
    mir_share = get_package_share_directory('mir_ur_warehouse')
    extras = os.path.join(mir_share, 'urdf', 'jackal_ur_suction.urdf.xacro')
    controllers = os.path.join(mir_share, 'config', 'controllers_jackal.yaml')
    xacro_file = os.path.join(jackal_share, 'urdf', 'jackal.urdf.xacro')

    env = os.environ.copy()
    env['JACKAL_URDF_EXTRAS'] = extras

    urdf = subprocess.check_output([
        'xacro', xacro_file,
        'is_sim:=true',
        'prefix:=',
        f'gazebo_controllers:={controllers}',
    ], env=env, text=True)

    # Use mir_ur_warehouse's namespaced gazebo_ros2_control fork — the stock
    # plugin ignores <ros><namespace> here and loads params into the process-wide
    # context, which wipes Husky's root /controller_manager when Jackal spawns.
    # Also uniquify imu/gps plugin names (Husky already owns those ROS nodes).
    urdf = urdf.replace(
        'filename="libgazebo_ros2_control.so" name="gazebo_ros2_control"',
        'filename="libmir_ur_ros2_control.so" name="jackal_gazebo_ros2_control"',
        1)
    urdf = urdf.replace('name="imu_plugin"', 'name="jackal_imu_plugin"')
    urdf = urdf.replace('name="gps_plugin"', 'name="jackal_gps_plugin"')
    urdf = urdf.replace('Gazebo/DarkGrey', 'Gazebo/Blue')

    needle = 'filename="libmir_ur_ros2_control.so" name="jackal_gazebo_ros2_control">'
    inject = (needle + '\n'
              '      <ros>\n'
              '        <namespace>/jackal</namespace>\n'
              '      </ros>\n'
              '      <robot_param_node>/jackal/jackal_state_publisher</robot_param_node>')
    if needle in urdf and 'robot_param_node>/jackal/jackal_state_publisher' not in urdf:
        urdf = urdf.replace(needle, inject, 1)
    urdf = urdf.replace(
        'package://jackal_description/',
        f'file://{jackal_share}/')

    try:
        ur_share = get_package_share_directory('ur_description')
        urdf = urdf.replace('package://ur_description/', f'file://{ur_share}/')
    except Exception:
        pass

    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='jackal_state_publisher',
        namespace='jackal',
        parameters=[{'robot_description': urdf, 'use_sim_time': True,
                     'frame_prefix': 'jackal_'}],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_jackal',
        arguments=['-entity', 'jackal',
                   '-topic', '/jackal/robot_description',
                   '-x', x, '-y', y, '-z', '0.2', '-Y', yaw],
        output='screen',
    )

    cm = '/jackal/controller_manager'
    js_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jackal_js',
        arguments=['joint_state_broadcaster', '-c', cm],
        output='screen',
    )
    vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jackal_vel',
        arguments=['jackal_velocity_controller', '-c', cm],
        output='screen',
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jackal_arm',
        arguments=['joint_trajectory_controller', '-c', cm],
        output='screen',
    )

    # gazebo_ros2_control needs a few seconds after spawn before CM is up.
    return [
        robot_state_publisher,
        spawn,
        TimerAction(period=4.0, actions=[js_spawner]),
        TimerAction(period=6.0, actions=[vel_spawner]),
        TimerAction(period=8.0, actions=[arm_spawner]),
    ]


def generate_launch_description():
    jackal_share = get_package_share_directory('jackal_description')
    gz_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.environ.get('GAZEBO_MODEL_PATH', '') + ':/usr/share/gazebo-11/models/:'
        + os.path.dirname(jackal_share))

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='8.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_model_path,
        OpaqueFunction(function=_build_robot_description),
    ])
