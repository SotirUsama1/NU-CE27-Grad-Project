"""Spawn a Jackal into an already-running Gazebo (hetero warehouse).

Uses the Clearpath jackal packages from sotirusama/gzclassic:devvv
(/opt/ros2_robots). Prefer this over jackal_gazebo/gazebo.launch.py when the
MiR hetero world is already up.

Usage (inside the hetero container, Gazebo already running):
  ros2 launch warehouse_husky jackal_spawn.launch.py
  ros2 launch warehouse_husky jackal_spawn.launch.py x:=4.0 y:=2.0 yaw:=0.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Prefer the stock jackal description shipped in the robots image.
    jackal_share = get_package_share_directory('jackal_description')

    # Clearpath jackal.urdf.xacro layout varies slightly by release; this matches
    # the humble jackal_description used in sotirusama/gzclassic:devvv.
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(jackal_share, 'urdf', 'jackal.urdf.xacro'),
    ]), value_type=str)

    gz_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.environ.get('GAZEBO_MODEL_PATH', '') + ':/usr/share/gazebo-11/models/:'
        + os.path.dirname(jackal_share))

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='4.0'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_model_path,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='jackal_state_publisher',
            namespace='jackal',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True,
                         'frame_prefix': 'jackal_'}],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_jackal',
            arguments=['-entity', 'jackal',
                       '-topic', '/jackal/robot_description',
                       '-x', LaunchConfiguration('x'),
                       '-y', LaunchConfiguration('y'),
                       '-z', '0.2',
                       '-Y', LaunchConfiguration('yaw')],
            output='screen',
        ),
    ])
