"""Husky in Gazebo Classic, ready for SLAM and Nav2.

Compared with husky_gazebo/launch/gazebo.launch.py it:
  * adds a 2D lidar from urdf/husky_lidar.urdf.xacro (topic /scan)
  * publishes odom -> base_link from an EKF (config/ekf.yaml) that fuses the wheel speed with the IMU
    gyroscope, because the skid-steer wheels misjudge turns; the upstream EKF is left out because it
    listens on the wrong odometry topic and never publishes that transform
  * runs every node on simulation time
  * routes Nav2 (cmd_vel_nav) through twist_mux, below keyboard teleop (cmd_vel)
  * puts every topic, node and Gazebo plugin of the robot under its namespace (default husky1):
    /husky1/scan, /husky1/odometry/filtered... The Gazebo model has the same name.
  * gives each robot its own TF tree: /tf and /tf_static are remapped to /husky1/tf and /husky1/tf_static,
    so frame names stay plain (map, odom, base_link) and two robots do not overwrite each other

Usage (inside the container):
  ros2 launch warehouse_husky husky.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world
  ros2 launch warehouse_husky husky.launch.py world_path:=... gui:=false x:=2.0 y:=0.0 yaw:=0.0
A second robot joins the running Gazebo (gazebo:=false) under another namespace and spawn pose:
  ros2 launch warehouse_husky husky.launch.py gazebo:=false namespace:=husky2 x:=-1.5 y:=4.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, EnvironmentVariable, FindExecutable, LaunchConfiguration,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Each robot has its own TF tree, /<namespace>/tf, with plain frame names (the usual Nav2 multi-robot setup).
# Every node that publishes or reads TF gets these remappings.
TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    description_share = get_package_share_directory('husky_description')
    pkg_share = get_package_share_directory('warehouse_husky')
    control_config = os.path.join(pkg_share, 'config', 'control.yaml')
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    namespace = LaunchConfiguration('namespace')

    # Our own top-level xacro (urdf/husky.urdf.xacro): the upstream Husky body plus our lidar and contact sensors,
    # with every Gazebo plugin in the robot's namespace
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'urdf', 'husky.urdf.xacro'),
        ' is_sim:=true',
        ' gazebo_controllers:=', control_config,
        ' namespace:=', namespace,
    ]), value_type=str)

    # Fast DDS settings for every process started here (config/fastdds_profile.xml explains why): without
    # them, service answers were sometimes lost while several robots started, and start-up hung
    fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(pkg_share, 'config', 'fastdds_profile.xml'))

    # Lets Gazebo resolve model://husky_description/... and model://jackal_description/... meshes (a Jackal may join
    # this Gazebo later); without it, Gazebo tries to download them and stalls for about 100 s.
    gz_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', [
        EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ':/usr/share/gazebo-11/models/:',
        os.path.dirname(description_share), ':',
        os.path.dirname(get_package_share_directory('jackal_description')),
    ])

    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world_path')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gazebo')),
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen', condition=IfCondition(PythonExpression([
        "'", LaunchConfiguration('gazebo'), "' == 'true' and '", LaunchConfiguration('gui'), "' == 'true'"])))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        remappings=TF_REMAPS,
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        namespace=namespace,
        # The Gazebo model is named after the namespace (two models cannot share a name). The plugins' namespace
        # is already in the robot description (no -robot_namespace: see urdf/husky.urdf.xacro)
        arguments=['-entity', namespace, '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.15', '-Y', LaunchConfiguration('yaw')],
        output='screen',
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', ['/', namespace, '/controller_manager']],
        output='screen',
    )
    spawn_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', ['/', namespace, '/controller_manager']],
        output='screen',
    )

    # cmd_vel sources (keyboard on /cmd_vel, Nav2 on /cmd_vel_nav) -> wheel controller
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=namespace,
        parameters=[twist_mux_config, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'husky_velocity_controller/cmd_vel_unstamped')],
        output='screen',
    )

    # wheels + gyroscope -> odom -> base_link (on the robot's own tf) and odometry/filtered
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        parameters=[ekf_config],
        remappings=TF_REMAPS,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_path', default_value='',
                              description='World file, relative to Sim/worlds or absolute; empty world if unset'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('gazebo', default_value='true',
                              description='Start Gazebo; false to add this robot to a Gazebo that is already running'),
        DeclareLaunchArgument('namespace', default_value='husky1',
                              description='Robot name: namespace of its topics and nodes, and its Gazebo model name'),
        DeclareLaunchArgument('x', default_value='-1.5'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        fastdds_profile,
        gz_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        spawn_joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(target_action=spawn_joint_state_broadcaster,
                                           on_exit=[spawn_velocity_controller])),
        twist_mux,
        ekf,
    ])
