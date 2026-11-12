"""The whole simulated fleet plus the ROSBridge the CyberScape orchestrator talks to.

One command brings up, in order:
  * Gazebo Classic (gzserver, gzclient only with gui:=true) with the warehouse world,
  * one Husky per entry of the robots argument (husky1, husky2), each in its own
    namespace with its own TF tree (warehouse_husky husky.launch.py; only the first
    robot starts Gazebo, the others join it),
  * an isolated Nav2 stack per robot (warehouse_navigation navigation.launch.py:
    its own map_server, AMCL, planner, controller, behaviors, bt_navigator and
    lifecycle managers under /<robot>/...),
  * the Phase 5 heartbeat publisher per robot (/{robot}/heartbeat at 1 Hz: sim-time
    Header + nearest topological node),
  * the rosbridge_server WebSocket bridge on port 9090 (rosbridge_port) - the
    orchestrator's rosbridge_client.py connects to ws://localhost:9090 to subscribe
    to /{robot}/amcl_pose and /{robot}/heartbeat and dispatch NavigateToPose goals
    on /{robot}/navigate_to_pose.

RobotNavigator instances (warehouse_nav/navigator.py) attach to any robot's
namespace without further setup: their action clients resolve to the robot's own
Nav2 servers, e.g. `ros2 run warehouse_navigation go_to.py --robot husky2 dock`
drives husky2 with the full recovery logic (unstick, escape_to_free_space, stall
detection, collision counting) on top of that robot's Nav2 stack.

Usage (inside the container):
  ros2 launch warehouse_navigation multi_robot.launch.py
  ros2 launch warehouse_navigation multi_robot.launch.py gui:=true rviz:=true
  ros2 launch warehouse_navigation multi_robot.launch.py robots:=husky1,husky2
  ros2 launch warehouse_navigation multi_robot.launch.py rosbridge_port:=9091
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HUSKY_SHARE = get_package_share_directory('warehouse_husky')
NAV_SHARE = get_package_share_directory('warehouse_navigation')

# Spawn poses per robot: husky1 is husky.launch.py / localization.launch.py's default,
# husky2 the pose navigation.launch.py's docs use for the second robot.
ROBOT_SPAWNS = {
    'husky1': {'x': -1.5, 'y': 2.0, 'yaw': 0.0},
    'husky2': {'x': -1.5, 'y': 4.0, 'yaw': 0.0},
}
FALLBACK_SPREAD_M = 2.0   # extra robots line up along +y behind the defaults


def _robot_spawn(name: str, index: int) -> dict:
    return ROBOT_SPAWNS.get(name) or {
        'x': -1.5, 'y': 2.0 + FALLBACK_SPREAD_M * index, 'yaw': 0.0,
    }


def _fleet(context: LaunchContext) -> list:
    """Build the per-robot includes and the heartbeat nodes from the arguments."""
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    gazebo = LaunchConfiguration('gazebo').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    topo_graph_path = LaunchConfiguration('topo_graph_path').perform(context)

    actions: list = []
    for index, name in enumerate(robots):
        spawn = _robot_spawn(name, index)
        # Robot + Gazebo (only the first robot starts Gazebo; the rest join it)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HUSKY_SHARE, 'launch', 'husky.launch.py')),
            launch_arguments={
                'namespace': name,
                'world_path': LaunchConfiguration('world_path'),
                'gazebo': 'true' if (gazebo and index == 0) else 'false',
                'gui': LaunchConfiguration('gui'),
                'arm': 'none',
                'x': str(spawn['x']), 'y': str(spawn['y']), 'yaw': str(spawn['yaw']),
            }.items(),
        ))
        # Isolated Nav2 stack for this robot (its own map, AMCL, servers, bt_navigator)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(NAV_SHARE, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'namespace': name,
                'x': str(spawn['x']), 'y': str(spawn['y']), 'yaw': str(spawn['yaw']),
                'rviz': 'true' if (rviz and index == 0) else 'false',
            }.items(),
        ))
        # Phase 5 heartbeat: /<name>/heartbeat at 1 Hz (sim-time Header + nearest node)
        actions.append(Node(
            package='warehouse_navigation',
            executable='heartbeat_publisher.py',
            name='heartbeat_publisher',
            namespace=name,
            parameters=[{
                'use_sim_time': True,
                'rate': 1.0,
                'topo_graph_path': topo_graph_path,
            }],
            output='screen',
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        # The orchestrator's fixed entry point (plan 5.4: ws://localhost:9090).
        # Goals must be sent in new threads: with the default (False) each
        # NavigateToPose blocks the protocol thread, so robots could not be
        # dispatched concurrently.
        parameters=[{'port': LaunchConfiguration('rosbridge_port'),
                     'send_action_goals_in_new_thread': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='husky1,husky2',
                              description='Comma-separated robot namespaces to spawn'),
        DeclareLaunchArgument('world_path',
                              default_value='Our_Structured_Warehouse/Our_Structured_Warehouse.world',
                              description='World file, relative to Sim/worlds or absolute (husky.launch.py)'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Open the Gazebo window (the fleet runs headless by default)'),
        DeclareLaunchArgument('gazebo', default_value='true',
                              description='Start Gazebo from the first robot (false joins a running Gazebo)'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz for the first robot'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='rosbridge_server WebSocket port'),
        DeclareLaunchArgument('topo_graph_path', default_value='/main/topo_graph.json',
                              description='Absolute path of topo_graph.json for the heartbeat nodes '
                                          '(default: /main/topo_graph.json)'),
        rosbridge,
        OpaqueFunction(function=_fleet),
    ])
