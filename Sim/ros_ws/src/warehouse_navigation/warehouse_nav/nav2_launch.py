"""Pieces shared by localization.launch.py and navigation.launch.py to run Nav2 for one robot of several."""
import os

from launch.substitutions import PythonExpression
from nav2_common.launch import RewrittenYaml

# Each robot has its own TF tree, /<namespace>/tf, with plain frame names (the usual Nav2 multi-robot setup).
# Give these to the container (for the extra nodes created inside the servers, such as the costmaps) and to
# each composable node (composable nodes do not see the container's command-line remappings).
TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

# Outline of each robot type around its base_link, for both costmaps (nav2_params.yaml has the Husky's).
#   husky   body 0.99 m long, wheels 0.69 m wide
#   jackal  0.52 x 0.44 m over the wheels and fenders (jackal_description)
FOOTPRINTS = {
    'husky': '[[0.50, 0.345], [0.50, -0.345], [-0.50, -0.345], [-0.50, 0.345]]',
    'jackal': '[[0.26, 0.22], [0.26, -0.22], [-0.26, -0.22], [-0.26, 0.22]]',
}


def nav2_params(pkg_share, namespace, robot_type='husky'):
    """nav2_params.yaml for the robot `namespace` (a launch substitution):
      * everything moved under '<namespace>:', so the node names in the file match the namespaced nodes
        (/husky1/amcl, /husky1/global_costmap/global_costmap...);
      * the costmaps' lidar topic set to /<namespace>/scan_costmap: the real scan plus the outlines of the other
        robots (scripts/other_robots.py), which the lidar beam passes over. In Humble a costmap resolves a relative
        topic inside its own namespace (/husky1/global_costmap/scan), so it must be written out in full;
      * both costmaps' footprint set to the outline of `robot_type` (a launch substitution, see FOOTPRINTS);
      * the collision monitor's slowdown zone of `robot_type` switched on, the other one off."""
    scan = ['/', namespace, '/scan_costmap']
    footprint = PythonExpression([repr(FOOTPRINTS), "['", robot_type, "']"])

    def zone_on(zone_type):   # 'true' if the robot is of this type
        return PythonExpression(["'true' if '", robot_type, f"' == '{zone_type}' else 'false'"])

    return RewrittenYaml(
        source_file=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        root_key=namespace,
        convert_types=True,
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': scan,
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': scan,
            'global_costmap.global_costmap.ros__parameters.footprint': footprint,
            'local_costmap.local_costmap.ros__parameters.footprint': footprint,
            'collision_monitor.ros__parameters.slowdown_husky.enabled': zone_on('husky'),
            'collision_monitor.ros__parameters.slowdown_jackal.enabled': zone_on('jackal'),
        })
