"""Pieces shared by localization.launch.py and navigation.launch.py to run Nav2 for one robot of several."""
import os

from nav2_common.launch import RewrittenYaml

# Each robot has its own TF tree, /<namespace>/tf, with plain frame names (the usual Nav2 multi-robot setup).
# Give these to the container (for the extra nodes created inside the servers, such as the costmaps) and to
# each composable node (composable nodes do not see the container's command-line remappings).
TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def nav2_params(pkg_share, namespace):
    """nav2_params.yaml for the robot `namespace` (a launch substitution):
      * everything moved under '<namespace>:', so the node names in the file match the namespaced nodes
        (/husky1/amcl, /husky1/global_costmap/global_costmap...);
      * the costmaps' lidar topic set to /<namespace>/scan_costmap: the real scan plus the outlines of the other
        robots (scripts/other_robots.py), which the lidar beam passes over. In Humble a costmap resolves a relative
        topic inside its own namespace (/husky1/global_costmap/scan), so it must be written out in full."""
    scan = ['/', namespace, '/scan_costmap']
    return RewrittenYaml(
        source_file=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        root_key=namespace,
        convert_types=True,
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': scan,
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': scan,
        })
