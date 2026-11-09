"""Helpers shared by the navigation scripts of the warehouse project.

  grid          the map as a numpy grid: obstacle growing and flood fill
  places        the named places of the warehouse (config/waypoints.yaml)
  navigator     RobotNavigator: go_to(place) for one robot, with collision counting and getting unstuck
  nav2_launch   for the launch files: Nav2 settings and TF remappings for one robot of several
"""
