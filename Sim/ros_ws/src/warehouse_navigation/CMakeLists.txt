cmake_minimum_required(VERSION 3.8)
project(warehouse_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# import warehouse_nav (places, navigator, grid helpers) from any script
ament_python_install_package(warehouse_nav)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

# ros2 run warehouse_navigation go_far.py / go_to.py
install(PROGRAMS scripts/go_far.py scripts/go_to.py scripts/generate_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
