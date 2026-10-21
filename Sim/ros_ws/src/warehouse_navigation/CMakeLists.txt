cmake_minimum_required(VERSION 3.8)
project(warehouse_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

# ros2 run warehouse_navigation go_far.py
install(PROGRAMS scripts/go_far.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
