cmake_minimum_required(VERSION 3.8)
project(warehouse_husky)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/arm_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
