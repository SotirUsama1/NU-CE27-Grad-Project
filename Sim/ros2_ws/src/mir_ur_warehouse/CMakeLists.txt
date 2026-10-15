cmake_minimum_required(VERSION 3.8)
project(mir_ur_warehouse)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(vacuum_gripper SHARED src/vacuum_gripper.cpp)
ament_target_dependencies(vacuum_gripper gazebo_dev gazebo_ros rclcpp std_msgs std_srvs)
install(TARGETS vacuum_gripper LIBRARY DESTINATION lib)

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})

ament_package()
