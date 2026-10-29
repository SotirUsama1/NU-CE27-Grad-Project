cmake_minimum_required(VERSION 3.8)
project(mir_ur_warehouse)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(vacuum_gripper SHARED src/vacuum_gripper.cpp)
ament_target_dependencies(vacuum_gripper gazebo_dev gazebo_ros rclcpp std_msgs std_srvs)

add_library(mir_ur_ros2_control SHARED src/gazebo_ros2_control_plugin.cpp)
target_include_directories(mir_ur_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(mir_ur_ros2_control
  gazebo_dev gazebo_ros gazebo_ros2_control control_toolbox controller_manager hardware_interface
  pluginlib rclcpp rclcpp_lifecycle)

install(TARGETS vacuum_gripper mir_ur_ros2_control LIBRARY DESTINATION lib)

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})

ament_package()
