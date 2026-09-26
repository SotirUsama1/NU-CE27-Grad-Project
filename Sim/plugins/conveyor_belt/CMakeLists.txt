cmake_minimum_required(VERSION 3.10)
project(conveyor_belt_plugin)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(conveyor_belt SHARED conveyor_belt_plugin.cc)
target_link_libraries(conveyor_belt ${GAZEBO_LIBRARIES})
