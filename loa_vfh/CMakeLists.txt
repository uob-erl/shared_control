cmake_minimum_required(VERSION 2.8.3)
project(loa_vfh)

## Compile as C++11, supported in ROS Kinetic and newer #git test
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vfh_node src/vfh_node.cpp)

target_link_libraries(vfh_node
  ${catkin_LIBRARIES}
)
