cmake_minimum_required(VERSION 2.8.3)
project(shared_control_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_filters
)

find_package(Boost REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(shared_control_node src/shared_control_node.cpp)

target_link_libraries(shared_control_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
