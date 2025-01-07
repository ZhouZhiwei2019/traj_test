cmake_minimum_required(VERSION 3.0.2)
project(traj_test_node)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  geometry_msgs
  std_msgs
)

## Declare a catkin package
catkin_package()

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Build the node executable
add_executable(traj_test_node src/traj_test.cpp)

## Link libraries
target_link_libraries(traj_test_node
  ${catkin_LIBRARIES}
)

