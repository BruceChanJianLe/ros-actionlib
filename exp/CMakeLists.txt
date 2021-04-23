cmake_minimum_required(VERSION 3.0.2)
project(exp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  exp_msgs
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES exp
#  CATKIN_DEPENDS actionlib exp_msgs roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# exp server
add_executable(
  exp_server_node
  src/exp_server_node.cpp
  src/exp_server.cpp
)

target_link_libraries(
  exp_server_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  exp_server_node
  ${catkin_EXPORTED_TARGETS}
)

# exp server
add_executable(
  exp_client_node
  src/exp_client_node.cpp
  src/exp_client.cpp
)

target_link_libraries(
  exp_client_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  exp_client_node
  ${catkin_EXPORTED_TARGETS}
)