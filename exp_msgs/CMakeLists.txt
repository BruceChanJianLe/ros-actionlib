cmake_minimum_required(VERSION 3.0.2)
project(exp_msgs)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
)

add_action_files(
  DIRECTORY action
  FILES Exp.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exp_msgs
#  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_generation message_runtime roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)