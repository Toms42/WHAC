cmake_minimum_required(VERSION 2.8.3)
project(whac_motion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  std_msgs
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  tf2_sensor_msgs
  )

catkin_python_setup()

add_service_files(
  FILES
  GotoGoal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  tf2_sensor_msgs
  )