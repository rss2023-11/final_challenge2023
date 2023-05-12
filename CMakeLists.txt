cmake_minimum_required(VERSION 3.1)

project(final_challenge2023)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  ConeLocation.msg
  ConeLocationPixel.msg
  DetectedObject.msg
  ParkingError.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)