cmake_minimum_required(VERSION 3.0.2)
project(sensor_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  message_generation
)

add_message_files(
  FILES
  SensorHub.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_filters
    message_runtime
)

install(PROGRAMS
  scripts/sensor_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
