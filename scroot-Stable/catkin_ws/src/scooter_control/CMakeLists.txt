cmake_minimum_required(VERSION 3.0.2)
project(scooter_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  AICommand.msg
  YoloDetection.msg
  YoloDetectionArray.msg
  SensorHub.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs message_runtime
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/command_arbitrator_node.py
  scripts/cruise_control_node.py
  scripts/dual_yolo_node.py
  scripts/lane_assist_node.py
  scripts/object_avoidance_node.py
  scripts/safety_supervisor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)