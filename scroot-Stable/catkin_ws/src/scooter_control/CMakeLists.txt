cmake_minimum_required(VERSION 3.5)
project(scooter_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
)

add_message_files(
  FILES
    Detection.msg
    DetectionArray.msg
    ActuatorCommand.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
    cv_bridge
)

catkin_install_python(
  PROGRAMS
    scripts/dual_yolo_node.py
    scripts/sensor_fusion_planner_node.py
    scripts/controller_node.py
    scripts/live_overlay_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
