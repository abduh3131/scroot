cmake_minimum_required(VERSION 3.0.2)
project(scroot_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_interface
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_interface
)

catkin_install_python(
  PROGRAMS
    scripts/ai_input_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
