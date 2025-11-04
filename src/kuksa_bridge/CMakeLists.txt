cmake_minimum_required(VERSION 3.5)
project(kuksa_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY kuksa_bridge
  DESTINATION lib/python3.10/site-packages
)

ament_package()

