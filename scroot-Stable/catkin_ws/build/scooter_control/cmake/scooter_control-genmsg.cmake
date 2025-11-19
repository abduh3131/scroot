# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "scooter_control: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iscooter_control:/home/jetson/catkin_ws/src/scooter_control/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(scooter_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg" NAME_WE)
add_custom_target(_scooter_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "scooter_control" "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg" ""
)

get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg" NAME_WE)
add_custom_target(_scooter_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "scooter_control" "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg" ""
)

get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg" NAME_WE)
add_custom_target(_scooter_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "scooter_control" "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg" "scooter_control/YoloDetection:std_msgs/Header"
)

get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg" NAME_WE)
add_custom_target(_scooter_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "scooter_control" "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg" "sensor_msgs/Image:std_msgs/Header:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scooter_control
)
_generate_msg_cpp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scooter_control
)
_generate_msg_cpp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scooter_control
)
_generate_msg_cpp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scooter_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(scooter_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scooter_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(scooter_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(scooter_control_generate_messages scooter_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_cpp _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_cpp _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_cpp _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_cpp _scooter_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scooter_control_gencpp)
add_dependencies(scooter_control_gencpp scooter_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scooter_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scooter_control
)
_generate_msg_eus(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scooter_control
)
_generate_msg_eus(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scooter_control
)
_generate_msg_eus(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scooter_control
)

### Generating Services

### Generating Module File
_generate_module_eus(scooter_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scooter_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(scooter_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(scooter_control_generate_messages scooter_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_eus _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_eus _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_eus _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_eus _scooter_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scooter_control_geneus)
add_dependencies(scooter_control_geneus scooter_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scooter_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scooter_control
)
_generate_msg_lisp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scooter_control
)
_generate_msg_lisp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scooter_control
)
_generate_msg_lisp(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scooter_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(scooter_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scooter_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(scooter_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(scooter_control_generate_messages scooter_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_lisp _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_lisp _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_lisp _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_lisp _scooter_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scooter_control_genlisp)
add_dependencies(scooter_control_genlisp scooter_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scooter_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/scooter_control
)
_generate_msg_nodejs(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/scooter_control
)
_generate_msg_nodejs(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/scooter_control
)
_generate_msg_nodejs(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/scooter_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(scooter_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/scooter_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(scooter_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(scooter_control_generate_messages scooter_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_nodejs _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_nodejs _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_nodejs _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_nodejs _scooter_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scooter_control_gennodejs)
add_dependencies(scooter_control_gennodejs scooter_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scooter_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control
)
_generate_msg_py(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control
)
_generate_msg_py(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control
)
_generate_msg_py(scooter_control
  "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control
)

### Generating Services

### Generating Module File
_generate_module_py(scooter_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(scooter_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(scooter_control_generate_messages scooter_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/AICommand.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_py _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetection.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_py _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_py _scooter_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/scooter_control/msg/SensorHub.msg" NAME_WE)
add_dependencies(scooter_control_generate_messages_py _scooter_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(scooter_control_genpy)
add_dependencies(scooter_control_genpy scooter_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS scooter_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scooter_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/scooter_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(scooter_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(scooter_control_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(scooter_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scooter_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/scooter_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(scooter_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(scooter_control_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(scooter_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scooter_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/scooter_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(scooter_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(scooter_control_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(scooter_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/scooter_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/scooter_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(scooter_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(scooter_control_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(scooter_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/scooter_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(scooter_control_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(scooter_control_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(scooter_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
