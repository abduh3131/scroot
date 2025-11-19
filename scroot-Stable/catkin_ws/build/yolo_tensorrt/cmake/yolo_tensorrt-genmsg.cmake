# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "yolo_tensorrt: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iyolo_tensorrt:/home/jetson/catkin_ws/src/yolo_tensorrt/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(yolo_tensorrt_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg" NAME_WE)
add_custom_target(_yolo_tensorrt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yolo_tensorrt" "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg" ""
)

get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg" NAME_WE)
add_custom_target(_yolo_tensorrt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yolo_tensorrt" "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg" "std_msgs/Header:yolo_tensorrt/YoloDetection"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_tensorrt
)
_generate_msg_cpp(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_tensorrt
)

### Generating Services

### Generating Module File
_generate_module_cpp(yolo_tensorrt
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_tensorrt
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(yolo_tensorrt_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(yolo_tensorrt_generate_messages yolo_tensorrt_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_cpp _yolo_tensorrt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_cpp _yolo_tensorrt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_tensorrt_gencpp)
add_dependencies(yolo_tensorrt_gencpp yolo_tensorrt_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_tensorrt_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_tensorrt
)
_generate_msg_eus(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_tensorrt
)

### Generating Services

### Generating Module File
_generate_module_eus(yolo_tensorrt
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_tensorrt
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(yolo_tensorrt_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(yolo_tensorrt_generate_messages yolo_tensorrt_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_eus _yolo_tensorrt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_eus _yolo_tensorrt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_tensorrt_geneus)
add_dependencies(yolo_tensorrt_geneus yolo_tensorrt_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_tensorrt_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_tensorrt
)
_generate_msg_lisp(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_tensorrt
)

### Generating Services

### Generating Module File
_generate_module_lisp(yolo_tensorrt
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_tensorrt
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(yolo_tensorrt_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(yolo_tensorrt_generate_messages yolo_tensorrt_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_lisp _yolo_tensorrt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_lisp _yolo_tensorrt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_tensorrt_genlisp)
add_dependencies(yolo_tensorrt_genlisp yolo_tensorrt_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_tensorrt_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_tensorrt
)
_generate_msg_nodejs(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_tensorrt
)

### Generating Services

### Generating Module File
_generate_module_nodejs(yolo_tensorrt
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_tensorrt
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(yolo_tensorrt_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(yolo_tensorrt_generate_messages yolo_tensorrt_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_nodejs _yolo_tensorrt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_nodejs _yolo_tensorrt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_tensorrt_gennodejs)
add_dependencies(yolo_tensorrt_gennodejs yolo_tensorrt_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_tensorrt_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_tensorrt
)
_generate_msg_py(yolo_tensorrt
  "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_tensorrt
)

### Generating Services

### Generating Module File
_generate_module_py(yolo_tensorrt
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_tensorrt
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(yolo_tensorrt_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(yolo_tensorrt_generate_messages yolo_tensorrt_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetection.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_py _yolo_tensorrt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/catkin_ws/src/yolo_tensorrt/msg/YoloDetectionArray.msg" NAME_WE)
add_dependencies(yolo_tensorrt_generate_messages_py _yolo_tensorrt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yolo_tensorrt_genpy)
add_dependencies(yolo_tensorrt_genpy yolo_tensorrt_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yolo_tensorrt_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_tensorrt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yolo_tensorrt
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(yolo_tensorrt_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(yolo_tensorrt_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_tensorrt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yolo_tensorrt
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(yolo_tensorrt_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(yolo_tensorrt_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_tensorrt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yolo_tensorrt
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(yolo_tensorrt_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(yolo_tensorrt_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_tensorrt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yolo_tensorrt
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(yolo_tensorrt_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(yolo_tensorrt_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_tensorrt)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_tensorrt\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yolo_tensorrt
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(yolo_tensorrt_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(yolo_tensorrt_generate_messages_py sensor_msgs_generate_messages_py)
endif()
