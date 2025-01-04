# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "deprojection_pipeline_msgs: 2 messages, 1 services")

set(MSG_I_FLAGS "-Ideprojection_pipeline_msgs:/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(deprojection_pipeline_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg" NAME_WE)
add_custom_target(_deprojection_pipeline_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deprojection_pipeline_msgs" "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg" "geometry_msgs/PointStamped:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg" NAME_WE)
add_custom_target(_deprojection_pipeline_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deprojection_pipeline_msgs" "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg" "geometry_msgs/PointStamped:std_msgs/Header:geometry_msgs/Point:deprojection_pipeline_msgs/ObjectPosition"
)

get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv" NAME_WE)
add_custom_target(_deprojection_pipeline_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deprojection_pipeline_msgs" "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv" "deprojection_pipeline_msgs/ObjectPositions:std_msgs/Header:geometry_msgs/PointStamped:geometry_msgs/Point:deprojection_pipeline_msgs/ObjectPosition"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deprojection_pipeline_msgs
)
_generate_msg_cpp(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Services
_generate_srv_cpp(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv"
  "${MSG_I_FLAGS}"
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Module File
_generate_module_cpp(deprojection_pipeline_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deprojection_pipeline_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(deprojection_pipeline_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(deprojection_pipeline_msgs_generate_messages deprojection_pipeline_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_cpp _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_cpp _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_cpp _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deprojection_pipeline_msgs_gencpp)
add_dependencies(deprojection_pipeline_msgs_gencpp deprojection_pipeline_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deprojection_pipeline_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deprojection_pipeline_msgs
)
_generate_msg_eus(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Services
_generate_srv_eus(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv"
  "${MSG_I_FLAGS}"
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Module File
_generate_module_eus(deprojection_pipeline_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deprojection_pipeline_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(deprojection_pipeline_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(deprojection_pipeline_msgs_generate_messages deprojection_pipeline_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_eus _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_eus _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_eus _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deprojection_pipeline_msgs_geneus)
add_dependencies(deprojection_pipeline_msgs_geneus deprojection_pipeline_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deprojection_pipeline_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deprojection_pipeline_msgs
)
_generate_msg_lisp(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Services
_generate_srv_lisp(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv"
  "${MSG_I_FLAGS}"
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Module File
_generate_module_lisp(deprojection_pipeline_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deprojection_pipeline_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(deprojection_pipeline_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(deprojection_pipeline_msgs_generate_messages deprojection_pipeline_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_lisp _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_lisp _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_lisp _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deprojection_pipeline_msgs_genlisp)
add_dependencies(deprojection_pipeline_msgs_genlisp deprojection_pipeline_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deprojection_pipeline_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deprojection_pipeline_msgs
)
_generate_msg_nodejs(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Services
_generate_srv_nodejs(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv"
  "${MSG_I_FLAGS}"
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Module File
_generate_module_nodejs(deprojection_pipeline_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deprojection_pipeline_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(deprojection_pipeline_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(deprojection_pipeline_msgs_generate_messages deprojection_pipeline_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_nodejs _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_nodejs _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_nodejs _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deprojection_pipeline_msgs_gennodejs)
add_dependencies(deprojection_pipeline_msgs_gennodejs deprojection_pipeline_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deprojection_pipeline_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deprojection_pipeline_msgs
)
_generate_msg_py(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Services
_generate_srv_py(deprojection_pipeline_msgs
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv"
  "${MSG_I_FLAGS}"
  "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deprojection_pipeline_msgs
)

### Generating Module File
_generate_module_py(deprojection_pipeline_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deprojection_pipeline_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(deprojection_pipeline_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(deprojection_pipeline_msgs_generate_messages deprojection_pipeline_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPosition.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_py _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/msg/ObjectPositions.msg" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_py _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/barracuda/catkin_ws/src/deprojection_pipeline_msgs/srv/GetObjectLocations.srv" NAME_WE)
add_dependencies(deprojection_pipeline_msgs_generate_messages_py _deprojection_pipeline_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deprojection_pipeline_msgs_genpy)
add_dependencies(deprojection_pipeline_msgs_genpy deprojection_pipeline_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deprojection_pipeline_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deprojection_pipeline_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deprojection_pipeline_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(deprojection_pipeline_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deprojection_pipeline_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deprojection_pipeline_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(deprojection_pipeline_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deprojection_pipeline_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deprojection_pipeline_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(deprojection_pipeline_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deprojection_pipeline_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deprojection_pipeline_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(deprojection_pipeline_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deprojection_pipeline_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deprojection_pipeline_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deprojection_pipeline_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(deprojection_pipeline_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
