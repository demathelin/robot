# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "velocity_qp: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ivelocity_qp:/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(velocity_qp_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg" NAME_WE)
add_custom_target(_velocity_qp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "velocity_qp" "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg" "geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(velocity_qp
  "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velocity_qp
)

### Generating Services

### Generating Module File
_generate_module_cpp(velocity_qp
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velocity_qp
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(velocity_qp_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(velocity_qp_generate_messages velocity_qp_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg" NAME_WE)
add_dependencies(velocity_qp_generate_messages_cpp _velocity_qp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velocity_qp_gencpp)
add_dependencies(velocity_qp_gencpp velocity_qp_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velocity_qp_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(velocity_qp
  "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velocity_qp
)

### Generating Services

### Generating Module File
_generate_module_eus(velocity_qp
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velocity_qp
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(velocity_qp_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(velocity_qp_generate_messages velocity_qp_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg" NAME_WE)
add_dependencies(velocity_qp_generate_messages_eus _velocity_qp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velocity_qp_geneus)
add_dependencies(velocity_qp_geneus velocity_qp_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velocity_qp_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(velocity_qp
  "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velocity_qp
)

### Generating Services

### Generating Module File
_generate_module_lisp(velocity_qp
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velocity_qp
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(velocity_qp_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(velocity_qp_generate_messages velocity_qp_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg" NAME_WE)
add_dependencies(velocity_qp_generate_messages_lisp _velocity_qp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velocity_qp_genlisp)
add_dependencies(velocity_qp_genlisp velocity_qp_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velocity_qp_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(velocity_qp
  "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velocity_qp
)

### Generating Services

### Generating Module File
_generate_module_nodejs(velocity_qp
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velocity_qp
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(velocity_qp_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(velocity_qp_generate_messages velocity_qp_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg" NAME_WE)
add_dependencies(velocity_qp_generate_messages_nodejs _velocity_qp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velocity_qp_gennodejs)
add_dependencies(velocity_qp_gennodejs velocity_qp_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velocity_qp_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(velocity_qp
  "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velocity_qp
)

### Generating Services

### Generating Module File
_generate_module_py(velocity_qp
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velocity_qp
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(velocity_qp_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(velocity_qp_generate_messages velocity_qp_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/panda_ws/src/auctuspanda/velocity_qp/msg/PandaRunMsg.msg" NAME_WE)
add_dependencies(velocity_qp_generate_messages_py _velocity_qp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velocity_qp_genpy)
add_dependencies(velocity_qp_genpy velocity_qp_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velocity_qp_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velocity_qp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velocity_qp
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(velocity_qp_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(velocity_qp_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velocity_qp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velocity_qp
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(velocity_qp_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(velocity_qp_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velocity_qp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velocity_qp
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(velocity_qp_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(velocity_qp_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velocity_qp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velocity_qp
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(velocity_qp_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(velocity_qp_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velocity_qp)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velocity_qp\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velocity_qp
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(velocity_qp_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(velocity_qp_generate_messages_py geometry_msgs_generate_messages_py)
endif()
