# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "racecarDepth: 1 messages, 0 services")

set(MSG_I_FLAGS "-IracecarDepth:/home/aaron/racecar_ws4.1/src/racecarDepth/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(racecarDepth_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg" NAME_WE)
add_custom_target(_racecarDepth_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "racecarDepth" "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(racecarDepth
  "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecarDepth
)

### Generating Services

### Generating Module File
_generate_module_cpp(racecarDepth
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecarDepth
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(racecarDepth_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(racecarDepth_generate_messages racecarDepth_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg" NAME_WE)
add_dependencies(racecarDepth_generate_messages_cpp _racecarDepth_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecarDepth_gencpp)
add_dependencies(racecarDepth_gencpp racecarDepth_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecarDepth_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(racecarDepth
  "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecarDepth
)

### Generating Services

### Generating Module File
_generate_module_eus(racecarDepth
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecarDepth
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(racecarDepth_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(racecarDepth_generate_messages racecarDepth_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg" NAME_WE)
add_dependencies(racecarDepth_generate_messages_eus _racecarDepth_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecarDepth_geneus)
add_dependencies(racecarDepth_geneus racecarDepth_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecarDepth_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(racecarDepth
  "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecarDepth
)

### Generating Services

### Generating Module File
_generate_module_lisp(racecarDepth
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecarDepth
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(racecarDepth_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(racecarDepth_generate_messages racecarDepth_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg" NAME_WE)
add_dependencies(racecarDepth_generate_messages_lisp _racecarDepth_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecarDepth_genlisp)
add_dependencies(racecarDepth_genlisp racecarDepth_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecarDepth_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(racecarDepth
  "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecarDepth
)

### Generating Services

### Generating Module File
_generate_module_nodejs(racecarDepth
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecarDepth
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(racecarDepth_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(racecarDepth_generate_messages racecarDepth_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg" NAME_WE)
add_dependencies(racecarDepth_generate_messages_nodejs _racecarDepth_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecarDepth_gennodejs)
add_dependencies(racecarDepth_gennodejs racecarDepth_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecarDepth_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(racecarDepth
  "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecarDepth
)

### Generating Services

### Generating Module File
_generate_module_py(racecarDepth
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecarDepth
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(racecarDepth_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(racecarDepth_generate_messages racecarDepth_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aaron/racecar_ws4.1/src/racecarDepth/msg/ObsPose.msg" NAME_WE)
add_dependencies(racecarDepth_generate_messages_py _racecarDepth_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(racecarDepth_genpy)
add_dependencies(racecarDepth_genpy racecarDepth_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS racecarDepth_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecarDepth)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/racecarDepth
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(racecarDepth_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecarDepth)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/racecarDepth
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(racecarDepth_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecarDepth)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/racecarDepth
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(racecarDepth_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecarDepth)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/racecarDepth
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(racecarDepth_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecarDepth)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecarDepth\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/racecarDepth
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(racecarDepth_generate_messages_py std_msgs_generate_messages_py)
endif()
