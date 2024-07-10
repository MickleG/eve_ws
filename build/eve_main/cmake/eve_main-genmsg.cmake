# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "eve_main: 1 messages, 2 services")

set(MSG_I_FLAGS "-Ieve_main:/root/eve_ws/src/eve_main/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(eve_main_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg" NAME_WE)
add_custom_target(_eve_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eve_main" "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg" ""
)

get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GetPosition.srv" NAME_WE)
add_custom_target(_eve_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eve_main" "/root/eve_ws/src/eve_main/srv/GetPosition.srv" ""
)

get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GoToPosition.srv" NAME_WE)
add_custom_target(_eve_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eve_main" "/root/eve_ws/src/eve_main/srv/GoToPosition.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(eve_main
  "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eve_main
)

### Generating Services
_generate_srv_cpp(eve_main
  "/root/eve_ws/src/eve_main/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eve_main
)
_generate_srv_cpp(eve_main
  "/root/eve_ws/src/eve_main/srv/GoToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eve_main
)

### Generating Module File
_generate_module_cpp(eve_main
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eve_main
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(eve_main_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(eve_main_generate_messages eve_main_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg" NAME_WE)
add_dependencies(eve_main_generate_messages_cpp _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GetPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_cpp _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GoToPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_cpp _eve_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eve_main_gencpp)
add_dependencies(eve_main_gencpp eve_main_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eve_main_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(eve_main
  "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eve_main
)

### Generating Services
_generate_srv_eus(eve_main
  "/root/eve_ws/src/eve_main/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eve_main
)
_generate_srv_eus(eve_main
  "/root/eve_ws/src/eve_main/srv/GoToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eve_main
)

### Generating Module File
_generate_module_eus(eve_main
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eve_main
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(eve_main_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(eve_main_generate_messages eve_main_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg" NAME_WE)
add_dependencies(eve_main_generate_messages_eus _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GetPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_eus _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GoToPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_eus _eve_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eve_main_geneus)
add_dependencies(eve_main_geneus eve_main_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eve_main_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(eve_main
  "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eve_main
)

### Generating Services
_generate_srv_lisp(eve_main
  "/root/eve_ws/src/eve_main/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eve_main
)
_generate_srv_lisp(eve_main
  "/root/eve_ws/src/eve_main/srv/GoToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eve_main
)

### Generating Module File
_generate_module_lisp(eve_main
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eve_main
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(eve_main_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(eve_main_generate_messages eve_main_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg" NAME_WE)
add_dependencies(eve_main_generate_messages_lisp _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GetPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_lisp _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GoToPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_lisp _eve_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eve_main_genlisp)
add_dependencies(eve_main_genlisp eve_main_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eve_main_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(eve_main
  "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eve_main
)

### Generating Services
_generate_srv_nodejs(eve_main
  "/root/eve_ws/src/eve_main/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eve_main
)
_generate_srv_nodejs(eve_main
  "/root/eve_ws/src/eve_main/srv/GoToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eve_main
)

### Generating Module File
_generate_module_nodejs(eve_main
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eve_main
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(eve_main_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(eve_main_generate_messages eve_main_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg" NAME_WE)
add_dependencies(eve_main_generate_messages_nodejs _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GetPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_nodejs _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GoToPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_nodejs _eve_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eve_main_gennodejs)
add_dependencies(eve_main_gennodejs eve_main_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eve_main_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(eve_main
  "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eve_main
)

### Generating Services
_generate_srv_py(eve_main
  "/root/eve_ws/src/eve_main/srv/GetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eve_main
)
_generate_srv_py(eve_main
  "/root/eve_ws/src/eve_main/srv/GoToPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eve_main
)

### Generating Module File
_generate_module_py(eve_main
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eve_main
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(eve_main_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(eve_main_generate_messages eve_main_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/eve_ws/src/eve_main/msg/EndEffectorPosition.msg" NAME_WE)
add_dependencies(eve_main_generate_messages_py _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GetPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_py _eve_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/eve_ws/src/eve_main/srv/GoToPosition.srv" NAME_WE)
add_dependencies(eve_main_generate_messages_py _eve_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eve_main_genpy)
add_dependencies(eve_main_genpy eve_main_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eve_main_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eve_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eve_main
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(eve_main_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eve_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eve_main
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(eve_main_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eve_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eve_main
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(eve_main_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eve_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eve_main
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(eve_main_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eve_main)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eve_main\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eve_main
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(eve_main_generate_messages_py std_msgs_generate_messages_py)
endif()
