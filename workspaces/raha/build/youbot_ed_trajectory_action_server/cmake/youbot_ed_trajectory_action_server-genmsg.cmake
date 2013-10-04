# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "youbot_ed_trajectory_action_server: 0 messages, 0 services")

set(MSG_I_FLAGS "-Iactionlib_msgs:/opt/ros/groovy/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(youbot_ed_trajectory_action_server_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(youbot_ed_trajectory_action_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_ed_trajectory_action_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(youbot_ed_trajectory_action_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(youbot_ed_trajectory_action_server_generate_messages youbot_ed_trajectory_action_server_generate_messages_cpp)

# target for backward compatibility
add_custom_target(youbot_ed_trajectory_action_server_gencpp)
add_dependencies(youbot_ed_trajectory_action_server_gencpp youbot_ed_trajectory_action_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_ed_trajectory_action_server_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(youbot_ed_trajectory_action_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_ed_trajectory_action_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(youbot_ed_trajectory_action_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(youbot_ed_trajectory_action_server_generate_messages youbot_ed_trajectory_action_server_generate_messages_lisp)

# target for backward compatibility
add_custom_target(youbot_ed_trajectory_action_server_genlisp)
add_dependencies(youbot_ed_trajectory_action_server_genlisp youbot_ed_trajectory_action_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_ed_trajectory_action_server_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(youbot_ed_trajectory_action_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_ed_trajectory_action_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(youbot_ed_trajectory_action_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(youbot_ed_trajectory_action_server_generate_messages youbot_ed_trajectory_action_server_generate_messages_py)

# target for backward compatibility
add_custom_target(youbot_ed_trajectory_action_server_genpy)
add_dependencies(youbot_ed_trajectory_action_server_genpy youbot_ed_trajectory_action_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_ed_trajectory_action_server_generate_messages_py)


debug_message(2 "youbot_ed_trajectory_action_server: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_ed_trajectory_action_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_ed_trajectory_action_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_ed_trajectory_action_server_generate_messages_cpp actionlib_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_ed_trajectory_action_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_ed_trajectory_action_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_ed_trajectory_action_server_generate_messages_lisp actionlib_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_ed_trajectory_action_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_ed_trajectory_action_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_ed_trajectory_action_server
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_ed_trajectory_action_server_generate_messages_py actionlib_msgs_generate_messages_py)
