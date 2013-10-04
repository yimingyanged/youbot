# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "youbot_common: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iyoubot_common:/home/youbot/ros/workspaces/youbot/src/youbot_oodl/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(youbot_common_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(youbot_common
  "/home/youbot/ros/workspaces/youbot/src/youbot_oodl/msg/PowerBoardState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_common
)

### Generating Services

### Generating Module File
_generate_module_cpp(youbot_common
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_common
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(youbot_common_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(youbot_common_generate_messages youbot_common_generate_messages_cpp)

# target for backward compatibility
add_custom_target(youbot_common_gencpp)
add_dependencies(youbot_common_gencpp youbot_common_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_common_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(youbot_common
  "/home/youbot/ros/workspaces/youbot/src/youbot_oodl/msg/PowerBoardState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_common
)

### Generating Services

### Generating Module File
_generate_module_lisp(youbot_common
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_common
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(youbot_common_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(youbot_common_generate_messages youbot_common_generate_messages_lisp)

# target for backward compatibility
add_custom_target(youbot_common_genlisp)
add_dependencies(youbot_common_genlisp youbot_common_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_common_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(youbot_common
  "/home/youbot/ros/workspaces/youbot/src/youbot_oodl/msg/PowerBoardState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_common
)

### Generating Services

### Generating Module File
_generate_module_py(youbot_common
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_common
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(youbot_common_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(youbot_common_generate_messages youbot_common_generate_messages_py)

# target for backward compatibility
add_custom_target(youbot_common_genpy)
add_dependencies(youbot_common_genpy youbot_common_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS youbot_common_generate_messages_py)


debug_message(2 "youbot_common: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/youbot_common
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_common_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/youbot_common
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_common_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_common)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_common\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/youbot_common
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(youbot_common_generate_messages_py std_msgs_generate_messages_py)
