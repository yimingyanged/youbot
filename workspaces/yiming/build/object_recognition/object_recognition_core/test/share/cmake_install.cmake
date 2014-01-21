# Install script for directory: /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/test/share

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/yiming/ros_workspace/youbot/workspaces/yiming/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_recognition_core/test" TYPE PROGRAM FILES
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/test/share/run_test.sh"
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/test/share/desktop_test.sh"
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/test/share/test_config.py"
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/test/share/test_help.py"
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/test/share/test_sink.py"
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/test/share/test_source.py"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

