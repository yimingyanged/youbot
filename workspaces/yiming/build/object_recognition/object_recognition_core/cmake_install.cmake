# Install script for directory: /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/catkin_generated/installspace/object_recognition_core.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_recognition_core/cmake" TYPE FILE FILES "/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/catkin_generated/installspace/test.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_recognition_core/cmake" TYPE FILE FILES
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/catkin_generated/installspace/object_recognition_coreConfig.cmake"
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/catkin_generated/installspace/object_recognition_coreConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_recognition_core" TYPE FILE FILES "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  INCLUDE("/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/catkin_generated/safe_execute_install.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/object_recognition_core" TYPE DIRECTORY FILES "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/include/object_recognition_core/")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_recognition_core" TYPE DIRECTORY FILES "/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/share/object_recognition_core/test" USE_SOURCE_PERMISSIONS)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_recognition_core" TYPE PROGRAM FILES
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/apps/detection"
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/apps/training"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_recognition_core" TYPE DIRECTORY FILES "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core/apps/dbscripts" USE_SOURCE_PERMISSIONS)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/python/cmake_install.cmake")
  INCLUDE("/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/src/cmake_install.cmake")
  INCLUDE("/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/web_ui/cmake_install.cmake")
  INCLUDE("/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/test/cmake_install.cmake")
  INCLUDE("/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/doc/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

