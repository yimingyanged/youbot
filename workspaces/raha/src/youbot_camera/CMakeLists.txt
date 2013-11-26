cmake_minimum_required(VERSION 2.8.3)
project(youbot_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
########### ROS ###########
find_package(catkin REQUIRED COMPONENTS roscpp openni_launch tf)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES YouBotDriver soem
#  CATKIN_DEPENDS rosconsole
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_definitions(-DUSE_ROS_LOGGING)
set(CMAKE_BUILD_TYPE Release)

#copy header files
#install(DIRECTORY include/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
#)

add_executable(ar_camera_calibrator src/ARCameraCalibrator.cpp)
target_link_libraries(ar_camera_calibrator ${catkin_LIBRARIES})
