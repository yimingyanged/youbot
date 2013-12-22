cmake_minimum_required(VERSION 2.8.3)
project(kdl_kinematics_plugin_upgrade)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time program_options signals thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  rosconsole
  message_filters
  srdfdom
  urdf
  tf
  tf_conversions
  moveit_ros_planning
  dynamic_reconfigure
)

find_package(Eigen REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

generate_dynamic_reconfigure_options(
  cfg/kdl_ik_params.cfg
)

catkin_package(
  LIBRARIES
    moveit_rdf_loader
    moveit_ros_planning
    moveit_robot_model_loader
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pluginlib
    moveit_core
    moveit_ros_planning
)

include_directories(
					include
                    ${catkin_INCLUDE_DIRS}
		    )
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(MOVEIT_LIB_NAME moveit_kdl_kinematics_upgrade_plugin)

add_library(${MOVEIT_LIB_NAME} src/kdl_kinematics_plugin.cpp 
  src/chainiksolver_pos_nr_jl_mimic.cpp 
  src/chainiksolver_vel_pinv_mimic.cpp)
  
add_dependencies(${MOVEIT_LIB_NAME} ${PROJECT_NAME}_gencfg) 

target_link_libraries(${MOVEIT_LIB_NAME} moveit_rdf_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(
  FILES
    kdl_kinematics_plugin_upgrade_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
















