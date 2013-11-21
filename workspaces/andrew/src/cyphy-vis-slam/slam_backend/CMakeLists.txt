cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(/usr/include/suitesparse)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#link_directories("/usr/local/lib")
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(slam_backend src/slam_backend.cpp)
target_link_libraries(slam_backend csparse cxsparse boost_thread g2o_cli g2o_opengl_helper g2o_solver_pcg g2o_types_sclam2d g2o_core g2o_parser g2o_solver_slam2d_linear  g2o_types_sim3 g2o_csparse_extension g2o_simulator g2o_solver_structure_only g2o_types_slam2d g2o_ext_freeglut_minimal g2o_solver_cholmod g2o_stuff g2o_types_slam3d g2o_incremental g2o_solver_csparse g2o_types_data g2o_interactive g2o_solver_dense g2o_types_icp g2o_interface g2o_solver_eigen g2o_types_sba)
