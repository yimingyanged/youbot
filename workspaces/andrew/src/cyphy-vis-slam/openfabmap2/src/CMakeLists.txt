# source files
set(NODE_SOURCES 
  BOWMSCTrainer.cpp
  ChowLiuTree.cpp
  FabMap.cpp
  openfabmap2_ros.cpp)

# TODO: Learn + Run
#rosbuild_add_executable(openfabmap2 openfabmap2.cpp ${NODE_SOURCES})
#target_link_libraries(openfabmap2 ${opencv})

rosbuild_add_executable(run_node run_node.cpp ${NODE_SOURCES})
target_link_libraries(run_node ${opencv})

rosbuild_add_executable(learn_node learn_node.cpp ${NODE_SOURCES})
target_link_libraries(learn_node ${opencv})