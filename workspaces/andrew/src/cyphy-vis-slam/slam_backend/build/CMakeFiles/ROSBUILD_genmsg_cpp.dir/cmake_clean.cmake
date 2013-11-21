FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/slam_backend/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/slam_backend/AddLoopClosure.h"
  "../msg_gen/cpp/include/slam_backend/Graph.h"
  "../msg_gen/cpp/include/slam_backend/NodeAdded.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
