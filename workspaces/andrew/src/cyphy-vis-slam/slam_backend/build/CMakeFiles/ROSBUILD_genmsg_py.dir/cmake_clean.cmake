FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/slam_backend/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/slam_backend/msg/__init__.py"
  "../src/slam_backend/msg/_AddLoopClosure.py"
  "../src/slam_backend/msg/_Graph.py"
  "../src/slam_backend/msg/_NodeAdded.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
