FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/slam_backend/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/AddLoopClosure.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_AddLoopClosure.lisp"
  "../msg_gen/lisp/Graph.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Graph.lisp"
  "../msg_gen/lisp/NodeAdded.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_NodeAdded.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
