FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/image_cache/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/image_cache/GetNeighbours.h"
  "../srv_gen/cpp/include/image_cache/GetInfo.h"
  "../srv_gen/cpp/include/image_cache/GetImage.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
