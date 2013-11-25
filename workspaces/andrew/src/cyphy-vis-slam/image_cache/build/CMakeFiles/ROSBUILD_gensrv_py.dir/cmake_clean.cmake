FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/image_cache/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/image_cache/srv/__init__.py"
  "../src/image_cache/srv/_GetNeighbours.py"
  "../src/image_cache/srv/_GetInfo.py"
  "../src/image_cache/srv/_GetImage.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
