# - Find the yaml-cpp include file and library
#
#  yaml-cpp_FOUND - system has yaml-cpp library
#  yaml-cpp_LIBRARY - The libraries needed to use yaml-cpp
#  yaml-cpp_LIBRARY_DIRS - The linking directories needed to use yaml-cpp
#  yaml-cpp_HAVE_H - true if yaml.h is available
#  yaml-cpp_INCLUDE_DIR - yaml.h path
#

# The first of the find_library calls that succeeds will be the one that takes effect and the result will be cached.
find_library(yaml-cpp_LIBRARY NAMES yaml-cpp-0.5)
find_library(yaml-cpp_LIBRARY HINTS /usr/local/lib NAMES yaml-cpp)

get_filename_component(yaml-cpp_LIBRARY_DIRS "${yaml-cpp_LIBRARY}" PATH)

find_file(yaml-cpp_HAVE_H yaml-cpp-0.5/yaml-cpp/yaml.h)
find_file(yaml-cpp_HAVE_H yaml-cpp/yaml.h)

find_path(yaml-cpp_H_INCLUDE_DIR yaml-cpp-0.5/yaml-cpp/yaml.h yaml-cpp/yaml.h)

if (yaml-cpp_H_INCLUDE_DIR)
 SET(yaml-cpp_INCLUDE_DIR ${yaml-cpp_H_INCLUDE_DIR}/yaml-cpp-0.5)
else()
 find_path(yaml-cpp_H_INCLUDE_DIR yaml-cpp/yaml.h)
 SET(yaml-cpp_INCLUDE_DIR ${yaml-cpp_H_INCLUDE_DIR})
endif(yaml-cpp_H_INCLUDE_DIR)

SET(yaml-cpp_FOUND FALSE)
if (yaml-cpp_HAVE_H)
 if (yaml-cpp_LIBRARY)
  SET(yaml-cpp_FOUND TRUE)
 endif (yaml-cpp_LIBRARY)
endif (yaml-cpp_HAVE_H)

if(NOT yaml-cpp_FOUND)
 message(STATUS "YamlCpp Can't be found")
endif(NOT yaml-cpp_FOUND)
message(STATUS "YamlCpp Library: " ${yaml-cpp_LIBRARY} )
message(STATUS "YamlCpp Linking Dirs: " ${yaml-cpp_LIBRARY_DIRS} )
message(STATUS "YamlCpp Header: " ${yaml-cpp_INCLUDE_DIR} )
