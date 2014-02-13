# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()


if(youbot_manipulator_CONFIG_INCLUDED)
  return()
endif()
set(youbot_manipulator_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(youbot_manipulator_SOURCE_PREFIX /home/yiming/ros_workspace/youbot/workspaces/yiming/src/youbot_manipulator)
  set(youbot_manipulator_DEVEL_PREFIX /home/yiming/ros_workspace/youbot/workspaces/yiming/devel)
  set(youbot_manipulator_INSTALL_PREFIX "")
  set(youbot_manipulator_PREFIX ${youbot_manipulator_DEVEL_PREFIX})
else()
  set(youbot_manipulator_SOURCE_PREFIX "")
  set(youbot_manipulator_DEVEL_PREFIX "")
  set(youbot_manipulator_INSTALL_PREFIX /home/yiming/ros_workspace/youbot/workspaces/yiming/install)
  set(youbot_manipulator_PREFIX ${youbot_manipulator_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'youbot_manipulator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(youbot_manipulator_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/youbot_manipulator/include" STREQUAL "")
  set(youbot_manipulator_INCLUDE_DIRS "")
  set(_include_dirs "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/youbot_manipulator/include")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir}" STREQUAL "include")
      get_filename_component(include "${youbot_manipulator_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'youbot_manipulator' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'yiming <yiming@todo.todo>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'youbot_manipulator' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/yiming/ros_workspace/youbot/workspaces/yiming/src/youbot_manipulator/${idir}'.  Ask the maintainer 'yiming <yiming@todo.todo>' to fix it.")
    endif()
    _list_append_unique(youbot_manipulator_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "youbot_manipulator")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^debug|optimized|general$")
    list(APPEND youbot_manipulator_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND youbot_manipulator_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND youbot_manipulator_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib;/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib;/opt/ros/hydro/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(youbot_manipulator_LIBRARY_DIRS ${lib_path})
      list(APPEND youbot_manipulator_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'youbot_manipulator'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND youbot_manipulator_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(youbot_manipulator_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${youbot_manipulator_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "geometry_msgs;moveit_core;roscpp;moveit_msgs;moveit_ros_planning_interface;actionlib;actionlib_msgs;std_msgs;tf")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 youbot_manipulator_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${youbot_manipulator_dep}_FOUND)
      find_package(${youbot_manipulator_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${youbot_manipulator_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(youbot_manipulator_INCLUDE_DIRS ${${youbot_manipulator_dep}_INCLUDE_DIRS})
  _list_append_deduplicate(youbot_manipulator_LIBRARIES ${${youbot_manipulator_dep}_LIBRARIES})
  _list_append_unique(youbot_manipulator_LIBRARY_DIRS ${${youbot_manipulator_dep}_LIBRARY_DIRS})
  list(APPEND youbot_manipulator_EXPORTED_TARGETS ${${youbot_manipulator_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${youbot_manipulator_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
