# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/build

# Include any dependencies generated for this target.
include CMakeFiles/cache_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cache_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cache_server.dir/flags.make

CMakeFiles/cache_server.dir/src/cache_server.cpp.o: CMakeFiles/cache_server.dir/flags.make
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: ../src/cache_server.cpp
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: ../manifest.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/cpp_common/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/rostime/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/roscpp_traits/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/roscpp_serialization/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/genmsg/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/genpy/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/message_runtime/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/rosconsole/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/std_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/rosgraph_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/xmlrpcpp/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/roscpp/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/geometry_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/sensor_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/camera_calibration_parsers/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/catkin/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/opencv2/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/cv_bridge/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/stereo_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/message_filters/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/console_bridge/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/class_loader/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/rospack/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/roslib/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/pluginlib/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/image_transport/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/image_geometry/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /media/sf_youbot_edinburgh/workspaces/andrew/src/g2o/manifest.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/gencpp/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/genlisp/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/message_generation/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/actionlib_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/nav_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /opt/ros/hydro/share/visualization_msgs/package.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/slam_backend/manifest.xml
CMakeFiles/cache_server.dir/src/cache_server.cpp.o: /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/slam_backend/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cache_server.dir/src/cache_server.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/cache_server.dir/src/cache_server.cpp.o -c /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/src/cache_server.cpp

CMakeFiles/cache_server.dir/src/cache_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cache_server.dir/src/cache_server.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/src/cache_server.cpp > CMakeFiles/cache_server.dir/src/cache_server.cpp.i

CMakeFiles/cache_server.dir/src/cache_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cache_server.dir/src/cache_server.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/src/cache_server.cpp -o CMakeFiles/cache_server.dir/src/cache_server.cpp.s

CMakeFiles/cache_server.dir/src/cache_server.cpp.o.requires:
.PHONY : CMakeFiles/cache_server.dir/src/cache_server.cpp.o.requires

CMakeFiles/cache_server.dir/src/cache_server.cpp.o.provides: CMakeFiles/cache_server.dir/src/cache_server.cpp.o.requires
	$(MAKE) -f CMakeFiles/cache_server.dir/build.make CMakeFiles/cache_server.dir/src/cache_server.cpp.o.provides.build
.PHONY : CMakeFiles/cache_server.dir/src/cache_server.cpp.o.provides

CMakeFiles/cache_server.dir/src/cache_server.cpp.o.provides.build: CMakeFiles/cache_server.dir/src/cache_server.cpp.o

# Object files for target cache_server
cache_server_OBJECTS = \
"CMakeFiles/cache_server.dir/src/cache_server.cpp.o"

# External object files for target cache_server
cache_server_EXTERNAL_OBJECTS =

../bin/cache_server: CMakeFiles/cache_server.dir/src/cache_server.cpp.o
../bin/cache_server: CMakeFiles/cache_server.dir/build.make
../bin/cache_server: /opt/ros/hydro/lib/libopencv_calib3d.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_contrib.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_core.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_features2d.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_flann.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_gpu.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_highgui.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_imgproc.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_legacy.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_ml.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_nonfree.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_objdetect.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_photo.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_stitching.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_superres.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_video.so
../bin/cache_server: /opt/ros/hydro/lib/libopencv_videostab.so
../bin/cache_server: CMakeFiles/cache_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/cache_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cache_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cache_server.dir/build: ../bin/cache_server
.PHONY : CMakeFiles/cache_server.dir/build

CMakeFiles/cache_server.dir/requires: CMakeFiles/cache_server.dir/src/cache_server.cpp.o.requires
.PHONY : CMakeFiles/cache_server.dir/requires

CMakeFiles/cache_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cache_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cache_server.dir/clean

CMakeFiles/cache_server.dir/depend:
	cd /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/build /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/build /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/build/CMakeFiles/cache_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cache_server.dir/depend
