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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yiming/ros_workspace/youbot/workspaces/yiming/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yiming/ros_workspace/youbot/workspaces/yiming/build

# Include any dependencies generated for this target.
include object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/depend.make

# Include the progress variables for this target.
include object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/progress.make

# Include the compile flags for this target's objects.
include object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/flags.make

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o: object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/flags.make
object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o: /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/ork_renderer/src/renderer2d.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yiming/ros_workspace/youbot/workspaces/yiming/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o"
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/ork_renderer/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o -c /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/ork_renderer/src/renderer2d.cpp

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.i"
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/ork_renderer/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/ork_renderer/src/renderer2d.cpp > CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.i

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.s"
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/ork_renderer/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/ork_renderer/src/renderer2d.cpp -o CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.s

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.requires:
.PHONY : object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.requires

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.provides: object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.requires
	$(MAKE) -f object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/build.make object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.provides.build
.PHONY : object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.provides

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.provides.build: object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o

# Object files for target object_recognition_renderer_2d
object_recognition_renderer_2d_OBJECTS = \
"CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o"

# External object files for target object_recognition_renderer_2d
object_recognition_renderer_2d_EXTERNAL_OBJECTS =

/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_calib3d.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_contrib.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_core.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_features2d.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_flann.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_gpu.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_highgui.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_imgproc.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_legacy.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_ml.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_nonfree.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_objdetect.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_photo.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_stitching.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_superres.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_video.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: /opt/ros/hydro/lib/libopencv_videostab.so
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/build.make
/home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so: object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so"
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/ork_renderer/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/object_recognition_renderer_2d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/build: /home/yiming/ros_workspace/youbot/workspaces/yiming/devel/lib/libobject_recognition_renderer_2d.so
.PHONY : object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/build

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/requires: object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/renderer2d.cpp.o.requires
.PHONY : object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/requires

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/clean:
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/ork_renderer/src && $(CMAKE_COMMAND) -P CMakeFiles/object_recognition_renderer_2d.dir/cmake_clean.cmake
.PHONY : object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/clean

object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/depend:
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yiming/ros_workspace/youbot/workspaces/yiming/src /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/ork_renderer/src /home/yiming/ros_workspace/youbot/workspaces/yiming/build /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/ork_renderer/src /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : object_recognition/ork_renderer/src/CMakeFiles/object_recognition_renderer_2d.dir/depend

