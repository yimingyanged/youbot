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

# Utility rule file for object_recognition_core-doxygen.

# Include the progress variables for this target.
include object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/progress.make

object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen:
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yiming/ros_workspace/youbot/workspaces/yiming/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating API documentation with Doxygen"

object_recognition_core-doxygen: object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen
object_recognition_core-doxygen: object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/build.make
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core && /usr/bin/doxygen /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/Doxyfile
.PHONY : object_recognition_core-doxygen

# Rule to build all files generated by this target.
object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/build: object_recognition_core-doxygen
.PHONY : object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/build

object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/clean:
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core && $(CMAKE_COMMAND) -P CMakeFiles/object_recognition_core-doxygen.dir/cmake_clean.cmake
.PHONY : object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/clean

object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/depend:
	cd /home/yiming/ros_workspace/youbot/workspaces/yiming/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yiming/ros_workspace/youbot/workspaces/yiming/src /home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/object_recognition_core /home/yiming/ros_workspace/youbot/workspaces/yiming/build /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core /home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : object_recognition/object_recognition_core/CMakeFiles/object_recognition_core-doxygen.dir/depend

