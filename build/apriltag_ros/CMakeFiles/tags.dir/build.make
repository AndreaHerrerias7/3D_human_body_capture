# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/andre/ros2_ws/src/apriltag_ros-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andre/ros2_ws/build/apriltag_ros

# Include any dependencies generated for this target.
include CMakeFiles/tags.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/tags.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/tags.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tags.dir/flags.make

CMakeFiles/tags.dir/src/tag_functions.cpp.o: CMakeFiles/tags.dir/flags.make
CMakeFiles/tags.dir/src/tag_functions.cpp.o: /home/andre/ros2_ws/src/apriltag_ros-master/src/tag_functions.cpp
CMakeFiles/tags.dir/src/tag_functions.cpp.o: CMakeFiles/tags.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andre/ros2_ws/build/apriltag_ros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tags.dir/src/tag_functions.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tags.dir/src/tag_functions.cpp.o -MF CMakeFiles/tags.dir/src/tag_functions.cpp.o.d -o CMakeFiles/tags.dir/src/tag_functions.cpp.o -c /home/andre/ros2_ws/src/apriltag_ros-master/src/tag_functions.cpp

CMakeFiles/tags.dir/src/tag_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tags.dir/src/tag_functions.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andre/ros2_ws/src/apriltag_ros-master/src/tag_functions.cpp > CMakeFiles/tags.dir/src/tag_functions.cpp.i

CMakeFiles/tags.dir/src/tag_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tags.dir/src/tag_functions.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andre/ros2_ws/src/apriltag_ros-master/src/tag_functions.cpp -o CMakeFiles/tags.dir/src/tag_functions.cpp.s

tags: CMakeFiles/tags.dir/src/tag_functions.cpp.o
tags: CMakeFiles/tags.dir/build.make
.PHONY : tags

# Rule to build all files generated by this target.
CMakeFiles/tags.dir/build: tags
.PHONY : CMakeFiles/tags.dir/build

CMakeFiles/tags.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tags.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tags.dir/clean

CMakeFiles/tags.dir/depend:
	cd /home/andre/ros2_ws/build/apriltag_ros && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andre/ros2_ws/src/apriltag_ros-master /home/andre/ros2_ws/src/apriltag_ros-master /home/andre/ros2_ws/build/apriltag_ros /home/andre/ros2_ws/build/apriltag_ros /home/andre/ros2_ws/build/apriltag_ros/CMakeFiles/tags.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tags.dir/depend

