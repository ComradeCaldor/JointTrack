# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/caldor/gazetrack/MyAruco

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/caldor/gazetrack/MyAruco/build

# Include any dependencies generated for this target.
include CMakeFiles/myObjects.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myObjects.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myObjects.dir/flags.make

myObjects: CMakeFiles/myObjects.dir/build.make

.PHONY : myObjects

# Rule to build all files generated by this target.
CMakeFiles/myObjects.dir/build: myObjects

.PHONY : CMakeFiles/myObjects.dir/build

CMakeFiles/myObjects.dir/requires:

.PHONY : CMakeFiles/myObjects.dir/requires

CMakeFiles/myObjects.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myObjects.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myObjects.dir/clean

CMakeFiles/myObjects.dir/depend:
	cd /home/caldor/gazetrack/MyAruco/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/caldor/gazetrack/MyAruco /home/caldor/gazetrack/MyAruco /home/caldor/gazetrack/MyAruco/build /home/caldor/gazetrack/MyAruco/build /home/caldor/gazetrack/MyAruco/build/CMakeFiles/myObjects.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myObjects.dir/depend

