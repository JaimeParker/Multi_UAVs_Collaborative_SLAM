# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src

# Include any dependencies generated for this target.
include CMakeFiles/keyboard.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/keyboard.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/keyboard.dir/flags.make

CMakeFiles/keyboard.dir/GetKeyBoard.cc.o: CMakeFiles/keyboard.dir/flags.make
CMakeFiles/keyboard.dir/GetKeyBoard.cc.o: GetKeyBoard.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/keyboard.dir/GetKeyBoard.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/GetKeyBoard.cc.o -c /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/GetKeyBoard.cc

CMakeFiles/keyboard.dir/GetKeyBoard.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/GetKeyBoard.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/GetKeyBoard.cc > CMakeFiles/keyboard.dir/GetKeyBoard.cc.i

CMakeFiles/keyboard.dir/GetKeyBoard.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/GetKeyBoard.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/GetKeyBoard.cc -o CMakeFiles/keyboard.dir/GetKeyBoard.cc.s

CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.requires:

.PHONY : CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.requires

CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.provides: CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.requires
	$(MAKE) -f CMakeFiles/keyboard.dir/build.make CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.provides.build
.PHONY : CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.provides

CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.provides.build: CMakeFiles/keyboard.dir/GetKeyBoard.cc.o


# Object files for target keyboard
keyboard_OBJECTS = \
"CMakeFiles/keyboard.dir/GetKeyBoard.cc.o"

# External object files for target keyboard
keyboard_EXTERNAL_OBJECTS =

keyboard: CMakeFiles/keyboard.dir/GetKeyBoard.cc.o
keyboard: CMakeFiles/keyboard.dir/build.make
keyboard: CMakeFiles/keyboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable keyboard"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keyboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/keyboard.dir/build: keyboard

.PHONY : CMakeFiles/keyboard.dir/build

CMakeFiles/keyboard.dir/requires: CMakeFiles/keyboard.dir/GetKeyBoard.cc.o.requires

.PHONY : CMakeFiles/keyboard.dir/requires

CMakeFiles/keyboard.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/keyboard.dir/cmake_clean.cmake
.PHONY : CMakeFiles/keyboard.dir/clean

CMakeFiles/keyboard.dir/depend:
	cd /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles/keyboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/keyboard.dir/depend

