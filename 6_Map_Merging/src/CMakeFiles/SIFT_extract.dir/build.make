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
include CMakeFiles/SIFT_extract.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SIFT_extract.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SIFT_extract.dir/flags.make

CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o: CMakeFiles/SIFT_extract.dir/flags.make
CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o: SIFT_extract.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o -c /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/SIFT_extract.cpp

CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/SIFT_extract.cpp > CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.i

CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/SIFT_extract.cpp -o CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.s

CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.requires:

.PHONY : CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.requires

CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.provides: CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.requires
	$(MAKE) -f CMakeFiles/SIFT_extract.dir/build.make CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.provides.build
.PHONY : CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.provides

CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.provides.build: CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o


# Object files for target SIFT_extract
SIFT_extract_OBJECTS = \
"CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o"

# External object files for target SIFT_extract
SIFT_extract_EXTERNAL_OBJECTS =

SIFT_extract: CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o
SIFT_extract: CMakeFiles/SIFT_extract.dir/build.make
SIFT_extract: /usr/local/lib/libopencv_gapi.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_highgui.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_ml.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_objdetect.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_photo.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_stitching.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_video.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_videoio.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_dnn.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_imgcodecs.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_calib3d.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_features2d.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_flann.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_imgproc.so.4.5.3
SIFT_extract: /usr/local/lib/libopencv_core.so.4.5.3
SIFT_extract: CMakeFiles/SIFT_extract.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SIFT_extract"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SIFT_extract.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SIFT_extract.dir/build: SIFT_extract

.PHONY : CMakeFiles/SIFT_extract.dir/build

CMakeFiles/SIFT_extract.dir/requires: CMakeFiles/SIFT_extract.dir/SIFT_extract.cpp.o.requires

.PHONY : CMakeFiles/SIFT_extract.dir/requires

CMakeFiles/SIFT_extract.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SIFT_extract.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SIFT_extract.dir/clean

CMakeFiles/SIFT_extract.dir/depend:
	cd /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles/SIFT_extract.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SIFT_extract.dir/depend

