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
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cpp.o -c /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/main.cpp

CMakeFiles/main.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/main.cpp > CMakeFiles/main.dir/main.cpp.i

CMakeFiles/main.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/main.cpp -o CMakeFiles/main.dir/main.cpp.s

CMakeFiles/main.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/main.cpp.o.requires

CMakeFiles/main.dir/main.cpp.o.provides: CMakeFiles/main.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/main.cpp.o.provides

CMakeFiles/main.dir/main.cpp.o.provides.build: CMakeFiles/main.dir/main.cpp.o


# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/main.cpp.o
main: CMakeFiles/main.dir/build.make
main: lib6_Map_Merging.so
main: /usr/local/lib/libopencv_gapi.so.4.5.3
main: /usr/local/lib/libopencv_highgui.so.4.5.3
main: /usr/local/lib/libopencv_ml.so.4.5.3
main: /usr/local/lib/libopencv_objdetect.so.4.5.3
main: /usr/local/lib/libopencv_photo.so.4.5.3
main: /usr/local/lib/libopencv_stitching.so.4.5.3
main: /usr/local/lib/libopencv_video.so.4.5.3
main: /usr/local/lib/libopencv_calib3d.so.4.5.3
main: /usr/local/lib/libopencv_dnn.so.4.5.3
main: /usr/local/lib/libopencv_features2d.so.4.5.3
main: /usr/local/lib/libopencv_flann.so.4.5.3
main: /usr/local/lib/libopencv_videoio.so.4.5.3
main: /usr/local/lib/libopencv_imgcodecs.so.4.5.3
main: /usr/local/lib/libopencv_imgproc.so.4.5.3
main: /usr/local/lib/libopencv_core.so.4.5.3
main: /opt/ros/melodic/lib/librosbag.so
main: /opt/ros/melodic/lib/librosbag_storage.so
main: /opt/ros/melodic/lib/libclass_loader.so
main: /usr/lib/libPocoFoundation.so
main: /usr/lib/x86_64-linux-gnu/libdl.so
main: /opt/ros/melodic/lib/libroslib.so
main: /opt/ros/melodic/lib/librospack.so
main: /usr/lib/x86_64-linux-gnu/libpython2.7.so
main: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
main: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
main: /opt/ros/melodic/lib/libroslz4.so
main: /usr/lib/x86_64-linux-gnu/liblz4.so
main: /opt/ros/melodic/lib/libtopic_tools.so
main: /opt/ros/melodic/lib/libroscpp.so
main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
main: /opt/ros/melodic/lib/libxmlrpcpp.so
main: /opt/ros/melodic/lib/libcv_bridge.so
main: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
main: /opt/ros/melodic/lib/librosconsole.so
main: /opt/ros/melodic/lib/librosconsole_log4cxx.so
main: /opt/ros/melodic/lib/librosconsole_backend_interface.so
main: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
main: /usr/lib/x86_64-linux-gnu/libboost_regex.so
main: /opt/ros/melodic/lib/libroscpp_serialization.so
main: /opt/ros/melodic/lib/librostime.so
main: /opt/ros/melodic/lib/libcpp_common.so
main: /usr/lib/x86_64-linux-gnu/libboost_system.so
main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
main: /usr/lib/x86_64-linux-gnu/libpthread.so
main: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
main: /home/hazyparker/下载/Pangolin-master/build/libpango_glgeometry.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_geometry.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_plot.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_python.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_scene.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_tools.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_display.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_vars.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_video.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_packetstream.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_windowing.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_opengl.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_image.so
main: /home/hazyparker/下载/Pangolin-master/build/libpango_core.so
main: /usr/lib/x86_64-linux-gnu/libGLEW.so
main: /usr/lib/x86_64-linux-gnu/libOpenGL.so
main: /usr/lib/x86_64-linux-gnu/libGLX.so
main: /usr/lib/x86_64-linux-gnu/libGLU.so
main: /home/hazyparker/下载/Pangolin-master/build/libtinyobj.so
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/requires: CMakeFiles/main.dir/main.cpp.o.requires

.PHONY : CMakeFiles/main.dir/requires

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src /home/hazyparker/project/Multi_UAVs_Collaborative_SLAM/6_Map_Merging/src/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

