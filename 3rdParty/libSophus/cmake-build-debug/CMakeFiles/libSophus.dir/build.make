# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /snap/clion/169/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/169/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hazyparker/project/my-research/3rdParty/libSophus

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/libSophus.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/libSophus.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/libSophus.dir/flags.make

CMakeFiles/libSophus.dir/main.cpp.o: CMakeFiles/libSophus.dir/flags.make
CMakeFiles/libSophus.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/libSophus.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libSophus.dir/main.cpp.o -c /home/hazyparker/project/my-research/3rdParty/libSophus/main.cpp

CMakeFiles/libSophus.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libSophus.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hazyparker/project/my-research/3rdParty/libSophus/main.cpp > CMakeFiles/libSophus.dir/main.cpp.i

CMakeFiles/libSophus.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libSophus.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hazyparker/project/my-research/3rdParty/libSophus/main.cpp -o CMakeFiles/libSophus.dir/main.cpp.s

CMakeFiles/libSophus.dir/useSophus.cpp.o: CMakeFiles/libSophus.dir/flags.make
CMakeFiles/libSophus.dir/useSophus.cpp.o: ../useSophus.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/libSophus.dir/useSophus.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libSophus.dir/useSophus.cpp.o -c /home/hazyparker/project/my-research/3rdParty/libSophus/useSophus.cpp

CMakeFiles/libSophus.dir/useSophus.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libSophus.dir/useSophus.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hazyparker/project/my-research/3rdParty/libSophus/useSophus.cpp > CMakeFiles/libSophus.dir/useSophus.cpp.i

CMakeFiles/libSophus.dir/useSophus.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libSophus.dir/useSophus.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hazyparker/project/my-research/3rdParty/libSophus/useSophus.cpp -o CMakeFiles/libSophus.dir/useSophus.cpp.s

# Object files for target libSophus
libSophus_OBJECTS = \
"CMakeFiles/libSophus.dir/main.cpp.o" \
"CMakeFiles/libSophus.dir/useSophus.cpp.o"

# External object files for target libSophus
libSophus_EXTERNAL_OBJECTS =

libSophus: CMakeFiles/libSophus.dir/main.cpp.o
libSophus: CMakeFiles/libSophus.dir/useSophus.cpp.o
libSophus: CMakeFiles/libSophus.dir/build.make
libSophus: CMakeFiles/libSophus.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable libSophus"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libSophus.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/libSophus.dir/build: libSophus
.PHONY : CMakeFiles/libSophus.dir/build

CMakeFiles/libSophus.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/libSophus.dir/cmake_clean.cmake
.PHONY : CMakeFiles/libSophus.dir/clean

CMakeFiles/libSophus.dir/depend:
	cd /home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hazyparker/project/my-research/3rdParty/libSophus /home/hazyparker/project/my-research/3rdParty/libSophus /home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug /home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug /home/hazyparker/project/my-research/3rdParty/libSophus/cmake-build-debug/CMakeFiles/libSophus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/libSophus.dir/depend

