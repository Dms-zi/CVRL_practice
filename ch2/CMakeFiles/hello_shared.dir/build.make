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
CMAKE_SOURCE_DIR = /home/cvrl/ubuntu_20.04/vslam_ws

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cvrl/ubuntu_20.04/vslam_ws

# Include any dependencies generated for this target.
include ch2/CMakeFiles/hello_shared.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ch2/CMakeFiles/hello_shared.dir/compiler_depend.make

# Include the progress variables for this target.
include ch2/CMakeFiles/hello_shared.dir/progress.make

# Include the compile flags for this target's objects.
include ch2/CMakeFiles/hello_shared.dir/flags.make

ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o: ch2/CMakeFiles/hello_shared.dir/flags.make
ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o: ch2/libHelloSLAM.cpp
ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o: ch2/CMakeFiles/hello_shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cvrl/ubuntu_20.04/vslam_ws/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o"
	cd /home/cvrl/ubuntu_20.04/vslam_ws/ch2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o -MF CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o.d -o CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o -c /home/cvrl/ubuntu_20.04/vslam_ws/ch2/libHelloSLAM.cpp

ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.i"
	cd /home/cvrl/ubuntu_20.04/vslam_ws/ch2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cvrl/ubuntu_20.04/vslam_ws/ch2/libHelloSLAM.cpp > CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.i

ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.s"
	cd /home/cvrl/ubuntu_20.04/vslam_ws/ch2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cvrl/ubuntu_20.04/vslam_ws/ch2/libHelloSLAM.cpp -o CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.s

# Object files for target hello_shared
hello_shared_OBJECTS = \
"CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o"

# External object files for target hello_shared
hello_shared_EXTERNAL_OBJECTS =

ch2/libhello_shared.so: ch2/CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o
ch2/libhello_shared.so: ch2/CMakeFiles/hello_shared.dir/build.make
ch2/libhello_shared.so: ch2/CMakeFiles/hello_shared.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cvrl/ubuntu_20.04/vslam_ws/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libhello_shared.so"
	cd /home/cvrl/ubuntu_20.04/vslam_ws/ch2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hello_shared.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ch2/CMakeFiles/hello_shared.dir/build: ch2/libhello_shared.so
.PHONY : ch2/CMakeFiles/hello_shared.dir/build

ch2/CMakeFiles/hello_shared.dir/clean:
	cd /home/cvrl/ubuntu_20.04/vslam_ws/ch2 && $(CMAKE_COMMAND) -P CMakeFiles/hello_shared.dir/cmake_clean.cmake
.PHONY : ch2/CMakeFiles/hello_shared.dir/clean

ch2/CMakeFiles/hello_shared.dir/depend:
	cd /home/cvrl/ubuntu_20.04/vslam_ws && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cvrl/ubuntu_20.04/vslam_ws /home/cvrl/ubuntu_20.04/vslam_ws/ch2 /home/cvrl/ubuntu_20.04/vslam_ws /home/cvrl/ubuntu_20.04/vslam_ws/ch2 /home/cvrl/ubuntu_20.04/vslam_ws/ch2/CMakeFiles/hello_shared.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ch2/CMakeFiles/hello_shared.dir/depend

