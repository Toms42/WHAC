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
CMAKE_SOURCE_DIR = /home/matthew/build_ws/src/dep/libfreenect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matthew/build_ws/src/dep/libfreenect

# Include any dependencies generated for this target.
include wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/depend.make

# Include the progress variables for this target.
include wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/progress.make

# Include the compile flags for this target's objects.
include wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/flags.make

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o: wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/flags.make
wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o: wrappers/cpp/cpp_pc_view.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matthew/build_ws/src/dep/libfreenect/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o"
	cd /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o -c /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp/cpp_pc_view.cpp

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.i"
	cd /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp/cpp_pc_view.cpp > CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.i

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.s"
	cd /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp/cpp_pc_view.cpp -o CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.s

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.requires:

.PHONY : wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.requires

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.provides: wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.requires
	$(MAKE) -f wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/build.make wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.provides.build
.PHONY : wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.provides

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.provides.build: wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o


# Object files for target freenect-cpp_pcview
freenect__cpp_pcview_OBJECTS = \
"CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o"

# External object files for target freenect-cpp_pcview
freenect__cpp_pcview_EXTERNAL_OBJECTS =

bin/freenect-cpp_pcview: wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o
bin/freenect-cpp_pcview: wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/build.make
bin/freenect-cpp_pcview: lib/libfreenect.so.0.6.0
bin/freenect-cpp_pcview: /usr/lib/x86_64-linux-gnu/libGL.so
bin/freenect-cpp_pcview: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/freenect-cpp_pcview: /usr/lib/x86_64-linux-gnu/libglut.so
bin/freenect-cpp_pcview: /usr/lib/x86_64-linux-gnu/libXmu.so
bin/freenect-cpp_pcview: /usr/lib/x86_64-linux-gnu/libXi.so
bin/freenect-cpp_pcview: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
bin/freenect-cpp_pcview: wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matthew/build_ws/src/dep/libfreenect/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/freenect-cpp_pcview"
	cd /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/freenect-cpp_pcview.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/build: bin/freenect-cpp_pcview

.PHONY : wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/build

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/requires: wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/cpp_pc_view.cpp.o.requires

.PHONY : wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/requires

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/clean:
	cd /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp && $(CMAKE_COMMAND) -P CMakeFiles/freenect-cpp_pcview.dir/cmake_clean.cmake
.PHONY : wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/clean

wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/depend:
	cd /home/matthew/build_ws/src/dep/libfreenect && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matthew/build_ws/src/dep/libfreenect /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp /home/matthew/build_ws/src/dep/libfreenect /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp /home/matthew/build_ws/src/dep/libfreenect/wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : wrappers/cpp/CMakeFiles/freenect-cpp_pcview.dir/depend

