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
include examples/CMakeFiles/freenect-camtest.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/freenect-camtest.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/freenect-camtest.dir/flags.make

examples/CMakeFiles/freenect-camtest.dir/camtest.c.o: examples/CMakeFiles/freenect-camtest.dir/flags.make
examples/CMakeFiles/freenect-camtest.dir/camtest.c.o: examples/camtest.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matthew/build_ws/src/dep/libfreenect/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/freenect-camtest.dir/camtest.c.o"
	cd /home/matthew/build_ws/src/dep/libfreenect/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freenect-camtest.dir/camtest.c.o   -c /home/matthew/build_ws/src/dep/libfreenect/examples/camtest.c

examples/CMakeFiles/freenect-camtest.dir/camtest.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freenect-camtest.dir/camtest.c.i"
	cd /home/matthew/build_ws/src/dep/libfreenect/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/matthew/build_ws/src/dep/libfreenect/examples/camtest.c > CMakeFiles/freenect-camtest.dir/camtest.c.i

examples/CMakeFiles/freenect-camtest.dir/camtest.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freenect-camtest.dir/camtest.c.s"
	cd /home/matthew/build_ws/src/dep/libfreenect/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/matthew/build_ws/src/dep/libfreenect/examples/camtest.c -o CMakeFiles/freenect-camtest.dir/camtest.c.s

examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.requires:

.PHONY : examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.requires

examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.provides: examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.requires
	$(MAKE) -f examples/CMakeFiles/freenect-camtest.dir/build.make examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.provides.build
.PHONY : examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.provides

examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.provides.build: examples/CMakeFiles/freenect-camtest.dir/camtest.c.o


# Object files for target freenect-camtest
freenect__camtest_OBJECTS = \
"CMakeFiles/freenect-camtest.dir/camtest.c.o"

# External object files for target freenect-camtest
freenect__camtest_EXTERNAL_OBJECTS =

bin/freenect-camtest: examples/CMakeFiles/freenect-camtest.dir/camtest.c.o
bin/freenect-camtest: examples/CMakeFiles/freenect-camtest.dir/build.make
bin/freenect-camtest: lib/libfreenect.so.0.6.0
bin/freenect-camtest: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
bin/freenect-camtest: examples/CMakeFiles/freenect-camtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matthew/build_ws/src/dep/libfreenect/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable ../bin/freenect-camtest"
	cd /home/matthew/build_ws/src/dep/libfreenect/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/freenect-camtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/freenect-camtest.dir/build: bin/freenect-camtest

.PHONY : examples/CMakeFiles/freenect-camtest.dir/build

examples/CMakeFiles/freenect-camtest.dir/requires: examples/CMakeFiles/freenect-camtest.dir/camtest.c.o.requires

.PHONY : examples/CMakeFiles/freenect-camtest.dir/requires

examples/CMakeFiles/freenect-camtest.dir/clean:
	cd /home/matthew/build_ws/src/dep/libfreenect/examples && $(CMAKE_COMMAND) -P CMakeFiles/freenect-camtest.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/freenect-camtest.dir/clean

examples/CMakeFiles/freenect-camtest.dir/depend:
	cd /home/matthew/build_ws/src/dep/libfreenect && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matthew/build_ws/src/dep/libfreenect /home/matthew/build_ws/src/dep/libfreenect/examples /home/matthew/build_ws/src/dep/libfreenect /home/matthew/build_ws/src/dep/libfreenect/examples /home/matthew/build_ws/src/dep/libfreenect/examples/CMakeFiles/freenect-camtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/freenect-camtest.dir/depend

