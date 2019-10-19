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
CMAKE_BINARY_DIR = /home/matthew/build_ws/src/dep/libfreenect/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/freenect-wavrecord.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/freenect-wavrecord.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/freenect-wavrecord.dir/flags.make

examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o: examples/CMakeFiles/freenect-wavrecord.dir/flags.make
examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o: ../examples/wavrecord.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matthew/build_ws/src/dep/libfreenect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o"
	cd /home/matthew/build_ws/src/dep/libfreenect/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o   -c /home/matthew/build_ws/src/dep/libfreenect/examples/wavrecord.c

examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freenect-wavrecord.dir/wavrecord.c.i"
	cd /home/matthew/build_ws/src/dep/libfreenect/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/matthew/build_ws/src/dep/libfreenect/examples/wavrecord.c > CMakeFiles/freenect-wavrecord.dir/wavrecord.c.i

examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freenect-wavrecord.dir/wavrecord.c.s"
	cd /home/matthew/build_ws/src/dep/libfreenect/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/matthew/build_ws/src/dep/libfreenect/examples/wavrecord.c -o CMakeFiles/freenect-wavrecord.dir/wavrecord.c.s

examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.requires:

.PHONY : examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.requires

examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.provides: examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.requires
	$(MAKE) -f examples/CMakeFiles/freenect-wavrecord.dir/build.make examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.provides.build
.PHONY : examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.provides

examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.provides.build: examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o


# Object files for target freenect-wavrecord
freenect__wavrecord_OBJECTS = \
"CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o"

# External object files for target freenect-wavrecord
freenect__wavrecord_EXTERNAL_OBJECTS =

bin/freenect-wavrecord: examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o
bin/freenect-wavrecord: examples/CMakeFiles/freenect-wavrecord.dir/build.make
bin/freenect-wavrecord: lib/libfreenect.so.0.6.0
bin/freenect-wavrecord: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
bin/freenect-wavrecord: examples/CMakeFiles/freenect-wavrecord.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matthew/build_ws/src/dep/libfreenect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable ../bin/freenect-wavrecord"
	cd /home/matthew/build_ws/src/dep/libfreenect/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/freenect-wavrecord.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/freenect-wavrecord.dir/build: bin/freenect-wavrecord

.PHONY : examples/CMakeFiles/freenect-wavrecord.dir/build

examples/CMakeFiles/freenect-wavrecord.dir/requires: examples/CMakeFiles/freenect-wavrecord.dir/wavrecord.c.o.requires

.PHONY : examples/CMakeFiles/freenect-wavrecord.dir/requires

examples/CMakeFiles/freenect-wavrecord.dir/clean:
	cd /home/matthew/build_ws/src/dep/libfreenect/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/freenect-wavrecord.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/freenect-wavrecord.dir/clean

examples/CMakeFiles/freenect-wavrecord.dir/depend:
	cd /home/matthew/build_ws/src/dep/libfreenect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matthew/build_ws/src/dep/libfreenect /home/matthew/build_ws/src/dep/libfreenect/examples /home/matthew/build_ws/src/dep/libfreenect/build /home/matthew/build_ws/src/dep/libfreenect/build/examples /home/matthew/build_ws/src/dep/libfreenect/build/examples/CMakeFiles/freenect-wavrecord.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/freenect-wavrecord.dir/depend

