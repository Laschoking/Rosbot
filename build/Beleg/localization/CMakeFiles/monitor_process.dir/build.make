# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/kotname/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kotname/ros_ws/build

# Include any dependencies generated for this target.
include Beleg/localization/CMakeFiles/monitor_process.dir/depend.make

# Include the progress variables for this target.
include Beleg/localization/CMakeFiles/monitor_process.dir/progress.make

# Include the compile flags for this target's objects.
include Beleg/localization/CMakeFiles/monitor_process.dir/flags.make

Beleg/localization/CMakeFiles/monitor_process.dir/src/monitor_process.cpp.o: Beleg/localization/CMakeFiles/monitor_process.dir/flags.make
Beleg/localization/CMakeFiles/monitor_process.dir/src/monitor_process.cpp.o: /home/kotname/ros_ws/src/Beleg/localization/src/monitor_process.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kotname/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Beleg/localization/CMakeFiles/monitor_process.dir/src/monitor_process.cpp.o"
	cd /home/kotname/ros_ws/build/Beleg/localization && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/monitor_process.dir/src/monitor_process.cpp.o -c /home/kotname/ros_ws/src/Beleg/localization/src/monitor_process.cpp

Beleg/localization/CMakeFiles/monitor_process.dir/src/monitor_process.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/monitor_process.dir/src/monitor_process.cpp.i"
	cd /home/kotname/ros_ws/build/Beleg/localization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kotname/ros_ws/src/Beleg/localization/src/monitor_process.cpp > CMakeFiles/monitor_process.dir/src/monitor_process.cpp.i

Beleg/localization/CMakeFiles/monitor_process.dir/src/monitor_process.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/monitor_process.dir/src/monitor_process.cpp.s"
	cd /home/kotname/ros_ws/build/Beleg/localization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kotname/ros_ws/src/Beleg/localization/src/monitor_process.cpp -o CMakeFiles/monitor_process.dir/src/monitor_process.cpp.s

# Object files for target monitor_process
monitor_process_OBJECTS = \
"CMakeFiles/monitor_process.dir/src/monitor_process.cpp.o"

# External object files for target monitor_process
monitor_process_EXTERNAL_OBJECTS =

/home/kotname/ros_ws/devel/lib/libmonitor_process.so: Beleg/localization/CMakeFiles/monitor_process.dir/src/monitor_process.cpp.o
/home/kotname/ros_ws/devel/lib/libmonitor_process.so: Beleg/localization/CMakeFiles/monitor_process.dir/build.make
/home/kotname/ros_ws/devel/lib/libmonitor_process.so: Beleg/localization/CMakeFiles/monitor_process.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kotname/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/kotname/ros_ws/devel/lib/libmonitor_process.so"
	cd /home/kotname/ros_ws/build/Beleg/localization && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/monitor_process.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Beleg/localization/CMakeFiles/monitor_process.dir/build: /home/kotname/ros_ws/devel/lib/libmonitor_process.so

.PHONY : Beleg/localization/CMakeFiles/monitor_process.dir/build

Beleg/localization/CMakeFiles/monitor_process.dir/clean:
	cd /home/kotname/ros_ws/build/Beleg/localization && $(CMAKE_COMMAND) -P CMakeFiles/monitor_process.dir/cmake_clean.cmake
.PHONY : Beleg/localization/CMakeFiles/monitor_process.dir/clean

Beleg/localization/CMakeFiles/monitor_process.dir/depend:
	cd /home/kotname/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kotname/ros_ws/src /home/kotname/ros_ws/src/Beleg/localization /home/kotname/ros_ws/build /home/kotname/ros_ws/build/Beleg/localization /home/kotname/ros_ws/build/Beleg/localization/CMakeFiles/monitor_process.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Beleg/localization/CMakeFiles/monitor_process.dir/depend
