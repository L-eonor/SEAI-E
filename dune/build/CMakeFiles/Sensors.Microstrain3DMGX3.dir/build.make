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
CMAKE_SOURCE_DIR = /home/leonor/Desktop/dune/source

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leonor/Desktop/dune/build

# Include any dependencies generated for this target.
include CMakeFiles/Sensors.Microstrain3DMGX3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Sensors.Microstrain3DMGX3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Sensors.Microstrain3DMGX3.dir/flags.make

CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o: CMakeFiles/Sensors.Microstrain3DMGX3.dir/flags.make
CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o: /home/leonor/Desktop/dune/source/src/Sensors/Microstrain3DMGX3/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Sensors::Microstrain3DMGX3::Task, SensorsMicrostrain3DMGX3Task)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o -c /home/leonor/Desktop/dune/source/src/Sensors/Microstrain3DMGX3/Task.cpp

CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Sensors::Microstrain3DMGX3::Task, SensorsMicrostrain3DMGX3Task)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/leonor/Desktop/dune/source/src/Sensors/Microstrain3DMGX3/Task.cpp > CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.i

CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Sensors::Microstrain3DMGX3::Task, SensorsMicrostrain3DMGX3Task)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/leonor/Desktop/dune/source/src/Sensors/Microstrain3DMGX3/Task.cpp -o CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.s

CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.requires:

.PHONY : CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.requires

CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.provides: CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.requires
	$(MAKE) -f CMakeFiles/Sensors.Microstrain3DMGX3.dir/build.make CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.provides.build
.PHONY : CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.provides

CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.provides.build: CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o


# Object files for target Sensors.Microstrain3DMGX3
Sensors_Microstrain3DMGX3_OBJECTS = \
"CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o"

# External object files for target Sensors.Microstrain3DMGX3
Sensors_Microstrain3DMGX3_EXTERNAL_OBJECTS =

libSensors.Microstrain3DMGX3.a: CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o
libSensors.Microstrain3DMGX3.a: CMakeFiles/Sensors.Microstrain3DMGX3.dir/build.make
libSensors.Microstrain3DMGX3.a: CMakeFiles/Sensors.Microstrain3DMGX3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libSensors.Microstrain3DMGX3.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Sensors.Microstrain3DMGX3.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sensors.Microstrain3DMGX3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Sensors.Microstrain3DMGX3.dir/build: libSensors.Microstrain3DMGX3.a

.PHONY : CMakeFiles/Sensors.Microstrain3DMGX3.dir/build

CMakeFiles/Sensors.Microstrain3DMGX3.dir/requires: CMakeFiles/Sensors.Microstrain3DMGX3.dir/src/Sensors/Microstrain3DMGX3/Task.cpp.o.requires

.PHONY : CMakeFiles/Sensors.Microstrain3DMGX3.dir/requires

CMakeFiles/Sensors.Microstrain3DMGX3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Sensors.Microstrain3DMGX3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Sensors.Microstrain3DMGX3.dir/clean

CMakeFiles/Sensors.Microstrain3DMGX3.dir/depend:
	cd /home/leonor/Desktop/dune/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build/CMakeFiles/Sensors.Microstrain3DMGX3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Sensors.Microstrain3DMGX3.dir/depend

