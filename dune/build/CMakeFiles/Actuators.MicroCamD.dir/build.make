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
include CMakeFiles/Actuators.MicroCamD.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Actuators.MicroCamD.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Actuators.MicroCamD.dir/flags.make

CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o: CMakeFiles/Actuators.MicroCamD.dir/flags.make
CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o: /home/leonor/Desktop/dune/source/src/Actuators/MicroCamD/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Actuators::MicroCamD::Task, ActuatorsMicroCamDTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o -c /home/leonor/Desktop/dune/source/src/Actuators/MicroCamD/Task.cpp

CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Actuators::MicroCamD::Task, ActuatorsMicroCamDTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/leonor/Desktop/dune/source/src/Actuators/MicroCamD/Task.cpp > CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.i

CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Actuators::MicroCamD::Task, ActuatorsMicroCamDTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/leonor/Desktop/dune/source/src/Actuators/MicroCamD/Task.cpp -o CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.s

CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.requires:

.PHONY : CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.requires

CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.provides: CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.requires
	$(MAKE) -f CMakeFiles/Actuators.MicroCamD.dir/build.make CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.provides.build
.PHONY : CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.provides

CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.provides.build: CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o


# Object files for target Actuators.MicroCamD
Actuators_MicroCamD_OBJECTS = \
"CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o"

# External object files for target Actuators.MicroCamD
Actuators_MicroCamD_EXTERNAL_OBJECTS =

libActuators.MicroCamD.a: CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o
libActuators.MicroCamD.a: CMakeFiles/Actuators.MicroCamD.dir/build.make
libActuators.MicroCamD.a: CMakeFiles/Actuators.MicroCamD.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libActuators.MicroCamD.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Actuators.MicroCamD.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Actuators.MicroCamD.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Actuators.MicroCamD.dir/build: libActuators.MicroCamD.a

.PHONY : CMakeFiles/Actuators.MicroCamD.dir/build

CMakeFiles/Actuators.MicroCamD.dir/requires: CMakeFiles/Actuators.MicroCamD.dir/src/Actuators/MicroCamD/Task.cpp.o.requires

.PHONY : CMakeFiles/Actuators.MicroCamD.dir/requires

CMakeFiles/Actuators.MicroCamD.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Actuators.MicroCamD.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Actuators.MicroCamD.dir/clean

CMakeFiles/Actuators.MicroCamD.dir/depend:
	cd /home/leonor/Desktop/dune/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build/CMakeFiles/Actuators.MicroCamD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Actuators.MicroCamD.dir/depend

