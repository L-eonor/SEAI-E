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
include CMakeFiles/Transports.Radio.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Transports.Radio.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Transports.Radio.dir/flags.make

CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o: CMakeFiles/Transports.Radio.dir/flags.make
CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o: /home/leonor/Desktop/dune/source/src/Transports/Radio/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Transports::Radio::Task, TransportsRadioTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o -c /home/leonor/Desktop/dune/source/src/Transports/Radio/Task.cpp

CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Transports::Radio::Task, TransportsRadioTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/leonor/Desktop/dune/source/src/Transports/Radio/Task.cpp > CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.i

CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Transports::Radio::Task, TransportsRadioTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/leonor/Desktop/dune/source/src/Transports/Radio/Task.cpp -o CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.s

CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.requires:

.PHONY : CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.requires

CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.provides: CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.requires
	$(MAKE) -f CMakeFiles/Transports.Radio.dir/build.make CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.provides.build
.PHONY : CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.provides

CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.provides.build: CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o


# Object files for target Transports.Radio
Transports_Radio_OBJECTS = \
"CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o"

# External object files for target Transports.Radio
Transports_Radio_EXTERNAL_OBJECTS =

libTransports.Radio.a: CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o
libTransports.Radio.a: CMakeFiles/Transports.Radio.dir/build.make
libTransports.Radio.a: CMakeFiles/Transports.Radio.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libTransports.Radio.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Transports.Radio.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Transports.Radio.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Transports.Radio.dir/build: libTransports.Radio.a

.PHONY : CMakeFiles/Transports.Radio.dir/build

CMakeFiles/Transports.Radio.dir/requires: CMakeFiles/Transports.Radio.dir/src/Transports/Radio/Task.cpp.o.requires

.PHONY : CMakeFiles/Transports.Radio.dir/requires

CMakeFiles/Transports.Radio.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Transports.Radio.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Transports.Radio.dir/clean

CMakeFiles/Transports.Radio.dir/depend:
	cd /home/leonor/Desktop/dune/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build/CMakeFiles/Transports.Radio.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Transports.Radio.dir/depend

