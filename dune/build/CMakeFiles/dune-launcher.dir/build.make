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
include CMakeFiles/dune-launcher.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dune-launcher.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dune-launcher.dir/flags.make

CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o: CMakeFiles/dune-launcher.dir/flags.make
CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o: /home/leonor/Desktop/dune/source/src/Main/Launcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -o CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o -c /home/leonor/Desktop/dune/source/src/Main/Launcher.cpp

CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -E /home/leonor/Desktop/dune/source/src/Main/Launcher.cpp > CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.i

CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -S /home/leonor/Desktop/dune/source/src/Main/Launcher.cpp -o CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.s

CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.requires:

.PHONY : CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.requires

CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.provides: CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.requires
	$(MAKE) -f CMakeFiles/dune-launcher.dir/build.make CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.provides.build
.PHONY : CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.provides

CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.provides.build: CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o


# Object files for target dune-launcher
dune__launcher_OBJECTS = \
"CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o"

# External object files for target dune-launcher
dune__launcher_EXTERNAL_OBJECTS =

dune-launcher: CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o
dune-launcher: CMakeFiles/dune-launcher.dir/build.make
dune-launcher: libdune-core.a
dune-launcher: CMakeFiles/dune-launcher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable dune-launcher"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dune-launcher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dune-launcher.dir/build: dune-launcher

.PHONY : CMakeFiles/dune-launcher.dir/build

CMakeFiles/dune-launcher.dir/requires: CMakeFiles/dune-launcher.dir/src/Main/Launcher.cpp.o.requires

.PHONY : CMakeFiles/dune-launcher.dir/requires

CMakeFiles/dune-launcher.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dune-launcher.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dune-launcher.dir/clean

CMakeFiles/dune-launcher.dir/depend:
	cd /home/leonor/Desktop/dune/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build/CMakeFiles/dune-launcher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dune-launcher.dir/depend

