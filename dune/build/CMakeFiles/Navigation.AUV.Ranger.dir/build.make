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
include CMakeFiles/Navigation.AUV.Ranger.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Navigation.AUV.Ranger.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Navigation.AUV.Ranger.dir/flags.make

CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o: CMakeFiles/Navigation.AUV.Ranger.dir/flags.make
CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o: /home/leonor/Desktop/dune/source/src/Navigation/AUV/Ranger/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Navigation::AUV::Ranger::Task, NavigationAUVRangerTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o -c /home/leonor/Desktop/dune/source/src/Navigation/AUV/Ranger/Task.cpp

CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Navigation::AUV::Ranger::Task, NavigationAUVRangerTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/leonor/Desktop/dune/source/src/Navigation/AUV/Ranger/Task.cpp > CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.i

CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Navigation::AUV::Ranger::Task, NavigationAUVRangerTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/leonor/Desktop/dune/source/src/Navigation/AUV/Ranger/Task.cpp -o CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.s

CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.requires:

.PHONY : CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.requires

CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.provides: CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.requires
	$(MAKE) -f CMakeFiles/Navigation.AUV.Ranger.dir/build.make CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.provides.build
.PHONY : CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.provides

CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.provides.build: CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o


# Object files for target Navigation.AUV.Ranger
Navigation_AUV_Ranger_OBJECTS = \
"CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o"

# External object files for target Navigation.AUV.Ranger
Navigation_AUV_Ranger_EXTERNAL_OBJECTS =

libNavigation.AUV.Ranger.a: CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o
libNavigation.AUV.Ranger.a: CMakeFiles/Navigation.AUV.Ranger.dir/build.make
libNavigation.AUV.Ranger.a: CMakeFiles/Navigation.AUV.Ranger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libNavigation.AUV.Ranger.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Navigation.AUV.Ranger.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Navigation.AUV.Ranger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Navigation.AUV.Ranger.dir/build: libNavigation.AUV.Ranger.a

.PHONY : CMakeFiles/Navigation.AUV.Ranger.dir/build

CMakeFiles/Navigation.AUV.Ranger.dir/requires: CMakeFiles/Navigation.AUV.Ranger.dir/src/Navigation/AUV/Ranger/Task.cpp.o.requires

.PHONY : CMakeFiles/Navigation.AUV.Ranger.dir/requires

CMakeFiles/Navigation.AUV.Ranger.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Navigation.AUV.Ranger.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Navigation.AUV.Ranger.dir/clean

CMakeFiles/Navigation.AUV.Ranger.dir/depend:
	cd /home/leonor/Desktop/dune/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build/CMakeFiles/Navigation.AUV.Ranger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Navigation.AUV.Ranger.dir/depend

