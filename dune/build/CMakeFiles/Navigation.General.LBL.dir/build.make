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
include CMakeFiles/Navigation.General.LBL.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Navigation.General.LBL.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Navigation.General.LBL.dir/flags.make

CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o: CMakeFiles/Navigation.General.LBL.dir/flags.make
CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o: /home/leonor/Desktop/dune/source/src/Navigation/General/LBL/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Navigation::General::LBL::Task, NavigationGeneralLBLTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o -c /home/leonor/Desktop/dune/source/src/Navigation/General/LBL/Task.cpp

CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Navigation::General::LBL::Task, NavigationGeneralLBLTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/leonor/Desktop/dune/source/src/Navigation/General/LBL/Task.cpp > CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.i

CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Navigation::General::LBL::Task, NavigationGeneralLBLTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/leonor/Desktop/dune/source/src/Navigation/General/LBL/Task.cpp -o CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.s

CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.requires:

.PHONY : CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.requires

CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.provides: CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.requires
	$(MAKE) -f CMakeFiles/Navigation.General.LBL.dir/build.make CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.provides.build
.PHONY : CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.provides

CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.provides.build: CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o


# Object files for target Navigation.General.LBL
Navigation_General_LBL_OBJECTS = \
"CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o"

# External object files for target Navigation.General.LBL
Navigation_General_LBL_EXTERNAL_OBJECTS =

libNavigation.General.LBL.a: CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o
libNavigation.General.LBL.a: CMakeFiles/Navigation.General.LBL.dir/build.make
libNavigation.General.LBL.a: CMakeFiles/Navigation.General.LBL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonor/Desktop/dune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libNavigation.General.LBL.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Navigation.General.LBL.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Navigation.General.LBL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Navigation.General.LBL.dir/build: libNavigation.General.LBL.a

.PHONY : CMakeFiles/Navigation.General.LBL.dir/build

CMakeFiles/Navigation.General.LBL.dir/requires: CMakeFiles/Navigation.General.LBL.dir/src/Navigation/General/LBL/Task.cpp.o.requires

.PHONY : CMakeFiles/Navigation.General.LBL.dir/requires

CMakeFiles/Navigation.General.LBL.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Navigation.General.LBL.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Navigation.General.LBL.dir/clean

CMakeFiles/Navigation.General.LBL.dir/depend:
	cd /home/leonor/Desktop/dune/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/source /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build /home/leonor/Desktop/dune/build/CMakeFiles/Navigation.General.LBL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Navigation.General.LBL.dir/depend

