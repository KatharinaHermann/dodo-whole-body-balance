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
CMAKE_SOURCE_DIR = /home/kathi/Documents/Dodo/dodo-whole-body-balance

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kathi/Documents/Dodo/dodo-whole-body-balance/build

# Include any dependencies generated for this target.
include CMakeFiles/balancerWrench.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/balancerWrench.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/balancerWrench.dir/flags.make

CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o: CMakeFiles/balancerWrench.dir/flags.make
CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o: ../sample/balancerWrench.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kathi/Documents/Dodo/dodo-whole-body-balance/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o -c /home/kathi/Documents/Dodo/dodo-whole-body-balance/sample/balancerWrench.cpp

CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kathi/Documents/Dodo/dodo-whole-body-balance/sample/balancerWrench.cpp > CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.i

CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kathi/Documents/Dodo/dodo-whole-body-balance/sample/balancerWrench.cpp -o CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.s

CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.requires:

.PHONY : CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.requires

CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.provides: CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.requires
	$(MAKE) -f CMakeFiles/balancerWrench.dir/build.make CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.provides.build
.PHONY : CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.provides

CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.provides.build: CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o


# Object files for target balancerWrench
balancerWrench_OBJECTS = \
"CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o"

# External object files for target balancerWrench
balancerWrench_EXTERNAL_OBJECTS =

balancerWrench: CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o
balancerWrench: CMakeFiles/balancerWrench.dir/build.make
balancerWrench: ../libraries/libmujoco200.so
balancerWrench: ../libraries/libglfw.so.3
balancerWrench: libeigenUtils.so
balancerWrench: CMakeFiles/balancerWrench.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kathi/Documents/Dodo/dodo-whole-body-balance/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable balancerWrench"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/balancerWrench.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/balancerWrench.dir/build: balancerWrench

.PHONY : CMakeFiles/balancerWrench.dir/build

CMakeFiles/balancerWrench.dir/requires: CMakeFiles/balancerWrench.dir/sample/balancerWrench.cpp.o.requires

.PHONY : CMakeFiles/balancerWrench.dir/requires

CMakeFiles/balancerWrench.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/balancerWrench.dir/cmake_clean.cmake
.PHONY : CMakeFiles/balancerWrench.dir/clean

CMakeFiles/balancerWrench.dir/depend:
	cd /home/kathi/Documents/Dodo/dodo-whole-body-balance/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kathi/Documents/Dodo/dodo-whole-body-balance /home/kathi/Documents/Dodo/dodo-whole-body-balance /home/kathi/Documents/Dodo/dodo-whole-body-balance/build /home/kathi/Documents/Dodo/dodo-whole-body-balance/build /home/kathi/Documents/Dodo/dodo-whole-body-balance/build/CMakeFiles/balancerWrench.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/balancerWrench.dir/depend

