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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/michi/Dev/AccelSim_clean

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/michi/Dev/AccelSim_clean

# Include any dependencies generated for this target.
include CMakeFiles/AccelSim.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/AccelSim.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/AccelSim.dir/flags.make

CMakeFiles/AccelSim.dir/src/Body.cpp.o: CMakeFiles/AccelSim.dir/flags.make
CMakeFiles/AccelSim.dir/src/Body.cpp.o: src/Body.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michi/Dev/AccelSim_clean/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/AccelSim.dir/src/Body.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AccelSim.dir/src/Body.cpp.o -c /home/michi/Dev/AccelSim_clean/src/Body.cpp

CMakeFiles/AccelSim.dir/src/Body.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AccelSim.dir/src/Body.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michi/Dev/AccelSim_clean/src/Body.cpp > CMakeFiles/AccelSim.dir/src/Body.cpp.i

CMakeFiles/AccelSim.dir/src/Body.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AccelSim.dir/src/Body.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michi/Dev/AccelSim_clean/src/Body.cpp -o CMakeFiles/AccelSim.dir/src/Body.cpp.s

CMakeFiles/AccelSim.dir/src/Main.cpp.o: CMakeFiles/AccelSim.dir/flags.make
CMakeFiles/AccelSim.dir/src/Main.cpp.o: src/Main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michi/Dev/AccelSim_clean/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/AccelSim.dir/src/Main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AccelSim.dir/src/Main.cpp.o -c /home/michi/Dev/AccelSim_clean/src/Main.cpp

CMakeFiles/AccelSim.dir/src/Main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AccelSim.dir/src/Main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michi/Dev/AccelSim_clean/src/Main.cpp > CMakeFiles/AccelSim.dir/src/Main.cpp.i

CMakeFiles/AccelSim.dir/src/Main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AccelSim.dir/src/Main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michi/Dev/AccelSim_clean/src/Main.cpp -o CMakeFiles/AccelSim.dir/src/Main.cpp.s

# Object files for target AccelSim
AccelSim_OBJECTS = \
"CMakeFiles/AccelSim.dir/src/Body.cpp.o" \
"CMakeFiles/AccelSim.dir/src/Main.cpp.o"

# External object files for target AccelSim
AccelSim_EXTERNAL_OBJECTS =

AccelSim: CMakeFiles/AccelSim.dir/src/Body.cpp.o
AccelSim: CMakeFiles/AccelSim.dir/src/Main.cpp.o
AccelSim: CMakeFiles/AccelSim.dir/build.make
AccelSim: CMakeFiles/AccelSim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/michi/Dev/AccelSim_clean/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable AccelSim"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AccelSim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/AccelSim.dir/build: AccelSim

.PHONY : CMakeFiles/AccelSim.dir/build

CMakeFiles/AccelSim.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/AccelSim.dir/cmake_clean.cmake
.PHONY : CMakeFiles/AccelSim.dir/clean

CMakeFiles/AccelSim.dir/depend:
	cd /home/michi/Dev/AccelSim_clean && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/michi/Dev/AccelSim_clean /home/michi/Dev/AccelSim_clean /home/michi/Dev/AccelSim_clean /home/michi/Dev/AccelSim_clean /home/michi/Dev/AccelSim_clean/CMakeFiles/AccelSim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/AccelSim.dir/depend

