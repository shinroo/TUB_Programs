# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/liliac/lilac/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liliac/lilac/build

# Include any dependencies generated for this target.
include lilac_fundamentals/CMakeFiles/playSong.dir/depend.make

# Include the progress variables for this target.
include lilac_fundamentals/CMakeFiles/playSong.dir/progress.make

# Include the compile flags for this target's objects.
include lilac_fundamentals/CMakeFiles/playSong.dir/flags.make

lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o: lilac_fundamentals/CMakeFiles/playSong.dir/flags.make
lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o: /home/liliac/lilac/src/lilac_fundamentals/src/playSong.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/liliac/lilac/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o"
	cd /home/liliac/lilac/build/lilac_fundamentals && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/playSong.dir/src/playSong.cpp.o -c /home/liliac/lilac/src/lilac_fundamentals/src/playSong.cpp

lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/playSong.dir/src/playSong.cpp.i"
	cd /home/liliac/lilac/build/lilac_fundamentals && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/liliac/lilac/src/lilac_fundamentals/src/playSong.cpp > CMakeFiles/playSong.dir/src/playSong.cpp.i

lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/playSong.dir/src/playSong.cpp.s"
	cd /home/liliac/lilac/build/lilac_fundamentals && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/liliac/lilac/src/lilac_fundamentals/src/playSong.cpp -o CMakeFiles/playSong.dir/src/playSong.cpp.s

lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.requires:
.PHONY : lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.requires

lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.provides: lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.requires
	$(MAKE) -f lilac_fundamentals/CMakeFiles/playSong.dir/build.make lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.provides.build
.PHONY : lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.provides

lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.provides.build: lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o

# Object files for target playSong
playSong_OBJECTS = \
"CMakeFiles/playSong.dir/src/playSong.cpp.o"

# External object files for target playSong
playSong_EXTERNAL_OBJECTS =

/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: lilac_fundamentals/CMakeFiles/playSong.dir/build.make
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/libroscpp.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libboost_signals.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/librosconsole.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/liblog4cxx.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libboost_regex.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/librostime.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libboost_date_time.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /opt/ros/indigo/lib/libcpp_common.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libboost_system.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libboost_thread.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libpthread.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/home/liliac/lilac/devel/lib/lilac_fundamentals/playSong: lilac_fundamentals/CMakeFiles/playSong.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/liliac/lilac/devel/lib/lilac_fundamentals/playSong"
	cd /home/liliac/lilac/build/lilac_fundamentals && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/playSong.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lilac_fundamentals/CMakeFiles/playSong.dir/build: /home/liliac/lilac/devel/lib/lilac_fundamentals/playSong
.PHONY : lilac_fundamentals/CMakeFiles/playSong.dir/build

lilac_fundamentals/CMakeFiles/playSong.dir/requires: lilac_fundamentals/CMakeFiles/playSong.dir/src/playSong.cpp.o.requires
.PHONY : lilac_fundamentals/CMakeFiles/playSong.dir/requires

lilac_fundamentals/CMakeFiles/playSong.dir/clean:
	cd /home/liliac/lilac/build/lilac_fundamentals && $(CMAKE_COMMAND) -P CMakeFiles/playSong.dir/cmake_clean.cmake
.PHONY : lilac_fundamentals/CMakeFiles/playSong.dir/clean

lilac_fundamentals/CMakeFiles/playSong.dir/depend:
	cd /home/liliac/lilac/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liliac/lilac/src /home/liliac/lilac/src/lilac_fundamentals /home/liliac/lilac/build /home/liliac/lilac/build/lilac_fundamentals /home/liliac/lilac/build/lilac_fundamentals/CMakeFiles/playSong.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lilac_fundamentals/CMakeFiles/playSong.dir/depend

