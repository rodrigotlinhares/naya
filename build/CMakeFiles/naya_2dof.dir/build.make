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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rodrigolinhares/code/naya

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rodrigolinhares/code/naya/build

# Include any dependencies generated for this target.
include CMakeFiles/naya_2dof.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/naya_2dof.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/naya_2dof.dir/flags.make

CMakeFiles/naya_2dof.dir/main_2dof.cpp.o: CMakeFiles/naya_2dof.dir/flags.make
CMakeFiles/naya_2dof.dir/main_2dof.cpp.o: ../main_2dof.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/naya_2dof.dir/main_2dof.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/naya_2dof.dir/main_2dof.cpp.o -c /home/rodrigolinhares/code/naya/main_2dof.cpp

CMakeFiles/naya_2dof.dir/main_2dof.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/naya_2dof.dir/main_2dof.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/main_2dof.cpp > CMakeFiles/naya_2dof.dir/main_2dof.cpp.i

CMakeFiles/naya_2dof.dir/main_2dof.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/naya_2dof.dir/main_2dof.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/main_2dof.cpp -o CMakeFiles/naya_2dof.dir/main_2dof.cpp.s

CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.requires:
.PHONY : CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.requires

CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.provides: CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.requires
	$(MAKE) -f CMakeFiles/naya_2dof.dir/build.make CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.provides.build
.PHONY : CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.provides

CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.provides.build: CMakeFiles/naya_2dof.dir/main_2dof.cpp.o

# Object files for target naya_2dof
naya_2dof_OBJECTS = \
"CMakeFiles/naya_2dof.dir/main_2dof.cpp.o"

# External object files for target naya_2dof
naya_2dof_EXTERNAL_OBJECTS =

naya_2dof: CMakeFiles/naya_2dof.dir/main_2dof.cpp.o
naya_2dof: CMakeFiles/naya_2dof.dir/build.make
naya_2dof: liblibs.a
naya_2dof: /usr/local/lib/libopencv_calib3d.so
naya_2dof: /usr/local/lib/libopencv_contrib.so
naya_2dof: /usr/local/lib/libopencv_core.so
naya_2dof: /usr/local/lib/libopencv_features2d.so
naya_2dof: /usr/local/lib/libopencv_flann.so
naya_2dof: /usr/local/lib/libopencv_gpu.so
naya_2dof: /usr/local/lib/libopencv_highgui.so
naya_2dof: /usr/local/lib/libopencv_imgproc.so
naya_2dof: /usr/local/lib/libopencv_legacy.so
naya_2dof: /usr/local/lib/libopencv_ml.so
naya_2dof: /usr/local/lib/libopencv_nonfree.so
naya_2dof: /usr/local/lib/libopencv_objdetect.so
naya_2dof: /usr/local/lib/libopencv_photo.so
naya_2dof: /usr/local/lib/libopencv_stitching.so
naya_2dof: /usr/local/lib/libopencv_superres.so
naya_2dof: /usr/local/lib/libopencv_ts.so
naya_2dof: /usr/local/lib/libopencv_video.so
naya_2dof: /usr/local/lib/libopencv_videostab.so
naya_2dof: CMakeFiles/naya_2dof.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable naya_2dof"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/naya_2dof.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/naya_2dof.dir/build: naya_2dof
.PHONY : CMakeFiles/naya_2dof.dir/build

CMakeFiles/naya_2dof.dir/requires: CMakeFiles/naya_2dof.dir/main_2dof.cpp.o.requires
.PHONY : CMakeFiles/naya_2dof.dir/requires

CMakeFiles/naya_2dof.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/naya_2dof.dir/cmake_clean.cmake
.PHONY : CMakeFiles/naya_2dof.dir/clean

CMakeFiles/naya_2dof.dir/depend:
	cd /home/rodrigolinhares/code/naya/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rodrigolinhares/code/naya /home/rodrigolinhares/code/naya /home/rodrigolinhares/code/naya/build /home/rodrigolinhares/code/naya/build /home/rodrigolinhares/code/naya/build/CMakeFiles/naya_2dof.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/naya_2dof.dir/depend

