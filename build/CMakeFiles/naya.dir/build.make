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
include CMakeFiles/naya.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/naya.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/naya.dir/flags.make

CMakeFiles/naya.dir/main_2dof.cpp.o: CMakeFiles/naya.dir/flags.make
CMakeFiles/naya.dir/main_2dof.cpp.o: ../main_2dof.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/naya.dir/main_2dof.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/naya.dir/main_2dof.cpp.o -c /home/rodrigolinhares/code/naya/main_2dof.cpp

CMakeFiles/naya.dir/main_2dof.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/naya.dir/main_2dof.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/main_2dof.cpp > CMakeFiles/naya.dir/main_2dof.cpp.i

CMakeFiles/naya.dir/main_2dof.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/naya.dir/main_2dof.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/main_2dof.cpp -o CMakeFiles/naya.dir/main_2dof.cpp.s

CMakeFiles/naya.dir/main_2dof.cpp.o.requires:
.PHONY : CMakeFiles/naya.dir/main_2dof.cpp.o.requires

CMakeFiles/naya.dir/main_2dof.cpp.o.provides: CMakeFiles/naya.dir/main_2dof.cpp.o.requires
	$(MAKE) -f CMakeFiles/naya.dir/build.make CMakeFiles/naya.dir/main_2dof.cpp.o.provides.build
.PHONY : CMakeFiles/naya.dir/main_2dof.cpp.o.provides

CMakeFiles/naya.dir/main_2dof.cpp.o.provides.build: CMakeFiles/naya.dir/main_2dof.cpp.o

CMakeFiles/naya.dir/nayaActive.cpp.o: CMakeFiles/naya.dir/flags.make
CMakeFiles/naya.dir/nayaActive.cpp.o: ../nayaActive.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/naya.dir/nayaActive.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/naya.dir/nayaActive.cpp.o -c /home/rodrigolinhares/code/naya/nayaActive.cpp

CMakeFiles/naya.dir/nayaActive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/naya.dir/nayaActive.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/nayaActive.cpp > CMakeFiles/naya.dir/nayaActive.cpp.i

CMakeFiles/naya.dir/nayaActive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/naya.dir/nayaActive.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/nayaActive.cpp -o CMakeFiles/naya.dir/nayaActive.cpp.s

CMakeFiles/naya.dir/nayaActive.cpp.o.requires:
.PHONY : CMakeFiles/naya.dir/nayaActive.cpp.o.requires

CMakeFiles/naya.dir/nayaActive.cpp.o.provides: CMakeFiles/naya.dir/nayaActive.cpp.o.requires
	$(MAKE) -f CMakeFiles/naya.dir/build.make CMakeFiles/naya.dir/nayaActive.cpp.o.provides.build
.PHONY : CMakeFiles/naya.dir/nayaActive.cpp.o.provides

CMakeFiles/naya.dir/nayaActive.cpp.o.provides.build: CMakeFiles/naya.dir/nayaActive.cpp.o

CMakeFiles/naya.dir/nayaSCV.cpp.o: CMakeFiles/naya.dir/flags.make
CMakeFiles/naya.dir/nayaSCV.cpp.o: ../nayaSCV.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/naya.dir/nayaSCV.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/naya.dir/nayaSCV.cpp.o -c /home/rodrigolinhares/code/naya/nayaSCV.cpp

CMakeFiles/naya.dir/nayaSCV.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/naya.dir/nayaSCV.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/nayaSCV.cpp > CMakeFiles/naya.dir/nayaSCV.cpp.i

CMakeFiles/naya.dir/nayaSCV.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/naya.dir/nayaSCV.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/nayaSCV.cpp -o CMakeFiles/naya.dir/nayaSCV.cpp.s

CMakeFiles/naya.dir/nayaSCV.cpp.o.requires:
.PHONY : CMakeFiles/naya.dir/nayaSCV.cpp.o.requires

CMakeFiles/naya.dir/nayaSCV.cpp.o.provides: CMakeFiles/naya.dir/nayaSCV.cpp.o.requires
	$(MAKE) -f CMakeFiles/naya.dir/build.make CMakeFiles/naya.dir/nayaSCV.cpp.o.provides.build
.PHONY : CMakeFiles/naya.dir/nayaSCV.cpp.o.provides

CMakeFiles/naya.dir/nayaSCV.cpp.o.provides.build: CMakeFiles/naya.dir/nayaSCV.cpp.o

CMakeFiles/naya.dir/tracking_aux.cpp.o: CMakeFiles/naya.dir/flags.make
CMakeFiles/naya.dir/tracking_aux.cpp.o: ../tracking_aux.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/naya.dir/tracking_aux.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/naya.dir/tracking_aux.cpp.o -c /home/rodrigolinhares/code/naya/tracking_aux.cpp

CMakeFiles/naya.dir/tracking_aux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/naya.dir/tracking_aux.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/tracking_aux.cpp > CMakeFiles/naya.dir/tracking_aux.cpp.i

CMakeFiles/naya.dir/tracking_aux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/naya.dir/tracking_aux.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/tracking_aux.cpp -o CMakeFiles/naya.dir/tracking_aux.cpp.s

CMakeFiles/naya.dir/tracking_aux.cpp.o.requires:
.PHONY : CMakeFiles/naya.dir/tracking_aux.cpp.o.requires

CMakeFiles/naya.dir/tracking_aux.cpp.o.provides: CMakeFiles/naya.dir/tracking_aux.cpp.o.requires
	$(MAKE) -f CMakeFiles/naya.dir/build.make CMakeFiles/naya.dir/tracking_aux.cpp.o.provides.build
.PHONY : CMakeFiles/naya.dir/tracking_aux.cpp.o.provides

CMakeFiles/naya.dir/tracking_aux.cpp.o.provides.build: CMakeFiles/naya.dir/tracking_aux.cpp.o

# Object files for target naya
naya_OBJECTS = \
"CMakeFiles/naya.dir/main_2dof.cpp.o" \
"CMakeFiles/naya.dir/nayaActive.cpp.o" \
"CMakeFiles/naya.dir/nayaSCV.cpp.o" \
"CMakeFiles/naya.dir/tracking_aux.cpp.o"

# External object files for target naya
naya_EXTERNAL_OBJECTS =

naya: CMakeFiles/naya.dir/main_2dof.cpp.o
naya: CMakeFiles/naya.dir/nayaActive.cpp.o
naya: CMakeFiles/naya.dir/nayaSCV.cpp.o
naya: CMakeFiles/naya.dir/tracking_aux.cpp.o
naya: CMakeFiles/naya.dir/build.make
naya: /usr/local/lib/libopencv_calib3d.so
naya: /usr/local/lib/libopencv_contrib.so
naya: /usr/local/lib/libopencv_core.so
naya: /usr/local/lib/libopencv_features2d.so
naya: /usr/local/lib/libopencv_flann.so
naya: /usr/local/lib/libopencv_gpu.so
naya: /usr/local/lib/libopencv_highgui.so
naya: /usr/local/lib/libopencv_imgproc.so
naya: /usr/local/lib/libopencv_legacy.so
naya: /usr/local/lib/libopencv_ml.so
naya: /usr/local/lib/libopencv_nonfree.so
naya: /usr/local/lib/libopencv_objdetect.so
naya: /usr/local/lib/libopencv_photo.so
naya: /usr/local/lib/libopencv_stitching.so
naya: /usr/local/lib/libopencv_superres.so
naya: /usr/local/lib/libopencv_ts.so
naya: /usr/local/lib/libopencv_video.so
naya: /usr/local/lib/libopencv_videostab.so
naya: CMakeFiles/naya.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable naya"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/naya.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/naya.dir/build: naya
.PHONY : CMakeFiles/naya.dir/build

CMakeFiles/naya.dir/requires: CMakeFiles/naya.dir/main_2dof.cpp.o.requires
CMakeFiles/naya.dir/requires: CMakeFiles/naya.dir/nayaActive.cpp.o.requires
CMakeFiles/naya.dir/requires: CMakeFiles/naya.dir/nayaSCV.cpp.o.requires
CMakeFiles/naya.dir/requires: CMakeFiles/naya.dir/tracking_aux.cpp.o.requires
.PHONY : CMakeFiles/naya.dir/requires

CMakeFiles/naya.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/naya.dir/cmake_clean.cmake
.PHONY : CMakeFiles/naya.dir/clean

CMakeFiles/naya.dir/depend:
	cd /home/rodrigolinhares/code/naya/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rodrigolinhares/code/naya /home/rodrigolinhares/code/naya /home/rodrigolinhares/code/naya/build /home/rodrigolinhares/code/naya/build /home/rodrigolinhares/code/naya/build/CMakeFiles/naya.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/naya.dir/depend

