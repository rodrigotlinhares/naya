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
include CMakeFiles/libs.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/libs.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/libs.dir/flags.make

CMakeFiles/libs.dir/main_6dof.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/main_6dof.cpp.o: ../main_6dof.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/main_6dof.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/main_6dof.cpp.o -c /home/rodrigolinhares/code/naya/main_6dof.cpp

CMakeFiles/libs.dir/main_6dof.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/main_6dof.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/main_6dof.cpp > CMakeFiles/libs.dir/main_6dof.cpp.i

CMakeFiles/libs.dir/main_6dof.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/main_6dof.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/main_6dof.cpp -o CMakeFiles/libs.dir/main_6dof.cpp.s

CMakeFiles/libs.dir/main_6dof.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/main_6dof.cpp.o.requires

CMakeFiles/libs.dir/main_6dof.cpp.o.provides: CMakeFiles/libs.dir/main_6dof.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/main_6dof.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/main_6dof.cpp.o.provides

CMakeFiles/libs.dir/main_6dof.cpp.o.provides.build: CMakeFiles/libs.dir/main_6dof.cpp.o

CMakeFiles/libs.dir/main_tps.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/main_tps.cpp.o: ../main_tps.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/main_tps.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/main_tps.cpp.o -c /home/rodrigolinhares/code/naya/main_tps.cpp

CMakeFiles/libs.dir/main_tps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/main_tps.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/main_tps.cpp > CMakeFiles/libs.dir/main_tps.cpp.i

CMakeFiles/libs.dir/main_tps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/main_tps.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/main_tps.cpp -o CMakeFiles/libs.dir/main_tps.cpp.s

CMakeFiles/libs.dir/main_tps.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/main_tps.cpp.o.requires

CMakeFiles/libs.dir/main_tps.cpp.o.provides: CMakeFiles/libs.dir/main_tps.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/main_tps.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/main_tps.cpp.o.provides

CMakeFiles/libs.dir/main_tps.cpp.o.provides.build: CMakeFiles/libs.dir/main_tps.cpp.o

CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o: CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o -c /home/rodrigolinhares/code/naya/build/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp

CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/build/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp > CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.i

CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/build/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp -o CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.s

CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.requires

CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.provides: CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.provides

CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.provides.build: CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o

CMakeFiles/libs.dir/nayaActive.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/nayaActive.cpp.o: ../nayaActive.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/nayaActive.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/nayaActive.cpp.o -c /home/rodrigolinhares/code/naya/nayaActive.cpp

CMakeFiles/libs.dir/nayaActive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/nayaActive.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/nayaActive.cpp > CMakeFiles/libs.dir/nayaActive.cpp.i

CMakeFiles/libs.dir/nayaActive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/nayaActive.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/nayaActive.cpp -o CMakeFiles/libs.dir/nayaActive.cpp.s

CMakeFiles/libs.dir/nayaActive.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/nayaActive.cpp.o.requires

CMakeFiles/libs.dir/nayaActive.cpp.o.provides: CMakeFiles/libs.dir/nayaActive.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/nayaActive.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/nayaActive.cpp.o.provides

CMakeFiles/libs.dir/nayaActive.cpp.o.provides.build: CMakeFiles/libs.dir/nayaActive.cpp.o

CMakeFiles/libs.dir/main_8dof.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/main_8dof.cpp.o: ../main_8dof.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/main_8dof.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/main_8dof.cpp.o -c /home/rodrigolinhares/code/naya/main_8dof.cpp

CMakeFiles/libs.dir/main_8dof.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/main_8dof.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/main_8dof.cpp > CMakeFiles/libs.dir/main_8dof.cpp.i

CMakeFiles/libs.dir/main_8dof.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/main_8dof.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/main_8dof.cpp -o CMakeFiles/libs.dir/main_8dof.cpp.s

CMakeFiles/libs.dir/main_8dof.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/main_8dof.cpp.o.requires

CMakeFiles/libs.dir/main_8dof.cpp.o.provides: CMakeFiles/libs.dir/main_8dof.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/main_8dof.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/main_8dof.cpp.o.provides

CMakeFiles/libs.dir/main_8dof.cpp.o.provides.build: CMakeFiles/libs.dir/main_8dof.cpp.o

CMakeFiles/libs.dir/main_2dof.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/main_2dof.cpp.o: ../main_2dof.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/main_2dof.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/main_2dof.cpp.o -c /home/rodrigolinhares/code/naya/main_2dof.cpp

CMakeFiles/libs.dir/main_2dof.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/main_2dof.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/main_2dof.cpp > CMakeFiles/libs.dir/main_2dof.cpp.i

CMakeFiles/libs.dir/main_2dof.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/main_2dof.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/main_2dof.cpp -o CMakeFiles/libs.dir/main_2dof.cpp.s

CMakeFiles/libs.dir/main_2dof.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/main_2dof.cpp.o.requires

CMakeFiles/libs.dir/main_2dof.cpp.o.provides: CMakeFiles/libs.dir/main_2dof.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/main_2dof.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/main_2dof.cpp.o.provides

CMakeFiles/libs.dir/main_2dof.cpp.o.provides.build: CMakeFiles/libs.dir/main_2dof.cpp.o

CMakeFiles/libs.dir/nayaSCV.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/nayaSCV.cpp.o: ../nayaSCV.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/nayaSCV.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/nayaSCV.cpp.o -c /home/rodrigolinhares/code/naya/nayaSCV.cpp

CMakeFiles/libs.dir/nayaSCV.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/nayaSCV.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/nayaSCV.cpp > CMakeFiles/libs.dir/nayaSCV.cpp.i

CMakeFiles/libs.dir/nayaSCV.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/nayaSCV.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/nayaSCV.cpp -o CMakeFiles/libs.dir/nayaSCV.cpp.s

CMakeFiles/libs.dir/nayaSCV.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/nayaSCV.cpp.o.requires

CMakeFiles/libs.dir/nayaSCV.cpp.o.provides: CMakeFiles/libs.dir/nayaSCV.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/nayaSCV.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/nayaSCV.cpp.o.provides

CMakeFiles/libs.dir/nayaSCV.cpp.o.provides.build: CMakeFiles/libs.dir/nayaSCV.cpp.o

CMakeFiles/libs.dir/main_4dof.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/main_4dof.cpp.o: ../main_4dof.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/main_4dof.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/main_4dof.cpp.o -c /home/rodrigolinhares/code/naya/main_4dof.cpp

CMakeFiles/libs.dir/main_4dof.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/main_4dof.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/main_4dof.cpp > CMakeFiles/libs.dir/main_4dof.cpp.i

CMakeFiles/libs.dir/main_4dof.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/main_4dof.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/main_4dof.cpp -o CMakeFiles/libs.dir/main_4dof.cpp.s

CMakeFiles/libs.dir/main_4dof.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/main_4dof.cpp.o.requires

CMakeFiles/libs.dir/main_4dof.cpp.o.provides: CMakeFiles/libs.dir/main_4dof.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/main_4dof.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/main_4dof.cpp.o.provides

CMakeFiles/libs.dir/main_4dof.cpp.o.provides.build: CMakeFiles/libs.dir/main_4dof.cpp.o

CMakeFiles/libs.dir/tracking_aux.cpp.o: CMakeFiles/libs.dir/flags.make
CMakeFiles/libs.dir/tracking_aux.cpp.o: ../tracking_aux.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rodrigolinhares/code/naya/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/libs.dir/tracking_aux.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/libs.dir/tracking_aux.cpp.o -c /home/rodrigolinhares/code/naya/tracking_aux.cpp

CMakeFiles/libs.dir/tracking_aux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libs.dir/tracking_aux.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rodrigolinhares/code/naya/tracking_aux.cpp > CMakeFiles/libs.dir/tracking_aux.cpp.i

CMakeFiles/libs.dir/tracking_aux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libs.dir/tracking_aux.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rodrigolinhares/code/naya/tracking_aux.cpp -o CMakeFiles/libs.dir/tracking_aux.cpp.s

CMakeFiles/libs.dir/tracking_aux.cpp.o.requires:
.PHONY : CMakeFiles/libs.dir/tracking_aux.cpp.o.requires

CMakeFiles/libs.dir/tracking_aux.cpp.o.provides: CMakeFiles/libs.dir/tracking_aux.cpp.o.requires
	$(MAKE) -f CMakeFiles/libs.dir/build.make CMakeFiles/libs.dir/tracking_aux.cpp.o.provides.build
.PHONY : CMakeFiles/libs.dir/tracking_aux.cpp.o.provides

CMakeFiles/libs.dir/tracking_aux.cpp.o.provides.build: CMakeFiles/libs.dir/tracking_aux.cpp.o

# Object files for target libs
libs_OBJECTS = \
"CMakeFiles/libs.dir/main_6dof.cpp.o" \
"CMakeFiles/libs.dir/main_tps.cpp.o" \
"CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o" \
"CMakeFiles/libs.dir/nayaActive.cpp.o" \
"CMakeFiles/libs.dir/main_8dof.cpp.o" \
"CMakeFiles/libs.dir/main_2dof.cpp.o" \
"CMakeFiles/libs.dir/nayaSCV.cpp.o" \
"CMakeFiles/libs.dir/main_4dof.cpp.o" \
"CMakeFiles/libs.dir/tracking_aux.cpp.o"

# External object files for target libs
libs_EXTERNAL_OBJECTS =

liblibs.a: CMakeFiles/libs.dir/main_6dof.cpp.o
liblibs.a: CMakeFiles/libs.dir/main_tps.cpp.o
liblibs.a: CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o
liblibs.a: CMakeFiles/libs.dir/nayaActive.cpp.o
liblibs.a: CMakeFiles/libs.dir/main_8dof.cpp.o
liblibs.a: CMakeFiles/libs.dir/main_2dof.cpp.o
liblibs.a: CMakeFiles/libs.dir/nayaSCV.cpp.o
liblibs.a: CMakeFiles/libs.dir/main_4dof.cpp.o
liblibs.a: CMakeFiles/libs.dir/tracking_aux.cpp.o
liblibs.a: CMakeFiles/libs.dir/build.make
liblibs.a: CMakeFiles/libs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library liblibs.a"
	$(CMAKE_COMMAND) -P CMakeFiles/libs.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/libs.dir/build: liblibs.a
.PHONY : CMakeFiles/libs.dir/build

CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/main_6dof.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/main_tps.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/CMakeFiles/2.8.11.2/CompilerIdCXX/CMakeCXXCompilerId.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/nayaActive.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/main_8dof.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/main_2dof.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/nayaSCV.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/main_4dof.cpp.o.requires
CMakeFiles/libs.dir/requires: CMakeFiles/libs.dir/tracking_aux.cpp.o.requires
.PHONY : CMakeFiles/libs.dir/requires

CMakeFiles/libs.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/libs.dir/cmake_clean.cmake
.PHONY : CMakeFiles/libs.dir/clean

CMakeFiles/libs.dir/depend:
	cd /home/rodrigolinhares/code/naya/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rodrigolinhares/code/naya /home/rodrigolinhares/code/naya /home/rodrigolinhares/code/naya/build /home/rodrigolinhares/code/naya/build /home/rodrigolinhares/code/naya/build/CMakeFiles/libs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/libs.dir/depend

