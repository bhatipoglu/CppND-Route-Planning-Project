# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.28.1/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.28.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/test/utest_rp_a_star_search.cpp
CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o: CMakeFiles/test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o -MF CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o.d -o CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/test/utest_rp_a_star_search.cpp

CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/test/utest_rp_a_star_search.cpp > CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.i

CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/test/utest_rp_a_star_search.cpp -o CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.s

CMakeFiles/test.dir/src/route_planner.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/src/route_planner.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp
CMakeFiles/test.dir/src/route_planner.cpp.o: CMakeFiles/test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test.dir/src/route_planner.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test.dir/src/route_planner.cpp.o -MF CMakeFiles/test.dir/src/route_planner.cpp.o.d -o CMakeFiles/test.dir/src/route_planner.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp

CMakeFiles/test.dir/src/route_planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test.dir/src/route_planner.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp > CMakeFiles/test.dir/src/route_planner.cpp.i

CMakeFiles/test.dir/src/route_planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test.dir/src/route_planner.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp -o CMakeFiles/test.dir/src/route_planner.cpp.s

CMakeFiles/test.dir/src/model.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/src/model.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp
CMakeFiles/test.dir/src/model.cpp.o: CMakeFiles/test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test.dir/src/model.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test.dir/src/model.cpp.o -MF CMakeFiles/test.dir/src/model.cpp.o.d -o CMakeFiles/test.dir/src/model.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp

CMakeFiles/test.dir/src/model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test.dir/src/model.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp > CMakeFiles/test.dir/src/model.cpp.i

CMakeFiles/test.dir/src/model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test.dir/src/model.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp -o CMakeFiles/test.dir/src/model.cpp.s

CMakeFiles/test.dir/src/route_model.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/src/route_model.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp
CMakeFiles/test.dir/src/route_model.cpp.o: CMakeFiles/test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/test.dir/src/route_model.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test.dir/src/route_model.cpp.o -MF CMakeFiles/test.dir/src/route_model.cpp.o.d -o CMakeFiles/test.dir/src/route_model.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp

CMakeFiles/test.dir/src/route_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test.dir/src/route_model.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp > CMakeFiles/test.dir/src/route_model.cpp.i

CMakeFiles/test.dir/src/route_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test.dir/src/route_model.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp -o CMakeFiles/test.dir/src/route_model.cpp.s

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o" \
"CMakeFiles/test.dir/src/route_planner.cpp.o" \
"CMakeFiles/test.dir/src/model.cpp.o" \
"CMakeFiles/test.dir/src/route_model.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/test/utest_rp_a_star_search.cpp.o
test: CMakeFiles/test.dir/src/route_planner.cpp.o
test: CMakeFiles/test.dir/src/model.cpp.o
test: CMakeFiles/test.dir/src/route_model.cpp.o
test: CMakeFiles/test.dir/build.make
test: lib/libgtest_main.a
test: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/lib/libpugixml.a
test: lib/libgtest.a
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test
.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles/test.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/test.dir/depend

