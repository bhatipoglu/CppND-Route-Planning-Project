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
include CMakeFiles/OSM_A_star_search.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/OSM_A_star_search.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/OSM_A_star_search.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OSM_A_star_search.dir/flags.make

CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o: CMakeFiles/OSM_A_star_search.dir/flags.make
CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/main.cpp
CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o: CMakeFiles/OSM_A_star_search.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o -MF CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o.d -o CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/main.cpp

CMakeFiles/OSM_A_star_search.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/OSM_A_star_search.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/main.cpp > CMakeFiles/OSM_A_star_search.dir/src/main.cpp.i

CMakeFiles/OSM_A_star_search.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/OSM_A_star_search.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/main.cpp -o CMakeFiles/OSM_A_star_search.dir/src/main.cpp.s

CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o: CMakeFiles/OSM_A_star_search.dir/flags.make
CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp
CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o: CMakeFiles/OSM_A_star_search.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o -MF CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o.d -o CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp

CMakeFiles/OSM_A_star_search.dir/src/model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/OSM_A_star_search.dir/src/model.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp > CMakeFiles/OSM_A_star_search.dir/src/model.cpp.i

CMakeFiles/OSM_A_star_search.dir/src/model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/OSM_A_star_search.dir/src/model.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/model.cpp -o CMakeFiles/OSM_A_star_search.dir/src/model.cpp.s

CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o: CMakeFiles/OSM_A_star_search.dir/flags.make
CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/render.cpp
CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o: CMakeFiles/OSM_A_star_search.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o -MF CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o.d -o CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/render.cpp

CMakeFiles/OSM_A_star_search.dir/src/render.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/OSM_A_star_search.dir/src/render.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/render.cpp > CMakeFiles/OSM_A_star_search.dir/src/render.cpp.i

CMakeFiles/OSM_A_star_search.dir/src/render.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/OSM_A_star_search.dir/src/render.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/render.cpp -o CMakeFiles/OSM_A_star_search.dir/src/render.cpp.s

CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o: CMakeFiles/OSM_A_star_search.dir/flags.make
CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp
CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o: CMakeFiles/OSM_A_star_search.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o -MF CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o.d -o CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp

CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp > CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.i

CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_model.cpp -o CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.s

CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o: CMakeFiles/OSM_A_star_search.dir/flags.make
CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp
CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o: CMakeFiles/OSM_A_star_search.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o -MF CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o.d -o CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o -c /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp

CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp > CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.i

CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/src/route_planner.cpp -o CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.s

# Object files for target OSM_A_star_search
OSM_A_star_search_OBJECTS = \
"CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o" \
"CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o" \
"CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o" \
"CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o" \
"CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o"

# External object files for target OSM_A_star_search
OSM_A_star_search_EXTERNAL_OBJECTS =

OSM_A_star_search: CMakeFiles/OSM_A_star_search.dir/src/main.cpp.o
OSM_A_star_search: CMakeFiles/OSM_A_star_search.dir/src/model.cpp.o
OSM_A_star_search: CMakeFiles/OSM_A_star_search.dir/src/render.cpp.o
OSM_A_star_search: CMakeFiles/OSM_A_star_search.dir/src/route_model.cpp.o
OSM_A_star_search: CMakeFiles/OSM_A_star_search.dir/src/route_planner.cpp.o
OSM_A_star_search: CMakeFiles/OSM_A_star_search.dir/build.make
OSM_A_star_search: /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/lib/libpugixml.a
OSM_A_star_search: /usr/local/lib/libio2d_cairo_xlib.a
OSM_A_star_search: /usr/local/lib/libio2d_cairo.a
OSM_A_star_search: /usr/local/lib/libio2d_core.a
OSM_A_star_search: /opt/homebrew/Cellar/cairo/1.18.0/lib/libcairo.dylib
OSM_A_star_search: /opt/homebrew/Cellar/graphicsmagick/1.3.42_1/lib/libGraphicsMagick.dylib
OSM_A_star_search: /opt/homebrew/lib/libpixman-1.dylib
OSM_A_star_search: /opt/homebrew/lib/libfreetype.dylib
OSM_A_star_search: /opt/homebrew/lib/libfontconfig.dylib
OSM_A_star_search: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk/usr/lib/libbz2.tbd
OSM_A_star_search: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk/usr/lib/libz.tbd
OSM_A_star_search: /opt/homebrew/lib/libjpeg.dylib
OSM_A_star_search: /opt/homebrew/lib/libpng16.dylib
OSM_A_star_search: /opt/homebrew/lib/libtiff.dylib
OSM_A_star_search: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk/usr/lib/libexpat.tbd
OSM_A_star_search: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk/usr/lib/liblzma.tbd
OSM_A_star_search: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk/usr/lib/libiconv.tbd
OSM_A_star_search: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.2.sdk/usr/lib/libcharset.tbd
OSM_A_star_search: /opt/homebrew/lib/libX11.dylib
OSM_A_star_search: CMakeFiles/OSM_A_star_search.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable OSM_A_star_search"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OSM_A_star_search.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OSM_A_star_search.dir/build: OSM_A_star_search
.PHONY : CMakeFiles/OSM_A_star_search.dir/build

CMakeFiles/OSM_A_star_search.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OSM_A_star_search.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OSM_A_star_search.dir/clean

CMakeFiles/OSM_A_star_search.dir/depend:
	cd /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build /Users/bhatipoglu/Udacity-Cpp-ND/CppND-Route-Planning-Project/build/CMakeFiles/OSM_A_star_search.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/OSM_A_star_search.dir/depend
