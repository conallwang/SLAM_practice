# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/johnson/SLAM/Fun

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/johnson/SLAM/Fun/build

# Include any dependencies generated for this target.
include bin/CMakeFiles/build_map.dir/depend.make

# Include the progress variables for this target.
include bin/CMakeFiles/build_map.dir/progress.make

# Include the compile flags for this target's objects.
include bin/CMakeFiles/build_map.dir/flags.make

bin/CMakeFiles/build_map.dir/build_map.cpp.o: bin/CMakeFiles/build_map.dir/flags.make
bin/CMakeFiles/build_map.dir/build_map.cpp.o: ../src/build_map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/johnson/SLAM/Fun/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object bin/CMakeFiles/build_map.dir/build_map.cpp.o"
	cd /home/johnson/SLAM/Fun/build/bin && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/build_map.dir/build_map.cpp.o -c /home/johnson/SLAM/Fun/src/build_map.cpp

bin/CMakeFiles/build_map.dir/build_map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/build_map.dir/build_map.cpp.i"
	cd /home/johnson/SLAM/Fun/build/bin && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/johnson/SLAM/Fun/src/build_map.cpp > CMakeFiles/build_map.dir/build_map.cpp.i

bin/CMakeFiles/build_map.dir/build_map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/build_map.dir/build_map.cpp.s"
	cd /home/johnson/SLAM/Fun/build/bin && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/johnson/SLAM/Fun/src/build_map.cpp -o CMakeFiles/build_map.dir/build_map.cpp.s

bin/CMakeFiles/build_map.dir/build_map.cpp.o.requires:

.PHONY : bin/CMakeFiles/build_map.dir/build_map.cpp.o.requires

bin/CMakeFiles/build_map.dir/build_map.cpp.o.provides: bin/CMakeFiles/build_map.dir/build_map.cpp.o.requires
	$(MAKE) -f bin/CMakeFiles/build_map.dir/build.make bin/CMakeFiles/build_map.dir/build_map.cpp.o.provides.build
.PHONY : bin/CMakeFiles/build_map.dir/build_map.cpp.o.provides

bin/CMakeFiles/build_map.dir/build_map.cpp.o.provides.build: bin/CMakeFiles/build_map.dir/build_map.cpp.o


# Object files for target build_map
build_map_OBJECTS = \
"CMakeFiles/build_map.dir/build_map.cpp.o"

# External object files for target build_map
build_map_EXTERNAL_OBJECTS =

bin/build_map: bin/CMakeFiles/build_map.dir/build_map.cpp.o
bin/build_map: bin/CMakeFiles/build_map.dir/build.make
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
bin/build_map: /home/johnson/Pangolin/build/src/libpangolin.so
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
bin/build_map: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
bin/build_map: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libGL.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libGLEW.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libwayland-client.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libwayland-egl.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libwayland-cursor.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libSM.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libICE.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libX11.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libXext.so
bin/build_map: /usr/lib/libOpenNI.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libpng.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libz.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libjpeg.so
bin/build_map: /usr/lib/x86_64-linux-gnu/libtiff.so
bin/build_map: /usr/lib/x86_64-linux-gnu/liblz4.so
bin/build_map: bin/CMakeFiles/build_map.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/johnson/SLAM/Fun/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable build_map"
	cd /home/johnson/SLAM/Fun/build/bin && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/build_map.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
bin/CMakeFiles/build_map.dir/build: bin/build_map

.PHONY : bin/CMakeFiles/build_map.dir/build

bin/CMakeFiles/build_map.dir/requires: bin/CMakeFiles/build_map.dir/build_map.cpp.o.requires

.PHONY : bin/CMakeFiles/build_map.dir/requires

bin/CMakeFiles/build_map.dir/clean:
	cd /home/johnson/SLAM/Fun/build/bin && $(CMAKE_COMMAND) -P CMakeFiles/build_map.dir/cmake_clean.cmake
.PHONY : bin/CMakeFiles/build_map.dir/clean

bin/CMakeFiles/build_map.dir/depend:
	cd /home/johnson/SLAM/Fun/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/johnson/SLAM/Fun /home/johnson/SLAM/Fun/src /home/johnson/SLAM/Fun/build /home/johnson/SLAM/Fun/build/bin /home/johnson/SLAM/Fun/build/bin/CMakeFiles/build_map.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : bin/CMakeFiles/build_map.dir/depend

