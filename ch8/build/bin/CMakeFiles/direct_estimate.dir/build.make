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
CMAKE_SOURCE_DIR = /home/johnson/SLAM/ch8

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/johnson/SLAM/ch8/build

# Include any dependencies generated for this target.
include bin/CMakeFiles/direct_estimate.dir/depend.make

# Include the progress variables for this target.
include bin/CMakeFiles/direct_estimate.dir/progress.make

# Include the compile flags for this target's objects.
include bin/CMakeFiles/direct_estimate.dir/flags.make

bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o: bin/CMakeFiles/direct_estimate.dir/flags.make
bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o: ../src/direct_estimate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/johnson/SLAM/ch8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o"
	cd /home/johnson/SLAM/ch8/build/bin && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o -c /home/johnson/SLAM/ch8/src/direct_estimate.cpp

bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/direct_estimate.dir/direct_estimate.cpp.i"
	cd /home/johnson/SLAM/ch8/build/bin && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/johnson/SLAM/ch8/src/direct_estimate.cpp > CMakeFiles/direct_estimate.dir/direct_estimate.cpp.i

bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/direct_estimate.dir/direct_estimate.cpp.s"
	cd /home/johnson/SLAM/ch8/build/bin && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/johnson/SLAM/ch8/src/direct_estimate.cpp -o CMakeFiles/direct_estimate.dir/direct_estimate.cpp.s

bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.requires:

.PHONY : bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.requires

bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.provides: bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.requires
	$(MAKE) -f bin/CMakeFiles/direct_estimate.dir/build.make bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.provides.build
.PHONY : bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.provides

bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.provides.build: bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o


# Object files for target direct_estimate
direct_estimate_OBJECTS = \
"CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o"

# External object files for target direct_estimate
direct_estimate_EXTERNAL_OBJECTS =

bin/direct_estimate: bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o
bin/direct_estimate: bin/CMakeFiles/direct_estimate.dir/build.make
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
bin/direct_estimate: /home/johnson/Pangolin/build/src/libpangolin.so
bin/direct_estimate: /usr/lib/liborb_utils.so
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
bin/direct_estimate: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libGL.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libGLEW.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libwayland-client.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libwayland-egl.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libwayland-cursor.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libSM.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libICE.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libX11.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libXext.so
bin/direct_estimate: /usr/lib/libOpenNI.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libpng.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libz.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libjpeg.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/libtiff.so
bin/direct_estimate: /usr/lib/x86_64-linux-gnu/liblz4.so
bin/direct_estimate: bin/CMakeFiles/direct_estimate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/johnson/SLAM/ch8/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable direct_estimate"
	cd /home/johnson/SLAM/ch8/build/bin && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/direct_estimate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
bin/CMakeFiles/direct_estimate.dir/build: bin/direct_estimate

.PHONY : bin/CMakeFiles/direct_estimate.dir/build

bin/CMakeFiles/direct_estimate.dir/requires: bin/CMakeFiles/direct_estimate.dir/direct_estimate.cpp.o.requires

.PHONY : bin/CMakeFiles/direct_estimate.dir/requires

bin/CMakeFiles/direct_estimate.dir/clean:
	cd /home/johnson/SLAM/ch8/build/bin && $(CMAKE_COMMAND) -P CMakeFiles/direct_estimate.dir/cmake_clean.cmake
.PHONY : bin/CMakeFiles/direct_estimate.dir/clean

bin/CMakeFiles/direct_estimate.dir/depend:
	cd /home/johnson/SLAM/ch8/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/johnson/SLAM/ch8 /home/johnson/SLAM/ch8/src /home/johnson/SLAM/ch8/build /home/johnson/SLAM/ch8/build/bin /home/johnson/SLAM/ch8/build/bin/CMakeFiles/direct_estimate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : bin/CMakeFiles/direct_estimate.dir/depend

