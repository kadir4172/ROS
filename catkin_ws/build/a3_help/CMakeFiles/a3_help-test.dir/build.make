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
CMAKE_SOURCE_DIR = /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build

# Include any dependencies generated for this target.
include a3_help/CMakeFiles/a3_help-test.dir/depend.make

# Include the progress variables for this target.
include a3_help/CMakeFiles/a3_help-test.dir/progress.make

# Include the compile flags for this target's objects.
include a3_help/CMakeFiles/a3_help-test.dir/flags.make

a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o: a3_help/CMakeFiles/a3_help-test.dir/flags.make
a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o: /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/src/a3_help/test/utest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o"
	cd /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/a3_help && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3_help-test.dir/test/utest.cpp.o -c /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/src/a3_help/test/utest.cpp

a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3_help-test.dir/test/utest.cpp.i"
	cd /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/a3_help && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/src/a3_help/test/utest.cpp > CMakeFiles/a3_help-test.dir/test/utest.cpp.i

a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3_help-test.dir/test/utest.cpp.s"
	cd /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/a3_help && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/src/a3_help/test/utest.cpp -o CMakeFiles/a3_help-test.dir/test/utest.cpp.s

a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.requires:

.PHONY : a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.requires

a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.provides: a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.requires
	$(MAKE) -f a3_help/CMakeFiles/a3_help-test.dir/build.make a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.provides.build
.PHONY : a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.provides

a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.provides.build: a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o


# Object files for target a3_help-test
a3_help__test_OBJECTS = \
"CMakeFiles/a3_help-test.dir/test/utest.cpp.o"

# External object files for target a3_help-test
a3_help__test_EXTERNAL_OBJECTS =

/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: a3_help/CMakeFiles/a3_help-test.dir/build.make
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: gtest/libgtest.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libcv_bridge.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_core3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libimage_transport.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libmessage_filters.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libclass_loader.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/libPocoFoundation.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libdl.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libroslib.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/librospack.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libroscpp.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/librosconsole.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/librostime.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /opt/ros/kinetic/lib/libcpp_common.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test: a3_help/CMakeFiles/a3_help-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test"
	cd /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/a3_help && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a3_help-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
a3_help/CMakeFiles/a3_help-test.dir/build: /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/devel/lib/a3_help/a3_help-test

.PHONY : a3_help/CMakeFiles/a3_help-test.dir/build

a3_help/CMakeFiles/a3_help-test.dir/requires: a3_help/CMakeFiles/a3_help-test.dir/test/utest.cpp.o.requires

.PHONY : a3_help/CMakeFiles/a3_help-test.dir/requires

a3_help/CMakeFiles/a3_help-test.dir/clean:
	cd /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/a3_help && $(CMAKE_COMMAND) -P CMakeFiles/a3_help-test.dir/cmake_clean.cmake
.PHONY : a3_help/CMakeFiles/a3_help-test.dir/clean

a3_help/CMakeFiles/a3_help-test.dir/depend:
	cd /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/src /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/src/a3_help /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/a3_help /home/kadir/Desktop/Freelancer/ROS/ROS/catkin_ws/build/a3_help/CMakeFiles/a3_help-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : a3_help/CMakeFiles/a3_help-test.dir/depend

