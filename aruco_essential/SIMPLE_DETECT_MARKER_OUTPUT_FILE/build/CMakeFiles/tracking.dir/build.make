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
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lia/Scrivania/SIMPLE_DETECT_MARKER

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build

# Include any dependencies generated for this target.
include CMakeFiles/tracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tracking.dir/flags.make

CMakeFiles/tracking.dir/program/aruco_simple.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/program/aruco_simple.cpp.o: ../program/aruco_simple.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/program/aruco_simple.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/program/aruco_simple.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/program/aruco_simple.cpp

CMakeFiles/tracking.dir/program/aruco_simple.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/program/aruco_simple.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/program/aruco_simple.cpp > CMakeFiles/tracking.dir/program/aruco_simple.cpp.i

CMakeFiles/tracking.dir/program/aruco_simple.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/program/aruco_simple.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/program/aruco_simple.cpp -o CMakeFiles/tracking.dir/program/aruco_simple.cpp.s

CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.requires

CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.provides: CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.provides

CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.provides.build: CMakeFiles/tracking.dir/program/aruco_simple.cpp.o

CMakeFiles/tracking.dir/src/board.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/board.cpp.o: ../src/board.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/board.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/board.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/board.cpp

CMakeFiles/tracking.dir/src/board.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/board.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/board.cpp > CMakeFiles/tracking.dir/src/board.cpp.i

CMakeFiles/tracking.dir/src/board.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/board.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/board.cpp -o CMakeFiles/tracking.dir/src/board.cpp.s

CMakeFiles/tracking.dir/src/board.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/board.cpp.o.requires

CMakeFiles/tracking.dir/src/board.cpp.o.provides: CMakeFiles/tracking.dir/src/board.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/board.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/board.cpp.o.provides

CMakeFiles/tracking.dir/src/board.cpp.o.provides.build: CMakeFiles/tracking.dir/src/board.cpp.o

CMakeFiles/tracking.dir/src/cameraparameters.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/cameraparameters.cpp.o: ../src/cameraparameters.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/cameraparameters.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/cameraparameters.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/cameraparameters.cpp

CMakeFiles/tracking.dir/src/cameraparameters.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/cameraparameters.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/cameraparameters.cpp > CMakeFiles/tracking.dir/src/cameraparameters.cpp.i

CMakeFiles/tracking.dir/src/cameraparameters.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/cameraparameters.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/cameraparameters.cpp -o CMakeFiles/tracking.dir/src/cameraparameters.cpp.s

CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.requires

CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.provides: CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.provides

CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.provides.build: CMakeFiles/tracking.dir/src/cameraparameters.cpp.o

CMakeFiles/tracking.dir/src/boarddetector.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/boarddetector.cpp.o: ../src/boarddetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/boarddetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/boarddetector.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/boarddetector.cpp

CMakeFiles/tracking.dir/src/boarddetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/boarddetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/boarddetector.cpp > CMakeFiles/tracking.dir/src/boarddetector.cpp.i

CMakeFiles/tracking.dir/src/boarddetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/boarddetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/boarddetector.cpp -o CMakeFiles/tracking.dir/src/boarddetector.cpp.s

CMakeFiles/tracking.dir/src/boarddetector.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/boarddetector.cpp.o.requires

CMakeFiles/tracking.dir/src/boarddetector.cpp.o.provides: CMakeFiles/tracking.dir/src/boarddetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/boarddetector.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/boarddetector.cpp.o.provides

CMakeFiles/tracking.dir/src/boarddetector.cpp.o.provides.build: CMakeFiles/tracking.dir/src/boarddetector.cpp.o

CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o: ../src/cvdrawingutils.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/cvdrawingutils.cpp

CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/cvdrawingutils.cpp > CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.i

CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/cvdrawingutils.cpp -o CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.s

CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.requires

CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.provides: CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.provides

CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.provides.build: CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o

CMakeFiles/tracking.dir/src/marker.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/marker.cpp.o: ../src/marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/marker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/marker.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/marker.cpp

CMakeFiles/tracking.dir/src/marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/marker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/marker.cpp > CMakeFiles/tracking.dir/src/marker.cpp.i

CMakeFiles/tracking.dir/src/marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/marker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/marker.cpp -o CMakeFiles/tracking.dir/src/marker.cpp.s

CMakeFiles/tracking.dir/src/marker.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/marker.cpp.o.requires

CMakeFiles/tracking.dir/src/marker.cpp.o.provides: CMakeFiles/tracking.dir/src/marker.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/marker.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/marker.cpp.o.provides

CMakeFiles/tracking.dir/src/marker.cpp.o.provides.build: CMakeFiles/tracking.dir/src/marker.cpp.o

CMakeFiles/tracking.dir/src/markerdetector.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/markerdetector.cpp.o: ../src/markerdetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/markerdetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/markerdetector.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/markerdetector.cpp

CMakeFiles/tracking.dir/src/markerdetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/markerdetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/markerdetector.cpp > CMakeFiles/tracking.dir/src/markerdetector.cpp.i

CMakeFiles/tracking.dir/src/markerdetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/markerdetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/markerdetector.cpp -o CMakeFiles/tracking.dir/src/markerdetector.cpp.s

CMakeFiles/tracking.dir/src/markerdetector.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/markerdetector.cpp.o.requires

CMakeFiles/tracking.dir/src/markerdetector.cpp.o.provides: CMakeFiles/tracking.dir/src/markerdetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/markerdetector.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/markerdetector.cpp.o.provides

CMakeFiles/tracking.dir/src/markerdetector.cpp.o.provides.build: CMakeFiles/tracking.dir/src/markerdetector.cpp.o

CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o: ../src/subpixelcorner.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/subpixelcorner.cpp

CMakeFiles/tracking.dir/src/subpixelcorner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/subpixelcorner.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/subpixelcorner.cpp > CMakeFiles/tracking.dir/src/subpixelcorner.cpp.i

CMakeFiles/tracking.dir/src/subpixelcorner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/subpixelcorner.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/subpixelcorner.cpp -o CMakeFiles/tracking.dir/src/subpixelcorner.cpp.s

CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.requires

CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.provides: CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.provides

CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.provides.build: CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o

CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o: CMakeFiles/tracking.dir/flags.make
CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o: ../src/arucofidmarkers.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o -c /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/arucofidmarkers.cpp

CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/arucofidmarkers.cpp > CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.i

CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/SIMPLE_DETECT_MARKER/src/arucofidmarkers.cpp -o CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.s

CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.requires:
.PHONY : CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.requires

CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.provides: CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracking.dir/build.make CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.provides.build
.PHONY : CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.provides

CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.provides.build: CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o

# Object files for target tracking
tracking_OBJECTS = \
"CMakeFiles/tracking.dir/program/aruco_simple.cpp.o" \
"CMakeFiles/tracking.dir/src/board.cpp.o" \
"CMakeFiles/tracking.dir/src/cameraparameters.cpp.o" \
"CMakeFiles/tracking.dir/src/boarddetector.cpp.o" \
"CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o" \
"CMakeFiles/tracking.dir/src/marker.cpp.o" \
"CMakeFiles/tracking.dir/src/markerdetector.cpp.o" \
"CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o" \
"CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o"

# External object files for target tracking
tracking_EXTERNAL_OBJECTS =

tracking: CMakeFiles/tracking.dir/program/aruco_simple.cpp.o
tracking: CMakeFiles/tracking.dir/src/board.cpp.o
tracking: CMakeFiles/tracking.dir/src/cameraparameters.cpp.o
tracking: CMakeFiles/tracking.dir/src/boarddetector.cpp.o
tracking: CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o
tracking: CMakeFiles/tracking.dir/src/marker.cpp.o
tracking: CMakeFiles/tracking.dir/src/markerdetector.cpp.o
tracking: CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o
tracking: CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o
tracking: CMakeFiles/tracking.dir/build.make
tracking: /usr/local/lib/libopencv_videostab.so.3.0.0
tracking: /usr/local/lib/libopencv_videoio.so.3.0.0
tracking: /usr/local/lib/libopencv_video.so.3.0.0
tracking: /usr/local/lib/libopencv_superres.so.3.0.0
tracking: /usr/local/lib/libopencv_stitching.so.3.0.0
tracking: /usr/local/lib/libopencv_shape.so.3.0.0
tracking: /usr/local/lib/libopencv_photo.so.3.0.0
tracking: /usr/local/lib/libopencv_objdetect.so.3.0.0
tracking: /usr/local/lib/libopencv_ml.so.3.0.0
tracking: /usr/local/lib/libopencv_imgproc.so.3.0.0
tracking: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
tracking: /usr/local/lib/libopencv_highgui.so.3.0.0
tracking: /usr/local/lib/libopencv_hal.a
tracking: /usr/local/lib/libopencv_flann.so.3.0.0
tracking: /usr/local/lib/libopencv_features2d.so.3.0.0
tracking: /usr/local/lib/libopencv_core.so.3.0.0
tracking: /usr/local/lib/libopencv_calib3d.so.3.0.0
tracking: /usr/local/lib/libopencv_features2d.so.3.0.0
tracking: /usr/local/lib/libopencv_ml.so.3.0.0
tracking: /usr/local/lib/libopencv_highgui.so.3.0.0
tracking: /usr/local/lib/libopencv_videoio.so.3.0.0
tracking: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
tracking: /usr/local/lib/libopencv_flann.so.3.0.0
tracking: /usr/local/lib/libopencv_video.so.3.0.0
tracking: /usr/local/lib/libopencv_imgproc.so.3.0.0
tracking: /usr/local/lib/libopencv_core.so.3.0.0
tracking: /usr/local/lib/libopencv_hal.a
tracking: /usr/local/share/OpenCV/3rdparty/lib/libippicv.a
tracking: CMakeFiles/tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable tracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tracking.dir/build: tracking
.PHONY : CMakeFiles/tracking.dir/build

CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/program/aruco_simple.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/board.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/cameraparameters.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/boarddetector.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/cvdrawingutils.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/marker.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/markerdetector.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/subpixelcorner.cpp.o.requires
CMakeFiles/tracking.dir/requires: CMakeFiles/tracking.dir/src/arucofidmarkers.cpp.o.requires
.PHONY : CMakeFiles/tracking.dir/requires

CMakeFiles/tracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tracking.dir/clean

CMakeFiles/tracking.dir/depend:
	cd /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lia/Scrivania/SIMPLE_DETECT_MARKER /home/lia/Scrivania/SIMPLE_DETECT_MARKER /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build /home/lia/Scrivania/SIMPLE_DETECT_MARKER/build/CMakeFiles/tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tracking.dir/depend

