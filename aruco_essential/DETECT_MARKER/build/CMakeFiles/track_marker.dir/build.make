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
CMAKE_SOURCE_DIR = /home/lia/Scrivania/DETECT_MARKER

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lia/Scrivania/DETECT_MARKER/build

# Include any dependencies generated for this target.
include CMakeFiles/track_marker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/track_marker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/track_marker.dir/flags.make

CMakeFiles/track_marker.dir/program/detection_marker.cpp.o: CMakeFiles/track_marker.dir/flags.make
CMakeFiles/track_marker.dir/program/detection_marker.cpp.o: ../program/detection_marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/track_marker.dir/program/detection_marker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_marker.dir/program/detection_marker.cpp.o -c /home/lia/Scrivania/DETECT_MARKER/program/detection_marker.cpp

CMakeFiles/track_marker.dir/program/detection_marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_marker.dir/program/detection_marker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/DETECT_MARKER/program/detection_marker.cpp > CMakeFiles/track_marker.dir/program/detection_marker.cpp.i

CMakeFiles/track_marker.dir/program/detection_marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_marker.dir/program/detection_marker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/DETECT_MARKER/program/detection_marker.cpp -o CMakeFiles/track_marker.dir/program/detection_marker.cpp.s

CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.requires:
.PHONY : CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.requires

CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.provides: CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.requires
	$(MAKE) -f CMakeFiles/track_marker.dir/build.make CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.provides.build
.PHONY : CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.provides

CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.provides.build: CMakeFiles/track_marker.dir/program/detection_marker.cpp.o

CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o: CMakeFiles/track_marker.dir/flags.make
CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o: ../src/cameraparameters.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o -c /home/lia/Scrivania/DETECT_MARKER/src/cameraparameters.cpp

CMakeFiles/track_marker.dir/src/cameraparameters.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_marker.dir/src/cameraparameters.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/DETECT_MARKER/src/cameraparameters.cpp > CMakeFiles/track_marker.dir/src/cameraparameters.cpp.i

CMakeFiles/track_marker.dir/src/cameraparameters.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_marker.dir/src/cameraparameters.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/DETECT_MARKER/src/cameraparameters.cpp -o CMakeFiles/track_marker.dir/src/cameraparameters.cpp.s

CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.requires:
.PHONY : CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.requires

CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.provides: CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.requires
	$(MAKE) -f CMakeFiles/track_marker.dir/build.make CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.provides.build
.PHONY : CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.provides

CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.provides.build: CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o

CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o: CMakeFiles/track_marker.dir/flags.make
CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o: ../src/cvdrawingutils.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o -c /home/lia/Scrivania/DETECT_MARKER/src/cvdrawingutils.cpp

CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/DETECT_MARKER/src/cvdrawingutils.cpp > CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.i

CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/DETECT_MARKER/src/cvdrawingutils.cpp -o CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.s

CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.requires:
.PHONY : CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.requires

CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.provides: CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.requires
	$(MAKE) -f CMakeFiles/track_marker.dir/build.make CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.provides.build
.PHONY : CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.provides

CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.provides.build: CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o

CMakeFiles/track_marker.dir/src/marker.cpp.o: CMakeFiles/track_marker.dir/flags.make
CMakeFiles/track_marker.dir/src/marker.cpp.o: ../src/marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/track_marker.dir/src/marker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_marker.dir/src/marker.cpp.o -c /home/lia/Scrivania/DETECT_MARKER/src/marker.cpp

CMakeFiles/track_marker.dir/src/marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_marker.dir/src/marker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/DETECT_MARKER/src/marker.cpp > CMakeFiles/track_marker.dir/src/marker.cpp.i

CMakeFiles/track_marker.dir/src/marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_marker.dir/src/marker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/DETECT_MARKER/src/marker.cpp -o CMakeFiles/track_marker.dir/src/marker.cpp.s

CMakeFiles/track_marker.dir/src/marker.cpp.o.requires:
.PHONY : CMakeFiles/track_marker.dir/src/marker.cpp.o.requires

CMakeFiles/track_marker.dir/src/marker.cpp.o.provides: CMakeFiles/track_marker.dir/src/marker.cpp.o.requires
	$(MAKE) -f CMakeFiles/track_marker.dir/build.make CMakeFiles/track_marker.dir/src/marker.cpp.o.provides.build
.PHONY : CMakeFiles/track_marker.dir/src/marker.cpp.o.provides

CMakeFiles/track_marker.dir/src/marker.cpp.o.provides.build: CMakeFiles/track_marker.dir/src/marker.cpp.o

CMakeFiles/track_marker.dir/src/markerdetector.cpp.o: CMakeFiles/track_marker.dir/flags.make
CMakeFiles/track_marker.dir/src/markerdetector.cpp.o: ../src/markerdetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/track_marker.dir/src/markerdetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_marker.dir/src/markerdetector.cpp.o -c /home/lia/Scrivania/DETECT_MARKER/src/markerdetector.cpp

CMakeFiles/track_marker.dir/src/markerdetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_marker.dir/src/markerdetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/DETECT_MARKER/src/markerdetector.cpp > CMakeFiles/track_marker.dir/src/markerdetector.cpp.i

CMakeFiles/track_marker.dir/src/markerdetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_marker.dir/src/markerdetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/DETECT_MARKER/src/markerdetector.cpp -o CMakeFiles/track_marker.dir/src/markerdetector.cpp.s

CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.requires:
.PHONY : CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.requires

CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.provides: CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/track_marker.dir/build.make CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.provides.build
.PHONY : CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.provides

CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.provides.build: CMakeFiles/track_marker.dir/src/markerdetector.cpp.o

CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o: CMakeFiles/track_marker.dir/flags.make
CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o: ../src/subpixelcorner.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o -c /home/lia/Scrivania/DETECT_MARKER/src/subpixelcorner.cpp

CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/DETECT_MARKER/src/subpixelcorner.cpp > CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.i

CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/DETECT_MARKER/src/subpixelcorner.cpp -o CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.s

CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.requires:
.PHONY : CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.requires

CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.provides: CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.requires
	$(MAKE) -f CMakeFiles/track_marker.dir/build.make CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.provides.build
.PHONY : CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.provides

CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.provides.build: CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o

CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o: CMakeFiles/track_marker.dir/flags.make
CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o: ../src/arucofidmarkers.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o -c /home/lia/Scrivania/DETECT_MARKER/src/arucofidmarkers.cpp

CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lia/Scrivania/DETECT_MARKER/src/arucofidmarkers.cpp > CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.i

CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lia/Scrivania/DETECT_MARKER/src/arucofidmarkers.cpp -o CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.s

CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.requires:
.PHONY : CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.requires

CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.provides: CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.requires
	$(MAKE) -f CMakeFiles/track_marker.dir/build.make CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.provides.build
.PHONY : CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.provides

CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.provides.build: CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o

# Object files for target track_marker
track_marker_OBJECTS = \
"CMakeFiles/track_marker.dir/program/detection_marker.cpp.o" \
"CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o" \
"CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o" \
"CMakeFiles/track_marker.dir/src/marker.cpp.o" \
"CMakeFiles/track_marker.dir/src/markerdetector.cpp.o" \
"CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o" \
"CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o"

# External object files for target track_marker
track_marker_EXTERNAL_OBJECTS =

track_marker: CMakeFiles/track_marker.dir/program/detection_marker.cpp.o
track_marker: CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o
track_marker: CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o
track_marker: CMakeFiles/track_marker.dir/src/marker.cpp.o
track_marker: CMakeFiles/track_marker.dir/src/markerdetector.cpp.o
track_marker: CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o
track_marker: CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o
track_marker: CMakeFiles/track_marker.dir/build.make
track_marker: /usr/local/lib/libopencv_videostab.so.3.0.0
track_marker: /usr/local/lib/libopencv_videoio.so.3.0.0
track_marker: /usr/local/lib/libopencv_video.so.3.0.0
track_marker: /usr/local/lib/libopencv_superres.so.3.0.0
track_marker: /usr/local/lib/libopencv_stitching.so.3.0.0
track_marker: /usr/local/lib/libopencv_shape.so.3.0.0
track_marker: /usr/local/lib/libopencv_photo.so.3.0.0
track_marker: /usr/local/lib/libopencv_objdetect.so.3.0.0
track_marker: /usr/local/lib/libopencv_ml.so.3.0.0
track_marker: /usr/local/lib/libopencv_imgproc.so.3.0.0
track_marker: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
track_marker: /usr/local/lib/libopencv_highgui.so.3.0.0
track_marker: /usr/local/lib/libopencv_hal.a
track_marker: /usr/local/lib/libopencv_flann.so.3.0.0
track_marker: /usr/local/lib/libopencv_features2d.so.3.0.0
track_marker: /usr/local/lib/libopencv_core.so.3.0.0
track_marker: /usr/local/lib/libopencv_calib3d.so.3.0.0
track_marker: /usr/local/lib/libopencv_features2d.so.3.0.0
track_marker: /usr/local/lib/libopencv_ml.so.3.0.0
track_marker: /usr/local/lib/libopencv_highgui.so.3.0.0
track_marker: /usr/local/lib/libopencv_videoio.so.3.0.0
track_marker: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
track_marker: /usr/local/lib/libopencv_flann.so.3.0.0
track_marker: /usr/local/lib/libopencv_video.so.3.0.0
track_marker: /usr/local/lib/libopencv_imgproc.so.3.0.0
track_marker: /usr/local/lib/libopencv_core.so.3.0.0
track_marker: /usr/local/lib/libopencv_hal.a
track_marker: /usr/local/share/OpenCV/3rdparty/lib/libippicv.a
track_marker: CMakeFiles/track_marker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable track_marker"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/track_marker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/track_marker.dir/build: track_marker
.PHONY : CMakeFiles/track_marker.dir/build

CMakeFiles/track_marker.dir/requires: CMakeFiles/track_marker.dir/program/detection_marker.cpp.o.requires
CMakeFiles/track_marker.dir/requires: CMakeFiles/track_marker.dir/src/cameraparameters.cpp.o.requires
CMakeFiles/track_marker.dir/requires: CMakeFiles/track_marker.dir/src/cvdrawingutils.cpp.o.requires
CMakeFiles/track_marker.dir/requires: CMakeFiles/track_marker.dir/src/marker.cpp.o.requires
CMakeFiles/track_marker.dir/requires: CMakeFiles/track_marker.dir/src/markerdetector.cpp.o.requires
CMakeFiles/track_marker.dir/requires: CMakeFiles/track_marker.dir/src/subpixelcorner.cpp.o.requires
CMakeFiles/track_marker.dir/requires: CMakeFiles/track_marker.dir/src/arucofidmarkers.cpp.o.requires
.PHONY : CMakeFiles/track_marker.dir/requires

CMakeFiles/track_marker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/track_marker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/track_marker.dir/clean

CMakeFiles/track_marker.dir/depend:
	cd /home/lia/Scrivania/DETECT_MARKER/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lia/Scrivania/DETECT_MARKER /home/lia/Scrivania/DETECT_MARKER /home/lia/Scrivania/DETECT_MARKER/build /home/lia/Scrivania/DETECT_MARKER/build /home/lia/Scrivania/DETECT_MARKER/build/CMakeFiles/track_marker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/track_marker.dir/depend

