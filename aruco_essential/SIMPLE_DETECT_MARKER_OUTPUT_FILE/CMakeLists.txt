CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(obj_tracking)

FIND_PACKAGE(OpenCV REQUIRED)

set(SOURCE program/aruco_simple.cpp 
	src/board.cpp 
	src/cameraparameters.cpp 
	src/boarddetector.cpp 
	src/cvdrawingutils.cpp 
	src/marker.cpp 
	src/markerdetector.cpp 
	src/subpixelcorner.cpp 
	src/arucofidmarkers.cpp)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

INCLUDE_DIRECTORIES(src)


ADD_EXECUTABLE(tracking ${SOURCE})#nome dell'eseguibile

TARGET_LINK_LIBRARIES(tracking ${OpenCV_LIBS})





