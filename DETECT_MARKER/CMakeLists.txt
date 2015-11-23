CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(obj_tracking)

FIND_PACKAGE(OpenCV REQUIRED)


FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()




set(SOURCE program/detection_marker.cpp 
	src/cameraparameters.cpp 
	src/cvdrawingutils.cpp 
	src/marker.cpp 
	src/markerdetector.cpp 
	src/subpixelcorner.cpp 
	src/fidmarkers.cpp)


set(SOURCE1 program/create_marker.cpp 
	src/cameraparameters.cpp 
	src/cvdrawingutils.cpp 
	src/marker.cpp 
	src/markerdetector.cpp 
	src/subpixelcorner.cpp 
	src/fidmarkers.cpp)




INCLUDE_DIRECTORIES(src)#dove si trovano i sorgenti sia .cpp che .h


ADD_EXECUTABLE(track_marker ${SOURCE})#nome dell'eseguibile
ADD_EXECUTABLE(create_marker ${SOURCE1})#nome dell'eseguibile


TARGET_LINK_LIBRARIES(create_marker ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(track_marker ${OpenCV_LIBS})




