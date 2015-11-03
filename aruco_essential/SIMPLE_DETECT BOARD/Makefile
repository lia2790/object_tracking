CC = g++ 
CFLAGS = `pkg-config --libs --cflags opencv`


track: aruco_simple.cpp board.cpp cameraparameters.cpp boarddetector.cpp cvdrawingutils.cpp marker.cpp markerdetector.cpp ar_omp.cpp subpixelcorner.cpp arucofidmarkers.cpp
	$(CC) aruco_simple.cpp board.cpp cameraparameters.cpp boarddetector.cpp cvdrawingutils.cpp marker.cpp markerdetector.cpp ar_omp.cpp subpixelcorner.cpp arucofidmarkers.cpp $(CFLAGS) -o tracking
