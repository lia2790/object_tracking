#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>
#include <cstdio>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "markerdetector.h"
#include "cvdrawingutils.h"

using namespace std;

namespace mid
{

class ObjectTracking
{

	public :

		mid::CameraParameters CamParam;//use the pinhole camera model
		
		//camera value of intrinsics matrix obtained with opencv calibration example with chessboard
		float MarkerSize;//size of marker side

		vector< Marker > Markers;//marker candidate

		ObjectTracking();
		
		~ObjectTracking() {}

		void setValue(const char *filename, float Mark_size, cv::Mat InImage);

		void ObjectPose(MarkerDetector &MDetector,cv::Mat Img);

};

}
