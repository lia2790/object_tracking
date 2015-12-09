#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "FlyCapture2.h"
#include <iostream>

using namespace FlyCapture2;
using namespace cv;

class FlyCap	
{
public:

	FlyCapture2::Error error;
	Camera *camera;
	CameraInfo camInfo;
	Mat image_for_OPENCV;

	FlyCap() {camera = new Camera; }
	~FlyCap() { delete camera;}

	void startVideo();
	void takeImageFromVideo();
	void closeVideo();
	Mat getImageOpencv() {	return image_for_OPENCV; }
};
