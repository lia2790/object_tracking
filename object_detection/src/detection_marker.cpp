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
#include "ObjectTracking.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


using namespace std;
using namespace cv;
using namespace mid;



//********************************************************
//	GLOBAL VAR
//********************************************************
cv::Mat Image;//input image
MarkerDetector MDetector;//marker to find in image processing
VideoCapture vreader;

//********************************************************
//	THRESHOLD AT RUN-TIME------similar of Opencv example
//********************************************************
void cvTackBarEvents(int pos, void *);
double ThresParam1, ThresParam2;
int iThresParam1, iThresParam2;

void cvTackBarEvents(int pos, void *) 
{
    Mat TheInputImageCopy;


    if (iThresParam1 < 3)
        iThresParam1 = 3;
    if (iThresParam1 % 2 != 1)
        iThresParam1++;
    if (ThresParam2 < 1)
        ThresParam2 = 1;
    ThresParam1 = iThresParam1;
    ThresParam2 = iThresParam2;
    MDetector.setThresholdParams(ThresParam1, ThresParam2);
    
    Image.copyTo(TheInputImageCopy);
  

    cv::imshow("in", TheInputImageCopy);  
    cv::imshow("thres", MDetector.getThresholdedImage());
}
//*****************************************************




int main(int argc, char **argv) 
{
	ros::init(argc, argv, "image_processing");
	ros::NodeHandle node;
	ros::Rate loop_rate(100);

	ObjectTracking Ob_track; //declare and automatic acquire stream video from /dev/video0
// try opening as video
	VideoCapture vreader(0);
	if (vreader.isOpened()) 
	{
		vreader.grab();
		vreader.retrieve(Image);
	}

	vreader >> Image;//first image 	
	
	Ob_track.setValue(argv[1], atof(argv[2]), Image);//	
       
	
       
	//create a window for show image captured
	cv::namedWindow("in", 1);
	cv::namedWindow("thres", 1);

	//create threshold_bar for choose a runtime the value
	MDetector.getThresholdParams(ThresParam1, ThresParam2);
	iThresParam1 = ThresParam1;
	iThresParam2 = ThresParam2;
	cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, cvTackBarEvents);
	cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, cvTackBarEvents);

	int count_image = 0;


	while(node.ok())
	{	
	  vreader >> Image;//get image from video for image processing	 

	  Ob_track.ObjectPose(MDetector, Image);//image processing with estimate marker pose	
    
     

        // show input with augmented information
        cv::imshow("in", Image);
        // show also the internal image resulting from the threshold operation
        cv::imshow("thres", MDetector.getThresholdedImage());
       
	  char c = waitKey(10); 
	  if (c == 27) break;// ESC for exit

	  ros::spinOnce();
	  loop_rate.sleep();

	  count_image++;
	}//end_while    


	cout<<"image processed are : "<<count_image<<endl;



	return 0;
}//ENDmain




