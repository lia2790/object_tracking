#include "TrackerNode.h"

using namespace std;
using namespace cv;

TrackerNode::TrackerNode()
{
	n = ros::NodeHandle("ObjectTracker");
   start_serv =  n.advertiseService("Start",&TrackerNode::startTrack,this);
	stop_serv =  n.advertiseService("Stop",&TrackerNode::stopTrack, this);
	n.param<std::string>("filename_camera", filename_camera, " ");
	n.param<double>("size_marker", size_marker, -1);

	action = false;
       
	//create a window for show image captured
	cv::namedWindow("in", 1);
	cv::namedWindow("thres", 1);

	//create threshold_bar for choose a runtime the value
	MDetector.getThresholdParams(ThresParam1, ThresParam2);
	iThresParam1 = ThresParam1;
	iThresParam2 = ThresParam2;
	cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, &TrackerNode::cvTackBarEvents);
	cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, &TrackerNode::cvTackBarEvents);
}


bool TrackerNode::startTrack(object_detection::start::Request &req, object_detection::start::Response &res)
{
	action = true;
	vreader.open(0);
	if (vreader.isOpened()) 
	{
		vreader.grab();
		vreader.retrieve(Image);
	}
	vreader >> Image;//load a first image

	
	Ob_track.setValue(filename_camera.c_str(), size_marker, Image);//preload size marker, camera parameters 


	return true;
}

bool TrackerNode::stopTrack(object_detection::stop::Request &req, object_detection::stop::Response &res)
{
	action = false;
	vreader.release();
	return true;
}

void TrackerNode::cvTackBarEvents(int pos, void *) 
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

void TrackerNode::go()
{
	vreader >> Image;//get image from video for image processing	 

	Ob_track.ObjectPose(MDetector, Image);//image processing with estimate marker pose	
    
	// show input with augmented information
	cv::imshow("in", Image);
	// show also the internal image resulting from the threshold operation
	cv::imshow("thres", MDetector.getThresholdedImage());
}




