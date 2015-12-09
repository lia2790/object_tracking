#include <iostream> 
#include "TrackerNode.h"

using namespace std;
using namespace cv;

TrackerNode::TrackerNode()
{
	n = ros::NodeHandle("ObjectTracker");//create ROS node

	//create the service
   start_serv =  n.advertiseService("Start",&TrackerNode::startTrack,this);
	stop_serv =  n.advertiseService("Stop",&TrackerNode::stopTrack, this);

	//take the param from file
	n.param<std::string>("filename_camera", filename_camera, " ");
	n.param<double>("size_marker", size_marker, -1);
	n.param<int>("use_camera",use_camera, 0);

	//iniz the camera parameters
	std::string pathFile = ros::package::getPath("object_detection") +"/"+ filename_camera;
	CamParam.readFromXMLFile(pathFile); 
	
	//for start without the obj track
	action = false;
}


///********************************************************************
///		RESEARCH MARKERS AND VISUAL FEEDBACK
///********************************************************************

void TrackerNode::go(bool set)
{
	//choose camera
	//get image from video for image processing
	if(use_camera)
		vreader >> Image;
	else
	{
		FCapture.takeImageFromVideo();
		Image = FCapture.getImageOpencv();
	}



	// function for detection markers on image
	MDetector.detect(Image, Markers, CamParam, size_marker);

	if(set)
	{	//Try to draw the result
		for (unsigned int i = 0; i < Markers.size(); i++) 
		{
			cout << Markers[i] << endl;//print the value of sigle marker
      	Markers[i].draw(Image, Scalar(0, 0, 255), 2);
		}

		// draw a 3d cube in each marker if there is 3d info
		if (CamParam.isValid() && size_marker != -1)
			for (unsigned int i = 0; i < Markers.size(); i++) 
      	{   
				CvDrawingUtils::draw3dCube(Image, Markers[i], CamParam);
				CvDrawingUtils::draw3dAxis(Image, Markers[i], CamParam);
			}   
		// show input with augmented information
		cv::imshow("in", Image);
		// show also the internal image resulting from the threshold operation
		cv::imshow("thres", MDetector.getThresholdedImage());
		int key = waitKey(5);
	}
}

//************************************************************



///*************************************************************
///		THRESHOLD SETTING A RUN-TIME
///*************************************************************

void TrackerNode::cvTrackBarEventsStatic(int pos, void *that)
{
	TrackerNode th = *(TrackerNode *)(that);

	th.cvTrackBarEvents(pos, (TrackerNode *)(that));
}



void TrackerNode::cvTrackBarEvents(int pos, TrackerNode *pr) 
{
	Mat TheInputImageCopy;

   if (pr->iThresParam1 < 3)
      pr->iThresParam1 = 3;
   if (pr->iThresParam1 % 2 != 1)
   	 pr->iThresParam1++;
   if (pr->ThresParam2 < 1)
   	 pr->ThresParam2 = 1;
  		  
	pr->ThresParam1 = pr->iThresParam1;
	pr->ThresParam2 = pr->iThresParam2;
 
	pr->MDetector.setThresholdParams(pr->ThresParam1, pr->ThresParam2);
    
  	pr->Image.copyTo(TheInputImageCopy);
  
	cv::imshow("in", TheInputImageCopy);  
	cv::imshow("thres", pr->MDetector.getThresholdedImage());
}

///************************************************************************



///************************************************************************
///		START and STOP SERVICE IN ROS NODE
///*************************************************************************

void TrackerNode::openWithVideo0()
{
	vreader.open(0);//open the stream video

	if (vreader.isOpened()) 
	{
		vreader.grab();
		vreader.retrieve(Image);
	}
	else {cout<<"error open video"<<endl;}

	vreader >> Image;
}


void TrackerNode::openWithFlyCap()
{
	FCapture.startVideo();
	FCapture.takeImageFromVideo();
	Image = FCapture.getImageOpencv();
}

bool TrackerNode::startTrack(object_detection::start::Request &req, object_detection::start::Response &res)
{
	action = true;//flag for starting track

	//create a window for show image captured
	cv::namedWindow("in", 1);
	cv::namedWindow("thres", 1);

	//create threshold_bar for choose a runtime the value
	//MDetector.getThresholdParams(ThresParam1, ThresParam2);
	iThresParam1 = 13;
	iThresParam2 = 10;
	cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, &TrackerNode::cvTrackBarEventsStatic, this);
	cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, &TrackerNode::cvTrackBarEventsStatic, this);
	//TrackerNode::cvTrackBarEvents(0,0);
	
	//select the camera using for get image
	//get the first image
	if(use_camera)
		openWithVideo0();   
	else
		openWithFlyCap();


	CamParam.resize(Image.size()); 	
	

	return true;
}


bool TrackerNode::stopTrack(object_detection::stop::Request &req, object_detection::stop::Response &res)
{
	action = false;
	
	//release the stream video
	if(use_camera)	vreader.release();
	else		      FCapture.closeVideo();

	destroyAllWindows();
	return true;
}

///**************************************************************************+




