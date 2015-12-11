#include <iostream> 
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

	
	std::string pathFile = ros::package::getPath("object_detection") +"/"+ filename_camera;
	CamParam.readFromXMLFile(pathFile); 
	

	action = false;
}


void TrackerNode::go(bool set)
{
	vreader >> Image;//get image from video for image processing	 

	// function for detection marker on image
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


void TrackerNode::cvTrackBarEventsStatic(int pos, void *that)
{
	TrackerNode &th = *(TrackerNode *)(that);

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


bool TrackerNode::startTrack(object_detection::start::Request &req, object_detection::start::Response &res)
{
	action = true;//flag di attivazione del tracking

	vreader.open(0);

	if (vreader.isOpened()) 
	{
		vreader.grab();
		vreader.retrieve(Image);
	}
	else {cout<<"error open video"<<endl;}


	vreader >> Image;
   CamParam.resize(Image.size()); 	

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

	
	return true;
}

bool TrackerNode::stopTrack(object_detection::stop::Request &req, object_detection::stop::Response &res)
{
	action = false;
	vreader.release();
	destroyAllWindows();
	return true;
}






