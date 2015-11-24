#include "ObjectTracking.h"
#include "markerdetector.h"
#include "cvdrawingutils.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "object_detection/start.h"
#include "object_detection/stop.h"

using namespace std;
using namespace cv;
using namespace mid;

class TrackerNode
{
	public:
	
		ros::NodeHandle n;

		ros::ServiceServer start_serv;
	   ros::ServiceServer stop_serv;
		bool action;
		std::string filename_camera;
		double size_marker;

		ObjectTracking Ob_track; //class for tracking

		cv::VideoCapture vreader; //streamvideo
		cv::Mat Image;//input image for show it
		MarkerDetector MDetector; //marker to find in image processing

		//create a gui
		double ThresParam1, ThresParam2;
		int iThresParam1, iThresParam2;


		TrackerNode();
		~TrackerNode() {}

		//callback function
		bool startTrack(object_detection::start::Request &req, object_detection::start::Response &res);
		bool stopTrack(object_detection::stop::Request &req, object_detection::stop::Response &res);
		
		//threshold a runtime 
		void cvTackBarEvents(int pos, void *);

		void go(); //call it in your program for starting track algorithm  

};  
