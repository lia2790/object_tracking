#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>
#include <cstdio>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "TrackerNode.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


using namespace std;
using namespace cv;
using namespace mid;//mid ->namespace marker image detection



int main(int argc, char **argv) 
{

	//node ROS
	ros::init(argc, argv, "ObjectTracker");
	ros::Rate loop_rate(100);
	
	TrackerNode track;

	int count_image = 0;


	while(track.n.ok())
	{
			if(track.action)//if true, tracking
			{
				track.go();
				count_image++;
			}	  
	  ros::spinOnce();
	  loop_rate.sleep();

	  
	}//end_while    


	cout<<"image processed are : "<<count_image<<endl;



	return 0;
}//ENDmain





