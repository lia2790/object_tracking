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
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <geometry_msgs/PoseStamped.h>



using namespace std;
using namespace cv;
using namespace mid;//mid ->namespace marker image detection


int main(int argc, char **argv) 
{
	//node ROS
	ros::init(argc, argv, "ObjectTracker");

	TrackerNode track;
	
	ros::Rate loop_rate(100);
	ros::Publisher pub = track.n.advertise<geometry_msgs::Pose>("Pose",10);
	geometry_msgs::Pose pose_msgs;
   
   double modelview_matrix[16];

	int count_image = 0;

	while(track.n.ok())
	{
		if(track.action)//if true, tracking
		{
			track.go(true);//true for to able feedback video

			for(int i = 0; i < track.Markers.size(); i++) 
			{
				track.Markers[i].glGetModelViewMatrix(modelview_matrix);//print the value of sigle marker

				
				tf::Quaternion q;
				tf::Matrix3x3 Rot(modelview_matrix[0],modelview_matrix[1],modelview_matrix[2],modelview_matrix[4],modelview_matrix[5],modelview_matrix[6],modelview_matrix[8],modelview_matrix[9],modelview_matrix[10]);
				Rot.getRotation(q);
			/*	btQuaternion q;
				btMatrix3x3 Rot;
				Rot.setFromOpenGLSubMatrix(modelview_matrix);
				Rot.getRotation(q);
          */

				pose_msgs.position.x = modelview_matrix[3] ;
				pose_msgs.position.y = modelview_matrix[7] ;
				pose_msgs.position.z = modelview_matrix[11] ;
				pose_msgs.orientation.x = q.getX();
				pose_msgs.orientation.y = q.getY();
				pose_msgs.orientation.z = q.getZ();
				pose_msgs.orientation.w = q.getW();
				pub.publish(pose_msgs);
			}

			count_image++;
		}	  

	  ros::spinOnce();
	  loop_rate.sleep();
	}//end_while    


	cout<<"image processed are : "<<count_image<<endl;

	return 0;
}//ENDmain





