#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream> 
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include "cameraParameters.h"



using namespace std;
using namespace cv;


//********************************************
//	read XML file for CAMERA PARAMETERS
//********************************************

CameraParam webcam_set;
int dev_video; //number of /dev/video?
string TheIntrinsicFile; //file of intrisincs matrix , camera calibration






//***************************************************
/// Global variables for thresholding
//****************************************************
int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat src_gray, dst;

char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";

/// Function headers
void Threshold_Demo( int, void* )
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */

  threshold( src_gray, dst, threshold_value, max_BINARY_value,threshold_type );

  imshow( "thres", dst );
}
//*****************************END THRESHOLDING *********************************************





int main( int argc, char **argv)
{
try
{
	if (argc < 2) 
	{
	        cerr << "Invalid number of arguments" << endl;
	        cerr << "Usage: (0-1-.. )  [intrinsics-camera-calibration.yml]" << endl;
	        return false;
	}

	dev_video = atoi(argv[1]);
	TheIntrinsicFile = argv[2];

	// read camera parameters if passed
        if (TheIntrinsicFile != "") 
            webcam_set.readFromXMLFile(TheIntrinsicFile);
        




	cvNamedWindow("in", CV_WINDOW_AUTOSIZE); //creazione della finestra
  	//cvNamedWindow( "thres", CV_WINDOW_AUTOSIZE );
	VideoCapture capture(dev_video); //elemento che serve per catturare lo stream video
	if(!capture.isOpened())  return -1;// check if we succeeded


	Mat frame;
	Mat frameTHRES;

	while(1)
	{
		
		capture >> frame;
		capture >> frameTHRES;

		imshow("in", frame);//mostra a video il frame
	

		//qui preraro la finestra per il threshold
		/// Convert the image to Gray
		cvtColor( frameTHRES, src_gray, CV_RGB2GRAY );
  		/// Create Trackbar to choose type of Threshold
  		createTrackbar( trackbar_type, "thres", &threshold_type, max_type, Threshold_Demo );
  		createTrackbar( trackbar_value, "thres", &threshold_value,  max_value, Threshold_Demo );
		/// Call the function to initialize
  		Threshold_Demo( 0, 0 );


			
		char c = cvWaitKey(10); //ogni 10ms viene letto un frame
		if (c == 27) break;//tasto ESC per uscire dal programma
	}



	capture.release(); //viene rilasciata la memoria
	cvDestroyWindow("in"); //e distrutta
	cvDestroyWindow("thres"); //e distrutta


}//end_try
catch(std::exception &ex)
{
	cout<<"eccezione iniziale del main : "<<ex.what()<<endl;
}


	return 0;
//end_main
}





