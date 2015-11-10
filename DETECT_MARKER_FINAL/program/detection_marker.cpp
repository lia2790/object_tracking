#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>
#include <cstdio>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "include_src.h"
#include "cvdrawingutils.h"

using namespace std;
using namespace cv;
using namespace mid;



//********************************************************
//	GLOBAL VAR
//********************************************************
cv::Mat InImage;//input image
cv::Mat TheInputImageCopy;//copy the input image for manipulate

//use the pinhole camera model
//camera value of intrinsics matrix obtained with opencv calibration example with chessboard
mid::CameraParameters CamParam;
MarkerDetector MDetector;//marker to find in image processing
vector< Marker > Markers;//marker candidate
float MarkerSize = -1;//size of marker side


//********************************************************
//	TIME OF DETECTION
//********************************************************
pair< double, double > AvrgTime(0, 0); // determines the average time required for detection


//********************************************************
// OUTPUT-file Rxyz Txyz - TIME DETECTION - MARKER DETECT
//********************************************************
ofstream file_output;
double matrix_hom[4][4]; // homogeneous_matrix obtained from vector Rodrigues(Rvec) and the traslation vector.


void homogeneous_matrix()
{
	Mat Rot(3, 3, CV_32FC1), Jacob;//matrix of the result

	//for any marker 
	for (unsigned int k = 0; k < Markers.size(); k++) 
	{	
		Rodrigues(Markers[k].Rvec, Rot, Jacob);
		

		//copy the result of Rodrigues function in homogeneus matrix
		for (int i = 0; i < 3; i++)
        	for (int j = 0; j < 3; j++)
         	   matrix_hom[i][j] = Rot.at< float >(i, j);

		matrix_hom[0][3] = Markers[k].Tvec.at< float >(0, 0);
    	matrix_hom[1][3] = Markers[k].Tvec.at< float >(1, 0);
    	matrix_hom[2][3] = Markers[k].Tvec.at< float >(2, 0);

		matrix_hom[3][0] = 0;
		matrix_hom[3][1] = 0;
		matrix_hom[3][2] = 0;
		matrix_hom[3][3] = 1;

			
		file_output<<"Id : "<<Markers[k].id<<endl;
		file_output<<"["; 

		//print of file
		for (int i = 0; i < 4; i++)
        {	for (int j = 0; j < 4; j++)
			  	file_output<<"   "<<matrix_hom[i][j];
	
			if(i==3)file_output<<";]"<<endl;
			else file_output<<";"<<endl;
		}
				
	}
		
}		



//********************************************************
//	THRESHOLD AT RUN-TIME------similar of Opencv example
//********************************************************
void cvTackBarEvents(int pos, void *);
double ThresParam1, ThresParam2;
int iThresParam1, iThresParam2;

void cvTackBarEvents(int pos, void *) 
{
    if (iThresParam1 < 3)
        iThresParam1 = 3;
    if (iThresParam1 % 2 != 1)
        iThresParam1++;
    if (ThresParam2 < 1)
        ThresParam2 = 1;
    ThresParam1 = iThresParam1;
    ThresParam2 = iThresParam2;
    MDetector.setThresholdParams(ThresParam1, ThresParam2);
    
    InImage.copyTo(TheInputImageCopy);
  

    cv::imshow("in", TheInputImageCopy);  
    cv::imshow("thres", MDetector.getThresholdedImage());
}
//*****************************************************




int main(int argc, char **argv) 
{
try 
{
	if (argc < 2) 
	{
		cerr << "Usage: (live|in.avi) [cameraParams.yml] [markerSize]" << endl;
		exit(0);
	}

	// try opening as video
	VideoCapture vreader(0);
	if (vreader.isOpened()) 
	{
		vreader.grab();
		vreader.retrieve(InImage);
	}

	vreader >> InImage;//first image 
	
	if (argc >= 3) // read camera parameters if specifed
	{
		CamParam.readFromXMLFile(argv[2]); 
		CamParam.resize(InImage.size()); // resizes the parameters to fit the size of the input image
	}
       
	if (argc >= 4)  // read marker size if specified
		MarkerSize = atof(argv[3]);
       
	//create a window for show image captured
	cv::namedWindow("in", 1);
	cv::namedWindow("thres", 1);

	//create threshold_bar for choose a runtime the value
	MDetector.getThresholdParams(ThresParam1, ThresParam2);
	iThresParam1 = ThresParam1;
	iThresParam2 = ThresParam2;
	cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, cvTackBarEvents);
	cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, cvTackBarEvents);



	file_output.open("matrix_detection.txt", ios::app);

	


	int count_image = 0;


	while(1)
	{	
		vreader >> InImage; //captured image save in type Mat 
		count_image++;//count image processed by algorithm

		double tick = (double)getTickCount(); // for checking the speed
            
		
        // function for detection marker on image
        MDetector.detect(InImage, Markers, CamParam, MarkerSize);



        
		//estimate the time of detection
		AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
		AvrgTime.second++;
		cout << "Time detection=" << 1000 * AvrgTime.first / AvrgTime.second << " milliseconds" << endl;



		//save the time detection on file
		file_output<< "Time detection = " << 1000 * AvrgTime.first / AvrgTime.second << " milliseconds" << endl;
		homogeneous_matrix();//call function for calculate the homogeneus matrix



		
        for (unsigned int i = 0; i < Markers.size(); i++) 
		{
            cout << Markers[i] << endl;//print the value of sigle marker
            Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (CamParam.isValid() && MarkerSize != -1)
            for (unsigned int i = 0; i < Markers.size(); i++) 
            {   
				CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
       			CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
			}

     
        // show input with augmented information
        cv::imshow("in", InImage);
        // show also the internal image resulting from the threshold operation
        cv::imshow("thres", MDetector.getThresholdedImage());
       

		
		char c = waitKey(10); 
		if (c == 27) break;// ESC for exit

	}    


	file_output<<"image processed are : "<<count_image<<endl;
	cout<<"image processed are : "<<count_image<<endl;
	file_output.close();


}catch (std::exception &ex){cout << "Exception :" << ex.what() << endl;}


}//ENDmain





