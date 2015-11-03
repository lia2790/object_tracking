/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/


#include <fstream>
#include <sstream>
#include <iostream>
#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace aruco;



//********************************************************
//	GLOBAL VAR
//********************************************************
cv::Mat InImage;//input image
cv::Mat TheInputImageCopy;//copy the input image 
aruco::CameraParameters CamParam;//camera value of intrinsics matrix
MarkerDetector MDetector;//marker to find in image processing
vector< Marker > Markers;
float MarkerSize = -1;//size of marker side


//********************************************************
//	TIME OF DETECTION
//********************************************************
pair< double, double > AvrgTime(0, 0); // determines the average time required for detection



//********************************************************
//	THRESHOLD AT RUN-TIME
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
    // recompute
    // Detection of the board
  //  float probDetect = TheBoardDetector.detect(TheInputImage);
    InImage.copyTo(TheInputImageCopy);
   // if (TheCameraParameters.isValid() && probDetect > 0.2)
   //     aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheBoardDetector.getDetectedBoard(), TheCameraParameters);


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

	vreader >> InImage;
	
	if (argc >= 3) // read camera parameters if specifed
	{
		CamParam.readFromXMLFile(argv[2]); // resizes the parameters to fit the size of the input image
		CamParam.resize(InImage.size());
	}
       
	if (argc >= 4)  // read marker size if specified
		MarkerSize = atof(argv[3]);
       

	cv::namedWindow("in", 1);
	cv::namedWindow("thres", 1);
	MDetector.getThresholdParams(ThresParam1, ThresParam2);
	iThresParam1 = ThresParam1;
	iThresParam2 = ThresParam2;
	cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, cvTackBarEvents);
	cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, cvTackBarEvents);





	int count_image = 0;


	while(1)
	{	
		vreader >> InImage;
		count_image++;

		 double tick = (double)getTickCount(); // for checking the speed
            
		
        // Ok, let's detect
        MDetector.detect(InImage, Markers, CamParam, MarkerSize);
        // for each marker, draw info and its boundaries in the image


		AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
		AvrgTime.second++;
		cout << "Time detection=" << 1000 * AvrgTime.first / AvrgTime.second << " milliseconds" << endl;


        for (unsigned int i = 0; i < Markers.size(); i++) 
		{
            cout << Markers[i] << endl;
            Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
        }
        // draw a 3d cube in each marker if there is 3d info
        if (CamParam.isValid() && MarkerSize != -1)
            for (unsigned int i = 0; i < Markers.size(); i++) 
            {   CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
       			CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
			}

     
        // show input with augmented information
        cv::imshow("in", InImage);
        // show also the internal image resulting from the threshold operation
        cv::imshow("thres", MDetector.getThresholdedImage());
       
		
		char c = waitKey(10); //ogni 10ms viene letto un frame
		if (c == 27) break;//tasto ESC per uscire dal programma

	}    

	cout<<"image processed are : "<<count_image<<endl;

}catch (std::exception &ex){cout << "Exception :" << ex.what() << endl;}


}//ENDmain





