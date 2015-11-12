#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "markerdetector.h"
#include "cvdrawingutils.h"
#include "fidmarkers.h"
using namespace cv;
using namespace std;

//****************************
//	VARIABILI 
//****************************
int pixel_default_Size = 500;




int main(int argc, char **argv) 
{    
try {

	if (argc < 3) 
	{
	// You can also use ids 2000-2007 but it is not safe since there are a lot of false positives.
		cerr << "Usage: <makerid(0:1023)> name_OF_outfile.(jpg|png|ppm|bmp) [sizeInPixels:500 default]" << endl;
		return -1;
	}
       
	int pixSize = pixel_default_Size;
	if (argc >= 4)	pixSize = atoi(argv[3]);

	Mat marker = mid::FiducidalMarkers::createMarkerImage(atoi(argv[1]), pixSize, true, false);

	cv::imwrite(argv[2], marker);

	} catch (std::exception &ex) {cout << ex.what() << endl;}
}
