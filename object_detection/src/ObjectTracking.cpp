#include "ObjectTracking.h"

using namespace cv;
namespace mid{

ObjectTracking::ObjectTracking()
{
	MarkerSize = -1;
}

void ObjectTracking::setValue(const char *filename, float Mark_size, Mat InImage)
{
	CamParam.readFromXMLFile(filename); 
	CamParam.resize(InImage.size()); 	

	MarkerSize = Mark_size;
}

void ObjectTracking::ObjectPose(MarkerDetector &MDetector, cv::Mat Img)
{
//-----------while

	// function for detection marker on image
	MDetector.detect(Img, Markers, CamParam, MarkerSize);


	//Try to draw the result
	for (unsigned int i = 0; i < Markers.size(); i++) 
	{
		cout << Markers[i] << endl;//print the value of sigle marker
            Markers[i].draw(Img, Scalar(0, 0, 255), 2);
	}

	// draw a 3d cube in each marker if there is 3d info
	if (CamParam.isValid() && MarkerSize != -1)
		for (unsigned int i = 0; i < Markers.size(); i++) 
            {   
			CvDrawingUtils::draw3dCube(Img, Markers[i], CamParam);
			CvDrawingUtils::draw3dAxis(Img, Markers[i], CamParam);
		}

}

}
