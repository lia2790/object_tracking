#include "cameraParameters.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;


    /**Empty constructor
     */
CameraParam::CameraParam()
{
    CameraMatrix = cv::Mat();
    Distorsion = cv::Mat();
    CamSize = cv::Size(-1, -1);
}

   /**Copy constructor
     */
CameraParam::CameraParam(const CameraParam &CI)
{
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize = CI.CamSize;

}

   /**Assign operator
    */
CameraParam &CameraParam::operator=(const CameraParam &CI)
{
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize = CI.CamSize;
    return *this;
}

CameraParam::CameraParam(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size) throw(cv::Exception) 
{
	if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3)
        throw cv::Exception(9000, "invalid input cameraMatrix", "CameraParameters::setParams", __FILE__, __LINE__);
    
	cameraMatrix.convertTo(CameraMatrix, CV_32FC1);

        if (distorsionCoeff.total() < 4 || distorsionCoeff.total() >= 7)
        throw cv::Exception(9000, "invalid input distorsionCoeff", "CameraParameters::setParams", __FILE__, __LINE__);	

	distorsionCoeff.convertTo(Distorsion, CV_32FC1);

	CamSize = size;
}

void CameraParam::readFromXMLFile(string filePath)throw(cv::Exception) 
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    int w = -1, h = -1;
    cv::Mat MCamera, MDist;

    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;

    if (MCamera.cols == 0 || MCamera.rows == 0)
        throw cv::Exception(9007, "File :" + filePath + " does not contains valid camera matrix", "CameraParameters::readFromXML", __FILE__, __LINE__);
    if (w == -1 || h == 0)
        throw cv::Exception(9007, "File :" + filePath + " does not contains valid camera dimensions", "CameraParameters::readFromXML", __FILE__, __LINE__);

    if (MCamera.type() != CV_32FC1)
        MCamera.convertTo(CameraMatrix, CV_32FC1);
    else
        CameraMatrix = MCamera;

    if (MDist.total() < 4)
        throw cv::Exception(9007, "File :" + filePath + " does not contains valid distortion_coefficients", "CameraParameters::readFromXML", __FILE__, __LINE__);
    // convert to 32 and get the 4 first elements only
    cv::Mat mdist32;
    MDist.convertTo(mdist32, CV_32FC1);
    //     Distorsion.create(1,4,CV_32FC1);
    //     for (int i=0;i<4;i++)
    //         Distorsion.ptr<float>(0)[i]=mdist32.ptr<float>(0)[i];

    Distorsion.create(1, 5, CV_32FC1);
    for (int i = 0; i < 5; i++)
        Distorsion.ptr< float >(0)[i] = mdist32.ptr< float >(0)[i];
    CamSize.width = w;
    CamSize.height = h;
}
























