#include <opencv2/core/core.hpp>
#include <string>

using namespace std;

class CameraParam
{
	 
public:
    // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Mat CameraMatrix;
    // 4x1 matrix (k1,k2,p1,p2)
    cv::Mat Distorsion;
    // size of the image
    cv::Size CamSize;

    /**Empty constructor
     */
    CameraParam();

   /**Copy constructor
     */
    CameraParam(const CameraParam &CI);

   /**Assign operator
    */
    CameraParam &operator=(const CameraParam &CI);

     /**Creates the object from the info passed
     * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
     * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
     * @param size image size
     */
    CameraParam(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size) throw(cv::Exception);


 /**Reads from a YAML file generated with the opencv2.2 calibration utility
     */
    void readFromXMLFile(string filePath) throw(cv::Exception);
};


