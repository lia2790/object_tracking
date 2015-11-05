
#ifndef _Aruco_BoardDetector_H
#define _Aruco_BoardDetector_H
#include <opencv2/core/core.hpp>
//#include "exports.h"
#include "board.h"
#include "cameraparameters.h"
#include "markerdetector.h"
using namespace std;

namespace mid {

/**\brief This class detects AR boards
 * Version 1.2
 * There are two modes for board detection.
 * First, the old way. (You first detect markers with MarkerDetector and then call to detect in this class.
 *
 * Second: New mode, marker detection is included in the class
 * \code

  CameraParameters CP;
  CP.readFromFile(path_cp)
  BoardConfiguration BC;
  BC.readFromFile(path_bc);
  BoardDetector BD;
  BD.setParams(BC,CP); //or only BD.setParams(BC)
  //capture image
  cv::Mat im;
  capture_image(im);

  float prob=BD.detect(im);
  if (prob>0.3)
        CvDrawingUtils::draw3DAxis(im,BD.getDetectedBoard(),CP);

 \endcode
 *
*/
class BoardDetector {
  public:
    /** See discussion in @see enableRotateXAxis.
     * Do not change unless you know what you are doing
     */
    BoardDetector(bool setYPerpendicular = false);


    /**
     * Use if you plan to let this class to perform marker detection too
     */
    void setParams(const BoardConfiguration &bc, const CameraParameters &cp, float markerSizeMeters = -1);
    void setParams(const BoardConfiguration &bc);
    /**
     * Detect markers, and then, look for the board indicated in setParams()
     * @return value indicating  the  likelihood of having found the marker
     */
    float detect(const cv::Mat &im) throw(cv::Exception);
    /**Returns a reference to the board detected
     */
    Board &getDetectedBoard() { return _boardDetected; }
    /**Returns a reference to the internal marker detector
     */
    MarkerDetector &getMarkerDetector() { return _mdetector; }
    /**Returns the vector of markers detected
     */
    vector< Marker > &getDetectedMarkers() { return _vmarkers; }


    // ALTERNATIVE DETECTION METHOD, BASED ON MARKERS PREVIOUSLY DETECTED

    /** Given the markers detected, determines if there is the board passed
    * @param detectedMarkers result provided by aruco::ArMarkerDetector
    * @param BConf the board you want to see if is present
    * @param Bdetected output information of the detected board
    * @param camMatrix camera matrix with intrinsics
    * @param distCoeff camera distorsion coeff
    * @param camMatrix intrinsic camera information.
    * @param distCoeff camera distorsion coefficient. If set Mat() if is assumed no camera distorion
    * @param markerSizeMeters size of the marker sides expressed in meters
    * @return value indicating  the  likelihood of having found the marker
    */
    float detect(const vector< Marker > &detectedMarkers, const BoardConfiguration &BConf, Board &Bdetected, cv::Mat camMatrix = cv::Mat(),
                 cv::Mat distCoeff = cv::Mat(), float markerSizeMeters = -1) throw(cv::Exception);
    float detect(const vector< Marker > &detectedMarkers, const BoardConfiguration &BConf, Board &Bdetected, const CameraParameters &cp,
                 float markerSizeMeters = -1) throw(cv::Exception);

    /**Static version (all in one). Detects the board indicated
   * @param Image input image
   * @param bc the board you want to see if is present
   * @param cp camera parameters
   * @param markerSizeMeters size of the marker sides expressed in meters (not needed in the board is expressed in meters)
   * @return Board detected
   */
    static Board detect(const cv::Mat &Image, const BoardConfiguration &bc, const CameraParameters &cp, float markerSizeMeters = -1);

    /**
     * By default, the Y axis is set to point up. However this is not the default
     * operation mode of opencv, which produces the Z axis pointing up instead.
     * So, to achieve this change, we have to rotate the X axis.
     */
    void setYPerpendicular(bool enable) { _setYPerpendicular = enable; }
    void setYPerperdicular(bool enable) { setYPerpendicular(enable); } // TODO mark as deprecated
    bool isYPerpendicular() { return _setYPerpendicular; }

    /**Sets the threshold for reprjection test. Pixels that after  estimating the camera location
     * projects 'repj_err_thres' pixels farther from its original location are discarded as outliers.
     * By default it is set to -1, meaning that not reprojection test is performed
     */
    void set_repj_err_thres(float Repj_err_thres) { repj_err_thres = Repj_err_thres; }
    float get_repj_err_thres() const { return repj_err_thres; }


  private:
    void rotateXAxis(cv::Mat &rotation);
    bool _setYPerpendicular;

    //-- Functionality to detect markers inside
    bool _areParamsSet;
    BoardConfiguration _bconf;
    Board _boardDetected;
    float _markerSize, repj_err_thres;
    CameraParameters _camParams;
    MarkerDetector _mdetector; // internal markerdetector
    vector< Marker > _vmarkers; // markers detected in the call to : float  detect(const cv::Mat &im);
};
};
#endif
