
#ifndef _ArUco_DrawUtils_H_
#define _ArUco_DrawUtils_H_
//#include "exports.h"
#include "include_src.h"
namespace mid {
/**\brief A set of functions to draw in opencv images
 */
class CvDrawingUtils {
  public:
    static void draw3dAxis(cv::Mat &Image, Marker &m, const CameraParameters &CP);

    static void draw3dCube(cv::Mat &Image, Marker &m, const CameraParameters &CP, bool setYperpendicular = false);

   // static void draw3dAxis(cv::Mat &Image, Board &m, const CameraParameters &CP);

   // static void draw3dCube(cv::Mat &Image, Board &m, const CameraParameters &CP, bool setYperpendicular = false);
};
};

#endif
