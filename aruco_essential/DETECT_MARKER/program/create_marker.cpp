#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "include_src.h"
#include "arucofidmarkers.h"
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    try {
        if (argc < 3) {

            // You can also use ids 2000-2007 but it is not safe since there are a lot of false positives.
            cerr << "Usage: <makerid(0:1023)> outfile.(jpg|png|ppm|bmp) [sizeInPixels:500 default] [locked (0,1) : 0 default]" << endl;
            return -1;
        }
        int pixSize = 500;
        if (argc >= 4)
            pixSize = atoi(argv[3]);
        bool locked = false;
        if (argc >= 5)
            locked = atoi(argv[4]);

        Mat marker = mid::FiducidalMarkers::createMarkerImage(atoi(argv[1]), pixSize, true, locked);

        cv::imwrite(argv[2], marker);

    } catch (std::exception &ex) {
        cout << ex.what() << endl;
    }
}
