#include "board.h"
#include <cstdio>
#include <opencv2/highgui/highgui.hpp>
#include "arucofidmarkers.h"

using namespace std;
using namespace cv;

int XSize, YSize;//quanti marker per ciascun lato della board
int pixSize = 100;//valore di default se non specificato da terminale
float interMarkerDistance = 0.2;//valore di default se non specificato da terminale
mid::BoardConfiguration BInfo;//informazioni in uscita della board
Mat BoardImage;//conterrà l'immagine da stampare


int main(int argc, char **argv)
{
    
try 
{	if (argc < 4) 
	{
		cerr << "Usage: X:Y boardImage.png boardConfiguration.yml [pixSize] [interMarkerDistance(0,1)]" << endl;
		return -1;
	}
        
    if(sscanf(argv[1], "%d:%d", &XSize, &YSize) != 2) 
	{
		cerr << "Incorrect X:Y specification" << endl;
		return -1;
	}

	if (argc >= 4)
		pixSize = atoi(argv[4]);
	if (argc >= 5)
		interMarkerDistance = atof(argv[5]);
	if ((interMarkerDistance > 1.f) || (interMarkerDistance < 0.f)) 
	{
		cerr << "Incorrect interMarkerDistance '" << interMarkerDistance << "' -- needs to be [0,1]" << endl;
		return -1;
	}




	BoardImage = mid::FiducidalMarkers::createBoardImage(Size(XSize, YSize), pixSize, pixSize * interMarkerDistance, BInfo);




	imwrite(argv[2], BoardImage);
    BInfo.saveToFile(argv[3]);


}catch (std::exception &ex) { cout << ex.what() << endl; }

}


