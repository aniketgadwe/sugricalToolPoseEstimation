

#ifndef CENTROID_FINDER_H
#define CENTROID_FINDER_H

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <math.h>
#include <fstream> 


using namespace std;
using namespace cv;

class Centroid_Finder
{

	public:

	double* center_finder(Mat thresholded); // finds centroid


};

#endif // CENTROI_FINDER_H
