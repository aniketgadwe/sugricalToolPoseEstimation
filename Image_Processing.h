


#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

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


class Image_Processing
{
 	public:

	Image_Processing(Mat src);

	void thresholder(Mat src);
	Mat denoising(Mat thresh, int size_variable);// to denoise the image
	Mat getHsvThreshGreen();// to get green threholded image
	Mat getThreshImage();// to get thresholded image
	Mat getgrayImage(); // to get gray image

	private:
	
	Mat img_hsv_thresh_green;
	Mat thresh_img;// store thresholded image
	Mat src_gray; // store gray image
	Mat img_hsv; // store hsv image
};
#endif // IMAGE_PROCESSING_H
