#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <math.h>
#include <fstream> 
#include "Image_Processing.h"
#include "Pose.h"
#include "Centroid_Finder.h"

using namespace cv;
using namespace std;



int main( int argc, char** argv )
{

	system("mkdir -p pose"); // create directory for pose

	VideoCapture cap(1);   // capture video from external camera 
	if ( !cap.isOpened() ) // if not success, exit program
	{
	cout << "Cannot open the video file" << endl;
	return -1;
	}

	// open the video file for reading
	
	while(1)
	{
		Mat src; 
		bool bSuccess = cap.read(src); // read a new frame from video
		if (!bSuccess) //if not success, break loop
			break;
  
		Image_Processing thresholder_object(src); // declare object for Image_Processing class
		
		Centroid_Finder centeroid;// declare object for Centroid_Finder class

		//getting cntroid
		double *green_centroid = centeroid.center_finder(thresholder_object.denoising(thresholder_object.getHsvThreshGreen(),5));
	        
		Pose estimator;// declare object for Pose class

		//getting pose estimation
		estimator.contour_detection(thresholder_object.denoising(thresholder_object.getThreshImage(),3),src,thresholder_object.getgrayImage(),green_centroid);
		
       
	
  		if(waitKey(3)==27) 
			break;   //  Wait until user exit program by pressing a key
  	}

  return 0;
 }



