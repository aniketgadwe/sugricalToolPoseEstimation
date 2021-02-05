#include "Centroid_Finder.h"

double* Centroid_Finder::center_finder(Mat thresholded)
{
   
	Moments oMoments = moments(thresholded);
	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;
	static double coordinates[2];
        
	// if the area <= 10000, I consider that the there are no object in the image and it's because of the

	if (dArea > 2000)
	{
		//calculate the position
		double posX_circle= dM10 / dArea;
		double posY_circle= dM01 / dArea;	

     	circle(thresholded, Point(posX_circle,posY_circle),2,Scalar(0,0,0),2);
		//imshow("thresholded: ", thresholded);
		//cout<<"center_X :  "<< posX_circle << "center_Y : " << posY_circle << endl;
		coordinates[0]=posX_circle;
		coordinates[1]=posY_circle;
           
	}

	return coordinates;

}
