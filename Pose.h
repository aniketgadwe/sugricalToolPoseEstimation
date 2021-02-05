
#ifndef POSE_H
#define POSE_H

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <math.h>
#include <fstream> 


using namespace cv;
using namespace std;


class Pose
{

	public:
	
	Pose();
	Mat rotation_matrices(double angle);
	Mat quaternion(double angle, double x_dash, double y_dash, double z_dash );
	void contour_detection(Mat thresh_img,Mat src,Mat src_gray,double *green_centroid); //used to detect contours
	double* pose_estimation(const std::vector<cv::Point3f> &objectPoints,const std::vector<cv::Point2f> &imagePoints,double posX, double posY);// used to estimate pose



	private:

	double circumference;//store circumference of tube in mm
	double radius;// store radius of tube in mm 
	double module_width;// square module width in mm 
	double x_dash_values[12];// store x coordinates of module centers from origin
	double y_dash_values[12];// store y coordinates of module centers from origin
	double z_dash_values[12];// store z coordinates of module centers from origin
	double module_adjacent_distance;// distance between two square modules in mm 
	double adjacent_module_center_distance;// distance between two square modules in mm 
	double origin_offset;// distance of first module edge from origin in mm 
	double theta[12]; // storing angles between square module center and global Y axis
	double Focal_length; // storing focal length of camera
	Mat cameraIntrinsicParams; // to store camera parameters 
	Mat distCoeffs;// to store distortion coefficients
	Mat G_R_L;
	Mat quaternion_matrix;
	Mat thresh_img_mask; // to store mask of thresholded image

	vector<Vec4i> hierarchy1;//hierarchy of contours
	vector<Vec4i> hierarchy;//hierarchy of contours
	vector<Vec4i> hierarchy_circle;//to store hierarchy of circle contours
	vector<Vec4i> hierarchy_circle_inscribed;//to store hierarchy of inscribed circle contours
        vector< vector<Point> > contours; // Vector for storing selected contour
  	vector< vector<Point> > contours0;// Vector for storing original contour
        vector< vector<Point> > contours1;// Vector for storing selcted contour
      
      	vector< vector<Point> > contours_circle;// Vector for storing circle contour
	vector< vector<Point> > contours_circle1;// Vector for storing selected  circle contour
	vector< vector<Point> > contours_circle2;
       	vector< vector<Point> > contours_circle_inscribed;// Vector for storing inscribed circle contour
	
	std::vector<Point2f> imagePoints; // vector for storing image points 
       
	Point2f quad_pts[4]; // image points for wrapping
    	Point2f squre_pts[4];// bounding rect co-ordinates
       
        std::vector<Point3f> objectPoints;//store 3D world Points 
        
	
        Mat drawing; // used to store cirlce
	Mat drawing_not_filled;
	Mat drawing_circle; // used to store filled circle 
	Mat drawing_circle_black; // used to store circle with black circle in it
        int j;// used to iterate loop 
	int d;// used to store distance between quadrangles corner
	int r;
        int flag_l;// used to store index of centroid of circle

	Mat transmtx;// get transformation
	Mat transformed;// save transformed image i.e square
	Mat transformed_thresh;

	Rect boundRect;// store bounding rect of quadrangle

	double pixel_distance;// store distance between square module center and green band center
        double World_Distance; // store distance of marker from camera
        double Width; // store calculated real distance between square module center and green band center  
        double ratio; // store ration of width to height
	int minimum;  // store minimum distance 
        int flag; // store index of selected square module
	double mean_diff[10]; // store difference between square modules and green band center 
	double corner_coordinates[10];// store centroid of circle which are used as corner 
        double index[10];// store x,y values of circle centroid in sorted pattern 
	int flag_child_contour_index;// store index of contour which have black circle inscribed in it
	int index_contours_circle2[20];// store index od circle 
	double tip_roll;   


};

#endif // POSE_H
