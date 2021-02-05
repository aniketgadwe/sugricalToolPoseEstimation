

#include "Pose.h"
#include "Centroid_Finder.h"

Pose::Pose()
{

///////// marker properties /// ( you can change according to nature and type of marker choosen) 

	circumference = 21.0 ; // circumference of tube in mm
	radius = 21.0 / (2 * CV_PI); // radius of tube in mm 
	module_width = 5.0 ; // square module width in mm 
	module_adjacent_distance = 1.5 ; // distance between two square modules in mm 
	adjacent_module_center_distance = module_width + module_adjacent_distance ; // distance between two square modules in mm 
	origin_offset = 4.0; // distance of first module edge from origin in mm 

/////////////////////marker properties end ///////////////////////////////////////////////////////


	for(int a = 0 ; a < 12 ; a++)
	{
		// equation for calculating theta angles made by center of square modules with Y axis of global refrence.
		theta[a] = ((((module_width/2) + (module_adjacent_distance * a)) * 180.0)/(radius * CV_PI)) ; 
		//cout<<" theta[" << a << "] = " << theta[a] << endl;

		// arithmetic progression for calculating x values of square module
		x_dash_values[a] = (origin_offset + (module_width/2)) + (a * adjacent_module_center_distance) ;  	
	}
	
 	/// y' and z' i.e y,z co-ordinates of module center w.r.t global frame origin
	y_dash_values[0] = radius * cos(theta[0]*CV_PI/180.0) ;
	z_dash_values[0] = radius * sin(theta[0]*CV_PI/180.0) ;
	
	y_dash_values[1] = radius * cos(theta[1]*CV_PI/180.0) ;
	z_dash_values[1] = radius * sin(theta[1]*CV_PI/180.0) ;

	y_dash_values[2] = -radius * cos((theta[2] - 90.0)*CV_PI/180.0) ;
	z_dash_values[2] = radius * sin((theta[2] - 90.0)*CV_PI/180.0) ;

	y_dash_values[3] = -radius * cos((theta[3] - 90.0)*CV_PI/180.0) ;
	z_dash_values[3] = radius * sin((theta[3] - 90.0)*CV_PI/180.0) ;
 
	y_dash_values[4] = -radius * cos((theta[4] - 90.0)*CV_PI/180.0) ;
	z_dash_values[4] = radius * sin((theta[4] - 90.0)*CV_PI/180.0) ;

	y_dash_values[5] = -radius * cos((theta[5] - 90.0)*CV_PI/180.0) ;
	z_dash_values[5] = radius * sin((theta[5] - 90.0)*CV_PI/180.0) ;

	y_dash_values[6] = -radius * cos((theta[6] - 180.0)*CV_PI/180.0) ;
	z_dash_values[6] = -radius * sin((theta[6] -180.0)*CV_PI/180.0) ;

	y_dash_values[7] = -radius * cos((theta[7] - 180.0)*CV_PI/180.0) ;
	z_dash_values[7] = -radius * sin((theta[7] - 180.0)*CV_PI/180.0) ;

	y_dash_values[8] = -radius * cos((theta[8] - 180.0)*CV_PI/180.0) ;
	z_dash_values[8] = -radius * sin((theta[8] -180.0)*CV_PI/180.0) ;

	y_dash_values[9] = radius * cos((theta[9] - 270.0)*CV_PI/180.0) ;
	z_dash_values[9] = -radius * sin((theta[9] - 270.0)*CV_PI/180.0) ;

	y_dash_values[10] = radius * cos((theta[10] - 270.0)*CV_PI/180.0) ;
	z_dash_values[10] = -radius * sin((theta[10] - 270.0)*CV_PI/180.0) ;

	y_dash_values[11] = radius * cos((theta[11] - 270.0)*CV_PI/180.0) ;
	z_dash_values[11] = -radius * sin((theta[11] - 270.0)*CV_PI/180.0) ;


	// from camera calibration
 
	Focal_length=794.805415f; 

	cameraIntrinsicParams=Mat(Size(3,3),CV_64FC1);

	// camera intrinsic paramters taken camera calibration file
	cameraIntrinsicParams.at<double>(0,0)= 794.805415f;
	cameraIntrinsicParams.at<double>(0,1)= 0.0;
	cameraIntrinsicParams.at<double>(0,2)= 276.305323f;
	cameraIntrinsicParams.at<double>(1,0)= 0 ;
	cameraIntrinsicParams.at<double>(1,1)= 792.979401f;
	cameraIntrinsicParams.at<double>(1,2)= 206.005075f;
	cameraIntrinsicParams.at<double>(2,0)= 0.0 ;
	cameraIntrinsicParams.at<double>(2,1)= 0.0 ;
	cameraIntrinsicParams.at<double>(2,2)= 1.0 ;

	// camera disstortion coefficients taken camera calibration file

	distCoeffs=Mat(Size(1,5),CV_64FC1);
	distCoeffs.at<double>(0,0)=0.122703;
    distCoeffs.at<double>(0,1)=-0.739647;
    distCoeffs.at<double>(0,2)=-0.006595;
    distCoeffs.at<double>(0,3)=-0.009928;
    distCoeffs.at<double>(0,4)=0.000000;


	j=0; // used to store index for contours 
	d=0; // used to store distance between corners of quadrangle 
	r=0; // used to store index of quadrangles
	flag_l=0;// used to store index of centroids of four circles
	flag=0; //used to store index of square from quadrangle
}

Mat Pose::rotation_matrices(double angle)
{
/*
       	G_R_L = 	| 1  	0  	  0     |
			| 0  cos(phi)  sin(phi) |
			| 0 -sin(phi)  cos(phi) |
			|				
*/
       G_R_L =Mat(Size(3,3),CV_64FC1);
       G_R_L.at<double>(0,0)=  1.0;
       G_R_L.at<double>(0,1)=  0.0;          
       G_R_L.at<double>(0,2)=  0.0;
       G_R_L.at<double>(1,0)=  0.0;
       G_R_L.at<double>(1,1)=  cos(angle*CV_PI/180.0);
       G_R_L.at<double>(1,2)=  sin(angle*CV_PI/180.0);
       G_R_L.at<double>(2,0)=  0.0;
       G_R_L.at<double>(2,1)= -sin(angle*CV_PI/180.0);
       G_R_L.at<double>(2,2)=  cos(angle*CV_PI/180.0);

return G_R_L;
}

Mat Pose::quaternion(double angle, double x_dash, double y_dash, double z_dash )
{ 

	
/*
       quaternion_matrix = 	| 1  	0  	  0      x'|
				| 0  cos(phi)  sin(phi)  y'|
				| 0 -sin(phi)  cos(phi)  z'|
				| 0     0         0      1 |
								
*/
	quaternion_matrix = Mat(Size(4,4),CV_64FC1) ;
	quaternion_matrix.at<double>(0,0)= 1.0;
	quaternion_matrix.at<double>(0,1)= 0.0;
	quaternion_matrix.at<double>(0,2)= 0.0;
	quaternion_matrix.at<double>(0,3)= x_dash;
	quaternion_matrix.at<double>(1,0)= 0.0;
	quaternion_matrix.at<double>(1,1)= cos(angle*CV_PI/180.0);
	quaternion_matrix.at<double>(1,2)= sin(angle*CV_PI/180.0);
	quaternion_matrix.at<double>(1,3)= y_dash;
	quaternion_matrix.at<double>(2,0)= 0.0;
	quaternion_matrix.at<double>(2,1)= -sin(angle*CV_PI/180.0);
	quaternion_matrix.at<double>(2,2)= cos(angle*CV_PI/180.0);
	quaternion_matrix.at<double>(2,3)= z_dash;
	quaternion_matrix.at<double>(3,0)= 0.0;
	quaternion_matrix.at<double>(3,1)= 0.0;
	quaternion_matrix.at<double>(3,2)= 0.0;
	quaternion_matrix.at<double>(3,3)= 1.0;
      
#if 0	
	cout<<"quaternion in: "<<endl; 

      for (int k = 0; k < 4 ; k++) 
	{	
		for( int l = 0; l < 4 ; l++) 
		{
			cout << " " << quaternion_matrix.at<double>(k,l);

		}  
	cout << "" << endl;
	}
        
#endif

	return quaternion_matrix;
}

void  Pose::contour_detection(Mat thresh_img,Mat src,Mat src_gray,double *green_centroid)
{
	double posX = green_centroid[0];
    double posY = green_centroid[1];
	
	Centroid_Finder center; // creating object for Centoid finder
	static double *location;  // store pose pointer
	std::vector<Point2f> preciseCorners(4);// vector for storing precise corners
	thresh_img.copyTo( thresh_img_mask);  // creating deep copy of threholded image

	findContours( thresh_img, contours0, hierarchy1,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0)); // Find the contours in the image
	
	contours.resize(contours0.size()); // size should be same for transfering conoturs *** after removing gives segementation fault
    contours1.resize(contours0.size());

	drawing = Mat::zeros( thresh_img.size(), CV_8UC1 ); // binary image to display passed  quadrangle  
	drawing_not_filled = Mat::zeros( thresh_img.size(), CV_8UC1 );// to store copy of drawing for transforming
	drawing_circle = Mat::zeros( thresh_img.size(), CV_8UC1 );// binary image to display filled white circle 
	drawing_circle_black = Mat::zeros( thresh_img.size(), CV_8UC1 );// binary image to display black circle 
     

	for(int k=0; k < contours0.size(); k++ )
    {    
		// arcLength(Mat(contours0[k]), true)*0.03
		approxPolyDP(contours0[k], contours[k],  arcLength(Mat(contours0[k]), true)*0.02 , true); // approximate the polygons// 
	
		if (contours[k].size() != 4)  // 1st test must have 4 corners
			continue;

		if (!cv::isContourConvex(contours[k])) // 2nd test  to be convex
			continue;
		
		// 3rd test minimum distance between alternative corners 

		for(int m = 0; m < 4; m++)  
		{
			d = (double)(contours[k][m].x - contours[k][(m + 1) % 4].x) *(double)(contours[k][m].x - contours[k][(m + 1) % 4].x) +
					(double)(contours[k][m].y - contours[k][(m + 1) % 4].y) *(double)(contours[k][m].y - contours[k][(m + 1) % 4].y);
	
		}
		
		if (d < 5)    
			continue;
			
			
		if( contours[k].size() == 4 && fabs(contourArea(Mat(contours[k]))) > 200 && fabs(contourArea(Mat(contours[k]))) < 6000 &&
			isContourConvex(Mat(contours[k])) )
		{
			contours1[j]=contours[k]; // store selected quadrangle contours in contours1 
			j=j+1; // increment index
			
		}  
  
	}
	




	if(j>0 & j<5)  // tests on number of selected quadrangles according to known geometry
    {     
            
                 
		for( int i = 0; i< j; i++ )
     	{
            mean_diff[i] = abs(contours1[i][0].x - green_centroid[0] ) ; // distance calculation between green band and square center
        }

		// finding the square module whose distance is minimum from green band if more than 1 square module are detected.	

		if (j!=1)
        {
			minimum = mean_diff[0];
			for (int l=0; l < j ; l++)
			{
				if ( mean_diff[l] < minimum)
				{	
					minimum = mean_diff[l];
					flag = l ; // store index of selected contour
				}
			} 
                         
		}

		else 
		    flag=0;				
		
			Scalar color = Scalar(255,255,255);			                                 	       
            drawContours( drawing, contours1,flag,color,CV_FILLED );
			thresh_img_mask.copyTo(drawing_not_filled,drawing); // making deep copy of drawing 
			
			boundRect=boundingRect(contours1[flag]);  // finding bounding rect of passed square module 
           // orientation of marker staring from II quadrant 
             
	        if(contours1[flag][1].y < contours1[flag][3].y)                      
            {
				// image points
                quad_pts[0]=Point2f(contours1[flag][0].x,contours1[flag][0].y);
    			quad_pts[1]=Point(contours1[flag][1].x,contours1[flag][1].y);
    			quad_pts[2]=Point(contours1[flag][2].x,contours1[flag][2].y); 
    			quad_pts[3]=Point(contours1[flag][3].x,contours1[flag][3].y); 

				// bounding rect points
				squre_pts[0]=Point(boundRect.x,boundRect.y);
				squre_pts[1]=Point(boundRect.x+boundRect.width,boundRect.y);
				squre_pts[2]=Point(boundRect.x+boundRect.width,boundRect.y+boundRect.height);
                squre_pts[3]=Point(boundRect.x,boundRect.y+boundRect.height);                
          	}
         
		 	// orientation of marker staring from I quadrant 
            else
            {
				// image points
				quad_pts[0]=Point2f(contours1[flag][0].x,contours1[flag][0].y);
    			quad_pts[1]=Point(contours1[flag][1].x,contours1[flag][1].y);        
    			quad_pts[2]=Point(contours1[flag][2].x,contours1[flag][2].y);
    			quad_pts[3]=Point(contours1[flag][3].x,contours1[flag][3].y); 
                       
				// bounding rect points
                squre_pts[0]=Point2f(boundRect.x+boundRect.width,boundRect.y);             
				squre_pts[1]=Point(boundRect.x+boundRect.width,boundRect.y+boundRect.height);
                squre_pts[2]=Point(boundRect.x,boundRect.y+boundRect.height);
                squre_pts[3]=Point(boundRect.x,boundRect.y);             	
			}
             		 
			// transformation and wrapping 			 

			transmtx = getPerspectiveTransform(quad_pts,squre_pts); // getting tranform matrix   
			transformed = Mat::zeros(drawing.rows, drawing.cols,CV_8UC1);
			transformed_thresh = Mat::zeros(drawing.rows, drawing.cols,CV_8UC1);
			warpPerspective(drawing, transformed, transmtx, drawing.size());    // drawing rectange 
			warpPerspective(drawing_not_filled, transformed_thresh, transmtx, drawing_not_filled.size()); 
			imshow("drawing ", drawing);
			imshow("transformed ", transformed);
			
			ratio= double(boundRect.width) / double(boundRect.height) ;  // taking ration of width to height 
			cout << "ratio : " << ratio <<  endl;
                        
			if(ratio >0.9 && ratio < 1.25) // test whether detected quadrangle is square
            {
	    
				for (int c=0;c<4;c++)
				{		
					preciseCorners[c] = contours1[flag][c];     // storing in precise corner array
				}
         	                   
				cv::cornerSubPix(src_gray, preciseCorners, cvSize(5,5),cvSize(-1,-1),    
				cvTermCriteria(  CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,30,0.1));//finding accurate corners
                                       
                for (int c=0;c<4;c++)
				{
					contours1[flag][c] = preciseCorners[c];   // storing precise corner in contours list 
				}
                                      
				// storing four points
				Point P1=contours1[flag][0];
				Point P2=contours1[flag][1];
				Point P3=contours1[flag][2];
				Point P4=contours1[flag][3];

								// drawing red color rectangle
				line(src,P1,P2, Scalar(0,0,255),0,CV_AA,0);
				line(src,P2,P3, Scalar(0,0,255),0,CV_AA,0);
				line(src,P3,P4, Scalar(0,0,255),0,CV_AA,0);
				line(src,P4,P1, Scalar(0,0,255),0,CV_AA,0);

///////////////////////////////////////////////////////////// identifing position of white circle containing black circle 
					
                findContours( drawing_not_filled, contours_circle, 
			  	hierarchy_circle,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
                                   
				// finding white circle with black circle inscribed in it		
				for( int i = 0; i< contours_circle.size(); i++) // iterate through each contour.
     			{   
	 
        			Rect r_= boundingRect(contours_circle[i]);
			
        			if(hierarchy_circle[i][2]>0) // checking for child in hierarchy
					{	
            			rectangle(src,Point(r_.x,r_.y), Point(r_.x+r_.width,r_.y+r_.height), Scalar(0,255,255),1,1,0);
						flag_child_contour_index = i;           	
        			}
				}

/////////////////////////////////////////////////////// determining 4 white circle in that square module                                      
						
				findContours( drawing_not_filled, contours_circle_inscribed,
				hierarchy_circle_inscribed,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
                // making size equal needed for storing values
                contours_circle1.resize(contours_circle.size());
				contours_circle2.resize(contours_circle.size()); 
       	
				for(int p=0; p < contours_circle_inscribed.size(); p++ )
		        {    
          
					approxPolyDP(contours_circle_inscribed[p], contours_circle1[p],
					arcLength(Mat(contours_circle_inscribed[p]), true)*0.02, true); // approximate the polygons//  
                       
	              	if (contours_circle1[p].size() < 7 )            // 1st test must have atleast 7 corners
						continue;

	                if (!cv::isContourConvex(contours_circle1[p])) // 2nd test to be convex
						continue;
                	
					if( contours_circle1[p].size() >= 7 && fabs(contourArea(Mat(contours_circle1[p]))) > 100) 
					// area test to avoid noise
                   	{
                       
                  		contours_circle2[r]=contours_circle1[p]; // store contours
						index_contours_circle2[r]=p; // storing index 
        	        	r=r+1;
			
					}
	
		        }
   
		       	for(int k=0; k < r; k++)
				{

					if(index_contours_circle2[k] == flag_child_contour_index )
			        	flag_child_contour_index = k ;   // storing index of circle contour which have black circle
				}
     		
                // getting centroids of all 4 circles	
				for (int l=0; l < r ; l++)
                {
		         
					Scalar color_circle = Scalar(255,255,255);                        
                   	drawContours( drawing_circle, contours_circle2,l,color_circle,CV_FILLED);
					drawContours(drawing_circle_black, contours_circle2,l,color_circle,CV_FILLED);
				               
				  	double *centroid_coordinates = center.center_finder(drawing_circle); // calling from class Centroid_Finder
					drawing_circle = Mat::zeros( thresh_img.size(), CV_8UC1 );
                         
				   	corner_coordinates[l+flag_l]=centroid_coordinates[0];//x
				   	corner_coordinates[l+flag_l+1]=centroid_coordinates[1];//y	
        	        flag_l=flag_l + 1;
			              
                }
			
				// arranging the order of centroids of circle in following order in an image ref.

/*			  (0,0)	-----------------x     ( image frame coordinate)
				|	
				|      (x3,y3) = (index[4],index[5])------>-----------> (x4,y4) = (index[6],index[7]) 
				|                         |
				|			  ^----------------<----------<	
				|    						      |		
				|      (x1,y1) = (index[0],index[1])------>-----------^ (x2,y2) = (index[2],index[3]) 
                                y
*/

				if(corner_coordinates[0]<corner_coordinates[2])
				{
					index[0]=corner_coordinates[0];	
					index[1]=corner_coordinates[1];
					index[2]=corner_coordinates[2];	
					index[3]=corner_coordinates[3];
				}	
				else
				{	
					index[0]=corner_coordinates[2];	
					index[1]=corner_coordinates[3];
					index[2]=corner_coordinates[0];	
					index[3]=corner_coordinates[1];
				}
				
				if(corner_coordinates[4]<corner_coordinates[6])
				{
					index[4]=corner_coordinates[4];	
					index[5]=corner_coordinates[5];
					index[6]=corner_coordinates[6];	
					index[7]=corner_coordinates[7];
				}	
				else
				{	
					index[4]=corner_coordinates[6];	
					index[5]=corner_coordinates[7];
					index[6]=corner_coordinates[4];	
					index[7]=corner_coordinates[5];
				}											
		
				// store imagepoints in a vector format
				imagePoints.clear();
				imagePoints.push_back(Point2f(index[0],index[1]));
				imagePoints.push_back(Point2f(index[2],index[3]));
				imagePoints.push_back(Point2f(index[4],index[5]));
				imagePoints.push_back(Point2f(index[6],index[7]));
			               
				Point2f C1 =  Point2f(index[0],index[1]);                
				Point2f C2 =  Point2f(index[2],index[3]);
				Point2f C3 =  Point2f(index[4],index[5]);
				Point2f C4 =  Point2f(index[6],index[7]);

				// getting correct point to point correspsondence
									
//////////////////////0 R condition //////////  (check report for more details)
				if(( flag_child_contour_index == 1  || flag_child_contour_index == 0 )
						 && corner_coordinates[flag_child_contour_index*2] == index[0]
						 && corner_coordinates[(flag_child_contour_index*2)+1] == index[1] ) 
				           
				{
	
					objectPoints.clear();
					objectPoints.push_back(cv::Point3f(-1.2f,1.174383843f,-0.2131192462f));//p1
                    objectPoints.push_back(cv::Point3f(-1.2f,-1.174383843f,-0.2131192462f));//p4
					objectPoints.push_back(cv::Point3f(1.2f,1.174383843f,-0.2131192462f));//p2
					objectPoints.push_back(cv::Point3f(1.2f,-1.174383843f,-0.2131192462f)); //p3


				}
					
					
///////////////////1 G condition //////////////////////////////


				else if ((flag_child_contour_index == 0 || flag_child_contour_index == 1  )
						  && corner_coordinates[flag_child_contour_index*2] == index[2]
						  && corner_coordinates[(flag_child_contour_index*2)+1] == index[3] )
				{

					objectPoints.clear();
					objectPoints.push_back(cv::Point3f(1.2f,1.174383843f,-0.2131192462f));//p2
					objectPoints.push_back(cv::Point3f(-1.2f,1.174383843f,-0.2131192462f));//p1
					objectPoints.push_back(cv::Point3f(1.2f,-1.174383843f,-0.2131192462f));//p3
					objectPoints.push_back(cv::Point3f(-1.2f,-1.174383843f,-0.2131192462f));//p4
				
				}


///////////////////2 B condition ////////////

				else if( ( flag_child_contour_index == 3  ||  flag_child_contour_index == 2 )
						 && corner_coordinates[flag_child_contour_index*2] == index[4]
						 && corner_coordinates[(flag_child_contour_index*2)+1] == index[5] ) 
				{

					objectPoints.clear();
					objectPoints.push_back(cv::Point3f(-1.2f,-1.174383843f,-0.2131192462f));//p4
                    objectPoints.push_back(cv::Point3f(1.2f,-1.174383843f,-0.2131192462f));//p3
					objectPoints.push_back(cv::Point3f(-1.2f,1.174383843f,-0.2131192462f));//p1
					objectPoints.push_back(cv::Point3f(1.2f,1.174383843f,-0.2131192462f));//p2

				}
					
					
					
////////////////////////3 Y condition //////////////////					
				else if(( flag_child_contour_index == 3 || flag_child_contour_index == 2 ) 
						&& corner_coordinates[flag_child_contour_index*2] == index[6] 
						&& corner_coordinates[(flag_child_contour_index*2)+1] == index[7] )
				{

					objectPoints.clear();
					objectPoints.push_back(cv::Point3f(1.2f,-1.174383843f,-0.2131192462f));//p3
                    objectPoints.push_back(cv::Point3f(1.2f,1.174383843f,-0.2131192462f));//p2
					objectPoints.push_back(cv::Point3f(-1.2f,-1.174383843f,-0.2131192462f));//p4
					objectPoints.push_back(cv::Point3f(-1.2f,1.174383843f,-0.2131192462f));//p1

				}


////////////////condition end////////////


				// drawing circles at centroid
				circle(src, Point(C1.x,C1.y),2,Scalar(0,0,255),2);
				circle(src, Point(C2.x,C2.y),2,Scalar(0,255,0),2);
				circle(src, Point(C3.x,C3.y),2,Scalar(255,0,0),2);
				circle(src, Point(C4.x,C4.y),2,Scalar(0,255,255),2);
					
				// drawing yellow lines
				line(src,C1,C2,Scalar(0,255,255),0,CV_AA,0);
				line(src,C2,C3, Scalar(0,255,255),0,CV_AA,0);
				line(src,C3,C4, Scalar(0,255,255),0,CV_AA,0);
				line(src,C4,C1, Scalar(0,255,255),0,CV_AA,0);
					
				// getting pose from pose estimation function 
				location=pose_estimation(objectPoints,imagePoints,posX,posY); 
				//if(location[0]!=0 && location[1]!=0 && location[2]!=0 && location[3]!=0 && location[4]!=0 
				// && location[5]!=0 ) // condition to check if no pose is estimated
				//{
				cout << "//////////////////////////// " << endl;
				cout << " " << endl; 
				cout<<" X_tip : "<< location[0] <<endl;
				cout<<" Y_tip : "<< location[1] <<endl;
				cout<<" Z_tip : "<< location[2] <<endl;
				cout<<"Roll_tip : "<<  location[3]<<endl;
				cout<<"Pitch_tip : "<< location[4] <<endl;
				cout<<"Yaw_tip : "<< location[5] <<endl;
				cout << "//////////////////////////// " << endl;
				cout << " " << endl;
				
				std::ofstream ofs;
				ofs.open ("./pose/pose.ods", std::ofstream::out | std::ofstream::app);    
		
				ofs << "\n 	"<<location[0] << "	"<<location[1]<<"	"<<location[2]<<"	"<<location[3]<<"	"<<location[4]<<"	"<<location[5]<<"	"<<sum;
                 
 	 			ofs.close();

                                        //} 
			}
		}
	imshow("src",src); // showing source image
}

double* Pose::pose_estimation(const std::vector<cv::Point3f> &objectPoints,const std::vector<cv::Point2f> &imagePoints,double posX, double posY)
{
  
    static double position[6]; // for storing local pose 
    static double tip_pose[6]={0.0,0.0,0.0,0.0,0.0,0.0} ;// for storing tip pose	
    static double empty_pose[]={0.0,0.0,0.0,0.0,0.0,0.0} ; // used to return when no pose is estimated 
	    
			       	
	if(objectPoints.size() == imagePoints.size()) // conditon if points are not same
	{
							
			Mat rvec,tvec,inliers;
			bool useExtrinsicGuess =false; 
			int method = CV_ITERATIVE ;
			int iterationsCount = 500;        // number of Ransac iterations.
			float reprojectionError = 8;    // maximum allowed distance to consider it an inlier.
			float confidence = 0.95;          // ransac successful confidence.
			int flags=0;

			cv::solvePnPRansac(objectPoints, imagePoints,cameraIntrinsicParams,distCoeffs,rvec,tvec,
										useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
									inliers, flags ); // solving PnP problem using Ransac

			Mat J;
			vector<Point2f> p(3);
			projectPoints(objectPoints,rvec,tvec, cameraIntrinsicParams,distCoeffs, p, J);// projecting image points to get pixel error

			// finding pixel error between detected image points and projected image points  
			float sum = 0.;
			for (size_t i = 0; i <p.size(); i++)
			{
				sum += sqrt((p[i].x - imagePoints[i].x)* (p[i].x - imagePoints[i].x)+
						(p[i].y - imagePoints[i].y)* (p[i].y - imagePoints[i].y));
			}

				
				
			Mat L_R_C=Mat(Size(3,3),CV_64FC1);
			Mat jacobian=Mat(Size(3,1),CV_64FC1);

			Rodrigues(rvec,L_R_C,jacobian);
			Mat quat_matrix=Mat(Size(4,4),CV_64FC1);
			//getEulerAngles(distant,eulerAngles);


			Mat translation = L_R_C	* tvec ;
			Mat R= L_R_C.t(); 
			Mat T = -R * tvec ;
				
			Mat translation_homogenous  = Mat(Size(1,4),CV_64FC1); 
			translation_homogenous.at<double>(0,0) = T.at<double>(0,0);
			translation_homogenous.at<double>(1,0) = T.at<double>(1,0);
			translation_homogenous.at<double>(2,0) = T.at<double>(2,0);
			translation_homogenous.at<double>(3,0) = 1.0;

			
			position[0] =T.at<double>(0,0);  // x 
			position[1]=T.at<double>(1,0);   // y
			position[2]=T.at<double>(2,0);   // z 
			position[3] = ((atan2(-R.at<double>(2,1), R.at<double>(2,2)))*180)/3.142;  //roll
			position[4] = ((asin(R.at<double>(2,0)))*180)/3.142;                       //pitch
			position[5] = ((atan2(-R.at<double>(1,0), R.at<double>(0,0)))*180)/3.142;   //yaw

			//getting roll from +ve x axis. 

			if(position[3] > 0 )
				position[3]=position[3] - 180;
			else
				position[3]= position[3] + 180;

			// determine pixel distance between detected square module center and green band center 
			pixel_distance= sqrt((((boundRect.x+(boundRect.width/2)) - posX )* ((boundRect.x+(boundRect.width/2)) - posX ) )  + (((boundRect.y+(boundRect.height/2)) - posY ) * ((boundRect.y+(boundRect.height/2)) - posY ) )) ;  
				
			World_Distance = position[2];
			// finding real distance between detected square module center and green band center 
			Width = (pixel_distance * World_Distance) / Focal_length ;
			cout<<"Width: "<< Width << endl;
			// transformations are applied to local pose (check documentations to understand which transformations are used)
			
			Mat G_R_C=Mat(Size(3,3),CV_64FC1);
			Mat translation_global_frame;//= Mat(Size(4,1),CV_64FC1);
			Mat G_R_L_;

			// width conditions are used to determine which square number is selected 
			if(Width > 4.0  && Width < 12.0 )
			{

			/////////////////////////////////////////////////////////////

				tip_roll = theta[0] - position[3] ; //roll	
			
				quat_matrix =  quaternion(450.0 - theta[0], x_dash_values[0], y_dash_values[0], z_dash_values[0] );//get quaternion matrix

				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[0]); // get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}

			else if(Width >12.0 && Width <18.0 )
			{
				
			
			/////////////////////////////////////////////////////////////

				tip_roll = theta[1] - position[3] ; //roll

				quat_matrix =  quaternion(450.0 - theta[1], x_dash_values[1], y_dash_values[1], z_dash_values[1] );//get quaternion matrix
				
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[1]); // get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////
			}

			else if(Width > 18.0 && Width <24.0 )
			{


			/////////////////////////////////////////////////////////////

				tip_roll = theta[2] - position[3] ; //roll				
									
				quat_matrix =  quaternion(450.0 - theta[2], x_dash_values[2], y_dash_values[2], z_dash_values[2] );//get quaternion matrix
				
				translation_global_frame = quat_matrix * translation_homogenous;
				

				G_R_L_ = rotation_matrices(450.0 - theta[2]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////
			
			}  
	
			else if(Width >24 && Width <31.0 )
			{
			

			/////////////////////////////////////////////////////////////

				tip_roll = theta[3] - position[3] ; //roll	

				quat_matrix =  quaternion(450.0 - theta[3], x_dash_values[3], y_dash_values[3], z_dash_values[3] );//get quaternion matrix
				
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[3]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}

			else if(Width >32.0 && Width <37.0 )
			{

			/////////////////////////////////////////////////////////////

				tip_roll = theta[4] - position[3] ; //roll	
		
				quat_matrix =  quaternion(450.0 - theta[4], x_dash_values[4], y_dash_values[4], z_dash_values[4] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[4]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}

			else if(Width >39.0 && Width <44.0 )
			{

			
			/////////////////////////////////////////////////////////////

				tip_roll = theta[5] - position[3] ; //roll	

				quat_matrix =  quaternion(450.0 - theta[5], x_dash_values[5], y_dash_values[5], z_dash_values[5] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[5]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}

			else if(Width >45.0 && Width <50.0 )
			{

			
			/////////////////////////////////////////////////////////////

				tip_roll = theta[6] - position[3] ; //roll

				quat_matrix =  quaternion(450.0 - theta[6], x_dash_values[6], y_dash_values[6], z_dash_values[6] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[6]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}

			else if(Width > 51.0 && Width < 56.0 )
			{


			/////////////////////////////////////////////////////////////

				tip_roll = theta[7] - position[3] ; //roll					
				
				quat_matrix =  quaternion(450.0 - theta[7], x_dash_values[7], y_dash_values[7], z_dash_values[7] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[7]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////


			}
							
			else if(Width > 58.0 && Width <63.0 )
			{

	
			/////////////////////////////////////////////////////////////

				tip_roll = theta[8] - position[3] ; //roll		
			
				quat_matrix =  quaternion(450.0 - theta[8], x_dash_values[8], y_dash_values[8], z_dash_values[8] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[8]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////


			}

			else if(Width >64.0 && Width <69.0 )
			{

			
			/////////////////////////////////////////////////////////////

				tip_roll = theta[9] - position[3] ; //roll
		
				quat_matrix =  quaternion(450.0 - theta[9], x_dash_values[9], y_dash_values[9], z_dash_values[9] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[9]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}
		
			else if(Width >71.0 && Width < 76.0 )
			{

			
			/////////////////////////////////////////////////////////////

				tip_roll = theta[10] - position[3] ; //roll

				quat_matrix =  quaternion(450.0 - theta[10], x_dash_values[10], y_dash_values[10], z_dash_values[10] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[10]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}

			else if(Width >78.0 && Width <82.0)
			{
			

			/////////////////////////////////////////////////////////////

				tip_roll = theta[11] - position[3] ; //roll

				quat_matrix =  quaternion(450.0 - theta[11], x_dash_values[11], y_dash_values[11], z_dash_values[11] );//get quaternion matrix
				translation_global_frame = quat_matrix * translation_homogenous;

				G_R_L_ = rotation_matrices(450.0 - theta[11]);// get rotation matrix between global frame and local  frame
				G_R_C = G_R_L_ * L_R_C;// get rotation matrix between global frame and camera frame

			/////////////////////////////////////////////////////////////

			}

			if (Width > 4.0  && Width < 12.0  || Width >12.0 && Width <18.0 || Width > 18.0 && Width <24.0 || Width >24.0 && Width <31.0 || Width >32.0 && Width <37.0 || Width >39.0 && Width <44.0 || Width >45.0 && Width <50.0 || Width > 51.0 && Width < 56.0 || Width > 58.0 && Width <63.0 || Width >64.0 && Width <69.0 || Width >71.0 && Width < 76.0 || Width >78.0 && Width <82.0 )
			{
				Mat global_rotation = G_R_C.t();
				tip_pose[0] = translation_global_frame.at<double>(0,0);
				tip_pose[1] = translation_global_frame.at<double>(1,0);
				tip_pose[2] = translation_global_frame.at<double>(2,0);
				tip_pose[3] =  tip_roll;//roll
				tip_pose[4] = ((asin(global_rotation.at<double>(2,0)))*180)/3.142;  //pitch
				tip_pose[5] = ((atan2(-global_rotation.at<double>(1,0),global_rotation.at<double>(0,0)))*180)/3.142;   //yaw
			}

			return tip_pose;
	}
	
	return empty_pose;	
}


