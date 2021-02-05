

#include "Image_Processing.h"

Image_Processing::Image_Processing(Mat src)
{
	
        thresholder(src);
}

void Image_Processing::thresholder(Mat src)
{
	cvtColor( src, src_gray, CV_BGR2GRAY );
	cvtColor(src, img_hsv, COLOR_BGR2HSV);

	inRange(img_hsv, Scalar(27, 50, 77), Scalar(87, 188, 234), img_hsv_thresh_green); //green threshold in the image
	//imshow("img_hsv_thresh_green",img_hsv_thresh_green); 
	threshold(src_gray, thresh_img, 25 , 255, CV_THRESH_BINARY | CV_THRESH_OTSU );//Otsu threshold in the image
    imshow("thresh_img", thresh_img);
    
    #if 0
        
        Mat z = src_gray;
        Mat M = Mat_<double>(z.rows*z.cols,6);
        Mat I=Mat_<double>(z.rows*z.cols,1);
        for (int i=0;i<z.rows;i++)
        {
            for (int j = 0; j < z.cols; j++)
            {
                double x=(j - z.cols / 2) / double(z.cols),y= (i - z.rows / 2) / double(z.rows);
                M.at<double>(i*z.cols+j, 0) = x*x;
                M.at<double>(i*z.cols+j, 1) = y*y;
                M.at<double>(i*z.cols+j, 2) = x*y;
                M.at<double>(i*z.cols+j, 3) = x;
                M.at<double>(i*z.cols+j, 4) = y;
                M.at<double>(i*z.cols+j, 5) = 1;
                I.at<double>(i*z.cols+j, 0) = z.at<uchar>(i,j);
            }
        }
        SVD s(M);
        Mat q;
        s.backSubst(I,q);
        //cout<<q;
        //imshow("Orignal",z);
        //cout<<q.at<double>(2,0);
        Mat background(z.rows,z.cols,CV_8UC1);
        for (int i=0;i<z.rows;i++)
        {
            for (int j = 0; j < z.cols; j++)
            {
                double x=(j - z.cols / 2) / double(z.cols),y= (i - z.rows / 2) / double(z.rows);
                double quad=q.at<double>(0,0)*x*x+q.at<double>(1,0)*y*y+q.at<double>(2,0)*x*y;
                quad+=q.at<double>(3,0)*x+q.at<double>(4,0)*y+q.at<double>(5,0);
                background.at<uchar>(i,j) = saturate_cast<uchar>(quad);
            }
        }
        
        //imshow("Simulated background",background);
        Mat diff;
        absdiff(background,z,diff);
        double mind,maxd;
        minMaxLoc(diff,&mind,&maxd);
        Mat background_sub;
        background_sub =  diff*(256/(maxd-mind));
        imshow("background original ", background_sub );
    #endif
}	

Mat Image_Processing::denoising(Mat thresh, int size_variable)
{
	// morphological filters	
	erode(thresh,thresh, getStructuringElement(MORPH_ELLIPSE, Size(size_variable, size_variable )) );
 	dilate(thresh,thresh, getStructuringElement(MORPH_ELLIPSE, Size(size_variable,size_variable)) );
 	
	dilate(thresh,thresh, getStructuringElement(MORPH_ELLIPSE, Size(size_variable, size_variable)) );
    erode(thresh,thresh, getStructuringElement(MORPH_ELLIPSE, Size(size_variable, size_variable )) );

	return thresh;
}

Mat Image_Processing::getHsvThreshGreen()
{
	return img_hsv_thresh_green;

}

Mat Image_Processing::getThreshImage()
{
	return thresh_img;

}
Mat Image_Processing::getgrayImage()
{
	return src_gray;

}

