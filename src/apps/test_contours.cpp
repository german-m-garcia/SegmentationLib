/*
 * test_contours.cpp
 *
 *  Created on: 8 Jul 2016
 *      Author: martin
 */



#include "contours/Contours.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  imshow( window_name, dst );
 }


/** @function main */
int main_( int argc, char** argv )
{
  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window
  namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
}

Mat mat2gray(const cv::Mat& src)
{
    Mat dst;
    normalize(src, dst, 0.0, 255.0, cv::NORM_MINMAX, CV_8U);

    return dst;
}



/*
 * main for computing edge orientation
 */
int main_ori(int argc, char* argv[])
{

//	cv::Mat image;
//	if(argc ==2){
//		image = cv::imread(argv[1]);
//	}
//	else{
//		image = cv::Mat::zeros(Size(320, 240), CV_8UC1);
//		cv::circle(image, cv::Point(160, 120), 80, cv::Scalar(255, 255, 255), -1, CV_AA);
//	}
//
//    cv::imshow("original", image);
//
//    cv::Mat Sx;
//    cv::Sobel(image, Sx, CV_32F, 1, 0, 3);
//
//    cv::Mat Sy;
//    cv::Sobel(image, Sy, CV_32F, 0, 1, 3);
//
//    cv::Mat mag, ori;
//    cv::magnitude(Sx, Sy, mag);
//    cv::phase(Sx, Sy, ori, true);
//
//    cv::Mat oriMap = orientationMap(mag, ori, 1.0);
//
//    cv::imshow("magnitude", mat2gray(mag));
//    cv::imshow("orientation", mat2gray(ori));
//    cv::imshow("orientation map", oriMap);
//    cv::waitKey();
//
//    return 0;
}




int main(int argc, char ** argv) {


	Contours contours;
	cv::Mat colour_img, gray_img, edges;

	if(argc != 2){
		std::cout <<"$> test_contours <img>"<<std::endl;
		return -1;
	}

	colour_img = cv::imread(argv[1]);
	/// Reduce noise with a kernel 3x3
	blur( colour_img, colour_img, Size(3,3) );

	/// Convert the image to grayscale
	cvtColor( colour_img, gray_img, CV_BGR2GRAY );


	contours.compute_edges(gray_img, edges);
	contours.trace_contours(colour_img,edges);
	return 0;


}
