/*
 * segmenter.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "segmentation.h"
#include "ms_segmentation.h"
#include "utils.h"

using namespace std;
using namespace cv;

int scharrmain(int argc, char** argv) {

	string text("Scharr segmentation: ");
	Mat original_img,contours_mat, gradient,grayGradient;
	double threshold = 0.01;//0.05;
	Utils utils;
	int scales = 3;
	int starting_scale = 0;
	int gpu = 0;
	string input_img_path,output_path;
	int null;
	utils.parse_args(argc,argv,threshold,scales,starting_scale, null,gpu,input_img_path,output_path);

	cout <<"starting scale set to:"<<starting_scale<<endl;

	original_img = cv::imread(input_img_path, -1);

	utils.tick();
	Segmentation segmentation(original_img,gpu,scales,starting_scale);
	segmentation.segment_pyramid(threshold);
	segmentation.map_segments(0);


	utils.tock(text);

	//show some segments

//	namedWindow("segment",WINDOW_NORMAL );
//	for(Segment* seg: segs){
//		Mat display = Mat::zeros(original_img.rows,original_img.cols,CV_8UC3);
//		const Rect& rect = seg->getBoundRect();
//		display(rect) = display(rect) + seg->getRandomColourMat();
//		imshow("segment",display);
//		waitKey(0);
//	}
	//return 0;


	Mat outMat;
	int show_n_last_scales = -1;//(scales-starting_scale);
	segmentation.print_results(outMat,show_n_last_scales);
	imwrite(output_path,outMat);

	//imshow("outMat",outMat);
	//waitKey(0);

//	gradient*= 255.;
//	imwrite(outputPath,gradient);

	//Mat tmp;
	//segmentation.intensity_histogram(gradient,tmp);

}

int main(int argc, char** argv) {

	string text("Mean Shift segmentation: ");
	Mat original_img,contours_mat, gradient,grayGradient;
	double threshold = 0.01;//0.05;
	Utils utils;
	int scales = 3;
	int starting_scale = 0;
	int gpu = 0;
	string input_img_path,output_path;
	int null;
	utils.parse_args(argc,argv,threshold,scales,starting_scale, null,gpu,input_img_path,output_path);

	cout <<"starting scale set to:"<<starting_scale<<endl;

	original_img = cv::imread(input_img_path, -1);

	utils.tick();
	MSSegmentation segmentation(original_img,gpu,scales,starting_scale);
	segmentation.segment_pyramid(threshold);
	segmentation.map_segments(0);


	utils.tock(text);



	Mat outMat;
	int show_n_last_scales = -1;//(scales-starting_scale);
	segmentation.print_results(outMat,show_n_last_scales);
	imwrite(output_path,outMat);

	//imshow("outMat",outMat);
	//waitKey(0);

//	gradient*= 255.;
//	imwrite(outputPath,gradient);

	//Mat tmp;
	//segmentation.intensity_histogram(gradient,tmp);
	return 0;

}

