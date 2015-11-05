/*
 * segmenter.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "segmentation.h"
#include "utils.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	Mat original_img,contours_mat, gradient,grayGradient;
	double threshold = 0.01;//0.05;

	if(argc < 4){
		cout <<"Usage: $> ./segmenter <input img> <output path> <scales> [<use gpu, default 0=false>]"<< endl;
	}

	original_img = cv::imread(argv[1], -1);
	std::string outputPath(argv[2]);
	int scales = atoi(argv[3]);
	int starting_scale = 0;
	int gpu = 0;
	if(argc == 5){
		gpu = atoi(argv[4]);
	}
	else if(argc == 6){
		gpu = atoi(argv[4]);
		threshold = atof(argv[5]);
	}

	bool use_gpu = gpu > 0 ? true: false;
	Utils utils;
	utils.tick();
	Segmentation segmentation(original_img,gpu,scales,starting_scale);
	segmentation.segment_pyramid(threshold);

	string text("Scharr segmentation: ");
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
	int show_n_last_scales = 1;
	segmentation.print_results(outMat,show_n_last_scales);
	imwrite(outputPath,outMat);

//	imshow("outMat",outMat);
//	waitKey(0);

//	gradient*= 255.;
//	imwrite(outputPath,gradient);

	//Mat tmp;
	//segmentation.intensity_histogram(gradient,tmp);

}

