/*
 * segmenter.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "Segmentation.h"


using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	Mat img0,contours_mat, gradient,grayGradient;
	double threshold = 0.01;//0.05;
	Segmentation segmentation;
	if(argc < 4){
		cout <<"Usage: $> ./segmenter <input img> <output path> <scales> [<use gpu, default 0=false>]"<< endl;
	}

	img0 = cv::imread(argv[1], -1);
	std::string outputPath(argv[2]);
	int scales = atoi(argv[3]);
	int gpu = 0;
	if(argc == 5){
		gpu = atoi(argv[4]);
	}
	else if(argc == 6){
		gpu = atoi(argv[4]);
		threshold = atof(argv[5]);
	}

	bool use_gpu = gpu > 0 ? true: false;

	segmentation.preprocess(use_gpu,img0, scales);
	cv::Mat outMat(img0.rows, img0.cols * 3, CV_8UC3);
	cout << "> scales =" <<scales << endl;
	cv::Rect rect_orig(0, 0, img0.cols, img0.rows);
	cv::Rect rect_segments(img0.cols, 0, img0.cols, img0.rows);
	cv::Rect rect_edges(img0.cols*2, 0, img0.cols, img0.rows);

	segmentation.scharr_segment(img0,contours_mat,gradient,grayGradient,threshold,true);


//	{
//		double sp  = 15.;
//		double sr  = 15.;
//		double min_size = 15;
//
//		Mat dst;
//
//		segmentation.mean_shift(contours_mat,sp,sr,min_size, dst);
//		imshow("mean shift",dst);
//		waitKey(0);
//	}


	//segmentation.canny_segment(img0,contours_mat,gradient,grayGradient,threshold);
	cout << "> outMat.type()=" <<outMat.type()<<" outMat.size()="<<outMat.size() << endl;
	cout << "> img0.type()=" <<img0.type()<<" img0.size()="<<img0.size() << endl;
	cout << "> contours_mat.type()=" <<contours_mat.type() <<" contours_mat.size()="<<contours_mat.size() << endl;
	//img0.convertTo(img0, CV_32FC3);
	img0.copyTo(outMat(rect_orig));
	contours_mat.copyTo(outMat(rect_segments));

	grayGradient *= 255.;
	//grayGradient.convertTo(img0, CV_8UC3);
    cvtColor(grayGradient,grayGradient,COLOR_GRAY2BGR);
	cout << "> grayGradient.type()=" <<grayGradient.type()<<" grayGradient.size()="<<grayGradient.size() << endl;

	gradient*= 255.;
	gradient.copyTo(outMat(rect_edges));
	imwrite(outputPath,outMat);

//	imshow("outMat",outMat);
//	waitKey(0);

//	gradient*= 255.;
//	imwrite(outputPath,gradient);

	//Mat tmp;
	//segmentation.intensity_histogram(gradient,tmp);

}

