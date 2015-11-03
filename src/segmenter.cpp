/*
 * segmenter.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "segmentation.h"


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
	int gpu = 0;
	if(argc == 5){
		gpu = atoi(argv[4]);
	}
	else if(argc == 6){
		gpu = atoi(argv[4]);
		threshold = atof(argv[5]);
	}

	bool use_gpu = gpu > 0 ? true: false;
	Segmentation segmentation(original_img,gpu,scales);
	segmentation.segment_pyramid(threshold);

	//show some segments
	const vector<Segment*>& segs = segmentation.getSegmentsPyramid()[1];
//	namedWindow("segment",WINDOW_NORMAL );
//	for(Segment* seg: segs){
//		Mat display = Mat::zeros(original_img.rows,original_img.cols,CV_8UC3);
//		const Rect& rect = seg->getBoundRect();
//		display(rect) = display(rect) + seg->getRandomColourMat();
//		imshow("segment",display);
//		waitKey(0);
//	}
	return 0;


	vector<Mat> outputPyr = segmentation.getOutputSegmentsPyramid();

	cv::Mat outMat(original_img.rows, original_img.cols * 4, CV_8UC3);
	cout << "> scales =" <<scales << endl;
	cv::Rect rect_orig(0, 0, original_img.cols, original_img.rows);
	cv::Rect rect_level_0(original_img.cols, 0, original_img.cols, original_img.rows);
	cv::Rect rect_level_1(original_img.cols*2, 0, original_img.cols, original_img.rows);
	cv::Rect rect_level_2(original_img.cols*3, 0, original_img.cols, original_img.rows);


	//img0.convertTo(img0, CV_32FC3);

	segmentation.getBilateralFilteredPyramid()[0].copyTo(outMat(rect_orig));


//	grayGradient *= 255.;
//	//grayGradient.convertTo(img0, CV_8UC3);
//    cvtColor(grayGradient,grayGradient,COLOR_GRAY2BGR);
//	cout << "> grayGradient.type()=" <<grayGradient.type()<<" grayGradient.size()="<<grayGradient.size() << endl;
//
//	gradient*= 255.;
	Mat tmp_out_0,tmp_out_1,tmp_out_2;

	resize(outputPyr[1],tmp_out_0,outputPyr[0].size());
	resize(outputPyr[2],tmp_out_1,outputPyr[0].size());
	resize(outputPyr[3],tmp_out_2,outputPyr[0].size());
	tmp_out_0.copyTo(outMat(rect_level_0));
	tmp_out_1.copyTo(outMat(rect_level_1));
	tmp_out_2.copyTo(outMat(rect_level_2));
	imwrite(outputPath,outMat);

//	imshow("outMat",outMat);
//	waitKey(0);

//	gradient*= 255.;
//	imwrite(outputPath,gradient);

	//Mat tmp;
	//segmentation.intensity_histogram(gradient,tmp);

}

