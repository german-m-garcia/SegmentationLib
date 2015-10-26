/*
 * segment.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: gmartin
 */

#include "segment.h"
#include <vector>
using namespace std;

Segment::Segment(Mat& original):original_(original){
 img_ = Mat::zeros(original.rows,original.cols,CV_8UC3);
}

Segment::~Segment(){

}

Segment::Segment (const Segment &obj):visualFeatures(obj.visualFeatures), img_(obj.img_) ,original_(obj.original_),
		histImage_(obj.histImage_),h_hist(obj.h_hist), s_hist(obj.s_hist), v_hist(obj.v_hist){
   // body of constructor
}

void Segment::addPoint(Point2i& point,Vec3b& colour){

	img_.at<Vec3b>(point.x,point.y) = colour;
}

void Segment::computeFeatures(){
	computeHistogram();
	visualFeatures = Mat(1,NUMBER_VISUAL_FEATS, CV_32FC1);
	vector<Mat> vectorFeats = {h_hist.t(),s_hist.t(),v_hist.t()};
	hconcat(vectorFeats,visualFeatures);
}

void Segment::computeHuMoments() {

}

void Segment::computeHistogram() {
		if (!original_.data) {
			return;
		}

		Mat mask = Mat::zeros(original_.rows,original_.cols,CV_8UC1);
		//threshold grayscale to binary image
		cv::cvtColor(img_, mask, CV_BGR2GRAY);
		mask = mask > 0;
		//imshow("original_",original_);
		//waitKey(0);
		/// Separate the image in 3 places ( B, G and R )
		vector<Mat> bgr_planes;
		split(original_, bgr_planes);

		/// Establish the number of bins
		int histSize = NHISTOGRAMBINS;

		/// Set the ranges ( for B,G,R) )
		float range[] = { 0, NHISTOGRAMBINS };
		const float* histRange = { range };

		bool uniform = true;
		bool accumulate = false;



		/// Compute the histograms:
		calcHist(&bgr_planes[0], 1, 0, mask, h_hist, 1, &histSize, &histRange,
				uniform, accumulate);
		calcHist(&bgr_planes[1], 1, 0, mask, s_hist, 1, &histSize, &histRange,
				uniform, accumulate);
		calcHist(&bgr_planes[2], 1, 0, mask, v_hist, 1, &histSize, &histRange,
				uniform, accumulate);

//		//remove NaNs
//		Mat mask_h = Mat(h_hist != h_hist);
//		h_hist.setTo(0,mask_h);
//		Mat mask_s = Mat(s_hist != s_hist);
//		s_hist.setTo(0,mask_s);
//		Mat mask_v = Mat(v_hist != v_hist);
//		v_hist.setTo(0,mask_v);

		// Draw the histograms for H, S and V
		int hist_w = 120;
		int hist_h = 100;
		int bin_w = cvRound((double) hist_w / histSize);

		histImage_= Mat(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

		/// Normalize the result to [ 0, histImage.rows ]
		normalize(h_hist, h_hist, 0, histImage_.rows, NORM_MINMAX, -1, Mat());
		normalize(s_hist, s_hist, 0, histImage_.rows, NORM_MINMAX, -1, Mat());
		normalize(v_hist, v_hist, 0, histImage_.rows, NORM_MINMAX, -1, Mat());

		/// Draw for each channel
		for (int i = 1; i < histSize; i++) {
			line(histImage_,
					Point(bin_w * (i - 1),
							hist_h - cvRound(h_hist.at<float>(i - 1))),
					Point(bin_w * (i), hist_h - cvRound(h_hist.at<float>(i))),
					Scalar(255, 0, 0), 2, 8, 0);
			line(histImage_,
					Point(bin_w * (i - 1),
							hist_h - cvRound(s_hist.at<float>(i - 1))),
					Point(bin_w * (i), hist_h - cvRound(s_hist.at<float>(i))),
					Scalar(0, 255, 0), 2, 8, 0);
			line(histImage_,
					Point(bin_w * (i - 1),
							hist_h - cvRound(v_hist.at<float>(i - 1))),
					Point(bin_w * (i), hist_h - cvRound(v_hist.at<float>(i))),
					Scalar(0, 0, 255), 2, 8, 0);
		}

	}
