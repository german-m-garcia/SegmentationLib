/*
 * Segmentation.h
 *
 *  Created on: Nov 21, 2013
 *      Author: martin
 */

#ifndef ABSSEGMENTATION_H_
#define ABSSEGMENTATION_H_

#include <opencv2/opencv.hpp>
#include <set>

using namespace std;
using namespace cv;

class Segment {

public:
	Segment(Mat&);
	Segment (const Segment &obj);
	virtual ~Segment();
	void addPoint(Point2i& point,Vec3b& colour);

	void computeFeatures();


	const Mat& getImg() const {
		return img_;
	}

	const Mat& getHistImage() const {
		return histImage_;
	}

	float getClassLabel() const {
		return class_label;
	}

	void setClassLabel(float classLabel) {
		class_label = classLabel;
	}

	Mat visualFeatures;
	static const int NHISTOGRAMBINS = 128; //256
	static const int NUMBER_VISUAL_FEATS=NHISTOGRAMBINS*3;

protected:
	//the segment in the original colour
	Mat img_;
	//the original colour img
	Mat original_;
	//the histogram to be displayed
	Mat histImage_;
	//the HSV histograms
	Mat h_hist, s_hist, v_hist;

	float class_label;

	//add some features

	void computeHistogram();

	void computeHuMoments();
};


#endif /* SEGMENTATION_H_ */
