/*
 * Thinner.h
 *
 *  Created on: 7 Dec 2015
 *      Author: martin
 */

#ifndef SRC_THINNING_THINNER_H_
#define SRC_THINNING_THINNER_H_

#include <opencv2/opencv.hpp>

class Thinner {
public:
	Thinner();
	virtual ~Thinner();

	//these two where obtained from http://opencv-code.com/quick-tips/implementation-of-guo-hall-thinning-algorithm/
	void thinningGuoHallIteration(cv::Mat& im, int iter);
	void thinningGuoHall(cv::Mat& im);

	//these two from http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/
	void thinning(const cv::Mat& src, cv::Mat& dst);
	void thinningIteration(cv::Mat& img, int iter);
};

#endif /* SRC_THINNING_THINNER_H_ */
