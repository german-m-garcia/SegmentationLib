/*
 * Segmentation.h
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#ifndef SRC_SEGMENTATION_H_
#define SRC_SEGMENTATION_H_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/types_c.h"


#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudaimgproc.hpp"


#define DEBUG false



class Segmentation {
public:
	Segmentation();
	virtual ~Segmentation();

	void preprocess(bool gpu,cv::Mat& src,int scales);

	void mean_shift(const cv::Mat& src, double sp, double sr, double min_size, cv::Mat& dst);

	void canny_segment(cv::Mat& src,
				cv::Mat& contours_mat,cv::Mat& gradient,cv::Mat& grayGradient, double gradient_threshold);


	void scharr_segment(cv::Mat& src,
			cv::Mat& contours_mat,cv::Mat& gradient,cv::Mat& grayGradient, double gradient_threshold, bool rnd_colours);

	void intensity_histogram( cv::Mat& src,cv::Mat& dst);

	void thinning(const cv::Mat& src, cv::Mat& dst);

private:
	void segment_contours(const cv::Mat& src, cv::Mat& draw,cv::Mat& paint, bool rnd_colours);

	void thinningIteration(cv::Mat& img, int iter);

	void nms(cv::Mat& gradx, cv::Mat& grady, cv::Mat& gradient);

	void thin_contours(cv::Mat& grayGradient);

};

#endif /* SRC_SEGMENTATION_H_ */
