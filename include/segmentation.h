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

#ifdef CUDA_ENABLED
	#include "opencv2/cudaarithm.hpp"
	#include "opencv2/cudaimgproc.hpp"
#endif

#include "base_segmentation.h"


#define DEBUG false
#define MIN_SIZE_PER_LEVEL 29



class Segmentation : public BaseSegmentation {
public:
	Segmentation();
	Segmentation(cv::Mat& src, bool gpu, int scales);
	virtual ~Segmentation();

	void pyramid(bool gpu,cv::Mat& src,int scales);

	void preprocess(bool gpu,cv::Mat& src,int scale);

	void mean_shift(const cv::Mat& src, double sp, double sr, double min_size, cv::Mat& dst);

	void canny_segment(cv::Mat& src,
				cv::Mat& contours_mat,cv::Mat& gradient,cv::Mat& grayGradient, double gradient_threshold, int scale);


	void scharr_segment(cv::Mat& src,
			cv::Mat& contours_mat,cv::Mat& gradient,cv::Mat& grayGradient, double gradient_threshold,int scale, bool rnd_colours);

	void intensity_histogram( cv::Mat& src,cv::Mat& dst);

	void thinning(const cv::Mat& src, cv::Mat& dst);

	void show_pyramids();

	void segment_pyramid(double thres);

	void edge_tests(cv::Mat& src,double gradient_threshold);

	void save_colours(int scale,vector<Vec3b>& colours);

	void reset_colours(int scale, vector<Vec3b>& colours);

	const vector<cv::Mat>& getOutputSegmentsPyramid() const {
		return output_segments_pyramid_;
	}

	const vector<cv::Mat>& getBilateralFilteredPyramid() const {
		return bilateral_filtered_pyramid_;
	}

	const vector<vector<Segment*> >& getSegmentsPyramid() const {
		return segments_pyramid_;
	}

private:
	void segment_contours(const cv::Mat& src, cv::Mat& draw,cv::Mat& paint,int scale, bool rnd_colours);

	void thinningIteration(cv::Mat& img, int iter);

	void nms(cv::Mat& gradx, cv::Mat& grady, cv::Mat& gradient);

	void thin_contours(cv::Mat& grayGradient);

	void bilateral_filter(bool gpu, cv::Mat& src_dst);

	void expand_mat(cv::Mat& src, cv::Mat& dst);

	void join_edges(Mat& expanded, int dx, int dy);

	void expanded_scharr(Mat& src,Mat& expanded,Mat &dst,int dx,int dy);

	void expanded_scharr_positive(Mat& src,Mat& expanded,Mat &dst,int dx,int dy);

	cv::Mat original_img_;
	vector<cv::Mat> image_pyramid_, bilateral_filtered_pyramid_, output_segments_pyramid_;
	vector<cv::Mat> scharr_pyramid_, bilateral_scharr_pyramid_;

	vector< vector <Segment*> > segments_pyramid_;


};

#endif /* SRC_SEGMENTATION_H_ */
