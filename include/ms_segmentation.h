/*
 * Segmentation.h
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#ifndef SRC_MSSEGMENTATION_H_
#define SRC_MSSEGMENTATION_H_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/types_c.h"



#ifdef CUDA_ENABLED
	//#include "opencv2/cudaarithm.hpp"
	//#include "opencv2/cudaimgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#endif

#include "base_segmentation.h"
#include "utils.h"


#define DEBUG false
#define MIN_SIZE_PER_LEVEL 1 //29



class MSSegmentation : public BaseSegmentation {
public:
	MSSegmentation();
	MSSegmentation(cv::Mat& src, bool gpu, int scales, int starting_scale);
	virtual ~MSSegmentation();

	void init(cv::Mat& src, bool gpu, int scales,
			int starting_scale);

	void pyramid(bool gpu,cv::Mat& src,int scales, int starting_scale);

	void preprocess(bool gpu,cv::Mat& src,int scale);

	void share_mat(cv::Mat& src,key_t key);


	void mean_shift(cv::Mat& src, cv::Mat& dst, double sp = 15., double sr = 15., double min_size = 30.);




	Size getSize(int scale);


	void show_pyramids();

	void segment_pyramid(double sp, double sr,
			double min_size);


	void save_colours(int scale,vector<Vec3b>& colours);

	void reset_colours(int scale, vector<Vec3b>& colours);

	vector<cv::Mat>& getOutputSegmentsPyramid() {
		return output_segments_pyramid_;
	}

	const vector<cv::Mat>& getBilateralFilteredPyramid() const {
		return bilateral_filtered_pyramid_;
	}

	const vector<Segment*>& getPropagatedSegments() const {
		return segments;
	}

	void map_segments(int scale);

	void print_results(Mat& dst, int last_n_scales);

private:


	void bilateral_filter(bool gpu, cv::Mat& src_dst);


	cv::Mat original_img_;
	vector<cv::Mat> image_pyramid_, bilateral_filtered_pyramid_, output_segments_pyramid_;
	vector<cv::Mat> scharr_pyramid_, bilateral_scharr_pyramid_;

	vector< vector <Segment*> > segments_pyramid_;

	int absolute_scales_;
	int actual_scales_;
	int starting_scale_;

	const bool do_bilateral = true;

	Size original_size_;
	Utils utils_;


};

#endif /* SRC_SEGMENTATION_H_ */
