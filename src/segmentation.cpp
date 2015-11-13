/*
 * Segmentation.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "segmentation.h"
#include "base_segmentation.h"
#include "segment.h"
#include <string>

using namespace std;
using namespace cv;

Segmentation::Segmentation() {
	// TODO Auto-generated constructor stub

}

Segmentation::Segmentation(cv::Mat& src, bool gpu, int scales, int starting_scale) :
		original_img_(src),absolute_scales_(scales+1),actual_scales_(scales+1-starting_scale),starting_scale_(starting_scale) {
	assert(scales>0&&starting_scale>=0 && starting_scale<scales);

	image_pyramid_.reserve(absolute_scales_);
	bilateral_filtered_pyramid_.reserve(absolute_scales_);
	pyramid(gpu, src, absolute_scales_,starting_scale_);
	segments_pyramid_.resize(actual_scales_);
	output_segments_pyramid_.resize(actual_scales_);
}

Segmentation::~Segmentation() {

	// TODO Auto-generated destructor stub
	for (unsigned int i = 0; i < actual_scales_; i++) {
		image_pyramid_[i].release();
		bilateral_filtered_pyramid_[i].release();
		//output_segments_pyramid_[i].release();
		for (unsigned int j = 0; j < segments_pyramid_[i].size(); j++) {
			delete segments_pyramid_[i][j];
		}
	}

}

void Segmentation::show_pyramids() {
	for (unsigned int i = 0; i < bilateral_filtered_pyramid_.size(); i++) {
		string name("bilateral pyramid - scale ");
		string level = to_string(i);
		imshow(name + level, bilateral_filtered_pyramid_[i]);
	}
	waitKey(0);
}

void Segmentation::bilateral_filter(bool gpu, cv::Mat& src_dst) {
	cv::Mat tmp_f;
#ifdef CUDA_ENABLED
	if (gpu) {
		//cv::bilateralFilter(src, tmp_f, 0, 25, 50);
		cuda::GpuMat d_src(src_dst), d_src_a, d_tmp_f;
		//cuda::cvtColor(d_src, d_src_a, CV_BGR2BGRA);

		cuda::bilateralFilter(d_src, d_tmp_f, 0, 25, 50);
		d_tmp_f.download(src_dst);

	} else {
		//cv::bilateralFilter(src_dst, tmp_f, 0, 25, 50);
		cv::bilateralFilter(src_dst, tmp_f, 0, 15, 7);

		src_dst = tmp_f;
	}
#else
	cv::bilateralFilter(src_dst, tmp_f, 0, 15, 7);
	src_dst = tmp_f;
#endif
}

/*
 * computes a Gaussian pyramid representation for the input image
 */
void Segmentation::pyramid(bool gpu, cv::Mat& src, int scales, int starting_scale) {

	// if starting_scale = 0 we start the pyramid by upsampling the original image
	if(starting_scale==0){
		cv::pyrUp(src, src, cv::Size(src.cols * 2, src.rows * 2));
		//add scale 0
		//bilateral filtering
		Mat img_scale_0 = src.clone();
		image_pyramid_.push_back(src);
		bilateral_filter(gpu, img_scale_0);
		bilateral_filtered_pyramid_.push_back(img_scale_0);
	}
	Mat src_copy = src;

	// if starting_scale = 1 then we start the pyramid with the original image
	// otherwise, we skip scales until we reach the starting_scale
	for(int i=1;i<starting_scale;i++){
		cv::pyrDown(src_copy, src_copy,
						cv::Size(src_copy.cols / 2, src_copy.rows / 2));
	}

	for (int i = starting_scale; i < scales; i++) {
		Mat img_scale_i;

		cv::pyrDown(src_copy, img_scale_i,
				cv::Size(src_copy.cols / 2, src_copy.rows / 2));

		src_copy = img_scale_i.clone();
		image_pyramid_.push_back(img_scale_i);
		bilateral_filter(gpu, img_scale_i);
		bilateral_filtered_pyramid_.push_back(img_scale_i);
	}
}

void Segmentation::segment_pyramid(double thres) {
	bool do_bilateral = true;

	if (do_bilateral)
		for (int i = 0; i < actual_scales_; i++) {
			Mat contours_mat, gradient, gray_gradient;

			//edge_tests(bilateral_filtered_pyramid_[i], thres);

			//cout <<"processing scale "<<i<<endl;
			scharr_segment(bilateral_filtered_pyramid_[i], contours_mat,
					gradient, gray_gradient, thres, i, true);
			output_segments_pyramid_[i]=contours_mat;
			//thres -= 0.01;

			if (DEBUG) {
				string name("image pyramid - scale ");
				string gradient_name("gradient pyramid - scale ");
				string gray_gradient_name("gray gradient pyramid - scale ");
				string contours("contours pyramid - scale ");
				string level = to_string(i);
				imshow(name + level, image_pyramid_[i]);
				imshow(gradient_name + level, gradient);
				imshow(gray_gradient_name + level, gray_gradient);
				imshow(contours + level, contours_mat);
				waitKey(0);
			}

		}
	else
		for (int i = 0; i < actual_scales_; i++) {
			Mat contours_mat, gradient, gray_gradient;
			scharr_segment(image_pyramid_[i], contours_mat, gradient,
					gray_gradient, thres, i, true);
			output_segments_pyramid_[i]=contours_mat;

			if (DEBUG) {
				string name("image pyramid - scale ");
				string gradient_name("gradient pyramid - scale ");
				string gray_gradient_name("gray gradient pyramid - scale ");
				string contours("contours pyramid - scale ");
				string level = to_string(i);
				imshow(name + level, image_pyramid_[i]);
				imshow(gradient_name + level, gradient);
				imshow(gray_gradient_name + level, gray_gradient);
				imshow(contours + level, contours_mat);
				waitKey(0);

			}

		}
}

/*
 * it reduces the size of src by a number of scales
 * given by scale
 */
void Segmentation::preprocess(bool gpu, cv::Mat& src, int scale) {
	cv::Mat tmp_f;
	for (int i = 0; i < scale; i++)
		cv::pyrDown(src, src, cv::Size(src.cols / 2, src.rows / 2));
#ifdef CUDA_ENABLED
	if (gpu) {
		//cv::bilateralFilter(src, tmp_f, 0, 25, 50);
		cuda::GpuMat d_src(src), d_src_a, d_tmp_f;
		//cuda::cvtColor(d_src, d_src_a, CV_BGR2BGRA);

		cuda::bilateralFilter(d_src, d_tmp_f, 0, 25, 50);
		d_tmp_f.download(tmp_f);
		src = tmp_f;
	} else {
		cv::bilateralFilter(src, tmp_f, 0, 25, 50);
		src = tmp_f;
	}
#else
	cv::bilateralFilter(src, tmp_f, 0, 25, 50);
	src = tmp_f;
#endif
}

void Segmentation::mean_shift(const cv::Mat& src, double sp, double sr,
		double min_size, cv::Mat& dst) {

	Mat segmentation_result;
#ifdef CUDA_ENABLED
	cuda::GpuMat k_img(src), k_imga, k_dst, k_dsta;

	cuda::cvtColor(k_img, k_imga, CV_BGR2BGRA);

	cuda::bilateralFilter(k_imga, k_dsta, -1, 5, BORDER_CONSTANT);
	//Mat filtered;
	//k_dst.download(filtered);
	//cout << "mean shift..." << endl;
	meanShiftSegmentation(k_dsta, segmentation_result, sp, sr, min_size,
			TermCriteria(2, -1, 0.001));
	cv::cvtColor(segmentation_result, segmentation_result, CV_BGRA2BGR);
	//cout << "downloading from GPU..." << endl;

#endif
	read_segments(src, segmentation_result, dst);

}

void Segmentation::save_colours(int scale, vector<Vec3b>& colours) {

	colours.resize(0);
	for (Segment* seg : segments_pyramid_[scale]) {
		colours.push_back(seg->getRandomColour());
	}
}

void Segmentation::reset_colours(int scale, vector<Vec3b>& colours) {

	int i = 0;
	for (Segment* seg : segments_pyramid_[scale]) {
		seg->re_colour(colours[i]);
		i++;
	}
}

/*
 * Finds the segments delimited by the gradient:
 *
 * grayGradient CV_64F [0..1]: the Mat containing the edges
 * original: the Mat containing the original image
 * paint: the Mat with the segments
 */
void Segmentation::segment_contours(const cv::Mat& grayGradient,
		cv::Mat& original, cv::Mat& paint, int scale, bool rnd_colours) {
	cv::Mat binary_copy;

	//cout << "Segmentation::segment_contours scale="<<scale<<endl;
	if (rnd_colours)
		paint = Mat::zeros(grayGradient.rows, grayGradient.cols, CV_8UC3);
	else
		paint = original.clone();

	grayGradient.copyTo(binary_copy);
	binary_copy *= 255.;
	binary_copy.convertTo(binary_copy, CV_8UC1);
	//cv::imshow("finding contours here", binary_copy);
	vector<Vec4i> hierarchy;
	vector<Vec4i> hierarchy_best;
	vector<vector<Point> > contours;

	cv::findContours(binary_copy, contours, hierarchy, RETR_CCOMP,
			CHAIN_APPROX_NONE);
	//cout <<"found the contours"<<endl;
	//cv::findContours(binary_copy, contours, RETR_CCOMP, CHAIN_APPROX_NONE);

	//cout << "contours.size()=" << contours.size() << endl;

	for (int idx = 0; idx < contours.size(); idx++) {

		Mat mask = Mat::zeros(grayGradient.rows, grayGradient.cols, CV_8UC1);
		Mat segment = Mat::zeros(grayGradient.rows, grayGradient.cols, CV_8UC3);

		//if it has parents we skip it at first
		if (hierarchy[idx][3] != -1) {
			continue;
		}

		int segment_size = contourArea(contours[idx]);
		if (segment_size < MIN_SIZE_PER_LEVEL)
			continue;

		//figure out the avg colour of this segment

		cv::Scalar colour, avg_colour;
		if (rnd_colours)
			colour = Scalar(rand() & 255, rand() & 255, rand() & 255);
		else
			colour = mean(original, mask);

		Mat segment_mat = Mat::zeros(grayGradient.rows, grayGradient.cols,
		CV_8UC3);

#if CV_MAJOR_VERSION == 2
// do opencv 2 code
		//cout <<"drawing contours "<< idx  <<" out of contours.size()="<<contours.size()<<endl;
		cv::drawContours(segment_mat, contours, idx, colour, CV_FILLED, 8,
				hierarchy);
		cv::drawContours(paint, contours, idx, colour, CV_FILLED, 8, hierarchy);
#elif CV_MAJOR_VERSION == 3
// do opencv 3 code
		cv::drawContours(segment_mat, contours, idx, colour, FILLED, 8, hierarchy);
		cv::drawContours(paint, contours, idx, colour, FILLED, 8, hierarchy);
#endif

		//cv::drawContours(paint, contours, idx, colour, 6, 8, hierarchy);
		for (Point p : contours[idx]) {
			paint.at<Vec3b>(p.y, p.x) = Vec3b(colour[0], colour[1], colour[2]);
			segment_mat.at<Vec3b>(p.y, p.x) = Vec3b(colour[0], colour[1],
					colour[2]);
		}
		Rect bounding_rect = boundingRect(contours[idx]);
		Mat sub_mat_original = original(bounding_rect);
		Mat sub_mat_segment = segment_mat(bounding_rect);
		Mat binary_original_;// == Mat::zeros(original.rows,original.cols,CV_8UC1)
		cvtColor(segment_mat, binary_original_, CV_BGR2GRAY);

		Segment *seg = new Segment(sub_mat_original, sub_mat_segment,
				binary_original_, contours[idx], bounding_rect, segment_size,
				Vec3b(colour[0], colour[1], colour[2]));
		//cout <<"segments_pyramid_.size()="<<segments_pyramid_.size()<<endl;
		segments_pyramid_[scale].push_back(seg);

	}
//	imshow("without parents",paint);
//	for (int idx = 0; idx < contours.size(); idx++) {
//
//		Mat mask = Mat::zeros(grayGradient.rows, grayGradient.cols, CV_8UC1);
//		Mat segment = Mat::zeros(grayGradient.rows, grayGradient.cols, CV_8UC3);
//
//		//if it has parents we skip it at first
//		if ( hierarchy[idx][3] != -1 ) {
//			continue;
//		}
//
//		if (contours[idx].size() < MIN_SIZE_PER_LEVEL)
//			continue;
//
//		//figure out the avg colour of this segment
//
//		cv::Scalar colour, avg_colour;
//		if (rnd_colours)
//			colour = Scalar(rand() & 255, rand() & 255, rand() & 255);
//		else
//			colour = mean(original, mask);
//
//		cv::drawContours(paint, contours, idx, colour, FILLED, 8, hierarchy);
//		//cv::drawContours(paint, contours, idx, colour, 6, 8, hierarchy);
//		for (Point p : contours[idx]) {
//			paint.at<Vec3b>(p.y, p.x) = Vec3b(colour[0], colour[1], colour[2]);
//		}
//
//	}
//	imshow("with parents",paint);


}

void Segmentation::nms(Mat& gradx, Mat& grady, Mat& gradient) {

	//compute the orientation of the gradient in degrees
	Mat orientation;
	phase(gradx, grady, orientation, true);
	//quantize the orientations

}

/*
 * src: input image  CV_8UC3 [0..255]
 * contours_mat: image with the segments CV_8UC3 [0..255]
 * gradient: Mat with the edges CV_64FC3 [0 ..1]
 * grayGradient: Mat with the edges CV_64F [0..1]
 * gradient_threshold: the threshold applied to the edge mat
 */
void Segmentation::canny_segment(cv::Mat& src, cv::Mat& contours_mat,
		cv::Mat& gradient, cv::Mat& grayGradient, double gradient_threshold,
		int scale) {

	//cv::imshow("Original Img", src);

	int edgeThresh = 1;
	int lowThreshold = 25;
	int const max_lowThreshold = 100;
	int ratio = 2;
	int kernel_size = 7;

	/**
	 * @function CannyThreshold
	 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
	 */

	/// Canny detector
	cv::Canny(src, gradient, lowThreshold, lowThreshold * ratio, kernel_size);
	imshow("canny", gradient);
	waitKey(0);

//	Mat scharr_gradient, diff;
	//cv::nms_Scharr(src, scharr_gradient, lowThreshold, lowThreshold * ratio,
	//		kernel_size);

	//cv::dilate(scharr_gradient,scharr_gradient,Mat());
	//cv::erode(scharr_gradient,scharr_gradient,Mat());

//	scharr_gradient.convertTo(scharr_gradient, CV_64FC1);
	double min, max;
	cv::minMaxLoc(gradient, &min, &max);
	gradient -= min;
	gradient *= 1.0 / (max - min);
	gradient = gradient < gradient_threshold;

	segment_contours(gradient, src, contours_mat, scale, true);

}

void Segmentation::thin_contours(cv::Mat& grayGradient) {

	Mat copy = grayGradient < 1.;

	copy.convertTo(copy, CV_8UC1);
	//imshow("gradient before thining", copy);
	thinning(copy, copy);
	//imshow("gradient after thining", copy);
	//waitKey(0);
	copy.convertTo(grayGradient, CV_32FC1);
	grayGradient = grayGradient < 1.;

}

void Segmentation::expand_mat(Mat& src, Mat& dst) {

	//the expanded mat is twice as big
	dst = Mat::zeros(src.rows * 2 + 1, src.cols * 2 + 1, src.type());
	//cout << "dst.type()=" << dst.type() << " dst.size()" << dst.size() << endl;
	for (int i = 0; i < src.rows; i++)
		for (int j = 0; j < src.cols; j++) {
			int map_row = (i + 1) * 2 - 1;
			int map_col = (j + 1) * 2 - 1;
			dst.at<Vec3d>(map_row, map_col) = src.at<Vec3d>(i, j);
		}
	double min, max;
	cv::minMaxLoc(dst, &min, &max);
	dst -= min;
	dst *= 1.0 / (max - min);
}

void Segmentation::join_edges(Mat& expanded, int dx, int dy) {
	if (dy == 1) {
		for (int i = 2; i < expanded.rows - 2; i += 2)
			for (int j = 2; j < expanded.cols - 2; j += 2) {

				expanded.at<double>(i, j) = (expanded.at<double>(i, j - 1)
						+ expanded.at<double>(i, j + 1)) / 2.;
			}
	} else if (dx == 1) {
		for (int i = 2; i < expanded.rows - 2; i += 2)
			for (int j = 2; j < expanded.cols - 2; j += 2) {

				expanded.at<double>(i, j) = (expanded.at<double>(i - 1, j)
						+ expanded.at<double>(i + 1, j)) / 2.;
			}
	}

}

void Segmentation::expanded_scharr(Mat& float_src, Mat& expanded,
		Mat &expanded_gradient, int dx, int dy) {

	Mat tmp_dst;
	Mat gradx, grady;

	// finds horizontal edges ->cols axis
	if (dx > 0) {

		double kernel_x_array[15] = { -0.09375, 0., 0.09375, 0., 0., 0.,
				-0.3125, 0., 0.3125, 0., 0., 0., -0.09375, 0., 0.09375 };
		Mat kernel_x_mat = cv::Mat(5, 3, CV_64F, kernel_x_array);
		Mat kernel_x_mat_3_channels;
		vector<Mat> kernel_x_vector =
				{ kernel_x_mat, kernel_x_mat, kernel_x_mat };
		merge(kernel_x_vector, kernel_x_mat_3_channels);

		cout << kernel_x_mat_3_channels << endl;
		expand_mat(float_src, expanded);
		cout << "expanded.type()=" << expanded.type() << " expanded.size()"
				<< expanded.size() << endl;
		imshow("expanded", expanded);

		expanded_gradient = Mat::zeros(expanded.rows, expanded.cols,
		CV_64F);
		for (int i = 3; i < expanded.rows - 5; i += 2)
			for (int j = 2; j < expanded.cols - 3; j += 2) {

				// the rectangle is vertical: width = 3, height = 5
				Rect sub_rect(j - 1, i - 2, 3, 5);

				double product = expanded(sub_rect).dot(
						kernel_x_mat_3_channels);
				expanded_gradient.at<double>(i, j) = product;
			}
		//join_edges(expanded_gradient,1,0);

		// finds vertical edges ->rows axis
	} else if (dy > 0) {

		double kernel_y_array[15] = { -0.09375, 0., -0.3125, 0., -0.09375, 0.,
				0., 0., 0., 0., 0.09375, 0., 0.3125, 0., 0.09375, };
		Mat kernel_y_mat = cv::Mat(3, 5, CV_64F, kernel_y_array);
		Mat kernel_y_mat_3_channels;
		vector<Mat> kernel_x_vector =
				{ kernel_y_mat, kernel_y_mat, kernel_y_mat };
		merge(kernel_x_vector, kernel_y_mat_3_channels);

		cout << kernel_y_mat_3_channels << endl;
		expand_mat(float_src, expanded);
		cout << "expanded.type()=" << expanded.type() << " expanded.size()"
				<< expanded.size() << endl;
		imshow("expanded", expanded);

		expanded_gradient = Mat::zeros(expanded.rows, expanded.cols,
		CV_64F);
		for (int i = 2; i < expanded.rows - 5; i += 2)
			for (int j = 3; j < expanded.cols - 3; j += 2) {

				// the rectangle is horizontal: width = 5, height = 3
				Rect sub_rect(j - 2, i - 1, 5, 3);

				double product = expanded(sub_rect).dot(
						kernel_y_mat_3_channels);
				expanded_gradient.at<double>(i, j) = product;
			}
		//join_edges(expanded_gradient,0,1);

	}
}

void Segmentation::edge_tests(Mat& src, double gradient_threshold) {

	Mat expanded, tmp_dst;
	Mat gradx, grady;
	Mat gradient;
	Mat float_src;
	src.convertTo(float_src, CV_64FC3);

	expanded_scharr(float_src, expanded, gradx, 1, 0);
	expanded_scharr(float_src, expanded, grady, 0, 1);

	cv::magnitude(gradx, grady, gradient);
	double min, max;
	cv::minMaxLoc(gradient, &min, &max);
	gradient -= min;
	gradient *= 1.0 / (max - min);
	cv::minMaxLoc(gradient, &min, &max);
	cout << " min and max in gradient = " << min << " " << max << endl;
	for (int i = 0; i < gradient.rows; i++)
		for (int j = 0; j < gradient.cols; j++) {

			if (gradient.at<double>(i, j) > gradient_threshold)
				expanded.at<Vec3d>(i, j) = Vec3d(0., 0., 255.);
		}

	Mat grayGradient = gradient < gradient_threshold;

	Mat contours_mat;
	//segment_contours(grayGradient, src, contours_mat, true);

	imshow("expanded_gradient", gradient);
	imshow("expanded", expanded);
	imshow("grayGradient", grayGradient);

	waitKey(0);

}


/*
 * maps the segments of a given scale so that they can be retrieved
 * by get_segment_at_scale
 */
void Segmentation::map_segments(int scale){
	assert(scale >= 0 && scale <actual_scales_);
	component_id = cv::Mat::zeros(output_segments_pyramid_[scale].size(), CV_16UC1);
	vector<Segment*>& segments = segments_pyramid_[scale];
	//iterate over the segments
	unsigned int id = 2;
	for(Segment* seg: segments){
		//iterate over their points
		seg->computeFeatures();
		Mat idMat = (seg->getBinaryMat()/255)*id;
		idMat.convertTo(idMat,CV_16UC1);
//		imshow("seg->getBinaryMat()",seg->getBinaryMat());
//		waitKey(0);
		idMat.copyTo(component_id,seg->getBinaryMat());
		//cout <<component_id(seg->getBoundRect())<<endl;
		//cout <<" added segment #"<<id<<endl;
		mapSegments[id] = seg;
		id++;
	}
	//Mat tmp;
	//component_id.convertTo(tmp,CV_8UC1);
	//imshow("component_id",tmp);
	//waitKey(0);
}

/*
 * src: input image  CV_8UC3 [0..255]
 * contours_mat: image with the segments CV_8UC3 [0..255]
 * gradient: Mat with the edges CV_64FC3 [0 ..1]
 * grayGradient: Mat with the edges CV_64F [0..1]
 * gradient_threshold: the threshold applied to the edge mat
 */
void Segmentation::scharr_segment(cv::Mat& src, cv::Mat& contours_mat,
		cv::Mat& gradient, cv::Mat& grayGradient, double gradient_threshold,
		int scale, bool rnd_colours) {

	//cv::imshow("Original Img", src);

	cv::Rect rect_orig(0, 0, src.cols, src.rows);
	cv::Rect rect_edges(src.cols, 0, src.cols, src.rows);

	//cv::globalPb(img0, gPb, gPb_thin, gPb_ori);
	cv::Mat gradx, grady, abs_grad_x, abs_grad_y, abs_grad;

	int ksize = 5;

	cv::Scharr(src, gradx, CV_64F, 1, 0, 1);
	cv::Scharr(src, grady, CV_64F, 0, 1, 1);

	//edge_tests(src, gradient_threshold);

	cv::magnitude(gradx, grady, gradient);
	double min, max;
	cv::minMaxLoc(gradient, &min, &max);

	gradient -= min;
	gradient *= 1.0 / (max - min);
	cv::minMaxLoc(gradient, &min, &max);
	//cout << "min max " << min << " " << max << endl;

	//segmentation
	std::vector<cv::Mat> gradients;
	cv::split(gradient, gradients);

	grayGradient = (gradients[0] + gradients[1] + gradients[2]) / 3.;
	//imshow("gradient before thining", grayGradient);

	grayGradient = grayGradient < gradient_threshold;

//	cv::erode(grayGradient,grayGradient,Mat());
//	cv::dilate(grayGradient,grayGradient,Mat());
//	cv::dilate(grayGradient,grayGradient,Mat());

//	imshow("grayGradient before",grayGradient);
//	thin_contours(grayGradient);
//	imshow("grayGradient after",grayGradient);
//	waitKey(0);
	// grayGradient CV_64F [0..1]

	segment_contours(grayGradient, src, contours_mat, scale, rnd_colours);

}

/**

 */

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * Parameters:
 * 		im    Binary image with range = [0,1]
 * 		iter  0=even, 1=odd
 *
 *
 * Code for thinning a binary image using Zhang-Suen algorithm.
 *
 * Author:  Nash (nash [at] opencv-code [dot] com)
 * Website: http://opencv-code.com
 *
 */
void Segmentation::thinningIteration(cv::Mat& img, int iter) {
	CV_Assert(img.channels() == 1);
	CV_Assert(img.depth() != sizeof(uchar));
	CV_Assert(img.rows > 3 && img.cols > 3);

	cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

	int nRows = img.rows;
	int nCols = img.cols;

	if (img.isContinuous()) {
		nCols *= nRows;
		nRows = 1;
	}

	int x, y;
	uchar *pAbove;
	uchar *pCurr;
	uchar *pBelow;
	uchar *nw, *no, *ne;    // north (pAbove)
	uchar *we, *me, *ea;
	uchar *sw, *so, *se;    // south (pBelow)

	uchar *pDst;

	// initialize row pointers
	pAbove = NULL;
	pCurr = img.ptr<uchar>(0);
	pBelow = img.ptr<uchar>(1);

	for (y = 1; y < img.rows - 1; ++y) {
		// shift the rows up by one
		pAbove = pCurr;
		pCurr = pBelow;
		pBelow = img.ptr<uchar>(y + 1);

		pDst = marker.ptr<uchar>(y);

		// initialize col pointers
		no = &(pAbove[0]);
		ne = &(pAbove[1]);
		me = &(pCurr[0]);
		ea = &(pCurr[1]);
		so = &(pBelow[0]);
		se = &(pBelow[1]);

		for (x = 1; x < img.cols - 1; ++x) {
			// shift col pointers left by one (scan left to right)
			nw = no;
			no = ne;
			ne = &(pAbove[x + 1]);
			we = me;
			me = ea;
			ea = &(pCurr[x + 1]);
			sw = so;
			so = se;
			se = &(pBelow[x + 1]);

			int A = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1)
					+ (*ea == 0 && *se == 1) + (*se == 0 && *so == 1)
					+ (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1)
					+ (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
			int B = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
			int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
			int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				pDst[x] = 1;
		}
	}

	img &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * Parameters:
 * 		src  The source image, binary with range = [0,255]
 * 		dst  The destination image
 */
void Segmentation::thinning(const cv::Mat& src, cv::Mat& dst) {
	dst = src.clone();
	dst /= 255;         // convert to binary image

	cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
	cv::Mat diff;

//	do {
//		thinningIteration(dst, 0);
//		thinningIteration(dst, 1);
//		cv::absdiff(dst, prev, diff);
//		dst.copyTo(prev);
//	} while (cv::countNonZero(diff) > 0);
//
	for (int iter = 0; iter < 1; iter++) {
		thinningIteration(dst, 0);
		thinningIteration(dst, 1);

	}

	dst *= 255;
}

void Segmentation::intensity_histogram(Mat& src, Mat& dst) {

	Mat src_gray(src.rows, src.cols, CV_64FC1);
	if (!src.data)
		return;
	src /= 255.;
	/// Establish the number of bins
	int histSize = 512;

	std::vector<cv::Mat> gradients;
	cv::split(src, gradients);

	src_gray = (gradients[0] + gradients[1] + gradients[2]) / 3.;
	src_gray.convertTo(src_gray, CV_32FC1);

	/// Set the ranges ( for B,G,R) )

	float range[] = { 0.002, 1. };
	const float* histRange = { range };

	bool uniform = true;
	bool accumulate = false;

	Mat i_hist;
	vector<Mat> src_array = { src };

	cout << "src_gray.type()=" << src_gray.type() << "src_gray.channels()="
			<< src_gray.channels() << endl;
	/// Compute the intensity histograms:
	calcHist(&src_gray, 1, 0, Mat(), i_hist, 1, &histSize, &histRange, uniform,
			accumulate);

	// Draw the histograms
	int hist_w = 512;
	int hist_h = 400;
	int bin_w = cvRound((double) hist_w / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

	/// Normalize the result to [ 0, histImage.rows ]
	normalize(i_hist, i_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

	/// Draw for each channel
	for (int i = 1; i < histSize; i++) {
		line(histImage,
				Point(bin_w * (i - 1),
						hist_h - cvRound(i_hist.at<float>(i - 1))),
				Point(bin_w * (i), hist_h - cvRound(i_hist.at<float>(i))),
				Scalar(255, 255, 255), 2, 8, 0);

	}

	/// Display
	namedWindow("calcHist Demo", cv::WINDOW_AUTOSIZE);

	imshow("src", src);
	imshow("calcHist Demo", histImage);

	waitKey(0);

}

void Segmentation::print_results(Mat& dst, int last_n_scales){

	dst  = Mat::zeros(original_img_.rows, original_img_.cols * (last_n_scales+1), CV_8UC3);
	cout << "> showing # of scales =" <<last_n_scales << endl;
	vector<Rect> rects;
	Rect rect_orig(0, 0, original_img_.cols, original_img_.rows);
	for(int i=0;i<last_n_scales;i++){
		Rect rect_level_i(original_img_.cols*(i+1), 0, original_img_.cols, original_img_.rows);
		rects.push_back(rect_level_i);
	}


//	Rect rect_level_0(original_img.cols, 0, original_img.cols, original_img.rows);
//	Rect rect_level_1(original_img.cols*2, 0, original_img.cols, original_img.rows);
//	Rect rect_level_2(original_img.cols*3, 0, original_img.cols, original_img.rows);


	//img0.convertTo(img0, CV_32FC3);
	original_img_.copyTo(dst(rect_orig));

//	grayGradient *= 255.;
//	//grayGradient.convertTo(img0, CV_8UC3);
//    cvtColor(grayGradient,grayGradient,COLOR_GRAY2BGR);
//	cout << "> grayGradient.type()=" <<grayGradient.type()<<" grayGradient.size()="<<grayGradient.size() << endl;
//
//	gradient*= 255.;

	for(int i=0;i<last_n_scales;i++){
		Mat tmp_out;
		resize(output_segments_pyramid_[output_segments_pyramid_.size()-1-i],tmp_out,original_img_.size());
		tmp_out.copyTo(dst(rects[last_n_scales-1-i]));
	}
}
