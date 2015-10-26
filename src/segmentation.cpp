/*
 * Segmentation.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "segmentation.h"

using namespace std;
using namespace cv;

Segmentation::Segmentation() {
	// TODO Auto-generated constructor stub

}

Segmentation::~Segmentation() {
	// TODO Auto-generated destructor stub
}

void Segmentation::preprocess(bool gpu, cv::Mat& src, int scales) {
	cv::Mat tmp_f;
	for (int i = 0; i < scales; i++)
		cv::pyrDown(src, src, cv::Size(src.cols / 2, src.rows / 2));

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

}

void Segmentation::mean_shift(const cv::Mat& src, double sp, double sr,
		double min_size, cv::Mat& dst) {

	cuda::GpuMat k_img(src), k_imga, k_dst, k_dsta;

	cuda::cvtColor(k_img, k_imga, CV_BGR2BGRA);

	cuda::bilateralFilter(k_imga, k_dsta, -1, 5, BORDER_CONSTANT);
	//Mat filtered;
	//k_dst.download(filtered);
	//cout << "mean shift..." << endl;
	meanShiftSegmentation(k_dsta, dst, sp, sr, min_size,
			TermCriteria(2, -1, 0.001));
	cv::cvtColor(dst, dst, CV_BGRA2BGR);
	//cout << "downloading from GPU..." << endl;

}

/*
 * Finds the segments delimited by the gradient:
 *
 * grayGradient CV_64F [0..1]: the Mat containing the edges
 * original: the Mat containing the original image
 * paint: the Mat with the segments
 */
void Segmentation::segment_contours(const cv::Mat& grayGradient,
		cv::Mat& original, cv::Mat& paint, bool rnd_colours) {
	cv::Mat binary_copy;

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

	cv::findContours(binary_copy, contours, RETR_LIST, CHAIN_APPROX_NONE);

	cout << "contours.size()=" << contours.size() << endl;

	for (int idx = 0; idx < contours.size(); idx++) {

		Mat mask = Mat::zeros(grayGradient.rows, grayGradient.cols, CV_8UC1);
		Mat segment = Mat::zeros(grayGradient.rows, grayGradient.cols, CV_8UC3);
		cv::drawContours(mask, contours, idx, Scalar(255), FILLED, 8,
				hierarchy);

		//figure out the avg colour of this segment

		cv::Scalar colour, avg_colour;
		if (rnd_colours)
			colour = Scalar(rand() & 255, rand() & 255, rand() & 255);
		else
			colour = mean(original, mask);

		cv::drawContours(paint, contours, idx, colour, FILLED, 8, hierarchy);
		cv::drawContours(paint, contours, idx, colour, 6, 8, hierarchy);
		for (Point p : contours[idx]) {

			paint.at<Vec3b>(p.y, p.x) = Vec3b(colour[0], colour[1], colour[2]);

		}
		//cv::drawContours(segment, contours, idx, colour, CV_FILLED, 8, hierarchy);
		//cv::dilate(segment,segment,Mat());
		//cv::dilate(mask,mask,Mat());
		//segment.copyTo(paint,mask);

		//cv::drawContours(mask, contours, idx, colour, CV_FILLED, 8, hierarchy);
		//cout <<"contour:"<<idx<<endl;
		//imshow("mask", mask);
		//waitKey(0);

	}
	//cv::imshow("contours", paint);
	//cv::waitKey(0);
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
		cv::Mat& gradient, cv::Mat& grayGradient, double gradient_threshold) {

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

	Mat scharr_gradient, diff;
	//cv::nms_Scharr(src, scharr_gradient, lowThreshold, lowThreshold * ratio,
	//		kernel_size);

	//cv::dilate(scharr_gradient,scharr_gradient,Mat());
	//cv::erode(scharr_gradient,scharr_gradient,Mat());

	scharr_gradient.convertTo(scharr_gradient, CV_64FC1);
	double min, max;
	cv::minMaxLoc(scharr_gradient, &min, &max);

	gradient -= min;
	gradient *= 1.0 / (max - min);
	imshow("scharr_gradient", scharr_gradient);
	scharr_gradient = scharr_gradient < 1;
	imshow("inverted scharr_gradient", scharr_gradient);
	segment_contours(scharr_gradient, src, contours_mat, true);

	imshow("contours_mat", contours_mat);
	waitKey(0);

}

void Segmentation::thin_contours(cv::Mat& grayGradient){

	Mat copy = grayGradient < 1.;

	copy.convertTo(copy, CV_8UC1);
	//imshow("gradient before thining", copy);
	thinning(copy, copy);
	//imshow("gradient after thining", copy);
	//waitKey(0);
	copy.convertTo(grayGradient,CV_32FC1);
	grayGradient = grayGradient < 1.;

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
		bool rnd_colours) {

	//cv::imshow("Original Img", src);

	cv::Rect rect_orig(0, 0, src.cols, src.rows);
	cv::Rect rect_edges(src.cols, 0, src.cols, src.rows);

	//cv::globalPb(img0, gPb, gPb_thin, gPb_ori);
	cv::Mat gradx, grady;
	int scale = 1;
	cv::Scharr(src, gradx, CV_64F, 1, 0, scale);
	cv::Scharr(src, grady, CV_64F, 0, 1, scale);

	cv::magnitude(gradx, grady, gradient);
	double min, max;
	cv::minMaxLoc(gradient, &min, &max);

	gradient -= min;
	gradient *= 1.0 / (max - min);
	cv::minMaxLoc(gradient, &min, &max);
	cout << "min max " << min << " " << max << endl;

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
	segment_contours(grayGradient, src, contours_mat, rnd_colours);
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

	do {
		thinningIteration(dst, 0);
		thinningIteration(dst, 1);
		cv::absdiff(dst, prev, diff);
		dst.copyTo(prev);
	} //while (cv::countNonZero(diff) > 0);
	while(false);

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

