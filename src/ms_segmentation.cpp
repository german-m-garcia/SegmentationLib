/*
 * Segmentation.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "ms_segmentation.h"
#include "base_segmentation.h"
#include "segment.h"
#include <string>

using namespace std;
using namespace cv;

MSSegmentation::MSSegmentation() {
	// TODO Auto-generated constructor stub

}

MSSegmentation::MSSegmentation(cv::Mat& src, bool gpu, int scales,
		int starting_scale) :
		original_img_(src), absolute_scales_(scales), actual_scales_(
				scales - starting_scale), starting_scale_(starting_scale),original_size_(src.size()) {

	assert(scales > 0 && starting_scale >= 0 && starting_scale <= scales);

	image_pyramid_.reserve(absolute_scales_);
	bilateral_filtered_pyramid_.reserve(absolute_scales_);
	pyramid(gpu, src, absolute_scales_, starting_scale_);
	segments_pyramid_.resize(actual_scales_);
	output_segments_pyramid_.resize(actual_scales_);

	//show_pyramids();

}

void MSSegmentation::init(cv::Mat& src, bool gpu, int scales,
		int starting_scale)
		 {

	assert(scales > 0 && starting_scale >= 0 && starting_scale <= scales);
	original_size_ = src.size();
	original_img_ =src;
	absolute_scales_=scales;
	actual_scales_=				scales - starting_scale;
	starting_scale_=starting_scale;

	image_pyramid_.reserve(absolute_scales_);
	bilateral_filtered_pyramid_.reserve(absolute_scales_);
	pyramid(gpu, src, absolute_scales_, starting_scale_);
	segments_pyramid_.resize(actual_scales_);
	output_segments_pyramid_.resize(actual_scales_);

	//show_pyramids();

}

MSSegmentation::~MSSegmentation() {

	// TODO Auto-generated destructor stub
	for (int i = 0; i < actual_scales_; i++) {
		cout <<"> ~MSSegmentation() i="<<i<<" actual_scales_="<<actual_scales_<<endl;
		cout <<"> ~MSSegmentation() image_pyramid_.size()="<<image_pyramid_.size()<<endl;
		image_pyramid_[i].release();
		bilateral_filtered_pyramid_[i].release();
		//output_segments_pyramid_[i].release();
		for (unsigned int j = 0; j < segments_pyramid_[i].size(); j++) {
			delete segments_pyramid_[i][j];
		}
	}

}

Size MSSegmentation::getSize(int scale){
	return image_pyramid_[scale].size();
}

void MSSegmentation::show_pyramids() {
	for (unsigned int i = 0; i < bilateral_filtered_pyramid_.size(); i++) {
		string name("bilateral pyramid - scale ");
		string level = to_string(i);
		imshow(name + level, bilateral_filtered_pyramid_[i]);
		imwrite("/home/martin/Research/bil_" + name + level + ".png",
				bilateral_filtered_pyramid_[i]);
		imwrite("/home/martin/Research/original_" + name + level + ".png",
				image_pyramid_[i]);
	}
	waitKey(0);
}


void MSSegmentation::bilateral_filter(bool gpu, cv::Mat& src_dst) {
	cv::Mat tmp_f;
#ifdef CUDA_ENABLED
	if (gpu) {
		//cv::bilateralFilter(src, tmp_f, 0, 25, 50);
		gpu::GpuMat d_src(src_dst), d_src_a, d_tmp_f;
		//gpu::cvtColor(d_src, d_src_a, CV_BGR2BGRA);

		gpu::bilateralFilter(d_src, d_tmp_f, 0, 25, 50);
		d_tmp_f.download(src_dst);

	} else {
		//cv::bilateralFilter(src_dst, tmp_f, 0, 25, 50);
		cv::bilateralFilter(src_dst, tmp_f, 0, 15, 7);

		src_dst = tmp_f;
	}
#else

	//cv::medianBlur(src_dst, tmp_f, 3);
	//cv::medianBlur(src_dst, tmp_f, 5);
	cv::bilateralFilter(src_dst, tmp_f, -1, 15, 7);
	//cv::bilateralFilter(src_dst, tmp_f, 0, 15, 7);

	src_dst = tmp_f;
#endif
}

/*
 * computes a Gaussian pyramid representation for the input image
 */
void MSSegmentation::pyramid(bool gpu, cv::Mat& src, int scales,
		int starting_scale) {



	// we  skip as many scales as necessary
	for (int i = 0; i < starting_scale; i++) {
		cv::pyrDown(src, src, cv::Size(src.cols / 2, src.rows / 2));
	}



	// iterate starting at the starting_scale until the number of scales is reached
	for (int i = starting_scale; i < scales; i++) {
		Mat img_scale_i = src;
		image_pyramid_.push_back(img_scale_i);
		bilateral_filter(gpu, img_scale_i);
		bilateral_filtered_pyramid_.push_back(img_scale_i);

		cv::pyrDown(src, src, cv::Size(src.cols / 2, src.rows / 2));
		cout <<"> MSSegmentation::pyramid image_pyramid_.size()="<<image_pyramid_.size()<<endl;

	}
}

void MSSegmentation::segment_pyramid(double sp, double sr,
		double min_size) {


	if (do_bilateral)
		for (int i = 0; i < actual_scales_; i++) {
			Mat contours_mat, gradient, gray_gradient;


			//scharr_segment(bilateral_filtered_pyramid_[i], contours_mat,
			//		gradient, gray_gradient, thres, i, true);
			mean_shift(bilateral_filtered_pyramid_[i],contours_mat,sp,sr,min_size);
			output_segments_pyramid_[i] = contours_mat;


		}
	else
		for (int i = 0; i < actual_scales_; i++) {
			Mat contours_mat, gradient, gray_gradient;
//			scharr_segment(image_pyramid_[i], contours_mat, gradient,
//					gray_gradient, thres, i, true);
			mean_shift(image_pyramid_[i],contours_mat,sp,sr,min_size);
			output_segments_pyramid_[i] = contours_mat;



		}
}

/*
 * it reduces the size of src by a number of scales
 * given by scale
 */
void MSSegmentation::preprocess(bool gpu, cv::Mat& src, int scale) {
	cv::Mat tmp_f;
	for (int i = 0; i < scale; i++)
		cv::pyrDown(src, src, cv::Size(src.cols / 2, src.rows / 2));
#ifdef CUDA_ENABLED
	if (gpu) {
		//cv::bilateralFilter(src, tmp_f, 0, 25, 50);
		gpu::GpuMat d_src(src), d_src_a, d_tmp_f;
		//gpu::cvtColor(d_src, d_src_a, CV_BGR2BGRA);

		gpu::bilateralFilter(d_src, d_tmp_f, 0, 25, 50);
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

void MSSegmentation::mean_shift(cv::Mat& src, cv::Mat& dst, double sp, double sr,
		double min_size) {


	Mat img2;

	//cout <<">bilateralFilter..."<<endl;

	cv::imwrite("/home/martin/workspace/EGBISegmentation/build/input.png",src);
	cout <<" saved input image for segmentation"<<endl;
	std::string call("~/workspace/EGBISegmentation/build/ms_segment ~/workspace/EGBISegmentation/build/input.png ");
	call = call + utils_.stringify(sp)+" "+ utils_.stringify(sr)+" "+ utils_.stringify(min_size)+"  ~/workspace/EGBISegmentation/build/out.png";
	int rc =std::system (call.c_str());
	cout <<" system call returned "<<rc<<endl;
	//open the segmentation result
	dst = imread("/home/martin/workspace/EGBISegmentation/build/out.png",CV_LOAD_IMAGE_COLOR);
	//read_segments(src,dst);

}

void MSSegmentation::save_colours(int scale, vector<Vec3b>& colours) {

	colours.resize(0);
	for (Segment* seg : segments_pyramid_[scale]) {
		colours.push_back(seg->getRandomColour());
	}
}

void MSSegmentation::reset_colours(int scale, vector<Vec3b>& colours) {

	int i = 0;
	for (Segment* seg : segments_pyramid_[scale]) {
		seg->re_colour(colours[i]);
		i++;
	}
}




/*
 * maps the segments of a given scale so that they can be retrieved
 * by get_segment_at_scale
 */
void MSSegmentation::map_segments(int scale) {


	if (do_bilateral){

		read_segments(bilateral_filtered_pyramid_[scale],output_segments_pyramid_[scale]);
	}

	else{
		read_segments(image_pyramid_[scale],output_segments_pyramid_[scale]);
	}

	cout <<" at scale "<<scale << " the size of component_id is  "<<component_id.size()<<endl;
	cv::resize(component_id,component_id,original_size_);
	cout <<" size of component_id is now "<<component_id.size()<<endl;




	//	assert(scale >= 0 && scale <= actual_scales_);
//	component_id = cv::Mat::zeros(output_segments_pyramid_[scale].size(),
//	CV_16UC1);
//	vector<Segment*>& segments = segments_pyramid_[scale];
//	//iterate over the segments
//	unsigned int id = 2;
//	for (Segment* seg : segments) {
//		//iterate over their points
//		seg->computeFeatures();
//		Mat idMat = (seg->getBinaryMat() / 255) * id;
//		idMat.convertTo(idMat, CV_16UC1);
////		imshow("seg->getBinaryMat()",seg->getBinaryMat());
////		waitKey(0);
//		idMat.copyTo(component_id, seg->getBinaryMat());
//		//cout <<component_id(seg->getBoundRect())<<endl;
//		//cout <<" added segment #"<<id<<endl;
//		mapSegments[id] = seg;
//		id++;
//	}
	//Mat tmp;
	//component_id.convertTo(tmp,CV_8UC1);
	//imshow("component_id",tmp);
	//waitKey(0);
}






void MSSegmentation::print_results(Mat& dst, int last_n_scales) {

	if (last_n_scales == -1)
		last_n_scales = actual_scales_;
	//first row for the original image and the segmentations
	//second row for the thresholded gradients
	dst = Mat::zeros(original_img_.rows * 3,
			original_img_.cols * (last_n_scales + 1), CV_8UC3);
	cout << "> showing # of scales =" << last_n_scales << endl;
	vector<Rect> rects, rects_gradients, rects_th_gradients;
	Rect rect_orig(0, 0, original_img_.cols, original_img_.rows);
	for (int i = 0; i < last_n_scales; i++) {
		Rect rect_level_i(original_img_.cols * (i + 1), 0, original_img_.cols,
				original_img_.rows);
		Rect rect_grad_i(original_img_.cols * (i + 1), original_img_.rows,
				original_img_.cols, original_img_.rows);
		Rect rect_th_grad_i(original_img_.cols * (i + 1),
				original_img_.rows * 2, original_img_.cols, original_img_.rows);
		rects.push_back(rect_level_i);
		rects_gradients.push_back(rect_grad_i);
		rects_th_gradients.push_back(rect_th_grad_i);
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

	for (int i = 0; i < last_n_scales; i++) {
		Mat tmp_out;
		resize(
				output_segments_pyramid_[output_segments_pyramid_.size() - 1 - i],
				tmp_out, original_img_.size());
		tmp_out.copyTo(dst(rects[last_n_scales - 1 - i]));



	}
}
