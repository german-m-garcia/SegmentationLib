/*
 * utils.h
 *
 *  Created on: 5 Nov 2015
 *      Author: martin
 */

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include <chrono>
#include <string>
#include <ctime>
#include <ratio>
#include <iostream>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/imgproc/imgproc.hpp>



using namespace std;
using namespace cv;
using namespace pcl;
using namespace std::chrono;
using std::ostream;

const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;


class Utils {
public:
	Utils() {
	}
	virtual ~Utils() {
	}

	int parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale,  int& gpu,string& img_path, string& output_path);

	int parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale, int& gpu,string& img_path, string& output_path, string& svm_path);

	int parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale, int& gpu,string& img_path, string& output_path, string& svm_path, string& pcls_path);

	std::string remove_extension(const std::string& filename);

	string get_file_name(const string& s);

	void merge_two_bounding_rects(Rect& rec1,Rect& rec2, Rect& res);


	void image_to_pcl(Mat& img, Mat& depth,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, int cx, int cy, Rect& rect);

	void image_to_pcl(Mat& img, Mat& depth,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud);


void tick() {
	t0 = high_resolution_clock::now();
}

void tock(string& text) {
	tf = std::chrono::high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(tf - t0);

	cout << text << " took: " << time_span.count() * 1000 << " ms" << endl;
}
private:
high_resolution_clock::time_point t0, tf;
};

#endif /* INCLUDE_UTILS_H_ */
