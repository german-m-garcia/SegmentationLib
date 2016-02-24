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
#include <pcl/features/fpfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/vfh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include "segment.h"




using namespace std;
using namespace std::chrono;
using std::ostream;

const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;


using PointXYZ = pcl::PointXYZ;
using PointXYZRGB = pcl::PointXYZRGB;

class Utils {
public:
	Utils() {
	}
	virtual ~Utils() {
	}


	std::string stringify(double x);

	std::string stringify(int x);

	int parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale,  int& gpu,string& img_path, string& output_path);

	int parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale, int& gpu,string& img_path, string& output_path, string& svm_path);

	int parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale, int& gpu,string& img_path, string& output_path, string& svm_path, string& pcls_path);

	std::string remove_extension(const std::string& filename);

	string get_file_name(const string& s);

	void merge_two_bounding_rects(Rect& rec1,Rect& rec2, Rect& res);

	void cropped_pcl_from_mask(Mat& img, Mat& depth,
			Mat& mask,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Mat& tmp_img,
			Mat& tmp_depth);

	void cropped_pcl_from_segments(cv::Mat& img, cv::Mat& depth,vector<Segment*>&segments,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,cv::Mat& tmp_img,cv::Mat& tmp_depth);

	void compute_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			pcl::PointCloud<pcl::Normal>::Ptr& normals);

	void compute_integral_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
			pcl::PointCloud<pcl::Normal>::Ptr& normals);

	void compute_vfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, cv::Mat& vfh_features);

	void compute_fpfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs);

	void compute_cvfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointCloud<pcl::VFHSignature308>::Ptr& vfhFeatures);


	void compute_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pca_cloud,  Eigen::Matrix3f& eigenVectors);

	double icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,Eigen::Matrix4f& transform);

	double validate_transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,Eigen::Matrix4f& transform);

	void compute_pca_alt(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pca_cloud, Eigen::Matrix3f& eigenVectors);

	void euler_from_R(Eigen::Matrix3f R,Point3d& angles);


	void remove_HOLDER_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) ;

	void remove_SCREW_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud);

	void remove_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud);

	void remove_zeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,std::vector<int>& indices);

	void subtract_gravity_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src, Point3d& gravity_center);

	void subtract_gravity_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,std::vector<int>& indices, Point3d& gravity_center);


	void build_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, pcl::PolygonMesh::Ptr& mesh1);

	void display_mesh(pcl::PolygonMesh::Ptr& mesh);

	void display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,string& text);


	void display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, string& text);

	void display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,std::vector<int>& indices,string& text);

	void display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Matrix3f& eigenVectors, string& text);

	void compute_inertia(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Point3d& dimensions_3d);

	void find_pcl_yaw(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, Point3d& orientation);


	void compute_contours(cv::Mat& mask,vector<vector<Point> >& contours);

	void addPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud);

	void fill_mask(cv::Mat& mask);

	void sub_sample_screw(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud);

	void sub_sample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud);

	void find_detection_yaw(cv::Mat& mask, cv::Mat& img,cv::Mat& depth, Point3d& position,Point3d& orientation);

	void shift_to_min(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud);

	void shift_to_min(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, std::vector<int>& indices);

	void rotate_eigen_axes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud);

	void obtain_eigen_axes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud, Eigen::Matrix4f& projectionTransform);

	void compute_bounding_box(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,Point3d& dimensions_3d);

	void cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);


	void xyz_gravity_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,  Point3d& gravity_center, double &max_z/*,Eigen::Matrix3f& eigenVectors*/);

	void mask_to_pcl_indices(cv::Mat& mask,vector<int>& indices );


	void scale_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, float factor);

	void image_to_pcl(cv::Mat& img, cv::Mat& depth,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, int cx, int cy, Rect& rect);

	void copy_clouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud_1,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_cloud_2);

	void image_to_pcl(cv::Mat& img, cv::Mat& depth,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud);

	void prune_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_dst);

	void nanify_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_dst);


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
