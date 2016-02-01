/*
 * ObjectDetector.h
 *
 *  Created on: 16 Nov 2015
 *      Author: martin
 */

#ifndef INCLUDE_OBJECTS_OBJECTDETECTOR_H_
#define INCLUDE_OBJECTS_OBJECTDETECTOR_H_

#include "MRSMapWrap.h"
#include "segment.h"
#include "svm_wrapper.h"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/features/from_meshes.h>
#include <pcl/surface/organized_fast_mesh.h>
#include "utils.h"


//using namespace mrsmap;


typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PlainCloudptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloudptr;


using Mat = cv::Mat;
using Point2i = cv::Point2i;
using Point = cv::Point;
using Rect = cv::Rect;
using RNG = cv::RNG;
using Size = cv::Size;
using Vec4i = cv::Vec4i;




class ObjectDetector {
public:
	ObjectDetector();
	ObjectDetector(int mode,string svm_path,string model_path);
	ObjectDetector(int mode,string svm_path,string model_path, string object_name);
	virtual ~ObjectDetector();

	void create_dirs();

	void load_current_data();

	void save_current_data();

	void test_pcl_segments(cv::Mat&img, cv::Mat& depth_float,vector<Segment*>& fg_segments);

	void add_selected_segments(Mat&img, Mat& depth_float,vector<Segment*>& fg_segments,vector<Segment*>& bg_segments);

	void add_training_data(std::vector<Segment*>& foreground_segments,
			 std::vector<Segment*>& background_segments);

	 void add_training_data(std::vector<Segment*>& foreground_segments,
			 std::vector<Segment*>& background_segments, Cloudptr point_cloud, Mat& cropped_img, Mat& cropped_depth);

	 void test_data(std::vector<Segment*>& test_segments);

	 void unify_detections(Mat& mask);


	 bool test_data(std::vector<Segment*>& test_segments,Mat& original_img, Mat& original_depth,vector<Mat>& masks, Mat& debug);

	 bool test_data(std::vector<Segment*>& test_segments,Mat& original_img, Mat& original_depth,vector<Mat>& masks, Mat& debug, vector<Point3d>& slc_position, vector<Point3d>& slc_orientation);


	 void normalize_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud,Point3d& gravity_center);

	 void draw_contours_detections(Mat& src,Mat& mask, Mat& debug);

	 void find_slc_bounding_box(Mat& detections, vector<Rect>& rect, vector<Mat>& masks);

	void train();

	void view_examples();

	void align_point_clouds();

	void run_kinfu(float vsz);

	const static int TRAIN_MODE = 0;
	const static int TEST_MODE = 1;
private:

	//need a bunch of segments for each frame
	std::vector < std::vector <Segment*> > segments;
	std::vector <Segment*> full_view_segments;
	//the real apperances for each frame
	std::vector <cv::Mat> frames_;
	std::vector< cv::Mat> depths_;

	std::vector<Cloudptr> point_clouds;
	//svm classifier for classifying segments
	SVMWrapper svm;
	//a path to where the data can be stored and debugged
	std::string model_path_;
	std::string svm_path_;
	bool train_, test_;

	const static int dimensions_img = 300;
	//const static int rows = 480, cols = 640;
	const static int rows = 1234, cols = 1624;

	float fx = 1368.30, fy = 1368.3, cx = cols/2., cy = rows/2.;

	double threshold_positive_class;

	std::string object_name_;

	MRSMapWrap mrsmap;

	Utils utils_;

	//path to the features data of the SVM that we might want to temporally store
	std::string svm_tmp_data;


	void display_cloud(PlainCloudptr& cloud);

	void display_cloud(Cloudptr& cloud);



};



#endif /* INCLUDE_OBJECTS_OBJECTDETECTOR_H_ */
