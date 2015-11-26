/*
 * ObjectDetector.cpp
 *
 *  Created on: 16 Nov 2015
 *      Author: martin
 */

#include <pcl/registration/icp.h>
#include "objects/ObjectDetector.h"
#include <pcl/visualization/pcl_visualizer.h>

/*
 * Kinfu
 */

#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/common/angles.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>

using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;

ObjectDetector::ObjectDetector() {
	// TODO Auto-generated constructor stub

}

ObjectDetector::ObjectDetector(int mode, std::string model_path) :
		svm(model_path), model_path_(model_path), train_(mode == TRAIN_MODE), test_(
				mode == TEST_MODE) {
	if (test_) {
		svm.load_model();
	}

}

void ObjectDetector::train() {
	svm.trainSVM();
}

void ObjectDetector::test_data(std::vector<Segment*>& test_segments) {

	svm.testSVM(test_segments);

}

void ObjectDetector::add_training_data(
		std::vector<Segment*>& foreground_segments,
		std::vector<Segment*>& background_segments) {

	cout << "ObjectDetector::foreground_segments.size()="
			<< foreground_segments.size() << " background_segments="
			<< background_segments.size() << endl;
	svm.add_training_data(foreground_segments, background_segments);
	segments.push_back(foreground_segments);

}

void ObjectDetector::add_training_data(
		std::vector<Segment*>& foreground_segments,
		std::vector<Segment*>& background_segments, Cloudptr point_cloud,
		Mat& cropped_img, Mat& cropped_depth) {
	svm.add_training_data(foreground_segments, background_segments);
	point_clouds.push_back(point_cloud);
	Mat img_to_push = Mat::zeros(dimensions_img, dimensions_img, CV_8UC3);
	Mat depth_to_push = Mat::zeros(dimensions_img, dimensions_img, CV_32FC1);
	int x = dimensions_img / 2 - cropped_img.cols / 2;
	int y = dimensions_img / 2 - cropped_img.rows / 2;
	Rect rect(x, y, cropped_img.cols, cropped_img.rows);
	cout << " rect=" << rect << " cropped_img.size()=" << cropped_img.size()
			<< endl;
	cout << "img_to_push.size()=" << img_to_push.size()
			<< "img_to_push(rect).size()=" << img_to_push(rect).size() << endl;
	cropped_img.copyTo(img_to_push(rect));
	cropped_depth.copyTo(depth_to_push(rect));
//	imshow("img_to_push",img_to_push);
//	waitKey(0);
	frames_.push_back(cropped_img);
	double min,max;

	minMaxLoc(depth_to_push,&min,&max);
//	for(int i=0;i<depth_to_push.rows;i++){
//		for(int j=0;j<depth_to_push.cols;j++){
//			if(depth_to_push.at<float>(i,j)>0.)
//				cout <<depth_to_push.at<float>(i,j)<<" ";
//		}
//		//cout << endl;
//	}
	cout <<" in depth found min,max="<<min<<" "<<max<<endl;
	//depth_to_push = depth_to_push*1000.;
	depth_to_push.convertTo(depth_to_push,CV_16UC1);

	depths_.push_back(depth_to_push);

}

void ObjectDetector::view_examples() {

	for (Cloudptr& cloud : point_clouds) {
		display_cloud(cloud);
	}

}

void ObjectDetector::display_cloud(PlainCloudptr& cloud) {
	pcl::visualization::PCLVisualizer viewer("3d Viewer");
	viewer.setBackgroundColor(0, 0, 0);

	viewer.addPointCloud < pcl::PointXYZ > (cloud, "Kinfu Cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.spin();
}

void ObjectDetector::display_cloud(Cloudptr& cloud) {
	pcl::visualization::PCLVisualizer viewer("3d Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud < pcl::PointXYZRGB > (cloud, "sample cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.spin();
}

void ObjectDetector::align_point_clouds() {
	if (point_clouds.size() < 2)
		return;
	Cloudptr& cloud_1 = point_clouds[0];
	Cloudptr& cloud_2 = point_clouds[1];

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_1);
	icp.setInputTarget(cloud_2);
	Cloudptr final(new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*final);
	std::cout << "has converged:" << icp.hasConverged() << " score: "
			<< icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	//display_cloud(final);
}

void ObjectDetector::run_kinfu(float vsz) {

	pcl::gpu::KinfuTracker kinfu_(dimensions_img, dimensions_img);
	//Init Kinfu Tracker

	Eigen::Vector3f volume_size = Vector3f::Constant(vsz/*meters*/);
	kinfu_.volume().setSize(volume_size);

	Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
	Eigen::Vector3f t = volume_size * 0.5f
			- Vector3f(0, 0, volume_size(2) / 2 * 1.2f);

	Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

	kinfu_.setInitalCameraPose(pose);
	kinfu_.volume().setTsdfTruncDist(0.030f/*meters*/);
	kinfu_.setIcpCorespFilteringParams(0.1f/*meters*/, sin(pcl::deg2rad(20.f)));
	//kinfu_.setDepthTruncationForICP(5.f/*meters*/);
	kinfu_.setCameraMovementThreshold(0.001f);

	for (unsigned int i = 0; i < frames_.size(); i++) {
		Mat frame = frames_[i];
		Mat depth = depths_[i];



		//upload depth data to GPU
		PtrStepSz<const unsigned short> depth_;
		std::vector<unsigned short> source_depth_data_;
		source_depth_data_.resize(depth.cols * depth.rows);
		memcpy(&source_depth_data_[0], depth.data,
				sizeof(unsigned short int) * depth.cols * depth.rows);
		depth_.data = &source_depth_data_[0];
		KinfuTracker::DepthMap depth_device_;
		depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);

		//upload colour data to GPU
		PtrStepSz<const KinfuTracker::PixelRGB> rgb24;
		std::vector<KinfuTracker::PixelRGB> source_image_data_;
		source_image_data_.resize(frame.cols * frame.rows);
		//WTF:memcpy asi, a pincho y sin contemplaciones, la receta del essssito :P
		memcpy(&source_image_data_[0], frame.data,
				sizeof(Vec3b) * frame.cols * frame.rows);
		rgb24.data = &source_image_data_[0];
		KinfuTracker::View colors_device_;
		colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);

		//run kinfu
		kinfu_ (depth_device_, colors_device_);
		cout << "KinfuTracker started without complaining" << endl;


	}
	//download the point cloud
	PointCloud<PointXYZ>::Ptr cloud_ptr_ (new PointCloud<PointXYZ>);
	DeviceArray<PointXYZ> cloud_buffer_device_;
	DeviceArray<RGB> point_colors_device_;

	DeviceArray<PointXYZ> extracted = kinfu_.volume().fetchCloud (cloud_buffer_device_);

	cloud_buffer_device_.download(cloud_ptr_->points);

	//kinfu_.colorVolume().fetchColors(extracted, point_colors_device_);
	//point_colors_device_.download(cloud_ptr_->points);
	cloud_ptr_->width = (int)cloud_ptr_->points.size ();
	cloud_ptr_->height = 1;


	cout <<"> kinfu fetched cloud has this many points: "<<(int)cloud_ptr_->points.size ()<<endl;

	display_cloud(cloud_ptr_);



}

ObjectDetector::~ObjectDetector() {
	// TODO Auto-generated destructor stub
}

