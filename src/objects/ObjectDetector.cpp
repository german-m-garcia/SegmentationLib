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

//#include "cuda_runtime.h"
#include "utils.h"

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

//finds the bounding rectangle of the slc
void ObjectDetector::find_slc_bounding_box(Mat& src, vector<Rect>& rects, vector<Mat>& masks) {

	Mat mask = src.clone();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Find contours
	findContours(mask, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_NONE,
			Point(0, 0));
	/// Draw contours
	RNG rng(12345);
	Mat drawing = Mat::zeros(mask.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++) {
		//if it has parents we skip it at first
		if (hierarchy[i][3] != -1) {
			continue;
		}
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
				rng.uniform(0, 255));
		//drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		Mat object = Mat::zeros(mask.size(), CV_8UC1);
		drawContours(object, contours, i, Scalar(255), -1);
		Rect rect = boundingRect(contours[i]);
		masks.push_back(object);
		rects.push_back(rect);
	}

	/// Show in a window
	//namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	//imshow("Contours", drawing);
	//waitKey(0);

}

void ObjectDetector::unify_detections(Mat& mask) {

	for(int i=0;i<5;i++)
		dilate(mask, mask, Mat());

	for(int i=0;i<2;i++)
		erode(mask, mask, Mat());
//	imshow("unified mask",mask);
//	waitKey(0);
}

//Mat& ref = segmentation_1.getOutputSegmentsPyramid()[scale_for_propagation];
bool ObjectDetector::test_data(std::vector<Segment*>& test_segments,
		Mat& original_img, Mat& original_depth, vector<Mat>& masks, Mat& debug,
		vector<Point3d>& slc_positions, vector<Point3d>& slc_orientations) {

	/*
	 * compute the point cloud
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals;

	Utils utils;
	utils.image_to_pcl(original_img, original_depth, pcl_cloud);
	utils.compute_integral_normals(pcl_cloud, normals);

	for (Segment *seg : test_segments) {
		seg->add_precomputed_pcl(pcl_cloud, normals);
		//seg->addPcl(img_1,depth_float);
		seg->computeFeatures();
	}

	debug = original_img.clone();

	svm.testSVM(test_segments);
	if (test_segments.size() == 0)
		return false;
	Mat ref = test_segments[0]->getBinaryMat();
	Mat detections = Mat::zeros(ref.rows, ref.cols, CV_8UC3);
	//cout <<" iterating segments"<<endl;
	int ndetections = 0;
	for (Segment *seg : test_segments) {

		if (seg->getClassLabel() > 0.3) {
			//cout <<" bounding rect="<<seg->getBoundRect()<<endl;
			//cout <<"detections.size()="<<detections.size()<<endl;
			//cout <<"detections(seg->getBoundRect()).size()="<<detections(seg->getBoundRect()).size()<< " seg->getRandomColourMat().size()= "<<seg->getRandomColourMat().size()<<endl;

			//imshow("seg->getRandomColourMat()", seg->getRandomColourMat());
			//waitKey(0);
			detections(seg->getBoundRect()) += seg->getRandomColourMat();
			ndetections++;
		}
	}
	if (ndetections == 0)
		return false;

	//get the mask and minimum bounding rectangle
	Rect rect;
	Mat pointsMat;
	resize(detections, detections, original_img.size());
	Mat mask;
	cvtColor(detections, mask, CV_RGB2GRAY);
	mask = mask > 0;
	unify_detections(mask);

	vector<Rect> rects;

	find_slc_bounding_box(mask, rects,masks);

	for(Mat mask: masks){
		Point3d slc_position;
		Point3d slc_orientation;
		//iterate for each blob in the mask
		cv::findNonZero(mask, pointsMat);
		rect = boundingRect(pointsMat);
		rectangle(debug, rect, Scalar(0, 0, 255), 3);
		double yaw = 0.;
		utils.find_detection_yaw(mask,original_img,original_depth, slc_position,slc_orientation);
		//store as roll,pitch,yaw
		//slc_orientation.x = 0.;
		//slc_orientation.y = 0.;
		//slc_orientation.z = yaw;

//		Eigen::Matrix3f eigenVectors;
//		double max_z = 0.;
//		utils.xyz_gravity_center(cloud, slc_position, max_z);
//		utils.compute_pca(cloud, eigenVectors);
//		slc_orientation.x = eigenVectors.col(0)(0);
//		slc_orientation.y = eigenVectors.col(0)(1);
//		slc_orientation.z = eigenVectors.col(0)(2);
		slc_positions.push_back(slc_position);
		slc_orientations.push_back(slc_orientation);

	}



	return true;

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

//void ObjectDetector::add_training_data(
//		std::vector<Segment*>& foreground_segments,
//		std::vector<Segment*>& background_segments, Cloudptr point_cloud,
//		Mat& cropped_img, Mat& cropped_depth) {
//	svm.add_training_data(foreground_segments, background_segments);
//	point_clouds.push_back(point_cloud);
//	Mat img_to_push = Mat::zeros(dimensions_img, dimensions_img, CV_8UC3);
//	Mat depth_to_push = Mat::zeros(dimensions_img, dimensions_img, CV_32FC1);
//	int x = dimensions_img / 2 - cropped_img.cols / 2;
//	int y = dimensions_img / 2 - cropped_img.rows / 2;
//	Rect rect(x, y, cropped_img.cols, cropped_img.rows);
//	cout << " rect=" << rect << " cropped_img.size()=" << cropped_img.size()
//			<< endl;
//	cout << "img_to_push.size()=" << img_to_push.size()
//			<< "img_to_push(rect).size()=" << img_to_push(rect).size() << endl;
//	cropped_img.copyTo(img_to_push(rect));
//	cropped_depth.copyTo(depth_to_push(rect));
////	imshow("img_to_push",img_to_push);
////	waitKey(0);
//	frames_.push_back(cropped_img);
//	double min,max;
//
//	minMaxLoc(depth_to_push,&min,&max);
////	for(int i=0;i<depth_to_push.rows;i++){
////		for(int j=0;j<depth_to_push.cols;j++){
////			if(depth_to_push.at<float>(i,j)>0.)
////				cout <<depth_to_push.at<float>(i,j)<<" ";
////		}
////		//cout << endl;
////	}
//	cout <<" in depth found min,max="<<min<<" "<<max<<endl;
//	//depth_to_push = depth_to_push*1000.;
//	depth_to_push.convertTo(depth_to_push,CV_16UC1);
//
//	depths_.push_back(depth_to_push);
//
//}

void ObjectDetector::add_training_data(
		std::vector<Segment*>& foreground_segments,
		std::vector<Segment*>& background_segments, Cloudptr point_cloud,
		Mat& img, Mat& depth) {
	svm.add_training_data(foreground_segments, background_segments);
	point_clouds.push_back(point_cloud);
	resize(img, img, Size(cols, rows));
	frames_.push_back(img);
	double min, max;
	minMaxLoc(depth, &min, &max);
	cout << " in depth found min,max=" << min << " " << max << endl;
	//depth_to_push = depth_to_push*1000.;

	//depth *= 1000.;
//	cv::Mat_<unsigned short int> depth_int(depth.rows,depth.cols);
//	for(int i=0;i<depth.rows;i++){
//		for(int j=0;j<depth.cols;j++){
//			depth_int(i,j) = (unsigned short int)(depth.at<float>(i,j)*1000.f);
//		}
//	}
	minMaxLoc(depth, &min, &max);
	cout << " in depth found min,max=" << min << " " << max << endl;

	cout << "original resolution=" << depth.size() << endl;
	resize(depth, depth, Size(cols, rows));
	depths_.push_back(depth);

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
	pcl::visualization::PCLVisualizer viewer("Kinfu");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud < pcl::PointXYZRGB > (cloud, "sample cloud");
	//viewer.addCoordinateSystem(1.0);
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

	pcl::gpu::KinfuTracker kinfu_(rows, cols);
	//Init Kinfu Tracker

	kinfu_.setDepthIntrinsics(fx, fy, cx, cy);

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
	kinfu_.setCameraMovementThreshold(0.0001f);

//	const int max_color_integration_weight = 2;
//	kinfu_.initColorIntegration(max_color_integration_weight);

	KinfuTracker::DepthMap depth_device_;
	KinfuTracker::View colors_device_;
	for (unsigned int i = 0; i < frames_.size(); i++) {
		Mat frame = frames_[i];
		Mat depth = depths_[i];

		//imshow("kinfuing with this frame",frame);
		//waitKey(0);

		//upload depth data to GPU
		PtrStepSz<const unsigned short int> depth_;
		depth_.cols = depth.cols;
		depth_.rows = depth.rows;
		depth_.step = depth_.cols * sizeof(const unsigned short int);//depth_.elemSize();

		//cout <<" params: depth_.step="<<depth_.step<<" depth_.cols="<<depth_.cols<<" depth_.rows="<<depth_.rows<<endl;
		std::vector<unsigned short int> source_depth_data_;
		source_depth_data_.resize(depth_.cols * depth_.rows);

		memcpy(&source_depth_data_[0], depth.data,
				sizeof(unsigned short int) * depth.cols * depth.rows);
		depth_.data = &source_depth_data_[0];
		depth_device_.upload(depth_.data, depth_.step, depth_.rows,
				depth_.cols);

		kinfu_(depth_device_);

	}

	PointCloud<PointXYZ>::Ptr cloud_ptr_(new PointCloud<PointXYZ>);
	DeviceArray<PointXYZ> cloud_buffer_device_;

	DeviceArray<PointXYZ> extracted = kinfu_.volume().fetchCloud(
			cloud_buffer_device_);
	extracted.download(cloud_ptr_->points);
	cloud_ptr_->width = (int) cloud_ptr_->points.size();
	cloud_ptr_->height = 1;

	cout << "> kinfu fetched cloud_ptr_ has this many points: "
			<< (int) cloud_ptr_->points.size() << endl;

	display_cloud(cloud_ptr_);
	pcl::io::savePCDFileASCII("/home/martin/bagfiles/slc.pcd", *cloud_ptr_);

}

ObjectDetector::~ObjectDetector() {
	// TODO Auto-generated destructor stub
}

