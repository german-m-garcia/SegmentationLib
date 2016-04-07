/*
 * ObjectDetector.cpp
 *
 *  Created on: 16 Nov 2015
 *      Author: martin
 */

#include "objects/ObjectDetector.h"
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

/*
 * Kinfu
 */

//#include <pcl/gpu/kinfu/raycaster.h>
//#include <pcl/gpu/kinfu/marching_cubes.h>
//#include <pcl/gpu/containers/initialization.h>
#include <pcl/common/angles.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/range/iterator_range.hpp>
#include <boost/filesystem.hpp>

//#include "cuda_runtime.h"

ObjectDetector::ObjectDetector() :
		threshold_positive_class(THRESHOLD_POSITIVE_CLASS),threshold_score_gicp_(THRESHOLD_SCORE_GICP), object_name_(""),
		low_sub_sampling(false),high_sub_sampling(false){
	// TODO Auto-generated constructor stub


}

ObjectDetector::ObjectDetector(int mode, std::string svm_path,
		std::string model_path) :
		svm(svm_path), model_path_(model_path), train_(mode == TRAIN_MODE), test_(
				mode == TEST_MODE), threshold_positive_class(THRESHOLD_POSITIVE_CLASS),threshold_score_gicp_(THRESHOLD_SCORE_GICP), object_name_(
				""), svm_tmp_data(model_path_ + "/svmwrapper.data"), low_sub_sampling(false),high_sub_sampling(false) {
	cout << " svm_path=" << svm_path << endl;
	cout << " model_path=" << model_path << endl;
	if (test_) {
		cout << "> loading svm model for testing..." << endl;
		svm.load_model();
		//loads currently stored data if it exists
		load_current_data();
	} else {
		create_dirs();
	}


}

ObjectDetector::ObjectDetector(int mode, std::string svm_path,
		string model_path, string object_name) :
		svm(svm_path), model_path_(model_path), train_(mode == TRAIN_MODE), test_(
				mode == TEST_MODE), threshold_positive_class(THRESHOLD_POSITIVE_CLASS),threshold_score_gicp_(THRESHOLD_SCORE_GICP), object_name_(
				object_name), svm_tmp_data(model_path_ + "/svmwrapper.data"), low_sub_sampling(false),high_sub_sampling(false) {

	cout << " svm_path=" << svm_path << endl;
	cout << " model_path=" << model_path << endl;
	cout << " object_name=" << object_name << endl;

	if (test_) {
		cout << "> loading svm model for testing..." << endl;
		svm.load_model();
		//loads currently stored data if it exists
		load_current_data();

	} else {
		create_dirs();
	}


}

/*
 * !brief Creates the necessary directories for storing the
 * point clouds, images, depth maps and so far computed features
 * of this object detector.
 *
 */
void ObjectDetector::create_dirs() {

	boost::filesystem::path dir_clouds(model_path_ + "/clouds");
	boost::filesystem::path dir_imgs(model_path_ + "/imgs");
	boost::filesystem::path dir_depths(model_path_ + "/depths");
	boost::filesystem::path dir_mats(model_path_ + "/mats");
	boost::filesystem::create_directory(dir_clouds);
	boost::filesystem::create_directory(dir_imgs);
	boost::filesystem::create_directory(dir_depths);
	boost::filesystem::create_directory(dir_mats);
}

void ObjectDetector::load_point_clouds() {
	boost::filesystem::path dir_clouds(model_path_ + "/clouds");

	cout << ">  ObjectDetector::load_point_clouds path clouds = " << dir_clouds
			<< endl;

	if (boost::filesystem::is_directory(dir_clouds)) {
		cout << dir_clouds << " is a directory containing:\n";

		typedef vector<boost::filesystem::path> vec;             // store paths,
		vec v;                                // so we can sort them later

		copy(boost::filesystem::directory_iterator(dir_clouds),
				boost::filesystem::directory_iterator(), back_inserter(v));

		sort(v.begin(), v.end());             // sort, since directory iteration
											  // is not ordered on some file systems

		for (vec::const_iterator it(v.begin()); it != v.end(); ++it) {
			cout << "   " << *it << '\n';
			string cloud_file = (*it).string();

			Cloudptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (pcl::io::loadPCDFile < pcl::PointXYZRGB > (cloud_file, *cloud) == -1) //* load the file
					{
				PCL_ERROR("Couldn't read file test_pcd.pcd \n");
				return;
			}
			point_clouds_.push_back(cloud);
			mrsmap.addPcl(cloud);
			mrsmaps_.push_back(mrsmap);

		}

	}


}

void ObjectDetector::activate_ignore_registration(){
	this->ignore_registration_ = true;
}

void ObjectDetector::activate_high_sub_sampling(){
	this->high_sub_sampling = true;
	if(high_sub_sampling)
		threshold_score_gicp_=TIGHTER_THRESHOLD_SCORE_GICP;
}

void ObjectDetector::deactivate_high_sub_sampling(){
	this->high_sub_sampling = false;
}

int ObjectDetector::get_number_train_examples(){
	return point_clouds_.size();
}

void ObjectDetector::load_current_data() {
	svm.load_current_data(svm_tmp_data);
	//load point clouds
	load_point_clouds();
}

void ObjectDetector::save_current_data() {

	//save the stored frames, depth maps and point clouds
	for (unsigned int i = 0; i < point_clouds_.size(); i++) {
		std::string cloud_name(
				model_path_ + "/clouds/cloud_" + utils_.stringify((int) i)
						+ ".pcd");
		cout << "> saving point cloud: " << cloud_name << endl;
		pcl::io::savePCDFileASCII(cloud_name, *point_clouds_[i]);
	}
	for (unsigned int i = 0; i < frames_.size(); i++) {
		cv::Mat& frame = frames_[i];
		std::string name(
				model_path_ + "/imgs/frame_" + utils_.stringify((int) i)
						+ ".png");
		cv::imwrite(name, frame);
	}
	//save the SVM wrapper as it is right now
	svm.save_current_data(svm_tmp_data);

}

void ObjectDetector::train() {
	svm.trainSVM();
	save_current_data();
}

void ObjectDetector::test_data(std::vector<Segment*>& test_segments) {

	svm.testSVM(test_segments);
}


/*
 * !brief Takes as input a binary image with all the detections
 * and splits them into individual objects and their bounding boxes
 *
 */
void ObjectDetector::split_detections_masks_rects(cv::Mat& src, vector<Rect>& rects,
		vector<cv::Mat>& masks) {

	cv::Mat mask = src.clone();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Find contours
	findContours(mask, contours, hierarchy, cv::RETR_CCOMP,
			CV_CHAIN_APPROX_NONE, Point(0, 0));
	/// Draw contours
	cv::RNG rng(12345);
	cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC3);
	for (unsigned int i = 0; i < contours.size(); i++) {
		//if it has parents we skip it at first
		if (hierarchy[i][3] != -1) {
			continue;
		}
//		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
//				rng.uniform(0, 255));
		//drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		cv::Mat object = cv::Mat::zeros(mask.size(), CV_8UC1);
		drawContours(object, contours, i, Scalar(255), -1);
		Rect rect = boundingRect(contours[i]);
		masks.push_back(object);
		rects.push_back(rect);
	}
}

void ObjectDetector::test_pcl_segments(cv::Mat&img, cv::Mat& depth_float,
		vector<Segment*>& fg_segments) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals;

	//obtain point cloud from the image and depth maps
	Utils utils;
	utils.image_to_pcl(img, depth_float, pcl_cloud);
	utils.compute_integral_normals(pcl_cloud, normals);

	//add the point cloud data to the segments
	//so that they can have computed 3D features
	for (Segment* seg : fg_segments) {
		seg->add_precomputed_pcl(pcl_cloud, normals);
		seg->computeFeatures();
	}

	//crop the part of the cloud that corresponds to the input segments
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>), cropped_cloud_rotated;
	cv::Mat tmp_img, tmp_depth;
	utils.cropped_pcl_from_segments(img, depth_float, fg_segments,
			cropped_cloud, tmp_img, tmp_depth);

	cv::Point3d gravity_center;
	//center the point cloud on the gravity center and rotate it around its eigen axes
	normalize_pcl(cropped_cloud, cropped_cloud_rotated, gravity_center);
	//subsample the point cloud
	utils.sub_sample(cropped_cloud_rotated, cropped_cloud_rotated);

	//iterate over the stored models of the object
	for (MRSMapWrap& stored_map : mrsmaps_) {
		//and try to register the point cloud with them
		double score = stored_map.test_cloud(cropped_cloud_rotated);
		cout <<"> ObjectDetector::test_pcl_segments GICP score="<<score<<endl;
	}

}

/*
 * !brief The function should be called for every frame for which we want
 * to add foreground and background segments to the model
 * The point cloud of the object is computed and added to the pcl_clouds vector
 *
 */
void ObjectDetector::add_selected_segments(cv::Mat&img, cv::Mat& depth_float,
		vector<Segment*>& fg_segments, vector<Segment*>& bg_segments) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>), filtered(
			new pcl::PointCloud<pcl::PointXYZRGB>), filtered_copy(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals;

	Utils utils;
	utils.image_to_pcl(img, depth_float, pcl_cloud);
	utils.compute_integral_normals(pcl_cloud, normals);

	for (Segment* seg : fg_segments) {
		seg->add_precomputed_pcl(pcl_cloud, normals);
		seg->computeFeatures();
	}
	for (Segment* seg : bg_segments) {
		seg->add_precomputed_pcl(pcl_cloud, normals);
		seg->computeFeatures();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud_rotated, cropped_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	cv::Mat tmp_img, tmp_depth;
	//obtains the point cloud that corresponds to the selected segments
	utils.cropped_pcl_from_segments(img, depth_float, fg_segments,
			cropped_cloud, tmp_img, tmp_depth);

	cv::Point3d gravity_center;
	//centers the point cloud on the gravity center and rotates it around its eigen axes
	normalize_pcl(cropped_cloud, cropped_cloud_rotated, gravity_center);
	//sub samples the point cloud to reduce the amount of points
	if(cropped_cloud_rotated->size() > MIN_POINTS_TO_SUBSAMPLE)
		utils.sub_sample(cropped_cloud_rotated, cropped_cloud_rotated);
	mrsmap.addPcl(cropped_cloud_rotated);
	mrsmaps_.push_back(mrsmap);
	//add training data to the svm and store img, depth map and point cloud
	add_training_data(fg_segments, bg_segments, cropped_cloud_rotated, img,
			depth_float);
}

void ObjectDetector::normalize_pcl(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud,
		Point3d& gravity_center) {



	Eigen::Matrix4f projectionTransform;
	normalize_pcl(pcl_cloud,dst_cloud,gravity_center,projectionTransform);

}

void ObjectDetector::normalize_pcl(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud,
		Point3d& gravity_center,Eigen::Matrix4f& projectionTransform) {

	Utils utils;
	double max_z = 0.;


	std::vector<int> indices;
	dst_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	//utils.remove_outliers(copy_cloud, copy_cloud);

	//copy the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*pcl_cloud, *copy_cloud);

	//prune it
	utils.prune_pcl(copy_cloud, copy_cloud);

	//subtract gravity center
	utils.xyz_gravity_center(copy_cloud, gravity_center, max_z);
	//shift the point cloud to the origin
	utils.subtract_gravity_center(copy_cloud, gravity_center);
	utils.obtain_eigen_axes(copy_cloud, projectionTransform);
	string text1("first cloud");
	//utils.display_cloud(copy_cloud,text1);

	//now operate this changes on the original point cloud
	pcl::removeNaNFromPointCloud(*pcl_cloud, *pcl_cloud, indices);
	utils.remove_zeros(pcl_cloud, indices);
	utils.subtract_gravity_center(pcl_cloud, indices, gravity_center);
	//rotate along its eigenvectors
	pcl::transformPointCloud(*pcl_cloud, *dst_cloud, projectionTransform);

	//set the displacement back into the projection transformation
	//projectionTransform(0, 3) = gravity_center.x;
	//projectionTransform(1, 3) = gravity_center.y;
	//projectionTransform(2, 3) = gravity_center.z;


//	Eigen::Vector4d centroid;
//		pcl::compute3DCentroid(*pcl_cloud,centroid);
//		cout <<" compute3DCentroid="<<centroid<<endl;
}

/*
 * Mat src: the original image in colour
 * Mat mask: the binary map with the detections
 * Mat debug: the output image with the contour of each detection
 * overlaid on the input image
 *
 */
void ObjectDetector::draw_contours_detections(cv::Mat& src, cv::Mat& mask,
		cv::Mat& debug) {

	debug = src.clone();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Find contours
	findContours(mask, contours, hierarchy, cv::RETR_CCOMP,
			CV_CHAIN_APPROX_NONE, Point(0, 0));
	/// Draw contours
	RNG rng(5345);
	cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC3);
	for (unsigned int i = 0; i < contours.size(); i++) {
		//if it has parents we skip it at first
		if (hierarchy[i][3] != -1) {
			continue;
		}
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
				rng.uniform(200, 255));
		//drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		Mat object = Mat::zeros(mask.size(), CV_8UC1);
		int thickness = 5;
		drawContours(debug, contours, i, color, thickness);

		Rect boundRect = boundingRect(Mat(contours[i]));
		putText(debug, object_name_, Point(boundRect.x, boundRect.y),
				cv::FONT_HERSHEY_SIMPLEX, 1.5, color, 5);

	}

	/// Show in a window
	//namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	//imshow("Contours", drawing);
	//waitKey(0);

}

/*
 * Mat src: the original image in colour
 * Mat mask: the binary map with the detections
 * Mat debug: the output image with the contour of each detection
 * overlaid on the input image
 *
 */
void ObjectDetector::draw_contours_detections(cv::Mat& src, vector<cv::Mat>& masks,
		cv::Mat& debug) {

	debug = src.clone();

	for(cv::Mat& mask : masks){
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		/// Find contours
		findContours(mask, contours, hierarchy, cv::RETR_CCOMP,
				CV_CHAIN_APPROX_NONE, Point(0, 0));
		/// Draw contours
		RNG rng(5345);
		cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC3);
		for (unsigned int i = 0; i < contours.size(); i++) {
			//if it has parents we skip it at first
			if (hierarchy[i][3] != -1) {
				continue;
			}
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
					rng.uniform(200, 255));
			//drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
			Mat object = Mat::zeros(mask.size(), CV_8UC1);
			int thickness = 5;
			drawContours(debug, contours, i, color, thickness);

			Rect boundRect = boundingRect(Mat(contours[i]));
			putText(debug, object_name_, Point(boundRect.x, boundRect.y),
					cv::FONT_HERSHEY_SIMPLEX, 1.5, color, 5);

		}
	}


}

void ObjectDetector::unify_detections(Mat& mask) {

	//values used to be 5 and 2
	for (int i = 0; i < 5; i++)
		dilate(mask, mask, Mat());

	for (int i = 0; i < 2; i++)
		erode(mask, mask, Mat());
//	imshow("unified mask",mask);
//	waitKey(0);
}

int ObjectDetector::get_detections(std::vector<Segment*>& test_segments,
		Mat& original_img, Mat& original_depth, Mat& detections) {


	cout <<" calling get_detections"<<endl;
	/*
	 * compute the point cloud
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals;

	Utils utils;
	//cout <<"> ObjectDetector::get_detections computing point cloud. Sizes= "<<original_img.size()<<" "<<original_depth.size()<<endl;
	utils.image_to_pcl(original_img, original_depth, pcl_cloud);
	//string text("cloud");
	//utils.display_cloud(pcl_cloud,text);
	utils.compute_integral_normals(pcl_cloud, normals);

	//cout <<" iterating over "<<test_segments.size()<<" segments"<<endl;
	for (Segment *seg : test_segments) {
		seg->add_precomputed_pcl(pcl_cloud, normals);
		//seg->addPcl(img_1,depth_float);
		seg->computeFeatures();
		if(high_sub_sampling){
			seg->set_original_mat_segment_mat();
		}
	}




	svm.testSVM(test_segments);
	if (test_segments.size() == 0)
		return 0;
	Mat ref = test_segments[0]->getBinaryMat();
	detections = Mat::zeros(ref.rows, ref.cols, CV_8UC3);
	//cout <<" iterating segments"<<endl;
	int ndetections = 0;
	//cout <<"> ObjectDetector::get_detections evaluating the segments"<<endl;
	for (Segment *seg : test_segments) {


		if (seg->getClassLabel() > threshold_positive_class) {
			//cout <<" bounding rect="<<seg->getBoundRect()<<endl;
			//cout <<"detections.size()="<<detections.size()<<endl;
			//cout <<"detections(seg->getBoundRect()).size()="<<detections(seg->getBoundRect()).size()<< " seg->getRandomColourMat().size()= "<<seg->getRandomColourMat().size()<<endl;

			//imshow("seg->getRandomColourMat()", seg->getRandomColourMat());
			//waitKey(0);
			detections(seg->getBoundRect()) += seg->getRandomColourMat();
			ndetections++;
			cout <<"detection confidence="<<seg->getClassLabel()<<endl;
		}
	}
	return ndetections;
}

void ObjectDetector::activate_low_sub_sampling(){
	this->low_sub_sampling = true;
}

void ObjectDetector::deactivate_low_sub_sampling(){
	this->low_sub_sampling = false;
}



bool ObjectDetector::test_data(std::vector<Segment*>& test_segments,
		Mat& original_img, Mat& original_depth, vector<Mat>& masks,
		Mat& debug,vector<Detection>& detections_vector, bool unify) {

	debug = original_img.clone();


	Mat detections;
	int ndetections = get_detections(test_segments, original_img,original_depth,detections);
	if (ndetections == 0){
		debug = original_img.clone();
		//cout <<"> ObjectDetector::test_data:: ndetections == 0 "<<endl;
		return false;
	}


	//get the mask and minimum bounding rectangle
	Rect rect;
	Mat pointsMat;
	resize(detections, detections, original_img.size());
	Mat mask;
	cvtColor(detections, mask, CV_RGB2GRAY);
	mask = mask > 0;
	if(unify)
		unify_detections(mask);
	else{
		//dilate(mask,mask,Mat());
	}

	//split the mask into individual detections
	vector<Rect> rects;
	cv::imshow("test_data mask",mask);
	cv::waitKey(1);
	split_detections_masks_rects(mask, rects, masks);
	vector<Mat> masks_verified;

	//iterate over each detection and verify it
	for (Mat mask : masks) {
		Cloudptr cloud_detection,save_original_cloud_detection(new pcl::PointCloud<pcl::PointXYZRGB>);
		Mat tmp_img,tmp_depth;
		cv::Point3d gravity_center;

		//obtain the point cloud of this detection

		//1-fill the holes in this mask
		utils_.fill_mask(mask);
		Point2i mid_point(0.,0.);
		utils_.find_mid_point_mask(mask,mid_point);

		utils_.cropped_pcl_from_mask(original_img,original_depth,mask,cloud_detection, tmp_img,tmp_depth);


		cout <<" detection of "<<cloud_detection->size()<<" points "<<endl;
		if(cloud_detection->size() > MIN_POINTS_TO_SUBSAMPLE){

			//if it's a holder
			if(high_sub_sampling){
				cout <<" high_sub_sampling"<<endl;
				utils_.sub_sample(cloud_detection, save_original_cloud_detection);
				utils_.remove_HOLDER_outliers(save_original_cloud_detection,save_original_cloud_detection);
			}
			//if it's a screw
			else if(low_sub_sampling){
				utils_.sub_sample_screw(cloud_detection, save_original_cloud_detection);
				utils_.remove_SCREW_outliers(save_original_cloud_detection,save_original_cloud_detection);
			}

			else{
				utils_.sub_sample(cloud_detection, save_original_cloud_detection);
				utils_.remove_outliers(save_original_cloud_detection,save_original_cloud_detection);

			}


			pcl::copyPointCloud(*save_original_cloud_detection,*cloud_detection);
			//cloud_detection = save_original_cloud_detection;
		}
		else{
			pcl::copyPointCloud(*cloud_detection,*save_original_cloud_detection);
		}
		cout <<" filtered detection of "<<cloud_detection->size()<<" points "<<endl;

		//string text("candidate detection");
		//utils_.display_cloud(cloud_detection,text);

		//crop the part of the cloud that corresponds to the input segments
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cropped_cloud_rotated;

		//center the point cloud on the gravity center and rotate it around its eigen axes
		Eigen::Matrix4f projectionTransform;
		normalize_pcl(cloud_detection, cropped_cloud_rotated, gravity_center,projectionTransform);





		Point3d detection_dimensions_3d;
		utils_.compute_bounding_box(cropped_cloud_rotated,detection_dimensions_3d);
		std::cout <<"ObjectDetector::test_data detection dimensions: "<<detection_dimensions_3d.x<<" "<<detection_dimensions_3d.y<<" "<<detection_dimensions_3d.z<<std::endl;

		//subsample the point cloud
		//if(cropped_cloud_rotated->size() > MIN_POINTS_TO_SUBSAMPLE)
		//	utils_.sub_sample(cropped_cloud_rotated, cropped_cloud_rotated);
		if(cropped_cloud_rotated->size() < 10){
			cout <<" discarding candidate of "<<cropped_cloud_rotated->size()<<" points"<<endl;
			continue;
		}



		//iterate over the stored models of the object
		double min_score = 99999.;
		Eigen::Matrix4f best_transform;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud;

		//get the model of the object
		MRSMapWrap& stored_map = mrsmaps_[0];
		Point3d model_dimensions_3d;
		model_cloud = stored_map.getModelCloud();
		utils_.compute_bounding_box(model_cloud,model_dimensions_3d);
		std::cout <<"ObjectDetector::test_data model_dimensions_3d dimensions: "<<model_dimensions_3d.x<<" "<<model_dimensions_3d.y<<" "<<model_dimensions_3d.z<<std::endl;


		double score_x = model_dimensions_3d.x > detection_dimensions_3d.x ? detection_dimensions_3d.x/model_dimensions_3d.x: model_dimensions_3d.x/detection_dimensions_3d.x;
		double score_y = model_dimensions_3d.y > detection_dimensions_3d.y ? detection_dimensions_3d.y/model_dimensions_3d.y: model_dimensions_3d.y/detection_dimensions_3d.y;
		double score_z = model_dimensions_3d.z > detection_dimensions_3d.z ? detection_dimensions_3d.z/model_dimensions_3d.z: model_dimensions_3d.z/detection_dimensions_3d.z;

		//average the scores obtained for each dimension of the detection
		if(high_sub_sampling)
			min_score = 1 - (score_x*0.6+score_y*0.4);
		else if(low_sub_sampling)
			min_score = 1 - (score_x*0.6 + score_y*0.2+ score_z*0.2);// (score_x*0.5 +score_y*0.25+score_z*0.25)/3.;
		else
			min_score = 1 - (score_x+score_y+score_z)/3.;
		Eigen::Matrix4f model_transform;
		//register the model to the detection
		stored_map.test_cloud(cropped_cloud_rotated,model_transform);
		best_transform = model_transform;

//		{
//			Eigen::Matrix4f transform_1 = best_transform.inverse(); //????
//			//the transform from the original pose of the detection to the normalized frame of reference
//			Eigen::Matrix4f transform_2 = projectionTransform.inverse();
//			//shift the point cloud to the origin
//			Point3d gravity_shift(-gravity_center.x,-gravity_center.y,-gravity_center.z);
//			pcl::transformPointCloud(*model_cloud, *model_cloud, transform_1);
//			pcl::transformPointCloud(*model_cloud, *model_cloud, transform_2);
//			utils_.subtract_gravity_center(model_cloud, gravity_shift);
//			utils_.display_cloud(model_cloud,text);
//		}



		cout<<" score based on dimensions="<<min_score<<endl;


		if(ignore_registration_ || min_score < threshold_score_gicp_){
			masks_verified.push_back(mask);

			//transform the model to the frame of reference cloud

			//the transform of the model to the detection in the normalized frame of reference
			Eigen::Matrix4f transform_1 = best_transform.inverse(); //????
			//the transform from the original pose of the detection to the normalized frame of reference
			Eigen::Matrix4f transform_2 = projectionTransform.inverse();



			Detection detection;

			pcl::copyPointCloud(*model_cloud, *detection.model_cloud);
			pcl::copyPointCloud(*save_original_cloud_detection, *detection.cloud);

			detection.mid_point = mid_point;
			detection.confidence = min_score;
			detection.position = gravity_center;
			detections_vector.push_back(detection);

		}
		else
			cout <<" discarding candidate because of dimensions score"<<endl;
		//cv::imshow("current detection",mask);
		//cv::waitKey(0);


	}
	//sort the detections
	if(ignore_registration_)
		sort(detections_vector.begin(),detections_vector.end(),Detection::sort_detections_right_most);
	else
		sort(detections_vector.begin(),detections_vector.end(),Detection::sort_detections);
	for(Detection& detect : detections_vector){
		cout <<" detection confidence = "<<detect.confidence<<endl;
	}

	//draw the contours around the detections
	draw_contours_detections(original_img, masks_verified, debug);





	return true;

}

//Mat& ref = segmentation_1.getOutputSegmentsPyramid()[scale_for_propagation];
bool ObjectDetector::test_data(std::vector<Segment*>& test_segments,
		Mat& original_img, Mat& original_depth, vector<Mat>& masks, Mat& debug,
		vector<Point3d>& slc_positions, vector<Point3d>& slc_orientations, bool unify) {

	debug = original_img.clone();


	Mat detections;
	int ndetections = get_detections(test_segments, original_img,original_depth,detections);
	if (ndetections == 0)
		return false;
	if (ndetections == 0)
		return false;

	//get the mask and minimum bounding rectangle
	Rect rect;
	Mat pointsMat;
	resize(detections, detections, original_img.size());
	Mat mask;
	cvtColor(detections, mask, CV_RGB2GRAY);
	mask = mask > 0;
	if(unify)
		unify_detections(mask);
	else{
		dilate(mask,mask,Mat());
	}

	vector<Rect> rects;
	split_detections_masks_rects(mask, rects, masks);

	for (Mat mask : masks) {
		Point3d slc_position;
		Point3d slc_orientation;
		//iterate for each blob in the mask
		cv::findNonZero(mask, pointsMat);
		rect = boundingRect(pointsMat);
		rectangle(debug, rect, Scalar(0, 0, 255), 3);

		utils_.find_detection_yaw(mask, original_img, original_depth,
				slc_position, slc_orientation);
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

/*
 * !brief Adds positive and negative examples to the SVM
 * Stores the image and depth map of the positive examples
 *
 *
 */
void ObjectDetector::add_training_data(
		std::vector<Segment*>& foreground_segments,
		std::vector<Segment*>& background_segments, Cloudptr point_cloud,
		Mat& img, Mat& depth) {
	svm.add_training_data(foreground_segments, background_segments);
	point_clouds_.push_back(point_cloud);
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

	for (Cloudptr& cloud : point_clouds_) {
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
	if (point_clouds_.size() < 2)
		return;
	Cloudptr& cloud_1 = point_clouds_[0];
	Cloudptr& cloud_2 = point_clouds_[1];

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

/*
 * !brief Runs the KinectFusion algorithm for the images and depth maps
 * stored in frames_ and depths_ respectively. Takes as input the required
 * volume size
 *
 * @vsz [input] the volume size
 *
 */
void ObjectDetector::run_kinfu(float vsz) {

//	pcl::gpu::KinfuTracker kinfu_(rows, cols);
//	//Init Kinfu Tracker
//
//	kinfu_.setDepthIntrinsics(fx, fy, cx, cy);
//
//	Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(vsz/*meters*/);
//	kinfu_.volume().setSize(volume_size);
//
//	Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
//	Eigen::Vector3f t = volume_size * 0.5f
//			- Eigen::Vector3f(0, 0, volume_size(2) / 2 * 1.2f);
//
//	Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);
//
//	kinfu_.setInitalCameraPose(pose);
//	kinfu_.volume().setTsdfTruncDist(0.030f/*meters*/);
//	kinfu_.setIcpCorespFilteringParams(0.1f/*meters*/, sin(pcl::deg2rad(20.f)));
//	//kinfu_.setDepthTruncationForICP(5.f/*meters*/);
//	kinfu_.setCameraMovementThreshold(0.0001f);
//
////	const int max_color_integration_weight = 2;
////	kinfu_.initColorIntegration(max_color_integration_weight);
//
//	pcl::gpu::KinfuTracker::DepthMap depth_device_;
//	pcl::gpu::KinfuTracker::View colors_device_;
//	for (unsigned int i = 0; i < frames_.size(); i++) {
//		Mat frame = frames_[i];
//		Mat depth = depths_[i];
//
//		//imshow("kinfuing with this frame",frame);
//		//waitKey(0);
//
//		//upload depth data to GPU
//		pcl::gpu::PtrStepSz<const unsigned short int> depth_;
//		depth_.cols = depth.cols;
//		depth_.rows = depth.rows;
//		depth_.step = depth_.cols * sizeof(const unsigned short int);//depth_.elemSize();
//
//		//cout <<" params: depth_.step="<<depth_.step<<" depth_.cols="<<depth_.cols<<" depth_.rows="<<depth_.rows<<endl;
//		std::vector<unsigned short int> source_depth_data_;
//		source_depth_data_.resize(depth_.cols * depth_.rows);
//
//		memcpy(&source_depth_data_[0], depth.data,
//				sizeof(unsigned short int) * depth.cols * depth.rows);
//		depth_.data = &source_depth_data_[0];
//		depth_device_.upload(depth_.data, depth_.step, depth_.rows,
//				depth_.cols);
//
//		kinfu_(depth_device_);
//
//	}
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_(
//			new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::gpu::DeviceArray<pcl::PointXYZ> cloud_buffer_device_;
//
//	pcl::gpu::DeviceArray<pcl::PointXYZ> extracted = kinfu_.volume().fetchCloud(
//			cloud_buffer_device_);
//	extracted.download(cloud_ptr_->points);
//	cloud_ptr_->width = (int) cloud_ptr_->points.size();
//	cloud_ptr_->height = 1;
//
//	cout << "> kinfu fetched cloud_ptr_ has this many points: "
//			<< (int) cloud_ptr_->points.size() << endl;
//
//	display_cloud(cloud_ptr_);
//	pcl::io::savePCDFileASCII("/home/martin/bagfiles/slc.pcd", *cloud_ptr_);

}

void ObjectDetector::get_point_cloud_at(int i,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud) {
	if (i >= (int) point_clouds_.size()) {
		pcl_cloud = nullptr;
		return;
	}
	pcl_cloud = point_clouds_[i];
}

ObjectDetector::~ObjectDetector() {
	// TODO Auto-generated destructor stub
}

