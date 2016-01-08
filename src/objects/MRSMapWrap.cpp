/*
 * MRSMapWrap.cpp
 *
 *  Created on: 7 Jan 2016
 *      Author: martin
 */

#include "objects/MRSMapWrap.h"
#include "utils.h"

MRSMapWrap::MRSMapWrap() {
	// TODO Auto-generated constructor stub

}

MRSMapWrap::~MRSMapWrap() {
	// TODO Auto-generated destructor stub
}

void MRSMapWrap::show(){

}

void MRSMapWrap::initialize(cv::Mat& src, cv::Mat& depth, std::vector<Segment*>& segments){
	Utils utils;
	cv::Mat tmp_img,tmp_depth;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	utils.cropped_pcl_from_segments(src, depth,segments, cloud, tmp_img, tmp_depth);
	std::string text("MRSMapWrap cloud");
	utils.display_cloud(cloud,text);
	initialize(cloud);
}

void MRSMapWrap::initialize(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudIn) {

	// change reference frame of point cloud to point mean and oriented along principal axes
	Eigen::Vector4d mean;
	Eigen::Vector3d eigenvalues;
	Eigen::Matrix3d cov;
	Eigen::Matrix3d eigenvectors;
	pcl::computeMeanAndCovarianceMatrix(*pointCloudIn, cov, mean);
	pcl::eigen33(cov, eigenvectors, eigenvalues);

	if (Eigen::Vector3d(eigenvectors.col(0)).dot(Eigen::Vector3d::UnitZ())
			> 0.0)
		eigenvectors.col(0) = (-eigenvectors.col(0)).eval();

	// transform from object reference frame to camera
	Eigen::Matrix4d objectTransform = Eigen::Matrix4d::Identity();
	objectTransform.block<3, 1>(0, 0) = eigenvectors.col(2);
	objectTransform.block<3, 1>(0, 1) = eigenvectors.col(1);
	objectTransform.block<3, 1>(0, 2) = eigenvectors.col(0);
	objectTransform.block<3, 1>(0, 3) = mean.block<3, 1>(0, 0);

	if (objectTransform.block<3, 3>(0, 0).determinant() < 0) {
		objectTransform.block<3, 1>(0, 0) = -objectTransform.block<3, 1>(0, 0);
	}

	Eigen::Matrix4d objectTransformInv = objectTransform.inverse();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectPointCloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::transformPointCloud(*pointCloudIn, *objectPointCloud,
			(objectTransformInv).cast<float>());

	objectPointCloud->sensor_origin_ =
			objectTransformInv.block<4, 1>(0, 3).cast<float>();
	objectPointCloud->sensor_orientation_ = Eigen::Quaternionf(
			objectTransformInv.block<3, 3>(0, 0).cast<float>());

	treeNodeAllocator_->reset();
	map_ = boost::shared_ptr<MultiResolutionSurfelMap>(
			new MultiResolutionSurfelMap(max_resolution_, max_radius_,
					treeNodeAllocator_));

	map_->params_.dist_dependency = dist_dep_;

	std::vector<int> pointIndices(objectPointCloud->points.size());
	for (unsigned int i = 0; i < pointIndices.size(); i++)
		pointIndices[i] = i;
	map_->imageAllocator_ = imageAllocator_;
	map_->addPoints(*objectPointCloud, pointIndices);
	map_->octree_->root_->establishNeighbors();
	map_->evaluateSurfels();
	map_->buildShapeTextureFeatures();

	map_->save(map_folder_ + "/" + object_name_ + ".map");
}
