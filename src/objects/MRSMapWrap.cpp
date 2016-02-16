/*
 * MRSMapWrap.cpp
 *
 *  Created on: 7 Jan 2016
 *      Author: martin
 */

#include "objects/MRSMapWrap.h"

#include <Eigen/Core>

#include <pcl/filters/filter.h>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/from_meshes.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_validation_euclidean.h>

MRSMapWrap::MRSMapWrap() {

//	max_radius_ = 30.f;
//	min_resolution_ = 0.0125f;
//
//	imageAllocator_ = boost::shared_ptr<
//			MultiResolutionSurfelMap::ImagePreAllocator>(
//			new MultiResolutionSurfelMap::ImagePreAllocator());
//
//	modelImageAllocator_ = boost::shared_ptr<
//			MultiResolutionSurfelMap::ImagePreAllocator>(
//			new MultiResolutionSurfelMap::ImagePreAllocator());
//
//	curMapTreeNodeAllocator_ = boost::shared_ptr<
//			spatialaggregate::OcTreeNodeDynamicAllocator<float,
//					MultiResolutionSurfelMap::NodeValue> >(
//			new spatialaggregate::OcTreeNodeDynamicAllocator<float,
//					MultiResolutionSurfelMap::NodeValue>(1000));
//
//	modelTreeNodeAllocator_ = boost::shared_ptr<
//			spatialaggregate::OcTreeNodeDynamicAllocator<float,
//					MultiResolutionSurfelMap::NodeValue> >(
//			new spatialaggregate::OcTreeNodeDynamicAllocator<float,
//					MultiResolutionSurfelMap::NodeValue>(1000));

}

MRSMapWrap::~MRSMapWrap() {
	// TODO Auto-generated destructor stub
}

void MRSMapWrap::show() {

}

void MRSMapWrap::addPcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {

	model_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new  pcl::PointCloud<pcl::PointXYZRGB>);
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	//sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.setLeafSize(0.005f, 0.005f, 0.005f);
	sor.filter(*model_cloud_);

	//model_cloud_ = cloud;
	//std::string text("model cloud");
	//utils_.display_cloud(model_cloud_, text);



//	float register_start_resolution = min_resolution_;
//	const float register_stop_resolution = 32.f * min_resolution_;
//	model_gravity_center_ = gravity_center;
//	curMapTreeNodeAllocator_->reset();
//	current_mrsmap = boost::shared_ptr<MultiResolutionSurfelMap>(
//			new MultiResolutionSurfelMap(register_stop_resolution, max_radius_,
//					curMapTreeNodeAllocator_));
//
//	//current_mrsmap->params_.dist_dependency = dist_dep_;
//
//	std::vector<int> pointIndices; //(cloud->points.size());
////	for (unsigned int i = 0; i < pointIndices.size(); i++){
////		if(cloud->points[i].x != 0.)
////			pointIndices[i] = i;
////
////	}
//	pcl::removeNaNFromPointCloud(*cloud, *cloud, pointIndices);
//	Utils utils;
//	utils.remove_zeros(cloud, pointIndices);
//
//	current_mrsmap->imageAllocator_ = imageAllocator_;
//	current_mrsmap->addPoints(*cloud, pointIndices);
//
//
//	//current_mrsmap->addImage(*cloud, false,true);
//	current_mrsmap->octree_->root_->establishNeighbors();
//	current_mrsmap->evaluateSurfels();
//	current_mrsmap->buildShapeTextureFeatures();
//
//	//pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv( new pcl::PointCloud< pcl::PointXYZRGB >() );
//	//current_mrsmap->visualize3DColorDistribution( cloudv, -1, -1, false );
//
//	modelTreeNodeAllocator_->reset();
//
//	model_mrsmap_ = boost::shared_ptr<MultiResolutionSurfelMap>(
//			new MultiResolutionSurfelMap(register_stop_resolution, max_radius_,
//					modelTreeNodeAllocator_));
//
//	//current_mrsmap->params_.dist_dependency = dist_dep_;
//
//	model_mrsmap_->imageAllocator_ = modelImageAllocator_;
//	model_mrsmap_->addPoints(*cloud, pointIndices);
//	//model_mrsmap_->addImage(*cloud, false,true);
//	model_mrsmap_->octree_->root_->establishNeighbors();
//	model_mrsmap_->evaluateSurfels();
//	model_mrsmap_->buildShapeTextureFeatures();
//
//	Eigen::Matrix4d transform;
//	register_mrsmaps(current_mrsmap, model_mrsmap_, gravity_center,
//			model_gravity_center_, transform);

}

double MRSMapWrap::test_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,Eigen::Matrix4f& transform) {


	if(model_cloud_->size()< 20){
		cout <<"> MRSMapWrap::test_cloud :: model_cloud.size()="<<model_cloud_->size()<<endl;
		return 0.;
	}

	double score = generalized_icp(cloud, model_cloud_,transform);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::transformPointCloud(*cloud, *tmp_cloud, transform);

	cout <<"score="<<score<<endl;
	cout <<" transform="<<endl<<transform<<endl;
	std::string text("rotated cloud");
	//utils_.display_cloud(tmp_cloud, text);

	text = "model cloud";
	//utils_.display_cloud(model_cloud_, text);
	return score;
}

double MRSMapWrap::test_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {

	Eigen::Matrix4f transform;
	return test_cloud(cloud,transform);

//	float register_start_resolution = min_resolution_;
//	const float register_stop_resolution = 32.f * min_resolution_;
//	std::vector<int> pointIndices;
//	Utils utils;
//
//	curMapTreeNodeAllocator_->reset();
//
//	boost::shared_ptr<MultiResolutionSurfelMap> current_mrsmap =
//			boost::shared_ptr<MultiResolutionSurfelMap>(
//					new MultiResolutionSurfelMap(register_stop_resolution,
//							max_radius_, curMapTreeNodeAllocator_));
//
//	//current_mrsmap->params_.dist_dependency = dist_dep_;
//
//	utils.remove_zeros(cloud, pointIndices);
//	current_mrsmap->imageAllocator_ = imageAllocator_;
//	//current_mrsmap->addImage(*cloud, false,true);
//	current_mrsmap->addPoints(*cloud, pointIndices);
//	current_mrsmap->octree_->root_->establishNeighbors();
//	current_mrsmap->evaluateSurfels();
//	current_mrsmap->buildShapeTextureFeatures();
//
//	Eigen::Matrix4d transform, transformInv;
//	double confidence = register_mrsmaps(current_mrsmap, model_mrsmap_,
//			gravity_center, model_gravity_center_, transform);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud(
//			new pcl::PointCloud<pcl::PointXYZRGB>);
//	transformInv = transform.inverse();
//	pcl::transformPointCloud(*cloud, *tmp_cloud, transform);
//	std::string text("rotated cloud");
//	utils.display_cloud(tmp_cloud, text);
//	return confidence;

}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void MRSMapWrap::pairAlign(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
		Eigen::Matrix4f &final_transform, bool downsample) {
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::VoxelGrid<PointXYZRGB> grid;
//	if (downsample) {
//		grid.setLeafSize(0.05, 0.05, 0.05);
//		grid.setInputCloud(cloud_src);
//		grid.filter(*src);
//
//		grid.setInputCloud(cloud_tgt);
//		grid.filter(*tgt);
//	} else {
//		src = cloud_src;
//		tgt = cloud_tgt;
//	}
//
//	// Compute surface normals and curvature
//	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(
//			new pcl::PointCloud<pcl::PointNormal>);
//	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(
//			new pcl::PointCloud<pcl::PointNormal>);
//
//	pcl::NormalEstimation<PointXYZRGB, PointNormalT> norm_est;
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
//			new pcl::search::KdTree<pcl::PointXYZRGB>());
//	norm_est.setSearchMethod(tree);
//	norm_est.setKSearch(30);
//
//	norm_est.setInputCloud(src);
//	norm_est.compute(*points_with_normals_src);
//	pcl::copyPointCloud(*src, *points_with_normals_src);
//
//	norm_est.setInputCloud(tgt);
//	norm_est.compute(*points_with_normals_tgt);
//	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
//
//	//
//	// Instantiate our custom point representation (defined above) ...
//	MyPointRepresentation point_representation;
//	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
//	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
//	point_representation.setRescaleValues(alpha);
//
//	//
//	// Align
//	pcl::IterativeClosestPointNonLinear < PointNormalT, PointNormalT > reg;
//	reg.setTransformationEpsilon(1e-6);
//	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
//	// Note: adjust this based on the size of your datasets
//	reg.setMaxCorrespondenceDistance(0.1);
//	// Set the point representation
//	reg.setPointRepresentation(
//			boost::make_shared<const MyPointRepresentation>(
//					point_representation));
//
//	reg.setInputSource(points_with_normals_src);
//	reg.setInputTarget(points_with_normals_tgt);
//
//	//
//	// Run the same optimization in a loop and visualize the results
//	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
//	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
//	reg.setMaximumIterations(2);
//	for (int i = 0; i < 30; ++i) {
//		PCL_INFO("Iteration Nr. %d.\n", i);
//
//		// save cloud for visualization purpose
//		points_with_normals_src = reg_result;
//
//		// Estimate
//		reg.setInputSource(points_with_normals_src);
//		reg.align(*reg_result);
//
//		//accumulate transformation between each Iteration
//		Ti = reg.getFinalTransformation() * Ti;
//
//		//if the difference between this transformation and the previous one
//		//is smaller than the threshold, refine the process by reducing
//		//the maximal correspondence distance
//		if (fabs((reg.getLastIncrementalTransformation() - prev).sum())
//				< reg.getTransformationEpsilon())
//			reg.setMaxCorrespondenceDistance(
//					reg.getMaxCorrespondenceDistance() - 0.001);
//
//		prev = reg.getLastIncrementalTransformation();
//
//		// visualize current state
//		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
//	}
//
//	//
//	// Get the transformation from target to source
//	targetToSource = Ti.inverse();
//
//	//
//	// Transform target back in source frame
//	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
//
//	p->removePointCloud("source");
//	p->removePointCloud("target");
//
//	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
//	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
//	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
//	p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
//
//	PCL_INFO("Press q to continue the registration.\n");
//	p->spin();
//
//	p->removePointCloud("source");
//	p->removePointCloud("target");
//
//	//add the source to the transformed target
//	*output += *cloud_src;
//
//	final_transform = targetToSource;
}

//void MRSMapWrap::model_mean(
//		boost::shared_ptr<MultiResolutionSurfelMap>& model_1)
//
//		{
//	// initialize alignment by shifting the map centroids
//	Eigen::Vector3d scene_mean;
//	Eigen::Matrix3d scene_cov;
//	model_1->extents(scene_mean, scene_cov);
//
//	Eigen::Vector4d scene_mean4;
//	cout << "model mean=" << scene_mean << endl;
//}

double MRSMapWrap::icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,Eigen::Matrix4f& transform) {


//	std::cerr << "PointCloud before filtering: "
//				<< cloud_1->width * cloud_1->height << " data points ("
//				<< pcl::getFieldsList(*cloud_1) << ").";
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
//	// Create the filtering object
//	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//	sor.setInputCloud(cloud_1);
//	sor.setLeafSize(0.03f, 0.03f, 0.03f);
//	sor.filter(*cloud_filtered);
//	cloud_1 = cloud_filtered;
//	std::cerr << "PointCloud after filtering: "
//			<< cloud_1->width * cloud_1->height << " data points ("
//			<< pcl::getFieldsList(*cloud_1) << ").";



	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_1);
	icp.setInputTarget(cloud_2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: "
			<< icp.getFitnessScore() << std::endl;
	transform =  icp.getFinalTransformation();
	std::cout << icp.getFinalTransformation() << std::endl;


	//validate the transformation
	pcl::registration::TransformationValidationEuclidean<pcl::PointXYZRGB, pcl::PointXYZRGB> tve;
	tve.setMaxRange (0.5);  // 50cm
	double score = tve.validateTransformation (cloud_1, cloud_2, transform);
	cout << "TransformationValidationEuclidean score ="<<score <<endl;
	score = tve.validateTransformation (cloud_2, cloud_1, transform.inverse());
	cout << "TransformationValidationEuclidean score ="<<score <<endl;
	return score;

}

/*
 * !brief Computes the Generalized ICP transform of cloud_1 to cloud_2 and returns the score
 *
 * @cloud_1: input point cloud
 * @cloud_2: input point cloud
 * @transform: the transformation
 *
 */
double MRSMapWrap::generalized_icp(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,Eigen::Matrix4f& transform) {

//	pcl::PolygonMesh::Ptr mesh1(new pcl::PolygonMesh),mesh2(new pcl::PolygonMesh);
//
//	// reconstruct meshes for source and target
//	pcl::OrganizedFastMesh<pcl::PointXYZRGB> fast_mesh;
//	fast_mesh.setInputCloud(cloud_1);
//	fast_mesh.reconstruct(*mesh1);
//
//	utils_.display_mesh(mesh1);
//
//	fast_mesh.setInputCloud(cloud_2);
//	fast_mesh.reconstruct(*mesh2);
//
//
//	// compute normals and covariances for source and target
//	pcl::PointCloud<pcl::Normal>::Ptr normals_1(
//			new pcl::PointCloud<pcl::Normal>), normals_2(
//			new pcl::PointCloud<pcl::Normal>);
//	//boost::shared_ptr< std::vector<Eigen::Matrix3d> > covs_1,covs_2;
//	boost::shared_ptr<
//			std::vector<Eigen::Matrix3d,
//					Eigen::aligned_allocator<Eigen::Matrix3d> > > covs_1(
//			new std::vector<Eigen::Matrix3d,
//					Eigen::aligned_allocator<Eigen::Matrix3d> >);
//	boost::shared_ptr<
//			std::vector<Eigen::Matrix3d,
//					Eigen::aligned_allocator<Eigen::Matrix3d> > > covs_2(
//			new std::vector<Eigen::Matrix3d,
//					Eigen::aligned_allocator<Eigen::Matrix3d> >);
//
//	pcl::features::computeApproximateNormals(*cloud_1, mesh1->polygons,
//			*normals_1);
//	pcl::features::computeApproximateCovariances(*cloud_1, *normals_1, *covs_1);
//
//	pcl::features::computeApproximateNormals(*cloud_2, mesh2->polygons,
//			*normals_2);
//	pcl::features::computeApproximateCovariances(*cloud_2, *normals_2, *covs_2);

	// setup Generalized-ICP
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
	gicp.setMaxCorrespondenceDistance(999999.0);//0.1);

	gicp.setInputSource(cloud_1);
	gicp.setInputTarget(cloud_2);
//	gicp.setSourceCovariances(covs_1);
//	gicp.setTargetCovariances(covs_2);
	// run registration and get transformation
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	gicp.align(*output);
	double score = gicp.getFitnessScore();

	string text("cloud1");
	//utils_.display_cloud(cloud_1,text);
	text = "cloud2";
	//utils_.display_cloud(cloud_2,text);
	transform = gicp.getFinalTransformation();
	//cout <<" GICP score="<<score<<endl<<" transform="<<transform<<endl;

	//validate the transformation
	pcl::registration::TransformationValidationEuclidean<pcl::PointXYZRGB, pcl::PointXYZRGB> tve;
	tve.setMaxRange (100.005);  // 50cm
	score = tve.validateTransformation (cloud_1, cloud_2, transform);
	//cout << "TransformationValidationEuclidean score ="<<score <<endl;
	double score_2 = tve.validateTransformation (cloud_2, cloud_1, transform.inverse());
	//cout << "TransformationValidationEuclidean score ="<<score <<endl;


	if(score > score_2)
		return score;
	else return score_2;
}

//double MRSMapWrap::register_mrsmaps(
//		boost::shared_ptr<MultiResolutionSurfelMap>& model_1,
//		boost::shared_ptr<MultiResolutionSurfelMap>& model_2,
//		Point3d& gravity_center_1, Point3d& gravity_center_2,
//		Eigen::Matrix4d& transform) {

//	float register_start_resolution = min_resolution_;
//	const float register_stop_resolution = 32.f * min_resolution_;
//	//estimate registration
//	transform = Eigen::Matrix4d::Identity();
//	Point3d displacement = gravity_center_2 - gravity_center_1;
//
//	cout << "gravity_center_1=" << gravity_center_1 << endl
//			<< " gravity_center_2=" << gravity_center_2 << endl
//			<< "displacement=" << displacement << endl;
//
//	transform(0, 3) = displacement.x;
//	transform(1, 3) = displacement.y;
//	transform(2, 3) = displacement.z;
//	cout
//			<< "  -----  estimating registration between model and current point cloud -----"
//			<< endl;
//	MultiResolutionSurfelRegistration reg;
//	const int GRADIENT_ITS = 200;
//	const int NEWTON_FEAT_ITS = 0;
//	const int NEWTON_ITS = 10;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrSrc;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrTgt;
//	reg.params_.match_likelihood_use_color_ = false;
//	reg.params_.match_likelihood_use_normals_ = true;
//	reg.params_.registerSurfels_ = true;
//	//reg.params_.model_visibility_max_depth_ = 13;
//
//	model_mean(model_1);
//	model_mean(model_2);
//
//	bool retVal = reg.estimateTransformation(*model_1, *model_2, transform,
//			register_stop_resolution, register_start_resolution, corrSrc,
//			corrTgt, GRADIENT_ITS, NEWTON_FEAT_ITS, NEWTON_ITS);
//
//	double confidence = 0.;
//	if (retVal) {
//		MultiResolutionSurfelRegistration reg_hard;
//		reg_hard.params_.match_likelihood_use_color_ = false;
//		cout << " model and current frame registered, transform= " << transform
//				<< endl;
//		Eigen::Matrix4d transforminv = transform.inverse();
//		double logLikelihood1 = reg_hard.matchLogLikelihood(*model_2, *model_1,
//				transforminv);
//		std::cout << "likelihood1: " << logLikelihood1 << "\n";
//
//		//double logLikelihood2 = reg.matchLogLikelihood( *model_2, *model_1, transforminv );
//		double logLikelihood2 = reg_hard.selfMatchLogLikelihood(*model_2);
//		std::cout << "likelihood2: " << logLikelihood2 << "\n";
//
//		double confidence = std::min(1.0,
//				std::max(0.0, 4.0 * (logLikelihood1 / logLikelihood2 - 0.5)));
//		cout << "detection confidence: " << confidence << endl;
//
//	}
//
//	return confidence;

//}

void MRSMapWrap::initialize(cv::Mat& src, cv::Mat& depth,
		std::vector<Segment*>& segments) {
//	Utils utils;
//	cv::Mat tmp_img,tmp_depth;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	utils.cropped_pcl_from_segments(src, depth,segments, cloud, tmp_img, tmp_depth);
//	std::string text("MRSMapWrap cloud");
//	utils.display_cloud(cloud,text);
//	initialize(cloud);
}

void MRSMapWrap::initialize(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudIn) {

	// change reference frame of point cloud to point mean and oriented along principal axes
//	Eigen::Vector4d mean;
//	Eigen::Vector3d eigenvalues;
//	Eigen::Matrix3d cov;
//	Eigen::Matrix3d eigenvectors;
//	pcl::computeMeanAndCovarianceMatrix(*pointCloudIn, cov, mean);
//	pcl::eigen33(cov, eigenvectors, eigenvalues);
//
//	if (Eigen::Vector3d(eigenvectors.col(0)).dot(Eigen::Vector3d::UnitZ())
//			> 0.0)
//		eigenvectors.col(0) = (-eigenvectors.col(0)).eval();
//
//	// transform from object reference frame to camera
//	Eigen::Matrix4d objectTransform = Eigen::Matrix4d::Identity();
//	objectTransform.block<3, 1>(0, 0) = eigenvectors.col(2);
//	objectTransform.block<3, 1>(0, 1) = eigenvectors.col(1);
//	objectTransform.block<3, 1>(0, 2) = eigenvectors.col(0);
//	objectTransform.block<3, 1>(0, 3) = mean.block<3, 1>(0, 0);
//
//	if (objectTransform.block<3, 3>(0, 0).determinant() < 0) {
//		objectTransform.block<3, 1>(0, 0) = -objectTransform.block<3, 1>(0, 0);
//	}
//
//	Eigen::Matrix4d objectTransformInv = objectTransform.inverse();
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectPointCloud = pcl::PointCloud<
//			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
//	pcl::transformPointCloud(*pointCloudIn, *objectPointCloud,
//			(objectTransformInv).cast<float>());
//
//	objectPointCloud->sensor_origin_ =
//			objectTransformInv.block<4, 1>(0, 3).cast<float>();
//	objectPointCloud->sensor_orientation_ = Eigen::Quaternionf(
//			objectTransformInv.block<3, 3>(0, 0).cast<float>());
//
//	treeNodeAllocator_->reset();
//	map_ = boost::shared_ptr<MultiResolutionSurfelMap>(
//			new MultiResolutionSurfelMap(max_resolution_, max_radius_,
//					treeNodeAllocator_));
//
//	map_->params_.dist_dependency = dist_dep_;
//
//	std::vector<int> pointIndices(objectPointCloud->points.size());
//	for (unsigned int i = 0; i < pointIndices.size(); i++)
//		pointIndices[i] = i;
//	map_->imageAllocator_ = imageAllocator_;
//	map_->addPoints(*objectPointCloud, pointIndices);
//	map_->octree_->root_->establishNeighbors();
//	map_->evaluateSurfels();
//	map_->buildShapeTextureFeatures();
//
//	map_->save(map_folder_ + "/" + object_name_ + ".map");
}
