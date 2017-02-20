/*
 * utils.cpp
 *
 *  Created on: 6 Nov 2015
 *      Author: martin
 */

#include "utils.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/median_filter.h>
//#include <pcl/filters/model_outlier_removal.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/filters/random_sample.h>
#include "pcl_hack/uniform_sampling.h"
//#include <pcl/keypoints/uniform_sampling.h>
//PCL 1.8
//#include <pcl/filters/uniform_sampling.h>
//#include <pcl/filters/random_sample.h>
//#include <pcl/filters/moment_of_inertia_estimation.h>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/icp.h>

std::string Utils::stringify(double x) {
	std::ostringstream o;
	if ((o << x))

		return o.str();
	else
		return NULL;
}

std::string Utils::stringify(int x) {
	std::ostringstream o;
	if ((o << x))

		return o.str();
	else
		return NULL;
}

std::string Utils::remove_extension(const std::string& filename) {
	size_t lastdot = filename.find_last_of(".");
	if (lastdot == std::string::npos)
		return filename;
	return filename.substr(0, lastdot);
}

string Utils::get_file_name(const string& s) {

	char sep = '/';

	size_t i = s.rfind(sep, s.length());
	if (i != string::npos) {
		return (s.substr(i + 1, s.length() - i));
	}

	return ("");
}

int Utils::parse_args(int argc, char **argv, double& thres, int& scales,
		int& starting_scale, int& propagation_scale, int& gpu, string& img_path,
		string& output_path) {

	/** Define and parse the program options
	 */
	namespace po = boost::program_options;
	po::options_description desc("Options");

	string imagePath = "", depthPath = "", segmentsPath = "", outputPath = "",
			iorPath = "", tdPath = "", cfg_path = "";
	string salMode = "", segMode = "";
	desc.add_options()("help", "Print help messages")

	("scales,s", po::value<int>(&scales), "Number of scales")("gpu,g",
			po::value<int>(&gpu), "Enable or disable GPU")("starting_scale,r",
			po::value<int>(&starting_scale), "Starting scale")(
			"propagation_scale,p", po::value<int>(&propagation_scale),
			"Scale used for propagating the labels")

	("threshold,t", po::value<double>(&thres), "Gradient threshold value")

	("image,i", po::value<string>(&img_path), "Path to the input image")

	("output,o", po::value<string>(&output_path)->required(),
			"path to the output files");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Basic Command Line Parameter App" << std::endl << desc
					<< std::endl;
			return SUCCESS;
		}

		po::notify(vm);  // throws on error, so do after help in case
						 // there are any problems
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << desc << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	return 0;
}

int Utils::parse_args(int argc, char **argv, double& thres, int& scales,
		int& starting_scale, int& propagation_scale, int& gpu, string& img_path,
		string& output_path, string& svm_path) {

	/** Define and parse the program options
	 */
	namespace po = boost::program_options;
	po::options_description desc("Options");

	string imagePath = "", depthPath = "", segmentsPath = "", outputPath = "",
			iorPath = "", tdPath = "", cfg_path = "";
	string salMode = "", segMode = "";
	desc.add_options()("help", "Print help messages")

	("scales,s", po::value<int>(&scales), "Number of scales")("gpu,g",
			po::value<int>(&gpu), "Enable or disable GPU")("starting_scale,r",
			po::value<int>(&starting_scale), "Starting scale")(
			"propagation_scale,p", po::value<int>(&propagation_scale),
			"Scale used for propagating the labels")

	("threshold,t", po::value<double>(&thres), "Gradient threshold value")

	("image,i", po::value<string>(&img_path), "Path to the input image")

	("svm,v", po::value<string>(&svm_path), "Path to the SVM model")

	("output,o", po::value<string>(&output_path)->required(),
			"path to the output files");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Basic Command Line Parameter App" << std::endl << desc
					<< std::endl;
			return SUCCESS;
		}

		po::notify(vm);  // throws on error, so do after help in case
						 // there are any problems
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << desc << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	return 0;
}

int Utils::parse_args(int argc, char **argv, double& thres, int& scales,
		int& starting_scale, int& propagation_scale, int& gpu, string& img_path,
		string& output_path, string& svm_path, string& depth_path) {

	/** Define and parse the program options
	 */
	namespace po = boost::program_options;
	po::options_description desc("Options");

	string imagePath = "", depthPath = "", segmentsPath = "", outputPath = "",
			iorPath = "", tdPath = "", cfg_path = "";
	string salMode = "", segMode = "";
	desc.add_options()("help", "Print help messages")

	("scales,s", po::value<int>(&scales), "Number of scales")("gpu,g",
			po::value<int>(&gpu), "Enable or disable GPU")("starting_scale,r",
			po::value<int>(&starting_scale), "Starting scale")(
			"propagation_scale,p", po::value<int>(&propagation_scale),
			"Scale used for propagating the labels")

	("threshold,t", po::value<double>(&thres), "Gradient threshold value")

	("image,i", po::value<string>(&img_path), "Path to the input image")

	("clouds,c", po::value<string>(&depth_path), "Path to the input pcls")

	("svm,v", po::value<string>(&svm_path), "Path to the SVM model")

	("output,o", po::value<string>(&output_path)->required(),
			"path to the output files");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Basic Command Line Parameter App" << std::endl << desc
					<< std::endl;
			return SUCCESS;
		}

		po::notify(vm);  // throws on error, so do after help in case
						 // there are any problems
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << desc << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	return 0;
}

void Utils::merge_two_bounding_rects(Rect& rec1, Rect& rec2, Rect& res) {

	int min_x = rec1.x < rec2.x ? rec1.x : rec2.x;
	int min_y = rec1.y < rec2.y ? rec1.y : rec2.y;

	int r1_max_x = rec1.x + rec1.width;
	int r2_max_x = rec2.x + rec2.width;

	int r1_max_y = rec1.y + rec1.height;
	int r2_max_y = rec2.y + rec2.height;

	int width = r1_max_x > r2_max_x ? r1_max_x - min_x : r2_max_x - min_x;
	int height = r1_max_y > r2_max_y ? r1_max_y - min_y : r2_max_y - min_y;

	res.x = rec1.x < rec2.x ? rec1.x : rec2.x;
	res.y = rec1.y > rec2.y ? rec1.y : rec2.y;

	res.width = width;
	res.height = height;

}

/*
 * !brief Builds a mesh for the input point cloud
 *
 * @src_cloud [input]
 * @ mesh1 [output]
 *
 */
void Utils::build_mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PolygonMesh::Ptr& mesh1) {

	mesh1 = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);

	// reconstruct meshes for source and target
	pcl::OrganizedFastMesh<pcl::PointXYZRGB> fast_mesh;
	fast_mesh.setInputCloud(src_cloud);
	fast_mesh.reconstruct(*mesh1);

}

void Utils::display_mesh(pcl::PolygonMesh::Ptr& mesh) {
	pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPolygonMesh(*mesh, "meshes", 0);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

/*
 * !brief Displays the point cloud and surface normals and shows the specified text
 * as the title of the visualizer
 *
 *
 */
void Utils::display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointCloud<pcl::Normal>::Ptr normals, string& text) {
	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

/*
 * !brief Displays the point cloud and shows the specified text
 * as the title of the visualizer
 *
 *
 */
void Utils::display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		string& text) {
	pcl::visualization::PCLVisualizer viewer(text);
	//viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud < pcl::PointXYZRGB > (cloud, text);
	viewer.addCoordinateSystem(0.5);
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, text);
	viewer.spin();
}

void Utils::display_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
		string& text) {
	pcl::visualization::PCLVisualizer viewer(text);
	//viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud < pcl::PointXYZ> (cloud, text);
	viewer.addCoordinateSystem(0.5);
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, text);
	viewer.spin();
}

/*
 * !brief Displays the points of the cloud specified by the indices vector
 *
 *
 */
void Utils::display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		std::vector<int>& indices, string& text) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_copy(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int index : indices)
		cloud_copy->points.push_back(cloud->points[index]);

	pcl::visualization::PCLVisualizer viewer(text);
	//viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud < pcl::PointXYZRGB > (cloud_copy, text);
	viewer.addCoordinateSystem(1.0);
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, text);
	viewer.spin();
}

void Utils::display_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		Eigen::Matrix3f& eigenVectors, string& text) {
	pcl::visualization::PCLVisualizer viewer(text);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud < pcl::PointXYZRGB > (cloud, text);
	viewer.addCoordinateSystem(1.0);
	//add principal components

	const PointXYZ origin(0., 0., 0.);
	const PointXYZ first_eigen(eigenVectors.col(0)(0), eigenVectors.col(0)(1),
			eigenVectors.col(0)(2));
	const PointXYZ second_eigen(eigenVectors.col(1)(0), eigenVectors.col(1)(1),
			eigenVectors.col(1)(2));
	const PointXYZ third_eigen(eigenVectors.col(2)(0), eigenVectors.col(2)(1),
			eigenVectors.col(2)(2));
	cout << " firts eigen vector=" << first_eigen << endl;
	viewer.addArrow(first_eigen, origin, 0., 0., 1., false, "first");
	viewer.addArrow(second_eigen, origin, 0., 1., 0., false, "second");
	viewer.addArrow(third_eigen, origin, 1., 0., 0., false, "third");
	viewer.spin();
}

void Utils::cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<
			pcl::search::Search<pcl::PointXYZRGB> >(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size()
			<< std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size()
			<< " points." << endl;
	std::cout << "These are the indices of the points of the initial"
			<< std::endl << "cloud that belong to the first cluster:"
			<< std::endl;
	unsigned int counter = 0;
	while (counter < clusters[0].indices.size()) {
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud =
			reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped()) {
	}

	return;
}

void Utils::cropped_pcl_from_mask(Mat& img, Mat& depth, Mat& mask,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Mat& tmp_img,
		Mat& tmp_depth) {

	Mat cropped_img, cropped_depth;
	Mat pointsMat;
	tmp_img = Mat::zeros(img.rows, img.cols, CV_8UC3);
	tmp_depth = Mat::zeros(img.rows, img.cols, CV_32FC1);
	cv::findNonZero(mask, pointsMat);
	Rect minRect = boundingRect(pointsMat);
//	imshow("mask", mask);
//	imshow("mask cropped", mask(minRect));
//	waitKey(0);
	//cout <<"mask.size()="<<mask.size()<<" img2.size()="<<img2.size()<<endl;
	//cvtColor(mask,mask,CV_BGR2GRAY);

	img.copyTo(tmp_img, mask);
	depth.copyTo(tmp_depth, mask);

	Utils utils;
	cropped_img = tmp_img(minRect);
	cropped_depth = tmp_depth(minRect);
	//imshow("mask", mask);
	//imshow("cropped_img", cropped_img);
	//waitKey(0);
	int cx = img.cols / 2;
	int cy = img.rows / 2;
	utils.image_to_pcl(cropped_img, cropped_depth, cloud, cx, cy, minRect);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pruned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	utils.prune_pcl(cloud, pruned_cloud);
//	cout <<"original cloud ->points.size()="<<cloud->points.size()<<endl;
//	cout <<"pruned_cloud->points.size()="<<pruned_cloud->points.size()<<endl;
//	pcl::visualization::PCLVisualizer viewer("3d Viewer");
//	viewer.setBackgroundColor(0, 0, 0);
//	viewer.addPointCloud < pcl::PointXYZRGB > (pruned_cloud, "sample cloud");
//	viewer.spin();

}

/*
 * !brief Computes the point cloud that corresponds only to the input vector of segments
 *
 * @img [input] image of the full scene
 * @depth [input] depth map of the full scene
 * @segments [input] vector of the object's segments
 * @cloud [output] the point cloud corresponding to the segments
 * @tmp_img [output] the cropped image corresponding to the object
 * @tmp_depth [output] the cropped depth map corresponding to the object
 *
 *
 */
void Utils::cropped_pcl_from_segments(Mat& img, Mat& depth,
		vector<Segment*>&segments,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Mat& tmp_img,
		Mat& tmp_depth) {

	Size size_segments = segments[0]->getBinaryMat().size();
	cout << " target size=" << size_segments << endl;
	cout << " current size=" << img.size() << endl;

	Mat mask = Mat::zeros(size_segments, CV_8UC1);
	//iterate over the segments and fill in the binary mask
	for (Segment * seg : segments) {

		mask += seg->getBinaryMat();

	}
	resize(mask, mask, img.size());
	cropped_pcl_from_mask(img, depth, mask, cloud, tmp_img, tmp_depth);

}

void Utils::compute_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointCloud<pcl::Normal>::Ptr& normals) {

	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<
			pcl::search::Search<pcl::PointXYZRGB> >(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	normals.reset(new pcl::PointCloud<pcl::Normal>);
//
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setRadiusSearch(0.03);
	//normal_estimator.setKSearch(20);
	normal_estimator.compute(*normals);

//	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//	ne.setMaxDepthChangeFactor(0.02f);
//	ne.setNormalSmoothingSize(10.0f);
//	ne.setInputCloud(cloud);
//	ne.compute(*normals);
}

/*
 * !brief Computes the surface normals of an organized point cloud
 *
 * @cloud [input]: needs to be organized!!
 * @normals [output]
 *
 */
void Utils::compute_integral_normals(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointCloud<pcl::Normal>::Ptr& normals) {

	normals.reset(new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
}

void Utils::compute_vfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointCloud<pcl::Normal>::Ptr& normals,pcl::PointCloud<pcl::VFHSignature308>::Ptr& vfhFeatures){


	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Setup the feature computation
	typedef pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal,
			pcl::VFHSignature308> VFHEstimationType;
	VFHEstimationType vfhEstimation;

	// Provide the original point cloud (without normals)
	vfhEstimation.setInputCloud(cloud);

	// Provide the point cloud with normals
	vfhEstimation.setInputNormals(normals);

	// Use the same KdTree from the normal estimation
	vfhEstimation.setSearchMethod(tree);

	vfhEstimation.setRadiusSearch (0.2);
	vfhEstimation.setNormalizeBins(true);
	// Actually compute the VFH features
	vfhEstimation.compute(*vfhFeatures);
}

void Utils::compute_vfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointCloud<pcl::Normal>::Ptr& normals, Mat& vfh_features) {

	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
	compute_vfh(cloud,normals,vfhFeatures);

	//std::cout << "output points.size (): " << vfhFeatures->points.size()
	//		<< std::endl; // This outputs 1 - should be 397!

	// Display and retrieve the shape context descriptor vector for the 0th point.
	pcl::VFHSignature308 descriptor = vfhFeatures->points[0];

//	VFHEstimationType::PointCloudOut::PointType descriptor2 =
//			vfhFeatures->points[0];
	vfh_features.create(1, 308, CV_32FC1);
	for (int i = 0; i <308; i++) {
		vfh_features.at<float>(0, i) = descriptor.histogram[i];
	}

}

void Utils::compute_cvfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointCloud<pcl::VFHSignature308>::Ptr& vfhFeatures) {

//	pcl::PointCloud<PointXYZRGB>::Ptr ids_cloud (new pcl::PointXYZRGB<PointType>);
//
//	  std::vector<int> pointIds;
//
//	// Create 121 points, and use all of them
//	for (unsigned int i = 0; i < 121; ++i) {
//		PointXYZRGB p;
//		p.x = drand48();
//		p.y = drand48();
//		p.z = drand48();
//		cloud->push_back(p);
//		pointIds.push_back(i);
//	}
//
//	// Compute the normals
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
//	normalEstimation.setInputCloud(cloud);
//
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
//			new pcl::search::KdTree<pcl::PointXYZRGB>);
//	normalEstimation.setSearchMethod(tree);
//
//	pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals(
//			new pcl::PointCloud<pcl::Normal>);
//
//	normalEstimation.setRadiusSearch(0.03);
//
//	normalEstimation.setIndices(
//			boost::make_shared<std::vector<int> >(pointIds));
//	normalEstimation.compute(*cloudWithNormals);
//
//	std::cout << "Computed " << cloudWithNormals->points.size() << " normals."
//			<< std::endl;
//
//	// Setup the feature computation
//	pcl::CVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> cvfhEstimation;
//
//	// Provide the original point cloud (without normals)
//	cvfhEstimation.setInputCloud(cloud);
//
//	// Only use the selected indices
//	cvfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));
//
//	// Provide the point cloud with normals
//	cvfhEstimation.setInputNormals(cloudWithNormals);
//
//	// Use the same KdTree from the normal estimation
//	cvfhEstimation.setSearchMethod(tree);
//
//	//vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"
//
//	// Actually compute the VFH features
//	vfhFeatures.reset(new pcl::PointCloud<pcl::VFHSignature308>);
//	cvfhEstimation.compute(*vfhFeatures);
}

/*
 * !brief Computes the FPFH features for a given point cloud
 *
 * @cloud [input]
 * @fpfhs [output]
 *
 */
void Utils::compute_fpfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs) {
	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	compute_normals(cloud, normals);
	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<PointXYZRGB>);

	fpfh.setSearchMethod(tree);

	// Output datasets
	fpfhs.reset(new pcl::PointCloud<pcl::FPFHSignature33>());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch(0.10);

	// Compute the features
	fpfh.compute(*fpfhs);

}

void Utils::compute_inertia(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		Point3d& dimensions_3d) {

//	pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
//	feature_extractor.setInputCloud(cloud);
//	feature_extractor.compute();
//
//	std::vector<float> moment_of_inertia;
//	std::vector<float> eccentricity;
//	pcl::PointXYZRGB min_point_AABB;
//	pcl::PointXYZRGB max_point_AABB;
//	pcl::PointXYZRGB min_point_OBB;
//	pcl::PointXYZRGB max_point_OBB;
//	pcl::PointXYZRGB position_OBB;
//	Eigen::Matrix3f rotational_matrix_OBB;
//	float major_value, middle_value, minor_value;
//	Eigen::Vector3f major_vector, middle_vector, minor_vector;
//	Eigen::Vector3f mass_center;
//
//	feature_extractor.getMomentOfInertia(moment_of_inertia);
//	feature_extractor.getEccentricity(eccentricity);
//	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
//	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
//			rotational_matrix_OBB);
//	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
//	feature_extractor.getEigenVectors(major_vector, middle_vector,
//			minor_vector);
//	feature_extractor.getMassCenter(mass_center);
//
////	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
////			new pcl::visualization::PCLVisualizer("3D Viewer"));
////	viewer->setBackgroundColor(0, 0, 0);
////	viewer->addCoordinateSystem(1.0);
////	viewer->initCameraParameters();
////	viewer->addPointCloud < pcl::PointXYZRGB > (cloud, "sample cloud");
////	viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y,
////			max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0,
////			"AABB");
//
//	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
//	Eigen::Quaternionf quat(rotational_matrix_OBB);
//	dimensions_3d.x = max_point_OBB.x - min_point_OBB.x;
//	dimensions_3d.y = max_point_OBB.y - min_point_OBB.y;
//	dimensions_3d.z = max_point_OBB.z - min_point_OBB.z;
//	cout << "compute_inertia: |x|=" << max_point_OBB.x - min_point_OBB.x
//			<< " |y|=" << max_point_OBB.y - min_point_OBB.y << " |z|="
//			<< max_point_OBB.z - min_point_OBB.z << endl;

}

void Utils::find_detection_yaw(Mat& mask, Mat& img, Mat& depth,
		Point3d& position, Point3d& orientation) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	//compute the point cloud of this detection
	Mat masked_img, masked_depth;

	img.copyTo(masked_img, mask);
	depth.copyTo(masked_depth, mask);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	image_to_pcl(masked_img, masked_depth, cloud);

	//remove!
	//Point3d point;
	//compute_bounding_box(cloud,point);

	//find the center of the object
	double max_z = 0.;
	xyz_gravity_center(cloud, position, max_z);
	//position.z = max_z;

	//find its yaw
	find_pcl_yaw(cloud, orientation);
	//cout <<">Utils::find_detection_yaw roll,pitch,yaw="<<orientation<<endl;
	//string text("detection cloud");
	//display_cloud(cloud,text);

}

/*
 * rotates the point cloud around its eigen vectors
 *
 */
void Utils::rotate_eigen_axes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) {

	Eigen::Matrix3f eigenVectorsPCA;
	compute_pca_alt(src_cloud, eigenVectorsPCA);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*src_cloud, pcaCentroid);

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f
			* (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
	//TODO: change the code above with a call to obtain_eigen_axes
	dst_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*src_cloud, *dst_cloud, projectionTransform);
}

/*
 * !brief Calculates the transformation that would rotate
 * the point cloud around its eigen vectors.
 *
 * @src_cloud [input]
 * @projectionTransform [output]
 *
 *
 */
void Utils::obtain_eigen_axes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		Eigen::Matrix4f& projectionTransform) {

	Eigen::Matrix3f eigenVectorsPCA;
	compute_pca_alt(src_cloud, eigenVectorsPCA);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*src_cloud, pcaCentroid);

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	projectionTransform = Eigen::Matrix4f::Identity();
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f
			* (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());

}

/*
 * !brief Computes the minimum bounding box around the point cloud
 * and stores it in dimensions_3d
 *
 * @src_cloud [input]
 * @dimensions_3d [output]
 *
 */
void Utils::compute_bounding_box(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		Point3d& dimensions_3d) {

	//copy the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*src_cloud, *copy_cloud);
	Eigen::Matrix3f eigenVectorsPCA;
	compute_pca_alt(copy_cloud, eigenVectorsPCA);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*copy_cloud, pcaCentroid);

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3, 1>(0, 3) = -1.f
			* (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*copy_cloud, *cloudPointsProjected,
			projectionTransform);
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZRGB minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
//	const Eigen::Vector3f meanDiagonal = 0.5f
//			* (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	// Final transform
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	//const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal
	//		+ pcaCentroid.head<3>();

	//lets rotate the point cloud
	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>());
	// You can either apply transform_1 or transform_2; they are the same
	//pcl::transformPointCloud(*src_cloud, *transformed_cloud, bboxQuaternion);
	//string text("transformed");
	//Eigen::Matrix3f transformedEigen;
	//compute_pca_alt(transformed_cloud, transformedEigen);
	//display_cloud(transformed_cloud, text);

	double x = maxPoint.x - minPoint.x;
	double y = maxPoint.y - minPoint.y;
	double z = maxPoint.z - minPoint.z;
	vector<double> values( { x, y, z });
	sort(values.begin(), values.end());
	dimensions_3d.z = values[0];
	dimensions_3d.y = values[1];
	dimensions_3d.x = values[2];

	return;

	// This viewer has 4 windows, but is only showing images in one of them as written here.
//	pcl::visualization::PCLVisualizer *visu;
//	visu = new pcl::visualization::PCLVisualizer("PlyViewer");
//	int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
//	visu->createViewPort(0.0, 0.5, 0.5, 1.0, mesh_vp_1);
//	visu->createViewPort(0.5, 0.5, 1.0, 1.0, mesh_vp_2);
//	visu->createViewPort(0.0, 0, 0.5, 0.5, mesh_vp_3);
//	visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
//	visu->addPointCloud(src_cloud); //, ColorHandlerXYZRGB(src_cloud, 30, 144, 255), "bboxedCloud", mesh_vp_3);
//	visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x,
//			maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox",
//			mesh_vp_3);
//	visu->spin();
//	return;
}

/*
 * removes all the points that have (0,0,0) coordinates.
 * Beware, if using integral images to compute normals based
 * on the result of this function, because the point cloud
 * will not be structured anymore => cloud->height = 1
 */
void Utils::prune_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_dst) {
	int npoints = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tpm_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	for (pcl::PointXYZRGB& pt : *cloud_src) {
		if ((pt.x != 0 || pt.y != 0 || pt.z != 0)) {
			tpm_cloud->points.push_back(pt);
			npoints++;
		}
	}
	tpm_cloud->width = npoints;
	tpm_cloud->height = 1;
	cloud_dst = tpm_cloud;

}

/*
 * removes all the points that have (0,0,0) coordinates.
 * Beware, if using integral images to compute normals based
 * on the result of this function, because the point cloud
 * will not be structured anymore => cloud->height = 1
 */
void Utils::nanify_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_dst) {
	int npoints = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (pcl::PointXYZRGB& pt : *cloud_src) {
//		if (fabs(pt.x) > 0.00001 || fabs(pt.y) > 0.00001 || fabs(pt.z) > 0.00001) {
//			//cout <<" non nanifyied point p="<<pt<<endl;
//			npoints++;
//		} else {
//
//			pt.x = NAN;
//			pt.y = NAN;
//			pt.z = NAN;
//			//if(!isnan(pt.x))
//			//	cout <<"problem with nanyfying!"<<endl;
//		}
		if (pt.x == 0. && pt.y == 0. && pt.z == 0.) {
			pt.x = NAN;
			pt.y = -1.;	//NAN;
			pt.z = -1.;	//NAN;
			pt.r = 255;
		}
		tmp_cloud->points.push_back(pt);

	}
	cout << (int) (cloud_src->points.size()) - npoints << " points nanified"
			<< endl;
	cloud_dst = tmp_cloud;

}

void Utils::remove_zeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,
		std::vector<int>& indices) {

	indices.clear();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	int index = 0;
	for (pcl::PointXYZRGB& pt : *cloud_src) {

		if (isnan(pt.x) || (pt.x == 0. && pt.y == 0. && pt.z == 0.)) {

		} else
			indices.push_back(index);

		index++;
	}

}

void Utils::subtract_gravity_center(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,
		Point3d& gravity_center) {
	for (PointXYZRGB& p : *cloud_src) {
		p.x -= gravity_center.x;
		p.y -= gravity_center.y;
		p.z -= gravity_center.z;
	}
}

void Utils::subtract_gravity_center(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,
		std::vector<int>& indices, Point3d& gravity_center) {
	for (int index : indices) {
		pcl::PointXYZRGB& p = cloud_src->points[index];
		p.x -= gravity_center.x;
		p.y -= gravity_center.y;
		p.z -= gravity_center.z;
	}
}

void Utils::compute_pca(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pca_cloud,
		Eigen::Matrix3f& eigenVectors) {
	pcl::PCA<pcl::PointXYZRGB> pca;
	pca.setInputCloud(pca_cloud);
	eigenVectors = pca.getEigenVectors();
}

void Utils::remove_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	//median filter: introduces new points where there were none

//	pcl::MedianFilter<pcl::PointXYZRGB> fbf;
//	fbf.setInputCloud (src_cloud);
//	fbf.setMaxAllowedMovement (10);
//	fbf.setWindowSize (5);
//	fbf.filter (*tmp_cloud);
//	dst_cloud=tmp_cloud;

	// Create the filtering object
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//	sor.setInputCloud(src_cloud);
//	sor.setMeanK(10);
//	sor.setStddevMulThresh(1.0);
//	sor.filter(*dst_cloud);

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	// build the filter
	outrem.setInputCloud(src_cloud);
	outrem.setRadiusSearch(0.1);
	outrem.setMinNeighborsInRadius(40);
	// apply filter
	outrem.filter(*tmp_cloud);
	dst_cloud = tmp_cloud;
}

void Utils::remove_SCREW_outliers(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	// build the filter
	outrem.setInputCloud(src_cloud);
	outrem.setRadiusSearch(0.05);
	outrem.setMinNeighborsInRadius(25);
	// apply filter
	outrem.filter(*tmp_cloud);
	dst_cloud = tmp_cloud;
}

void Utils::remove_HOLDER_outliers(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	// build the filter
	outrem.setInputCloud(src_cloud);
	outrem.setRadiusSearch(0.05);
	outrem.setMinNeighborsInRadius(18);
	//outrem.setMinNeighborsInRadius(25);
	// apply filter
	outrem.filter(*tmp_cloud);
	dst_cloud = tmp_cloud;
}

double Utils::icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,
		Eigen::Matrix4f& transform) {

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_1);
	icp.setInputTarget(cloud_2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*Final);
	double score = icp.getFitnessScore();
	std::cout << "has converged:" << icp.hasConverged() << " score: " << score
			<< std::endl;
	transform = icp.getFinalTransformation();
	//std::cout << icp.getFinalTransformation() << std::endl;

	//validate the transformation
//	pcl::registration::TransformationValidationEuclidean<pcl::PointXYZRGB, pcl::PointXYZRGB> tve;
//	tve.setMaxRange (0.5);  // 50cm
//	double score = tve.validateTransformation (cloud_1, cloud_2, transform);
//	cout << "TransformationValidationEuclidean score ="<<score <<endl;
//	score = tve.validateTransformation (cloud_2, cloud_1, transform.inverse());
//	cout << "TransformationValidationEuclidean score ="<<score <<endl;

	return score;

}

double Utils::validate_transform(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,
		Eigen::Matrix4f& transform) {
	//validate the transformation
	pcl::registration::TransformationValidationEuclidean<pcl::PointXYZRGB,
			pcl::PointXYZRGB> tve;
	tve.setMaxRange(0.5);  // 50cm
	double score = tve.validateTransformation(cloud_1, cloud_2, transform);
	cout << "TransformationValidationEuclidean score =" << score << endl;
	score = tve.validateTransformation(cloud_2, cloud_1, transform.inverse());
	cout << "TransformationValidationEuclidean score =" << score << endl;
	return score;
}

/*
 * angles.x angles.y angles.z = roll pitch yaw
 */
void Utils::euler_from_R(Eigen::Matrix3f R, Point3d& angles) {
	double r11 = R.col(0)(0);
	//double r12 = R.col(1)(0);
	//double r13 = R.col(2)(0);

	double r21 = R.col(0)(1);
	//double r22 = R.col(1)(1);
	//double r23 = R.col(2)(1);

	double r31 = R.col(0)(2);
	double r32 = R.col(1)(2);
	double r33 = R.col(2)(2);

	angles.x = atan2(r32, r33);
	angles.y = atan2(-r31, sqrt(r32 * r32 + r33 * r33));
	angles.z = atan2(r21, r11);

	if (angles.x < 0.)
		angles.x += M_PI;
	if (angles.y < 0.)
		angles.y += M_PI;
	if (angles.z < 0.)
		angles.z += M_PI;

}

void Utils::compute_pca_alt(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pca_cloud,
		Eigen::Matrix3f& eigenVectorsPCA) {

	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*pca_cloud, pcaCentroid);
	//cout << " pcaCentroid=" << pcaCentroid[0] << " " << pcaCentroid[1] << " "
	//		<< pcaCentroid[2] << " " << pcaCentroid[3] << endl;
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*pca_cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver < Eigen::Matrix3f
			> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	//Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA = eigen_solver.eigenvectors();

//	cout<<"Utils::compute_pca_alt   eigenvalues="<<eigen_solver.eigenvalues()<<endl;
//	cout<<"Utils::compute_pca_alt   eigenVectorsPCA="<<eigenVectorsPCA<<endl;

//	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(
//			eigenVectorsPCA.col(1)); /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
//									 ///    the signs are different and the box doesn't get correctly oriented in
}

void Utils::find_pcl_yaw(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
		Point3d& orientation) {
	//flatten the point cloud
	//for (PointXYZRGB& p : *pcl_cloud) {
	//	p.z = 0;
	//}
	Eigen::Matrix3f eigenVectorsPCA;
	compute_pca(pcl_cloud, eigenVectorsPCA);
	//cout<<"Utils::find_pcl_yaw   eigenVectorsPCA="<<endl<<eigenVectorsPCA<<endl;

	euler_from_R(eigenVectorsPCA, orientation);
	return;

//	double x =eigenVectorsPCA.col(0)(0);
//	double y =eigenVectorsPCA.col(0)(1);
//	double z =eigenVectorsPCA.col(0)(2);
//	cout <<" Utils::find_pcl_yaw first eigen vector="<<x<<" "<<y<<" "<<z<<" "<<endl;
//
//
//
//
//	//pcl::getEulerAngles(eigenVectorsPCA,x, y, z);
//	orientation.x = x;
//	orientation.y = y;
//	orientation.z = z;
//	return;
//
//	x =eigenVectorsPCA.col(0)(0);
//	y =eigenVectorsPCA.col(0)(1);
//	z =eigenVectorsPCA.col(0)(2);
//
//	return;
//	Eigen::Quaternionf q(eigenVectorsPCA.transpose());
//	cout <<" found quaternion from pca: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
//	double q0 = q.w();
//	double q1 = q.x();
//	double q2 = q.y();
//	double q3 = q.z();
//	//obtain roll pitch yaw from the quaternion
//	double roll = atan2(2*q0*q1+q2*q3, 1 - 2*(q1*q1+q2*q2));
//	double yaw = atan2(2*q0*q3+q1*q2, 1 - 2*(q2*q2+q3*q3));
//	double pitch = asin(2*(q0*q2-q3*q1));
//	cout <<" found roll,pitch,yaw from pca: "<<roll<<" "<<pitch<<" "<<yaw<<endl;
//
//
//	q0 =  0.831709;
//	q1 = -0.167978;
//	q2 = 0.386484;
//	q3 = 0.361487;
//		//obtain roll pitch yaw from the quaternion
//	roll = atan2(2*q0*q1+q2*q3, 1 - 2*(q1*q1+q2*q2));
//	pitch = asin(2*(q0*q2-q3*q1));
//	yaw = atan2(2*q0*q3+q1*q2, 1 - 2*(q2*q2+q3*q3));
//
//		cout <<" desirably found roll,pitch,yaw from pca: "<<roll<<" "<<pitch<<" "<<yaw<<endl;
//				//orientation = q;
//
//	//debug the possible quaternions
//	for(int i=0;i<3;i++){
//		x =eigenVectorsPCA.col(i)(0);
//		y =eigenVectorsPCA.col(i)(1);
//		z =eigenVectorsPCA.col(i)(2);
//		//roll
//		orientation.x = atan2(z,y);
//		//pitch
//		orientation.y = atan2(z,x);
//		//yaw
//		orientation.z=atan2 (y,x);
//		cout <<"i="<<i<<" roll,pitch,yaw="<<orientation.x<<" "<<orientation.y<<" "<<orientation.z<<endl;
//
//
//	}
//
//	//roll
//	orientation.x = 0.;
//	//yaw
//	orientation.y = 0.;
//	//yaw
//	orientation.z = 1.57;
	//roll
//	orientation.x = atan2(z,y);
//	//pitch
//	orientation.y = atan2(z,x);
//	//yaw
//	orientation.z=atan2 (y,x);
}

void Utils::xyz_gravity_center(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
		Point3d& gravity_center,
		double &max_z/*,Eigen::Matrix3f& eigenVectors*/) {

	double& x = gravity_center.x;
	double& y = gravity_center.y;
	double& z = gravity_center.z;
	double n = 0.;
	max_z = 0.;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pca_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	for (PointXYZRGB& p : *pcl_cloud) {
		if (fabs(p.x) > 0.00001 || fabs(p.y) > 0.00001) {
			n++;
			x += p.x;
			y += p.y;
			z += p.z;
			if (p.z > max_z)
				max_z = p.z;

		}

	}
	//cout << " x,y,z=" << x << " " << y << " " << z << endl;
	x /= n;
	y /= n;
	z /= n;

}

/*
 * !brief Shifts the point cloud
 *
 */
void Utils::shift_to_min(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud) {

	double x = 999999.;
	double y = 999999.;
	double z = 999999.;

	for (PointXYZRGB& p : *pcl_cloud) {
//		if (p.x < x && p.y < y && p.z < z ) {
//
//			x = p.x;
//			y = p.y;
//			z = p.z;
//		}
		if (!isnan(p.x)) {
			if (p.x < x)
				x = p.x;
			if (p.y < y)
				y = p.y;
			if (p.z < z)
				z = p.z;

		}

	}
	cout << " min at " << x << " " << y << " " << z << endl;

	for (PointXYZRGB& p : *pcl_cloud) {
		p.x -= x;
		p.y -= y;
		p.z -= z;
	}
}

/*
 * !brief Shifts the point cloud
 *
 */
void Utils::shift_to_min(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
		std::vector<int>& indices) {

	double x = 9999.;
	double y = 9999.;
	double z = 9999.;

	for (int index : indices) {

		PointXYZRGB& p = pcl_cloud->points[index];

		if (p.x < x)
			x = p.x;
		if (p.y < y)
			y = p.y;
		if (p.z < z)
			z = p.z;

	}
	cout << " min at " << x << " " << y << " " << z << endl;

	if (x > 0.)
		x = -x;
	if (y > 0.)
		y = -y;
	if (z > 0.)
		z = -z;
	for (int index : indices) {
		PointXYZRGB& p = pcl_cloud->points[index];
		p.x -= x;
		p.y -= y;
		p.z -= z;
	}
}

void Utils::mask_to_pcl_indices(Mat& mask, vector<int>& indices) {
	register int depth_idx = 0;
	//cv::imshow("mask_to_pcl_indices", mask);
	//cv::waitKey(0);
	for (int i = 0; i < mask.rows; i++) {
		for (int j = 0; j < mask.cols; j++, depth_idx++) {
			if (mask.at<uint8_t>(i, j) > 0) {
				indices.push_back(depth_idx);
			}

		}
	}
}

void Utils::compute_contours(cv::Mat& mask, vector<vector<Point> >& contours) {

	//vector<Vec4i> hierarchy;

	/// Find contours
	//findContours(mask, contours, hierarchy, cv::RETR_CCOMP,
	//		CV_CHAIN_APPROX_NONE, Point(0, 0));
}

void Utils::addPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) {
	for (PointXYZRGB& p : *src_cloud) {
		dst_cloud->push_back(p);
	}
}

void Utils::harris(cv::Mat& src_gray){
	/** @function cornerHarris_demo */
	int thresh = 200;
	int max_thresh = 255;

	Mat dst, dst_norm, dst_norm_scaled;
	dst = Mat::zeros( src_gray.size(), CV_32FC1 );

	/// Detector parameters
	int blockSize = 3;
	int apertureSize = 3;
	double k = 0.04;

	/// Detecting corners
	cv::cornerHarris( src_gray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );

	/// Normalizing
	cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, Mat() );
	cv::convertScaleAbs( dst_norm, dst_norm_scaled );

	/// Drawing a circle around corners
	for( int j = 0; j < dst_norm.rows ; j++ )
	 { for( int i = 0; i < dst_norm.cols; i++ )
		  {
			if( (int) dst_norm.at<float>(j,i) > thresh )
			  {
			   circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
			  }
		  }
	 }
	/// Showing the result

	cv::imshow( "harris", dst_norm_scaled );
	cv::waitKey(0);

}

bool isEqual(const cv::Vec4i& _l1, const cv::Vec4i& _l2)
{
    cv::Vec4i l1(_l1), l2(_l2);

    float length1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float length2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));

    float product = (l1[2] - l1[0])*(l2[2] - l2[0]) + (l1[3] - l1[1])*(l2[3] - l2[1]);

    if (fabs(product / (length1 * length2)) < cos(CV_PI / 15))
        return false;

    //if they overlap return true
    //end point of line1
    cv::Point endLine1(l1[2], l1[3]);
    //starting point of line1
    cv::Point startLine2(l2[0], l2[1]);
    //if their distance is small
    if( cv::norm(endLine1 - startLine2) < 10)
    	return true;
    return false;
}

void mergeLines(std::vector<cv::Vec4i>& lines,std::vector<int>& labels, std::vector<cv::Vec4i>& outLines, std::vector<int>& horiClasses){


	std::vector<cv::Vec4i> horizontals;

	for( size_t i = 0; i < lines.size(); i++ ){
		cv::Vec4i line = lines[i];
		cv::Point p0(line[0], line[1]);
		cv::Point pf(line[2], line[3]);
		double hypo = cv::norm(pf - p0);
		double side = pf.y - p0.y;
		if (side <0)
			side *= -1.f;
		double angle = asin(side/hypo);

		if(angle < 0.1 ){
			std::cout <<"angle is="<<angle<<std::endl;
			horizontals.push_back(line);
			horiClasses.push_back(labels[i]);
		}

	}
}

void lines2(cv::Mat& mask){

	RNG rng(12345);
	cv::Vec4f lines;
	cv::Mat dst,cdst;
	vector<vector<cv::Point>> contours;
	cv::Canny(mask, dst, 50, 200, 3);
	cv::cvtColor(dst, cdst, CV_GRAY2BGR);
	cv::imshow("the edges", dst);
	cv::waitKey(0);

	cv::findContours(dst,contours,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
	/// Find the convex hull object for each contour
	vector<vector<cv::Point> >hull( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	  {  cv::convexHull( cv::Mat(contours[i]), hull[i], true ); }


	int id = 0;
	for(cv::Point& p : hull[id]){
		cv::circle(mask,p,10,cv::Scalar(155));

	}

//	/// Draw contours + hull results
//	cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );
//	for( int i = 0; i< 1/*contours.size()*/; i++ )
//	  {
//		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//		cv::drawContours( drawing, contours, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
//		cv::drawContours( drawing, hull, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
//	  }

	/// Show in a window
	cv::namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
	cv::imshow( "Hull demo", mask );
	cv::waitKey(0);
}

void min_row_point(cv::Mat& mask, cv::Point& minRowPoint){
	int min_x = 999999;
	int min_y = 999999;
	int max_x = 0;
	int max_y = 0;

	for (int i = 0; i < mask.rows; i++) {
			for (int j = 0; j < mask.cols; j++) {
				if (mask.at<uint8_t>(i, j) > 0) {

					//min row
					if (i < min_y) {
						min_y = i;
						minRowPoint.x = j;
						minRowPoint.y = i;
					}
					//min col
					if (j < min_x) {
						min_x = j;

					}

					if (i > max_y)
						max_y = i;
					//max col
					if (j > max_x){
						max_x = j;

					}

				}
			}
		}
	//cv::circle(mask,minRowPoint,10,cv::Scalar(155));
	//cv::imshow("min row point", mask);
	//cv::waitKey(0);
}

void rotation(cv::Mat& mask, bool positiveRotation, cv::Mat& transform, cv::Mat& warp_rotate_dst){
	 cv::Point2f srcTri[3];
	 cv::Point2f dstTri[3];


	 cv::Mat rot_mat( 2, 3, CV_32FC1 );
	 cv::Mat warp_mat( 2, 3, CV_32FC1 );
	 cv::Mat src, warp_dst;

	/// Load the image
	src = mask.clone();

	/// Set the dst image the same type and size as src
	warp_dst = Mat::zeros( src.rows, src.cols, src.type() );

	/// Set your 3 points to calculate the  Affine Transform
	srcTri[0] = cv::Point2f( 0,0 );
	srcTri[1] = cv::Point2f( src.cols - 1, 0 );
	srcTri[2] = cv::Point2f( 0, src.rows - 1 );

	dstTri[0] = cv::Point2f( src.cols*0.0, src.rows*0.33 );
	dstTri[1] = cv::Point2f( src.cols*0.85, src.rows*0.25 );
	dstTri[2] = cv::Point2f( src.cols*0.15, src.rows*0.7 );

	/// Get the Affine Transform
	warp_mat = cv::getAffineTransform( srcTri, dstTri );


	/// Apply the Affine Transform just found to the src image
	cv::warpAffine( src, warp_dst, warp_mat, warp_dst.size() );
	warp_dst = src;
	/** Rotating the image after Warp */

	/// Compute a rotation matrix with respect to the center of the image
	cv::Point center = cv::Point( warp_dst.cols/2, warp_dst.rows/2 );
	double angle = positiveRotation ? 50.0 :  -50.0;
	double scale = 1.0;

	/// Get the rotation matrix with the specifications above
	rot_mat = cv::getRotationMatrix2D( center, angle, scale );
	transform = rot_mat;

	/// Rotate the warped image
	cv::warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );


}

void Utils::find_lines(cv::Mat& mask){
	cv::Mat dst,cdst;
	cv::Canny(mask, dst, 50, 200, 3);
	 cv::cvtColor(dst, cdst, CV_GRAY2BGR);



	std::vector<cv::Vec4i> lines,horizontals;
	cv::HoughLinesP(dst, lines, 1, CV_PI/90, 20, 20, 10 );

	std::vector<int> labels,horiClasses;
	int numberOfLines = cv::partition(lines, labels, isEqual);

	//mergeLines(lines,labels,horizontals, horiClasses);

//	for( size_t i = 0; i < lines.size(); i++ )
//	{
//		std::cout <<"class="<<labels[i]<<std::endl;
//		cv::Vec4i l = lines[i];
//		line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),cv::Scalar(0,0,255), 3, CV_AA);
//		cv::imshow("detected lines", cdst);
//		cv::waitKey(0);
//	}
	for( size_t i = 0; i < horizontals.size(); i++ )
		{
			std::cout <<"class="<<horiClasses[i]<<std::endl;
			cv::Vec4i l = horizontals[i];
			line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),cv::Scalar(0,0,255), 3, CV_AA);
			cv::imshow("detected lines", cdst);
			cv::waitKey(0);
		}

	 cv::imshow("source", mask);
	 cv::imshow("detected lines", cdst);



}

void Utils::find_mid_point_mask(cv::Mat& mask, Point2i& mid_point) {
	bool positiveRot = true, negativeRot = false;
	cv::Mat rotatedMat, positiveRotMat, negativeRotMat;
	cv::Point2i minRowP1, minRowP2;
	cv::Mat maskclone1 = mask.clone(), maskclone2 = mask.clone();
	//rotate the mask
	rotation(maskclone1, positiveRot, positiveRotMat, rotatedMat);
	min_row_point(rotatedMat, minRowP1);

	//rotate the mask
	rotation(maskclone2, negativeRot, negativeRotMat, rotatedMat);
	min_row_point(rotatedMat, minRowP2);



	//find_lines(maskclone);
	//harris(maskclone);

	/*
	 * ----------> (X,j) cols
	 *
	 *
	 * | (Y,i) rows
	 *
	 *
	 */


	int min_x = 999999;
	int min_y = 999999;
	int max_x = 0;
	int max_y = 0;

	cv::Point2i minColPoint,maxColPoint;
	cv::Point2i minRowPoint,maxRowPoint;


	//undo the rotation for point
	minRowPoint.x = negativeRotMat.at<double>(0,0)*minRowP1.x +negativeRotMat.at<double>(0,1)*minRowP1.y + negativeRotMat.at<double>(0,2);
	minRowPoint.y = negativeRotMat.at<double>(1,0)*minRowP1.x + negativeRotMat.at<double>(1,1)*minRowP1.y + negativeRotMat.at<double>(1,2);

	minColPoint.x = positiveRotMat.at<double>(0,0)*minRowP2.x +positiveRotMat.at<double>(0,1)*minRowP2.y + positiveRotMat.at<double>(0,2);
	minColPoint.y = positiveRotMat.at<double>(1,0)*minRowP2.x + positiveRotMat.at<double>(1,1)*minRowP2.y + positiveRotMat.at<double>(1,2);


/*
	for (int i = 0; i < mask.rows; i++) {
		for (int j = 0; j < mask.cols; j++) {
			if (mask.at<uint8_t>(i, j) > 0) {

				//min row
				if (i <= min_y) {
					min_y = i;
					minRowPoint.x = j;
					minRowPoint.y = i;
				}
				//min col
				if (j < min_x) {
					min_x = j;
					minColPoint.x = j;
					minColPoint.y = i;
				}

				if (i > max_y)
					max_y = i;
				//max col
				if (j > max_x){
					max_x = j;
					maxColPoint.x = j;
					maxColPoint.y = i;
				}

			}
		}
	}*/
	cv::circle(mask,minRowPoint,20,cv::Scalar(155));
	cv::circle(mask,minColPoint,20,cv::Scalar(155));
	cv::imshow("find_mid_point_mask", mask);

	//if the rectangle is not parallel to the camera axes this is wrong
	//mid_point.x = (max_x + min_x)/2.;
	//mid_point.y = (max_y + min_y)/2.;
	mid_point.x = (minRowPoint.x + minColPoint.x)/2.;
	mid_point.y = (minRowPoint.y + minColPoint.y)/2.;

	double side = cv::norm(minRowPoint-minColPoint);
	std::cout <<"minColPoint="<<minColPoint<<" minRowPoint="<<minRowPoint<<std::endl;
	double angle = asin( (minColPoint.y -minRowPoint.y  )/side  );
	double y_offset = side/2. * cos(angle);
	double x_offset = side/2. * sin(angle);
	std::cout <<"angle="<<angle<<std::endl;
	std::cout <<"side="<<side<<" x_offset="<< x_offset<<" y_offset="<< y_offset<< std::endl;
	mid_point.x += x_offset;
	mid_point.y += y_offset;
	std::cout <<"mid_point="<<mid_point<<std::endl;
}

void Utils::fill_mask(cv::Mat& mask) {
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	/// Find contours
	cv::findContours(mask, contours, hierarchy, cv::RETR_CCOMP,
			CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	cv::Scalar color = Scalar(255);
	//drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());

	int thickness = -1;
	cv::drawContours(mask, contours, -1, color, thickness);
}

void Utils::sub_sample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::UniformSampling<pcl::PointXYZRGB> uni_sampling;
	uni_sampling.setInputCloud(src_cloud);
	//uni_sampling.setRadiusSearch(0.02f);
	uni_sampling.setRadiusSearch(0.02f);
	//PCL 1.8
	uni_sampling.filter(*tmp_cloud);
	//uni_sampling.detectKeypoints(*tmp_cloud);
	dst_cloud = tmp_cloud;

}

void Utils::sub_sample_screw(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst_cloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::UniformSampling<pcl::PointXYZRGB> uni_sampling;
	uni_sampling.setInputCloud(src_cloud);
	//uni_sampling.setRadiusSearch(0.02f);
	uni_sampling.setRadiusSearch(0.005f);
	//PCL 1.8
	uni_sampling.filter(*tmp_cloud);
	//uni_sampling.detectKeypoints(*tmp_cloud);
	dst_cloud = tmp_cloud;

}

void Utils::image_to_pcl(Mat& img, Mat& depth,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud) {

	// explicitly specify dsize=dst.size(); fx and fy will be computed from that.
	if (img.size() != depth.size())
		resize(depth, depth, img.size(), 0, 0, cv::INTER_CUBIC);

	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = img.cols;
	pcl_cloud->height = img.rows;
	pcl_cloud->points.resize(pcl_cloud->height * pcl_cloud->width);

	//int* depth_data = new int[pcl_cloud->height * pcl_cloud->width];
	//copy the depth values of every pixel in here

	// register float constant = 1.0f / 525;
	float constant = 1.0f / 1368.3;
	//  register int centerX = (pcl_cloud->width >> 1);
	//  int centerY = (pcl_cloud->height >> 1);

	int cx = img.cols / 2;
	int cy = img.rows / 2;
	register int depth_idx = 0;
	for (int u = 0; u < img.rows; ++u)
		for (int v = 0; v < img.cols; ++v, ++depth_idx) {
			pcl::PointXYZRGB& pt = pcl_cloud->points[depth_idx];
			float depthvalue = depth.at<float>(u, v);
			//cout <<"depth value="<<depthvalue<<endl;
			pt.z = depthvalue;	//* 1000.0f;//depth_data[depth_idx] * 0.001f;
			pt.x = static_cast<float>(v - cx) * pt.z * constant;
			pt.y = static_cast<float>(u - cy) * pt.z * constant;
			Vec3b& colour = img.at<Vec3b>(u, v);
			pt.b = colour[0];
			pt.g = colour[1];
			pt.r = colour[2];
		}
	//cout <<"image_to_pcl::pcl_cloud->points.size()="<<pcl_cloud->points.size()<<endl;

}

void Utils::image_to_pcl(Mat& img, Mat& depth,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,float constant) {

	// explicitly specify dsize=dst.size(); fx and fy will be computed from that.
	if (img.size() != depth.size())
		resize(depth, depth, img.size(), 0, 0, cv::INTER_CUBIC);

	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = img.cols;
	pcl_cloud->height = img.rows;
	pcl_cloud->points.resize(pcl_cloud->height * pcl_cloud->width);

	//int* depth_data = new int[pcl_cloud->height * pcl_cloud->width];
	//copy the depth values of every pixel in here

	// register float constant = 1.0f / 525;

	//  register int centerX = (pcl_cloud->width >> 1);
	//  int centerY = (pcl_cloud->height >> 1);

	int cx = img.cols / 2;
	int cy = img.rows / 2;
	register int depth_idx = 0;
	for (int u = 0; u < img.rows; ++u)
		for (int v = 0; v < img.cols; ++v, ++depth_idx) {
			pcl::PointXYZRGB& pt = pcl_cloud->points[depth_idx];
			float depthvalue = depth.at<float>(u, v);
			//cout <<"depth value="<<depthvalue<<endl;
			pt.z = depthvalue;	//* 1000.0f;//depth_data[depth_idx] * 0.001f;
			pt.x = static_cast<float>(v - cx) * pt.z * constant;
			pt.y = static_cast<float>(u - cy) * pt.z * constant;
			Vec3b& colour = img.at<Vec3b>(u, v);
			pt.b = colour[0];
			pt.g = colour[1];
			pt.r = colour[2];
		}
	//cout <<"image_to_pcl::pcl_cloud->points.size()="<<pcl_cloud->points.size()<<endl;

}

void Utils::scale_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
		float factor) {

	register int depth_idx = 0;

	for (unsigned int u = 0; u < pcl_cloud->width; ++u)
		for (unsigned int v = 0; v < pcl_cloud->height; ++v, ++depth_idx) {
			pcl::PointXYZRGB& pt = pcl_cloud->points[depth_idx];

			pt.z *= factor;
			pt.x *= factor;
			pt.y *= factor;
		}
}

/*
 *! brief Creates a colour point cloud for a given image and depth map
 *
 * @params
 * [in] Mat& img: the image in the original size given by the camera
 * [in] Mat& depth: the depth map in the original size. Its type should be CV_32FC1 (float)
 * and the range should be measured in meters.
 * [out] pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud: the result colour point cloud
 * [in] int cx: the center of the image in x coordinates (colums)
 * [in] int cy: the center of the image in y coordinates (rows)
 * [in] Rect& rect: the bounding rectangle from which the cropped img and depth maps where obtained
 * 					with respect to the original size images
 */
void Utils::image_to_pcl(Mat& img, Mat& depth,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, int cx, int cy,
		Rect& rect) {

	// explicitly specify dsize=dst.size(); fx and fy will be computed from that.
	if (img.size() != depth.size())
		resize(depth, depth, img.size(), 0, 0, cv::INTER_CUBIC);

	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = img.cols;
	pcl_cloud->height = img.rows;
	pcl_cloud->points.resize(pcl_cloud->height * pcl_cloud->width);

	//int* depth_data = new int[pcl_cloud->height * pcl_cloud->width];
	//copy the depth values of every pixel in here

	//register float constant = 1.0f / 525;
	float constant = 1.0f / 1368.3;
	//  register int centerX = (pcl_cloud->width >> 1);
	//  int centerY = (pcl_cloud->height >> 1);
	//register int centerX = (pcl_cloud->width / 2);
	//int centerY = (pcl_cloud->height / 2);
	//cout << " centerX: " << centerX << " centerY: " << centerY << endl;

	register int depth_idx = 0;

	for (int u = 0; u < img.rows; ++u)
		for (int v = 0; v < img.cols; ++v, ++depth_idx) {
			pcl::PointXYZRGB& pt = pcl_cloud->points[depth_idx];
			float depthvalue = depth.at<float>(u, v);

			pt.z = depthvalue;	//* 1000.0f;//depth_data[depth_idx] * 0.001f;
			pt.x = static_cast<float>(v + rect.x - cx) * pt.z * constant;
			pt.y = static_cast<float>(u + rect.y - cy) * pt.z * constant;
			Vec3b& colour = img.at<Vec3b>(u, v);
			pt.b = colour[0];
			pt.g = colour[1];
			pt.r = colour[2];
		}
	//for (int v = -centerY; v < centerY; ++v) {
	//	for (register int u = -centerX; u < centerX; ++u, ++depth_idx) {
	//cout <<" accessing u,v="<<u<<v<<endl;

	//}
	//}
	cout << "image_to_pcl::pcl_cloud->points.size()="
			<< pcl_cloud->points.size() << endl;

}

void Utils::copy_clouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud_1,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_cloud_2) {
	//pcl_cloud_2->points.resize(pcl_cloud_1->size());
	//int i = 0;
	for (pcl::PointXYZRGB& p : *pcl_cloud_1) {

		pcl::PointXYZRGBA prgba;
		prgba.x = p.x;
		prgba.y = p.y;
		prgba.z = p.z;

		prgba.r = p.r;
		prgba.g = p.g;
		prgba.b = p.b;
		prgba.a = 255;
		pcl_cloud_2->points.push_back(prgba);
		//i++;
	}
	pcl_cloud_2->width = pcl_cloud_1->width;
	pcl_cloud_2->height = pcl_cloud_1->height;
	cout << "pcl_cloud_2 #points=" << pcl_cloud_2->points.size();
}
