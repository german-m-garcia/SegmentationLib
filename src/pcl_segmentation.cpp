/*
 * SegmenterLib.cpp
 *
 *  Created on: Oct 2, 2015
 *      Author: gmartin
 */

#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/connected_components.hpp>

#include "cluster_normals_to_planes.h"
#include "pcl_segmentation.h"
#include "base_segmentation.h"
#include "surface_cluster.h"

PCLSegmentation::PCLSegmentation() {}

PCLSegmentation::~PCLSegmentation() {

}



void PCLSegmentation::refineSupervoxels(pcl::SupervoxelClustering<PointT>& super,
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters,
		std::vector<SmoothClusters>& smooth_clusters, cv::Mat& res) {

//float dot_threshold_ = 0.99f;

	float supvxls_th = 0.05f;
	float dot_threshold_ = 0.99f;

	pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxels_labels_cloud =
			super.getLabeledCloud();
	uint32_t max_label = super.getMaxLabel();
//cout <<" supervoxel_max_label="<<max_label<<endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud =
			super.makeSupervoxelNormalCloud(supervoxel_clusters);

	std::vector<int> label_to_idx;
	label_to_idx.resize(max_label + 1, -1);
	typename std::map<uint32_t, typename pcl::Supervoxel<PointT>::Ptr>::iterator
			sv_itr, sv_itr_end;
	sv_itr = supervoxel_clusters.begin();
	sv_itr_end = supervoxel_clusters.end();
	int i = 0;
	for (; sv_itr != sv_itr_end; ++sv_itr, i++) {
		label_to_idx[sv_itr->first] = i;
	}

	std::vector < std::vector<bool> > adjacent;
	adjacent.resize(supervoxel_clusters.size());
	for (size_t i = 0; i < (supervoxel_clusters.size()); i++)
		adjacent[i].resize(supervoxel_clusters.size(), false);

	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
//To make a graph of the supervoxel adjacency, we need to iterate 	through the 	supervoxel adjacency	multimap
	std::multimap<uint32_t, uint32_t>::iterator label_itr =
			supervoxel_adjacency.begin();
//std::cout << "super voxel adjacency size:" << supervoxel_adjacency.size()
//			<< std::endl;
	for (; label_itr != supervoxel_adjacency.end();) {
		//First get the label
		uint32_t supervoxel_label = label_itr->first;

		//added: 3D point of the original supervoxel
		pcl::PointXYZRGBA& point_0 =
				supervoxel_clusters[supervoxel_label]->centroid_;

		Eigen::Vector3f normal_super_voxel =
				sv_normal_cloud->points[label_to_idx[supervoxel_label]].getNormalVector3fMap();

		normal_super_voxel.normalize();
		//Now we need to iterate through the adjacent supervoxels and		make a 		point cloud 		of them
		std::multimap<uint32_t, uint32_t>::iterator adjacent_itr =
				supervoxel_adjacency.equal_range(supervoxel_label).first;
		for (;
				adjacent_itr
						!= supervoxel_adjacency.equal_range(supervoxel_label).second;
				++adjacent_itr) {

			//added: 3D point of the adjacent candidate supervoxel
			pcl::PointXYZRGBA& point_f =
					supervoxel_clusters[adjacent_itr->second]->centroid_;
			//pcl::PointXYZL& point_f = supervoxels_labels_cloud->points[label_to_idx[adjacent_itr->second]];
			//compute vector
			Eigen::Vector3f supervoxels_vector(point_f.x - point_0.x,
					point_f.y - point_0.y, point_f.z - point_0.z);
			supervoxels_vector.normalize();
			Eigen::Vector3f normal_neighbor_supervoxel =
					sv_normal_cloud->points[label_to_idx[adjacent_itr->second]].getNormalVector3fMap();

			//cout << "normal_neighbor_supervoxel="<<normal_neighbor_supervoxel<<endl;

			normal_neighbor_supervoxel.normalize();

//			if (normal_super_voxel.dot(normal_neighbor_supervoxel)
//					> dot_threshold_) {

			if (std::fabs(normal_super_voxel.dot(supervoxels_vector))
					< supvxls_th
					&& normal_super_voxel.dot(normal_neighbor_supervoxel)
							> dot_threshold_) {

				adjacent[label_to_idx[supervoxel_label]][label_to_idx[adjacent_itr->second]] =
						true;
			}
		}

		//Move iterator forward to next label
		label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
	}

	typedef boost::adjacency_matrix<boost::undirectedS, int> Graph;
	Graph G(supervoxel_clusters.size());
	for (size_t i = 0; i < supervoxel_clusters.size(); i++) {
		for (size_t j = (i + 1); j < supervoxel_clusters.size(); j++) {
			if (adjacent[i][j])
				boost::add_edge(i, j, G);
		}
	}

	std::vector<int> components(boost::num_vertices(G));
	int n_cc = static_cast<int>(boost::connected_components(G, &components[0]));
	std::cout << "Number of connected components..." << n_cc << std::endl;

	std::vector<int> cc_sizes;
	std::vector < std::vector<int> > ccs;
	std::vector<uint32_t> original_labels_to_merged;
	original_labels_to_merged.resize(supervoxel_clusters.size());

	ccs.resize(n_cc);
	cc_sizes.resize(n_cc, 0);
	typename boost::graph_traits<Graph>::vertex_iterator vertexIt, vertexEnd;
	boost::tie(vertexIt, vertexEnd) = vertices(G);
	for (; vertexIt != vertexEnd; ++vertexIt) {
		int c = components[*vertexIt];
		cc_sizes[c]++;
		ccs[c].push_back(*vertexIt);
		original_labels_to_merged[*vertexIt] = c;
	}

	for (size_t i = 0; i < supervoxels_labels_cloud->points.size(); i++) {
		//std::cout << supervoxels_labels_cloud->points[i].label << "
//		" << label_to_idx.size() << " " << original_labels_to_merged.size() <<
//				" " << label_to_idx[supervoxels_labels_cloud->points[i].label]
//				<< std::endl;
		if (label_to_idx[supervoxels_labels_cloud->points[i].label] < 0)
			continue;

		supervoxels_labels_cloud->points[i].label =
				original_labels_to_merged[label_to_idx[supervoxels_labels_cloud->points[i].label]];
	}

//create clusters
	smooth_clusters.resize(ccs.size());

	for (size_t i = 0; i < supervoxels_labels_cloud->points.size(); i++) {
		if (supervoxels_labels_cloud->points[i].label <= 0)
			continue;

		smooth_clusters[supervoxels_labels_cloud->points[i].label].indices_.indices.push_back(
				i);
	}
	//read_segments(res,res);
	show_patches(res, supervoxels_labels_cloud);

}

void PCLSegmentation::supervoxelSegment(Mat& src,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_cloud, bool rgbd_refine,
		cv::Mat& outMat) {

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_l; ///< labeled pcl point cloud
//bool use_transform = !pcl::console::find_switch(argc, argv, "--NT");
	bool use_transform = true;
	float voxel_resolution = 0.006f;
	float seed_resolution = 0.1f;
	float color_importance = 0.2f;
	float spatial_importance = 0.4f;

	float normal_importance = 1.5f;

//////////////////////////////  //////////////////////////////
////// This is how to use supervoxels
//////////////////////////////  //////////////////////////////

	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution,
			use_transform);
	super.setInputCloud(pcl_cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);

	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

	pcl::console::print_highlight("Extracting supervoxels!\n");
	super.extract(supervoxel_clusters);
//super.refineSupervoxels(5,supervoxel_clusters);
	pcl::console::print_info("Found %d supervoxels\n",
			supervoxel_clusters.size());

	PointCloudT::Ptr colored_cloud = super.getColoredCloud();

	std::vector<SmoothClusters> smooth_clusters;
	if (rgbd_refine)
		refineSupervoxels(super, supervoxel_clusters, smooth_clusters, outMat);

	else
		show_patches(outMat, super.getLabeledCloud());

	read_segments(src, outMat);

}
/*
 * Set of working parameters:
 *
 * 	---------------1------------------------
 * 	float voxel_resolution = 0.01f;
 float seed_resolution = 0.04f;
 float concavity_tolerance_threshold = 12;
 float color_importance = 1.5f;
 float spatial_importance = 0.0f;
 float normal_importance = 4.f;
 cout << "colour, spatial , and normal importances=" << color_importance
 << " " << spatial_importance << " " << normal_importance << endl;
 bool use_single_cam_transform = true;
 bool use_supervoxel_refinement = false;

 // LCCPSegmentation Stuff

 float smoothness_threshold = 0.5;
 uint32_t min_segment_size = 3;
 bool use_extended_convexity = true;
 bool use_sanity_criterion = true;
 * 	---------------1------------------------
 *
 *
 *
 * 	---------------3------------------------
 * 	float voxel_resolution = 0.006f;
 float seed_resolution = 0.1f;
 float color_importance = 1.5f;
 float spatial_importance = 1.0f;
 float normal_importance = 4.f;
 cout << "colour, spatial , and normal importances=" << color_importance
 << " " << spatial_importance << " " << normal_importance << endl;
 bool use_single_cam_transform = false;
 bool use_supervoxel_refinement = false;

 // LCCPSegmentation Stuff
 float concavity_tolerance_threshold = 25;
 float smoothness_threshold = 0.1;
 uint32_t min_segment_size = 0;
 bool use_extended_convexity = true;
 bool use_sanity_criterion = true;
 *  ---------------3------------------------
 */
void PCLSegmentation::lccpSegment(Mat& src,
		pcl::PointCloud<PointT>::Ptr input_cloud_ptr, cv::Mat& outMat) {

///  Default values of parameters before parsing
// Supervoxel Stuff

//	float voxel_resolution = 0.01f;
//	float seed_resolution = 0.04f;
//	float concavity_tolerance_threshold = 9;
//	float color_importance = 0.0f;
//	float spatial_importance = 1.0f;
//	float normal_importance = 4.f;
//	cout << "colour, spatial , and normal importances=" << color_importance
//			<< " " << spatial_importance << " " << normal_importance << endl;
//	bool use_single_cam_transform = true;
//	bool use_supervoxel_refinement = false;
//
//// LCCPSegmentation Stuff
//
//	float smoothness_threshold = 0.4;
//	uint32_t min_segment_size = 0;
//	bool use_extended_convexity = true;
//	bool use_sanity_criterion = true;
//
//	unsigned int k_factor = 0;
//	if (use_extended_convexity)
//		k_factor = 1;
//
///// Preparation of Input: Supervoxel Oversegmentation
//
//	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
//	//PCL 1.8
//	//super.setUseSingleCameraTransform(use_single_cam_transform);
//	super.setInputCloud(input_cloud_ptr);
////if (has_normals)
////	super.setNormalCloud(input_normals_ptr);
//	super.setColorImportance(color_importance);
//	super.setSpatialImportance(spatial_importance);
//	super.setNormalImportance(normal_importance);
//	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
//
//	PCL_INFO("Extracting supervoxels\n");
//	super.extract(supervoxel_clusters);
//
//	if (use_supervoxel_refinement) {
//		PCL_INFO("Refining supervoxels\n");
//		super.refineSupervoxels(2, supervoxel_clusters);
//	}
//	std::stringstream temp;
//	temp << "  Nr. Supervoxels: " << supervoxel_clusters.size() << "\n";
//	PCL_INFO(temp.str().c_str());
//
//	PCL_INFO("Getting supervoxel adjacency\n");
//	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
//	super.getSupervoxelAdjacency(supervoxel_adjacency);
//
///// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
//	pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud =
//			pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(
//					supervoxel_clusters);
//
///// The Main Step: Perform LCCPSegmentation
//
//	PCL_INFO("Starting Segmentation\n");
//	pcl::LCCPSegmentation<PointT> lccp;
//	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
//	lccp.setSanityCheck(use_sanity_criterion);
//	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution,
//			smoothness_threshold);
//	lccp.setKFactor(k_factor);
//	lccp.segment(supervoxel_clusters, supervoxel_adjacency);
//
//	if (min_segment_size > 0) {
//		PCL_INFO("Merging small segments\n");
//		lccp.mergeSmallSegments(min_segment_size);
//	}
//
//	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
//	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud =
//			super.getLabeledCloud();
//	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud =
//			sv_labeled_cloud->makeShared();
//	lccp.relabelCloud(*lccp_labeled_cloud);
//	SuperVoxelAdjacencyList sv_adjacency_list;
//	lccp.getSVAdjacencyList(sv_adjacency_list);  // Needed for visualization
//
//	show_patches(outMat, lccp_labeled_cloud);

	//read_segments(src, outMat);

/// Creating Colored Clouds and Output
//	if (lccp_labeled_cloud->size() == input_cloud_ptr->size()) {
//		if (output_specified) {
//			PCL_INFO("Saving output\n");
//			if (add_label_field) {
//				if (pcl::getFieldIndex(input_pointcloud2, "label") >= 0)
//					PCL_WARN(
//							"Input cloud already has a label field. It will be overwritten by the lccp segmentation output.\n");
//				pcl::PCLPointCloud2 output_label_cloud2, output_concat_cloud2;
//				pcl::toPCLPointCloud2(*lccp_labeled_cloud, output_label_cloud2);
//				pcl::concatenateFields(input_pointcloud2, output_label_cloud2,
//						output_concat_cloud2);
//				pcl::io::savePCDFile(outputname + "_out.pcd",
//						output_concat_cloud2, Eigen::Vector4f::Zero(),
//						Eigen::Quaternionf::Identity(), save_binary_pcd);
//			} else
//				pcl::io::savePCDFile(outputname + "_out.pcd",
//						*lccp_labeled_cloud, save_binary_pcd);
//
//			if (sv_output_specified) {
//				pcl::io::savePCDFile(outputname + "_svcloud.pcd",
//						*sv_centroid_normal_cloud, save_binary_pcd);
//			}
//		}
//	} else {
//		PCL_ERROR(
//				"ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
//	}

}



/*
 * Vienna Surface Patches
 *
 */

void PCLSegmentation::surfacePatches(Mat& src,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
		cv::Mat& outMat) {

	ClusterSurfaces clusterSurfaces;

	clusterSurfaces.run(input_cloud_ptr, outMat);
	read_segments(src, outMat);

}







/*
 *
 * MEANSHIFT
 *
 *
 */



void PCLSegmentation::remote_ms_segment(Mat& src, Mat& dst) {
	Mat img2;
	int sp = 15, sr = 40;
	//cout <<">bilateralFilter..."<<endl;

	cv::imwrite("/home/martin/workspace/EGBISegmentation/build/input.png",src);
	cout <<" saved input image for segmentation"<<endl;
	int rc =system (" ~/workspace/EGBISegmentation/build/ms_segment ~/workspace/EGBISegmentation/build/input.png 15 15 30 ~/workspace/EGBISegmentation/build/out.png");
	cout <<" system call returned "<<rc<<endl;
	//open the segmentation result
	dst = imread("/home/martin/workspace/EGBISegmentation/build/out.png",CV_LOAD_IMAGE_COLOR);
	read_segments(src,dst);

}

void PCLSegmentation::mssegment(Mat& src, Mat& dst) {
	Mat img2;
	int sp = 15, sr = 40;
	//cout <<">bilateralFilter..."<<endl;
	bilateralFilter(src, img2, 15, 25, 25);
	cout <<">pyrMeanShiftFiltering..."<<endl;
	Mat hsv;
	cv::cvtColor(img2, hsv, CV_BGR2HSV);
	pyrMeanShiftFiltering(hsv, dst, sp, sr, 1,
			cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 10, 1));
	flood_fill_post_process(dst, Scalar::all(5));

	read_segments(src,dst);

}
