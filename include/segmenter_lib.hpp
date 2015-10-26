/*
 * SegmenterLib.h
 *
 *  Created on: Oct 2, 2015
 *      Author: gmartin
 */

#ifndef SRC_SEGMENTERLIB_H_
#define SRC_SEGMENTERLIB_H_

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <fstream>
#include <vector>
#include <map>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

//PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// The segmentation class this example is for
#include <pcl/segmentation/lccp_segmentation.h>
//restore the include above if using the latest PCL
//#include "lccp_segmentation.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "segment.h"

using namespace std;
using namespace cv;


#define SEGMENTS 600

/*
 * Segmentation modes
 */
#define COLOUR 0
#define RGBD_SUPERVOXELS 1

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

class SmoothClusters {
public:
	pcl::PointIndices indices_;
	Eigen::Vector3f avg_normal_;
	Eigen::Vector4f plane_;

};

class SegmenterLib {
public:
	SegmenterLib();
	virtual ~SegmenterLib();

	void refineSupervoxels(pcl::SupervoxelClustering<PointT>& super,
			map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters,
			vector<SmoothClusters>& smooth_clusters, Mat& res);
	void supervoxelSegment(Mat& src,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_cloud,
			bool rgbd_refine, Mat& outMat);

	void lccpSegment(Mat& src,pcl::PointCloud<PointT>::Ptr input_cloud_ptr,
			Mat& outMat);

	void surfacePatches(Mat& src,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
			Mat& outMat);

	void cleanData();

	Segment* getComponentAt_fast(int row, int col);

	void mssegment(Mat& src, Mat& dst);

	vector<Segment*>& getSegments() {
		return segments;

	}

private:
	std::vector<Vec3b> mapLabels;
	Mat component_id;
	vector<Segment*> segments;
	map<uint16_t,Segment*> mapSegments;
	void showPatches(Mat &kImage,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_cloud,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& labeled_cloud);
	void showPatches(Mat &kImage,
			pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud);

	int getLabel(map<uint32_t, int>& labels, int label);


	void readSegments(Mat& original,Mat& img);

	void readSegments(Mat& img,std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>& supervoxel_clusters);

	void floodFillPostprocess(Mat& img, const Scalar& colorDiff);

};

#endif /* SRC_SEGMENTERLIB_H_ */

