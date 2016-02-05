/*
 * MRSMapWrap.h
 *
 *  Created on: 7 Jan 2016
 *      Author: martin
 */

#ifndef INCLUDE_OBJECTS_MRSMAPWRAP_H_
#define INCLUDE_OBJECTS_MRSMAPWRAP_H_


#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <Eigen/Core>

#include "utils.h"

//#include <mrsmap/map/multiresolution_surfel_map.h>

//#include <mrsmap/registration/multiresolution_soft_surfel_registration.h>
//#include <mrsmap/registration/multiresolution_surfel_registration.h>



//#include <mrsmap/utilities/utilities.h>
//
//#include <mrsmap/slam/slam.h>
//#include <mrsmap/visualization/visualization_slam.h>


#include <boost/shared_ptr.hpp>
#include "segment.h"
#include <opencv2/imgproc/imgproc.hpp>

//using namespace mrsmap;




class MRSMapWrap {
public:
	MRSMapWrap();
	virtual ~MRSMapWrap();

	void show();
	void addPcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

	double icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,Eigen::Matrix4f& transform);
	double generalized_icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_2,Eigen::Matrix4f& transform);
	//double register_mrsmaps(boost::shared_ptr<MultiResolutionSurfelMap>& model_1, boost::shared_ptr<MultiResolutionSurfelMap>& model_2,Point3d& gravity_center_1,Point3d& gravity_center_2,Eigen::Matrix4d& transform);

	void pairAlign (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, Eigen::Matrix4f &final_transform, bool downsample = false);
	double test_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,Eigen::Matrix4f& transform);
	double test_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
	void initialize(cv::Mat& src, cv::Mat& depth, std::vector<Segment*>& segments);
	void initialize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudIn);

	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& getModelCloud() const {
		return model_cloud_;
	}

private:

	pcl::PointCloud< pcl::PointXYZRGB >::Ptr model_cloud_;
	Utils utils_;

//	boost::shared_ptr<MultiResolutionSurfelMap> model_mrsmap_;
//	Point3d model_gravity_center_;
//	boost::shared_ptr<MultiResolutionSurfelMap> current_mrsmap;
//
//	boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_,modelImageAllocator_;
//	boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > > curMapTreeNodeAllocator_,modelTreeNodeAllocator_;
//
//	Eigen::Matrix4d lastTransform_;
//
//	double min_resolution_, max_radius_;
//
//	bool skip_past_frames_;
//
//
//	boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > > treeNodeAllocator_[ 2 ];
//
//	//ViewerSLAM viewer_;
//	bool graphChanged_;
//	int use_pointfeatures_, downsampling_, debug_;

	//void model_mean(boost::shared_ptr<MultiResolutionSurfelMap>& model_1);

};

#endif /* INCLUDE_OBJECTS_MRSMAPWRAP_H_ */
