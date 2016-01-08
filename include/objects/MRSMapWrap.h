/*
 * MRSMapWrap.h
 *
 *  Created on: 7 Jan 2016
 *      Author: martin
 */

#ifndef INCLUDE_OBJECTS_MRSMAPWRAP_H_
#define INCLUDE_OBJECTS_MRSMAPWRAP_H_

#include <mrsmap/map/multiresolution_surfel_map.h>

#include <mrsmap/registration/multiresolution_soft_surfel_registration.h>
#include <mrsmap/registration/multiresolution_surfel_registration.h>

#include <mrsmap/utilities/utilities.h>

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <boost/shared_ptr.hpp>
#include "segment.h"
#include <opencv2/imgproc/imgproc.hpp>

using namespace mrsmap;
using namespace pcl;

class MRSMapWrap {
public:
	MRSMapWrap();
	virtual ~MRSMapWrap();

	void show();
	void initialize(cv::Mat& src, cv::Mat& depth, std::vector<Segment*>& segments);
	void initialize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudIn);
private:
	boost::shared_ptr< MultiResolutionSurfelMap > map_;

	double max_resolution_, max_radius_, dist_dep_;

	bool create_map_;


	std::string map_folder_;
	std::string object_name_;
	std::string init_frame_;


	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudv;

	boost::shared_ptr< MultiResolutionSurfelMap::ImagePreAllocator > imageAllocator_;
	boost::shared_ptr< spatialaggregate::OcTreeNodeDynamicAllocator< float, MultiResolutionSurfelMap::NodeValue > > treeNodeAllocator_;
};

#endif /* INCLUDE_OBJECTS_MRSMAPWRAP_H_ */
