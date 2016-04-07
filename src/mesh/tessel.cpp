/*
 * tessel.cpp
 *
 *  Created on: 1 Mar 2016
 *      Author: martin
 */
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/texture_mapping.h>

#include <pcl/io/vtk_lib_io.h>

int
main (int argc, char** argv)
{


	// read mesh from plyfile
	PCL_INFO ("\nLoading mesh from file %s...\n", argv[1]);
	pcl::PolygonMesh model;
	pcl::io::loadPolygonFilePLY(argv[1], model);

	pcl::RenderViewsTesselatedSphere render_views;
	render_views.setResolution (resolution_);
	render_views.setTesselationLevel (1);
	//80 views
	render_views.addModelFromPolyData (model);
	//vtk model
	render_views.generateViews ();
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > views;
	std::vector < Eigen::Matrix4f > poses;
	render_views.getViews (views);
	render_views.getPoses (poses);
}
