/*
 * test.cpp
 *
 *  Created on: 20 Jan 2016
 *      Author: martin
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <boost/filesystem.hpp>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/pcl_macros.h"
#include "pcl/ros/conversions.h"
#include "pcl/io/pcd_io.h"

#include "utils.h"
#include "pcl_segmentation.h"

int main(int argc, char ** argv) {
	Utils utils;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_SLC(
				new pcl::PointCloud<pcl::PointXYZRGB>);

	/*
	 *
	 *      |
	 *6 cm  | (Y)
	 *      |_______  8 cm (X)
	 *     /
	 *    /  12 cm (Z)
	 *   /
	 *
	 */



	double side_8 =  0.08,side_6 =  0.06, side_12 =  0.12;
	double sampling_interval = 0.002;
	//ground plane
	for(double z = -side_12/2.; z < side_12/2.; z+= sampling_interval)
		for(double x = -side_8/2.; x < side_8/2.; x+= sampling_interval){
			pcl::PointXYZRGB p;
			p.x = x;
			p.y = -side_6/2.;
			p.z = z;
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud_SLC->push_back(p);
		}

	//back face
	for(double y = -side_6/2.; y < side_6/2.; y+= sampling_interval)
		for(double x = -side_8/2.; x < side_8/2.; x+= sampling_interval){
			pcl::PointXYZRGB p;
			p.x = x;
			p.y = y;
			p.z = -side_12/2.;
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud_SLC->push_back(p);
		}
	//left side face
	for(double z = -side_12/2.; z < side_12/2.; z+= sampling_interval)
			for(double y = -side_6/2.; y < side_6/2.; y+= sampling_interval){
				pcl::PointXYZRGB p;
				p.x = -side_8/2.;
				p.y = y;
				p.z = z;
				p.r = 0;
				p.g = 0;
				p.b = 255;
				cloud_SLC->push_back(p);
				p.x = side_8/2.;
				cloud_SLC->push_back(p);
			}

	std::string box("box");
	utils.display_cloud(cloud_SLC,box);
	pcl::io::savePCDFileASCII("/home/martin/bagfiles/slc_test.pcd", *cloud_SLC);

	return 0;
}


int main___(int argc, char ** argv) {

	PCLSegmentation pcl_segmentation;
	cv::Mat src = cv::imread("/home/martin/workspace/EGBISegmentation/build/input.png",CV_LOAD_IMAGE_COLOR);
	cv::Mat mat_segmentation;
	pcl_segmentation.remote_ms_segment(src, mat_segmentation);

	int row = 50;
	int col = 50;
	Segment * seg = pcl_segmentation.get_segment_at_fast( row,  col);
	cv::imshow("seg", seg->getMatOriginalColour());
	cv::waitKey(0);

	return 0;
}



int main__(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud_1.pcd", *cloud_in) == -1) //* load the file
	  {
	    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	    return (-1);
	  }

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud_2.pcd", *cloud_out) == -1) //* load the file
	  {
	    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	    return (-1);
	  }


	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: "
			<< icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}

int main_(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	// Fill in the CloudIn data
	cloud_in->width = 37;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);
	for (size_t i = 0; i < cloud_in->points.size(); ++i) {
		cloud_in->points[i].x = -1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = -1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = -1024 * rand() / (RAND_MAX + 1.0f);
	}
	cloud_in->points[0].x = NAN;
	cloud_in->points[10].x = NAN;
	cloud_in->points[15].x = -15.;
	cloud_in->points[16].x = -25.;
	cloud_in->points[17].x = -45.;

	std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
			<< std::endl;
//  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
//      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
//      cloud_in->points[i].z << std::endl;
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	cloud_out->points[10].x = NAN;
	std::cout << "Transformed " << cloud_in->points.size() << " data points:"
			<< std::endl;
//  for (size_t i = 0; i < cloud_out->points.size (); ++i)
//    std::cout << "    " << cloud_out->points[i].x << " " <<
//      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
	pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, indices);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_in);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: "
			<< icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}

