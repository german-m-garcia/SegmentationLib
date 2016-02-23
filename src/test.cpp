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
#include "objects/ObjectDetector.h"

const double PI = 3.141592;
const double PI_2 = 2* PI;
const double INTERVAL = 0.001;
const double NUT_HEIGHT = 0.014;
const double SCREW_HEIGHT = 0.1;

void circle_at_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double radius, double z){

	for(double angle = 0.; angle < PI_2; angle += INTERVAL){
		double x = radius * cos(angle);
		double y = radius * sin(angle);

		pcl::PointXYZRGB p;
		p.y = y;
		p.x = x;
		p.z = z;
		p.r = 0;
		p.g = 0;
		p.b = 255;
		cloud->push_back(p);
	}
}

void hexagon_at_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double z){
	/*
	 * | (Y)
	 * |
	 * |_____ (X)
	 *
	 *
	 *         -----
	 *
	 *    |				|
	 *    |				|
	 *
	 *         -----
	 *
	 */

	const double height = 0.035, width = 0.04, side = 0.02;

	//do the upper and lower sides
	for(double y = -height/2.; y <= height/2.; y += height)
		for(double x = -side/2.; x < side/2.; x +=INTERVAL){


			pcl::PointXYZRGB p;
			p.y = y;
			p.x = x;
			p.z = z;
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud->push_back(p);
		}
	//left upper side
	double origin_right_x = width-side*cos(PI/3.);

	for(double origin_left_x = -width/2, origin_left_y = 0. ; origin_left_x <= origin_right_x; origin_left_x+=(origin_right_x),
	origin_left_y -= height/2.)

	for(double length = 0.; length < side; length += INTERVAL){
		double angle = PI/3.;
		double x = origin_left_x + length * cos(angle);
		double y = origin_left_y +length * sin(angle);
		pcl::PointXYZRGB p;
		p.y = y;
		p.x = x;
		p.z = z;
		p.r = 0;
		p.g = 0;
		p.b = 255;
		cloud->push_back(p);
	}


	origin_right_x = width-side*cos(PI/3.);
	for(double origin_left_x = -width/2, origin_left_y = 0. ; origin_left_x <= origin_right_x; origin_left_x+=(origin_right_x),
		origin_left_y += height/2.)

		for(double length = 0.; length < side; length += INTERVAL){
			double angle = -PI/3.;
			double x = origin_left_x + length * cos(angle);
			double y = origin_left_y +length * sin(angle);
			pcl::PointXYZRGB p;
			p.y = y;
			p.x = x;
			p.z = z;
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud->push_back(p);
		}


}


//create SCREW
int main(int argc, char ** argv) {
	Utils utils;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_screw(
				new pcl::PointCloud<pcl::PointXYZRGB>);



	double radius = 0.01;
	//for(double z = NUT_HEIGHT; z < SCREW_HEIGHT+NUT_HEIGHT; z += INTERVAL)
	//	circle_at_height(cloud_screw,radius,z);

	for(double z = 0.; z < NUT_HEIGHT; z+= INTERVAL)
		hexagon_at_height(cloud_screw,z);



	std::string box("screw");
	Point3d point;
	//ObjectDetector::normalize_pcl(cloud_SLC,cloud_SLC,point);
	utils.display_cloud(cloud_screw,box);
	pcl::io::savePCDFileASCII("/home/martin/bagfiles/nut.pcd", *cloud_screw);

	return 0;
}

int create_SLC(int argc, char ** argv) {
	Utils utils;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_SLC(
				new pcl::PointCloud<pcl::PointXYZRGB>);

	/*
	 *
	 *        |
	 *7.5 cm  | (X)
	 *        |_______  9.5 cm (Y)
	 *       /
	 *      /  16 cm (Z)
	 *     /
	 *
	 */



	double side_8 =  0.095,side_6 =  0.075, side_12 =  0.16;
	double sampling_interval = 0.002;
	//ground plane
	for(double z = -side_12/2.; z < side_12/2.; z+= sampling_interval)
		for(double y = -side_8/2.; y < side_8/2.; y+= sampling_interval){
			pcl::PointXYZRGB p;
			p.y = y;
			p.x = -side_6/2.;
			p.z = z;
			p.r = 0;
			p.g = 0;
			p.b = 255;
			cloud_SLC->push_back(p);
		}

	//back face
	for(double x = -side_6/2.; x < side_6/2.; x+= sampling_interval)
		for(double y = -side_8/2.; y < side_8/2.; y+= sampling_interval){
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
			for(double x = -side_6/2.; x < side_6/2.; x+= sampling_interval){
				pcl::PointXYZRGB p;
				p.y = -side_8/2.;
				p.x = x;
				p.z = z;
				p.r = 0;
				p.g = 0;
				p.b = 255;
				cloud_SLC->push_back(p);
				p.y = side_8/2.;
				cloud_SLC->push_back(p);
			}

	std::string box("box");
	Point3d point;
	//ObjectDetector::normalize_pcl(cloud_SLC,cloud_SLC,point);
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

