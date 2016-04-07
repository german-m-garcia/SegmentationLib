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
#include <pcl/io/vtk_io.h>
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

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	//visualize it
	pcl::visualization::PCLVisualizer viewer("CLOUD");
	//viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud < pcl::PointXYZ > (cloud, "CLOUD");
	viewer.addCoordinateSystem(2.5);
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "CLOUD");
	viewer.spin();
}



int main_(int argc, char ** argv) {
	vtkSmartPointer < vtkPLYReader > readerQuery = vtkSmartPointer
			< vtkPLYReader > ::New();
	readerQuery->SetFileName(argv[1]);
	vtkSmartPointer < vtkPolyData > polydata = readerQuery->GetOutput();
	polydata->Update();

	pcl::visualization::PCLVisualizer vis("Visualizer");

	vis.addModelFromPolyData(polydata, "mesh1", 0);

//  vis.camera_.window_size ={480,480};
//  vis.camera_.pos = {0,5,5};
//  vis.camera_.focal = {0,0,0};
//  vis.camera_.view = {0,1,0};

	//vis.updateCamera();
	vis.resetCamera();

	vis.setRepresentationToSurfaceForAllActors();
	vis.spin();

	//call render in the visualizer to obtain a point cloud of the scene
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
			new pcl::PointCloud<pcl::PointXYZ>());
	vis.renderView(256, 256, cloud_out);
	visualize(cloud_out);

	pcl::io::savePCDFileASCII("scene.pcd", *cloud_out);

	return 0;
}

int main(int argc, char** argv) {
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(argv[1], cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in cloud
	visualize(cloud);



	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
			new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
			new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	pcl::io::saveVTKFile("mesh.vtk", triangles);

	// Finish
	return (0);
}

