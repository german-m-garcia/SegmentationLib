/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */



#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>

#include "surface_model.hpp"
#include "z_adaptive_normals.h"
#include "cluster_normals_to_planes.h"
#include "surface_cluster.h"




using namespace std;
using namespace cv;

//#define DEBUG 0



struct timespec t2, t3;
    

  


void tic() {
    clock_gettime(CLOCK_MONOTONIC,  &t2);
  }

void toc(std::string text) {
    clock_gettime(CLOCK_MONOTONIC,  &t3);
    //time in mseconds
    double dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;
    cout <<" >"<<text<<" elapsed time="<<dt1<<" s"<<endl;
}


namespace pclAddOns {

template<class T>
bool readPointCloud(std::string filename,
                    typename pcl::PointCloud<T>::Ptr &cloud) {
  if (cloud.get() == 0)
    cloud.reset(new pcl::PointCloud<T>);

  if (pcl::io::loadPCDFile < T > (filename, *cloud) == -1) {
    std::cerr << "[ERROR] Couldn't read point cloud." << std::endl;
    return false;
  }

  return (true);
}

void ClipDepthImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pcl_cloud) {
  for (unsigned row = 0; row < _pcl_cloud->height; row++) {
    int idx = row * _pcl_cloud->width;
    _pcl_cloud->points[idx].x = NAN;
    _pcl_cloud->points[idx].y = NAN;
    _pcl_cloud->points[idx].z = NAN;
  }
  for (unsigned col = 0; col < _pcl_cloud->width; col++) {
    int idx = (_pcl_cloud->height - 1) * _pcl_cloud->width + col;
    _pcl_cloud->points[idx].x = NAN;
    _pcl_cloud->points[idx].y = NAN;
    _pcl_cloud->points[idx].z = NAN;
  }
}

void ConvertPCLCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) {
  out.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  out->width = in->width;
  out->height = in->height;
  out->points.resize(in->width * in->height);
  for (unsigned row = 0; row < in->height; row++) {
    for (unsigned col = 0; col < in->width; col++) {
      int idx = row * in->width + col;
      pcl::PointXYZRGBL &pt = in->points[idx];
      pcl::PointXYZRGB &npt = out->points[idx];
      npt.x = pt.x;
      npt.y = pt.y;
      npt.z = pt.z;
      npt.rgb = pt.rgb;
    }
  }
}

void ConvertPCLCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in,
                     pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &out) {
  out.reset(new pcl::PointCloud<pcl::PointXYZRGBL>);

  out->width = in->width;
  out->height = in->height;
  out->points.resize(in->width * in->height);
  for (unsigned row = 0; row < in->height; row++) {
    for (unsigned col = 0; col < in->width; col++) {
      int idx = row * in->width + col;
      pcl::PointXYZRGB &pt = in->points[idx];
      pcl::PointXYZRGBL &npt = out->points[idx];
      npt.x = pt.x;
      npt.y = pt.y;
      npt.z = pt.z;
      npt.rgb = pt.rgb;
    }
  }
}

}

ClusterSurfaces::ClusterSurfaces() {
  rgbd_filename = "data/cloud34.pcd";
}

ClusterSurfaces::~ClusterSurfaces() {

}

void ClusterSurfaces::readData(std::string filename) {

  if (!(pclAddOns::readPointCloud<pcl::PointXYZRGBL>(filename.c_str(),
                                                     pcl_cloud_l))) {
    //exit(0);
    if (!(pclAddOns::readPointCloud<pcl::PointXYZRGB>(filename.c_str(),
                                                      pcl_cloud))) {
      exit(0);
    }
    pclAddOns::ClipDepthImage(pcl_cloud);
    return;
  }

  pclAddOns::ConvertPCLCloud(pcl_cloud_l, pcl_cloud);
  pclAddOns::ClipDepthImage(pcl_cloud);
}

void ClusterSurfaces::readData(std::string rgb_filename,
                               std::string depth_filename) {

  cout <<"Reading "<<rgb_filename<<" and "<<depth_filename<<endl;
  Mat img = imread(rgb_filename, CV_LOAD_IMAGE_COLOR);
  Mat depth = imread(depth_filename, CV_LOAD_IMAGE_UNCHANGED);

  // explicitly specify dsize=dst.size(); fx and fy will be computed from that.
  if(img.size() != depth.size())
	  resize(depth, depth, img.size(), 0, 0, INTER_CUBIC);

  pcl_cloud.reset(new pcl::PointCloud< pcl::PointXYZRGB>);
  pcl_cloud->width = img.cols;
  pcl_cloud->height = img.rows;
  pcl_cloud->points.resize(pcl_cloud->height * pcl_cloud->width);

  //int* depth_data = new int[pcl_cloud->height * pcl_cloud->width];
  //copy the depth values of every pixel in here

  register float constant = 1.0f / 525;
//  register int centerX = (pcl_cloud->width >> 1);
//  int centerY = (pcl_cloud->height >> 1);
  register int centerX = (pcl_cloud->width / 2);
  int centerY = (pcl_cloud->height / 2 );
  cout <<" centerX: "<<centerX<<" centerY: "<<centerY<<endl;
#ifdef DEBUG
  imshow("img",img);
#endif
  register int depth_idx = 0;
  for (int v = -centerY; v < centerY; ++v) {
    for (register int u = -centerX; u < centerX; ++u, ++depth_idx) {
      //cout <<" accessing u,v="<<u<<v<<endl;
      pcl::PointXYZRGB& pt = pcl_cloud->points[depth_idx];
      uint16_t depthvalue = depth.at<uint16_t>(v+centerY,u+centerX);
      pt.z = depthvalue* 0.001f;//depth_data[depth_idx] * 0.001f;
      pt.x = static_cast<float>(u) * pt.z * constant;
      pt.y = static_cast<float>(v) * pt.z * constant;
      Vec3b& colour = img.at<Vec3b>(v+centerY,u+centerX);
      pt.b = colour[0];
      pt.g = colour[1];
      pt.r = colour[2];
    }
  }

//  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//  viewer.showCloud(pcl_cloud);
//
//  while (!viewer.wasStopped()) {
//  }
  pclAddOns::ConvertPCLCloud(pcl_cloud,pcl_cloud_l);

  /*if (!(pclAddOns::readPointCloud<pcl::PointXYZRGBL>(filename.c_str(),
   pcl_cloud_l))) {
   //exit(0);
   if (!(pclAddOns::readPointCloud<pcl::PointXYZRGB>(filename.c_str(),
   pcl_cloud))) {
   exit(0);
   }
   pclAddOns::ClipDepthImage(pcl_cloud);
   return;
   }

   pclAddOns::ConvertPCLCloud(pcl_cloud_l, pcl_cloud);
   pclAddOns::ClipDepthImage(pcl_cloud);*/
}

void ClusterSurfaces::calculateNormals() {
  // calcuate normals
  std::string text ="normals.reset";
  //tic();
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  //toc(text);
text ="nor.compute";
  surface::ZAdaptiveNormals<pcl::PointXYZRGB>::Parameter param;
  param.adaptive = true;
  surface::ZAdaptiveNormals<pcl::PointXYZRGB> nor(param);
  nor.setInputCloud(pcl_cloud);
  // tic();
  nor.compute();
  //toc(text);
  nor.getNormals(normals);
}

void ClusterSurfaces::calculatePatches() {
  surface::ClusterNormalsToPlanes::Parameter param;
  param.adaptive = true;         // use adaptive thresholds
  param.epsilon_c = 0.58;         //0.62;//
  param.omega_c = -0.002;

  clusterNormals = surface::ClusterNormalsToPlanes::Ptr(
      new surface::ClusterNormalsToPlanes(param));
  // adaptive clustering
  clusterNormals->setInputCloud(pcl_cloud);
  clusterNormals->setNormals(normals);
  clusterNormals->setPixelCheck(true, 5);
  clusterNormals->compute();
  normals = clusterNormals->getNormals();
  surfaces = clusterNormals->getSurfaces();
}

// show resulting segmentation to png image
void ClusterSurfaces::showPatches(cv::Mat &kImage) {
  // create color image
  kImage = cv::Mat_<cv::Vec3b>::zeros(pcl_cloud->height, pcl_cloud->width);
  for (size_t i = 0; i < surfaces.size(); i++) {
    uchar r = std::rand() % 255;
    uchar g = std::rand() % 255;
    uchar b = std::rand() % 255;

    if (!(surfaces.at(i)->selected))
      continue;

    for (size_t j = 0; j < surfaces.at(i)->indices.size(); j++) {
      int row = surfaces.at(i)->indices.at(j) / pcl_cloud->width;
      int col = surfaces.at(i)->indices.at(j) % pcl_cloud->width;
      cv::Vec3b &cvp = kImage.at<cv::Vec3b>(row, col);
      cvp[0] = r;
      cvp[1] = g;
      cvp[2] = b;
    }
  }
#ifdef DEBUG
  imshow("patches",kImage);
  waitKey(0);
#endif
}

void ClusterSurfaces::run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,cv::Mat&  dst){
	pcl_cloud = cloud;
	pclAddOns::ConvertPCLCloud(pcl_cloud,pcl_cloud_l);
	calculateNormals();
	calculatePatches();
	showPatches(dst);
}

void ClusterSurfaces::run(std::string filename) {
  readData(filename);
  calculateNormals();
  calculatePatches();

  cv::Mat kImage;
  showPatches(kImage);
  boost::filesystem::path path(filename);
  std::string image_name = path.filename().c_str();
  image_name = image_name + "_patches.png";
  cout << "saving output at: "<<image_name<<endl;
  cv::imwrite(image_name, kImage);
}

void ClusterSurfaces::run(std::string rgb_filename,
                          std::string depth_filename) {
  readData(rgb_filename, depth_filename);
  calculateNormals();
  calculatePatches();

  cv::Mat kImage;
  showPatches(kImage);
  boost::filesystem::path path(rgb_filename);
  std::string image_name = path.filename().c_str();
  image_name = image_name + "_patches.png";
  cv::imwrite(image_name, kImage);
}
/*
void printUsage(char *av) {
  printf("Usage: %s [options] \n"
         " Options:\n"
         "   [-h] ... show this help.\n"
         "   [-f rgbd_filename] ... specify rgbd-image path and filename",
         av);
  std::cout << " Example: " << av << " -f data/cloud34.pcd" << std::endl;
}

int main(int argc, char *argv[]) {
  std::string rgbd_filename = "", rgb_filename = "", depth_filename = "";

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-h") == 0) {
      printUsage(argv[0]);
      exit(0);
    }
    //-f point_cloud
    if (i + 1 < argc) {
      if (strcmp(argv[i], "-f") == 0)
        rgbd_filename = argv[i + 1];
    }
    //-d depth file
    if (i + 1 < argc) {
      if (strcmp(argv[i], "-d") == 0)
        depth_filename = argv[i + 1];
    }
    //-i RGB file
    if (i + 1 < argc) {
      if (strcmp(argv[i], "-i") == 0)
        rgb_filename = argv[i + 1];
    } else
      printUsage(argv[0]);
  }

  ClusterSurfaces clusterSurfaces_;
  if (depth_filename != "" && rgb_filename != "")
    clusterSurfaces_.run(rgb_filename, depth_filename);
  else
    clusterSurfaces_.run(rgbd_filename);
}*/
