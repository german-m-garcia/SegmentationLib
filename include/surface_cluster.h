/*
 * clusterSurfaces.h
 *
 *  Created on: Oct 6, 2015
 *      Author: gmartin
 */

#ifndef SRC_SURFACECLUSTER_H_
#define SRC_SURFACECLUSTER_H_

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ClusterSurfaces {
 private:

  std::string rgbd_filename, rgb_filename, depth_filename;  ///< depth image filename

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_l;  ///< labeled pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;  ///< original pcl point cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;
  surface::ClusterNormalsToPlanes::Ptr clusterNormals;
  std::vector<surface::SurfaceModel::Ptr> surfaces;

 public:

 private:

  void showPatches(cv::Mat &kImage);
  void calculatePatches();
  void calculateNormals();
  //void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void readData(std::string filename);
  void readData(std::string rgb_filename, std::string depth_filename);

 public:
  ClusterSurfaces();
  ~ClusterSurfaces();

  void run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,cv::Mat&  dst);

  /** Run the pre-segmenter **/
  void run(std::string filename);

  /** Run the pre-segmenter **/
  void run(std::string rgb_filename, std::string depth_filename);

};


#endif /* SRC_SURFACECLUSTER_H_ */
