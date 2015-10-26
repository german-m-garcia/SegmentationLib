/**
 * $Id$
 *
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
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

#ifndef SURFACE_ZADAPTIVE_NORMALS_HH
#define SURFACE_ZADAPTIVE_NORMALS_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/eigen.h>
#include <pcl/common/eigen.h>
#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

namespace EPUtils
{
  
template <class T>
bool computeMean(const typename pcl::PointCloud<T> &cloud, Eigen::Vector3f &mean,
                 std::vector<int> indices = std::vector<int>());

template <class T>
bool computeCovarianceMatrix(const typename pcl::PointCloud<T> &cloud, const Eigen::Vector3f &mean,
                             Eigen::Matrix3f &cov, std::vector<int> indices = std::vector<int>());
  
}

namespace surface 
{
/**
 * Surface normals estimation
 */
template <typename T>
class ZAdaptiveNormals
{
public:
  class Parameter
  {
    public:
      double radius;            // euclidean inlier radius
      int kernel;               // kernel radius [px]
      bool adaptive;            // Activate z-adaptive normals calcualation
      float kappa;              // gradient
      float d;                  // constant
      float kernel_radius[8];   // Kernel radius for each 0.5 meter intervall (0-4m)
      Parameter(double _radius=0.02, int _kernel=5, bool _adaptive=false, float _kappa=0.005125, float _d = 0.0)
       : radius(_radius), kernel(_kernel), adaptive(_adaptive), kappa(_kappa), d(_d) {}
  };

private:
  Parameter param;

  float NaN;
  int width, height;

  float sqr_radius;
  
  cv::Mat mask;

  typename pcl::PointCloud<T>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  void estimateNormals();
  void getIndices(int u, int v, int kernel, std::vector<int> &indices) const;
  float computeNormal(const std::vector<int> &indices, Eigen::Matrix3f &eigen_vectors) const;


  inline int getIdx(short x, short y) const;
  inline short X(int idx) const;
  inline short Y(int idx) const;
  inline bool checkNotNaN(const T &p) const;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ZAdaptiveNormals(Parameter p=Parameter());
  ~ZAdaptiveNormals();

  void setParameter(Parameter p);
  void setInputCloud(const typename pcl::PointCloud<T>::Ptr &_cloud);

  void compute();
  void compute(const std::vector<int> &_mask);

  void getNormals(pcl::PointCloud<pcl::Normal> &_normals);
  // for compatibility
  void getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals);
  // print normals
  void print(std::string file_name);
};




}//namespace surface 

#endif

