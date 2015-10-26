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

#include "z_adaptive_normals.h"

namespace EPUtils
{
  
template <class T>
bool computeMean(const typename pcl::PointCloud<T> &cloud, Eigen::Vector3f &mean,
                 std::vector<int> indices = std::vector<int>())
{
  if(indices.size() == 0)
  {
    indices.reserve(cloud.size());
    for(unsigned int i = 0; i < cloud.size(); ++i)
    {
      if(std::isnan(cloud.points.at(i).x) ||
         std::isnan(cloud.points.at(i).y) ||
         std::isnan(cloud.points.at(i).z))
      {
        continue;
      }
      else
      {
        indices.push_back(i);
      }
    }
  }

  if(indices.size() == 0)
    return(false);
  
  mean.setZero();
  for (unsigned i=0; i < indices.size(); ++i)
  {
    int idx = indices.at(i);
    mean[0] += cloud.points.at(idx).x;
    mean[1] += cloud.points.at(idx).y;
    mean[2] += cloud.points.at(idx).z;
  }

  mean /= (float)indices.size();

  return(true);
}

template <class T>
bool computeCovarianceMatrix(const typename pcl::PointCloud<T> &cloud, const Eigen::Vector3f &mean,
                             Eigen::Matrix3f &cov, std::vector<int> indices = std::vector<int>())
{
  if(indices.size() == 0)
  {
    indices.reserve(cloud.size());
    for(unsigned int i = 0; i < cloud.size(); ++i)
    {
      if(std::isnan(cloud.points.at(i).x) ||
         std::isnan(cloud.points.at(i).y) ||
         std::isnan(cloud.points.at(i).z))
      {
        continue;
      }
      else
      {
        indices.push_back(i);
      }
    }
  }
  
  bool done = false;
  cov.setZero ();
  
  for (unsigned pi = 0; pi < indices.size (); ++pi)
  {
    float x = cloud.points.at(indices.at(pi)).x - mean[0];
    float y = cloud.points.at(indices.at(pi)).y - mean[1];
    float z = cloud.points.at(indices.at(pi)).z - mean[2];
    
    cov(0,0) += x*x;
    cov(0,1) += x*y;
    cov(0,2) += x*z;
    
    cov(1,0) += y*x;
    cov(1,1) += y*y;
    cov(1,2) += y*z;
    
    cov(2,0) += z*x;
    cov(2,1) += z*y;
    cov(2,2) += z*z;
    
    done = true;
  }
  
  return(done);
}
  
}

namespace surface 
{


/*********************** INLINE METHODES **************************/

template <typename T>
inline int ZAdaptiveNormals<T>::getIdx(short x, short y) const
{
  return y*width+x;
}

template <typename T>
inline short ZAdaptiveNormals<T>::X(int idx) const
{
  return idx%width;
}

template <typename T>
inline short ZAdaptiveNormals<T>::Y(int idx) const
{
  return idx/width;
}

// return true of point is not NaN
template <typename T>
inline bool ZAdaptiveNormals<T>::checkNotNaN(const T &p) const
{
  if(std::isnan(p.x) ||
     std::isnan(p.y) ||
     std::isnan(p.z))
  {
    return(false);
  }
  return(true);
}

/********************** ZAdaptiveNormals ************************
 * Constructor/Destructor
 */

template <typename T>
ZAdaptiveNormals<T>::ZAdaptiveNormals(Parameter p)
{
  NaN = std::numeric_limits<float>::quiet_NaN();
  
  setParameter(p);
  param.kernel_radius[0] = 3;
  param.kernel_radius[1] = 3;
  param.kernel_radius[2] = 3;
  param.kernel_radius[3] = 3;
  param.kernel_radius[4] = 4;
  param.kernel_radius[5] = 5;
  param.kernel_radius[6] = 6;
  param.kernel_radius[7] = 7;
}

template <typename T>
ZAdaptiveNormals<T>::~ZAdaptiveNormals()
{
}

/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
template <typename T>
void ZAdaptiveNormals<T>::setInputCloud(const typename pcl::PointCloud<T>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error ("[ZAdaptiveNormals::compute] Need an organized point cloud!");
  
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
  
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  normals->points.resize(cloud->points.size());
  normals->width = cloud->width;
  normals->height = cloud->height;
  normals->is_dense = cloud->is_dense;
}

/**
 * compute the normals
 */
template <typename T>
void ZAdaptiveNormals<T>::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[ZAdaptiveNormals::compute] No point cloud available!");
  
  mask = cv::Mat_<int>::ones(height,width);
  
  estimateNormals();
}

/**
 * compute the normals using a mask
 */
template <typename T>
void ZAdaptiveNormals<T>::compute(const std::vector<int> &_mask)
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[ZAdaptiveNormals::compute] No point cloud available!");
  
  mask = cv::Mat_<int>::zeros(height,width);
  for(int i = 0; i < _mask.size(); ++i)
  {
    int idx = _mask.at(i);
    mask.at<int>(Y(idx),X(idx)) = 1;
  }
  
  estimateNormals();
  
}


/**
 * getNormals
 */
template <typename T>
void ZAdaptiveNormals<T>::getNormals(pcl::PointCloud<pcl::Normal> &_normals)
{
  _normals = *normals;
}

template <typename T>
void ZAdaptiveNormals<T>::getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  _normals = normals;
}

/**
 * setParameter
 */
template <typename T>
void ZAdaptiveNormals<T>::setParameter(Parameter p)
{
  param = p;
  sqr_radius = p.radius*p.radius;
}

/************************** PRIVATE ************************/

/**
 * GetIndices of the neigbourhood of the point depending on the kernel size
 */
template <typename T>
void ZAdaptiveNormals<T>::getIndices(int u, int v, int kernel, std::vector<int> &indices) const
{
  indices.clear();
  
  const T &pt = cloud->points.at(getIdx(u,v));
  
  for (int vkernel = -kernel; vkernel <= kernel; ++vkernel)
  {
    for (int ukernel = -kernel; ukernel <= kernel; ++ukernel)
    {
      int y = v + vkernel;
      int x = u + ukernel;
      
      float center_dist = sqrt(vkernel*vkernel + ukernel*ukernel);
      
      if ( (x > 0) && (y > 0) && (x < width) && (y < height))
      {
        int idx = getIdx(x,y);
        const T &pt1 = cloud->points.at(idx);
        
        if(checkNotNaN(pt1))
        {
          float new_sqr_radius = sqr_radius;
          
          if(param.adaptive)
          {
            float val = param.kappa * center_dist * pt1.z + param.d;
            new_sqr_radius = val*val;
          }
          
          if ((pt.getVector3fMap()-pt1.getVector3fMap()).squaredNorm() < new_sqr_radius)
          {
            indices.push_back(idx);
          }
        }
      }
    }
  }
}

/**
 * ComputeNormal
 */
template <typename T>
float ZAdaptiveNormals<T>::computeNormal(const std::vector<int> &indices, Eigen::Matrix3f &eigen_vectors) const
{
  if (indices.size()<4)
    return NaN;
  
  Eigen::Vector3f mean;
  EPUtils::computeMean<T>(*cloud,mean,indices);
  
  Eigen::Matrix3f cov;
  EPUtils::computeCovarianceMatrix<T>(*cloud,mean,cov,indices);
  
  Eigen::Vector3f eigen_values;
  pcl::eigen33 (cov, eigen_vectors, eigen_values);
  float eigsum = eigen_values.sum();
  if (eigsum != 0)
    return fabs (eigen_values[0] / eigsum );
  
  return NaN;
}


/**
 * EstimateNormals
 */
template <typename T>
void ZAdaptiveNormals<T>::estimateNormals()
{
  bool havenan = false;
  
  #pragma omp parallel for shared(havenan)
  for (int v=0; v<height; v++)
  {
    for (int u=0; u<width; u++)
    {
    if(mask.at<int>(v,u) > 0)
    {
      std::vector<int> indices;
      int idx = getIdx(u,v);
      T &pt = cloud->points.at(idx);
      pcl::Normal &n = normals->points.at(idx);
      if(checkNotNaN(pt))
      {
        if(param.adaptive)
        {
          int dist = (int) (pt.z*2); // *2 => every 0.5 meter another kernel radius
	  if(dist > 7)
	    dist = 7;
          getIndices(u,v, param.kernel_radius[dist], indices);
        }
        else
          getIndices(u,v, param.kernel, indices);
      }
        
      if (indices.size()<4)
      {
        #pragma omp critical
        {
          havenan = true;
        }
        n.normal[0] = NaN;
        n.normal[1] = NaN;
        n.normal[2] = NaN;
        pt.x = NaN;
        pt.y = NaN;
        pt.z = NaN;
        continue;
      }
        
      EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
      n.curvature = computeNormal(indices, eigen_vectors);
        
      n.normal[0] = eigen_vectors (0,0);
      n.normal[1] = eigen_vectors (1,0);
      n.normal[2] = eigen_vectors (2,0);
        
      // orient normal to us
      if (n.getNormalVector3fMap().dot(pt.getVector3fMap()) > 0)
      {
        n.getNormalVector4fMap() *= -1;
      }
      // the fourth parameter is to complete hessian form --> d coefficient in the plane
      n.getNormalVector4fMap()[3] = 0;
      n.getNormalVector4fMap()[3] = -1 * n.getNormalVector3fMap().dot(pt.getVector3fMap());
    }
      
    }
  }
  
  if (havenan)
  {
    cloud->is_dense=false;
    normals->is_dense=false;
  }
}

/**
 * Print normals into file
 */
template <typename T>
void ZAdaptiveNormals<T>::print(std::string file_name)
{
  FILE *f = std::fopen(file_name.c_str(), "w");
  for(int i = 0; i < normals->size(); ++i)
  {
    pcl::Normal n = normals->points.at(i);
    fprintf(f,"%d %f %f %f %f \n",i,n.normal[0],n.normal[1],n.normal[2],n.curvature);
  }
  std::fclose(f);
}

}//namespace surface 


