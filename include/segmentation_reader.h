/*
 * SegmentationReader.h
 *
 *  Created on: Jul 15, 2014
 *      Author: martin
 */

#ifndef SEGMENTATIONREADER_H_
#define SEGMENTATIONREADER_H_

#include <opencv2/opencv.hpp>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "segment.h"

using namespace std;

//compute the normals of each patch
#define NORMALS false

class Segment {
 private:

  cv::Point2i centre,maxX;
  int points, contourPoints;
  cv::Vec3b colour;
  int minCol,minRow,maxCol,maxRow;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;

  void
   pointFromDepth(uint16_t depthvalue,cv::Vec3b colour,cv::Point2i point2D,pcl::PointXYZRGB& point3D);

 public:
  cv::Mat mat_original_colour_,contourMat,contourMat3C;


  Segment();
  Segment(cv::Size size,cv::Point2i centre,cv::Vec3b colour);
  Segment(cv::Size size,cv::Point2i centre,uint16_t depth,cv::Vec3b colour);

  cv::Mat& getSymmetryMat();

  void contourize();

  int pixels();

  void addMaxX(cv::Point2i& p);

  void addPoint(const cv::Point2i& p);
  void addPoint3D(cv::Point2i& p,cv::Vec3b colour,uint16_t depthvalue);

  cv::Point2i& getCentre(){
    return centre;
  }

  cv::Point2i getBariCentre(){
      return cv::Point2i((minRow+maxRow)/2,(minCol+maxCol)/2);
    }

  cv::Point2i& getMaxX(){
      return maxX;
    }

};

class SegmentationReader : public Segmentation{


 public:
  SegmentationReader();
  virtual ~SegmentationReader();

  Component* getComponentAt_fast(int x, int y) ;

  Component* getComponentAt(int x, int y) ;


  void readSegmentation(cv::Mat& img);

  void readSegmentation(cv::Mat& img, cv::Mat& depth);



  std::set<Component*>&
  getComponents();

  void cleanData();

  int getNoSegments(){
    return segments.size();
  }

  Segment* getSegment(int index) {
    if(index < segments.size())
      return segments[index];
    return nullptr;
  }

  vector<Segment*>& getSegments(){
    return segments;
  }

 private:
  cv::Mat component_id;
  vector<Component*> components;
  vector<Segment*> segments;
  set<Component*> components_set;
  int getLabel( map<uint32_t,int>& labels,int label);



};

#endif /* SEGMENTATIONREADER_H_ */
