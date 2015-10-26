/*
 * SegmentationReader.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: martin
 */

#include "segmentation_reader.h"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

Segment::Segment()
    : points(0) {
}

Segment::Segment(cv::Size size, cv::Point2i centre, cv::Vec3b colour)
    : centre(centre),
      maxX(centre),
      points(1),
      contourPoints(0),
      colour(colour),
      minCol(centre.y),
      minRow(centre.x),
      maxCol(centre.y),
      maxRow(centre.x) {
  img_ = cv::Mat::zeros(size, CV_8UC3);
  contourMat3C = cv::Mat::zeros(size, CV_8UC3);
  contourMat = cv::Mat::zeros(size, CV_8UC1);
  img_.at<cv::Vec3b>(centre.x, centre.y) = colour;

}

void Segment::pointFromDepth(uint16_t depthvalue, cv::Vec3b colour,
                             cv::Point2i point2D, pcl::PointXYZRGB& point3D) {

  register float constant = 1.0f / 525;
  register int centerX = (pcl_cloud->width >> 1);
  int centerY = (pcl_cloud->height >> 1);
  //uint16_t depthvalue = depth.at<uint16_t>(v+centerY,u+centerX);
  point3D.z = depthvalue * 0.001f;  //depth_data[depth_idx] * 0.001f;
  //Point2i(row,col) centre is created like this. pt.x depends on u coordinate (column)
  point3D.y = -static_cast<float>(point2D.x) * point3D.z * constant;
  //pt.y depends on v coordinate (row)
  point3D.x = static_cast<float>(point2D.y) * point3D.z * constant;
  point3D.r = colour[0];
  point3D.b = colour[1];
  point3D.g = colour[2];

}

Segment::Segment(cv::Size size, cv::Point2i centre, uint16_t depthvalue,
                 cv::Vec3b colour)
    : centre(centre),
      maxX(centre),
      points(1),
      contourPoints(0),
      colour(colour),
      minCol(centre.y),
      minRow(centre.x),
      maxCol(centre.y),
      maxRow(centre.x) {
  img_ = cv::Mat::zeros(size, CV_8UC3);
  contourMat3C = cv::Mat::zeros(size, CV_8UC3);
  contourMat = cv::Mat::zeros(size, CV_8UC1);
  img_.at<cv::Vec3b>(centre.x, centre.y) = colour;

  pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_cloud->width = img_.cols;
  pcl_cloud->height = img_.rows;
  pcl::PointXYZRGB pt;
  pointFromDepth(depthvalue, colour, centre, pt);
  pcl_cloud->push_back(pt);
}

cv::Mat& Segment::getSymmetryMat() {
  return contourMat3C;
}

void Segment::contourize() {
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  /// Find contours
  cv::Mat gray;
  cv::cvtColor(img_, gray, CV_BGR2GRAY);
  cv::findContours(gray, contours, hierarchy, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

  /// Draw contours

  for (int i = 0; i < contours.size(); i++) {
    cv::Scalar color = cv::Scalar(255, 255, 255);
    cv::drawContours(contourMat, contours, i, color, 2, 8, hierarchy, 0,
                     cv::Point());
  }
  cv::dilate(contourMat, contourMat, cv::Mat());
  cv::Scalar nonzero = countNonZero(contourMat);
  contourPoints = nonzero[0];
  cv::cvtColor(contourMat, contourMat3C, CV_GRAY2BGR);

  //imshow("contourMat3C",contourMat3C);
  if (NORMALS) {

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation < pcl::PointXYZRGB, pcl::Normal > ne;
    cout << "setting input cloud..." << endl;
    ne.setInputCloud(pcl_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.10);

    // Compute the features
    cout << "computing normals..." << endl;
    ne.compute(*cloud_normals);
    pcl::Normal avg_normal = cloud_normals->points[0];
    pcl::PointXYZRGB avg_point = pcl_cloud->points[0];
    for (int i = 1; i < cloud_normals->points.size(); i++) {
      pcl::Normal& normal = cloud_normals->points[i];
      pcl::PointXYZRGB point = pcl_cloud->points[i];
      avg_normal.normal_x += normal.normal_x;
      avg_normal.normal_y += normal.normal_y;
      avg_normal.normal_z += normal.normal_z;

      avg_point.x += point.x;
      avg_point.y += point.y;
      avg_point.z += point.z;

    }
    avg_normal.normal_x /= cloud_normals->points.size();
    avg_normal.normal_y /= cloud_normals->points.size();
    avg_normal.normal_z /= cloud_normals->points.size();

    avg_point.x /= pcl_cloud->points.size();
    avg_point.y /= pcl_cloud->points.size();
    avg_point.z /= pcl_cloud->points.size();

    pcl::PointCloud<pcl::Normal>::Ptr one_normal(
        new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr one_point(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    one_normal->push_back(avg_normal);
    one_point->push_back(avg_point);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(one_point,
                                                               one_normal, 1,
                                                               0.15, "normals");
    viewer.addPointCloud(pcl_cloud);

    while (!viewer.wasStopped()) {
      viewer.spinOnce();
    }
  }

}

int Segment::pixels() {
  return contourPoints;
}

void Segment::addMaxX(cv::Point2i& p) {
  if (p.y > maxX.y) {
    maxX.x = p.x;
    maxX.y = p.y;
  }

}

void Segment::addPoint(cv::Point2i& p) {
  centre += p;
  centre.x /= 2;
  centre.y /= 2;
  points++;
  img_.at<cv::Vec3b>(p.x, p.y) = colour;
  if (p.x < minRow)
    minRow = p.x;
  if (p.y < minCol)
    minCol = p.y;
  if (p.x > maxRow)
    maxRow = p.x;
  if (p.y > maxCol)
    maxCol = p.y;

}

void Segment::addPoint3D(cv::Point2i& p, cv::Vec3b colour,
                         uint16_t depthvalue) {
  centre += p;
  centre.x /= 2;
  centre.y /= 2;
  points++;
  img_.at<cv::Vec3b>(p.x, p.y) = colour;
  if (p.x < minRow)
    minRow = p.x;
  if (p.y < minCol)
    minCol = p.y;
  if (p.x > maxRow)
    maxRow = p.x;
  if (p.y > maxCol)
    maxCol = p.y;

  pcl::PointXYZRGB pt;
  pointFromDepth(depthvalue, colour, p, pt);
  pcl_cloud->push_back(pt);

}

SegmentationReader::SegmentationReader() {
  // TODO Auto-generated constructor stub

}

SegmentationReader::~SegmentationReader() {
  cleanData();
  for (int i = 0; i < segments.size(); i++) {
    delete segments[i];
  }
}

std::set<Component*>&
SegmentationReader::getComponents() {
  return components_set;
}

Component* SegmentationReader::getComponentAt(int x, int y) {

}

int SegmentationReader::getLabel(map<uint32_t, int>& labels, int label) {
  map<uint32_t, int>::iterator it = labels.find(label);
  //the label exists
  if (it != labels.end()) {
    //element found;
    return it->second;
  }
  //the label is new
  else {
//    labels[label] = labelcount++;
//    return labels[label];
    return -1;
  }
}

Component* SegmentationReader::getComponentAt_fast(int x, int y) {
  int id = (int) component_id.at<uint16_t>(x, y);
  if (id == 0)
    return nullptr;
  return components[id];
}

void SegmentationReader::cleanData() {
  std::vector<Component*>::iterator it;
  for (it = components.begin(); it != components.end(); ++it) {
    std::set<Vertex*>::iterator itV;
    for (itV = (*it)->getVertices().begin(); itV != (*it)->getVertices().end();
        ++itV) {
      if (*itV) {
        delete *itV;
      }

    }
    delete (*it);
  }
  components.clear();

}

void SegmentationReader::readSegmentation(cv::Mat& img, cv::Mat& depth) {

  component_id = cv::Mat::zeros(img.size(), CV_16UC1);
  map<uint32_t, int> labels;
  int labelcount = 0;

  //go through every pixel
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      cv::Vec3b& label = img.at<cv::Vec3b>(i, j);
      if (label[0] == 0 && label[1] == 0 && label[2] == 0) {
        component_id.at<uint16_t>(i, j) = 0;
        continue;
      }

      uint16_t depthvalue = depth.at<uint16_t>(i, j);
      uint32_t intLabel = label[2] * 1000000 + label[1] * 1000 + label[0];
      int componentId = getLabel(labels, intLabel);
      if (componentId == -1) {
        //the component is new
        labels[intLabel] = labelcount;
        componentId = labels[intLabel];
        labelcount++;
        //cout <<"--------------------- new component id: "<<labels[intLabel]<<" of colour: "<<intLabel<<endl;

        component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
        //cout << (int)component_id.at<uint16_t>(i,j)<<" ";
        Component* component = new Component();
        cv::Point2i point(i, j);
        Vertex* vertex = new Vertex(0, point);
        vertex->setColour(label);
        component->addVertice(vertex);
        components.push_back(component);
        components_set.insert(component);
        //cout <<" finished with new component"<<endl;

        //create new Segment object
        Segment* segment = new Segment(img.size(), point, depthvalue, label);
        segments.push_back(segment);

      } else {
        //the component exists
        //cout <<" component id: "<<componentId<<" of colour: "<<intLabel<<endl;
        component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
        //cout << (int)component_id.at<uint16_t>(i,j)<<" ";
        cv::Point2i point(i, j);
        Vertex* vertex = new Vertex(0, point);
        vertex->setColour(label);
        components[componentId]->addVertice(vertex);
        segments[componentId]->addPoint3D(point, label, depthvalue);
        segments[componentId]->addMaxX(point);
      }

    }

  }
  for (int i = 0; i < segments.size(); i++) {
    segments[i]->contourize();
  }

}

void SegmentationReader::readSegmentation(cv::Mat& img) {

  component_id = cv::Mat::zeros(img.size(), CV_16UC1);
  map<uint32_t, int> labels;
  int labelcount = 0;

  //go through every pixel
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      cv::Vec3b& label = img.at<cv::Vec3b>(i, j);
      uint32_t intLabel = label[2] * 1000000 + label[1] * 1000 + label[0];
      int componentId = getLabel(labels, intLabel);
      if (componentId == -1) {
        //the component is new
        labels[intLabel] = labelcount;
        componentId = labels[intLabel];
        labelcount++;
        //cout <<"--------------------- new component id: "<<labels[intLabel]<<" of colour: "<<intLabel<<endl;

        component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
        //cout << (int)component_id.at<uint16_t>(i,j)<<" ";
        Component* component = new Component();
        cv::Point2i point(i, j);
        Vertex* vertex = new Vertex(0, point);
        vertex->setColour(label);
        component->addVertice(vertex);
        components.push_back(component);
        components_set.insert(component);
        //cout <<" finished with new component"<<endl;

        //create new Segment object
        Segment* segment = new Segment(img.size(), point, label);
        segments.push_back(segment);

      } else {
        //the component exists
        //cout <<" component id: "<<componentId<<" of colour: "<<intLabel<<endl;
        component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
        //cout << (int)component_id.at<uint16_t>(i,j)<<" ";
        cv::Point2i point(i, j);
        Vertex* vertex = new Vertex(0, point);
        vertex->setColour(label);
        components[componentId]->addVertice(vertex);
        segments[componentId]->addPoint(point);
        segments[componentId]->addMaxX(point);
      }

    }

  }
  for (int i = 0; i < segments.size(); i++) {
    segments[i]->contourize();
  }

}
