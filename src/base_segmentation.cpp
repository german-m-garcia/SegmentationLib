/*
 * SegmenterLib.cpp
 *
 *  Created on: Oct 2, 2015
 *      Author: gmartin
 */

#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/connected_components.hpp>

#include "cluster_normals_to_planes.h"
#include "pcl_segmentation.h"
#include "surface_cluster.h"

BaseSegmentation::BaseSegmentation() :
		mapLabels(SEGMENTS) {
	// TODO Auto-generated constructor stub

	//fill in the labels
	cv::RNG rng(0xFFFFF);

	for (int i = 0; i < SEGMENTS; i++) {

		int icolor = rng.next();
		mapLabels[i] = cv::Vec3b(icolor & 255, (icolor >> 8) & 255,
				(icolor >> 16) & 255);
	}
	segments.reserve(SEGMENTS);

	//check if there are two equal labels
//	for (int i = 0; i < 1000; i++) {
//		for (int j = 0; j < 1000; j++) {
//			if(i != j){
//				if(mapLabels[i] == mapLabels[j]){
//					cout <<"labels "<<i<<" and "<<j<<" are equal ="<<mapLabels[i]<<endl;
//				}
//			}
//		}
//	}

}

BaseSegmentation::~BaseSegmentation() {
	// TODO Auto-generated destructor stub
	for (unsigned int i = 0; i < segments.size(); i++)
		delete segments[i];
}

void BaseSegmentation::clean_data() {
	for (unsigned int i = 0; i < segments.size(); i++)
		delete segments[i];
	segments.resize(0);
	segments.reserve(SEGMENTS);
}

Segment* BaseSegmentation::get_segment_at_fast(int row, int col) {

	int id = (int) component_id.at<uint16_t>(row, col);
	//cout <<"id="<<id<<endl;
	if (id == 0)
		return nullptr;
	return mapSegments[id];
	//return segments[id];

}

// show resulting segmentation to png image
void BaseSegmentation::show_patches(cv::Mat &kImage,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pcl_cloud,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& labeled_cloud) {
// create color image
	kImage = cv::Mat_<cv::Vec3b>::zeros(pcl_cloud->height, pcl_cloud->width);

	register int centerX = (pcl_cloud->width >> 1);
	int centerY = (pcl_cloud->height >> 1);
	register int depth_idx = 0;
	for (int v = -centerY; v < centerY; ++v) {
		for (register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			//cout <<" accessing u,v="<<u<<v<<endl;
			//pcl::PointXYZRGBA& pt = pcl_cloud->points[depth_idx];
			pcl::PointXYZRGBA& pt_label = labeled_cloud->points[depth_idx];

			cv::Vec3b &pixel = kImage.at<cv::Vec3b>(v + centerY, u + centerX);
			pixel[0] = pt_label.r;
			pixel[2] = pt_label.g;
			pixel[1] = pt_label.b;
		}
	}
}

// show resulting segmentation to png image
void BaseSegmentation::show_patches(cv::Mat &kImage,
		pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud) {
// create color image
	kImage = cv::Mat_<cv::Vec3b>::zeros(labeled_cloud->height,
			labeled_cloud->width);

	register int centerX = (labeled_cloud->width >> 1);
	int centerY = (labeled_cloud->height >> 1);
	register int depth_idx = 0;
	for (int v = -centerY; v < centerY; ++v) {
		for (register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			//cout <<" accessing u,v="<<u<<v<<endl;
			//pcl::PointXYZRGBA& pt = pcl_cloud->points[depth_idx];
			pcl::PointXYZL& pt_label = labeled_cloud->points[depth_idx];

			cv::Vec3b &pixel = kImage.at<cv::Vec3b>(v + centerY, u + centerX);

			cv::Vec3b& labInMap = mapLabels[pt_label.label];
			pixel[0] = labInMap[0];
			pixel[1] = labInMap[1];
			pixel[2] = labInMap[2];
		}
	}
}



int BaseSegmentation::get_label(map<uint32_t, int>& labels, int label) {
	map<uint32_t, int>::iterator it = labels.find(label);
//the label exists
	if (it != labels.end()) {
		//element found;
		return it->second;
	}
//the label is new
	else {
		return -1;
	}
}

void BaseSegmentation::read_segments(Mat& original, Mat& segmentation_mat) {

	component_id = cv::Mat::zeros(segmentation_mat.size(), CV_16UC1);

	map<uint32_t, int> labels;
	int labelcount = 0;

	Mat hsv;
	cv::cvtColor(original, hsv, CV_BGR2HSV);

	//go through every pixel
	for (int i = 0; i < segmentation_mat.rows; i++) {
		for (int j = 0; j < segmentation_mat.cols; j++) {
			cv::Vec3b& label = segmentation_mat.at<cv::Vec3b>(i, j);
			cv::Vec3b& colour = original.at<cv::Vec3b>(i, j);
			uint32_t intLabel = label[2] * 1000000 + label[1] * 1000 + label[0];
			int componentId = get_label(labels, intLabel);
			if (componentId == -1) {
				//the component is new
				labels[intLabel] = labelcount;
				componentId = labels[intLabel];
				labelcount++;
				//cout <<"--------------------- new component id: "<<labels[intLabel]<<" of colour: "<<intLabel<<endl;

				component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
				//cout << (int)component_id.at<uint16_t>(i,j)<<" ";
				Segment* segment = new Segment(hsv);
				cv::Point2i point(i, j);
				segment->addPoint(point, colour);
				segments.push_back(segment);
				mapSegments[componentId] = segment;

			} else {
				//the component exists
				//cout <<" component id: "<<componentId<<" of colour: "<<intLabel<<endl;
				component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
				//cout << (int)component_id.at<uint16_t>(i,j)<<" ";
				cv::Point2i point(i, j);
				mapSegments[componentId]->addPoint(point, colour);

			}

		}

	}
	for (Segment* segment : segments)
		segment->computeFeatures();

}

void BaseSegmentation::read_segments(const Mat& original, Mat& segmentation_mat, Mat& dst) {

	component_id = cv::Mat::zeros(segmentation_mat.size(), CV_16UC1);
	dst = cv::Mat::zeros(segmentation_mat.size(), CV_8UC3);

	map<uint32_t, int> labels;
	int labelcount = 0;

	Mat hsv;
	cv::cvtColor(original, hsv, CV_BGR2HSV);

	//go through every pixel
	for (int i = 0; i < segmentation_mat.rows; i++) {
		for (int j = 0; j < segmentation_mat.cols; j++) {
			cv::Vec3b& label = segmentation_mat.at<cv::Vec3b>(i, j);
			const cv::Vec3b& colour = original.at<cv::Vec3b>(i, j);
			uint32_t intLabel = label[2] * 1000000 + label[1] * 1000 + label[0];
			int componentId = get_label(labels, intLabel);
			if (componentId == -1) {
				//the component is new
				labels[intLabel] = labelcount;
				componentId = labels[intLabel];
				labelcount++;
				//cout <<"--------------------- new component id: "<<labels[intLabel]<<" of colour: "<<intLabel<<endl;

				component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
				//cout << (int)component_id.at<uint16_t>(i,j)<<" ";
				Segment* segment = new Segment(hsv);
				const cv::Point2i point(i, j);
				segment->addPoint(point, colour);
				segments.push_back(segment);
				mapSegments[componentId] = segment;

			} else {
				//the component exists
				//cout <<" component id: "<<componentId<<" of colour: "<<intLabel<<endl;
				component_id.at<uint16_t>(i, j) = (uint16_t) componentId;
				//cout << (int)component_id.at<uint16_t>(i,j)<<" ";
				const cv::Point2i point(i, j);
				mapSegments[componentId]->addPoint(point, colour);

			}

		}

	}
	cout <<">every pixel iterated."<<endl;
	for (Segment* segment : segments){
		//create random colour
		segment->colour_this_segment(dst);
	}

	//for (Segment* segment : segments)
	//	segment->computeFeatures();

}

void BaseSegmentation::read_segments(Mat& img,
		std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>& supervoxel_clusters) {

	cout << "SegmenterLib::readSegments > supervoxel_clusters.size()="
			<< supervoxel_clusters.size() << endl;

}

/*
 *
 * MEANSHIFT
 *
 *
 */

//This colors the segmentations
void BaseSegmentation::flood_fill_post_process(Mat& img, const Scalar& colorDiff ) {
	CV_Assert(!img.empty());
	RNG rng = theRNG();
	Mat mask(img.rows + 2, img.cols + 2, CV_8UC1, Scalar::all(0));
	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {
			if (mask.at<uchar>(y + 1, x + 1) == 0) {
				Scalar newVal(rng(256), rng(256), rng(256));
				floodFill(img, mask, Point(x, y), newVal, 0, colorDiff,
						colorDiff);
			}
		}
	}
}


