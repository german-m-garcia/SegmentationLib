/*
 * Contours.h
 *
 *  Created on: 8 Jul 2016
 *      Author: martin
 */

#ifndef SRC_CONTOURS_CONTOURS_H_
#define SRC_CONTOURS_CONTOURS_H_

 #define active(src, p)  (src.at<uint8_t>(p.y, p.x) > 0)

#include <opencv2/core/core.hpp>

struct Contour {


	std::vector<cv::Point2i> points;
	cv::Mat mask;
	cv::Vec3b colour;


};

class Contours {
public:
	Contours();
	virtual ~Contours();

	void compute_edges(cv::Mat& src, cv::Mat& dst);


	void trace_contours(const cv::Mat& original_img,  cv::Mat& src);
	cv::Mat visu_orientation_map(const cv::Mat& mag, const cv::Mat& ori, double thresh = 1.0);
	void harris(cv::Mat& edges);
	void display_contours(std::vector<Contour>& contours);




private:

	void orientation(const cv::Mat& src, cv::Mat& ori, cv::Mat& mag);
	bool trace_contour(const cv::Mat& src, cv::Mat& visited , cv::Mat& mask, cv::Mat& display,Contour& contour);
	std::vector<cv::Point2i> get_active_non_visited_neighbours(const cv::Mat&src, cv::Mat& visited, cv::Point&p);
	int neighbours(const cv::Mat& src, cv::Point& p);
	std::vector<cv::Point2i> get_neighbour_points(const cv::Mat& src, cv::Point&p);
	bool junction(const cv::Mat& src, cv::Point2i& p);
	void clear_junctions(cv::Mat& edges);
	void remove_noisy_contours(std::vector<Contour>& contours);


	static cv::RNG rng;
	cv::Mat orientation_mat;
	int minimum_size_;



};

#endif /* SRC_CONTOURS_CONTOURS_H_ */
