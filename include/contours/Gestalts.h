/*
 * Gestalts.h
 *
 *  Created on: 20 Jul 2016
 *      Author: martin
 */

#ifndef SRC_CONTOURS_GESTALTS_H_
#define SRC_CONTOURS_GESTALTS_H_

#include "Contours.h"

class Gestalts {
public:
	Gestalts();
	virtual ~Gestalts();
	double symmetry(Contour& c1, Contour&c2);
	double similarity(Contour& c1, Contour&c2);
	double parallelity(Contour& c1, Contour&c2);
private:
	std::vector<cv::Point2i> sort_contour_points(Contour& c);
	bool check_symmetry(std::vector<cv::Point2i> points1, std::vector<cv::Point2i> points2);
	bool check_parallelity(std::vector<cv::Point2i> points1, std::vector<cv::Point2i> points2);
};

#endif /* SRC_CONTOURS_GESTALTS_H_ */
