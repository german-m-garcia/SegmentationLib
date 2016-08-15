/*
 * Gestalts.cpp
 *
 *  Created on: 20 Jul 2016
 *      Author: martin
 */

#include "contours/Gestalts.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

Gestalts::Gestalts() {
	// TODO Auto-generated constructor stub

}

Gestalts::~Gestalts() {
	// TODO Auto-generated destructor stub
}


double Gestalts::symmetry(Contour& c1, Contour&c2){
	std::vector<cv::Point2i> points1 = sort_contour_points(c1);
	std::vector<cv::Point2i> points2 = sort_contour_points(c2);
	
	if(check_symmetry(points1, points2))
		return 1.;

	std::vector<cv::Point2i> reversedPoints2;
	for(int i=points2.size()-1; i>= 0; i--)
		reversedPoints2.push_back(points2[i]);

	if(check_symmetry(points1, reversedPoints2))
		return 1.;

	return 0.;
}

double Gestalts::similarity(Contour& c1, Contour&c2){

	double sim = cv::matchShapes(c1.points, c2.points, CV_CONTOURS_MATCH_I1, 0);

	return sim;
}

double Gestalts::parallelity(Contour& c1, Contour&c2){
	bool parallelity = false;
	std::vector<cv::Point2i> points1 = sort_contour_points(c1);
	std::vector<cv::Point2i> points2 = sort_contour_points(c2);

	double d1 = sqrt( pow(points1[0].x - points2[0].x,2) + pow(points1[0].y - points2[0].y,2) );
	double d2 = sqrt( pow(points1[0].x - points2[points2.size()-1].x,2) + pow(points1[0].y - points2[points2.size()-1].y,2) );

	if(d1 < d2)
		parallelity = check_parallelity(points1, points2);
	else{
		std::vector<cv::Point2i> reversedPoints2;
		for(int i=points2.size()-1; i>= 0; i--)
			reversedPoints2.push_back(points2[i]);
		parallelity = check_parallelity(points1, reversedPoints2);
	}

	if(parallelity)
		return 1.;

	return 0.;
}

//return the contour points sorted spatially from one endpoint to the other one
std::vector<cv::Point2i> Gestalts::sort_contour_points(Contour& c){
	std::vector<cv::Point2i> points;

	//get an endpoint from the contour (with only one active neighbor)
	cv::Mat tmp_mask = c.mask.clone();
	cv::Point2i endPoint;
	for(int i=0; i< c.points.size(); i++)
	{
		int nActive = 0;
		endPoint = c.points[i];
		for(int k=std::max(0, endPoint.y-1); k<=std::min(endPoint.y+1, tmp_mask.rows); k++)
			for(int l=std::max(0, endPoint.x-1); l<=std::min(endPoint.x+1, tmp_mask.cols); l++)
			{
				if(!( (k==endPoint.y)&&(l==endPoint.x) ))
					if(tmp_mask.at<uint8_t>(k,l) > 0)
						nActive++;
			}
		if(nActive == 1)
			break;
	}
	
	//grow the set of points starting from the endpoint
	std::vector<cv::Point2i> notProcessedPoints;
	notProcessedPoints.push_back(endPoint);
	tmp_mask.at<uint8_t>(endPoint.y, endPoint.x) = 0;

	while(notProcessedPoints.size()>0)
	{
		cv::Point2i p = notProcessedPoints.front();
		points.push_back(p);
		for(int k=std::max(0, p.y-1); k<=std::min(p.y+1, tmp_mask.rows); k++)
			for(int l=std::max(0, p.x-1); l<=std::min(p.x+1, tmp_mask.cols); l++)
			{
				if(!( (k==p.y)&&(l==p.x) ))
					if(tmp_mask.at<uint8_t>(k,l) > 0)
					{
						notProcessedPoints.push_back(cv::Point2i(l,k));
						tmp_mask.at<uint8_t>(k,l) = 0;
					}
			}
		notProcessedPoints.erase(notProcessedPoints.begin());
	}

	return points;
}

//check if the midpoints between the two point sets form a line
bool Gestalts::check_symmetry(std::vector<cv::Point2i> points1, std::vector<cv::Point2i> points2){

	std::vector<cv::Point2i> symmetryLine;

	for(int i=0; i< std::min(points1.size(), points2.size()) ; i++)
	{
		symmetryLine.push_back(cv::Point2i( (points1[i].x+points2[i].x)/2, (points1[i].y+points2[i].y)/2));
	}

	double eps=3.0;
	double conf = 0.85;
	int nInliers=0;
	cv::Point2i pt1 = symmetryLine[0];
	cv::Point2i pt2 = symmetryLine[symmetryLine.size()-1];

	for(int i=0; i< symmetryLine.size(); i++)
	{
		cv::Point2i pt3 = symmetryLine[i];
		double distance = fabs((pt2.y-pt1.y)*pt3.x - (pt2.x-pt1.x)*pt3.y + pt2.x*pt1.y -pt2.y*pt1.x )*1.0/sqrt(pow(pt1.y-pt2.y,2)+pow(pt1.x-pt2.x,2));
		if(distance < eps)
			nInliers++;
	}

	if(nInliers >= conf*symmetryLine.size())
		return true;

	return false;
}


bool Gestalts::check_parallelity(std::vector<cv::Point2i> points1, std::vector<cv::Point2i> points2){

	double tolerance = 5.;
	double refDist = sqrt( pow(points1[0].x - points2[0].x,2) + pow(points1[0].y - points2[0].y,2) );

	for(int i=0; i< std::min(points1.size(), points2.size()) ; i++)
	{
		double dist = sqrt( pow(points1[i].x - points2[i].x,2) + pow(points1[i].y - points2[i].y,2) );
		if(fabs(dist - refDist) > tolerance)
			return false;
	}

	return true;
}