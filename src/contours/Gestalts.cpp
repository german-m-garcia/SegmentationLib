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


//return the percentage of inliers for the line of symmetry
double Gestalts::symmetry(Contour& c1, Contour&c2){
	//sort the points from one endpoint to the other one to simplify correspondences matching between the contours
	//the set of correspondences is the set of points with the same index
	std::vector<cv::Point2i> points1 = sort_contour_points(c1);
	std::vector<cv::Point2i> points2 = sort_contour_points(c2);
	
	//check the symmetry for the current point order, and later for the reversed order
	double sym_measure = check_symmetry(points1, points2);

	//reverse one of the points set and then check symmetry
	std::vector<cv::Point2i> reversedPoints2;
	for(int i=points2.size()-1; i>= 0; i--)
		reversedPoints2.push_back(points2[i]);

	double tmp_measure = check_symmetry(points1, reversedPoints2);
	if(tmp_measure > sym_measure)
		sym_measure = tmp_measure;

	return sym_measure;
}

double Gestalts::similarity(Contour& c1, Contour&c2){
	//returns similarity measure based on Hu invariants,, smaller value(near zero) means higher similarity 
	double sim = cv::matchShapes(c1.points, c2.points, CV_CONTOURS_MATCH_I1, 0);

	return sim;
}


double Gestalts::parallelity(Contour& c1, Contour&c2){
	double parallelity = 0;
	std::vector<cv::Point2i> points1 = sort_contour_points(c1);
	std::vector<cv::Point2i> points2 = sort_contour_points(c2);

	//compute the distance of the start point of the first contour to the start and end points of the second
	//then use the minimum distance to decide if reversing the points is needed
	//(the points of 2nd contour are reversed if the distance between start and end point is less than the distance between the two strat points) 
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

	return parallelity;
}


double Gestalts::continuity(Contour& c1, Contour&c2){
	
	int margin = 10;
	if(c1.points.size() < margin || c2.points.size() < margin)
		return 0.;

	std::vector<cv::Point2i> points1 = sort_contour_points(c1);
	std::vector<cv::Point2i> points2 = sort_contour_points(c2);

	double dist[4];
	dist[0] = sqrt(pow(points1[0].x - points2[0].x,2) + pow(points1[0].y - points2[0].y,2) );
	dist[1] = sqrt(pow(points1[0].x - points2[points2.size()-1].x,2) + pow(points1[0].y - points2[points2.size()-1].y,2) );
	dist[2] = sqrt(pow(points1[points1.size()-1].x - points2[0].x,2) + pow(points1[points1.size()-1].y - points2[0].y,2) );
	dist[3] = sqrt(pow(points1[points1.size()-1].x - points2[points2.size()-1].x,2) + pow(points1[points1.size()-1].y - points2[points2.size()-1].y,2));

	double minDist = 99999.;
	int minIndx = 0;
	for(int i=0; i<4; i++)
	{
		if(dist[i] < minDist)
		{
			minDist = dist[i];
			minIndx = i;
		}
	}
	cv::Point2i p1A, p1B, p2A, p2B;
	switch(minIndx){
		case 0: p1A = points1[0];
				p2A = points2[0];
				p1B = points1[margin];
				p2B = points2[margin];
				break;
		case 1: p1A = points1[0];
				p2A = points2[points2.size()-1];
				p1B = points1[margin];
				p2B = points2[points2.size()-margin-1];
				break;
		case 2: p1A = points1[points1.size()-1];
				p2A = points2[0];
				p1B = points1[points1.size()-margin-1];
				p2B = points2[margin];
				break;
		case 3: p1A = points1[points1.size()-1];
				p2A = points2[points2.size()-1];
				p1B = points1[points1.size()-margin-1];
				p2B = points2[points2.size()-margin-1];
				break;
	}

	double d_1B1A = sqrt( pow(p1B.x - p1A.x,2) + pow(p1B.y - p1A.y,2) );
	double d_1B2A = sqrt( pow(p1B.x - p2A.x,2) + pow(p1B.y - p2A.y,2) );
	double d_2B1A = sqrt( pow(p2B.x - p1A.x,2) + pow(p2B.y - p1A.y,2) );
	double d_2B2A = sqrt( pow(p2B.x - p2A.x,2) + pow(p2B.y - p2A.y,2) );
	double d_1B2B = sqrt( pow(p1B.x - p2B.x,2) + pow(p1B.y - p2B.y,2) );

	double angle12 = acos( (d_1B1A*d_1B1A + d_2B1A*d_2B1A - d_1B2B*d_1B2B)/(2*d_1B1A*d_2B1A) ); //law of cosines
	double angle21 = acos( (d_1B2A*d_1B2A + d_2B2A*d_2B2A - d_1B2B*d_1B2B)/(2*d_1B2A*d_2B2A) );

	double refAngle = 135.*M_PI/180.; 
	if(angle12 > refAngle && angle21 > refAngle)
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
double Gestalts::check_symmetry(std::vector<cv::Point2i> points1, std::vector<cv::Point2i> points2){

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

	//std::cout<<"sym measure: "<<nInliers*1.0/symmetryLine.size()<<std::endl;
	//if(nInliers >= conf*symmetryLine.size())
	//	return true;

	//return false;

	return nInliers*1.0/symmetryLine.size();
}

//return the percentage of points that agrees with the avg distance between the correspondences
double Gestalts::check_parallelity(std::vector<cv::Point2i> points1, std::vector<cv::Point2i> points2){

	double tolerance = 5.;
	double avgDist = 0;
	std::vector<double> distances;

	//compute the avg distance between the two contours
	for(int i=0; i< std::min(points1.size(), points2.size()) ; i++)
	{
		double dist = sqrt( pow(points1[i].x - points2[i].x,2) + pow(points1[i].y - points2[i].y,2) );
		distances.push_back(dist);
		avgDist += dist;
	}

	avgDist = avgDist/distances.size();
	int nInliers = 0;
	//compute the number of points that agree with the avg distance
	for(int i=0; i< distances.size() ; i++)
	{
		if(fabs(distances[i] - avgDist) < tolerance)
			nInliers++;
	}

	return nInliers*1.0/distances.size();
}