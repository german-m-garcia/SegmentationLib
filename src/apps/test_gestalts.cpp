
#include "contours/Gestalts.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;


int main(int argc, char ** argv) {

	Gestalts g;
	Contour c1,c2;

	if(argc != 3){
		std::cout <<"$> gestalts <img_1> <img_2>"<<std::endl;
		return -1;
	}

	c1.mask = cv::imread(argv[1]);
	cv::cvtColor( c1.mask, c1.mask, CV_BGR2GRAY );
	for(int i=0; i< c1.mask.rows; i++)
		for(int j=0; j< c1.mask.cols; j++)
		{
			if(c1.mask.at<uint8_t>(i,j)>0)
				c1.points.push_back(cv::Point2i(j,i));
		}

	c2.mask = cv::imread(argv[2]);
	cv::cvtColor( c2.mask, c2.mask, CV_BGR2GRAY );
	for(int i=0; i< c2.mask.rows; i++)
		for(int j=0; j< c2.mask.cols; j++)
		{
			if(c2.mask.at<uint8_t>(i,j)>0)
				c2.points.push_back(cv::Point2i(j,i));
		}

	if(g.symmetry(c1,c2))
		cout<<"Symmetric ... "<<endl;
	else
		cout<<"Not Symmetric ... "<<endl;

	if(g.parallelity(c1,c2))
		cout<<"Parallel ... "<<endl;
	else
		cout<<"Not Parallel ... "<<endl;

	std::cout<<"similarity measure = "<<g.similarity(c1,c2)<<std::endl;;

	return 0;


}
