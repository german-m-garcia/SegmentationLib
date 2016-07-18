/*
 * Contours.cpp
 *
 *  Created on: 8 Jul 2016
 *      Author: martin
 */

#include <iostream>
#include "contours/Contours.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


//initialise the Random Number Generator
cv::RNG Contours::rng = cv::RNG();

Contours::Contours() {
	// TODO Auto-generated constructor stub

}

Contours::~Contours() {
	// TODO Auto-generated destructor stub
}

/*
 * src: input image, grayscale 8-bit
 */
void Contours::compute_edges(cv::Mat& src, cv::Mat& dst){

	int edgeThresh = 1;
	int lowThreshold = 15;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;


	/// Canny detector
	 /// Reduce noise with a kernel 3x3
	cv::blur( src, src, cv::Size(3,3) );
	cv::Canny(src, dst, lowThreshold, max_lowThreshold, kernel_size);
	cv::imshow("canny", dst);
	cv::waitKey(0);

}

std::vector<cv::Point2i> Contours::get_active_non_visited_neighbours(cv::Mat&src, cv::Mat& visited, cv::Point&p){
	std::vector<cv::Point2i> return_points;
	std::vector<cv::Point2i> neighs = get_neighbour_points(src,p);
	//if it is a junction skip the rest
	//if(junction(src,p))
	//	return return_points;
	for(cv::Point2i& neighbour : neighs){
		if( src.at<uint8_t>(neighbour.y, neighbour.x) > 0 &&  visited.at<uint8_t>(neighbour.y, neighbour.x) == 0){
			return_points.push_back(neighbour);
		}
	}
	return return_points;

}

/*
 * returns a vector with the neighbouring pixels for a given pixel p
 */
std::vector<cv::Point2i> Contours::get_neighbour_points(cv::Mat& src, cv::Point&p){

	/*
	 *  ------------> (x) cols
	 * |
	 * |    1 2 3
	 * |    4 p 5
	 * |    6 7 8
	 * v
	 * (y) rows
	 */

	cv::Point2i p1(p.x -1, p.y-1);
	cv::Point2i p2(p.x , p.y-1);
	cv::Point2i p3(p.x +1, p.y-1);

	cv::Point2i p4(p.x -1, p.y);
	cv::Point2i p5(p.x +1, p.y);

	cv::Point2i p6(p.x -1, p.y+1);
	cv::Point2i p7(p.x , p.y+1);
	cv::Point2i p8(p.x +1, p.y+1);


	std::vector<cv::Point2i> all_neighbour_points({p1,p2,p3,p4,p5,p6,p7,p8}), return_neighbour_points;
	// 			cols         					rows
	if((p.x >= 1 && p.x < src.cols -1) && (p.y >= 1 && p.y < src.rows -1)){
		//normal case
		return all_neighbour_points;
	}
	return return_neighbour_points;
}

/*
 * computes the number of active neighbours for a given pixel p
 */
int Contours::neighbours(cv::Mat& src, cv::Point& p){
	std::vector<cv::Point2i>  neighbour_points = get_neighbour_points(src, p);

	int active_neighbours = 0;
	for(cv::Point2i neighbour : neighbour_points){
		if(src.at<uint8_t>(neighbour.y,neighbour.x) > 0)
			active_neighbours++;
	}
	return active_neighbours;
}

cv::Mat Contours::visu_orientation_map(const cv::Mat& mag, const cv::Mat& ori, double thresh)
{
	cv::Mat oriMap = cv::Mat::zeros(ori.size(), CV_8UC3);
	cv::Vec3b red(0, 0, 255);
	cv::Vec3b cyan(255, 255, 0);
	cv::Vec3b green(0, 255, 0);
	cv::Vec3b yellow(0, 255, 255);
    for(int i = 0; i < mag.rows*mag.cols; i++)
    {
        float* magPixel = reinterpret_cast<float*>(mag.data + i*sizeof(float));
        if(*magPixel > thresh)
        {
            float* oriPixel = reinterpret_cast<float*>(ori.data + i*sizeof(float));
            cv::Vec3b* mapPixel = reinterpret_cast<cv::Vec3b*>(oriMap.data + i*3*sizeof(char));
            if(*oriPixel < 90.0)
                *mapPixel = red;
            else if(*oriPixel >= 90.0 && *oriPixel < 180.0)
                *mapPixel = cyan;
            else if(*oriPixel >= 180.0 && *oriPixel < 270.0)
                *mapPixel = green;
            else if(*oriPixel >= 270.0 && *oriPixel < 360.0)
                *mapPixel = yellow;
        }
    }



    return oriMap;
}

void Contours::trace_contours(cv::Mat& original_img, cv::Mat& edges){
	cv::Mat mask = cv::Mat::ones(edges.rows,edges.cols,CV_8UC1);
	cv::Mat visited = cv::Mat::zeros(edges.rows,edges.cols,CV_8UC1);
	cv::Mat display = cv::Mat::zeros(edges.rows,edges.cols, CV_8UC3);
	cv::Mat ori,mag,visualise;
	orientation(edges, ori,mag);
	cv::Mat oriMap = visu_orientation_map(mag,ori);
	oriMap.copyTo(visualise);
	cv::imshow("orimap",visualise);
	cv::waitKey(0);
	std::vector<Contour> contours;

	bool cont = true;
	while(cont){
		Contour contour;
		cont = trace_contour(edges, visited, mask,display, contour);
		contours.push_back(contour);
		std::cout << contours.size()<<" # contours"<<std::endl;
	}
	cv::imwrite("contours.png",display);
	for(Contour c : contours){
		cv::imshow("contour", c.mask);
		cv::waitKey(0);
	}


}

bool Contours::junction(cv::Mat& src, cv::Point2i& p){

	cv::Point2i p1(p.x -1, p.y-1);
	cv::Point2i p2(p.x , p.y-1);
	cv::Point2i p3(p.x +1, p.y-1);

	cv::Point2i p4(p.x -1, p.y);
	cv::Point2i p5(p.x +1, p.y);

	cv::Point2i p6(p.x -1, p.y+1);
	cv::Point2i p7(p.x , p.y+1);
	cv::Point2i p8(p.x +1, p.y+1);
	std::vector<cv::Point2i> neighbours({p1,p2,p3,p4,p5,p6,p7,p8});

	// 1)
	if( active(src,neighbours[0])&&  active(src,neighbours[2]) &&  (active(src,neighbours[6]) || active(src,neighbours[5]) || active(src,neighbours[7]) )  )
		return true;
	// 2)
	if( active(src,neighbours[2])&& (active(src,neighbours[3]) || active(src,neighbours[5]) || active(src,neighbours[0]) ) &&  active(src,neighbours[7])  )
		return true;
	// 3)
	if(  (active(src,neighbours[1]) || active(src,neighbours[0]) || active(src,neighbours[2]) ) &&  active(src,neighbours[5]) &&  active(src,neighbours[7])  )
			return true;
	// 4)
	if( active(src,neighbours[0])&&  (active(src,neighbours[4]) || active(src,neighbours[2]) || active(src,neighbours[7]) ) &&  active(src,neighbours[5])  )
				return true;


	return false;

}

/*
 * takes as input an image and computes the map with the edges orientations
 */
void Contours::orientation(cv::Mat& src, cv::Mat& ori, cv::Mat& mag){
	cv::Mat Sx;
	cv::Sobel(src, Sx, CV_32F, 1, 0, 3);
	cv::Mat Sy;
	cv::Sobel(src, Sy, CV_32F, 0, 1, 3);
	cv::phase(Sx, Sy, ori, true);
	cv::magnitude(Sx, Sy, mag);
}

bool Contours::trace_contour(cv::Mat& src, cv::Mat& visited , cv::Mat& mask, cv::Mat& display,Contour& contour){


	contour.mask = cv::Mat::zeros(src.rows,src.cols,CV_8UC1);
	const int junction_number = 6;
	//find a seed where to start the tracing
	double min,max;
	cv::Point2i minLoc,seed;
	cv::Vec3b random_colour(rng(256), rng(256), rng(256));
	cv::minMaxLoc(src,&min,&max,&minLoc,&seed, mask);
	//check whether it is active
	if(src.at<uint8_t>(seed.y,seed.x) == 0)
		return false;



	//mark it as visited
	visited.at<uint8_t>(seed.y, seed.x) = 255;
	contour.mask.at<uint8_t>(seed.y, seed.x) = 255;
	mask.at<uint8_t>(seed.y, seed.x) = 0;
	display.at<cv::Vec3b>(seed.y, seed.x) = random_colour;

	contour.colour = random_colour;
	contour.points.push_back(seed);




	//start exploring neighbours at the seed
	std::vector<cv::Point2i> neighbours = get_active_non_visited_neighbours(src, visited, seed);
	contour.points.insert( contour.points.end(), neighbours.begin(), neighbours.end() );
	while(neighbours.size()>0){
		cv::Point2i local_seed = neighbours[neighbours.size()-1];
		neighbours.pop_back();

		//if it has been visited, ignore it
		if(visited.at<uint8_t>(local_seed.y, local_seed.x) > 0)
			continue;

		//mark as visited
		visited.at<uint8_t>(local_seed.y, local_seed.x) = 255;
		mask.at<uint8_t>(local_seed.y, local_seed.x) = 0;
		contour.mask.at<uint8_t>(local_seed.y, local_seed.x) = 255;
		display.at<cv::Vec3b>(local_seed.y, local_seed.x) = random_colour;


		//cv::imshow("display",display);
		//cv::waitKey(0);

		std::vector<cv::Point2i> local_neighbours = get_active_non_visited_neighbours(src, visited, local_seed);
		if(local_neighbours.size()>0){
			neighbours.insert( neighbours.end(), local_neighbours.begin(), local_neighbours.end() );
			contour.points.insert( contour.points.end(), local_neighbours.begin(), local_neighbours.end() );
		}



	}

	return true;



}

