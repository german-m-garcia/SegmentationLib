/*
 * Contours.cpp
 *
 *  Created on: 8 Jul 2016
 *      Author: martin
 */

#include <iostream>
#include <map>
#include "contours/Contours.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


//initialise the Random Number Generator
cv::RNG Contours::rng = cv::RNG();

Contours::Contours(): minimum_size_(35),red_(0, 0, 255), red2_(0, 125, 255), cyan_(255, 255, 0), cyan2_(255, 255, 125),
green_(0, 255, 0), green2_(125, 255, 0), yellow_(0, 255, 255), yellow2_(125, 255, 255),
		nintervals_(8),intervals_(nintervals_), colours_({red_,red2_,cyan_,cyan2_,green_,green2_,yellow_,yellow2_}) {

	for(int i= 0;i<nintervals_;i++){
		colour_indices_[colours_[i]] = i;
	}

}

Contours::~Contours() {
	// TODO Auto-generated destructor stub
}

/*
 * @src: input image, grayscale 8-bit
 * Computes the edges of the src image using the Canny edge
 * detector.
 */
void Contours::compute_edges(cv::Mat& src, cv::Mat& edges){

	int edgeThresh = 1;
	int lowThreshold = 15;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;
	cv::Mat src_blurred;

	/// Canny detector
	 /// Reduce noise with a kernel 3x3
	cv::blur( src, src_blurred, cv::Size(3,3) );
	cv::Canny(src_blurred, edges, lowThreshold, max_lowThreshold, kernel_size);
	//cv::imshow("canny", dst);
	//cv::waitKey(0);

}

std::vector<cv::Point2i> Contours::get_neighbour_points_in_edge(const Contour& c, const cv::Point& p){
	std::vector<cv::Point2i> return_points;
	std::vector<cv::Point2i> neighs = get_neighbour_points(c.mask,p);

	for(cv::Point2i& neighbour : neighs){
		if( active( c.mask, neighbour) )
			return_points.push_back(neighbour);

	}
	return return_points;
}

std::vector<cv::Point2i> Contours::get_active_non_visited_neighbours(const cv::Mat&src, cv::Mat& visited, cv::Point&p){
	std::vector<cv::Point2i> return_points;
	std::vector<cv::Point2i> neighs = get_neighbour_points(src,p);

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
std::vector<cv::Point2i> Contours::get_neighbour_points(const cv::Mat& src, const cv::Point&p){

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
 * Computes the number of active neighbours for a given pixel p.
 */
int Contours::neighbours(const cv::Mat& src, cv::Point& p){
	std::vector<cv::Point2i>  neighbour_points = get_neighbour_points(src, p);

	int active_neighbours = 0;
	for(cv::Point2i neighbour : neighbour_points){
		if(src.at<uint8_t>(neighbour.y,neighbour.x) > 0)
			active_neighbours++;
	}
	return active_neighbours;
}

/*
 * @mag: the gradient magnitude
 * @ori: the orientation map
 * Visualises an orientation map.
 */
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

/*
 * @mag: the gradient magnitude
 * @ori: the orientation map
 *
 * Visualises an orientation map. Orientations are in modulo 180°, then discretised
 * in 4 bins of 45° each. Each bin is assigned a colour.
 *
 * @returns the Mat with the colours/orientation
 */
cv::Mat Contours::visu_orientation_map2(const cv::Mat& mag, const cv::Mat& ori, double thresh)
{
	cv::Mat oriMap = cv::Mat::zeros(ori.size(), CV_8UC3);




	//fill in the intervals
	for(int i=0;i<nintervals_;i++){
		intervals_[i] = 180. / (nintervals_*1.) * (i+1.);
		std::cout <<"interval["<<i<<"]="<<intervals_[i];
	}

	for(int i = 0; i < mag.rows*mag.cols; i++)
    {
        float* magPixel = reinterpret_cast<float*>(mag.data + i*sizeof(float));
        if(*magPixel > thresh)
        {
            float* oriPixel = reinterpret_cast<float*>(ori.data + i*sizeof(float));
            cv::Vec3b* mapPixel = reinterpret_cast<cv::Vec3b*>(oriMap.data + i*3*sizeof(char));
            float radians = (*oriPixel);
            if(radians > 180.0)
            	radians -= 180.f;

           if(radians < intervals_[0])
        	   *mapPixel = colours_[0];
           for(int ind = 1; ind <nintervals_; ind++){
        	   if(radians > intervals_[ind-1] && radians <= intervals_[ind]){
        		   *mapPixel = colours_[ind];
        	   }
           }
        }
    }

    return oriMap;
}




void Contours::get_majority_orientation(cv::Mat& orientation_map, cv::Point2i& p, cv::Vec3b& orientation){
	std::vector<cv::Point2i> neighbours = get_neighbour_points(orientation_map, p);
	std::map<cv::Vec3b, int, compare_colors<cv::Vec3b>> countmap;
	int max = 0;
	//by default, the orientation is the one of the pixel
	orientation =  orientation_map.at<cv::Vec3b>(p.y,p.x);
	if(orientation[0] >0 || orientation[1]>0 || orientation[2]>0){
		countmap[orientation]++;
		max = 1;
	}
	//iterate over the neighbours
	for(cv::Point2i& n : neighbours){
		//count the occurrences of the neighbours colours
		cv::Vec3b key = orientation_map.at<cv::Vec3b>(n.y,n.x);
		if(key[0] >0 || key[1]>0 || key[2]>0)
			countmap[key]++;
	}
	for (const auto& kv : countmap) {
	    if(kv.second > max){
	    	max = kv.second;
	    	orientation=kv.first;
	    }
		//if(kv.second[0] != 0 && kv.second[1] != 0  && kv.second[2] != 0)
	    //std::cout << kv.first << " has value " << kv.second << std::endl;
	}

}

/*
 * iterates over the pixels in the edge map, and assigns them the orientation
 * of the majority of the 3x3 neighbourhood
 */
void Contours::filter_majority_orientation(cv::Mat& orientation_map, cv::Mat& edges, cv::Mat& dst){
	dst = cv::Mat::zeros(orientation_map.size(),orientation_map.type());
	for(int i=1; i< edges.rows -1 ; i++){
		for(int j=1; j< edges.cols -1; j++){
			cv::Point2i p(j,i);
			if(active(edges, p)){
				cv::Vec3b max_value;
				get_majority_orientation(orientation_map,p,max_value);
				dst.at<cv::Vec3b>(p.y,p.x) = max_value;

			}
		}
	}
}



/*
 * @edges: CV_8UC1
 *
 * Iterates over the pixels in the edge map, detects junction points,
 * and in a final iteration a neighbourhood of 4 pixels around the junction points
 * is set to 0 in the edge map.
 *
 */
void Contours::clear_junctions(cv::Mat& edges){
	std::vector<Junction> junctions;


	//clear out junction points
	for(int i=0;i<edges.rows;i++){
		for(int j=0;j<edges.cols;j++){
			cv::Point2i p(j,i);
			Junction junc;
			if( junction(edges,p,junc)){
				junctions.push_back(junc);

			}

		}
	}
	for(Junction& j : junctions){

		clear_junction(edges, j);
	}
}

/*
 * Sets a neighbourhood of 4 pixels around the junction point to 0
 * in the edge map.
 *
 */
void Contours::clear_junction(cv::Mat& edges, Junction& j){

	/*
	 *  ------------> (x) cols
	 * |
	 * |    1 2 3
	 * |    4 p 5
	 * |    6 7 8
	 * v
	 * (y) rows
	 */

	cv::Point2i p1(j.p.x -1, j.p.y-1);
	cv::Point2i p2(j.p.x , j.p.y-1);
	cv::Point2i p3(j.p.x +1, j.p.y-1);

	cv::Point2i p4(j.p.x -1, j.p.y);
	cv::Point2i p5(j.p.x +1, j.p.y);

	cv::Point2i p6(j.p.x -1, j.p.y+1);
	cv::Point2i p7(j.p.x , j.p.y+1);
	cv::Point2i p8(j.p.x +1, j.p.y+1);

	if(j.top_left){
		edges.at<uint8_t>(j.p.y, j.p.x) = 0;
		edges.at<uint8_t>(p5.y, p5.x) = 0;
		edges.at<uint8_t>(p7.y, p7.x) = 0;
		edges.at<uint8_t>(p8.y, p8.x) = 0;
	}
	else if(j.top_right){
		edges.at<uint8_t>(j.p.y, j.p.x) = 0;
		edges.at<uint8_t>(p4.y, p4.x) = 0;
		edges.at<uint8_t>(p6.y, p6.x) = 0;
		edges.at<uint8_t>(p7.y, p7.x) = 0;
	}
	else if(j.bottom_left){
		edges.at<uint8_t>(j.p.y, j.p.x) = 0;
		edges.at<uint8_t>(p5.y, p5.x) = 0;
		edges.at<uint8_t>(p2.y, p2.x) = 0;
		edges.at<uint8_t>(p3.y, p3.x) = 0;
		}
	else if(j.bottom_right){
		edges.at<uint8_t>(j.p.y, j.p.x) = 0;
		edges.at<uint8_t>(p1.y, p1.x) = 0;
		edges.at<uint8_t>(p2.y, p2.x) = 0;
		edges.at<uint8_t>(p4.y, p4.x) = 0;
	}

}

/*
 * iterates over the contours, checks which have less than minimum_size_
 * points and removes them
 */
void Contours::remove_noisy_contours(std::vector<Contour>& contours){
	std::vector<Contour> new_contours;
	for(Contour& c : contours){
		if(c.points.size() > minimum_size_)
			new_contours.push_back(c);
	}
	contours.swap(new_contours);
}

void Contours::scharr_edges(cv::Mat& colour_img, cv::Mat& colour_edges, cv::Mat& gray_edges, cv::Mat& float_edges, cv::Mat& ori){
	cv::Mat gradx, grady, tmp;
	cv::Scharr(colour_img, gradx, CV_32F, 1, 0, 1);
	cv::Scharr(colour_img, grady, CV_32F, 0, 1, 1);

	cv::magnitude(gradx, grady, colour_edges);
	cv::phase(gradx, grady, ori, true);


	/// Convert the colour edges to grayscale
	colour_edges.convertTo(tmp,CV_8UC3);
	colour_edges.convertTo(float_edges,CV_32FC1);

	cvtColor( tmp, gray_edges, CV_BGR2GRAY );
	gray_edges = gray_edges > 100;

}

/*
 * Dilates the contours, and shows each of them in a different colour.
 */
void Contours::display_contours(std::vector<Contour>& contours){
	if(contours.size() == 0)
		return;
	cv::Mat display = cv::Mat::zeros(contours[0].mask.rows,contours[0].mask.cols, CV_8UC3);
	for(Contour& c : contours){
		cv::Mat mask;
		cv::dilate(c.mask, mask, cv::Mat());
		display.setTo(c.colour,mask);
		/*cv::imshow("colour contours", display);
		cv::imshow("current contour", c.mask);
		cv::waitKey(0);*/
	}
	cv::imshow("colour contours", display);
	cv::waitKey(0);
}

/*
 * Traces every edge present in the edge map into a contour.
 * Right now it finds junctions as breaking points along the contours.
 * It should also find points where the curvature changes.
 */
void Contours::trace_contours(const cv::Mat& original_img,  cv::Mat& edges){
	cv::Mat mask = cv::Mat::ones(edges.rows,edges.cols,CV_8UC1);
	cv::Mat visited = cv::Mat::zeros(edges.rows,edges.cols,CV_8UC1);
	cv::Mat display = cv::Mat::zeros(edges.rows,edges.cols, CV_8UC3);
	cv::Mat ori,mag,visualise,oriMapFiltered;
	orientation(edges, ori,mag);
	cv::Mat oriMap = visu_orientation_map2(mag,ori);
	oriMap.copyTo(visualise/*, edges*/);

	filter_majority_orientation(oriMap,edges,oriMapFiltered);

	cv::imshow("orimap",visualise);
	cv::imshow("orimap filtered",oriMapFiltered);
	cv::waitKey(0);

	//sets the junction points to 0
	clear_junctions(edges);

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
		//cv::imshow("contour", c.mask);

		mark_high_curvature_points(c,oriMapFiltered);
		cv::Mat edges_disp = edges.clone();
		for(cv::Point2i& p : c.points){

		}
		//edges_disp.release();
		//cv::waitKey(0);
	}

	remove_noisy_contours(contours);
	display_contours(contours);


}

/*
 * --------------------------------------------
 * Methods for detecting high curvature
 * --------------------------------------------
 */



void Contours::mark_high_curvature_points(Contour& contour, const cv::Mat& orientation){

	cv::Mat cropped_orientation, display;
	orientation.copyTo(cropped_orientation, contour.mask);
	display = cropped_orientation.clone();

	for(cv::Point2i& p : contour.points){
		if( high_curvature_point(cropped_orientation, contour,p)){
			cv::circle(display,p,10,cv::Scalar(0,255,0));
			cv::imshow("display", display);

		}
	}
	cv::waitKey(0);
}

/*
 * returns true if p is a high curvature point
 */
bool Contours::high_curvature_point(const cv::Mat& orientation,Contour& contour, const cv::Point& p){

	std::vector<cv::Point2i> neighbours = get_neighbour_points_in_edge(contour,p);
	cv::Vec3b colour = orientation.at<cv::Vec3b>(p.y,p.x);
	int current_index = colour_indices_[colour];
	for(cv::Point2i& n : neighbours){
		colour = orientation.at<cv::Vec3b>(n.y,n.x);
		int neigh_index = colour_indices_[colour];

		if( std::abs(neigh_index - current_index) > 5)
			return true;
	}

	return false;
}

/*
 *
 * Returns true if point p is a junction. The junction information
 * is returned in &j.
 */
bool Contours::junction(const cv::Mat& src, cv::Point2i& p, Junction& j){



	/*
	 *  ------------> (x) cols
	 * |
	 * |    1 2 3
	 * |    4 p 5
	 * |    6 7 8
	 * v
	 * (y) rows
	 */
	j.p = p;

	cv::Point2i p1(p.x -1, p.y-1);
	cv::Point2i p2(p.x , p.y-1);
	cv::Point2i p3(p.x +1, p.y-1);

	cv::Point2i p4(p.x -1, p.y);
	cv::Point2i p5(p.x +1, p.y);

	cv::Point2i p6(p.x -1, p.y+1);
	cv::Point2i p7(p.x , p.y+1);
	cv::Point2i p8(p.x +1, p.y+1);
	std::vector<cv::Point2i> neighbours({p1,p2,p3,p4,p5,p6,p7,p8});
	if(p.x == 0|| p.x == src.cols-1 || p.y == 0 || p.y == src.rows-1)
		return false;
//	for(cv::Point2i& n : neighbours){
//		if(n.x < 0 || n.x >= src.cols || n.y < 0 || n.y >= src.rows){
//			std::cout <<" point our of range "<<n<<std::endl;
//		}
//	}



	/*
	 *  ------------> (x) cols
	 * |
	 * |    1   3
	 * |      p
	 * |      7
	 * v
	 * (y) rows
	 */
	if( active(src,neighbours[0])&&  active(src,neighbours[2]) &&  (active(src,neighbours[6]))){
		j.bottom_right = true;
		return true;
	}

	/*
	 *  ------------> (x) cols
	 * |
	 * |        3
	 * |    4 p
	 * |        8
	 * v
	 * (y) rows
	 */
	if( active(src,neighbours[2])&& (active(src,neighbours[3]) && active(src,neighbours[7]))){
		j.bottom_left = true;
		return true;
	}

	/*
	 *  ------------> (x) cols
	 * |
	 * |      2
	 * |      p
	 * |    6   8
	 * v
	 * (y) rows
	 */
	if(  active(src,neighbours[1]) &&  active(src,neighbours[5]) &&  active(src,neighbours[7])  ){
		j.top_right = true;
		return true;
	}

	/*
	 *  ------------> (x) cols
	 * |
	 * |    1
	 * |      p 5
	 * |    6
	 * v
	 * (y) rows
	 */
	if( active(src,neighbours[0])&&  active(src,neighbours[4]) &&  active(src,neighbours[5])  ){
		j.top_right = true;
		return true;
	}

	/*
	 *  ------------> (x) cols
	 * |
	 * |    1
	 * |      p 5
	 * |      7
	 * v
	 * (y) rows
	 */
	if( active(src,neighbours[0])&&  active(src,neighbours[4]) &&  active(src,neighbours[6])  ){
		j.bottom_right = true;
		return true;
	}

	/*
	 *  ------------> (x) cols
	 * |
	 * |        3
	 * |    4 p
	 * |      7
	 * v
	 * (y) rows
	 */
	if( active(src,neighbours[2])&&  active(src,neighbours[3]) &&  active(src,neighbours[6])  ){
		j.bottom_left = true;
		return true;
	}

	/*
	 *  ------------> (x) cols
	 * |
	 * |      2
	 * |    4 p
	 * |       8
	 * v
	 * (y) rows
	 */
	if( active(src,neighbours[1])&&  active(src,neighbours[3]) &&  active(src,neighbours[7])  ){
		j.top_left = true;
		return true;
	}

	/*
	 *  ------------> (x) cols
	 * |
	 * |      2
	 * |      p 5
	 * |    6
	 * v
	 * (y) rows
	 */
	if( active(src,neighbours[1])&&  active(src,neighbours[4]) &&  active(src,neighbours[5])  ){
		j.top_right = true;
		return true;
	}


	return false;

}

/*
 * takes as input an image and computes the map with the edges orientations
 */
void Contours::orientation(const cv::Mat& src, cv::Mat& ori, cv::Mat& mag){
	cv::Mat Sx;
	cv::Sobel(src, Sx, CV_32F, 1, 0, 3);
	cv::Mat Sy;
	cv::Sobel(src, Sy, CV_32F, 0, 1, 3);
	cv::phase(Sx, Sy, ori, true);
	cv::magnitude(Sx, Sy, mag);
}

/*
 * Finds a seed in the edge map, and traces it to find a connected component.
 */
bool Contours::trace_contour(const cv::Mat& src, cv::Mat& visited , cv::Mat& mask, cv::Mat& display,Contour& contour){


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


