/*
 * Contours.h
 *
 *  Created on: 8 Jul 2016
 *      Author: martin
 */

#ifndef SRC_CONTOURS_CONTOURS_H_
#define SRC_CONTOURS_CONTOURS_H_

 #define active(src, p)  (src.at<uint8_t>(p.y, p.x) > 0)
#define inactive(src, p)  (src.at<uint8_t>(p.y, p.x) == 0)

#include <opencv2/core/core.hpp>

class Contour {

public:
	std::vector<cv::Point2i> points;
	cv::Mat mask;
	cv::Vec3b colour;
	std::vector<cv::Point2i> end_points;
	cv::Vec3i col_avg_side1_,col_avg_side2_;

	void colour_sides(const cv::Mat& originalMat, cv::Mat& oriMapFiltered);

	static bool sort_points(const cv::Point2i& p1, const cv::Point2i& p2){
		return ((p1.x < p2.x) || ( p1.x == p2.x && p1.y < p2.y));


	}
	std::vector<cv::Point2i> get_neighbour_points_in_edge( const cv::Point& p);
	std::vector<cv::Point2i> get_neighbour_points(cv::Mat& src, const cv::Point& p);
	bool end_point(const cv::Mat& src, cv::Point2i& p);

	void get_sides_contour_point(cv::Point& p, cv::Mat& oriMapFiltered, cv::Point& side1, cv::Point& side2);

	static void histogram(const cv::Mat& src, cv::Mat& contour_mat);

	bool colour_similarity(const Contour& a, const Contour& b);




};

struct Junction {

	cv::Point2i p;
	bool top_left;
	bool top_right;
	bool bottom_left;
	bool bottom_right;
};

class Contours {

	friend class Contour;
public:
	Contours();
	virtual ~Contours();

	void compute_edges(cv::Mat& src, cv::Mat& edges);
	void scharr_edges(cv::Mat& colour_img, cv::Mat& colour_edges, cv::Mat& gray_edges, cv::Mat& float_edges, cv::Mat& ori);



	void trace_contours(const cv::Mat& original_img,  cv::Mat& src);
	cv::Mat visu_orientation_map(const cv::Mat& mag, const cv::Mat& ori, double thresh = 1.0);
	cv::Mat visu_orientation_map2(const cv::Mat& mag, const cv::Mat& ori, double thresh = 1.0);
	void display_contours(std::vector<Contour>& contours);




private:

	template<typename _Tp>
		struct compare_colors : public std::binary_function<_Tp, _Tp, bool> {
		  bool operator()(const _Tp& x, const _Tp& y) const {
		    return x[0] < y[0]
		        || (x[0] == y[0] && x[1] < y[1])
		        || (x[0] == y[0] && x[1] == y[1] && x[2] < y[2]);
		  }
		};

	void orientation(const cv::Mat& src, cv::Mat& ori, cv::Mat& mag);
	bool trace_contour(const cv::Mat& src, cv::Mat& visited , cv::Mat& mask, cv::Mat& display,Contour& contour);
	std::vector<cv::Point2i> get_active_non_visited_neighbours(const cv::Mat&src, cv::Mat& visited, cv::Point&p);
	int neighbours(const cv::Mat& src, cv::Point& p);
	std::vector<cv::Point2i> get_neighbour_points_in_edge(const Contour& c,const  cv::Point& p);
	std::vector<cv::Point2i> get_5x5_neighbour_points_in_edge(const Contour& c,const  cv::Point& p);
	void find_contours_on_edge_map(cv::Mat& edges,std::vector<Contour>& contours);
	std::vector<cv::Point2i> get_neighbour_points(const cv::Mat& src, const cv::Point&p);
	std::vector<cv::Point2i> get_5x5_neighbour_points(const cv::Mat& src, const cv::Point&p);
	void remove_noisy_contours(std::vector<Contour>& contours);
	/*
	 * junctions
	 */
	bool junction(const cv::Mat& src, cv::Point2i& p, Junction& j);
	void clear_junction(cv::Mat& edges, Junction& j);
	void clear_junctions(cv::Mat& edges);

	/*
	 * binned orientation map
	 */
	void filter_majority_orientation(const cv::Mat& orientation_map, cv::Mat& edges, cv::Mat& dst);
	void get_majority_orientation(const cv::Mat& orientation_map, cv::Point2i& p, cv::Vec3b& orientation);

	/*
	 * high curvature methods
	 */
	void mark_high_curvature_points(Contour& contour, const cv::Mat& orientation,std::vector<cv::Point>& curvature_points);
	bool high_curvature_point(const cv::Mat& orientation, Contour& contour,const cv::Point& p);
	int bins_distance(int b1, int b2);



	static cv::RNG rng;
	cv::Mat orientation_mat;
	int minimum_size_;

	cv::Vec3b red_,red2_,cyan_,cyan2_,green_,green2_,yellow_,yellow2_;

	int nintervals_;
	std::vector<int> intervals_;
	std::vector<cv::Vec3b> colours_;
	std::map<cv::Vec3b, int,compare_colors<cv::Vec3b>> colour_indices_;

	const int HIGH_CURVATURE_INDEX_DIFFERENCE = 3;




};

#endif /* SRC_CONTOURS_CONTOURS_H_ */
