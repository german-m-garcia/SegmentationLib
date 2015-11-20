/*
 * Segmentation.h
 *
 *  Created on: Nov 21, 2013
 *      Author: martin
 */

#ifndef ABSSEGMENTATION_H_
#define ABSSEGMENTATION_H_

#include <opencv2/opencv.hpp>
#include <set>

using namespace std;
using namespace cv;

#define DEBUG false

class Segment {

public:
	Segment(Mat&);
	Segment(Mat& original,Mat& segment_mat, Mat& binary_original,vector<Point2i>& contour, Rect& rect, int segment_size, Vec3b colour);
	Segment (const Segment &obj);
	virtual ~Segment();
	void addPoint(const Point2i& point,const Vec3b& colour);

	void computeFeatures();


	const Mat& getImg() const {
		return mat_original_colour_;
	}

	const Mat& getHistImage() const {
		return histImage_;
	}

	float getClassLabel() const {
		return class_label;
	}

	void setClassLabel(float classLabel) {
		class_label = classLabel;
	}

	const Mat& getRandomColourMat() const {
		return random_colour_mat_;
	}

	void colour_this_segment(Mat& dst);

	const Rect& getBoundRect() const {
		return bound_rect;
	}

	int getSegmentSize() const {
		return segment_size_;
	}

	const Mat& getBinaryMat() const {
		return binary_mat_;
	}

	const Mat& getMatOriginalColour() const {
		return mat_original_colour_;
	}

	const Mat& getHHist() const {
		return h_hist;
	}

	const Mat& getSHist() const {
		return s_hist;
	}

	const Mat& getVHist() const {
		return v_hist;
	}

	const vector<double>& getEigenVal() const {
		return eigen_val;
	}

	const vector<Point2d>& getEigenVecs() const {
		return eigen_vecs;
	}

	const vector<double>& getOrientations() const {
		return orientations_;
	}

	const Vec3b& getRandomColour() const {
		return random_colour_;
	}

	const Vec3b& getLabel() const {
		return random_colour_;
	}

	//for matching
	void re_colour(Vec3b new_colour);

	const Mat& getMask() const {
		return mask_;
	}

	bool part_of(Segment* other_segment);

	const Point2i& getCenter() const {
		return center_;
	}

	Mat visualFeatures;
	static const int NHISTOGRAMBINS = 128; //256
	static const int NUMBER_VISUAL_FEATS=NHISTOGRAMBINS*3;

protected:
	//the segment in the original colour in reduced size
	Mat mat_original_colour_;
	//the segment with a random colour in reduced size
	Mat random_colour_mat_;
	//the segment in binary in the original size
	Mat binary_mat_;
	//the contour points
	vector<Point2i> contour_;
	//the random colour
	Vec3b random_colour_;
	//the original colour img
	Mat original_;
	//the histogram to be displayed
	Mat histImage_;
	//the HSV histograms
	Mat h_hist, s_hist, v_hist;

	//the binary mask of the segment in reduced size
	Mat mask_;

	float class_label;

	Rect bound_rect;


	//add some features

	void computeHistogram();

	void computeHuMoments();

	void computePCA(const vector<Point> &);

	void drawAxis(Mat&, Point, Point, Scalar, const float);

	double computePCA(const vector<Point> &, Mat&);


private:
	static RNG rng;
	vector<Point2i> points;
	int segment_size_;
	vector<Point2d> eigen_vecs;
	vector<double> eigen_val;
	vector<double> orientations_;
	vector<double> modules_;
	Point2i center_;



};


#endif /* SEGMENTATION_H_ */
