/*
 * segment.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: gmartin
 */

#include "segment.h"
#include <vector>
using namespace std;


//initialise the Random Number Generator
RNG Segment::rng = theRNG();

Segment::Segment(Mat& original):original_(original),random_colour_(rng(256), rng(256), rng(256)){
 mat_original_colour_ = Mat::zeros(original.rows,original.cols,CV_8UC3);
 random_colour_mat_= Mat::zeros(original.rows,original.cols,CV_8UC3);

}

Segment::Segment(Mat& original,Mat& segment_mat, Mat& binary_original,vector<Point2i>& contour, Rect& rect, int segment_size, Vec3b colour):
		random_colour_mat_(segment_mat), binary_mat_(binary_original),contour_(contour),random_colour_(colour),bound_rect(rect),
		segment_size_(segment_size), eigen_vecs(2),eigen_val(2),orientations_(2),modules_(2){

	mat_original_colour_ = Mat::zeros(original.rows,original.cols,CV_8UC3);
	binary_mat_ = binary_mat_ > 0;
	original.copyTo(mat_original_colour_, binary_mat_(bound_rect));
	dilate(binary_mat_,binary_mat_,Mat());
	dilate(binary_mat_,binary_mat_,Mat());
	computeHistogram();
	if(DEBUG){
		Mat original_size_mat = Mat::zeros(binary_mat_.rows,binary_mat_.cols,CV_8UC3);
		original.copyTo(original_size_mat(bound_rect),binary_mat_(bound_rect));
		computePCA(contour_,original_size_mat);
		imshow("PCA",original_size_mat);
		waitKey(0);
	}
	else
		computePCA(contour_);

}

Segment::~Segment(){

	//cout <<" deleting Segment::~Segment()"<<endl;
}


Segment::Segment (const Segment &obj):visualFeatures(obj.visualFeatures), mat_original_colour_(obj.mat_original_colour_), random_colour_mat_(obj.random_colour_mat_)
,binary_mat_(obj.binary_mat_), contour_(obj.contour_),random_colour_(obj.random_colour_)
,original_(obj.original_),
		histImage_(obj.histImage_),h_hist(obj.h_hist), s_hist(obj.s_hist), v_hist(obj.v_hist), mask_(obj.mask_),class_label(obj.class_label),bound_rect(obj.bound_rect),
		segment_size_(obj.segment_size_){
   // body of constructor

}

void Segment::addPoint(const Point2i& point,const Vec3b& colour){

	mat_original_colour_.at<Vec3b>(point.x,point.y) = colour;
	points.push_back(point);

}

void Segment::colour_this_segment(Mat& dst){
	for(Point2i& p : points){
		dst.at<Vec3b>(p.x,p.y) = random_colour_;
	}

}

void Segment::computeFeatures(){
	computeHistogram();
	visualFeatures = Mat(1,NUMBER_VISUAL_FEATS, CV_32FC1);
	vector<Mat> vectorFeats = {h_hist.t(),s_hist.t(),v_hist.t()};
	hconcat(vectorFeats,visualFeatures);
}

void Segment::computeHuMoments() {

}

void Segment::computeHistogram() {
		if (!mat_original_colour_.data) {
			return;
		}

		mask_ = Mat::zeros(mat_original_colour_.rows,mat_original_colour_.cols,CV_8UC1);
		//threshold grayscale to binary image
		cv::cvtColor(mat_original_colour_, mask_, CV_BGR2GRAY);
		mask_ = mask_ > 0;
		//imshow("original_",original_);
		//waitKey(0);
		/// Separate the image in 3 places ( B, G and R )
		vector<Mat> bgr_planes;
		split(mat_original_colour_, bgr_planes);

		/// Establish the number of bins
		int histSize = NHISTOGRAMBINS;

		/// Set the ranges ( for B,G,R) )
		float range_sv[] = { 0, 255 };
		const float* histRange_sv = { range_sv };
		float range_h[] = { 0, 180 };
		const float* histRange_h = { range_h };

		bool uniform = true;
		bool accumulate = false;
//		imshow("mat_original_colour_",mat_original_colour_);
//		cout << "bgr_planes[0].size()="<<bgr_planes[0].size()<<endl;
//		cout << "bgr_planes[1].size()="<<bgr_planes[1].size()<<endl;
//		cout << "bgr_planes[2].size()="<<bgr_planes[2].size()<<endl;
//		cout <<"mask.size()="<<mask.size()<<endl;
//		waitKey(0);


		/// Compute the histograms:
		calcHist(&bgr_planes[0], 1, 0, mask_, h_hist, 1, &histSize, &histRange_h,
				uniform, accumulate);
		calcHist(&bgr_planes[1], 1, 0, mask_, s_hist, 1, &histSize, &histRange_sv,
				uniform, accumulate);
		calcHist(&bgr_planes[2], 1, 0, mask_, v_hist, 1, &histSize, &histRange_sv,
				uniform, accumulate);

//		//remove NaNs
//		Mat mask_h = Mat(h_hist != h_hist);
//		h_hist.setTo(0,mask_h);
//		Mat mask_s = Mat(s_hist != s_hist);
//		s_hist.setTo(0,mask_s);
//		Mat mask_v = Mat(v_hist != v_hist);
//		v_hist.setTo(0,mask_v);

		// Draw the histograms for H, S and V
		int hist_w = 120;
		int hist_h = 100;
		int bin_w = cvRound((double) hist_w / histSize);

		histImage_= Mat(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));


		/// Normalize the result to [ 0,1]
		normalize(h_hist, h_hist, 0, 1, NORM_MINMAX, -1, Mat());
		normalize(s_hist, s_hist, 0, 1, NORM_MINMAX, -1, Mat());
		normalize(v_hist, v_hist, 0, 1, NORM_MINMAX, -1, Mat());


		if(DEBUG){
			/// Normalize the result to [ 0, histImage.rows ]
			normalize(h_hist, h_hist, 0, histImage_.rows, NORM_MINMAX, -1, Mat());
			normalize(s_hist, s_hist, 0, histImage_.rows, NORM_MINMAX, -1, Mat());
			normalize(v_hist, v_hist, 0, histImage_.rows, NORM_MINMAX, -1, Mat());
			/// Draw for each channel
			for (int i = 1; i < histSize; i++) {
				line(histImage_,
						Point(bin_w * (i - 1),
								hist_h - cvRound(h_hist.at<float>(i - 1))),
						Point(bin_w * (i), hist_h - cvRound(h_hist.at<float>(i))),
						Scalar(255, 0, 0), 2, 8, 0);
				line(histImage_,
						Point(bin_w * (i - 1),
								hist_h - cvRound(s_hist.at<float>(i - 1))),
						Point(bin_w * (i), hist_h - cvRound(s_hist.at<float>(i))),
						Scalar(0, 255, 0), 2, 8, 0);
				line(histImage_,
						Point(bin_w * (i - 1),
								hist_h - cvRound(v_hist.at<float>(i - 1))),
						Point(bin_w * (i), hist_h - cvRound(v_hist.at<float>(i))),
						Scalar(0, 0, 255), 2, 8, 0);
			}

		}



	}


void Segment::drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}

void Segment::computePCA(const vector<Point> &pts)
{
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    center_ = cntr;
    //Store the eigenvalues and eigenvectors

    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    orientations_[0]= atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    orientations_[0] = (orientations_[0]/M_PI*180) + (orientations_[0] > 0 ? 0 : 360);
    orientations_[1]= atan2(eigen_vecs[1].y, eigen_vecs[1].x); // orientation in radians
    orientations_[1] = (orientations_[1]/M_PI*180) + (orientations_[1] > 0 ? 0 : 360);

    modules_[0] = sqrt( eigen_vecs[0].x * eigen_vecs[0].x + eigen_vecs[0].y * eigen_vecs[0].y  );
    modules_[1] = sqrt( eigen_vecs[1].x * eigen_vecs[1].x + eigen_vecs[1].y * eigen_vecs[1].y  );


}

double Segment::computePCA(const vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    center_ = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    orientations_[0]= atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    orientations_[1]= atan2(eigen_vecs[1].y, eigen_vecs[1].x); // orientation in radians

    // Draw the principal components
    circle(img, center_, 3, Scalar(255, 0, 255), 2);
    Point p1 = center_ + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = center_ - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, center_, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, center_, p2, Scalar(255, 255, 0), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle;
}


void Segment::re_colour(Vec3b new_colour){
	random_colour_ = new_colour;
	random_colour_mat_.setTo(random_colour_,mask_);
}


bool Segment::part_of(Segment* other_segment){

}
