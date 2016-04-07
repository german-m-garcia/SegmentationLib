/*
 * segment.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: gmartin
 */

#include <pcl/features/fpfh.h>
#include "utils.h"
#include "segment.h"
#include <vector>

using namespace std;


//initialise the Random Number Generator
RNG Segment::rng = RNG();

Segment::Segment(Mat& original) :
		random_colour_(rng(256), rng(256), rng(256)), original_(original), pcl_cloud_(
				new pcl::PointCloud<pcl::PointXYZRGB>),
				eigen_vecs(2),
				eigen_val(2),
				orientations_(2),
				modules_(2){
	mat_original_colour_ = Mat::zeros(original.rows, original.cols, CV_8UC3);
	random_colour_mat_ = Mat::zeros(original.rows, original.cols, CV_8UC3);
	pca_data_ = Mat::zeros(1, 4, CV_32FC1);
	huMat_ = Mat::zeros(1, 7, CV_32FC1);

}

Segment::Segment(Mat& original, Mat& segment_mat, Mat& binary_original,
		vector<Point2i>& contour, Rect& rect, int segment_size, Vec3b colour) :
		random_colour_mat_(segment_mat), binary_mat_(binary_original), contour_(
				contour), random_colour_(colour), bound_rect(rect), pcl_cloud_(
				new pcl::PointCloud<pcl::PointXYZRGB>), segment_size_(
				segment_size), eigen_vecs(2), eigen_val(2), orientations_(2), modules_(
				2) {

	pca_data_ = Mat::zeros(1, 4, CV_32FC1);
	huMat_ = Mat::zeros(1, 7, CV_32FC1);
	mat_original_colour_ = Mat::zeros(original.rows, original.cols, CV_8UC3);
	binary_mat_ = binary_mat_ > 0;
	original.copyTo(mat_original_colour_, binary_mat_(bound_rect));

	//dilate(binary_mat_,binary_mat_,Mat());
	//dilate(binary_mat_,binary_mat_,Mat());

	//	computeHistogram();
//	if(DEBUG){
//		Mat original_size_mat = Mat::zeros(binary_mat_.rows,binary_mat_.cols,CV_8UC3);
//		original.copyTo(original_size_mat(bound_rect),binary_mat_(bound_rect));
//		computePCA(contour_,original_size_mat);
//		imshow("PCA",original_size_mat);
//		waitKey(0);
//	}
//	else
//		computePCA(contour_);
	computeFeatures();

}


/*
 *
 *
 *
 * //the segment in the original colour in reduced size
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
 *
 *
 */

void Segment::reset(Mat& original, Mat& segment_mat, Mat& binary_original,
		vector<Point2i>& contour, Rect& rect, int segment_size, Vec3b colour)
		 {

	random_colour_mat_ = segment_mat;
	binary_mat_ = binary_original;
	contour_ = contour;
	random_colour_ = colour;
	bound_rect= rect;
	pcl_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	segment_size_= segment_size;


	pca_data_ = Mat::zeros(1, 4, CV_32FC1);
	huMat_ = Mat::zeros(1, 7, CV_32FC1);
	mat_original_colour_ = Mat::zeros(original.rows, original.cols, CV_8UC3);
	binary_mat_ = binary_mat_ > 0;
	original.copyTo(mat_original_colour_, binary_mat_(bound_rect));



}

Segment::~Segment() {

	//cout <<" deleting Segment::~Segment()"<<endl;
}

Segment::Segment(const Segment &obj) :
		class_label(
				obj.class_label), bound_rect(obj.bound_rect), pcl_cloud_(
				new pcl::PointCloud<pcl::PointXYZRGB>(*obj.pcl_cloud_)), segment_size_(
				obj.segment_size_) {


	visualFeatures_ = obj.visualFeatures_.clone();
	mat_original_colour_ = 	obj.mat_original_colour_.clone();
	random_colour_mat_ = 	obj.random_colour_mat_.clone();
	binary_mat_ = obj.binary_mat_.clone();
	contour_ =	obj.contour_;
	random_colour_ = obj.random_colour_;
	original_ = obj.original_.clone();
	histImage_ =obj.histImage_.clone();
	h_hist = obj.h_hist.clone();
	s_hist = obj.s_hist.clone();
	v_hist = obj.v_hist.clone();
	mask_ = obj.mask_.clone();

	// body of constructor
	pca_data_ = Mat::zeros(1, 4, CV_32FC1);
	huMat_ = Mat::zeros(1, 7, CV_32FC1);
}

void Segment::addPoint(const Point2i& point, const Vec3b& colour) {

	mat_original_colour_.at<Vec3b>(point.x, point.y) = colour;
	points.push_back(point);

}

void Segment::colour_this_segment(Mat& dst) {
	for (Point2i& p : points) {
		dst.at<Vec3b>(p.x, p.y) = random_colour_;
	}

}

Segment operator+(const Segment& segment_1,const Segment& segment_2)
{
    Segment added_segment(segment_1);
    added_segment.binary_mat_ += segment_2.binary_mat_;
    added_segment.mat_original_colour_ += segment_2.mat_original_colour_;
    added_segment.random_colour_mat_ += segment_2.random_colour_mat_;


    //recompute the contour
    vector<vector<Point> > contours;
    //compute_contours(added_segment.binary_mat_,contours);
    added_segment.contour_ = contours[0];


    return added_segment;
}

/*
 * !brief Masks the input point_cloud and normals according to the
 * pixels that are active for this segment. The point_cloud is
 * shifted to its gravity center, and its minimum bounding box
 * is found and stored in the member variable dimensions3DMat_
 */
void Segment::add_precomputed_pcl(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
		pcl::PointCloud<pcl::Normal>::Ptr& normals) {

	Utils utils;
	Mat mask;

	//sub-sample the point cloud
	//utils.sub_sample(pcl_cloud,pcl_cloud);


	resize(binary_mat_, mask, Size(pcl_cloud->width, pcl_cloud->height));
	utils.mask_to_pcl_indices(mask, pcl_indices_);
	pcl_cloud_.reset(
			new pcl::PointCloud<pcl::PointXYZRGB>(*pcl_cloud.get(),
					pcl_indices_));


	utils.prune_pcl(pcl_cloud_,pcl_cloud_);


	//normals_.reset(
	//		new pcl::PointCloud<pcl::Normal>(*normals.get(), pcl_indices_));


	double max_z = 0;
	Point3d gravity_center;
	utils.xyz_gravity_center(pcl_cloud_, gravity_center, max_z);
	//shift the point cloud to the origin
	utils.subtract_gravity_center(pcl_cloud_, gravity_center);
	Point3d dimensions_3d(0.,0.,0.);

	const double normalisation_3d = 100.;
	utils.compute_bounding_box(pcl_cloud_,dimensions_3d);
	dimensions3DMat_.create(1,3,CV_32FC1);
	dimensions3DMat_.at<float>(0,0) = dimensions_3d.x/normalisation_3d;
	dimensions3DMat_.at<float>(0,1) = dimensions_3d.y/normalisation_3d;
	dimensions3DMat_.at<float>(0,2) = dimensions_3d.z/normalisation_3d;

}

void Segment::addPcl(Mat &original_img, Mat& original_depth) {
	//obtain the segments' mask in the orginal image size
	Mat mask = binary_mat_.clone();
	resize(mask, mask, original_img.size());
	Mat pointsMat;
	cv::findNonZero(mask, pointsMat);
	//get the bounding rectangle in the original image size
	Rect minRectOriginalSize = boundingRect(pointsMat);

	//cout <<"::addPcl  minRectOriginalSize="<<minRectOriginalSize<<endl;

	//get the masked colour and depth images
	Mat tmp_img, tmp_depth, cropped_img, cropped_depth;
	original_img.copyTo(tmp_img, mask);
	original_depth.copyTo(tmp_depth, mask);
	Utils utils;
	cropped_img = tmp_img(minRectOriginalSize);
//	cout <<"original_depth.size()="<<original_depth.size()<<endl;
//	imshow("mask",mask);
//		imshow("binary_mat_",binary_mat_);
//		imshow("random_colour_mat_",random_colour_mat_);
//		imshow("masked img",tmp_img);
//		waitKey(0);
	cropped_depth = tmp_depth(minRectOriginalSize);

	//get the point cloud
	int cx = original_img.cols / 2;
	int cy = original_img.rows / 2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	utils.image_to_pcl(cropped_img, cropped_depth, pcl_cloud, cx, cy,
			minRectOriginalSize);
	string text("point cloud");
	cout <<"Segment::addPcl points="<<pcl_cloud->points.size()<<endl;

	//call the addPcl method that will prune the point cloud and compute features
	addPcl(pcl_cloud);

}

void Segment::addPcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud) {
	Eigen::Matrix3f eigenVectors;
	Point3d gravity_center;
	string text("centred point cloud");

	Utils utils;
	//remove points that are not meaningful
	pcl_cloud_ = pcl_cloud;
	//utils.prune_pcl(pcl_cloud,pcl_cloud_);

	double max_z = 0;
	utils.xyz_gravity_center(pcl_cloud_, gravity_center, max_z);
	//shift the point cloud to the origin
	utils.subtract_gravity_center(pcl_cloud_, gravity_center);
	Point3d dimensions_3d(0.,0.,0.);


	dimensions3DMat_.create(1,3,CV_32FC1);
	dimensions3DMat_.at<float>(0,0) = dimensions_3d.x;
	dimensions3DMat_.at<float>(0,1) = dimensions_3d.y;
	dimensions3DMat_.at<float>(0,2) = dimensions_3d.z;
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;

	//utils.compute_vfh(pcl_cloud_, normals_, vfhMat_);


	//compute a Mat with statistics about the 3D points

}

/*
 * !brief Computes the available features
 * for this segment:
 * - HSV histogram (1 x [3*NHISTOGRAMBINS] = 1 x 96)
 * - Hu moments (1x7)
 * - PCA (1 x 4)
 * and stores the results as a 1xN matrix in visualFeatures_
 *
 *
 */
void Segment::computeFeatures() {

	if(dimensions3DMat_.cols != 0){

		computeHistogram();
		computeHuMoments();
		//cout <<"dimensions3DMat_="<<dimensions3DMat_<<endl;
		visualFeatures_ = Mat(1,
				NUMBER_VISUAL_FEATS + huMat_.cols /*+ dimensions3DMat_.cols*/, CV_32FC1);
		vector<Mat> vectorFeats = { h_hist.t(), s_hist.t(), v_hist.t(), huMat_  /*,
				dimensions3DMat_ */};
		hconcat(vectorFeats, visualFeatures_);
	}
	else if (vfhMat_.cols != 0) {
		computeHistogram();
		computeHuMoments();
		//cout <<"vfhMat_="<<vfhMat_<<endl;
		visualFeatures_ = Mat(1,
				NUMBER_VISUAL_FEATS + huMat_.cols + vfhMat_.cols, CV_32FC1);
		vector<Mat> vectorFeats = { h_hist.t(), s_hist.t(), v_hist.t(), huMat_,
				vfhMat_ };
		hconcat(vectorFeats, visualFeatures_);
	} else {
		computeHistogram();
		computePCA(contour_);
		computeHuMoments();
		visualFeatures_ = Mat(1, NUMBER_VISUAL_FEATS+7, CV_32FC1);
		vector<Mat> vectorFeats = { h_hist.t(), s_hist.t(), v_hist.t(),
				/*pca_data_,*/ huMat_ };
		hconcat(vectorFeats, visualFeatures_);
	}

}

void Segment::computeHuMoments() {

	Moments moment = cv::moments(mask_, true);
	double hu[7];
	cv::HuMoments(moment, hu);
	for (int i = 0; i < 7; i++)
		huMat_.at<float>(0, i) = hu[i];
}

void Segment::set_original_mat_segment_mat(){
	//cv::imshow("random_colour_mat_",random_colour_mat_);
	//cv::imshow("mat_original_colour_",mat_original_colour_);
	//cv::waitKey(0);
	mat_original_colour_ = random_colour_mat_;

}

void Segment::computeHistogram() {
	assert(mat_original_colour_.data);

	set_original_mat_segment_mat();
	//convert to HSV
	cv::Mat hsv_mat;
	cv::cvtColor(mat_original_colour_, hsv_mat, CV_BGR2HSV);

	mask_ = Mat::zeros(mat_original_colour_.rows, mat_original_colour_.cols,
			CV_8UC1);
	//threshold grayscale to binary image
	cv::cvtColor(mat_original_colour_, mask_, CV_BGR2GRAY);
	mask_ = mask_ > 0;
	//imshow("original_",original_);
	//waitKey(0);
	/// Separate the image in 3 places ( B, G and R )
	vector<Mat> hsv_planes;
	split(hsv_mat, hsv_planes);

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
	calcHist(&hsv_planes[0], 1, 0, mask_, h_hist, 1, &histSize, &histRange_h,
			uniform, accumulate);
	calcHist(&hsv_planes[1], 1, 0, mask_, s_hist, 1, &histSize, &histRange_sv,
			uniform, accumulate);
	calcHist(&hsv_planes[2], 1, 0, mask_, v_hist, 1, &histSize, &histRange_sv,
			uniform, accumulate);

//		//remove NaNs
	Mat mask_h = Mat(h_hist != h_hist);
	h_hist.setTo(0,mask_h);
	Mat mask_s = Mat(s_hist != s_hist);
	s_hist.setTo(0,mask_s);
	Mat mask_v = Mat(v_hist != v_hist);
	v_hist.setTo(0,mask_v);

	// Draw the histograms for H, S and V
	int hist_w = 120;
	int hist_h = 100;
	int bin_w = cvRound((double) hist_w / histSize);

	histImage_ = Mat(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

	/// Normalize the result to [ 0,1]
	normalize(h_hist, h_hist, 0, 1, cv::NORM_MINMAX, -1, Mat());
	normalize(s_hist, s_hist, 0, 1, cv::NORM_MINMAX, -1, Mat());
	normalize(v_hist, v_hist, 0, 1, cv::NORM_MINMAX, -1, Mat());

	if (DEBUG) {
		/// Normalize the result to [ 0, histImage.rows ]
		normalize(h_hist, h_hist, 0, histImage_.rows, cv::NORM_MINMAX, -1, Mat());
		normalize(s_hist, s_hist, 0, histImage_.rows, cv::NORM_MINMAX, -1, Mat());
		normalize(v_hist, v_hist, 0, histImage_.rows, cv::NORM_MINMAX, -1, Mat());
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
		cv::imshow("histImage_",histImage_);
		cv::waitKey(0);

	}

}

void Segment::drawAxis(Mat& img, Point p, Point q, Scalar colour,
		const float scale = 0.2) {
	double angle;
	double hypotenuse;
	angle = atan2((double) p.y - q.y, (double) p.x - q.x); // angle in radians
	hypotenuse = sqrt(
			(double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
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

void Segment::computePCA(const vector<Point> &pts) {
	//Construct a buffer used by the pca analysis
	int sz = static_cast<int>(pts.size());
	Mat data_pts = Mat(sz, 2, CV_64FC1);
	for (int i = 0; i < data_pts.rows; ++i) {
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

	for (int i = 0; i < 2; ++i) {
		eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
				pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
	}
	orientations_[0] = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
	orientations_[0] = (orientations_[0] / M_PI * 180)
			+ (orientations_[0] > 0 ? 0 : 360);
	orientations_[1] = atan2(eigen_vecs[1].y, eigen_vecs[1].x); // orientation in radians
	orientations_[1] = (orientations_[1] / M_PI * 180)
			+ (orientations_[1] > 0 ? 0 : 360);

	modules_[0] = sqrt(
			eigen_vecs[0].x * eigen_vecs[0].x
					+ eigen_vecs[0].y * eigen_vecs[0].y);
	modules_[1] = sqrt(
			eigen_vecs[1].x * eigen_vecs[1].x
					+ eigen_vecs[1].y * eigen_vecs[1].y);
	pca_data_.at<float>(0, 0) = modules_[0] / binary_mat_.cols;
	pca_data_.at<float>(0, 1) = modules_[1] / binary_mat_.cols;
	pca_data_.at<float>(0, 2) = orientations_[0] / (360.);
	pca_data_.at<float>(0, 3) = orientations_[1] / (360.);

}

double Segment::computePCA(const vector<Point> &pts, Mat &img) {
	//Construct a buffer used by the pca analysis
	int sz = static_cast<int>(pts.size());
	Mat data_pts = Mat(sz, 2, CV_64FC1);
	for (int i = 0; i < data_pts.rows; ++i) {
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
	for (int i = 0; i < 2; ++i) {
		eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
				pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
	}
	orientations_[0] = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
	orientations_[1] = atan2(eigen_vecs[1].y, eigen_vecs[1].x); // orientation in radians

	// Draw the principal components
	circle(img, center_, 3, Scalar(255, 0, 255), 2);
	Point p1 = center_
			+ 0.02
					* Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
							static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
	Point p2 = center_
			- 0.02
					* Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]),
							static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
	drawAxis(img, center_, p1, Scalar(0, 255, 0), 1);
	drawAxis(img, center_, p2, Scalar(255, 255, 0), 5);
	double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
	return angle;
}

void Segment::re_colour(Vec3b new_colour) {
	random_colour_ = new_colour;
	random_colour_mat_.setTo(random_colour_, mask_);
}

bool Segment::part_of(Segment* other_segment) {
	return false;
}
