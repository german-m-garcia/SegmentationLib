/*
 * ObjectDetector.h
 *
 *  Created on: 16 Nov 2015
 *      Author: martin
 */

#ifndef INCLUDE_OBJECTS_OBJECTDETECTOR_H_
#define INCLUDE_OBJECTS_OBJECTDETECTOR_H_

#include "segment.h"
#include "svm_wrapper.h"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/gpu/kinfu/kinfu.h>


//using namespace std;
//using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PlainCloudptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloudptr;

class ObjectDetector {
public:
	ObjectDetector();
	ObjectDetector(int mode,string model_path);
	virtual ~ObjectDetector();

	 void add_training_data(std::vector<Segment*>& foreground_segments,
			 std::vector<Segment*>& background_segments);

	 void add_training_data(std::vector<Segment*>& foreground_segments,
			 std::vector<Segment*>& background_segments, Cloudptr point_cloud, Mat& cropped_img, Mat& cropped_depth);

	 void test_data(std::vector<Segment*>& test_segments);

	void train();

	void view_examples();

	void align_point_clouds();

	void run_kinfu(float vsz);

	const static int TRAIN_MODE = 0;
	const static int TEST_MODE = 1;
private:

	//need a bunch of segments for each frame
	std::vector < std::vector <Segment*> > segments;
	std::vector <Segment*> full_view_segments;
	//the real apperances for each frame
	std::vector <cv::Mat> frames_;
	std::vector< cv::Mat> depths_;

	std::vector<Cloudptr> point_clouds;
	//need an SVM?
	SVMWrapper svm;
	//a path to where the data can be stored and debugged
	std::string model_path_;
	bool train_, test_;

	const static int dimensions_img = 300;

	void display_cloud(PlainCloudptr& cloud);

	void display_cloud(Cloudptr& cloud);



};



#endif /* INCLUDE_OBJECTS_OBJECTDETECTOR_H_ */
