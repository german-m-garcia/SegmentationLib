/*
 * Wrapper.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: martin
 */

//#include <opencv2/core/core.hpp>
#include "objects/svm_wrapper.h"

using namespace cv;
using namespace std;

SVMWrapper::SVMWrapper() {
	// opencv3.0
	//svm = SVM::create();
	//svm->setType(SVM::C_SVC);
	//svm->setKernel(SVM::RBF);

}

SVMWrapper::SVMWrapper(string& model_path): model_path_(model_path){

}

SVMWrapper::~SVMWrapper() {

}

void SVMWrapper::prec_rec(Mat& gt_img, Mat& result_img, double& precision,
		double& recall, double& f1, double&pascal) {

	Scalar retrieved = countNonZero(result_img);
	Scalar total_gt = countNonZero(gt_img);
	Mat intersection = gt_img & result_img;

	Scalar overlap = countNonZero(intersection);
	double union_value = retrieved[0] + total_gt[0] - overlap[0];
	pascal = overlap[0] / union_value;
	precision = overlap[0] / retrieved[0];
	recall = overlap[0] / total_gt[0];
	f1 = 2 * (precision * recall) / (precision + recall);

}

void SVMWrapper::add_training_data(vector<Segment*>& objects,
		vector<Segment*>& background_objects) {


	cout <<"SVMWrapper::objects.size()="<< objects.size()<<" background_objects="<<background_objects.size()<<endl;
	//generate the training Mat
	Mat localTrainingDataMat = objects[0]->visualFeatures_; //(objects.size(), NUMBER_VISUAL_FEATS, CV_32FC1);
	cout <<"localTrainingDataMat="<<localTrainingDataMat<<endl;
	//add positive examples
	for (unsigned int i = 1; i < objects.size(); i++)
		vconcat(localTrainingDataMat, objects[i]->visualFeatures_,
				localTrainingDataMat);

	//add negative examples
	for (unsigned int i = 0; i < background_objects.size(); i++)
		vconcat(localTrainingDataMat, background_objects[i]->visualFeatures_,
				localTrainingDataMat);


	//fill the labels mat
	Mat localLabelsMat(objects.size() + background_objects.size(), 1, CV_32FC1);
	//add positive example labels
	for (unsigned int i = 0; i < objects.size(); i++)
		localLabelsMat.at<float>(i, 0) = +1.0;

	//add negative example labels
	for (unsigned int i = objects.size(); i < objects.size() + background_objects.size();
			i++)
		localLabelsMat.at<float>(i, 0) = 0.0;

	cout << localLabelsMat << endl;
	//attach both labels and training data to the main ones
	if (trainingData.empty())
		trainingData = localTrainingDataMat.clone();
	else
		vconcat(trainingData, localTrainingDataMat, trainingData);
	if (labels.empty())
		labels = localLabelsMat.clone();
	else
	vconcat(labels, localLabelsMat, labels);
}

void SVMWrapper::trainSVM() {
	//normalise the training data column wise
//	for (int i = 0; i < Segment::NUMBER_VISUAL_FEATS; i++) {
//		Rect rect(i, 0, 1, trainingData.rows);
//		Mat feat = trainingData(rect);
//		//normalize(feat1,feat1,-1.0,+1.0,cv::NORM_MINMAX);
//		Scalar meanv, stddev;
//		double min, max;
//		meanStdDev(feat, meanv, stddev);
//		minMaxLoc(feat, &min, &max);
//		cout << "mean= " << meanv << " sttdev= " << stddev << endl;
//
//		means[i] = meanv[0];
//		devs[i] = stddev[0];
//		mins[i] = min;
//		maxs[i] = max;
//		//feat1 -= meanv[0];
//		//feat1 /= stddev[0];
//		feat -= min;
//		feat *= 1.0 / (maxs[i] - mins[i]);
//	}



	//cout <<trainingData<<endl;
	//cout <<labels<<endl;


	// Set up SVM's parameters
	cv::SVMParams params;
	{
	params.svm_type = CvSVM::C_SVC;
	//params.svm_type = CvSVM::EPS_SVR;
	params.kernel_type = CvSVM::RBF;  //CvSVM::RBF, CvSVM::LINEAR ...
	params.degree = 0;  // for poly
	params.gamma = 10;  // for poly/rbf/sigmoid
	params.coef0 = 0;  // for poly/sigmoid

	params.C = 7;  // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
	params.nu = 0.0;  // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
	params.p = 0.1;  // for CV_SVM_EPS_SVR

	params.class_weights = NULL;  // for CV_SVM_C_SVC
	params.term_crit.type = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
	params.term_crit.max_iter = 1000;
	params.term_crit.epsilon = 1e-6;
	}

	// Train the SVM

	cout <<"trainingData.size()="<<trainingData.size()<<" trainingData.type()="<<trainingData.type()<<endl;
	cout <<"labels.size()="<<labels.size()<<" labels.type()="<<labels.type()<<endl;
	svm.train_auto(trainingData, labels, Mat(), Mat(), params, 10);
	svm.save(model_path_.c_str()); //"/home/martin/workspace/VideoSegmentationLib/slc_svm_model.xml");

	//SVM.save("/home/brego/martin/workspace/ObjectDetector/svm_model.xml");

//	OpenCV 3.0
//	Mat mask_nans = Mat(trainingData != trainingData);
//	trainingData.setTo(0.,mask_nans);
//	Ptr<TrainData> data = TrainData::create(trainingData,ROW_SAMPLE,labels);
//	svm->trainAuto(data, 10
//			,SVM::getDefaultGrid(SVM::C),
//			                 //CvParamGrid(0.00001,5000,2),
//			                 //CvSVM::get_default_grid(CvSVM::GAMMA),
//			                 ParamGrid(0.0000001,5000,5),
//			                 SVM::getDefaultGrid(SVM::P),
//			                 SVM::getDefaultGrid(SVM::NU),
//			                 SVM::getDefaultGrid(SVM::COEF),
//			                 SVM::getDefaultGrid(SVM::DEGREE), true);



}

void SVMWrapper::load_model(){
	svm.load(model_path_.c_str());
}

void SVMWrapper::testSVM(vector<Segment*>& test_objects) {

	Mat labelsMat(test_objects.size(), 1, CV_32FC1);
	//iterate over the objects and predict
	for (unsigned int i = 0; i < test_objects.size(); i++) {
//		for (int j = 0; j < Segment::NUMBER_VISUAL_FEATS; j++) {
//			//objects[i]->visualFeatures.at<float>(0,j) -= means[i];
//			//objects[i]->visualFeatures.at<float>(0,j) /= devs[i];
//			test_objects[i]->visualFeatures.at<float>(0, j) -= mins[j];
//			test_objects[i]->visualFeatures.at<float>(0, j) *= 1.0
//					/ (maxs[j] - mins[j]);
//
//		}
		//cout << test_objects[i]->visualFeatures << " ";

		float response = -svm.predict(test_objects[i]->visualFeatures_, true);
		// OpenCV 3.0
		//		float response = svm->predict(test_objects[i]->visualFeatures);
		test_objects[i]->setClassLabel(response);
		//cout << "predicted label=" << response << endl;


	}

}

//void SVMWrapper::processGT(vector<Segment*>& objects, Wrapper& wrapper,
//		Mat& gray_gt, Mat& colour_gt) {

//	//extracts the different objects in the ground truth image
//	vector<int> gt_ids;
//	vector<Mat> vectorGtMats;
//	wrapper.getObjectIds(gray_gt, gt_ids);
//	for (int i = 0; i < gt_ids.size(); i++) {
//		Mat gti;
//		wrapper.extractIdMat(gray_gt, gti, gt_ids[i]);
//		vectorGtMats.push_back(gti);
//	}
//
//	//now iterate for each object in the ground truth
//	vector<bool> gt_id_matched;
//	wrapper.evaluateObjects(gt_ids, vectorGtMats, objects, gt_id_matched);
//
//	//generate the training Mat
//	Mat trainingDataMat = objects[0]->visualFeatures; //(objects.size(), NUMBER_VISUAL_FEATS, CV_32FC1);
//	for (int i = 1; i < objects.size(); i++)
//		vconcat(trainingDataMat, objects[i]->visualFeatures, trainingDataMat);
//
//	//normalise the training data column wise
//	double means[NUMBER_VISUAL_FEATS];
//	double devs[NUMBER_VISUAL_FEATS];
//	double mins[NUMBER_VISUAL_FEATS];
//	double maxs[NUMBER_VISUAL_FEATS];
//	for (int i = 0; i < NUMBER_VISUAL_FEATS; i++) {
//		Rect rect(i, 0, 1, trainingDataMat.rows);
//		Mat feat1 = trainingDataMat(rect);
//		//normalize(feat1,feat1,-1.0,+1.0,cv::NORM_MINMAX);
//		Scalar meanv, stddev;
//		double min, max;
//		meanStdDev(feat1, meanv, stddev);
//		minMaxLoc(feat1, &min, &max);
//		cout << "mean= " << meanv << " sttdev= " << stddev << endl;
//
//		means[i] = meanv[0];
//		devs[i] = stddev[0];
//		mins[i] = min;
//		maxs[i] = max;
//		//feat1 -= meanv[0];
//		//feat1 /= stddev[0];
//		feat1 -= min;
//		feat1 *= 1.0 / (maxs[i] - mins[i]);
//	}
//
//	//fill the labels mat
//	Mat labelsMat(objects.size(), 1, CV_32FC1);
//	for (int i = 0; i < objects.size(); i++) {
//		float label = objects[i]->pascal_score > 0.500 ? 1.0 : -1.0;
//		labelsMat.at<float>(i, 0) = label;
//	}
//
//	cout << "trainingDataMat: " << endl;
//	for (int i = 0; i < trainingDataMat.rows; i++) {
//		for (int j = 0; j < trainingDataMat.cols; j++) {
//			cout << " " << trainingDataMat.at<float>(i, j);
//		}
//		cout << " label=" << labelsMat.at<float>(i, 0) << endl;
//	}
//
//	// Set up SVM's parameters
//	CvSVMParams params;
//	{
//		params.svm_type = CvSVM::C_SVC;
//		//params.svm_type = CvSVM::EPS_SVR;
//		params.kernel_type = CvSVM::RBF;  //CvSVM::RBF, CvSVM::LINEAR ...
//		params.degree = 0;  // for poly
//		params.gamma = 10;  // for poly/rbf/sigmoid
//		params.coef0 = 0;  // for poly/sigmoid
//
//		params.C = 7;  // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
//		params.nu = 0.0; // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
//		params.p = 0.1;  // for CV_SVM_EPS_SVR
//
//		params.class_weights = NULL;  // for CV_SVM_C_SVC
//		params.term_crit.type = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
//		params.term_crit.max_iter = 1000;
//		params.term_crit.epsilon = 1e-6;
//	}
//
//	// Train the SVM
//
//	//SVM.train(trainingDataMat, labelsMat, Mat(), Mat(), params);
//	SVM.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), params, 10);
//	SVM.save("/home/brego/martin/workspace/ObjectDetector/svm_model.xml");
//	params = SVM.get_params();
//	cout << "SVM parameters:  gamma=" << params.gamma << " C=" << params.C
//			<< " " << endl;
//	int correct = 0;
//	//iterate over the objects and predict
//	for (int i = 0; i < objects.size(); i++) {
//		for (int j = 0; j < NUMBER_VISUAL_FEATS; j++) {
//			//objects[i]->visualFeatures.at<float>(0,j) -= means[i];
//			//objects[i]->visualFeatures.at<float>(0,j) /= devs[i];
//			objects[i]->visualFeatures.at<float>(0, j) -= mins[j];
//			objects[i]->visualFeatures.at<float>(0, j) *= 1.0
//					/ (maxs[j] - mins[j]);
//
//		}
//		cout << objects[i]->visualFeatures << " ";
//		float response = -SVM.predict(objects[i]->visualFeatures, true);
//		cout << "predicted label=" << response << endl;
//		if (sgn(response) == sgn(labelsMat.at<float>(i, 0)))
//			correct++;
//	}
//	cout << correct / (1. * objects.size()) << " % correct classifications"
//			<< endl;

//}
