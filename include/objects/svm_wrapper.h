/*
 * Wrapper.h
 *
 *  Created on: Oct 9, 2014
 *      Author: martin
 */

#ifndef SVMWRAPPER_H_
#define SVMWRAPPER_H_


#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include <string>
#include <fstream>

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include "segment.h"
#include <string>


//using namespace cv::ml;
using namespace std;


/*
 * Sign function
 */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}



class SVMWrapper {
 public:
  SVMWrapper();

  SVMWrapper(string& model_path);

  virtual ~SVMWrapper();

  void load_model();

  void add_training_data(vector<Segment*>& objects,
			vector<Segment*>& background_objects);

  void test_training_data(vector<Segment*>& objects,Mat& gray_gt);

  void testSVM(vector<Segment*>& test_objects);

  void trainSVM();


 private:



  /*
   * SVM training variables
   */
  cv::SVM svm;
  //Ptr<SVM> svm;
  cv::Mat trainingData, labels;
  double means[Segment::NUMBER_VISUAL_FEATS];
  double devs[Segment::NUMBER_VISUAL_FEATS];
  double mins[Segment::NUMBER_VISUAL_FEATS];
  double maxs[Segment::NUMBER_VISUAL_FEATS];

  string model_path_;


  void prec_rec(cv::Mat& gt_img, cv::Mat& result_img, double& precision, double& recall,
                double& f1, double&pascal);


};

#endif /* SVMWRAPPER_H_ */
