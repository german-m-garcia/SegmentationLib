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

using namespace std;
using namespace cv;

class ObjectDetector {
public:
	ObjectDetector();
	virtual ~ObjectDetector();

	 void add_training_data(vector<Segment*>& foreground_segments,
				vector<Segment*>& background_segments);

	void train();
private:

	//need a bunch of segments for each frame
	vector < vector <Segment*> > segments;
	//need an SVM?
	SVMWrapper svm;
	//a path to where the data can be stored and debugged

};



#endif /* INCLUDE_OBJECTS_OBJECTDETECTOR_H_ */
