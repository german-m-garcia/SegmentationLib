/*
 * ObjectDetector.cpp
 *
 *  Created on: 16 Nov 2015
 *      Author: martin
 */

#include "objects/ObjectDetector.h"



ObjectDetector::ObjectDetector() {
	// TODO Auto-generated constructor stub

}

void ObjectDetector::train() {
	svm.trainSVM();
}

void ObjectDetector::add_training_data(vector<Segment*>& foreground_segments,
				vector<Segment*>& background_segments){

	svm.add_training_data(foreground_segments,background_segments);
	segments.push_back(foreground_segments);

}

ObjectDetector::~ObjectDetector() {
	// TODO Auto-generated destructor stub
}


