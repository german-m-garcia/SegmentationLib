/*
 * Training.h
 *
 *  Created on: 23 Nov 2015
 *      Author: martin
 */

#ifndef INCLUDE_OBJECTS_TRAINING_H_
#define INCLUDE_OBJECTS_TRAINING_H_

#include "segment.h"
#include <vector>
#include <string>

using namespace cv;
using namespace std;

class TrainingExample {

public:
	TrainingExample(const Point2i& center,int frame, string& seq_name,Segment& segment):
		center(center),frame(frame),seq_name(seq_name), seg(segment){
		frame_name = "/frame"+to_string(frame)+".png";
	}


	Point2i center;
	int frame;
	string seq_name;
	Segment seg;
	string frame_name;
};

class Training {
public:
	Training();
	Training(string path);
	void set_path(string& path);
	virtual ~Training();
	void add_training_examples(vector<TrainingExample>& fg_training_examples,vector<TrainingExample>& bg_training_examples);

	void save_training_examples();
private:
	//a vector of training examples
	vector<TrainingExample> fg_training_examples_,bg_training_examples_;
	string seq_name;
	string path_;
};

#endif /* INCLUDE_OBJECTS_TRAINING_H_ */
