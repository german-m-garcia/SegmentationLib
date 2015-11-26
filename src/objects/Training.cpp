/*
 * Training.cpp
 *
 *  Created on: 23 Nov 2015
 *      Author: martin
 */

#include "objects/Training.h"

Training::Training(string path):path_(path) {
	// TODO Auto-generated constructor stub

}

Training::Training() {
	// TODO Auto-generated constructor stub

}

void Training::set_path(string& path){
	path_ = path;
}

Training::~Training() {
	// TODO Auto-generated destructor stub
}

void Training::add_training_examples(vector<TrainingExample>& fg_training_examples,vector<TrainingExample>& bg_training_examples){
	fg_training_examples_.insert( fg_training_examples_.end(), fg_training_examples.begin(), fg_training_examples.end() );
	bg_training_examples_.insert( bg_training_examples_.end(), bg_training_examples.begin(), bg_training_examples.end() );

}

void Training::save_training_examples(){
	for(TrainingExample& example: fg_training_examples_){
		string outpath=path_+"/"+example.frame_name;
		cout <<">Training::save_training_examples: saving at "<<outpath<<endl;
		imwrite(outpath,example.seg.getRandomColourMat());
	}
}

