/*
 * segmenter.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: martin
 */

#include "segmentation.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	Mat img0, dst;

	Segmentation segmentation;
	if (argc < 3) {
		cout
				<< "Usage: $> ./segmenter <input img> <output path>"
				<< endl;
		return 0;
	}

	img0 = cv::imread(argv[1], -1);
	std::string outputPath(argv[2]);
	double sp = 15.;
	double sr = 15.;
	double min_size = 15;
	segmentation.preprocess(false,img0,2);
	segmentation.mean_shift(img0, sp, sr, min_size, dst);
	imwrite(outputPath, dst);

}

