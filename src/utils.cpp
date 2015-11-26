/*
 * utils.cpp
 *
 *  Created on: 6 Nov 2015
 *      Author: martin
 */

#include "utils.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>


/*
 *
 * double threshold = 0.01;//0.05;

	if(argc < 4){
		cout <<"Usage: $> ./segmenter <input img> <output path> <scales> [<use gpu, default 0=false>]"<< endl;
	}

	original_img = cv::imread(argv[1], -1);
	std::string outputPath(argv[2]);
	int scales = atoi(argv[3]);
	int starting_scale = 0;
	int gpu = 0;
	if(argc == 5){
		gpu = atoi(argv[4]);
	}
	else if(argc == 6){
		gpu = atoi(argv[4]);
		threshold = atof(argv[5]);
	}
 *
 *
 */

std::string Utils::remove_extension(const std::string& filename) {
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot);
}

string Utils::get_file_name(const string& s) {

	char sep = '/';

	size_t i = s.rfind(sep, s.length());
	if (i != string::npos) {
		return (s.substr(i + 1, s.length() - i));
	}

	return ("");
}

int Utils::parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale, int& gpu,string& img_path, string& output_path) {


	/** Define and parse the program options
	 */
	namespace po = boost::program_options;
	po::options_description desc("Options");
	int frame = -1;
	string imagePath = "", depthPath = "", segmentsPath = "", outputPath = "",
			iorPath = "", tdPath = "", cfg_path = "";
	string salMode = "", segMode = "";
	desc.add_options()("help", "Print help messages")

	("scales,s",
			po::value<int>(&scales), "Number of scales")
	("gpu,g",
				po::value<int>(&gpu), "Enable or disable GPU")
	("starting_scale,r",
				po::value<int>(&starting_scale), "Starting scale")
	("propagation_scale,p",
					po::value<int>(&propagation_scale), "Scale used for propagating the labels")

	("threshold,t",
				po::value<double>(&thres), "Gradient threshold value")

	("image,i", po::value<string>(&img_path), "Path to the input image")

	("output,o", po::value<string>(&output_path)->required(),
			"path to the output files");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Basic Command Line Parameter App" << std::endl << desc
					<< std::endl;
			return SUCCESS;
		}

		po::notify(vm);  // throws on error, so do after help in case
						 // there are any problems
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << desc << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	return 0;
}



int Utils::parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale, int& gpu,string& img_path, string& output_path, string& svm_path) {


	/** Define and parse the program options
	 */
	namespace po = boost::program_options;
	po::options_description desc("Options");
	int frame = -1;
	string imagePath = "", depthPath = "", segmentsPath = "", outputPath = "",
			iorPath = "", tdPath = "", cfg_path = "";
	string salMode = "", segMode = "";
	desc.add_options()("help", "Print help messages")

	("scales,s",
			po::value<int>(&scales), "Number of scales")
	("gpu,g",
				po::value<int>(&gpu), "Enable or disable GPU")
	("starting_scale,r",
				po::value<int>(&starting_scale), "Starting scale")
	("propagation_scale,p",
					po::value<int>(&propagation_scale), "Scale used for propagating the labels")

	("threshold,t",
				po::value<double>(&thres), "Gradient threshold value")

	("image,i", po::value<string>(&img_path), "Path to the input image")

	("svm,v", po::value<string>(&svm_path), "Path to the SVM model")

	("output,o", po::value<string>(&output_path)->required(),
			"path to the output files");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Basic Command Line Parameter App" << std::endl << desc
					<< std::endl;
			return SUCCESS;
		}

		po::notify(vm);  // throws on error, so do after help in case
						 // there are any problems
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << desc << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	return 0;
}

int Utils::parse_args(int argc, char **argv, double& thres, int& scales, int& starting_scale, int& propagation_scale, int& gpu,string& img_path, string& output_path, string& svm_path, string& cloud_path) {


	/** Define and parse the program options
	 */
	namespace po = boost::program_options;
	po::options_description desc("Options");
	int frame = -1;
	string imagePath = "", depthPath = "", segmentsPath = "", outputPath = "",
			iorPath = "", tdPath = "", cfg_path = "";
	string salMode = "", segMode = "";
	desc.add_options()("help", "Print help messages")

	("scales,s",
			po::value<int>(&scales), "Number of scales")
	("gpu,g",
				po::value<int>(&gpu), "Enable or disable GPU")
	("starting_scale,r",
				po::value<int>(&starting_scale), "Starting scale")
	("propagation_scale,p",
					po::value<int>(&propagation_scale), "Scale used for propagating the labels")

	("threshold,t",
				po::value<double>(&thres), "Gradient threshold value")

	("image,i", po::value<string>(&img_path), "Path to the input image")

	("clouds,c", po::value<string>(&cloud_path), "Path to the input pcls")

	("svm,v", po::value<string>(&svm_path), "Path to the SVM model")


	("output,o", po::value<string>(&output_path)->required(),
			"path to the output files");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Basic Command Line Parameter App" << std::endl << desc
					<< std::endl;
			return SUCCESS;
		}

		po::notify(vm);  // throws on error, so do after help in case
						 // there are any problems
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << desc << std::endl;
		return ERROR_IN_COMMAND_LINE;
	}

	return 0;
}

void Utils::merge_two_bounding_rects(Rect& rec1,Rect& rec2, Rect& res){

	int min_x = rec1.x < rec2.x ? rec1.x : rec2.x;
	int min_y = rec1.y < rec2.y ? rec1.y : rec2.y;

	int r1_max_x = rec1.x + rec1.width;
	int r2_max_x = rec2.x + rec2.width;

	int r1_max_y = rec1.y + rec1.height;
	int r2_max_y = rec2.y + rec2.height;

	int width = r1_max_x > r2_max_x ? r1_max_x-min_x : r2_max_x-min_x;
	int height = r1_max_y > r2_max_y ? r1_max_y-min_y : r2_max_y-min_y;

	res.x = rec1.x < rec2.x ? rec1.x : rec2.x;
	res.y = rec1.y > rec2.y ? rec1.y : rec2.y;

	res.width = width;
	res.height = height;



}

void Utils::image_to_pcl(Mat& img, Mat& depth,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud, int cx, int cy, Rect& rect) {

	// explicitly specify dsize=dst.size(); fx and fy will be computed from that.
	if (img.size() != depth.size())
		resize(depth, depth, img.size(), 0, 0, INTER_CUBIC);

	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = img.cols;
	pcl_cloud->height = img.rows;
	pcl_cloud->points.resize(pcl_cloud->height * pcl_cloud->width);

	//int* depth_data = new int[pcl_cloud->height * pcl_cloud->width];
	//copy the depth values of every pixel in here

	//register float constant = 1.0f / 525;
	float constant = 1.0f / 1368.3;
	//  register int centerX = (pcl_cloud->width >> 1);
	//  int centerY = (pcl_cloud->height >> 1);
	register int centerX = (pcl_cloud->width / 2);
	int centerY = (pcl_cloud->height / 2);
	cout << " centerX: " << centerX << " centerY: " << centerY << endl;

	register int depth_idx = 0;

	for (int u = 0; u < img.rows; ++u)
		  for (int v = 0; v < img.cols; ++v, ++depth_idx)
		  {
			  pcl::PointXYZRGB& pt = pcl_cloud->points[depth_idx];
			float depthvalue = depth.at<float>(u, v );
			//cout <<"depth value="<<depthvalue<<endl;
			pt.z = depthvalue;	//* 1000.0f;//depth_data[depth_idx] * 0.001f;
			pt.x = static_cast<float>(v+rect.x-cx) * pt.z * constant;
			pt.y = static_cast<float>(u+rect.y-cy) * pt.z * constant;
			Vec3b& colour = img.at<Vec3b>(u, v );
			pt.b = colour[0];
			pt.g = colour[1];
			pt.r = colour[2];
		  }
	//for (int v = -centerY; v < centerY; ++v) {
	//	for (register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			//cout <<" accessing u,v="<<u<<v<<endl;

		//}
	//}
	//cout <<"image_to_pcl::pcl_cloud->points.size()="<<pcl_cloud->points.size()<<endl;

}

void Utils::image_to_pcl(Mat& img, Mat& depth,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud) {

	// explicitly specify dsize=dst.size(); fx and fy will be computed from that.
	if (img.size() != depth.size())
		resize(depth, depth, img.size(), 0, 0, INTER_CUBIC);

	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = img.cols;
	pcl_cloud->height = img.rows;
	pcl_cloud->points.resize(pcl_cloud->height * pcl_cloud->width);

	//int* depth_data = new int[pcl_cloud->height * pcl_cloud->width];
	//copy the depth values of every pixel in here

	// register float constant = 1.0f / 525;
	float constant = 1.0f / 1368.3;
	//  register int centerX = (pcl_cloud->width >> 1);
	//  int centerY = (pcl_cloud->height >> 1);
	register int centerX = (pcl_cloud->width / 2);
	int centerY = (pcl_cloud->height / 2);
	cout << " centerX: " << centerX << " centerY: " << centerY << endl;

	register int depth_idx = 0;
	for (int v = -centerY; v < centerY; ++v) {
		for (register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			//cout <<" accessing u,v="<<u<<v<<endl;
			pcl::PointXYZRGB& pt = pcl_cloud->points[depth_idx];
			float depthvalue = depth.at<float>(v + centerY, u + centerX);
			//cout <<"depth value="<<depthvalue<<endl;
			pt.z = depthvalue;	//* 1000.0f;//depth_data[depth_idx] * 0.001f;
			pt.x = static_cast<float>(u) * pt.z * constant;
			pt.y = static_cast<float>(v) * pt.z * constant;
			Vec3b& colour = img.at<Vec3b>(v + centerY, u + centerX);
			pt.b = colour[0];
			pt.g = colour[1];
			pt.r = colour[2];
		}
	}
	//cout <<"image_to_pcl::pcl_cloud->points.size()="<<pcl_cloud->points.size()<<endl;

}
