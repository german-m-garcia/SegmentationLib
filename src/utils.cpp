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

	("image,i", po::value<string>(&img_path), "Path to the input image")(
			"output,o", po::value<string>(&output_path)->required(),
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
