/*
 * load_data.h
 *
 *  Created on: Dec 4, 2014
 *      Author: phanthanh
 */
#ifndef LOAD_DATA_H_
#define LOAD_DATA_H_

#include "object_recognition.h"
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include "typedef.h"

#include <string>
#include <fstream>
#include <vector>

ObjectRecognitionParameters loadPassthroughParams(int argc, char** argv) {
	ifstream params_file_stream;
	ObjectRecognitionParameters params;
	std::string fileName;
	pcl::console::parse_argument(argc, argv, "--params", fileName);
	params_file_stream.open(fileName.c_str());
	if (params_file_stream.is_open()) {
		params_file_stream >> params.min_depth;
		params_file_stream >> params.max_depth;
		params_file_stream >> params.min_x;
		params_file_stream >> params.max_x;
		params_file_stream >> params.min_y;
		params_file_stream >> params.max_y;

		params_file_stream.close();

	} else {
		pcl::console::print_info("Could not open passthrough params file.");

	}
	return params;
}


ObjectRecognitionParameters loadParams(int argc, char** argv) {
	ifstream params_file_stream;
	ObjectRecognitionParameters params;

	std::string global_file_name;
	pcl::console::parse_argument(argc, argv, "--global", global_file_name);
	params_file_stream.open(global_file_name.c_str());
	if (params_file_stream.is_open()) {
		params_file_stream >> params.global_max_registration;
		params_file_stream >> params.global_initial;
		params_file_stream >> params.global_icp;
		params_file_stream >> params.global_downsample;

		params_file_stream.close();

	} else {
		pcl::console::print_info("Could not open global params file.");
	}

	std::string filter_file_name;
	pcl::console::parse_argument(argc, argv, "--filter", filter_file_name);
	params_file_stream.open(filter_file_name.c_str());
	if (params_file_stream.is_open()) {
		params_file_stream >> params.min_depth;
		params_file_stream >> params.max_depth;
		params_file_stream >> params.min_x;
		params_file_stream >> params.max_x;
		params_file_stream >> params.min_y;
		params_file_stream >> params.max_y;
		params_file_stream >> params.downsample_leaf_size;
		params_file_stream >> params.min_dowsample_points;
		params_file_stream.close();

	} else {
		pcl::console::print_info("Could not open filter params file.");
	}

	std::string feature_file_name;
	pcl::console::parse_argument(argc, argv, "--feature", feature_file_name);
	params_file_stream.open(feature_file_name.c_str());
	if (params_file_stream.is_open()) {
		params_file_stream >> params.surface_normal_radius;
		params_file_stream >> params.keypoints_min_scale;
		params_file_stream >> params.keypoints_nr_octaves;
		params_file_stream >> params.keypoints_nr_scales_per_octave;
		params_file_stream >> params.keypoints_min_contrast;
		params_file_stream >> params.local_descriptor_radius;
		params_file_stream.close();

	} else {
		pcl::console::print_info("Could not open feature params file.");
	}

	std::string init_file_name;
	pcl::console::parse_argument(argc, argv, "--init", init_file_name);
	params_file_stream.open(init_file_name.c_str());
	if (params_file_stream.is_open()) {
		params_file_stream >> params.initial_alignment_min_sample_distance;
		params_file_stream
				>> params.initial_alignment_max_correspondence_distance;
		params_file_stream >> params.initial_alignment_nr_iterations;
		params_file_stream.close();

	} else {
		pcl::console::print_info("Could not open initial params file.");
	}
	std::string refine_file_name;
	pcl::console::parse_argument(argc, argv, "--refine", refine_file_name);
	params_file_stream.open(refine_file_name.c_str());
	if (params_file_stream.is_open()) {
		params_file_stream >> params.icp_max_correspondence_distance;
		params_file_stream >> params.icp_outlier_rejection_threshold;
		params_file_stream >> params.icp_transformation_epsilon;
		params_file_stream >> params.icp_max_iterations;
		params_file_stream.close();

	} else {
		pcl::console::print_info("Could not open initial params file.");
	}

	params.display();
	return params;

}
int loadData(int argc, char **argv,
		std::vector<PCD> &models,
		ObjectRecognitionParameters oParams) {
	if (argc < 1)
		return 0;
	else {
		string fileName;
		ifstream input(argv[1]);
		int numFile = 0;
		while (input >> fileName) {
			numFile++;
			cout << "fileName: " << fileName << endl;
			PCD m(oParams);
			int indexOfSlash = fileName.find_last_of("/\\");
			m.f_name = fileName.substr(indexOfSlash + 1);
			cout << "f_name: " << m.f_name << endl;
			pcl::io::loadPCDFile(fileName, *m.cloud);
			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

			models.push_back(m);

		}
		input.close();
		return numFile;
	}
}

#endif /* LOAD_DATA_H_ */
