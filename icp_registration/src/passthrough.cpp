/*
 * passthrough.cpp
 *
 *  Created on: Dec 7, 2014
 *      Author: phanthanh
 */

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "include/load_data.h"
#include "include/object_recognition.h"
#include "include/typedef.h"
#include "include/filter.h"
#include "include/visualization.h"
#include "include/feature_estimation.h"
#include "include/registration.h"

#include "ctime"

time_t start, end;

using namespace std;

int main(int argc, char** argv) {

	ObjectRecognitionParameters params;

	// Input foramt: filterInputList.txt --params filterParams.txt

	if (argc < 4) {
		cout << "Syntax is : inputList.txt --params filterParams.txt"
				<< endl;
		return 0;
	}

	params = loadPassthroughParams(argc, argv);


	std::vector<PCD> data;
	pcl::console::print_info("Loading data ...");
	time(&start);
	loadData(argc, argv, data, params);
	time(&end);

	PointCloud::Ptr filtered_cloud(new PointCloud);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	for (int i = 0; i < data.size(); i++) {
		cout << "filter input " << i << endl;
		std::ostringstream fileName;
		fileName << "./fitered_data/filtered" << i<< ".pcd";
		filtered_cloud = thresholdDepth(data[i].cloud, params.min_depth, params.max_depth);
		pcl::io::savePCDFile(fileName.str(), *filtered_cloud);

	}
	time(&end);
	pcl::console::print_info("takes %.2lf seconds.", difftime(end, start));
	return 0;
}

