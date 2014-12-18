/*
 * test_load_feature.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: phanthanh
 */
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "include/load_data.h"
#include "include/object_recognition.h"
#include "include/typedef.h"
#include "include/filter.h"

#include "ctime"
time_t start,end;

int main(int argc, char** argv) {
	if (argc < 10) {
		pcl::console::print_info(
				"Syntax is: inputList.txt --filter filterParams.txt --feature featureParams.txt --init initialParams.txt --refine refineParams.txt");
		return 1;
	}
	ObjectRecognitionParameters params = loadParams(argc, argv);
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	time(&start);
	pcl::console::print_info("Loading data ...");
	loadData(argc, argv, data, params);
	time(&end);
	pcl::console::print_info("take %.2lf seconds.\n",
			difftime(end, start));
	cout << data[0].cloud->points.size()<< endl;

	pcl::console::print_info("Calculating Keypoints...\n");
	time(&start);
	pcl::console::print_info("Keypoints: %d \n",
			data[0].features.getKeypoints()->points.size());
	data[0].features.getKeypoints(true);
	time(&end);
	pcl::console::print_info("take %.2lf seconds.\n",
			difftime(end, start));

	pcl::console::print_info("Calculating LocalDescriptor...\n");
	time(&start);
	pcl::console::print_info("Keypoints: %d \n",
			data[0].features.getLocalDescriptors()->points.size());
	time(&end);
	pcl::console::print_info("take %.2lf seconds.\n",
			difftime(end, start));

	/*pcl::console::print_info("Calculating normals cur...\n");
	time(&start);
	pcl::console::print_info("Normal Curvature: %d \n",
			data[0].features.getSurfaceCurvatureNormals()->points.size());
	time(&end);
	pcl::console::print_info("take %.2lf seconds.\n",
			difftime(end, start));
*/

	return 0;

}

