
/*
 * test_keypoints.cpp
 *
 *  Created on: Dec 8, 2014
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
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>


#include <pcl/point_types.h>


using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;

int main(int argc, char** argv){

	// Goal: check the interest points of one point cloud and show them.
	// Input format inputFile --filter  filterParams.txt --feature featureParams.txt
	if(argc <6){
		cout << "Syntax is : inputFile --filter  filterParams.txt --feature featureParams.txt" << endl;
	}
	ObjectRecognitionParameters params = loadParams(argc, argv);
	std::vector<PCD> data;
	loadData(argc, argv, data, params);


	for(int i=0; i<data.size(); i++){
		std::ostringstream stream;
		std::ostringstream rgb_stream;
		stream << "./keypoints_data/" << data[i].f_name << "_keypoints.pcd";
		PointCloud::Ptr cloud(new PointCloud);
		cloud = data[i].features.getKeypoints(true);

		rgb_stream << "./rgb_data/" << data[i].f_name << "_rgb.png";
		pcl::io::savePNGFile(rgb_stream.str(), *data[i].cloud);
		cloud->resize(10);
		for(int j=0; j<cloud->size(); j++){
			cloud->points[j].r = 255;
			cloud->points[j].g = 255;
			cloud->points[j].b = 0;
		}
		std::ostringstream finalStream;
		finalStream << "./rgb_keypoints_data/" << data[i].f_name << "with_keypoints.png";
		PointCloud::Ptr cloudWithKeypoints(new PointCloud);
		*cloudWithKeypoints = *data[i].cloud;
		pcl::copyPointCloud(*cloud, *cloudWithKeypoints);
		pcl::io::savePNGFile(finalStream.str(), *cloudWithKeypoints);
		pcl::io::savePCDFile(stream.str(), *cloud);



	}







	return 0;
}



