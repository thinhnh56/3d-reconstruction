/*
 * visualization.h
 *
 *  Created on: Dec 6, 2014
 *      Author: phanthanh
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include "typedef.h"
#include <pcl/visualization/cloud_viewer.h>

void view(PointCloud & cloud) {
	pcl::visualization::CloudViewer viewer1("Cloud Viewer");
	viewer1.showCloud(cloud.makeShared());
	while (!viewer1.wasStopped())
		;

	return;
}





#endif /* VISUALIZATION_H_ */
