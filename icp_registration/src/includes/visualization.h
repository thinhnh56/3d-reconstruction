#ifndef _VISUALIZATION_
#define _VISUALIZATION_

#include "typedefs.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

void view(PointCloudRGB & cloud);

void drawLine(pcl::visualization::PCLVisualizer &vis, PointRGB start_point,
		PointRGB end_point, std::string line_id, double r, double g, double b);

void showCloudsLeft(const PCD cloud_target, const PCD cloud_source,
		pcl::visualization::PCLVisualizer *v, int vp1);

void showCloudsRight(const PointCloudNormalPtr cloud_target,
		const PointCloudNormalPtr cloud_source,
		pcl::visualization::PCLVisualizer *v, int vp2);

#endif
