#ifndef _VISUALIZATION_
#define _VISUALIZATION_

#include "typedefs.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

void view( PointCloudRGB & cloud );

void showCloudsLeft(const PCD cloud_target, const PCD cloud_source, pcl::visualization::PCLVisualizer *v, int vp1);

void showCloudsRight(const PointCloudNormal::Ptr cloud_target, const PointCloudNormal::Ptr cloud_source, pcl::visualization::PCLVisualizer *v, int vp2);



#endif