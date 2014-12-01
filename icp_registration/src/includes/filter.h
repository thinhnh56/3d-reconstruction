#ifndef _FILTER_
#define _FILTER_

#include "typedefs.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

void downSampleCloud(const PointCloudRGBPtr cloud_src,
		PointCloudRGBPtr sampled_cloud, float VOXEL_GRID_SIZE);

void
thresholdDepth(PointCloudRGBPtr & input, PointCloudRGBPtr filtered,
		float min_depth, float max_depth);

void
downsample(PointCloudRGBPtr input, PointCloudRGBPtr filtered, float leaf_size);

void
removeOutliers(PointCloudRGBPtr & input, PointCloudRGBPtr filtered,
		float radius, int min_neighbors);

void
applyFilters(PointCloudRGBPtr input, PointCloudRGBPtr filtered,
		float leaf_size);

#endif
