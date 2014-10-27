#ifndef _FILTER_
#define _FILTER_

#include "typedefs.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>



void downSampleCloud(const PointCloudRGB::Ptr cloud_src,
				PointCloudRGB::Ptr sampled_cloud,
				float VOXEL_GRID_SIZE);
				
void
thresholdDepth ( PointCloudPtr & input, PointCloudPtr filtered, float min_depth, float max_depth);

void
downsampleF ( PointCloudPtr input, PointCloudPtr filtered, float leaf_size);

void
removeOutliers ( PointCloudPtr & input, PointCloudPtr filtered, float radius, int min_neighbors);

void
applyFilters ( PointCloudPtr input, PointCloudPtr filtered, float min_depth, float max_depth, float leaf_size, float radius, 
              float min_neighbors);


#endif