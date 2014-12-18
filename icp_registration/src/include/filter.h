/*
 * filter.h
 *
 *  Created on: Dec 4, 2014
 *      Author: phanthanh
 */

#ifndef FILTER_H_
#define FILTER_H_

#include "typedef.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>


PointCloud::Ptr
passthrough (const PointCloud::Ptr & input, char* dimension, float min, float max)
{
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud (input);
  pass_through.setFilterFieldName (dimension);
  pass_through.setFilterLimits (min, max);
  PointCloud::Ptr thresholded (new PointCloud);
  pass_through.filter (*thresholded);

  return (thresholded);
}
PointCloud::Ptr
thresholdDepth (const PointCloud::Ptr & input, float min_depth, float max_depth)
{
  pcl::PassThrough<PointT> pass_through;
  pass_through.setInputCloud (input);
  pass_through.setFilterFieldName ("z");
  pass_through.setFilterLimits (min_depth, max_depth);
  PointCloud::Ptr thresholded (new PointCloud);
  pass_through.filter (*thresholded);

  return (thresholded);
}

/* Use a VoxelGrid filter to reduce the number of points */
PointCloud::Ptr
downsampleF (const PointCloud::Ptr & input, float leaf_size)
{
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud (input);
  voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  PointCloud::Ptr downsampled (new PointCloud);
  voxel_grid.filter (*downsampled);

  return (downsampled);
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
PointCloud::Ptr
removeOutliers (const PointCloud::Ptr & input, float radius, int min_neighbors)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
  radius_outlier_removal.setInputCloud (input);
  radius_outlier_removal.setRadiusSearch (radius);
  radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
  PointCloud::Ptr inliers (new PointCloud);
  radius_outlier_removal.filter (*inliers);

  return (inliers);
}

/* Apply a series of filters (threshold depth, downsample, and remove outliers) */
PointCloud::Ptr
applyFilters (const PointCloud::Ptr & input, float min_depth, float max_depth, float leaf_size)
{
	PointCloud::Ptr filtered;
  filtered = thresholdDepth (input, min_depth, max_depth);
  filtered = downsampleF (filtered, leaf_size);
//filtered = removeOutliers (filtered, radius, min_neighbors);

  return (filtered);
}




#endif /* FILTER_H_ */
