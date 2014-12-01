#include "../filter.h"

void downSampleCloud(const PointCloudRGBPtr cloud_src,
		PointCloudRGBPtr sampled_cloud, float VOXEL_GRID_SIZE) {

	pcl::VoxelGrid<PointRGB> vox_grid;
	vox_grid.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
	vox_grid.setInputCloud(cloud_src);
	vox_grid.filter(*sampled_cloud);
}

void thresholdDepth(PointCloudRGBPtr & input, PointCloudRGBPtr filtered,
		float min_depth, float max_depth) {
	pcl::PassThrough<PointRGB> pass_through;
	pass_through.setInputCloud(input);
	pass_through.setFilterFieldName("z");
	pass_through.setFilterLimits(min_depth, max_depth);
	pass_through.filter(*filtered);

}

/* Use a VoxelGrid filter to reduce the number of points */
void downsample(PointCloudRGBPtr input, PointCloudRGBPtr filtered,
		float leaf_size) {
	pcl::VoxelGrid<PointRGB> voxel_grid;
	voxel_grid.setInputCloud(input);
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_grid.filter(*filtered);
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
void removeOutliers(PointCloudRGBPtr & input, PointCloudRGBPtr filtered,
		float radius, int min_neighbors) {
	pcl::RadiusOutlierRemoval<PointRGB> radius_outlier_removal;
	radius_outlier_removal.setInputCloud(input);
	radius_outlier_removal.setRadiusSearch(radius);
	radius_outlier_removal.setMinNeighborsInRadius(min_neighbors);
	radius_outlier_removal.filter(*filtered);
}

/* Apply a series of filters (threshold depth, downsample, and remove outliers) */
void applyFilters(PointCloudRGBPtr input, PointCloudRGBPtr filtered,
		float leaf_size) {
//	thresholdDepth (input, filtered, min_depth, max_depth);
	//filtered = downsampleF (filtered, leaf_size);
	//removeOutliers (input, filtered, radius, min_neighbors);
	downsample(input, filtered, leaf_size);

}

