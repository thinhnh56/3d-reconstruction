#ifndef _REGISTRATION_

#define _REGISTRATION_

#include "typedefs.h"
#include "feature_estimation.h"
#include "visualization.h"
#include "filter.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_ransac.h>

Eigen::Matrix4f
computeInitialAlignment(const PointCloudRGBPtr & source_points,
		const LocalDescriptorsPtr & source_descriptors,
		const PointCloudRGBPtr & target_points,
		const LocalDescriptorsPtr & target_descriptors,
		float min_sample_distance, float max_correspondence_distance,
		int nr_iterations);

Eigen::Matrix4f
refineAlignment(const PointCloudRGBPtr & source_points,
		const PointCloudRGBPtr & target_points,
		const Eigen::Matrix4f initial_alignment,
		float max_correspondence_distance, float outlier_rejection_threshold,
		float transformation_epsilon, float max_iterations);

Eigen::Matrix4f
incrementICPX(const PointCloudNormalPtr & source_points,
		const PointCloudNormalPtr & target_points,
		const Eigen::Matrix4f initial_alignment, double max_dist);

Eigen::Matrix4f pairAlign(const PointCloudRGBPtr cloud_src,
		const PointCloudRGBPtr cloud_tgt, PointCloudRGBPtr output,
		bool is_downsample, bool is_initialize, float voxel_grid_size,
		float min_sample_distance, float max_correspondence_distance,
		int nr_iterations, double max_dist,
		pcl::visualization::PCLVisualizer *v, int vp1, int vp2

		);

#endif
