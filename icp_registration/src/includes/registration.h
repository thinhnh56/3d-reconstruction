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
computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations);

Eigen::Matrix4f
refineAlignment (const PointCloudPtr & source_points, const PointCloudPtr & target_points, 
                 const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                 float outlier_rejection_threshold, float transformation_epsilon, float max_iterations);

Eigen::Matrix4f
incrementICPX(const PointCloudNormal::Ptr & source_points, const PointCloudNormal::Ptr & target_points,
			const Eigen::Matrix4f initial_alignment,
			double max_dist
			);

void pairAlign (const PointCloudRGB::Ptr cloud_src, 
				const PointCloudRGB::Ptr cloud_tgt, 
				PointCloudRGB::Ptr output, 
				Eigen::Matrix4f &final_transform, 
				double leaf_size, 
				bool downsample,
				bool initial_align,
				float voxel_grid_size,
				float min_sample_distance, float max_correspondence_distance, int nr_iterations,
				double max_dist,
				pcl::visualization::PCLVisualizer *v,
				int vp1, 
				int vp2
				
				);

			
#endif