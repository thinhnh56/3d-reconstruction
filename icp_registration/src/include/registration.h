/*
 * registration.h
 *
 *  Created on: Dec 6, 2014
 *      Author: phanthanh
 */

#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include "typedef.h"

using namespace std;

Eigen::Matrix4f computeInitialAlignment(const PointCloud::Ptr & source_points,
		const LocalDescriptorsPtr & source_descriptors,
		const PointCloud::Ptr & target_points,
		const LocalDescriptorsPtr & target_descriptors,
		float min_sample_distance, float max_correspondence_distance,
		int nr_iterations) {
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB,
			LocalDescriptorT> sac_ia;
	sac_ia.setMinSampleDistance(min_sample_distance);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
	sac_ia.setMaximumIterations(nr_iterations);

	sac_ia.setInputSource(source_points);
	sac_ia.setSourceFeatures(source_descriptors);

	sac_ia.setInputTarget(target_points);
	sac_ia.setTargetFeatures(target_descriptors);

	PointCloud::Ptr registration_output(new PointCloud);
	sac_ia.align(*registration_output);
	return (sac_ia.getFinalTransformation());
}

#endif /* REGISTRATION_H_ */
