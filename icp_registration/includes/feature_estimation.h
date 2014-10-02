#ifndef _FEATURE_ESTIMATION_H
#define _FEATURE_ESTIMATION_H

#include "typedefs.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>


struct ObjectFeatures{

	PointCloudRGB::Ptr points;
	SurfaceNormalsPtr normals;
	PointCloudPtr keypoints;
	LocalDescriptorsPtr local_descriptors;
	GlobalDescriptorsPtr global_descriptor;

};

SurfaceNormalsPtr
estimateSurfaceNormals (const PointCloudPtr & input, float radius);

PointCloudPtr
detectKeypoints (const PointCloudPtr & points, const SurfaceNormalsPtr & normals,
                 float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast);

LocalDescriptorsPtr
computeLocalDescriptors (const PointCloudPtr & points, const SurfaceNormalsPtr & normals, 
                         const PointCloudPtr & keypoints, float feature_radius);

GlobalDescriptorsPtr
computeGlobalDescriptor (const PointCloudPtr & points, const SurfaceNormalsPtr & normals);

ObjectFeatures
computeFeatures (const PointCloudPtr & input);

// Computer surface normal and curvature

void computerSurfaceNormal(PointCloudPtr source_points, 
				PointCloudNormal::Ptr points_with_normals);
#endif