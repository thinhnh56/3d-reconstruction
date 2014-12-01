#ifndef _FEATURE_ESTIMATION_H
#define _FEATURE_ESTIMATION_H

#include "typedefs.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>

struct ObjectFeatures {
	PointCloudRGBPtr points;
	SurfaceNormalsPtr normals;
	PointCloudRGBPtr keypoints;
	LocalDescriptorsPtr local_descriptors;
	GlobalDescriptorsPtr global_descriptor;

};

SurfaceNormalsPtr
estimateSurfaceNormals(const PointCloudRGBPtr & input, float radius);

PointCloudRGBPtr
detectKeypoints(const PointCloudRGBPtr & points,
		const SurfaceNormalsPtr & normals, float min_scale, int nr_octaves,
		int nr_scales_per_octave, float min_contrast);

LocalDescriptorsPtr
computeLocalDescriptors(const PointCloudRGBPtr & points,
		const SurfaceNormalsPtr & normals, const PointCloudRGBPtr & keypoints,
		float feature_radius);

GlobalDescriptorsPtr
computeGlobalDescriptor(const PointCloudRGBPtr & points,
		const SurfaceNormalsPtr & normals);

ObjectFeatures
computeFeatures(const PointCloudRGBPtr & input);

// Computer surface normal and curvature

void computerSurfaceNormal(PointCloudRGBPtr source_points,
		PointCloudNormalPtr points_with_normals);
#endif
