#include "../feature_estimation.h"

#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>

SurfaceNormalsPtr estimateSurfaceNormals(const PointCloudRGBPtr & input,
		float radius) {
	pcl::NormalEstimation<PointRGB, NormalT> normal_estimation;
	normal_estimation.setSearchMethod(
			pcl::search::KdTree<PointRGB>::Ptr(
					new pcl::search::KdTree<PointRGB>));
	normal_estimation.setRadiusSearch(radius);
	normal_estimation.setInputCloud(input);
	SurfaceNormalsPtr normals(new SurfaceNormals);
	normal_estimation.compute(*normals);

	return (normals);
}

PointCloudRGBPtr detectKeypoints(const PointCloudRGBPtr & points,
		const SurfaceNormalsPtr & normals, float min_scale, int nr_octaves,
		int nr_scales_per_octave, float min_contrast) {
	pcl::SIFTKeypoint<PointRGB, pcl::PointWithScale> sift_detect;
	sift_detect.setSearchMethod(
			pcl::search::KdTree<PointRGB>::Ptr(
					new pcl::search::KdTree<PointRGB>));
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);
	sift_detect.setInputCloud(points);
	pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
	sift_detect.compute(keypoints_temp);
	PointCloudRGBPtr keypoints(new PointCloudRGB);
	pcl::copyPointCloud(keypoints_temp, *keypoints);

	return (keypoints);
}

LocalDescriptorsPtr computeLocalDescriptors(const PointCloudRGBPtr & points,
		const SurfaceNormalsPtr & normals, const PointCloudRGBPtr & keypoints,
		float feature_radius) {
	pcl::FPFHEstimation<PointRGB, NormalT, LocalDescriptorT> fpfh_estimation;
	fpfh_estimation.setSearchMethod(
			pcl::search::KdTree<PointRGB>::Ptr(
					new pcl::search::KdTree<PointRGB>));
	fpfh_estimation.setRadiusSearch(feature_radius);
	fpfh_estimation.setSearchSurface(points);
	fpfh_estimation.setInputNormals(normals);
	fpfh_estimation.setInputCloud(keypoints);
	LocalDescriptorsPtr local_descriptors(new LocalDescriptors);
	fpfh_estimation.compute(*local_descriptors);

	return (local_descriptors);
}
GlobalDescriptorsPtr computeGlobalDescriptor(const PointCloudRGBPtr & points,
		const SurfaceNormalsPtr & normals) {
	pcl::VFHEstimation<PointRGB, NormalT, GlobalDescriptorT> vfh_estimation;
	vfh_estimation.setSearchMethod(
			pcl::search::KdTree<PointRGB>::Ptr(
					new pcl::search::KdTree<PointRGB>));
	vfh_estimation.setInputCloud(points);
	vfh_estimation.setInputNormals(normals);
	GlobalDescriptorsPtr global_descriptor(new GlobalDescriptors);
	vfh_estimation.compute(*global_descriptor);

	return (global_descriptor);
}

ObjectFeatures computeFeatures(const PointCloudRGBPtr & input) {
	ObjectFeatures features;
	features.points = input;
	features.normals = estimateSurfaceNormals(input, 0.05);
	features.keypoints = detectKeypoints(input, features.normals, 0.005, 10, 8,
			1.5);
	features.local_descriptors = computeLocalDescriptors(input,
			features.normals, features.keypoints, 0.1);
	//features.global_descriptor = computeGlobalDescriptor (input, features.normals);

	return (features);

}
void computerSurfaceNormal(PointCloudRGBPtr source_points,
		PointCloudNormalPtr points_with_normals) {
	pcl::NormalEstimation<PointRGB, PNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(source_points);
	norm_est.compute(*points_with_normals);
	pcl::copyPointCloud(*source_points, *points_with_normals);

}

