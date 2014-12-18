/*
 * feature_estimation.h
 *
 *  Created on: Dec 4, 2014
 *      Author: phanthanh
 */

#ifndef FEATURE_ESTIMATION_H_
#define FEATURE_ESTIMATION_H_

#include "typedef.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>

SurfaceNormals::Ptr estimateSurfaceNormals(const PointCloud::Ptr & input,
		float radius) {
	pcl::NormalEstimation<PointT, NormalT> normal_estimation;
	normal_estimation.setSearchMethod(
			pcl::search::Search<PointT>::Ptr(new pcl::search::KdTree<PointT>));
	normal_estimation.setRadiusSearch(radius);
	normal_estimation.setInputCloud(input);
	SurfaceNormals::Ptr normals(new SurfaceNormals);
	normal_estimation.compute(*normals);

	return (normals);
}

PointCloudWithNormals::Ptr estimateSurfaceCurvatureNormals(
		const PointCloud::Ptr &input) {
	PointCloudWithNormals::Ptr output(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(input);
	norm_est.compute(*output);
	return output;
}

PointCloud::Ptr detectKeypoints(const PointCloud::Ptr & points,
		const SurfaceNormals::Ptr & normals, float min_scale, int nr_octaves,
		int nr_scales_per_octave, float min_contrast) {
	pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
	sift_detect.setSearchMethod(
			pcl::search::Search<PointT>::Ptr(new pcl::search::KdTree<PointT>));
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);
	sift_detect.setInputCloud(points);
	pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
	sift_detect.compute(keypoints_temp);
	PointCloud::Ptr keypoints(new PointCloud);
	pcl::copyPointCloud(keypoints_temp, *keypoints);

	return (keypoints);
}

LocalDescriptorsPtr computeLocalDescriptors(const PointCloud::Ptr & points,
		const SurfaceNormals::Ptr & normals, const PointCloud::Ptr & keypoints,
		float feature_radius) {
	pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
	fpfh_estimation.setSearchMethod(
			pcl::search::Search<PointT>::Ptr(new pcl::search::KdTree<PointT>));
	fpfh_estimation.setRadiusSearch(feature_radius);
	fpfh_estimation.setSearchSurface(points);
	fpfh_estimation.setInputNormals(normals);
	fpfh_estimation.setInputCloud(keypoints);
	LocalDescriptorsPtr local_descriptors(new LocalDescriptors);
	fpfh_estimation.compute(*local_descriptors);

	return (local_descriptors);
}

#endif /* FEATURE_ESTIMATION_H_ */
