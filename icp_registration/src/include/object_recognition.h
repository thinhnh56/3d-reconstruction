#ifndef OBJECT_RECOGNITION_H_
#define OBJECT_RECOGNITION_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <string>

#include "feature_estimation.h"
#include "typedef.h"
#include "filter.h"

using namespace std;

class ObjectRecognitionParameters {
public:
	void display() {
		// global
		std::cout << "max_registration: " << global_max_registration << endl;
		std::cout << "global_initial " << global_initial << endl;
		std::cout << "global_icp " << global_icp << endl;
		std::cout << "downsample " << global_downsample << endl;
		// filters
		std::cout << "min_depth: " << min_depth << endl;
		std::cout << "max_depth: " << max_depth << endl;
		std::cout << "min_x: " << min_x << endl;
		std::cout << "max_x: " << max_x << endl;
		std::cout << "min_y: " << min_y << endl;
		std::cout << "max_y: " << max_y << endl;
		std::cout << "downsample_leaf_size: " << downsample_leaf_size << endl;
		std::cout << "min_dowsample_points: " << min_dowsample_points << endl;
		// feature estimation.
		std::cout << "surface_normal_radius: " << surface_normal_radius << endl;
		std::cout << "keypoints_min_scale: " << keypoints_min_scale << endl;
		std::cout << "keypoints_nr_octaves: " << keypoints_nr_octaves << endl;
		std::cout << "keypoints_nr_scales_per_octave: "
				<< keypoints_nr_scales_per_octave << endl;
		std::cout << "keypoints_min_contrast: " << keypoints_min_contrast
				<< endl;
		std::cout << "local_descriptor_radius: " << local_descriptor_radius
				<< endl;
		// registration.
		std::cout << "initial_alignment_min_sample_distance: "
				<< initial_alignment_min_sample_distance << endl;
		std::cout << "initial_alignment_max_correspondence_distance: "
				<< initial_alignment_max_correspondence_distance << endl;
		std::cout << "initial_alignment_nr_iterations: "
				<< initial_alignment_nr_iterations << endl;
		std::cout << "icp_max_correspondence_distance: "
				<< icp_max_correspondence_distance << endl;
		std::cout << "icp_outlier_rejection_threshold: "
				<< icp_outlier_rejection_threshold << endl;
		std::cout << "icp_transformation_epsilon: "
				<< icp_transformation_epsilon << endl;
		std::cout << "icp_max_iterations: " << icp_max_iterations << endl;

	}

	// Global parameters
	int global_max_registration;
	bool global_initial;
	bool global_icp;
	bool global_downsample;

	// Filter parameters
	float min_depth;
	float max_depth;
	float min_x;
	float max_x;
	float min_y;
	float max_y;
	float downsample_leaf_size;
	int min_dowsample_points;

	// Feature estimation parameters
	float surface_normal_radius;
	float keypoints_min_scale;
	float keypoints_nr_octaves;
	float keypoints_nr_scales_per_octave;
	float keypoints_min_contrast;
	float local_descriptor_radius;

	// intial alignment parameters
	float initial_alignment_min_sample_distance;
	float initial_alignment_max_correspondence_distance;
	int initial_alignment_nr_iterations;

	// refine alignment parameters.
	float icp_max_correspondence_distance;
	float icp_outlier_rejection_threshold;
	float icp_transformation_epsilon;
	int icp_max_iterations;

};

class ObjectFeatures {
private:
	PointCloud::Ptr points;
	PointCloud::Ptr downsampleCloud;
	SurfaceNormals::Ptr normals;
	PointCloudWithNormals::Ptr normalCurvatures;
	PointCloud::Ptr keypoints;
	LocalDescriptors::Ptr local_descriptors;
	ObjectRecognitionParameters params;
public:
	ObjectFeatures(PointCloud::Ptr &cloud,
			ObjectRecognitionParameters &params) {
		points = cloud;
		this->params = params;
	}

	ObjectFeatures() :
			points(new PointCloud) {

	}

	PointCloud::Ptr getOriginCloud() {
		return points;
	}

	void setUp(ObjectRecognitionParameters oParam, PointCloud::Ptr oCloud) {
		*points = *oCloud;
		params = oParam;

	}

	PointCloud::Ptr getDownsampleCloud() {
		if (downsampleCloud == NULL) {
			cout << "Inside: downsample" << endl;
			downsampleCloud = downsampleF(points, params.downsample_leaf_size);
			float leaf_size = params.downsample_leaf_size;
			while (downsampleCloud->points.size() <= params.min_dowsample_points) {
				leaf_size *= 0.7;
				downsampleCloud = downsampleF(points, leaf_size);

			}
		}
		return downsampleCloud;
	}

	PointCloud::Ptr getCloud(bool downsample = true) {
		if (!downsample) {
			return points;
		} else {
			return getDownsampleCloud();
		}

	}

	SurfaceNormals::Ptr getSurfaceNormals(bool downsample = true) {
		if (normals == NULL) {
			cout << "Inside: surface normals" << endl;
			normals = estimateSurfaceNormals(getCloud(downsample),
					params.downsample_leaf_size);
		}
		return normals;

	}
	PointCloud::Ptr getKeypoints(bool downsample = true) {
		//downsample = false;
		if (keypoints == NULL) {

			keypoints = detectKeypoints(this->getCloud(downsample),
					this->getSurfaceNormals(downsample),
					params.keypoints_min_scale, params.keypoints_nr_octaves,
					params.keypoints_nr_scales_per_octave,
					params.keypoints_min_contrast);
			cout << "Inside: " << keypoints->points.size() << " keypoints"
					<< endl;
		}
		return keypoints;
	}

	LocalDescriptors::Ptr getLocalDescriptors(bool downsample = true) {
		//downsample = false;
		if (local_descriptors == NULL) {
			cout << "Inside: local descriptors" << endl;
			local_descriptors = computeLocalDescriptors(
					this->getCloud(downsample),
					this->getSurfaceNormals(downsample),
					this->getKeypoints(downsample),
					params.local_descriptor_radius);
		}
		return local_descriptors;
	}
	PointCloudWithNormals::Ptr getSurfaceCurvatureNormals(
			bool downsample = true) {
		if (normalCurvatures == NULL) {
			cout << "Inside: curvature" << endl;
			normalCurvatures = estimateSurfaceCurvatureNormals(
									this->getCloud(downsample));
		}
		return normalCurvatures;
	}

};

class PCD {
public:
	PointCloud::Ptr cloud;
	std::string f_name;
	Eigen::Matrix4f transformToSource;
	ObjectFeatures features;

	PCD(ObjectRecognitionParameters oParams) :
			cloud(new PointCloud), features(cloud, oParams) {
	}
	PCD(ObjectRecognitionParameters oParams, PointCloud::Ptr mCloud) :
			cloud(new PointCloud), features() {
		cout << "pcd constructor." << endl;
		*cloud = *mCloud;
		features.setUp(oParams, mCloud);
		cout << "end pcd constructor." << endl;
	}

	void getObjectFeatures() {
		features.getDownsampleCloud();
		features.getSurfaceNormals();
		features.getLocalDescriptors();
		features.getSurfaceCurvatureNormals();
	}
};

class PCDComparator {
	bool operator ()(const PCD& p1, const PCD& p2) {
		return (p1.f_name < p2.f_name);
	}
};

#endif
