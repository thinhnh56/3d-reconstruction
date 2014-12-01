#include "../registration.h"

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

Eigen::Matrix4f computeInitialAlignment(const PointCloudRGBPtr & source_points,
		const LocalDescriptorsPtr & source_descriptors,
		const PointCloudRGBPtr & target_points,
		const LocalDescriptorsPtr & target_descriptors,
		float min_sample_distance, float max_correspondence_distance,
		int nr_iterations) {
	pcl::SampleConsensusInitialAlignment<PointRGB, PointRGB, LocalDescriptorT> sac_ia;
	sac_ia.setMinSampleDistance(min_sample_distance);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
	sac_ia.setMaximumIterations(nr_iterations);

	sac_ia.setInputSource(source_points);
	sac_ia.setSourceFeatures(source_descriptors);

	sac_ia.setInputTarget(target_points);
	sac_ia.setTargetFeatures(target_descriptors);

	PointCloudRGB registration_output;
	sac_ia.align(registration_output);

	return (sac_ia.getFinalTransformation());
}

Eigen::Matrix4f refineAlignment(const PointCloudRGBPtr & source_points,
		const PointCloudRGBPtr & target_points,
		const Eigen::Matrix4f initial_alignment,
		float max_correspondence_distance, float outlier_rejection_threshold,
		float transformation_epsilon, float max_iterations) {

	pcl::IterativeClosestPoint<PointRGB, PointRGB> icp;
	icp.setMaxCorrespondenceDistance(max_correspondence_distance);
	icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
	icp.setTransformationEpsilon(transformation_epsilon);
	icp.setMaximumIterations(max_iterations);

	PointCloudRGBPtr source_points_transformed(new PointCloudRGB);
	pcl::transformPointCloud(*source_points, *source_points_transformed,
			initial_alignment);

	icp.setInputSource(source_points_transformed);
	icp.setInputTarget(target_points);

	PointCloudRGB registration_output;
	icp.align(registration_output);

	return (icp.getFinalTransformation() * initial_alignment);
}

Eigen::Matrix4f incrementICPX(const PointCloudNormalPtr & source_points,
		const PointCloudNormalPtr & target_points,
		const Eigen::Matrix4f initial_alignment, double max_dist) {
	// Khoi tao ICP va set tham so
#define GICP
#define STANDARD_ICP

#ifdef GICP
	pcl::GeneralizedIterativeClosestPoint<PNormal, PNormal> reg;
#else
#ifndef STANDARD_ICP
	pcl::IterativeClosestPointNonLinear<PNormal, PNormal> reg;
#else
	pcl::IterativeClosestPoint<PNormal, PNormal> reg;
#endif
#endif
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	reg.setTransformationEpsilon(1e-16); //provo fino a 1e-16
	reg.setMaxCorrespondenceDistance(max_dist);
	reg.setPointRepresentation(
			boost::make_shared<const MyPointRepresentation>(
					point_representation));

	// su dung intial_alignment

	PointCloudNormalPtr source_points_transformed(new PointCloudNormal);
	pcl::transformPointCloud(*source_points, *source_points_transformed,
			initial_alignment);

	// align

	reg.setInputSource(source_points_transformed);
	reg.setInputTarget(target_points);

	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
	PointCloudNormalPtr reg_result = source_points_transformed;
	reg.setMaximumIterations(20); //provato 20

	for (int i = 0; i < 30; ++i) {
		PCL_INFO("Iteration Nr. %d.\n", i);

		source_points_transformed = reg_result;
		// Estimate
		reg.setInputSource(source_points_transformed);
		reg.align(*reg_result);
#ifdef GICP
		pcl::transformPointCloud(*reg_result, *reg_result,
				reg.getFinalTransformation());
#endif

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum())
				< reg.getTransformationEpsilon()) {
			reg.setMaxCorrespondenceDistance(
					0.9 * reg.getMaxCorrespondenceDistance());
			std::cout << "change max distance to "
					<< reg.getMaxCorrespondenceDistance() << std::endl;
		}

		prev = reg.getLastIncrementalTransformation();

	}

	return Ti;

}

Eigen::Matrix4f pairAlign(const PointCloudRGBPtr cloud_src,
		const PointCloudRGBPtr cloud_tgt, PointCloudRGBPtr output,
		bool is_downsample, bool is_initialize, float voxel_grid_size,
		float min_sample_distance, float max_correspondence_distance,
		int nr_iterations, double max_dist,
		pcl::visualization::PCLVisualizer *v, int vp1, int vp2) {
	PointCloudRGB final;
	PointCloudRGBPtr src(new PointCloudRGB);
	PointCloudRGBPtr tgt(new PointCloudRGB);
	if (is_downsample) {
		applyFilters(cloud_src, src, voxel_grid_size);
		applyFilters(cloud_tgt, tgt, voxel_grid_size);

		PCL_INFO("subsampled to %d and %d points.\n", src->size(), tgt->size());
	} else {
		src = cloud_src;
		tgt = cloud_tgt;
	}

	// load features
	ObjectFeatures cloud1Features = computeFeatures(src);
	ObjectFeatures cloud2Features = computeFeatures(tgt);
	Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity();

	if (is_initialize) {
		initial_alignment = computeInitialAlignment(cloud1Features.keypoints,
				cloud1Features.local_descriptors, cloud2Features.keypoints,
				cloud2Features.local_descriptors, min_sample_distance,
				max_correspondence_distance, nr_iterations);

	}

	// Compute surface normals and curvature
	PointCloudNormalPtr points_with_normals_src(new PointCloudNormal);
	PointCloudNormalPtr points_with_normals_tgt(new PointCloudNormal);

	computerSurfaceNormal(src, points_with_normals_src);
	computerSurfaceNormal(tgt, points_with_normals_tgt);

	Eigen::Matrix4f targetToSource, Ti = incrementICPX(points_with_normals_src,
			points_with_normals_tgt, initial_alignment, max_dist);

	targetToSource = (Ti * initial_alignment).inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	PCL_INFO("Press q to show the final result on the original cloud.\n");
	v->spin();

	v->removePointCloud("source");
	v->removePointCloud("target");

	PointCloudColorHandlerCustom<PointRGB> cloud_tgt_h(output, 0, 255, 0);
	PointCloudColorHandlerCustom<PointRGB> cloud_src_h(cloud_src, 255, 0, 0);
	v->addPointCloud(output, "target", vp2);
	// Not pickable
	v->getCloudActorMap()->at("target").actor->PickableOff();
	v->addPointCloud(cloud_src, "source", vp2);
	// Not pickable
	v->getCloudActorMap()->at("source").actor->PickableOff();

	PCL_INFO("Press q to continue the registration.\n");
	v->spin();

	v->removePointCloud("source");
	v->removePointCloud("target");

	//add the source to the transformed target
	*output += *cloud_src;

	return targetToSource;
}
