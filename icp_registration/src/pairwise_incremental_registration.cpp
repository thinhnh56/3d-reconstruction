#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/common/distances.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "include/load_data.h"
#include "include/object_recognition.h"
#include "include/typedef.h"
#include "include/filter.h"
#include "include/visualization.h"
#include "include/feature_estimation.h"
#include "include/registration.h"

#include "ctime"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
using namespace std;

pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

// Time
time_t start, end, gStart;

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target,
		const PointCloud::Ptr cloud_source) {
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");

	PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

	PCL_INFO("Press q to begin the registration.\n");
	p->spin();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target,
		const PointCloudWithNormals::Ptr cloud_source) {
	p->removePointCloud("source");
	p->removePointCloud("target");

	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(
			cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(
			cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}

Eigen::Matrix4f IncrementICP(PointCloudWithNormals::Ptr points_with_normals_src,
		PointCloudWithNormals::Ptr points_with_normals_tgt,
		Eigen::Matrix4f initial_alignment, ObjectRecognitionParameters params) {
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);
	PointCloudWithNormals::Ptr transformed_points_with_normals_src(
			new PointCloudWithNormals);
	pcl::transformPointCloud(*points_with_normals_src,
			*transformed_points_with_normals_src, initial_alignment);

	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon(params.icp_transformation_epsilon);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(params.icp_max_correspondence_distance);
	// Set the point representation
	reg.setPointRepresentation(
			boost::make_shared<const MyPointRepresentation>(
					point_representation));

	reg.setInputSource(transformed_points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
	PointCloudWithNormals::Ptr reg_result = transformed_points_with_normals_src;
	reg.setMaximumIterations(params.icp_max_iterations);
	for (int i = 0; i < 30; ++i) {
		PCL_INFO("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		transformed_points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource(transformed_points_with_normals_src);
		reg.align(*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum())
				< reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(
					reg.getMaxCorrespondenceDistance() * 0.9);

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		showCloudsRight(points_with_normals_tgt,
				transformed_points_with_normals_src);
	}
	return Ti;

}

void pairAlign(PCD &pcd_source, PCD &pcd_taget, PointCloud::Ptr output,
		Eigen::Matrix4f &final_transform, ObjectRecognitionParameters params) {
	bool downsample = params.global_downsample;
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);

	src = pcd_source.features.getCloud(downsample);
	tgt = pcd_taget.features.getCloud(downsample);
	pcl::console::print_info("Downsampled to %d - %d.\n", src->points.size(), tgt->points.size());
	Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f targetToSource = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();

	if (params.global_initial) {
		pcl::console::print_info("Computing initial alignment ...\n");
		initial_alignment = computeInitialAlignment(
				pcd_source.features.getKeypoints(),
				pcd_source.features.getLocalDescriptors(),
				pcd_taget.features.getKeypoints(),
				pcd_taget.features.getLocalDescriptors(),
				params.initial_alignment_min_sample_distance,
				params.initial_alignment_max_correspondence_distance,
				params.initial_alignment_nr_iterations);
		/*float before_distance, after_distance;
		PointCloud::Ptr temp_cloud(new PointCloud);
		pcl::transformPointCloud(*src, *temp_cloud, initial_alignment);
		before_distance = pcl::euclideanDistance(*src, *tgt);
		after_distance = pcl::euclideanDistance(*temp_cloud, *tgt);
		if(before_distance < after_distance){
			pcl::console::print_info("Initial failed.\n");
		}*/
	}

	if (params.global_icp) {
		pcl::console::print_info("Computing icp...\n");
		// Compute surface normals and curvature
		PointCloudWithNormals::Ptr points_with_normals_src(
				new PointCloudWithNormals);
		PointCloudWithNormals::Ptr points_with_normals_tgt(
				new PointCloudWithNormals);
		points_with_normals_src =
				pcd_source.features.getSurfaceCurvatureNormals(downsample);
		points_with_normals_tgt = pcd_taget.features.getSurfaceCurvatureNormals(
				downsample);
		pcl::copyPointCloud(*src, *points_with_normals_src);
		pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

		// Instantiate our custom point representation (defined above) ...

		/*---------------- ICP here -----------------------*/

		Ti = IncrementICP(points_with_normals_src,
				points_with_normals_tgt, initial_alignment, params);
	}
	// Get the transformation from target to source
	targetToSource = (Ti * initial_alignment).inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*pcd_taget.cloud, *output, targetToSource);

	p->removePointCloud("source");
	p->removePointCloud("target");

	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> cloud_src_h(pcd_source.cloud, 255, 0,
			0);
	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	p->addPointCloud(pcd_source.cloud, cloud_src_h, "source", vp_2);

	PCL_INFO("Press q to continue the registration.\n");
	p->spin();

	p->removePointCloud("source");
	p->removePointCloud("target");

	//add the source to the transformed target
	*output += *pcd_source.cloud;

	final_transform = targetToSource;
}

/* ---[ */
int main(int argc, char** argv) {

	time(&gStart);
	if (argc < 10) {
		pcl::console::print_info(
				"Syntax is: inputList.txt --filter filterParams.txt --feature featureParams.txt --init initialParams.txt --refine refineParams.txt");
		return 1;
	}
	ObjectRecognitionParameters params = loadParams(argc, argv);
	// Load data
	std::vector<PCD> data;
	pcl::console::print_info("Loading data ...");
	time(&start);
	loadData(argc, argv, data, params);
	time(&end);
	pcl::console::print_info("take %.2lf seconds.\n", difftime(end, start));

	// Check user input
	if (data.empty()) {
		PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
		PCL_ERROR(
				"[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
		return (-1);
	}
	PCL_INFO("Loaded %d datasets.", (int )data.size());

	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer(argc, argv,
			"Pairwise Incremental Registration example");
	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

	PointCloud::Ptr result(new PointCloud);
	PCD *source, *target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(),
			pairTransform;
	Eigen::Matrix4f MTransform = Eigen::Matrix4f::Identity();

	int center = data.size() / 2;
	data[center].transformToSource = Eigen::Matrix4f::Identity();
	// from
	int cloud_count = 0;
	for (int i = center - 1; i >= 0; i--) {
		cloud_count++;
		source = &data[i + 1];
		target = &data[i];

		// Add visualization data
		showCloudsLeft(source->cloud, target->cloud);

		PointCloud::Ptr temp(new PointCloud);
		PCL_INFO("Aligning %s (%d) with %s (%d).\n", source->f_name.c_str(),
				source->cloud->points.size(), target->f_name.c_str(),
				target->cloud->points.size());
		pairAlign(*source, *target, temp, pairTransform, params);

		//transform current pair into the global transform
		pcl::transformPointCloud(*temp, *result, GlobalTransform);
		//update the global transform
		GlobalTransform = GlobalTransform * pairTransform;
		MTransform = MTransform * pairTransform;
		data[i].transformToSource = GlobalTransform;
		//save aligned pair, transformed into the first cloud's frame
		/*std::stringstream ss;
		 ss << i << ".pcd";
		 pcl::io::savePCDFile(ss.str(), *result, true);*/

		// Refine main cloud.
		if (cloud_count >= params.global_max_registration) {
			cout << "Refine main cloud " << i << endl;
			PointCloud::Ptr temp_Cloud(new PointCloud);
			pcl::transformPointCloud(*target->cloud, *temp_Cloud, MTransform);
			PCD temp_PCD(params, temp_Cloud);
			source = &data[i + params.global_max_registration];
			target = &temp_PCD;
			showCloudsLeft(source->cloud, target->cloud);

			PointCloud::Ptr temp(new PointCloud);
			PCL_INFO("Aligning %s (%d) with cloud %d (%d).\n",
					source->f_name.c_str(), source->cloud->points.size(), i,
					target->cloud->points.size());
			pairAlign(*source, *target, temp, pairTransform, params);

			//update the global transform
			GlobalTransform = source->transformToSource
					* (MTransform * pairTransform);
			data[i].transformToSource = GlobalTransform;

			// Reset MTransform
			MTransform = Eigen::Matrix4f::Identity();
			cloud_count = 0;

		}

	}
	cout << "change to next section OK" << endl;
	// from center to right.

	GlobalTransform = Eigen::Matrix4f::Identity();
	MTransform = Eigen::Matrix4f::Identity();
	cloud_count = 0;
	for (int i = center + 1; i < data.size(); ++i) {
		cloud_count++;
		source = &data[i - 1];
		target = &data[i];

		// Add visualization data
		showCloudsLeft(source->cloud, target->cloud);

		PointCloud::Ptr temp(new PointCloud);
		PCL_INFO("Aligning %s (%d) with %s (%d).\n",
				source->f_name.c_str(), source->cloud->points.size(),
				target->f_name.c_str(), target->cloud->points.size());
		pairAlign(*source, *target, temp, pairTransform, params);

		//transform current pair into the global transform
		pcl::transformPointCloud(*temp, *result, GlobalTransform);
		//update the global transform
		GlobalTransform = GlobalTransform * pairTransform;
		MTransform = MTransform * pairTransform;
		data[i].transformToSource = GlobalTransform;

		// Refine main cloud.
		if (cloud_count >= params.global_max_registration) {
			cout << "Refine main cloud " << i << endl;
			PointCloud::Ptr temp_Cloud(new PointCloud);
			pcl::transformPointCloud(*target->cloud, *temp_Cloud, MTransform);
			PCD temp_PCD(params, temp_Cloud);
			source = &data[i - params.global_max_registration];
			target = &temp_PCD;
			showCloudsLeft(source->cloud, target->cloud);

			PointCloud::Ptr temp(new PointCloud);
			PCL_INFO("Aligning %s (%d) with %s (%d).\n",
					source->f_name.c_str(), source->cloud->points.size(),
					data[i].f_name.c_str(),target->cloud->points.size());
			pairAlign(*source, *target, temp, pairTransform, params);

			//update the global transform
			GlobalTransform = source->transformToSource
					* (MTransform * pairTransform);
			data[i].transformToSource = GlobalTransform;

			// Reset MTransform
			MTransform = Eigen::Matrix4f::Identity();
			cloud_count = 0;

		}

	}
	PointCloud::Ptr left_final(new PointCloud);
	PointCloud::Ptr temp(new PointCloud);
	for (int i = 0; i <= center; i++) {
		PointCloud::Ptr temp(new PointCloud);
		pcl::transformPointCloud(*data[i].cloud, *temp,
				data[i].transformToSource);
		*left_final += *temp;
	}

	PointCloud::Ptr right_final(new PointCloud);
	for (int i = center; i < data.size(); i++) {
		PointCloud::Ptr temp(new PointCloud);
		pcl::transformPointCloud(*data[i].cloud, *temp,
				data[i].transformToSource);
		*right_final += *temp;
	}
	PointCloud::Ptr final(new PointCloud);
	/*cout << "Final registration." << endl;
	 PCD left_PCD(params, left_final);
	 PCD right_PCD(params, right_final);
	 source = &left_PCD;
	 target = &right_PCD;
	 pairAlign(*source, *target, final, pairTransform, params);
	 */
	*final = *left_final + *right_final;
	pcl::io::savePCDFile("./results/left_final.pcd", *left_final);
	pcl::io::savePCDFile("./results/right_final.pcd", *right_final);
	pcl::io::savePCDFile("./results/final.pcd", *final);
	time(&end);
	pcl::console::print_info("Program taked %.2lf seconds.",
			difftime(end, gStart));
	view(*final);

}
/* ]--- */
