#include <iostream>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_cloud.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/fpfh.h"
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkActor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include "pcl/features/vfh.h"

//------------------------------------------------

using namespace pcl;
using namespace std;

//convenient typedefs
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointRGB>::ConstPtr PointCloudConstPtr;

typedef pcl::PointXYZRGBNormal PointRGBNormal;
typedef pcl::PointCloud<PointRGBNormal> PointCloudNormalM;
typedef pcl::PointNormal PNormal;
typedef pcl::PointCloud<PNormal> PointCloudNormal;
typedef pcl::PointWithScale PointWithScale;
typedef pcl::PointCloud<PointWithScale> PointCloudScale;
// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;
// Define "LocalDescriptors" to be a pcl::PointCloud of pcl::FPFHSignature33 points
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
typedef pcl::VFHSignature308 GlobalDescriptorT;
typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;
// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points


struct PCD
{
    PointCloudRGB::Ptr cloud;
    std::string f_name;
    Eigen::Matrix4f Ti;

    PCD() : cloud (new PointCloudRGB) {};
};

struct ObjectFeatures{
	PointCloudPtr points;
	SurfaceNormalsPtr normals;
	PointCloudPtr keypoints;
	LocalDescriptorsPtr local_descriptors;
	GlobalDescriptorsPtr global_descriptor;
};

bool INIT_ALIGN = true;
bool ICP = false;
bool isRGB = false;
bool DOWN_SAMPLE = true;
bool INCREASE_PAIR = false;
// Define a new point representation for  x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PNormal>
{
  using pcl::PointRepresentation<PNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


const double FILTER_LIMIT = 1000.0;
const int MAX_SACIA_ITERATIONS = 500;

//units are meters:
const float VOXEL_GRID_SIZE = 0.01;
const double NORMALS_RADIUS = 0.04;
const double FEATURES_RADIUS = 0.04;
const double SAC_MAX_CORRESPONDENCE_DIST = 0.001;

// For initial alignment
float MIN_SAMPLE_DISTANCE = 0.05f;
float MAX_CORRESPONDENCE_DISTANCE= 0.01f*0.01f;
int NR_ITERATIONS= 500;

// For ICP refine Alignment
float ICP_MAX_CORRESPONDENCE_DISTANCE = 0.05f;
float ICP_OUTLIER_REJECTION_THRESHOLD = 0.05f;
float ICP_TRANSFORM_EPSILON = 1e-8;
float ICP_MAX_ITERATIONS = 50;


// Parameters for sift computation
const float MIN_SCALE = 0.005f;
const int N_OCTAVES = 6;
const int N_SCALES_PER_OCTAVE= 4;
const float MIN_CONSTRAST= 0.005f;
  
void filterCloud( PointCloud<PointXYZRGB>::Ptr );
PointCloudNormal::Ptr getNormals( PointCloudRGB::Ptr incloud );
PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloudRGB::Ptr incloud, PointCloudNormal::Ptr normals );

void view( PointCloud<PointRGB> & cloud );
SampleConsensusInitialAlignment<PointRGB, PointRGB, pcl::FPFHSignature33>
        align( PointCloudRGB::Ptr c1, PointCloudRGB::Ptr c2,
                   PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2 );


// for normals visualization
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
				PointCloudRGB::Ptr cloud,
				PointCloudNormal::Ptr normals);
void viewWithNormals(PointCloudRGB::Ptr cloud,
						PointCloudNormal::Ptr normals);

// pair align
Eigen::Matrix4f pairAlign(PointCloudRGB::Ptr cloud_src, PointCloudRGB::Ptr cloud_tgt,
		PointCloudNormal::Ptr normals_src, PointCloudNormal::Ptr normals_tgt);						

// estimate surface normals
SurfaceNormalsPtr
estimateSurfaceNormals (const PointCloudPtr & input, float radius);

// detect sift keys
PointCloudPtr
detectKeypoints (const PointCloudPtr & points, const SurfaceNormalsPtr & normals,
                 float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast);

// get local features
LocalDescriptorsPtr
computeLocalDescriptors (const PointCloudPtr & points, const SurfaceNormalsPtr & normals, 
                         const PointCloudPtr & keypoints, float feature_radius);				 
GlobalDescriptorsPtr
computeGlobalDescriptor (const PointCloudPtr & points, const SurfaceNormalsPtr & normals);
ObjectFeatures
computeFeatures (const PointCloudPtr & input);

Eigen::Matrix4f
computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations);
						 
Eigen::Matrix4f
refineAlignment (const PointCloudPtr & source_points, const PointCloudPtr & target_points, 
                 const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                 float outlier_rejection_threshold, float transformation_epsilon, float max_iterations);
			 

void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models){
	for(int i=1; i< argc; i++){
		std::string fname = std::string(argv[i]);
		PCD m;
		m.f_name = argv[i];
		pcl::io::loadPCDFile (argv[i], *m.cloud);
		//remove NAN points from the cloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

		models.push_back (m);
	}

}
			
int main(int argc, char** argv)
{
	PointCloudRGB final;

    time_t starttime = time(NULL);

    cout << "Loading clouds...";
    cout.flush();

	
    //open the clouds
    PointCloudRGB::Ptr cloud1 (new PointCloudRGB);
    PointCloudRGB::Ptr cloud2 (new PointCloudRGB);
	
	
	
    pcl::io::loadPCDFile(argv[1], *cloud1);
    pcl::io::loadPCDFile(argv[2], *cloud2);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud1,*cloud1, indices);
	pcl::removeNaNFromPointCloud(*cloud2,*cloud2, indices);

	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nFiltering input clouds...";
    cout.flush();

    //pass both through filters first
    filterCloud( cloud1 );
    filterCloud( cloud2 );

    PointCloudRGB::Ptr cloud1ds (new PointCloudRGB);
	*cloud1ds = *cloud1;
	PointCloudRGB::Ptr cloud2ds (new PointCloudRGB);
	*cloud2ds = *cloud2;
	if(DOWN_SAMPLE)
	{
		//downsample the clouds, but store the downsampled clouds seperately
		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\nDownsampling the clouds...";
		cout.flush();
		VoxelGrid<PointRGB> vox_grid;
		vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
		vox_grid.setInputCloud( cloud1 );
		vox_grid.filter( *cloud1ds );

		vox_grid.setInputCloud( cloud2 );
		vox_grid.filter( *cloud2ds );
		
	
	}
	cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds.\nStarting compute features...";
    cout.flush();
	
	// load features
	ObjectFeatures cloud1Features = computeFeatures(cloud1ds);
	ObjectFeatures cloud2Features = computeFeatures(cloud2ds);
	
	
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	if(INIT_ALIGN){
		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds. \nStarting compute initial transform...";
		cout.flush();
	
		transform = computeInitialAlignment(cloud1Features.keypoints, cloud1Features.local_descriptors,
					cloud2Features.keypoints, cloud2Features.local_descriptors,
					MIN_SAMPLE_DISTANCE, MAX_CORRESPONDENCE_DISTANCE, NR_ITERATIONS
					);
	}
	
	if(ICP){
		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds.\nStarting compute ICP...";
		cout.flush();
	
		transform = refineAlignment (cloud1ds, cloud2ds, transform, ICP_MAX_CORRESPONDENCE_DISTANCE,  
                             ICP_OUTLIER_REJECTION_THRESHOLD, ICP_TRANSFORM_EPSILON , ICP_MAX_ITERATIONS);

	
	}else if(INCREASE_PAIR){
		cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds.\nStarting compute ICP Increment...";
		cout.flush();
	
		pcl::transformPointCloud(*cloud1ds, *cloud1ds, transform);
		transform = pairAlign(cloud1ds, cloud2ds, getNormals(cloud1ds), getNormals(cloud2ds)) * transform;
	
	}
	
	
	pcl::transformPointCloud (*cloud1, *cloud1, transform);
	final = *cloud1;
    final += *cloud2;

    cout << "done. Time elapsed: " << time(NULL) - starttime << " seconds\n";
    cout << "Opening aligned cloud; will return when viewer window is closed.";
    cout.flush();

    view(final);

    return 1;
}




//computes the transformation for cloud2 so that it is transformed so that it is aligned with cloud1
SampleConsensusInitialAlignment<PointRGB, PointRGB, pcl::FPFHSignature33>
         align( PointCloudRGB::Ptr cloud1, PointCloudRGB::Ptr cloud2,
                        PointCloud<FPFHSignature33>::Ptr features1,
						PointCloud<FPFHSignature33>::Ptr features2 ) {

         SampleConsensusInitialAlignment<PointRGB, PointRGB, pcl::FPFHSignature33> sac_ia;
         Eigen::Matrix4f final_transformation;
         sac_ia.setInputSource( cloud2 );
         sac_ia.setSourceFeatures( features2 );
         sac_ia.setInputTarget( cloud1 );
         sac_ia.setTargetFeatures( features1 );



         sac_ia.setMinSampleDistance (MIN_SAMPLE_DISTANCE);
         sac_ia.setMaxCorrespondenceDistance ( MAX_CORRESPONDENCE_DISTANCE);
         sac_ia.setMaximumIterations (NR_ITERATIONS);

         PointCloud<PointRGB> finalcloud;
         sac_ia.align( finalcloud );
         return sac_ia;
}



PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloudRGB::Ptr cloud, PointCloudNormal::Ptr normals ) {

        PointCloud<FPFHSignature33>::Ptr features = PointCloud<FPFHSignature33>::Ptr (new PointCloud<FPFHSignature33>);
        search::KdTree<PointRGB>::Ptr search_method_ptr = search::KdTree<PointRGB>::Ptr (new search::KdTree<PointRGB>);
        FPFHEstimation<PointRGB, PNormal, FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(cloud);
        fpfh_est.setInputNormals( normals );
        fpfh_est.setSearchMethod(search_method_ptr );
        fpfh_est.setRadiusSearch( 0.03 );
        fpfh_est.compute( *features );
        return features;



}




PointCloudNormal::Ptr getNormals( PointCloudRGB::Ptr incloud ) {

    PointCloudNormal::Ptr normalsPtr = PointCloudNormal::Ptr (new PointCloudNormal);
    NormalEstimation<PointRGB, PNormal> norm_est;
    norm_est.setInputCloud( incloud );
    norm_est.setRadiusSearch( NORMALS_RADIUS );
    norm_est.compute( *normalsPtr );
    return normalsPtr;
}




void filterCloud( PointCloud<PointXYZRGB>::Ptr pc ) {

    pcl::PassThrough<PointRGB> pass;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.filter(*pc);

}

void view( PointCloud<PointRGB> & cloud ) {

        pcl::visualization::CloudViewer viewer1("Cloud Viewer");
        viewer1.showCloud( cloud.makeShared() );
    while( !viewer1.wasStopped() );

        return;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
				PointCloudRGB::Ptr cloud,
				PointCloudNormal::Ptr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<PointRGB> rgb(cloud);
  viewer->addPointCloud<PointRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<PointRGB, PNormal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void viewWithNormals(PointCloudRGB::Ptr cloud,
						PointCloudNormal::Ptr normals
			){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = normalsVis(cloud, normals);

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}


Eigen::Matrix4f pairAlign(PointCloudRGB::Ptr cloud_src, PointCloudRGB::Ptr cloud_tgt,
		PointCloudNormal::Ptr normals_src, PointCloudNormal::Ptr normals_tgt)
{

	// Compute surface normals and curvature
	PointCloudNormal::Ptr points_with_normals_src (new PointCloudNormal);
	PointCloudNormal::Ptr points_with_normals_tgt (new PointCloudNormal);
	
	pcl::copyPointCloud (*cloud_src, *points_with_normals_src);
	pcl::copyPointCloud (*cloud_tgt, *points_with_normals_tgt);
	
	// Instantiate custom point representation 
	MyPointRepresentation point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);
	
	// Align
	pcl::GeneralizedIterativeClosestPoint<PNormal, PNormal> reg;
	reg.setTransformationEpsilon (1e-16);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	reg.setMaxCorrespondenceDistance (0.5);  
	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	reg.setInputSource (points_with_normals_src);
	reg.setInputTarget (points_with_normals_tgt);
	
	// Run the same optimization in a loop
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloudNormal::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations (5);
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO ("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);
		
		pcl::transformPointCloud (*reg_result, *reg_result, reg.getFinalTransformation ());
		
		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		{
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance ()*0.9);
			PCL_INFO("Change max distance to %f.\n", reg.getMaxCorrespondenceDistance());
		}
		prev = reg.getLastIncrementalTransformation ();

		
	}
	
	targetToSource = Ti.inverse();
	
	return targetToSource;

}

SurfaceNormalsPtr
estimateSurfaceNormals (const PointCloudPtr & input, float radius)
{
  pcl::NormalEstimation<PointRGB, NormalT> normal_estimation;
  normal_estimation.setSearchMethod (pcl::search::KdTree<PointRGB>::Ptr (new pcl::search::KdTree<PointRGB>));
  normal_estimation.setRadiusSearch (radius);
  normal_estimation.setInputCloud (input);
  SurfaceNormalsPtr normals (new SurfaceNormals);
  normal_estimation.compute (*normals);

  return (normals);
}

PointCloudPtr
detectKeypoints (const PointCloudPtr & points, const SurfaceNormalsPtr & normals,
                 float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast)
{
  pcl::SIFTKeypoint<PointRGB, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod (pcl::search::KdTree<PointRGB>::Ptr (new pcl::search::KdTree<PointRGB>));
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);
  sift_detect.setInputCloud (points);
  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  sift_detect.compute (keypoints_temp);
  PointCloudPtr keypoints (new PointCloudRGB);
  pcl::copyPointCloud (keypoints_temp, *keypoints);

  return (keypoints);
}

LocalDescriptorsPtr
computeLocalDescriptors (const PointCloudPtr & points, const SurfaceNormalsPtr & normals, 
                         const PointCloudPtr & keypoints, float feature_radius)
{
  pcl::FPFHEstimation<PointRGB, NormalT, LocalDescriptorT> fpfh_estimation;
  fpfh_estimation.setSearchMethod (pcl::search::KdTree<PointRGB>::Ptr (new pcl::search::KdTree<PointRGB>));
  fpfh_estimation.setRadiusSearch (feature_radius);
  fpfh_estimation.setSearchSurface (points);  
  fpfh_estimation.setInputNormals (normals);
  fpfh_estimation.setInputCloud (keypoints);
  LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
  fpfh_estimation.compute (*local_descriptors);

  return (local_descriptors);
}
GlobalDescriptorsPtr
computeGlobalDescriptor (const PointCloudPtr & points, const SurfaceNormalsPtr & normals)
{
  pcl::VFHEstimation<PointRGB, NormalT, GlobalDescriptorT> vfh_estimation;
  vfh_estimation.setSearchMethod (pcl::search::KdTree<PointRGB>::Ptr (new pcl::search::KdTree<PointRGB>));
  vfh_estimation.setInputCloud (points);
  vfh_estimation.setInputNormals (normals);
  GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
  vfh_estimation.compute (*global_descriptor);

  return (global_descriptor);
}

ObjectFeatures
computeFeatures (const PointCloudPtr & input)
{
  ObjectFeatures features;
  features.points = input;
  features.normals = estimateSurfaceNormals (input, 0.05);
  features.keypoints = detectKeypoints (input, features.normals, 0.005, 10, 8, 1.5);
  features.local_descriptors = computeLocalDescriptors (input, features.normals, features.keypoints, 0.1);
  //features.global_descriptor = computeGlobalDescriptor (input, features.normals);

  return (features);
}

Eigen::Matrix4f
computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations)
{
  pcl::SampleConsensusInitialAlignment<PointRGB, PointRGB, LocalDescriptorT> sac_ia;
  sac_ia.setMinSampleDistance (min_sample_distance);
  sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
  sac_ia.setMaximumIterations (nr_iterations);
  
  sac_ia.setInputSource (source_points);
  sac_ia.setSourceFeatures (source_descriptors);

  sac_ia.setInputTarget (target_points);
  sac_ia.setTargetFeatures (target_descriptors);

  PointCloudRGB registration_output;
  sac_ia.align (registration_output);

  return (sac_ia.getFinalTransformation ());
}

Eigen::Matrix4f
refineAlignment (const PointCloudPtr & source_points, const PointCloudPtr & target_points, 
                 const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                 float outlier_rejection_threshold, float transformation_epsilon, float max_iterations)
{

  pcl::IterativeClosestPoint<PointRGB, PointRGB> icp;
  icp.setMaxCorrespondenceDistance (max_correspondence_distance);
  icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
  icp.setTransformationEpsilon (transformation_epsilon);
  icp.setMaximumIterations (max_iterations);

  PointCloudPtr source_points_transformed (new PointCloudRGB);
  pcl::transformPointCloud (*source_points, *source_points_transformed, initial_alignment);

  icp.setInputSource (source_points_transformed);
  icp.setInputTarget (target_points);

  PointCloudRGB registration_output;
  icp.align (registration_output);

  return (icp.getFinalTransformation () * initial_alignment);
}

