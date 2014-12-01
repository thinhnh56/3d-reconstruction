#include "includes/load_data.h"
#include "includes/registration.h"
#include "includes/surface.h"
#include <pcl/filters/random_sample.h>
#include <pcl/common/common_headers.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>

using namespace pcl;
using namespace std;

bool INIT_ALIGN = true;

// DownSample
float VOXEL_GRID_SIZE = 0.02;

// For initial alignment
float MIN_SAMPLE_DISTANCE = 0.05f;
float MAX_CORRESPONDENCE_DISTANCE = 0.01f * 0.01f;
int NR_ITERATIONS = 500;

const double MAX_DIST = 0.1;  // orig 0.1

// Moving Least Square
bool MLS = true;

// Global variable for our visualizer
pcl::visualization::PCLVisualizer *p;
PointCloudRGBPtr output(new PointCloudRGB);

// its left and right viewports
int vp_1, vp_2;

/* ---[ */
int main(int argc, char** argv) {
	// Load data
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;

	loadData(argc, argv, data);

	// Check user input
	if (data.empty()) {
		PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
		PCL_ERROR(
				"[*] - multiple files can be added. The registration results of (i, i) will be registered against (i+2), etc");
		return (-1);
	}
	PCL_INFO("Loaded %d datasets.\n", (int )data.size());

	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer(argc, argv,
			"Pairwise Incremental Registration example");

	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

	PointCloudRGBPtr cloud_WRT_firstCloud(new PointCloudRGB);

	Eigen::Matrix4f pairTransform;

	/********************** My add **********************/
	PCD source, target;
	const unsigned int MAX_CLOUD_SIZE = 100000;
	for (size_t i = 1; i < data.size(); ++i) {
		source = data[i - 1];
		target = data[i];

		// Add coordinate system for viewport 1
		p->addCoordinateSystem(0.4, "Left", vp_1);
		// Set view from above
		p->setCameraPosition(0, 0, 10, 0, 0, 0, 0, 0, 0);

		// Add visualization data
		showCloudsLeft(target, source, p, vp_1);
		PointCloudRGBPtr temp(new PointCloudRGB);
		PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(),
				source.cloud->points.size(), data[i].f_name.c_str(),
				target.cloud->points.size());

		pairTransform = pairAlign(source.cloud, target.cloud, temp, true,
				INIT_ALIGN, VOXEL_GRID_SIZE, MIN_SAMPLE_DISTANCE,
				MAX_CORRESPONDENCE_DISTANCE, NR_ITERATIONS, MAX_DIST, p, vp_1,
				vp_2

				);

		data[i].Ti = pairTransform;

		//save aligned pair, transformed into the first cloud's frame

		/*    if(i==(int)data.size() - 1)
		 {
		 // Moving Least Square
		 PointCloudNormalPtr dst(new PointCloudNormal);
		 if(MLS){
		 std::cout << "Starting moving least square..." << std::endl;
		 *dst = movingLeastSquare(cloud_WRT_firstCloud, 0.01);
		 }else{
		 pcl::copyPointCloud(*cloud_WRT_firstCloud, *dst);

		 }

		 pcl::copyPointCloud(*dst, *output);
		 //view(*output);
		 greedyProjection(dst, "mesh.vtk");


		 }*/
	}
	for (int i = 1; i < data.size(); i++) {
		PointCloudRGBPtr output = data[i].cloud;
		for (int j = i; j > 0; j--) {
			Eigen::Matrix4f targetToSource = data[j].Ti;
			pcl::transformPointCloud(*output, *output, targetToSource);
		}
		//combine target to first cloud
		*data[0].cloud += *output;
	}
	//save the final result to final.pcd
	pcl::io::savePCDFile("final.pcd", *data[0].cloud, true);
	view(*data[0].cloud);

}

