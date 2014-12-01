#include <iostream>
#include "includes/surface.h"
#include "includes/typedefs.h"
#include "includes/visualization.h"
#include <pcl/filters/filter.h>

using namespace std;

int main(int argc, char** argv) {

	PointCloudRGBPtr cloud(new PointCloudRGB());
	// Load bun0.pcd -- should be available with the PCL archive in test 
	pcl::io::loadPCDFile("xinputCloud1.pcd", *cloud);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	PointCloudNormalPtr dst(new PointCloudNormal);
	*dst = movingLeastSquare(cloud, 0.01);
	PointCloudRGBPtr output(new PointCloudRGB);
	pcl::copyPointCloud(*dst, *output);
	view(*output);
	return 0;
}
