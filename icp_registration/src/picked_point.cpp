#include "includes/typedefs.h"
#include "includes/visualization.h"
#include "includes/utils.h"


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

using namespace pcl;
using namespace std;

visualization::PCLVisualizer vis;
	
PointRGB origin(0, 0, 0);
	
double r = 1;
double g = 0;
double b = 0;

int num_points = 0;
PointRGB first_point;

void pp_callback (const visualization::PointPickingEvent& event, void* viewer_void) 
{ 
	if (event.getPointIndex () == -1) 
		return; 
		
	PointRGB selected_point;
	event.getPoint(selected_point.x, selected_point.y, selected_point.z);
	cout << "select point: " << selected_point.x << " " << selected_point.y << " " << selected_point.z << endl;
//	cout << event.getPointIndex (); 
	
	num_points++;
	
	std::stringstream ss ("line");
    ss << event.getPointIndex();

	if(num_points==1){
		drawLine(vis, origin, selected_point, ss.str(), 0, 0, 1);	
		first_point = selected_point;
	}else{
		drawLine(vis, origin, selected_point, ss.str(), r, g, b);
		cout << "angle: " << calculate_angleA(origin, first_point, selected_point) << endl;
    }

	
} 

int main(int argc, char** argv){

	pcl::console::print_info("Hold shift key + click on the point in points cloud\n");
	pcl::console::print_info("to draw a line from origin to this point.\n");
	pcl::console::print_info("blue line is the first ray of angle.\n");
	pcl::console::print_info("red lines is other ray of angle.\n");

	vis.registerPointPickingCallback (pp_callback, (void*)&vis); 
	
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	pcl::io::loadPCDFile (argv[1], *cloud);
	//remove NAN points from the cloud
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	
	vis.addPointCloud(cloud, "point_cloud");
	
	
	vis.resetCamera();
	vis.spin();


}