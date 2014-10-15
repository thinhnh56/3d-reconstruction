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

//units are meters:
const float VOXEL_GRID_SIZE = 0.02;
// For initial alignment
float MIN_SAMPLE_DISTANCE = 0.05f;
float MAX_CORRESPONDENCE_DISTANCE= 0.01f*0.01f;
int NR_ITERATIONS= 500;

const double MAX_DIST  = 0.1;  // orig 0.1

// Moving Least Square
bool MLS = true;


// Global variable for our visualizer
pcl::visualization::PCLVisualizer *p;
PointCloudRGB::Ptr output(new PointCloudRGB);

// its left and right viewports
int vp_1, vp_2;


/* ---[ */
int main (int argc, char** argv)
{
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;

  double leaf_size = 0.1;
  bool downsample = true;

  loadData (argc, argv, data);

  // Check user input
  if (data.empty ())
    {
      PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
      PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i) will be registered against (i+2), etc");
      return (-1);
    }
  PCL_INFO ("Loaded %d datasets.\n", (int)data.size ());

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");


  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  PointCloudRGB::Ptr cloud_WRT_firstCloud (new PointCloudRGB);



  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), T_mouse_WRT_firstCloud;

  /********************** My add **********************/
  PCD source, target;
  const unsigned int MAX_CLOUD_SIZE = 100000;
  for (size_t i = 1; i < data.size (); ++i)
    {
      //set source to previous registered data
      source = data[i-1];
      /*
      if (i == 1)
	source = data[i-1];
      else{
	PointCloudRGB::Ptr temp (new PointCloudRGB);
	pcl::copyPointCloud(*cloud_WRT_firstCloud, *temp );
	source.cloud = temp;
	source.f_name = "previous_cloud";
      }*
      //source = data[i-1];
      //filter point cloud to make it small, resonable for registration
      //using random sample to extract specific number of point
      /*if (source.cloud->points.size() > MAX_CLOUD_SIZE){
	//begin filter

	pcl::RandomSample<PointRGB> random_filter;
	random_filter.setInputCloud(source.cloud);
	random_filter.setSample(MAX_CLOUD_SIZE);
	//put output directy to source
	random_filter.filter(*source.cloud);
	std::cout << std::endl  << "after filter: " << std::endl;
	}*/
      target = data[i];

      // Add coordinate system for viewport 1
      p->addCoordinateSystem(0.4, vp_1);
      // Set view from above
      p->setCameraPosition (0, 0, 10, 0, 0, 0, 0, 0, 0);

      // Add visualization data
      showCloudsLeft(target, source, p, vp_1);

       
      Eigen::Matrix4f T_target_WTR_mouse;
	
      PointCloudRGB::Ptr temp (new PointCloudRGB);
      PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source.cloud->points.size (), data[i].f_name.c_str (), target.cloud->points.size ());
			
      pairAlign (source.cloud, 
		 target.cloud, 
		 temp, 
		 T_mouse_WRT_firstCloud, 
		 leaf_size, 
		 true, 
		 INIT_ALIGN,
		 VOXEL_GRID_SIZE, 
		 MIN_SAMPLE_DISTANCE, MAX_CORRESPONDENCE_DISTANCE, NR_ITERATIONS,
		 MAX_DIST,
		 p, 
		 vp_1, vp_2
			
		 );
		
	
      //transform current pair into the global transform
      pcl::copyPointCloud(*temp, *cloud_WRT_firstCloud);
		
		
      data[i].Ti = T_mouse_WRT_firstCloud;
      //data[i].cloud = cloud_WRT_firstCloud;

      //Scrivo su file la trasformazione ottenuta
      ofstream myfile;
      myfile.open ("transformations.txt", std::ios_base::app);
      myfile << "\n";
      myfile << "from " << pcl::getFilenameWithoutPath(data[i].f_name) << " with respect to " << pcl::getFilenameWithoutPath(argv[1]) << "\n";
      myfile << data[i].Ti;
      myfile << "\n";
      myfile.close();

      //save aligned pair, transformed into the first cloud's frame
      std::stringstream ss;
      ss << i << ".pcd";
      pcl::io::savePCDFile (ss.str (), *cloud_WRT_firstCloud, true);
		
      /*    if(i==(int)data.size() - 1)
	{
	  // Moving Least Square
	  PointCloudNormal::Ptr dst(new PointCloudNormal);
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
  for (int i=1; i< data.size(); i++){
    PointCloudRGB::Ptr output = data[i].cloud;
    for (int j=i; j>0; j--){
      Eigen::Matrix4f targetToSource = data[j].Ti;
      pcl::transformPointCloud (*output, *output, targetToSource);
    }
    //combine target to first cloud
    *data[0].cloud += *output;
  }
  //save the final result to final.pcd
  pcl::io::savePCDFile("final.pcd", *data[o].cloud, true);
  view(*data[0].cloud);
  
}




  
