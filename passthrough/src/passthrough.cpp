#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  /* Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  
  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
	      << cloud->points[i].y << " " 
	      << cloud->points[i].z << std::endl;
  */

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test.pcd", *cloud) == -1 )
    {
      PCL_ERROR("Couldn't read file test.pcd\n");
      return (-1);
    }
  // Create the filtering object
  /*passthrough filter
  */
  // Create the filtering object

  if (strcmp(argv[1], "-r") == 0){
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (*cloud_filtered);
  }
  else if (strcmp(argv[1], "-c") == 0){
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new
						      pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
									      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
									      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 0.8)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_filtered);
  }
  else if (strcmp(argv[1], "-s") == 0){
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
  }
  else if (strcmp(argv[1], "-p") == 0)
    {
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud (cloud);

      pass.setFilterFieldName (argv[2]);
      double low = atof(argv[3]);
      double high = atof(argv[4]);
      pass.setFilterLimits (low, high);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);
    }
  /*
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
	      << cloud_filtered->points[i].y << " " 
	      << cloud_filtered->points[i].z << std::endl;
  */
  std::cerr << "Cloud after filtering: "<< cloud_filtered->points.size() <<std::endl;
  pcl::io::savePCDFileASCII("test_out.pcd", *cloud_filtered);
  return (0);
}
