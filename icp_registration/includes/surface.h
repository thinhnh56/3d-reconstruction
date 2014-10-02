#ifndef _SURFACE_
#define _SURFACE_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


#include "typedefs.h"
  
 
	PointCloudNormal movingLeastSquare(PointCloudRGB::Ptr cloud, float radius);
  
	void greedyProjection(PointCloudNormal::Ptr cloud_with_normals, char* fileName);
  #endif