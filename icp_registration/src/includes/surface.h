#ifndef _SURFACE_
#define _SURFACE_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "typedefs.h"

PointCloudNormal movingLeastSquare(PointCloudRGBPtr cloud, float radius);

pcl::PolygonMesh::Ptr greedyProjection(PointCloudNormalPtr cloud_with_normals,
		std::string fileName);

//pcl::PolygonMesh::Ptr vtkSmoother(pcl::PolygonMesh::Ptr mesh);
#endif
