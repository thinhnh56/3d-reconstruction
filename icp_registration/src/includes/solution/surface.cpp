  #include "../surface.h"
  
  PointCloudNormal movingLeastSquare(PointCloudRGB::Ptr cloud, float radius){
	  pcl::search::KdTree<PointRGB>::Ptr tree (new pcl::search::KdTree<PointRGB>);

	  PointCloudNormal mls_points;

	  pcl::MovingLeastSquares<PointRGB, PNormal> mls;
	 
	  mls.setComputeNormals (true);

	  // Set parameters
	  mls.setInputCloud (cloud);
	  mls.setPolynomialFit (true);
	  mls.setSearchMethod (tree);
	  mls.setSearchRadius (radius);

	  // Reconstruct
	  mls.process (mls_points);

	  // Save output
	  pcl::io::savePCDFile ("mls_sample.pcd", mls_points);
	  
	  return mls_points;
	  
  }
  
  	pcl::PolygonMesh::Ptr greedyProjection(PointCloudNormal::Ptr cloud_with_normals, std::string fileName){
		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius (0.025);

		// Set typical values for the parameters
		gp3.setMu (2.5);
		gp3.setMaximumNearestNeighbors (100);
		gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(false);

		// Get result
		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (*triangles);

		pcl::io::saveVTKFile (fileName, *triangles);
		
		return triangles;

	}
	
	/*pcl::PolygonMesh::Ptr vtkSmoother(pcl::PolygonMesh::Ptr mesh){
		pcl::surface::VTKSmoother vtkSmoother;
		vtkSmoother.convertToVTK(*mesh);
		vtkSmoother.smoothMeshWindowedSinc();
		vtkSmoother.convertToPCL(*mesh);
		return mesh;
	}*/
