#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

//convenient typedefs
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointRGB>::Ptr PointCloudRGBPtr;

typedef pcl::PointXYZRGBNormal PointRGBNormal;
typedef pcl::PointCloud<PointRGBNormal> PointCloudRGBNormalM;
typedef pcl::PointNormal PNormal;
typedef pcl::PointCloud<PNormal> PointCloudNormal;
typedef pcl::PointCloud<PNormal>::Ptr PointCloudNormalPtr;
typedef pcl::PointWithScale PointWithScale;
typedef pcl::PointCloud<PointWithScale> PointCloudWithScale;
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

class PCD {
public:
	PointCloudRGBPtr cloud;
	std::string f_name;
	Eigen::Matrix4f Ti;

	PCD() :
			cloud(new PointCloudRGB) {
	}
	;
};

class MyPointRepresentation: public pcl::PointRepresentation<PNormal> {
	using pcl::PointRepresentation<PNormal>::nr_dimensions_;
public:
	MyPointRepresentation() {
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PNormal &p, float * out) const {
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

#endif
