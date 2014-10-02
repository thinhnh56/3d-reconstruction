#include "../load_data.h"

#include <pcl/filters/filter.h>


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