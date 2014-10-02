  
#ifndef LOADDATA_H
#define LOADDATA_H

#include "typedefs.h"
#include <pcl/io/pcd_io.h>

void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);

#endif