#ifndef LOADDATA_H
#define LOADDATA_H

#include "typedefs.h"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string.h>

using namespace std;

void loadData(string fileName, PCD& pcdFile);
void loadData(int argc, char **argv,
		std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);

#endif
