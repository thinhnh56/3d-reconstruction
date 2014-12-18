cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(tuto-pairwise)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pairwise_incremental_registration pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES}) 

add_executable (test_keypoints test_keypoints.cpp)
target_link_libraries (test_keypoints ${PCL_LIBRARIES}) 


add_executable (passthrough passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES}) 

#add_executable (test_load_feature test_load_feature.cpp)
#target_link_libraries (test_load_feature ${PCL_LIBRARIES})
