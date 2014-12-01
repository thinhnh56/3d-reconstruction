cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project("3D Reconstruction")

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories ("${PROJECT_SOURCE_DIR}/includes")
add_subdirectory (includes/solution) 

add_executable (semiauto_registration semiauto_registration.cpp)
#add_executable (fov_example fov_example.cpp)
#add_executable (picked_point picked_point.cpp)

#add_executable (test_utils test_utils.cpp)

#add_executable (test_mls test_mls.cpp)
#add_executable (test_greedy_projection test_greedy_projection.cpp)

target_link_libraries (semiauto_registration surface load_data registration feature_estimation visualization filter ${PCL_LIBRARIES} )

#target_link_libraries (fov_example load_data visualization filter utils ${PCL_LIBRARIES} )
#target_link_libraries (picked_point load_data visualization filter utils ${PCL_LIBRARIES} )

#target_link_libraries (test_mls surface visualization ${PCL_LIBRARIES} )
#target_link_libraries (test_greedy_projection surface visualization ${PCL_LIBRARIES} )
#target_link_libraries (test_utils utils )
