cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl-tools)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (statistical_removal src/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})
add_executable (pcd2ply src/pcd2ply.cpp)
target_link_libraries (pcd2ply ${PCL_LIBRARIES})
