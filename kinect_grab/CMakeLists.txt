cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(kinect_grab_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kinect_grab kinect_grab.cpp)
target_link_libraries (kinect_grab ${PCL_LIBRARIES})

