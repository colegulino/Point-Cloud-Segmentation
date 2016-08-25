# CMake file for point cloud segmentation practice
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the project info
project(PointCloudSeg)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_executable(point_cloud_seg ${PROJECT_SOURCE_DIR}/src/point_cloud_seg.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add library for render_utils
add_library(render_utils
			SHARED
			 src/render_utils.cpp
			)
target_link_libraries(render_utils 
					  ${PCL_LIBRARIES}
					 )

# Link libraries for main file
target_link_libraries(point_cloud_seg 
					  render_utils
					  )