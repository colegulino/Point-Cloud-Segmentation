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

# Add library for point_cloud_utils
add_library(point_cloud_utils
			SHARED
			 src/point_cloud_utils.cpp
			)

# Add library for file_utils
add_library(file_utils
			SHARED
			 src/file_utils.cpp)
target_link_libraries(file_utils
					  point_cloud_utils
					 )

# Add library for kdtree
add_library(kdtree
			SHARED
			 src/kdtree.cpp)
target_link_libraries(kdtree
					  point_cloud_utils
					 )

# Link libraries for main file
target_link_libraries(point_cloud_seg 
					  render_utils
					  file_utils
					  kdtree
					  )