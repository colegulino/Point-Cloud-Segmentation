// 
// Utilities for rendering point cloud data
// 
#pragma once

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

// Standard library 
#include <memory>

namespace render_utils
{

// 
// Function that generates an RGB visualization of a 3D point cloud
// Example from: http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
// 
// @param point_cloud The point cloud that you want to visualize
// @return A PCL visualizer object
// 
std::shared_ptr<pcl::visualization::PCLVisualizer> 
	visualize_rgb_pc(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud);

// 
// Function that generates a simple visualization of a 3D point cloud without color
// Example from: http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
// 
// @param point_cloud The point cloud that you want to visualize
// @return A PCL visualizer object
// 
std::shared_ptr<pcl::visualization::PCLVisualizer>
	get_simple_viewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud);

// 
// Function that loops through visualization of a PCL visualizer so that it will maintain in 
// window until the window is closed.
// 
// @param viewer A PCL viewer object
// 
void loop_viewer(const std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

}