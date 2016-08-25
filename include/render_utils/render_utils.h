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

// Example from: http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
std::shared_ptr<pcl::visualization::PCLVisualizer> 
	visualize_rgb_pc(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud);

void loop_viewer(const std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

}