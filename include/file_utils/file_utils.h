// 
// File utilities - functions for reading point clouds from files
// 
#pragma once

// Point Cloud Utils
#include <point_cloud_utils/point_cloud_utils.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL Libraries
#include <vector>
#include <string>
#include <map>

using class_map = std::map<int, point_cloud_utils::RGB_info>;

namespace file_utils
{

///
/// Function that splits a string using a space as a delimiter
/// 
/// @param s String to split
/// @return A vector of words split from a string
/// 
std::vector<std::string> split_string_spaces(const std::string& s);

/// 
/// Helper function to open a file and check to ensure that it openned with a proper file
/// 
/// @param file_path Path of the file to be opened
/// @return An in file stream
/// @exception Throws if the file cannot be found
/// 
std::ifstream open_file(const std::string& file_path);

///
/// Helper function to determine if a string is a number
///
/// @param word String to analyze
/// @return A boolean to determine if what is provided is a number
///
bool is_number(const std::string& word);

/// 
/// Function that gets a class_maping from class number to RGB info from a .stats file
/// 
/// @param file_path Name of the file path (should end in .stats)
/// @return A class map that maps integer values to RGB info
/// 
class_map get_class_map_from_file(const std::string& file_path);

/// 
/// Generates a point cloud from a file where the points are in the format:
/// (x y z label_id confidence)
/// 
/// @param file_path Name of the file path that should end in .xyz_label_conf
/// @return Point cloud based on the text file
/// 
pcl::PointCloud<pcl::PointXYZ>::Ptr get_xyz_point_cloud_from_file(const std::string& file_path);

/// 
/// Generates a point cloud from a file where the points are in the format:
/// (x y z label_id confidence) with rgb color based on label_id
/// 
/// @param file_path Name of the file path that should end in .xyz_label_conf
/// @return Point cloud based on the text file with rgb color based on the label_ids
/// 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
	get_xyzrgb_point_cloud_from_file(const std::string& file_path, const class_map& map);

}