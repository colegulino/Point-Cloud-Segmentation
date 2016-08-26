// 
// Point cloud utilities - mostly helper functions
// 
#pragma once

// STL Libraries
#include <vector>
#include <string>

namespace point_cloud_utils
{

struct RGB_info
{
	std::string class_name;
	int class_number;
	double r;
	double g;
	double b;
};

///
/// Generates three random integers between 0 and 255 in place for an RGB_info object
///
/// @param info RGB_info object that we want to generate random RGB integer values for
///
void populate_random_rgb(RGB_info& info);

}