// 
// Point cloud utilities - mostly helper functions
// 
#include <point_cloud_utils/point_cloud_utils.h>

// STL Libraries
#include <sstream>
#include <iterator>
#include <random>

namespace point_cloud_utils
{

void populate_random_rgb(RGB_info& info)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> uni(0, 255);

	info.r = uni(gen);
	info.g = uni(gen);
	info.b = uni(gen);
}

}