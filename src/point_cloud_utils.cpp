// 
// Point cloud utilities - mostly helper functions
// 
#include <point_cloud_utils/point_cloud_utils.h>

// STL Libraries
#include <sstream>
#include <iterator>

namespace point_cloud_utils
{

std::vector<std::string> split_string_spaces(const std::string& s)
{
	std::istringstream iss(s);

	return {std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
}

}