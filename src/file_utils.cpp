// 
// File utilities - functions for reading point clouds from files
// 
#include <file_utils/file_utils.h>

// STL Libraries
#include <sstream>
#include <iterator>

namespace file_utils
{

std::vector<std::string> split_string_spaces(const std::string& s)
{
	std::istringstream iss(s);

	return {std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
}

}