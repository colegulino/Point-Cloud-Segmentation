// 
// File utilities - functions for reading point clouds from files
// 
#include <file_utils/file_utils.h>

// STL Libraries
#include <sstream>
#include <iterator>
#include <fstream>
#include <stdexcept>
#include <cctype>

namespace file_utils
{

std::vector<std::string> split_string_spaces(const std::string& s)
{
	std::istringstream iss(s);

	return {std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
}

std::ifstream open_file(const std::string& file_path)
{
	std::ifstream infile(file_path);

	if(!infile.good())
	{
		throw std::runtime_error("File path " + file_path + " cannot be found");
	}
	else
	{
		return infile;
	}
}

bool is_number(const std::string& word)
{
	auto it = std::find_if(word.begin(), word.end(),
			  [](char c)
			  {
			  	return !std::isdigit(c);
			  });

	return !word.empty() and it == word.end();
}

class_map get_class_map_from_file(const std::string& file_path)
{
	auto infile = file_utils::open_file(file_path);

	class_map c_map;

	std::string line;
	while(std::getline(infile, line))
	{
		auto words = file_utils::split_string_spaces(line);

		if(!is_number(words.front())) continue;

		// Here I want to randomly generate RGB values between 0 and 255
		// I assume that the data is given as: (class_no -- class_string -- num_class)
		// I assume that the file contains unique class integers 
		point_cloud_utils::RGB_info info;
		info.class_number = std::stoi(words.front());
		info.class_name = words[2];
		point_cloud_utils::populate_random_rgb(info);

		c_map[info.class_number] = info;
	}

	return c_map;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_xyz_point_cloud_from_file(const std::string& file_path)
{
	auto infile = open_file(file_path);

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);;

	std::string line;
	while(std::getline(infile, line))
	{
		auto words = split_string_spaces(line);

		// Make sure the first word does not begin with #
		const auto& first_word = words.front().c_str();
		const auto& first_letter = first_word[0];
		if(first_letter == '#') continue;

		pcl::PointXYZ point;

		point.x = std::stod(words[0]);
		point.y = std::stod(words[1]);
		point.z = std::stod(words[2]);

		point_cloud->push_back(point);
	}

	return point_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
	get_xyzrgb_point_cloud_from_file(const std::string& file_path, const class_map& map)
{
	auto infile = open_file(file_path);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);;

	std::string line;
	while(std::getline(infile, line))
	{
		auto words = split_string_spaces(line);

		// Make sure the first word does not begin with #
		const auto& first_word = words.front().c_str();
		const auto& first_letter = first_word[0];
		if(first_letter == '#') continue;

		pcl::PointXYZRGB point;

		point.x = std::stod(words[0]);
		point.y = std::stod(words[1]);
		point.z = std::stod(words[2]);

		auto info = map.find(std::stod(words[3]))->second;

		point.r = info.r;
		point.g = info.g;
		point.b = info.b;

		point_cloud->push_back(point);
	}

	return point_cloud;
}

}