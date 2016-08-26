// Point cloud rendering utilities
#include <render_utils/render_utils.h>

// File reading utils
#include <file_utils/file_utils.h>

// Standard Libraries
#include <iostream>

int main(int argc, const char * argv[]) 
{
	const auto file_name = "Data/cmu_oakland_data/oakland_part2_ae.xyz_label_conf";

	// std::cout << "Got file name" << std::endl;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
	// try
	// {
	// 	point_cloud = file_utils::get_xyz_point_cloud_from_file(file_name);
	// }
	// catch(const std::runtime_error& e)
	// {
	// 	throw;
	// }

	// std::cout << "Got point cloud." << std::endl;

	// std::cout << "Point Cloud Size: " << point_cloud->size() << std::endl;;

	// const auto viewer = render_utils::get_simple_viewer(point_cloud);

	// std::cout << "Got viewer" << std::endl;
	// render_utils::loop_viewer(viewer);

	const auto file_path = "Data/cmu_oakland_data/stats/all.stats";

	const auto& class_map = file_utils::get_class_map_from_file(file_path);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;
	try
	{
		point_cloud = file_utils::get_xyzrgb_point_cloud_from_file(file_name, class_map);
	}
	catch(const std::runtime_error& e)
	{
		throw;
	}

	const auto viewer = render_utils::visualize_rgb_pc(point_cloud);
	render_utils::loop_viewer(viewer);

	// for(auto it = class_map.begin(); it != class_map.end(); ++it)
	// {
	// 	auto class_no = it->first;
	// 	auto info = it->second;
	// 	std::cout << "No: " << class_no << " | Name: " << info.class_name
	// 			  << " | R: " << info.r << " | G: " << info.g << " | B: " << info.b << std::endl;
	// }

    return 0;
}
