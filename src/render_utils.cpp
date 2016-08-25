// 
// Utilities for rendering point cloud data
// 
#include <render_utils/render_utils.h>

// Boost libraries
#include <boost/thread/thread.hpp>

namespace render_utils
{

std::shared_ptr<pcl::visualization::PCLVisualizer> 
	visualize_rgb_pc(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud)
{
	auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
	viewer->setBackgroundColor(0,0,0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
											 3, 
											 "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

std::shared_ptr<pcl::visualization::PCLVisualizer>
	get_simple_viewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud)
{
	auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(point_cloud, "Point Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
											 1, 
											 "Point Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return viewer;
}

void loop_viewer(const std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
}

}