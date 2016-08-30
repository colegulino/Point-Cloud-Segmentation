//
// Implementation of kdtree
//
#pragma once

// Point cloud utils -> has point and point cloud data structures
#include <point_cloud_utils/point_cloud_utils.h>

// STL
#include <functional>
#include <limits>

using DistFxn = std::function<double(point_cloud_utils::point, point_cloud_utils::point)>;
using Cmp = std::function<bool(std::shared_ptr<point_cloud_utils::point_node>, 
							   std::shared_ptr<point_cloud_utils::point_node>)>;
using node_ptr = std::shared_ptr<point_cloud_utils::point_node>;
using p_queue = std::priority_queue<node_ptr, 
									std::vector<node_ptr>, 
									Cmp>;

class kdtree
{
private:
	node_ptr root_;

	point_cloud_t point_cloud_;

	///
	/// Helper function to recurrsively search for nearest neighbors
	///
	/// @param point Point to search for
	/// @param root Root of the tree to search
	/// @param q Reference of the priority queue that you will return at the end
	/// @param k Number of nearest neighbors you want to find
	/// @param depth Depth of the current tree. Starts at 0.
	///
	void find_neighbors(point_cloud_utils::point point, 
						node_ptr root, 
						p_queue& q, 
						int k,
						DistFxn dist, 
						int depth = 0);

	///
	/// Helper function to recurrsively build the kdtree
	///
	/// @param point_cloud point cloud type to create the kdtree based on
	/// @param depth Depth of the current tree. Starts at 0.
	///
	node_ptr grow(point_cloud_t point_cloud, int depth);

public:
	///
	/// Returns the root of the tree
	///
	/// @return Root of the tree
	///
	node_ptr root() { return root_; }

	///
	/// Gets the size of the kdtree
	///
	/// @return Size of the point_cloud_t you submitted
	///
	double size() { return point_cloud_.size(); }

	///
	/// Builds the kd tree and sets the root of the tree
	///
	/// @param point_cloud point cloud type to create the kdtree based on
	/// @param depth Depth of the current tree. Starts at 0.
	///
	void build(const point_cloud_t& point_cloud, int depth = 0);

	///
	/// Constructor
	/// @param point_cloud point cloud type to create the kdtree based on
	///
	kdtree(const point_cloud_t& point_cloud)
	{
		point_cloud_ = point_cloud;

		kdtree::build(point_cloud);
	}

	///
	/// Find the k-nearest neighbors to a point
	///
	/// @param point Point that you want to find the k-nearest neighbors too
	/// @param k Number of neighbors you want to find
	/// @param dist Distance function
	/// @return Priority queue that will be modified with the k-nearest neighbors
	///
	p_queue find_knn(point_cloud_utils::point point, int k, DistFxn dist);

};
