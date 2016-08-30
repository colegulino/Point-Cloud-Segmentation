// 
// Point cloud utilities - mostly helper functions
// 
#pragma once

// STL Libraries
#include <vector>
#include <string>
#include <queue>

namespace point_cloud_utils
{

struct RGB_info
{
	std::string class_name;
	int class_number;
	double r;
	double g;
	double b;

	RGB_info()
		: r(255), g(255), b(255)
	{

	}
};

struct point
{
	std::vector<double> location;
	RGB_info info;

	point(const std::vector<double>& loc, const RGB_info& inf)
		: location(loc), info(inf)
	{

	}

	point()
	{
		
	}
};

}

using point_cloud_t = std::vector<point_cloud_utils::point>;

namespace point_cloud_utils
{

struct point_node
{
	point p;

	std::shared_ptr<point_node> left;
	std::shared_ptr<point_node> right;

	///
	/// Constructor for the point node
	///
	/// @param P the point type where the node is
	/// @param l The left neighbor of the point node
	/// @param r The right neighbor of the point node
	///
	point_node(const point& P, 
			   const std::shared_ptr<point_node>& l, 
			   const std::shared_ptr<point_node>& r)
		: p(P), left(l), right(r)
	{

	}
};


///
/// Generates three random integers between 0 and 255 in place for an RGB_info object
///
/// @param info RGB_info object that we want to generate random RGB integer values for
///
void populate_random_rgb(RGB_info& info);

template<typename Func>
void preorder_traverse(const std::shared_ptr<point_node>& root, Func&& func)
{
	if(root == nullptr) return;

	func(root);

	preorder_traverse(root->left, func);
	preorder_traverse(root->right, func);
}

template<typename Func>
void bfs(const std::shared_ptr<point_node>& root, Func&& func)
{
	if(root == nullptr) return;

	std::queue<std::shared_ptr<point_node>> q;
	q.push(root);

	while(!q.empty())
	{
		auto node = q.front();
		func(node);
		q.pop();

		if(node->left != nullptr) q.push(node->left);

		if(node->right != nullptr) q.push(node->right);
	}
}

}
