//
// kd-tree implementation
//
#include <kdtree/kdtree.h>

// STL functions
#include <cmath>
#include <string>

void kdtree::build(const point_cloud_t& point_cloud, int depth)
{
	point_cloud_ = point_cloud;

	root_ = grow(point_cloud, depth);
}

node_ptr kdtree::grow(point_cloud_t point_cloud, int depth)
{
	if(point_cloud.size() == 0) return nullptr;

	int k = point_cloud.front().location.size();

	// Axis where to split the tree
	int axis = depth % k;

	// Sort on this axis
	std::sort(point_cloud.begin(), point_cloud.end(),
			  [axis](const point_cloud_utils::point& p1, const point_cloud_utils::point& p2) -> bool
			  {
			      return p1.location[axis] < p2.location[axis];
			  });

	// Split at the median
	int median = std::round(point_cloud.size() / 2);
	auto med_it = point_cloud.begin() + median;

	return std::make_shared<point_cloud_utils::point_node>(point_cloud[median],
			grow(point_cloud_t(point_cloud.begin(), med_it), depth + 1),
			grow(point_cloud_t(std::next(med_it, 1), point_cloud.end()),depth + 1));
}

p_queue kdtree::find_knn(point_cloud_utils::point point, int k, DistFxn dist)
{
	if(k > size())
	{
		throw std::runtime_error("Number of neighbors searched for " + std::to_string(k) +
								 " is greater than the size of the tree " + std::to_string(size()));
	}

	// Build up the priority queue
	// Should be less than so that those that are furthest from the point are 
	// poped first
	auto cmp = [&point, &dist](const std::shared_ptr<point_cloud_utils::point_node>& p1, 
							   const std::shared_ptr<point_cloud_utils::point_node>& p2) -> bool
	{
		return dist(point, p1->p) < dist(point, p2->p);
	};
	
	p_queue q(cmp);

	find_neighbors(point, root_, q, k, dist, 0);

	return q;
}

void kdtree::find_neighbors(point_cloud_utils::point point, 
							node_ptr root, 
							p_queue& q, 
							int k,
							DistFxn dist, 
							int depth)
{
	if(root == nullptr) return;

	int d = root->p.location.size();
	int axis = depth % d;

	q.push(root);
	if(q.size() > k) q.pop();

	// 0 for left
	// 1 for right
	int last_chosen = 0;

	// Search depending on the side that the best quess is on
	if(point.location[axis] < root->p.location[axis])
	{
		last_chosen = 0;
		find_neighbors(point, root->left, q, k, dist, depth + 1);
	}
	else
	{
		last_chosen = 1;
		find_neighbors(point, root->right, q, k, dist, depth + 1);
	}

	// If the candidate hypersphere doesn't cross the current splitting plane or if
	// the priority queue hasnt reached size k search on the other side
	if(q.size() < k or 
	   std::abs(root->p.location[axis] - point.location[axis]) < 
	   dist(point, q.top()->p))
	{
		if(last_chosen == 0)
		{
			find_neighbors(point, root->right, q, k, dist, depth + 1);
		}
		else
		{
			find_neighbors(point, root->left, q, k, dist, depth + 1);
		}
	}
}

