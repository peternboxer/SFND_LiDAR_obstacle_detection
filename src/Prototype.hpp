#ifndef PROTOTYPE_H
#define PROTOTYPE_H

#include <unordered_set>
#include "processPointClouds.h"

std::unordered_set<int> 
RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);

struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(pcl::PointXYZI arr, int setId)
		:id(setId), left(NULL), right(NULL) {
			point.emplace_back(arr.x);
			point.emplace_back(arr.y);
			point.emplace_back(arr.z);
		}
	~Node(){
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;
	KdTree(): root(nullptr){}
	~KdTree(){delete root;}

	void helper(Node **node, int depth, const pcl::PointXYZI& point, const int& id)
	{
		if (*node == nullptr){
			*node = new Node(point, id);
			(*node)->left = nullptr;
			(*node)->right = nullptr;
		}
		else{
			Node curr(point, id);	// convert to temporary node
			uint32_t cd = depth % 3;
			// left
			if ((*node)->point[cd] > curr.point[cd]){
				helper(&(*node)->left, depth + 1, point, id);
			}
			// right
			else{
				helper(&(*node)->right, depth + 1, point, id);
			}
		}
	}

	void insert(pcl::PointXYZI point, int id)
	{
		helper(&root, // Node
			   0,	  // Depth
			   point, // input point
			   id);	  // pont id
	}

	void search_helper(Node **node, const pcl::PointXYZI& target, std::vector<int> &idx, int ds, const float& distanceTol){

		float distance;
		Node curr(target, 0);
		while (*node != nullptr){
			distance = std::sqrt(std::pow(target.x - (*node)->point[0], 2) + std::pow(target.y - (*node)->point[1], 2) + std::pow(target.z - (*node)->point[2], 2));
			if (distance <= distanceTol){
				idx.emplace_back((*node)->id);
			}
			if ((curr.point[ds % 3] - distanceTol) < (*node)->point[ds % 3]){
				search_helper(&(*node)->left, target, idx, ds + 1, distanceTol);
			}
			if ((curr.point[ds % 3] + distanceTol) > (*node)->point[ds % 3]){
				search_helper(&(*node)->right, target, idx, ds + 1, distanceTol);
			}
			return;
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const pcl::PointXYZI& target, const float &distanceTol)
	{
		std::vector<int> ids;
		int depth = 0;
		search_helper(&root, target, ids, depth, distanceTol);
		return ids;
	}
};

void Proximity(
					const pcl::PointCloud<pcl::PointXYZI>::Ptr& points, 
					std::vector<int>& cluster, 
					const size_t& idx, 
					std::vector<bool>& processed, 
					KdTree* tree, 
					const float& distanceTol
				);

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
euclideanCluster(
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& points, 
                    KdTree* tree, 
                    const float& distanceTol,
					uint32_t minSize,
					uint32_t maxSize
                );

#endif