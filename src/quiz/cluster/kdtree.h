/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL) {}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(nullptr)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void helper(Node **node, int depth, const std::vector<float> &point, const int &id){
		if (*node == nullptr){
			*node = new Node(point, id);
			(*node)->left = nullptr;
			(*node)->right = nullptr;
		}
		else{
			uint8_t cd = depth % 2; // odd or even depth
			// left
			if ((*node)->point[cd] > point[cd]){
				helper(&(*node)->left, depth + 1, point, id);
			}
			// right
			else{
				helper(&(*node)->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		helper(&root, // Node
			   0,	  // Depth
			   point, // input point
			   id);	  // pont id
	}

	void search_helper(Node **node, const std::vector<float> &target, std::vector<int> &idx, int ds, const float &distanceTol)
	{

		float distance;
		while (*node != nullptr)
		{
			distance = std::sqrt(std::pow(target[0] - (*node)->point[0], 2) + std::pow(target[1] - (*node)->point[1], 2));
			if (distance <= distanceTol)
			{
				idx.emplace_back((*node)->id);
			}

			if ((target[ds % 2] - distanceTol) < (*node)->point[ds % 2])
			{
				search_helper(&(*node)->left, target, idx, ds + 1, distanceTol);
			}
			if ((target[ds % 2] + distanceTol) > (*node)->point[ds % 2])
			{
				search_helper(&(*node)->right, target, idx, ds + 1, distanceTol);
			}
			return;
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float> &target, const float &distanceTol)
	{
		std::vector<int> ids;
		ids.reserve(30);
		int depth = 0;

		search_helper(&root, target, ids, depth, distanceTol);
		//std::cout << "size = " << ids.size() << "\n";
		return ids;
	}
};
