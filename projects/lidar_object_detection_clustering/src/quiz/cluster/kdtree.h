/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node ** node, unsigned depth, std::vector<float> point, int id)
	{
		if(*node != NULL) {
			// x split when (depth % 2) = 0; y split when (depth % 2) = 1
			// index for accessing point.x is 0; index for accessing point.y is 1
			if(point[(depth % 2)] < ((*node)->point[(depth % 2)])) {
				node = &((*node)->left);
			}
			else {
				node = &((*node)->right);
			}

			// call this function recursively until a NULL is hit
			insertHelper(node, depth+1, point, id);
		}
		else {
			// create a node and insert the point
			*node = new Node(point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// This function inserts a new point into the tree
		// the function creates a new node and places correctly with in the root 
		insertHelper(&this->root, 0, point, id);		
	}

	// this function returns euclidian distance between two points
	float dist(std::vector<float> point_a, std::vector<float> point_b)
	{
		// Return the euclidian distance between points
		return sqrt(pow((point_a[0] - point_b[0]), 2) + pow((point_a[1] - point_b[1]), 2));
	}

	void searchHelper(Node * node, std::vector<float> target, float distanceTol, int depth, std::vector<int>& ids)
	{
		if(node != NULL) {
			// add this node id to the list if its distance from target is less than distanceTol
			if(dist(node->point, target) <= distanceTol) 
				ids.push_back(node->id);	

			// x split when (depth % 2) = 0; y split when (depth % 2) = 1
			// index for accessing point.x is 0; index for accessing point.y is 1
			if((target[depth % 2] - distanceTol) < node->point[(depth % 2)])
				searchHelper(node->left, target, distanceTol, depth+1, ids);
			if((target[depth % 2] + distanceTol) > node->point[(depth % 2)])
				searchHelper(node->right, target, distanceTol, depth+1, ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(this->root, target, distanceTol, 0, ids);
		return ids;
	}
};




