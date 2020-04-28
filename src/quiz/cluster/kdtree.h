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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		//Tree is empty
		if(*node==NULL)
			*node = new Node(point, id);
		
		// Places a new point correctly within the tree 
		// to the left or right of current node's memory address location
		else
		{
			//Calculate current dim
			uint cd = depth % 3;

			if(point[cd] < ((*node) -> point[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			
			else
				insertHelper(&((*node)->right), depth+1, point, id);
	
		}
	}
	
	void insert(std::vector<float> point, int id)
	{
		// Inserts a new point in the tree
		insertHelper(&root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>&ids)
	{
		if(node != NULL)
		{
			if ( (abs(node->point[0] - target[0]) <= distanceTol) && (abs(node->point[1] - target[1]) <= distanceTol) && (abs(node->point[2] - target[2]) <= distanceTol))
			{
				ids.push_back(node->id);
				//std::cout << distance << std::endl;
				//float distance = abs(node->point[0] - target[0]) + abs(node->point[1] - target[1]);
				//if (distance < distanceTol)
				//{
				//	ids.push_back(node->id);
				//	std::cout << distance << std::endl;
				//}
				//ids.push_back(node->id);
				//std::cout << distance << std::endl;
			
			}
			//check across boundary
			if ((target[depth%3] - distanceTol) < node->point[depth%3])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			
			if ((target[depth%3] + distanceTol) > node->point[depth%3])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		
		}
	}
	

};




