/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

// Structure to represent node of kd 
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>** node, int depth, PointT point, int id)
	{
		if (*node == NULL)
			*node = new Node<PointT>(point, id);
		else
		{
			uint d = depth % 3;
			switch(d)
			{
				case 0: 
				if(point.x < (*node)->point.x){
					insertHelper(&((*node)->left), depth + 1, point, id);
				} else {
					insertHelper(&((*node)->right), depth + 1, point, id);
				} break;

				case 1: 
				if(point.y < (*node)->point.y){
					insertHelper(&((*node)->left), depth + 1, point, id);
				} else {
					insertHelper(&((*node)->right), depth + 1, point, id);
				} break;

				case 2: 
				if(point.z < (*node)->point.z){
					insertHelper(&((*node)->left), depth + 1, point, id);
				} else {
					insertHelper(&((*node)->right), depth + 1, point, id);
				} break;
			}	
		}
	}

	void insert(PointT point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

		// return a list of point ids in the tree that are within distance of target
	
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}

	void searchHelper(Node<PointT>* node, int depth, PointT target, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			if ((node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x +distanceTol))&&(node->point.y >= (target.y -distanceTol) && node->point.y <= (target.y +distanceTol))&& (node->point.z >= (target.z -distanceTol) && node->point.z <= (target.z + distanceTol)))
			{
				float dist = sqrt((target.x - (node)->point.x)*(target.x - (node)->point.x) + (target.y - (node)->point.y)*(target.y - (node)->point.y)+ (target.z - (node)->point.z)*(target.z - (node)->point.z));
				if (dist <= distanceTol)
					ids.push_back((node)->id);
			}
			uint d = depth % 3;

			switch(d)
			{
				case 0: 
				if(node->point.x > (target.x-distanceTol))
					searchHelper(node->left, depth + 1, target, distanceTol, ids);
				if(node->point.x < (target.x+distanceTol))
					searchHelper(node->right, depth + 1, target, distanceTol, ids);
				break;

				case 1: 
				if(node->point.y > (target.y-distanceTol))
					searchHelper(node->left, depth + 1, target, distanceTol, ids);
				if(node->point.y < (target.y+distanceTol))
					searchHelper(node->right, depth + 1, target, distanceTol, ids);
				break;

				case 2: 
				if(node->point.z > (target.z-distanceTol))
					searchHelper(node->left, depth + 1, target, distanceTol, ids);
				if(node->point.z < (target.z+distanceTol))
					searchHelper(node->right, depth + 1, target, distanceTol, ids);
				break;
			}			
		}
	}
};
/*
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

template<typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else
		{
			uint d = depth % 2;
			if (point[d] < (*node)->point[d])
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}

	void searchHelper(Node* node, int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			uint d = depth % 2;
			if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0]+distanceTol))&&(node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)))
			{
				float dist = sqrt((target[0] - (node)->point[0])*(target[0] - (node)->point[0]) + (target[1] - (node)->point[1])*(target[1] - (node)->point[1]));
				if (dist <= distanceTol)
					ids.push_back((node)->id);
			}
			if (node->point[d] > (target[d]-distanceTol))
				searchHelper(node->left, depth + 1, target, distanceTol, ids);
			if (node->point[d] < (target[d]+distanceTol))
				searchHelper(node->right, depth + 1, target, distanceTol, ids);
		}
	}

};
*/


