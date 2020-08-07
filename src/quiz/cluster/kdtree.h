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

	void NodeInsert(Node **node,std::vector<float> point,uint depth,int id){
		if(*node==NULL){
			*node=new Node(point,id);
		}
		else{
			uint cd=depth%2;
			if(point[cd] < ((*node)->point[cd]))
				NodeInsert(&((*node)->left),point,depth+1,id);
			else
				NodeInsert(&((*node)->right),point,depth+1,id);
		}	
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		NodeInsert(&root,point,0,id);


	}


	void search_nearby(Node* node,std::vector<float>target,float distanceTol,std::vector<int>& ids,int depth){
		if(node!=NULL){
			bool a= ((node->point[0]>=(target[0]-distanceTol)) && (node->point[0]<=(target[0]+distanceTol)));
			bool b= ((node->point[1]>=(target[1]-distanceTol)) && (node->point[1]<=(target[1]+distanceTol)));
			if(a && b)
			{
				float distance=sqrt(((node->point[0]-target[0])*(node->point[0]-target[0]))+((node->point[1]-target[1])*(node->point[1]-target[1])));
				if(distance<=distanceTol){
					ids.push_back(node->id);
				}
			}

			if((target[depth%2]-distanceTol) < node->point[depth%2])
				search_nearby(node->left,target,distanceTol,ids,depth+1);
			if((target[depth%2]+distanceTol) > node->point[depth%2])
				search_nearby(node->right,target,distanceTol,ids,depth+1);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_nearby(root,target,distanceTol,ids,0);
		return ids;
	}
	

};




