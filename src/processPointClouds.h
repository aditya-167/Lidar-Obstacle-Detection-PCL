// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
// KD-Tree From Scratch 3-D
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

template <typename PointT>
struct KdTree_Scratch
{
	Node* root;


	KdTree_Scratch()
	: root(NULL)
	{}
	



    /*NodeInsert called recursively to create a KDTree

	 *1.Check if new data point is greater than or less than root.
	 *2.Insert if the child is null as left child if less than root else right child.
	 *3.For above steps at level 0:x coordinates, level 1: y coordinates , leve 2: z coordinates are compared
	 * */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void NodeInsert(Node *&node, uint depth, PointT point, int id)
	{
		/*Identify the axis*/
	    uint index = depth%3;
	    /*If the node is NULL insert the point along with index by creating a new node*/
		if(node == NULL)
		{
		// convert point.data arr to vector
		 std::vector<float> v_point(point.data, point.data+3);
		 node = new Node(v_point,id);
		}
		else if(point.data[index] < node->point[index])
		{
		/*data point is less than root insert in left child*/
		NodeInsert(node->left,depth+1,point,id);
		}
		else
		{
		/*data point is greater than root insert in right child*/
		NodeInsert(node->right,depth+1,point,id);
		}
	}
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    ///////////////////////////////////////////////////////////////////////////////
    /* insert function helps creating KDTree from a cloud.
	 * 1.This function shall loop through each of the cloud points
	 * 2.and call NodeInsert function for each point
	 * */

	void insert(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		for(uint index = 0; index < cloud->points.size(); index++)
		{
		   NodeInsert(root,0,cloud->points[index],index);
		}

	}
    //////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*search_nearby function looks for the target point in the KDTree
	 * 1. Check if the target point x,y,z are within node+/-distanceTol ,
	 * 2. if they are then check if the distance b/w node and target is with in distanceTol then add it to the list
	 * 3. If x,y,z of target are not with in distanceTol of node then check if target-/+distanceTol is less or greater node and
	 * 4. call helpersearch with left or right child node.
	 *
	 * */
	void search_nearby(Node *&node,uint depth,std::vector<int> *ids,PointT target, float distanceTol)
	{
		uint id = depth%3;
		if(node!=NULL)
		{
			/*Check if nodes x,y,z are with in target+/-distanceTol */
			if(((node->point[0]<target.data[0]+distanceTol)&&(node->point[0]>target.data[0]-distanceTol))&&
					((node->point[1]<target.data[1]+distanceTol)&&(node->point[1]>target.data[1]-distanceTol))&&
						((node->point[2]<target.data[2]+distanceTol)&&(node->point[2]>target.data[2]-distanceTol)))
			{
				/*calculate distance b/w node and point*/
				uint dis=sqrt((node->point[0]-target.data[0])*(node->point[0]-target.data[0])+
						(node->point[1]-target.data[1])*(node->point[1]-target.data[1])+
						(node->point[2]-target.data[2])*(node->point[2]-target.data[2]));

				/*is distance b/w node and point less than distanceTol then add it to vector*/
				if(dis<distanceTol)
				{
					ids->push_back(node->id);
				}
			}

			if(target.data[id]-distanceTol<node->point[id])
			{
				search_nearby(node->left,depth+1,ids,target,distanceTol);

			}
			if(target.data[id]+distanceTol>node->point[id])
			{
				search_nearby(node->right,depth+1,ids,target,distanceTol);

			}

		}
	}

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
    
    
    ///////////////////////////////////////////////////////////////////////////////////
    /*call search_nearby function.
	 * */
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		uint depth =0;
		uint maxdistance=0;

		search_nearby(root,depth,&ids,target,distanceTol);
        //cout<<"helpersearch end"<<endl;
		return ids;
	}

    ///////////////////////////////////////////////////////////////////////////////////
};



template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);


    // Clustering frome Scratch functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // RANSACE PLANE SEGMENTATION
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RANSAC_PlaneSegment_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
  
  // EUCLIDEAN CLUSTERING 
    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanCluster_scratch_clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
  
  // Helper function for proximity points for euclidean clustering

    void Proximity_Points(typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed_pts,int idx,typename KdTree_Scratch<PointT>::KdTree_Scratch* tree,float distanceTol, int maxSize);


	std::vector<std::vector<int>> EuclideanCluster_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree_Scratch<PointT>::KdTree_Scratch* tree, float distanceTol, int minSize, int maxSize);


};
#endif /* PROCESSPOINTCLOUDS_H_ */