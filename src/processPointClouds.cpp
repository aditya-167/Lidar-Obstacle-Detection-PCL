// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << "Total Number of Points "<<cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> voxgrid;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    voxgrid.setInputCloud (cloud);
    voxgrid.setLeafSize (filterRes, filterRes, filterRes);
    voxgrid.filter (*cloud_filtered);


    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indx;

    pcl::CropBox<PointT> roof (true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-4,1));
    roof.setInputCloud(cloud_region);
    roof.filter(indx);

    pcl::PointIndices::Ptr in (new pcl::PointIndices);
    for (int p:indx){
        in->indices.push_back(p);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(in);
    extract.setNegative(true);
    extract.filter(*cloud_region);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr ObstCloud (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr PlaneCloud (new pcl::PointCloud<PointT>());
  
  for(int index:inliers->indices){
      PlaneCloud->points.push_back(cloud->points[index]);
  }
  
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  //filter out points that are not in plane.
  extract.filter (*ObstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(ObstCloud, PlaneCloud);
  return segResult;
}


/////////////////////// SEGMENT PLANE RANSACE FROM SCRATCH/////////////////////////////////////////////////////////////

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RANSAC_PlaneSegment_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float disttanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT p1;
	PointT p2;
	PointT p3;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c,d,dist,denominator;

	// For max iterations
	for(int it=0;it<maxIterations;it++)
	{
		std::unordered_set<int> inliers;
		/*Identify 3 points randomly*/
		while(inliers.size()<3)
			inliers.insert((rand() % cloud->points.size()));
		auto iter = inliers.begin();
		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;

		p1 = cloud->points[idx1];
		p2 = cloud->points[idx2];
		p3 = cloud->points[idx3];

		/*Fit a plane using the above 3 points*/
		a = (((p2.y-p1.y)*(p3.z-p1.z))-((p2.z-p1.z)*(p3.y-p1.y)));
		b = (((p2.z-p1.z)*(p3.x-p1.x))-((p2.x-p1.x)*(p3.z-p1.z)));
		c = (((p2.x-p1.x)*(p3.y-p1.y))-((p2.y-p1.y)*(p3.x-p1.x)));
		d = -(a*p1.x+b*p1.y+c*p1.z);
		denominator = sqrt(a*a+b*b+c*c);

		// Measure disttance between every point and fitted plane
		for(int pt_cnt=0;pt_cnt<cloud->points.size();pt_cnt++)
		{
			if(pt_cnt!=idx1||pt_cnt!=idx2||pt_cnt!=idx3)
			{
				dist = (fabs(a*cloud->points[pt_cnt].x+b*cloud->points[pt_cnt].y+c*cloud->points[pt_cnt].z+d)/denominator);
				// If disttance is smaller than threshold count it as inlier
				if(dist<=disttanceThreshold)
				{
					inliers.insert(pt_cnt);
				}
			}
		}

		/*Store the temporary buffer if the size if more than previously idenfitied points */
		if(inliers.size()>inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = inliers;

		}

	}

	// Segment the largest planar component from the remaining cloud
	if (inliersResult.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	/*Copy the points from inputcloud in to cloudInliers if the indices is in inliersResult vector
	 * or else copy the point to cloudOutliers*/
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	/*Create a pair using inlier and outlier points*/
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float disttanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (disttanceThreshold);

    //Segmenting Largest Planar component from input cloud
    seg.setInputCloud(cloud);
    //segment the cloud based on inliers
    seg.segment(*inliers,*coefficients);
    
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    if(inliers->indices.size()==0){
        std::cout<<"Could not generate planar model for the given data set!"<<std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloud_cluster->points.push_back(cloud->points[index]);
        
        cloud_cluster->width=cloud_cluster->points.size();
        cloud_cluster->height=1;
        cloud_cluster->is_dense=true;

        clusters.push_back(cloud_cluster);
        
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

////////////////////////////////////////////////////// EUCLIDEAN CLUSTERING FROM SCRATCH ////////////////////////////////////////////////////////////////////////////



/* a recurssive function.
   Proximity_Point function look points distanceTol
 * distance from the given point in the cloud and return the indices of the points
 *  
 * 1. If the target point is not processed then set is as processed and search
 * 2. the KDTree for all the points within the distanceTol
 * 3. Use each of the nearby points and search for other points that are within
 * 4. distanceTol distance from this points
 *
 * */
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity_Points(typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed_pts,int idx,typename KdTree_Scratch<PointT>::KdTree_Scratch* tree,float distanceTol, int maxSize)
{
	if((processed_pts[idx]==false)&&
			(cluster.size()<maxSize))
	{
		processed_pts[idx]=true;
		cluster.push_back(idx);
		std::vector<int> nearby = tree->search(cloud->points[idx],distanceTol);
		for(int index : nearby)
		{
			if(processed_pts[index]==false)
			{
				Proximity_Points(cloud, cluster,processed_pts,index,tree,distanceTol,maxSize);
			}
		}
	}

}


/* euclideanCluster function looks for clusters that have points within min and max limits
 * 1. Take one point at a time from the cluster , call Proximity function to identify the
 * 2. list of points that are within distanceTol limits
 * 3. Check if the no of points in cluster ,returned by proximity function, are in (minSize, maxSize)
 * 4. limits if not discard
 * */


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanCluster_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree_Scratch<PointT>::KdTree_Scratch* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	/*Create a flag for each point in the cloud, to identified if the point is processed or not, and set it to false*/
	std::vector<bool> processed_ptslag(cloud->points.size(),false);

	/*Loop through each point of the cloud*/
	for(int idx=0;idx<cloud->points.size();idx++)
	{
		/*Pass the point to Proximity function only if it was not processed
		 * (either added to a cluster or discarded)*/
		if(processed_ptslag[idx]==false)
		{
			std::vector<int> cluster;
			Proximity_Points(cloud, cluster,processed_ptslag,idx,tree,distanceTol,maxSize);

			/*Check if the number of points in the identified cluster are with in limits */
			if((cluster.size()>=minSize)&&cluster.size()<=maxSize)
				clusters.push_back(cluster);
		}

	}
	return clusters;

}


/* Clustering_euclideanCluster function shall identify the cluster of point that have given no of min, max points and meet the cluster tolerance requirement.
 * 1. Using points in the given cloud KDTree is formed.
 * 2. Using Euclidena Clustering, clusters are searched in the created KDTree
 * 3. Identified clusters are filtered, clusters that dont have points in min, max points are discarded.
 * */

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanCluster_scratch_clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create the KdTree object using the points in cloud.
    typename KdTree_Scratch<PointT>::KdTree_Scratch *tree =new KdTree_Scratch<PointT>;
    tree->insert(cloud);

    //perform euclidean clustering to group detected obstacles
	std::vector<std::vector<int>> cluster_indices = EuclideanCluster_Scratch(cloud, tree,clusterTolerance ,minSize,maxSize);

	for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	    for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
	      cloud_cluster->points.push_back (cloud->points[*pit]); //*
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster);
	  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclideanClustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}