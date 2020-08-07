/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -10; i < 10; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 20;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (1, 1, 1);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	for(int i = 0; i < maxIterations; i++) {
		
		std::unordered_set<int> inliers;
		
		while (inliers.size() < 3) {
			//inliers.insert(static_cast<double>(std::rand()) / RAND_MAX) * cloud->points.size());
			inliers.insert(rand()%(cloud->points.size()));
		}

		auto itr = inliers.begin();
		
		pcl::PointXYZ p1 = cloud->points[*itr];
		float x1 = p1.x;
		float y1 = p1.y;
		float z1 = p1.z;
		++itr;

		pcl::PointXYZ p2 = cloud->points[*itr];
		float x2 = p2.x;
		float y2 = p2.y;
		float z2 = p2.z;
		++itr;
		
		pcl::PointXYZ p3 = cloud->points[*itr];
		float x3 = p3.x;
		float y3 = p3.y;
		float z3 = p3.z;

		std::tuple<float, float, float> v1(x2 - x1, y2 - y1, z2 - z1);
		std::tuple<float, float, float> v2(x3 - x1, y3 - y1, z3 - z1);

		std::tuple<float, float, float> perpendicular((y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1),(z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1),(x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));

		float A = std::get<0>(perpendicular);
		float B = std::get<1>(perpendicular);
		float C = std::get<2>(perpendicular);
		float D = -(A * x1 + B * y1 + C * z1);

		for(int i = 0; i < cloud->points.size(); i++)  {
			pcl::PointXYZ p0 = cloud->points[i];

			if (inliers.count(i) > 0)
				continue;

			float d = std::fabs(A * p0.x + B * p0.y + C * p0.z + D) / std::sqrt(A * A + B * B + C * C);
			// std::cout << "Point " << "(" << p0.x << "," << p0.y << "), " << "distance to line: " << d << "\n";
			
			if (d <= distanceTol) {
				inliers.insert(i);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
		std::cout << "END: Iteration " << i << ", with " << inliers.size() << " inlier points.\n\n";
	}

	return inliersResult;

}




std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while(maxIterations--){

		std::unordered_set<int> inliers;
		
		// Randomly sample subset and fit line
		
		while(inliers.size()<2)
			inliers.insert(rand()%(cloud->points.size()));
		float x1,x2,y1,y2;
		auto itr=inliers.begin();
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		itr++;
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		float a = y1-y2;
		float b= x2-x1;
		float c=(x1*y2-x2*y1);

		for(int index=0;index<cloud->points.size();index++){
			if(inliers.count(index)>0)
				continue;
			pcl::PointXYZ point =cloud->points[index];
			float x3=point.x;
			float y3=point.y;
			// Measure distance between every point and fitted line
			float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);
			// If distance is smaller than threshold count it as inlier
			if(d<=distanceTol)
				inliers.insert(index);
		}
	// Return indicies of inliers from fitted line with most inlier
		if (inliers.size()>inliersResult.size())
		inliersResult=inliers;
	}
	
	return inliersResult;

}







int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100,0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(0,0,1));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
