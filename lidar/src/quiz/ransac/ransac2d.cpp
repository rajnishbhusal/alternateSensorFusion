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
  	for(int i = -5; i < 5; i++)
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
  	int numOutliers = 10;
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
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	int maxInliers = 0;
	int maxInlier_ind1 = -1;
	int maxInlier_ind2 = -1;
	
	// For max iteration
	for (int i=0; i<maxIterations; i++){

		// Randomly sample subset and fit line
		int ind1 = rand() % cloud->width;
		int ind2 = rand() % cloud->width;
		double x1 = cloud->points[ind1].x;
		double y1 = cloud->points[ind1].y;
		double x2 = cloud->points[ind2].x;
		double y2 = cloud->points[ind2].y;

		double A = y1-y2;
		double B = x2-x1;
		double C = x1*y2-x2*y1;

		int countInliers = 0;

		// Measure distance between every point and fitted line
		for (int j = 0; j < cloud->width; j++){
			double x0 = cloud->points[j].x;
			double y0 = cloud->points[j].y;
			double d;
			d = (abs(A*x0+B*y0+C))/(sqrt(A*A+B*B)); 
			// If distance is smaller than threshold count it as inlier
			if (d<distanceTol){
				countInliers += 1;

			}
		}
		if (countInliers>maxInliers){
			maxInliers = countInliers;
			maxInlier_ind1 = ind1;
			maxInlier_ind2 = ind2;
		}
	}

	double x1 = cloud->points[maxInlier_ind1].x;
	double y1 = cloud->points[maxInlier_ind1].y;
	double x2 = cloud->points[maxInlier_ind2].x;
	double y2 = cloud->points[maxInlier_ind2].y;

	double A = y1-y2;
	double B = x2-x1;
	double C = x1*y2-x2*y1;

	// Measure distance between every point and fitted line
	for (int k = 0; k < cloud->width; k++){
		double x0 = cloud->points[k].x;
		double y0 = cloud->points[k].y;
		double d = (abs(A*x0+B*y0+C))/(sqrt(A*A+B*B)); 
		// If distance is smaller than threshold count it as inlier
		if (d<distanceTol){
			inliersResult.insert (k);
		}
	}

	
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
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
