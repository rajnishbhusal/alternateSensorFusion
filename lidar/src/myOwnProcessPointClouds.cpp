// PCL lib Functions for processing point clouds 

#include <unordered_set>
#include "myOwnProcessPointClouds.h"


//constructor:
template<typename PointT>
myOwnProcessPointClouds<PointT>::myOwnProcessPointClouds() {}


//de-constructor:
template<typename PointT>
myOwnProcessPointClouds<PointT>::~myOwnProcessPointClouds() {}


template<typename PointT>
void myOwnProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr myOwnProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    region.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    region.setInputCloud(cloudRegion);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point:indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    // end To DO

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion; // change cloud to cloudRegion

}


// Creating My Own PointCloud Separation

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> myOwnProcessPointClouds<PointT>::myOwnSeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloudInliers (new pcl::PointCloud<PointT> ());

    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    cloudInliers->width = cloudInliers->points.size();
    cloudOutliers->width = cloudOutliers->points.size();

    

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}

// Creating My Own PointCLoud Segmentation Algorithm Using Ransac algoirthm
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> myOwnProcessPointClouds<PointT>::myOwnRansac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	int maxInliers = 0;
	int maxInlier_ind1 = -1;
	int maxInlier_ind2 = -1;
    int maxInlier_ind3 = -1;
	
	// For max iteration
	for (int i=0; i<maxIterations; i++){

		// Randomly sample subset and fit line
		int ind1 = rand() % cloud->width;
		int ind2 = rand() % cloud->width;
        int ind3 = rand() % cloud->width;
		double x1 = cloud->points[ind1].x;
		double y1 = cloud->points[ind1].y;
        double z1 = cloud->points[ind1].z;
		double x2 = cloud->points[ind2].x;
		double y2 = cloud->points[ind2].y;
        double z2 = cloud->points[ind2].z;
        double x3 = cloud->points[ind3].x;
		double y3 = cloud->points[ind3].y;
        double z3 = cloud->points[ind3].z;

		double A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		double B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		double C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
        double D = -(A*x1+B*y1+C*z1);

		int countInliers = 0;

		// Measure distance between every point and fitted line
		for (int j = 0; j < cloud->width; j++){
			double x0 = cloud->points[j].x;
			double y0 = cloud->points[j].y;
            double z0 = cloud->points[j].z;
			double d;
			d = (abs(A*x0+B*y0+C*z0+D))/(sqrt(A*A+B*B+C*C)); 
			// If distance is smaller than threshold count it as inlier
			if (d<distanceThreshold){
				countInliers += 1;

			}
		}
		if (countInliers>maxInliers){
			maxInliers = countInliers;
			maxInlier_ind1 = ind1;
			maxInlier_ind2 = ind2;
            maxInlier_ind3 = ind3;
		}
	}

	double x1 = cloud->points[maxInlier_ind1].x;
	double y1 = cloud->points[maxInlier_ind1].y;
    double z1 = cloud->points[maxInlier_ind1].z;
	double x2 = cloud->points[maxInlier_ind2].x;
	double y2 = cloud->points[maxInlier_ind2].y;
    double z2 = cloud->points[maxInlier_ind2].z;
    double x3 = cloud->points[maxInlier_ind3].x;
	double y3 = cloud->points[maxInlier_ind3].y;
    double z3 = cloud->points[maxInlier_ind3].z;

	double A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
	double B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
	double C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
    double D = -(A*x1+B*y1+C*z1);


	// Measure distance between every point and fitted line
	for (int k = 0; k < cloud->width; k++){
		double x0 = cloud->points[k].x;
		double y0 = cloud->points[k].y;
        double z0 = cloud->points[k].z;
		double d = (abs(A*x0+B*y0+C*z0+D))/(sqrt(A*A+B*B+C*C)); 
		// If distance is smaller than threshold count it as inlier
		if (d<distanceThreshold){
			inliersResult.insert (k);
		}
	}

	
	// Return indicies of inliers from fitted line with most inliers
	
	// return inliersResult;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = myOwnSeparateClouds(inliersResult,cloud);
    
    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

///*
template<typename PointT>
void myOwnProcessPointClouds<PointT>::clusterHelper(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int i, typename pcl::PointCloud<PointT>::Ptr& cluster, std::vector<int>& processed){
	processed[i] = 1;
    PointT point;
    point.x = points[i][0];
    point.y = points[i][1];
    point.z = points[i][2];

	cluster->points.push_back(point);

	std::vector<int> ids;
	ids = tree->search(points[i], distanceTol);
	for (int id:ids){
		if (processed[id]==0){
			clusterHelper(points, tree, distanceTol, id, cluster, processed);
		}
	}
}
//*/

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> myOwnProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    //cout<<minSize;
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the 
    
    ///*
    
    KdTree* tree = new KdTree;

    // Create data
	std::vector<std::vector<float>> points;
    std::vector<float> pt;

    
    for (int j = 0; j < cloud->width; j++){
		float x0 = cloud->points[j].x;
		float y0 = cloud->points[j].y;
        float z0 = cloud->points[j].z;
        pt = {x0, y0, z0};
        points.push_back(pt);
    }

    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

	std::vector<int> processed(points.size(), 0);
	for (int i = 0; i<points.size(); i++){
		if (processed[i]==0){
			typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
			clusterHelper(points, tree, clusterTolerance, i, cluster, processed);
            cluster->width = cluster->points.size();
            //cluster->width = 1;
            cluster->is_dense = true;
			clusters.push_back(cluster);
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;


    return clusters;
}


template<typename PointT>
Box myOwnProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void myOwnProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr myOwnProcessPointClouds<PointT>::loadPcd(std::string file)
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
std::vector<boost::filesystem::path> myOwnProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}