/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "myOwnProcessPointClouds.h"
#include "myOwnProcessPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true; //true to view point cloud with shapes of obstacles // false to not see blocks of obstacles
    bool render_rays = false;
    bool render_pointCloud = false;
    bool render_obst = false;
    bool render_plane = false;
    bool render_clusters = false;
    bool render_box = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0); // create the lidar object in heap using 'new' keyword. Also, the class Lidar is instantiated (with constructor)
    pcl::PointCloud<pcl::PointXYZ>:: Ptr inputCloud = lidar->scan();

    if (render_rays){
        renderRays(viewer, lidar->position, inputCloud); // use renderRays only if you want to see the paths of lidar rays. 
    }

    if (render_pointCloud){
        renderPointCloud(viewer, inputCloud, "inputCloud"); // use renderPointCloud if you want to see the point clouds only.
    }

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // creating pointProcessor object using ProcessPointClouds class
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    //myOwnProcessPointClouds<pcl::PointXYZ> pointProcessor; // creating pointProcessor object using ProcessPointClouds class
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.myOwnRansac3d(inputCloud, 100, 0.2);
    if (render_obst){
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    }

    if (render_plane){
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    }


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2, 3, 35);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
      if (render_clusters){
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      }

      if (render_box){
          Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
      }
      
      ++clusterId;
    }
    
  
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, myOwnProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud){
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud){
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    bool render_pointCloud = false;
    bool render_obst = true;
    bool render_plane = true;
    bool render_clusters = true;
    bool render_box = true; // true


    // Comment the following two lines, since we are passing the pointProcessor and input cloud
    /*
    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    // Comment: Load one single data as inputCloud. 
    // Remember you are working on a pointCLoud and not on stream of pointClouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    */

    // steps for filtering using filtercloud in processPointClouds.cpp
    // filter resolution = 0.3 m (how close points do you want)
    // Eigen::Vector4f(-10, -5, -3, 1)----> min crop box relative to the origin (backwards, right, up)
    // Eigen::Vector4f(40, 6, 10, 1) ---> max crop box relative to the origin (frontwards, left, down)
    // FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1)) -> udacity
    // 

    inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -3, 1), Eigen::Vector4f(30, 6, 1, 1));

    if (render_pointCloud){
        renderPointCloud(viewer, inputCloud, "inputCloud"); // use renderPointCloud if you want to see the point clouds only.
    }

    // Segmentation
    // SegmentPlane(cloud, int maxIterations, float distanceThreshold)
    // SegmentPlane(inputCloud, 100, 0.2) -> mine
    // SegmentPlane(inputCloud, 25, 0.3) -> udacity
    
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.myOwnRansac3d(inputCloud, 25, 0.3);
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(inputCloud, 25, 0.3);

    //myOwnProcessPointClouds<pcl::PointXYZ> pointProcessor; // creating pointProcessor object using ProcessPointClouds class
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.myOwnRansac3d(inputCloud, 25, 0.3);
    if (render_obst){
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    }

    if (render_plane){
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    }

    // Clustering
    // segmentCloud.first = obstacle segment
    // minSize = minimum number of points for forming a cluster
    // maxSize = maximum number of points for forming a cluster
    // Clustering(cloud, float clusterTolerance, int minSize, int maxSize)
    // Clustering(segmentCloud.first, 0.5, 3, 650) -> mine
    // Clustering(segmentCloud.first, 0.53, 10, 500) -> udacity
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.53, 10, 650);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){
      if (render_clusters){
        std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
      }

      if (render_box){
          Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
      }
      
      ++clusterId;
    }
    

}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ///*
    // For a single pointCloud data, either call simpleHighway or cityBlock
    simpleHighway(viewer);
    //cityBlock(viewer);
    //*/

    ///*
    // For streaming point clouds
    // Call City Block after defining a processor in stack and input cloud in main 
    // (use '.' to access functions for the processor since it is in stack)
    // (not on heap with new keyword which uses '->')

    // Create point cloud processor with (X,Y,Z,I)
    /*
    myOwnProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    // ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    // Now, we are working on stream of point clouds and not on a single static point cloud
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    // Declare a null pointer to the inputCloudI
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //*/
    
    while (!viewer->wasStopped ())
    {   
        /*
        // Use the following for stream of point clouds

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load PCD and run obstacle detection process using cityBlock
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end()){
            streamIterator = stream.begin();
        }
        //*/

        // Use only the following line for a single point cloud data and comment the above
        viewer->spinOnce ();
    } 
}