#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <thread>
#include "Prototype.hpp"

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

/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    auto lidar = std::make_unique<Lidar>(Lidar(cars,0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "Input Cloud");

    // PCL implementation
    auto processor = std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();
    auto segmentCloud = processor->SegmentPlane(cloud,60,0.2);

    // renderPointCloud(viewer, segmentCloud.first,"obstacle", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second,"road", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = 
    processor->Clustering(
                segmentCloud.first, // obstacles
                            1,      // cluster tolerance
                            3,      // min size
                            30 );   // max size
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
        size_t clusterId = 0;
        
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        std::cout<<"cluster size ";
        processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // Bounding box
        Box box = processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}
*/

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

/* visualize PCD data */
void cityBlock(
                pcl::visualization::PCLVisualizer::Ptr& viewer,
                ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud
                )
    {

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    /* Downsampling and Filtering data */
    pcl::PointCloud<pcl::PointXYZI>::Ptr 
    filterCloud = pointProcessorI->FilterCloud(inputCloud,
                                                0.25,                                        // grid size
                                                Eigen::Vector4f(-20.01f, -6.0f, -3.0f,1),    // box lower bound
                                                Eigen::Vector4f(35.0f, 6.0f, 3.0f,1)         // box upper bound
    );
    auto processor = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();

    /* Ransac Algorithm to perform plane segmentation */
    auto inliers_idx = RansacPlane(filterCloud, // input Cloud
                                    36,         // # of iterations
                                    0.1);       // tolerance

    /* Initialize PointCloud container for Inliers and Outliers */
    pcl::PointCloud<pcl::PointXYZI>::Ptr Inliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr Outliers(new pcl::PointCloud<pcl::PointXYZI>());

    /* Separate Inliers and Outliers */
    for(size_t i=0; i<filterCloud->size(); i++){
        pcl::PointXYZI point = filterCloud->points[i];
        if(inliers_idx.count(i)){
            Inliers->push_back(point);
        }
        else{
            Outliers->push_back(point);
        }
    }
    /* Obstacles, Road */
    auto segmentCloud = std::make_pair(Outliers, Inliers);

    /* KD-Tree Clustering */
    KdTree* tree = new KdTree;
    for (int i=0; i<Outliers->size(); i++){
        tree->insert(Outliers->points[i],i); 
    } 
    auto cloudClusters = euclideanCluster(
                                            Outliers, // input pointCloud pointer
                                            tree, 
                                            0.3,     // distance tolerance
                                            30,      // min cluster size
                                            400      // max cluster size
    );

    //auto segmentCloud = processor->SegmentPlane(filterCloud,50,0.2);
    /* PCL Clustering Implementation */
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = 
    // processor->Clustering(
    //             segmentCloud.first, // obstacles
    //                         0.5,      // cluster tolerance
    //                         20,      // min size
    //                         500 );   // max size

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    size_t clusterId = 0;
    for( auto cluster : cloudClusters ){
        // std::cout<<"cluster size ";
        // processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
        /* Bounding box*/
        Box box = processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
    //renderPointCloud(viewer, segmentCloud.first, "Obstacles", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "Road", Color(0,1,0) );
    delete tree;
}

int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    /* Stream PCD */
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI (new ProcessPointClouds<pcl::PointXYZI>);
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        /* clear viewer */
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        /* load pcd and run obstacle detection */
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        ++streamIterator;
        /* loop */
        if(streamIterator == stream.end()){
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    } 
}