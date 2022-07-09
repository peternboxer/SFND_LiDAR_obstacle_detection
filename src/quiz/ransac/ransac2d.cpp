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

std::unordered_set<int> 
Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::srand(time(NULL));
	
	// TODO: Fill in this function
	size_t count = cloud->points.size();
	size_t i,j;
	float A,B,C,d;
	float x1, y1, x2, y2, x3, y3;

	for( i=0;i<maxIterations;i++){

		std::unordered_set<int> box;
		while(box.size()<2){
			box.insert(std::rand()%count);
		}
		auto iter = box.begin();
		auto a = cloud->points[*iter];
		iter++;
		auto b = cloud->points[*iter];

		x1 = a.x, y1 = a.y;
		x2 = b.x, y2 = b.y;

		A = y1 - y2;
		B = x2 - x1;
		C = x1*y2 - x2*y1;

		for(j = 0; j < cloud->points.size();j++){
			x3 = cloud->points[j].x;
			y3 = cloud->points[j].y;
			d = std::fabs(A*x3 + B*y3 + C)/std::sqrt(std::pow(A,2)+std::pow(B,2));
			if(d < distanceTol){
				box.insert(j);
			}
		}
		if(box.size() > inliersResult.size()){
			inliersResult = box;
		}
	}
	return inliersResult;
}

std::unordered_set<int> 
RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){

	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	std::srand(time(NULL));
	
	size_t count = cloud->points.size();
	size_t i,j;
	float A,B,C,D, distance;
	float x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4;
	float v1[3], v2[3], v3[3];

	while(maxIterations--){
		std::unordered_set<int> box;
		while(box.size()<3){
			box.insert(std::rand()%count);
		}
		auto iter = box.begin();
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
		iter++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;
		iter++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		v1[0] = x2 - x1;
		v1[1] = y2 - y1;
		v1[2] = z2 - z1;
		v2[0] = x3 - x1;
		v2[1] = y3 - y1;
		v2[2] = z3 - z1;

		v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
		v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
		v3[2] = v1[0]*v2[1] - v1[1]*v2[0];

		A = v3[0];
		B = v3[1];
		C = v3[2];
		D = -1*(A*x1 + B*y1 + C*z1);

		for(i = 0; i < cloud->points.size();i++){
			x4 = cloud->points[i].x;
			y4 = cloud->points[i].y;
			z4 = cloud->points[i].z;

			distance = std::fabs(A*x4 +B*y4+C*z4+D) /std::sqrt(std::pow(A,2)+std::pow(B,2)+std::pow(C,2));
			//std::cout<<"distance = "<<distance<<"\n";

			if(distance < distanceTol){
				box.insert(i);
			}
		}
		if(box.size() > inliersResult.size()){
			inliersResult = box;
		}
	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Plane Segmentation took " << elapsedTime.count() <<" milisecs.\n"<<std::endl;
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	//CreateData();
	
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 12, 1);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

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
