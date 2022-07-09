#include "Prototype.hpp"

std::unordered_set<int> 
RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol){

	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	pcl::PointIndices::Ptr Inliers;

	std::srand(time(NULL));
	
	size_t count = cloud->points.size();
	size_t i,j;
	float A,B,C,D, distance;
	float x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4;
	float v1[3], v2[3], v3[3];

	std::unordered_set<int> box;
	while(maxIterations--){
		box.clear();
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

			distance = 
				std::fabs(A*x4 +B*y4+C*z4+D) /
				std::sqrt(std::pow(A,2)+std::pow(B,2)+std::pow(C,2));

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
    std::cout << "Plane Segmentation took " << elapsedTime.count() <<" milisecs.\n";
	return inliersResult;
}

void Proximity(
				const pcl::PointCloud<pcl::PointXYZI>::Ptr& points, 
				pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, 
				const size_t& idx, 
				std::vector<bool>& processed, 
				KdTree* tree, 
				const float& distanceTol
				)
{
	processed[idx] = true;
	cluster->emplace_back(points->points[idx]);
	auto nearby = tree->search(points->points[idx], distanceTol);
	for(int x : nearby){
		if(!processed[x]){
			Proximity(points, cluster, x, processed, tree, distanceTol);
		}
	}
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
euclideanCluster(
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& points, 
                    KdTree* tree, 
                    const float& distanceTol,
					uint32_t minSize,
					uint32_t maxSize
                    )
{
	auto startTime = std::chrono::steady_clock::now();

	/* initialize list of unprocessed */
	std::vector<bool> processed(points->points.size(), false);

	//std::vector<std::vector<int>> clusters;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

	for(size_t i = 0; i<points->points.size();i++)
	{
		if(processed[i] == false)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZI>);
			Proximity(points, cluster, i, processed, tree, distanceTol);

			if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize)
			{
				clusters.emplace_back(cluster);
			}
		}
	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() <<" milisecs.\n\n";
	return clusters;
}
