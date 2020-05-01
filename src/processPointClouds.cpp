// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <cmath>



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>:: Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMax(maxPoint);
    region.setMin(minPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);

    std::vector<int> Idx;
    pcl::CropBox<PointT> carRoof(true);
    carRoof.setMax(Eigen::Vector4f (2, 2, 1, 1));
    carRoof.setMin(Eigen::Vector4f (-2, -2, -1, 1));
    carRoof.setInputCloud(cloudRegion);
    carRoof.filter(Idx);

    pcl::PointIndices:: Ptr inliers {new pcl::PointIndices};
    for(int point: Idx)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);  
    extract.setNegative(true);
    extract.filter (*cloudRegion);  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}

/*
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // TODO:: Fill in this function to find inliers for the cloud.
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0) 
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
*/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
    //pcl::PointIndices::Ptr inliersResult {new pcl::PointIndices};
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	
	while(maxIterations--)
	{ 
		std::unordered_set<int> inPlane;
        
		while (inPlane.size() < 3) 
			inPlane.insert(rand()%(cloud->points.size()));

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inPlane.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
				
		float a = (y2-y1)*(z3-z1)-(y3-y1)*(z2-z1);
		float b = (x2-x1)*(z3-z1)-(x3-x1)*(z2-z1);
		float c = (x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
		float d = -(a*x1+b*y1+c*z1);

		for(int index = 0; index < cloud->points.size(); index++)
		{
			if(inPlane.count(index)>0) 
				continue;

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			//The L1 norm compute is less intensive and is used to calculate the distance
            float dist = fabs(a*x4+b*y4+c*z4+d)/(fabs(a)+fabs(b)+fabs(c));

			if (dist <= distanceThreshold) 
				inPlane.insert(index);

		}

		if(inPlane.size() > inliersResult.size())
		{
			inliersResult = inPlane;
		}

	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC_Plane took  " << elapsedTime.count() << "  milliseconds" << std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (const auto& pointsIndex: inliersResult) 
    {
        inliers->indices.push_back(pointsIndex);
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
    // Creating the KdTree object for the search method of the extraction
    //typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    //tree->setInputCloud (cloud);

    KdTree *tree = new KdTree;

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<float> point;
    for (int i=0; i< cloud->points.size(); i++)
    {
        point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point,i);
    }

	std::vector<std::vector<int>> clusterIndices;
    std::vector<bool> isProcessed(cloud->points.size(),false);

	int count = 0;
	while (count < cloud->points.size())
	{
		if (isProcessed[count])
		{
			count++;
			continue;
		}
		std::vector<int> cluster;
		findClusters(count, cloud, cluster, isProcessed, tree, clusterTolerance);
		
        if ((cluster.size() >= minSize) && (cluster.size() <= maxSize))
		{   
            clusterIndices.push_back(cluster);
        }    
			
    }
    
    for (int i = 0; i< clusterIndices.size(); ++i)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>);
        for (int it = 0; it < clusterIndices[i].size(); ++it)

            clusterCloud->points.push_back(cloud->points[clusterIndices[i][it]]); 
        clusterCloud->width = clusterCloud->points.size ();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        clusters.push_back(clusterCloud);
        
    }
    
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
	
    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::findClusters(int Idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& isProcessed, KdTree* tree, float clusterTolerance)
{
	isProcessed[Idx] = true;
	cluster.push_back(Idx);

	std::vector<float> target = {cloud->points[Idx].x, cloud->points[Idx].y, cloud->points[Idx].z};
    std::vector<int> inCluster = tree->search(target, clusterTolerance);
        
	for (int pointIdx : inCluster)
	{
		if (!isProcessed[pointIdx])
			findClusters(pointIdx, cloud, cluster, isProcessed, tree, clusterTolerance);
	}

}

    /*std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm (0.02)
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices); 

    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)

            clusterCloud->points.push_back (cloud->points[*pit]); //*
        clusterCloud->width = clusterCloud->points.size ();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        clusters.push_back(clusterCloud);
        
    }    
      
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}*/


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