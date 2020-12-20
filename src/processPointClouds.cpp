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
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setInputCloud(cloud_filtered);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.filter(*cloud_region);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setInputCloud(cloud_region);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.24, 1));
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    if(inliers->indices.size()==0){
        std::cout<<"Could not estimate a planar model for the given dataset"<< std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// IMPLEMENTATION OF MY ALGORITHM - START:

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	std::vector<double> f1, f2, f3;
	
    for (auto i = 0; i < maxIterations; i++) 
	{
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(rand() % (cloud->points.size()));
        auto ptr = inliers.begin();
        auto point1 = cloud->points[*ptr];
        double x1 = point1.x;
        double y1 = point1.y;
		double z1 = point1.z;
        ptr++;
		auto point2 = cloud->points[*ptr];
        double x2 = point2.x;
        double y2 = point2.y;
		double z2 = point2.z;
		ptr++;
		auto point3 = cloud->points[*ptr];
        double x3 = point3.x;
        double y3 = point3.y;
		double z3 = point3.z;
        std::vector<double> v1 = {x2 - x1, y2 - y1, z2 - z1};
		std::vector<double> v2 = {x3 - x1, y3 - y1, z3 - z1};
		std::vector<double> v3 = {((y2 - y1)*(z3 - z1)-(z2 - z1)*(y3 - y1)),\
								((z2 - z1)*(x3 - x1)-(x2 - x1)*(z3 - z1)),\
								((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1))};
						
        double A = v3.at(0);
		double B = v3.at(1);
		double C = v3.at(2);
		double D = (-1) * (A * x1 + B * y1 + C * z1);

        for (auto index = 0; index < cloud->points.size(); index++) 
		{
            if (inliers.count(index) > 0)
                continue;
            double x = cloud->points[index].x;
            double y = cloud->points[index].y;
			double z = cloud->points[index].z;
            double d = fabs(A * x + B * y + C * z + D) / sqrt(A * A + B * B + C * C);
            if (d < distanceTol)
                inliers.insert(index);
        }
        if (inliers.size() > inliersResult.size())
		{
            inliersResult = inliers;
			inliers.clear(); 
		}
    }
    typename pcl::PointCloud<PointT>::Ptr  plane_cloud(new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles_cloud(new typename pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        auto point = cloud->points[index];
        if(inliersResult.count(index))
            plane_cloud->points.push_back(point);
        else
            obstacles_cloud->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (obstacles_cloud, plane_cloud);
	return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(const typename pcl::PointCloud<PointT>::Ptr& cloud, int i, std::vector<int> &cluster, std::vector<bool> &processed, KdTree<PointT>*& tree, float distanceTol)
{
    if (!processed[i])
    {
        processed[i] = true;
        cluster.push_back(i);
        std::vector<int> nearby_pts = tree->search(cloud->points[i], distanceTol);
        for (auto indice : nearby_pts)
		{
        	proximity(cloud, indice, cluster, processed, tree, distanceTol);
		}
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud, KdTree<PointT>*& tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(cloud->size(),false);
    for (auto i=0; i< cloud->size(); i++)
    {
        if (!processed[i])
        {
            std::vector<int> cluster;
            proximity(cloud, i, cluster, processed, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }
return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MyClustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree<PointT>* tree = new KdTree<PointT>;
    for (int i=0; i<cloud->size(); i++) 
    	tree->insert(cloud->points[i],i); 
    
    std::vector<std::vector<int>> clusters_indices = euclideanCluster(cloud, tree, clusterTolerance);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (auto it = clusters_indices.begin(); it != clusters_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (auto pit = (*it).begin(); pit != (*it).end(); ++pit)
            cloud_cluster->push_back(cloud->points[*pit]); 
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*cloud)[*pit]); 
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
        j++;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



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