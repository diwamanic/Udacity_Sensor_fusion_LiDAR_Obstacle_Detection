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
	
    for (auto i = 0; i < maxIterations; i++) 
	{
        std::unordered_set<int> inliers;
        while (inliers.size() < 2)
            inliers.insert(rand() % (cloud->points.size()));
        auto ptr = inliers.begin();
        pcl::PointXYZ point1 = cloud->points[*ptr];
        int x1 = point1.x;
        int y1 = point1.y;
        ptr++;
		pcl::PointXYZ point2 = cloud->points[*ptr];
        int x2 = point2.x;
        int y2 = point2.y;
        float A = y1 - y2;
        float B = x2 - x1;
        float C = x1 * y2 - x2 * y1;

        for (auto index = 0; index < cloud->points.size(); index++) 
		{
            if (inliers.count(index) > 0)
                continue;
            float x = cloud->points[index].x;
            float y = cloud->points[index].y;
            float d = fabs(A * x + B * y + C) / sqrt(A * A + B * B);
            if (d < distanceTol)
                inliers.insert(index);
        }
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
        pcl::PointXYZ point1 = cloud->points[*ptr];
        double x1 = point1.x;
        double y1 = point1.y;
		double z1 = point1.z;
        ptr++;
		pcl::PointXYZ point2 = cloud->points[*ptr];
        double x2 = point2.x;
        double y2 = point2.y;
		double z2 = point2.z;
		ptr++;
		pcl::PointXYZ point3 = cloud->points[*ptr];
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
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);


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
		std::cout << "Total Inliers" << cloudInliers->points.size() <<std::endl;

		std::cout << "Total Outliers" << cloudOutliers->points.size() <<std::endl;
		//renderPlane(viewer, f1, f2, f3);
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
