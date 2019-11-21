/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quiz/cluster/kdtree.h"

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
void clusterHelper(int indices, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indices] = true;
	cluster.push_back(indices);

	std::vector<int> nearest = tree->search(points[indices], distanceTol);
	for(int id : nearest)
	{
		if(!processed[id])
		{
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}


}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	int i = 0;
	while(i < points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	

	return clusters;

}//
/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
	Lidar* lidar = new Lidar(cars,0); // creates lidar pointer, 		which points to a Lidar object on the heap
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    ProcessPointClouds<pcl::PointXYZ>* PointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = PointProcessor->ransac_plane(inputCloud, 1000, 0.7);
    renderPointCloud(viewer,SegmentCloud.first,"Obstacle",Color(1,0,0));
    //renderPointCloud(viewer,SegmentCloud.second,"Plane",Color(0,1,0));
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<std::vector<float>> points;
	KdTree* tree = new KdTree;
    int minSize = 3; int maxSize = 30; float dist = 1;
    int i = 0;
	while(i < SegmentCloud.first->points.size())
    {
		pcl::PointXYZ point = SegmentCloud.first->points[i];
		tree->insert({point.x, point.y, point.z}, i);
		points.push_back({point.x, point.y, point.z});
        i++;
	}
	std::vector<std::vector<int>> indices = euclideanCluster(points, tree, dist);

	for (std::vector<int> index : indices)
    {
		typename pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
		if (index.size() >= minSize && index.size() <= maxSize)
        {
			for (int id : index) 
            {
				cluster->points.push_back(SegmentCloud.first->points[id]);
		    }
			clusters.push_back(cluster);
        }
    }
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudcluster = PointProcessor->Clustering(SegmentCloud.first, 1, 3, 30);
    int clusterid = 0;
    std::vector<Color> Colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
    {
        cout << "Cluster Size"; PointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster, "ObstacleCloud"+std::to_string(clusterid),Colors[clusterid%Colors.size()]);
        Box box = PointProcessor->BoundingBox(cluster);
    	renderBox(viewer, box, clusterid);
        ++clusterid;
    }
}*/

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->FilterCloud(cloud, 0.3f, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentCloud = pointProcessorI->ransac_plane(inputCloud, 25, 0.3);
    renderPointCloud(viewer,SegmentCloud.first,"Obstacle",Color(1,0,0));
    renderPointCloud(viewer,SegmentCloud.second,"Plane",Color(0,1,0));
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    std::vector<std::vector<float>> points;
	KdTree* tree = new KdTree;
    int minSize = 10; int maxSize = 500; float dist = 0.53;
    int i = 0;
	while(i < SegmentCloud.first->points.size())
    {
		auto point = SegmentCloud.first->points[i];
		tree->insert({point.x, point.y, point.z}, i);
		points.push_back({point.x, point.y, point.z});
        i++;
	}
	std::vector<std::vector<int>> indices = euclideanCluster(points, tree, dist);

	for (std::vector<int> index : indices)
    {
		typename pcl::PointCloud<pcl::PointXYZI>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZI>);
		if (index.size() >= minSize && index.size() <= maxSize)
        {
			for (int id : index) 
            {
				cluster->points.push_back(SegmentCloud.first->points[id]);
		    }
			clusters.push_back(cluster);
        }
    }
    
    int clusterid = 0;
    std::vector<Color> Colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
    {
        cout << "Cluster Size"; pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster, "ObstacleCloud"+std::to_string(clusterid),Colors[clusterid%Colors.size()]);//
        Box box = pointProcessorI->BoundingBox(cluster);
    	renderBox(viewer, box, clusterid);
        ++clusterid;
    }
    //renderPointCloud(viewer,inputCloud,"Cloud");
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

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    }
    
    
    
    /*simpleHighway(viewer);
    cityBlock(viewer);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }*/
}