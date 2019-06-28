/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar * lidar_sensor = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar_sensor->scan();
    //renderRays(viewer, lidar_sensor->position, cloud);
    //renderPointCloud(viewer, cloud, "point-cloud");
    
    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> * pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud
        = pointCloudProcessor->SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer, segmentedCloud.first, "ground-plane", Color(0, 1, 0));   // green
    renderPointCloud(viewer, segmentedCloud.second, "objects", Color(1, 1, 1));       // white

    const float distTolerance = 1.5; //meters
    const int minNumPoints = 4;     
    const int maxNumPoints = 100;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor->Clustering(segmentedCloud.second, distTolerance, minNumPoints, maxNumPoints);

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),Color(1, 0, 0));
        ++clusterId;

        Box box = pointCloudProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // filter cloud
    const float x_lim[] = {-20, 50};    // x-axis point to the front
    const float y_lim[] = {-8, 8.5};      // y-axis point to the left
    const float z_lim[] = {-5, 5};      // z-axis point up
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pointCloudProcessor->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(x_lim[0], y_lim[0], z_lim[0], 1.0), Eigen::Vector4f(x_lim[1], y_lim[1], z_lim[1], 1.0));
    //renderPointCloud(viewer,cloud,"inputCloud");

    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud
        = pointCloudProcessor->SegmentPlane(cloud, 100, 0.3);
    renderPointCloud(viewer, segmentedCloud.first, "ground-plane", Color(0, 1, 0));   // green
    renderPointCloud(viewer, segmentedCloud.second, "objects", Color(1, 1, 1));       // white

    const float distTolerance = 0.6; //meters
    const int minNumPoints = 10;     
    const int maxNumPoints = 10000;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointCloudProcessor->Clustering(segmentedCloud.second, distTolerance, minNumPoints, maxNumPoints);

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        ++clusterId;

        Box box = pointCloudProcessor->BoundingBox(cluster);

        // any cluster bigger than 'max_obj_len' could be either a building or side boundary of the road
        const float obj_len_thres[] = {0.5, 8};   // {min, max}
        const float obj_width_thres[] = {0.5, 5}; // {min, max}
        const float obj_height_thres[] = {0.5, 5}; // {min, max}

        if( ((box.x_max - box.x_min) >= obj_len_thres[0]) &&
            ((box.x_max - box.x_min) <= obj_len_thres[1]) &&
            ((box.y_max - box.y_min) >= obj_width_thres[0]) &&
            ((box.y_max - box.y_min) <= obj_width_thres[1]) &&
            ((box.z_max - box.z_min) >= obj_height_thres[0]) &&
            ((box.z_max - box.z_min) <= obj_height_thres[1]))
        {
            // cars
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),Color(1, 0, 0));
            renderBox(viewer,box,clusterId);
        }
        else {
            // everything else
            renderBox(viewer,box,clusterId,Color(0, 1, 0));
        }
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
    bool useSimulator = false;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    if(useSimulator) {
        simpleHighway(viewer);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        }
    }
    else {
        // create the point cloud processor object
        ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<boost::filesystem::path> stream = pointCloudProcessor->streamPcd("../src/sensors/data/pcd/data_1");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

        while (!viewer->wasStopped ())
        {
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Load pcd and run obstacle detection process
            inputCloud = pointCloudProcessor->loadPcd((*streamIterator).string());
            cityBlock(viewer, pointCloudProcessor, inputCloud);

            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();

            viewer->spinOnce ();
        }
    }
}
