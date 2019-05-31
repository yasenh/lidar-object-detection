/* \author Aaron Brown */
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

/**
 * Developer: Yasen Hu
 * Date: 05/25/2019
 */

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    Car egoCar(Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1(Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2(Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3(Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create LiDAR sensor
    constexpr double kGroundSlope = 0;
    std::shared_ptr<Lidar> lidar_ptr = std::make_shared<Lidar>(cars, kGroundSlope);
    auto input_cloud = lidar_ptr->scan();

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_cloud_processor;
    constexpr int kMaxIterations = 100;
    constexpr float kDistanceThreshold = 0.2;
    auto segment_cloud = point_cloud_processor.SegmentPlane(input_cloud, kMaxIterations, kDistanceThreshold);

    // render obstacles point cloud with red
    renderPointCloud(viewer, segment_cloud.first, "ObstacleCloud", Color(1, 0, 0));
    // render ground plane with green
    renderPointCloud(viewer, segment_cloud.second, "GroundCloud", Color(0, 1, 0));
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_cloud_processor, pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
    // Input point cloud, filter resolution, min Point, max Point
    constexpr float kFilterResolution = 0.2;
    const Eigen::Vector4f kMinPoint(-50, -6.0, -3, 1);
    const Eigen::Vector4f kMaxPoint(60, 6.5, 4, 1);
    auto filter_cloud = point_cloud_processor.FilterCloud(input_cloud, kFilterResolution, kMinPoint, kMaxPoint);

//    renderPointCloud(viewer, filter_cloud, "FilteredCloud");

    constexpr int kMaxIterations = 100;
    constexpr float kDistanceThreshold = 0.2;
    auto segment_cloud = point_cloud_processor.SegmentPlane(filter_cloud, kMaxIterations, kDistanceThreshold);

//    // render obstacles point cloud with red
//    renderPointCloud(viewer, segment_cloud.first, "ObstacleCloud", Color(1, 0, 0));
    // render ground plane with green
    renderPointCloud(viewer, segment_cloud.second, "GroundCloud", Color(0, 1, 0));


    /*** Euclidean clustering ***/
    // float clusterTolerance, int minSize, int maxSize
    constexpr float kClusterTolerance = 0.35;
    constexpr int kMinSize = 15;
    constexpr int kMaxSize = 600;
    auto cloud_clusters = point_cloud_processor.Clustering(segment_cloud.first, kClusterTolerance, kMinSize, kMaxSize);

    int cluster_ID = 1;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0.5, 0, 1)};
    int num_of_colors = colors.size();

    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0, 1), 0.8);

    constexpr float kBBoxMinHeight = 0.75;
    for(const auto& cluster : cloud_clusters) {
        std::cout << "cluster size ";
        point_cloud_processor.numPoints(cluster);

        renderPointCloud(viewer, cluster, "ObstacleCloud" + std::to_string(cluster_ID), colors[cluster_ID % num_of_colors]);

        Box box = point_cloud_processor.BoundingBox(cluster);
        // Filter out some cluster with little points and shorter in height
        if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2) {
            renderBox(viewer, box, cluster_ID);
        }

        cluster_ID++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    viewer->setBackgroundColor(0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle) {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem(1.0);
}


int main (int argc, char** argv) {
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

//    simpleHighway(viewer);
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce();
//    }

    ProcessPointClouds<pcl::PointXYZI> point_cloud_processor;
    std::vector<boost::filesystem::path> stream = point_cloud_processor.streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    while (!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        input_cloud = point_cloud_processor.loadPcd((*stream_iterator).string());
        cityBlock(viewer, point_cloud_processor, input_cloud);

        stream_iterator++;
        // keep looping
        if(stream_iterator == stream.end())
            stream_iterator = stream.begin();

        // viewer spin
        viewer->spinOnce();
    }
}