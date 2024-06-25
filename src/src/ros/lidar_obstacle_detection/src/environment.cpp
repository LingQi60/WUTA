#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <cassert>
#include <math.h>
#include <vector>
#include <sstream>

#include "lidar_obstacle_detection/obstacle_detection.h"

using namespace ObstacleDetection_ns;
#if 0
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *PointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud){
   // ProcessPointClouds<pcl::PointXYZI> *PointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = PointProcessorI -> loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
   //renderPointCloud(viewer,inputCloud,"inputCloud");
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = PointProcessorI->FilterCloud(inputCloud,0.3,Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f( 30, 6, 0.5, 1));

   std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = PointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);
   renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));             //实时显示点云信息
   //2 renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));       //显示地面的点云信息

    //clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = PointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);
   renderPointCloud(viewer,cloudClusters[2],"obstCloud",Color(1,0,0));               //显示个别的点云信息
    //std::cout<<"cloud size: "<<cloudClusters.size()<<std::endl;


    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) // cloudClusters 一帧点云中识别到了多少个物体
    {
        std::cout << "cloudClusters size " << cloudClusters.size() << endl;
        std::cout << "cluster size ";
        PointProcessorI->numPoints(cluster);                                                         // cluster->points.size() 检测到该一个物体点的个数
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);    //点云分割,显示不同的颜色
        std::cout<<"ClusterId: " << clusterId<<std::endl;
        //render box;
        Box box = PointProcessorI->BoundingBox(cluster);   // 返回 最大和最小点的位置
        renderBox(viewer,box,clusterId);                   // 框框包围物体
        ++clusterId;
    }
}





ros::Publisher cloud_pub;
ProcessPointClouds<pcl::PointXYZI> *PointProcessorI_;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr input)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr elevatedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Convert from ros msg to PCL::PointCloud data type
    pcl::fromROSMsg(*input, cloud);

    // 去除地面
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = PointProcessorI_->FilterCloud(cloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 6, 0.5, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = PointProcessorI_->SegmentPlane(cloudFiltered, 50, 0.3);

    pcl::toROSMsg(segmentCloud, output);
    output.header.frame_id = "velodyne";
    cloud_pub.publish(output);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "lidar_obstacle_detect");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe("cloud_input", 10, cloud_cb); //原始点云
    ros::Publisher raw_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("raw_cloud_output", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);

    sensor_msgs::PointCloud2 output;
    ros::Rate loop_rate(30);

    

    std::cout << "starting enviroment" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //点云显示句柄
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    //Stream PCD
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    PointProcessorI_ = pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/huazai/Git/Mayixia/catkin_ws_2021-03-16/src/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    while (ros::ok())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());  // 加载pcd点云文件
        //renderPointCloud(viewer,inputCloudI,"inputCloud");                   // 渲染点云,这里显示有点云强度信息,带有不同的颜色，效果更好
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        pcl::toROSMsg(*inputCloudI, output);
        output.header.frame_id = "velodyne";
        raw_cloud_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();

        //cityBlock(viewer, pointProcessorI, inputCloudI);                    // 点云处理 地面识别,点云分割,包围框

    }
}
#endif
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_obstacle_detect");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(30);

    ObstacleDetection obs_det(node, private_nh);
    ros::spinOnce();

    #if 0
    while (ros::ok() && obs_det.pool())
    {
        ros::spinOnce();
        rate.sleep();
    }
    #else
    ros::spin();
    #endif

    return 0;
}

// 调用流程
// pointProcessorI->loadPcd()
//     renderPointCloud() #显示不同的颜色

//     cloudFiltered = PointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 6, 0.5, 1));
// segmentCloud = PointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);

// renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
// renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

// cloudClusters = PointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);
// renderPointCloud(viewer, cloudClusters[2], "obstCloud", Color(1, 0, 0));

// for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
// {
//     std::cout << "cluster size ";
//     PointProcessorI->numPoints(cluster);
//     std::cout << "ClusterId: " << clusterId << std::endl;
//     //render box;
//     Box box = PointProcessorI->BoundingBox(cluster); //得到box的范围
//     renderBox(viewer, box, clusterId);               //画出框
//     ++clusterId;
// }
