#ifndef PUREPURSUIT_ALGORITHM_H
#define PUREPURSUIT_ALGORITHM_H

#include "lidar_obstacle_detection/render.h"             // 显示相关的库
#include "lidar_obstacle_detection/processPointClouds.h" // 点云处理的库
// using templates for processPointClouds so also include .cpp to help linker
#include "../../src/processPointClouds.cpp" // 点云处理的库

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl/io/pcd_io.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <chrono>
#include <math.h>

using namespace std;
namespace ObstacleDetection_ns
{
    class ObstacleDetection
    {
    public:
        ObstacleDetection(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~ObstacleDetection();
        bool pool();


    private:
        bool vehLocUpdatRdy, firstGetGoalIndxRdy;
        string frame_id;
        string lidar_type;
        // 读pcd文件
        string path_dir;
        vector<boost::filesystem::path> stream;
        vector<boost::filesystem::path>::iterator streamIterator; //指向 stream.begin()，需要加为:：iterator类型(在main中直接使用auto解决)
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
        ProcessPointClouds<pcl::PointXYZI> *pointProcessorI;
        ros::Subscriber cloud_sub;
        ros::Publisher raw_cloud_pub, cloud_pub, box_pub, box_pub_ab,box_car_pub, speed_ratio_pub;
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg); // topic回调
        void bounding_box(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters);
        void pub_speed_ratio(double min);
        bool speed_control(Box box_point,double speed_ratio[]);
        double car_length, car_width, car_hight;
        double front_stop_distance, edge_stop_distance, back_stop_distance;
        //double speed_ratio[3];
        
        jsk_recognition_msgs::BoundingBox box_car;
    };
}
#endif // PUREPURSUIT_ALGORITHM_H
