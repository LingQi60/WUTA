#include <ros/ros.h>
#include "std_msgs/String.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;
    // ros::Publisher pub = n.advertise<std_msgs::String>("chatter",1);
    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();


    }

}


// #include<ros/ros.h>
// #include"std_msgs/String.h"
// #include "union_detect_msgs/BoundingBox.h"
// #include "union_detect_msgs/BoundingBoxes.h"
// #include "union_detect_msgs/Objects.h"
// #include <string.h>
// #include <iostream>
// #include "geometry_msgs/Point32.h"
// #include "sensor_msgs/PointCloud.h"
// #include "sensor_msgs/PointCloud2.h"
// #include "union_detect_msgs/LidarBox.h"
// #include "union_detect_msgs/LidarBoxes.h"
// // #include <darknet_ros_msgs/BoundingBox.h>
// // #include <darknet_ros_msgs/BoundingBoxes.h>
// using namespace std;
// int main(int argc,char **argv)
// {   
//     ros::init(argc,argv,"talker");
//     ros::NodeHandle n;
//     ros::Publisher pub = n.advertise<union_detect_msgs::BoundingBoxes>("/bounding_boxes",100);
//     ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud>("/perception/lidar_cluster",100);
//     ros::Publisher pub3 = n.advertise< union_detect_msgs::LidarBoxes>("/lidar_boxes",100);

//     union_detect_msg::BoundingBox boundingBox;
//     boundingBox.Class='a';
//     boundingBox.id=1;
//     boundingBox.probability=0.1;
//     boundingBox.xmax=2;
//     boundingBox.xmin=0;
//     boundingBox.ymax=2;
//     boundingBox.ymin=0;

//    union_detect_msg::BoundingBoxes boundingBoxesResults_;
//     boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
//     boundingBoxesResults_.header.stamp = ros::Time::now();
//     boundingBoxesResults_.header.frame_id = "detection";
//     boundingBoxesResults_.header.seq = 1;
//     boundingBoxesResults_.image_header.frame_id = "detection";
//     boundingBoxesResults_.image_header.seq =1;
//     boundingBoxesResults_.image_header.stamp = ros::Time::now();

//     geometry_msgs::Point32 tmp;
//     tmp.x = 1;
//     tmp.y = 1;
//     tmp.z = 1;

//     sensor_msgs::PointCloud cluster_;
//     cluster_.points.push_back(tmp);
//     cluster_.header.frame_id = "/base_link";
//     cluster_.header.stamp = ros::Time::now();
//     cluster_.header.seq = 2;

//     union_detect_msg::LidarBox lidarbox;
//     lidarbox.h = 2;
//     lidarbox.l = 2;
//     lidarbox.w = 2;
//     lidarbox.x = 2;
//     lidarbox.y = 2;
//     lidarbox.z = 2;

//     union_detect_msg::LidarBoxes lidarBoxesResult_;
//     lidarBoxesResult_.lidarbox.push_back(lidarbox);
//     lidarBoxesResult_.header.frame_id = "/base_link";
//     lidarBoxesResult_.header.stamp = ros::Time::now();
//     lidarBoxesResult_.header.seq = 2;

//     ros::Rate loop_rate(100);


//     while(ros::ok())
//     {

//         pub.publish(boundingBoxesResults_);
//         pub2.publish(cluster_);
//         pub3.publish(lidarBoxesResult_);

//         // cout<<"I am talker\n" <<"boundingBoxesResults_ = "<<boundingBoxesResults_<<"\n"<<"cluster_ = "<<cluster_;

//         ros::spinOnce();
//         loop_rate.sleep();


//     }
// }