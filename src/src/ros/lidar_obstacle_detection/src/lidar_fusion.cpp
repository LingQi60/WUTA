#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

ros::Publisher output_;


//ros回调函数，拼接点云

void callback(const sensor_msgs::PointCloud2::ConstPtr &left_input, const sensor_msgs::PointCloud2::ConstPtr &right_input)
{

    static int count = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*left_input, *left_local_laser);

    // 坐标转换
    Eigen::Affine3f g_letf_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_letf_calibration_matrix.translation() << 1.105, -0.575, -0.92;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_letf_calibration_matrix.rotate(Eigen::AngleAxisf(-0.785, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr left_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*left_local_laser, *left_calibration_cloud, g_letf_calibration_matrix); // Eigen::Affine3f

    // for (std::size_t i = 0; i < left_calibration_cloud->size(); ++i)
    // {
    //     left_calibration_cloud->points[i].intensity = 64;
    // }

    // publishCloudI(&g_left_calib_point_pub, *left_calibration_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr right_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*right_input, *right_local_laser);

    // 坐标转换
    Eigen::Affine3f g_right_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_right_calibration_matrix.translation() << -1.105, 0.575, -0.92;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_right_calibration_matrix.rotate(Eigen::AngleAxisf(-0.785, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr right_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*right_local_laser, *right_calibration_cloud, g_right_calibration_matrix);
    // for (std::size_t i = 0; i < right_calibration_cloud->size(); ++i)
    // {
    //     right_calibration_cloud->points[i].intensity = 128;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr left_right_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *left_right_calibration_cloud = *left_calibration_cloud + *right_calibration_cloud;

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*left_right_calibration_cloud, outMsg);
    outMsg.header.frame_id = "velodyne";
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();
    output_.publish(outMsg);
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &left_input, const sensor_msgs::PointCloud2::ConstPtr &right_input, const sensor_msgs::PointCloud2::ConstPtr &middle_input)
{
    static int count = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*left_input, *left_local_laser);

    // 坐标转换
    Eigen::Affine3f g_letf_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_letf_calibration_matrix.translation() << 1.105, -0.575, -0.92;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_letf_calibration_matrix.rotate(Eigen::AngleAxisf(-0.785, Eigen::Vector3f::UnitZ()));


    pcl::PointCloud<pcl::PointXYZI>::Ptr left_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*left_local_laser, *left_calibration_cloud, g_letf_calibration_matrix);

    // for (std::size_t i = 0; i < left_calibration_cloud->size(); ++i)
    // {
    //     left_calibration_cloud->points[i].intensity = 64;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr middle_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*middle_input, *middle_local_laser);

    // 坐标转换
    Eigen::Affine3f g_middle_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_middle_calibration_matrix.translation() << 0.6, -0.1, 0.2;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_middle_calibration_matrix.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr middle_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*middle_local_laser, *middle_calibration_cloud, g_middle_calibration_matrix);

    // for (std::size_t i = 0; i < middle_calibration_cloud->size(); ++i)
    // {
    //     middle_calibration_cloud->points[i].intensity = 64;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr right_local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*right_input, *right_local_laser);

    // 坐标转换
    Eigen::Affine3f g_right_calibration_matrix = Eigen::Affine3f::Identity();
    // 在 X 轴上定义一个 2.5 米的平移.
    g_right_calibration_matrix.translation() << -1.105, 0.575, -0.92;
    // 和前面一样的旋转; Z 轴上旋转 theta 弧度
    g_right_calibration_matrix.rotate(Eigen::AngleAxisf(-0.785, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr right_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*right_local_laser, *right_calibration_cloud, g_right_calibration_matrix);
    // for (std::size_t i = 0; i < right_calibration_cloud->size(); ++i)
    // {
    //     right_calibration_cloud->points[i].intensity = 128;
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr left_right_middle_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *left_right_middle_calibration_cloud = *left_calibration_cloud + *right_calibration_cloud ;//+ *middle_calibration_cloud ;
    *left_right_middle_calibration_cloud = *left_right_middle_calibration_cloud + *middle_calibration_cloud ;

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*left_right_middle_calibration_cloud, outMsg);
    outMsg.header.frame_id = "velodyne";
    outMsg.header.seq = ++count;
    outMsg.header.stamp = ros::Time::now();
    output_.publish(outMsg);
}

int main(int argc, char **argv)//多雷达融合
{

    ros::init(argc, argv, "lidar_fusion");
    ros::NodeHandle nh;

    // 需要用message_filter容器对两个话题的数据发布进行初始化，这里不能指定回调函数
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_left(nh, "/lidar16_1/lslidar_point_cloud", 10); //,ros::TransportHints().tcpNoDelay()
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_right(nh, "/lidar16_2/lslidar_point_cloud", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_middle(nh, "/lidar32_1/lslidar_point_cloud", 10);

    output_ = nh.advertise<sensor_msgs::PointCloud2>("output_fusion_points", 10);

#if 0
    // 将两个话题的数据进行同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    // 注意修改 100000同步时间,10
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100000), points_sub_left, points_sub_right, points_sub_middle);

    // 指定一个回调函数，就可以实现两个话题数据的同步获取
    sync.registerCallback(boost::bind(&callback, _1, _2,_3));
#else
    // 将两个话题的数据进行同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    // 注意修改 100000同步时间,10
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100000), points_sub_left, points_sub_right);
    // 指定一个回调函数，就可以实现两个话题数据的同步获取
    sync.registerCallback(boost::bind(&callback, _1, _2));
#endif
    ros::spin();
    return 0;
}
