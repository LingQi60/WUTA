/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "lidar_cluster.hpp"
#include <ros/ros.h>
#include <sstream>
#include <utility>

namespace ns_lidar_cluster {

LidarCluster::LidarCluster(ros::NodeHandle &nh) : nh_(nh) { loadParameters(); };

//获得标志参数
void LidarCluster::loadParameters() {
  getRawLidar_ = false;
  is_ok_flag_ = false;
}
//将cluster_的值返回给getLidarCluster函数
sensor_msgs::PointCloud LidarCluster::getLidarCluster() { return cluster_; }

bool LidarCluster::is_ok() const { return is_ok_flag_; }

//接受点云信息,将点云数据传给raw_pc2
void LidarCluster::setRawLidar(const sensor_msgs::PointCloud2 &msg) {
  raw_pc2_ = msg;
  getRawLidar_ = true;
}

//执行过滤分割聚类
void LidarCluster::runAlgorithm() {
  
  //确保正确接收到点云信息
  if (raw_pc2_.fields.empty() || !getRawLidar_) {
    return;
  }
  getRawLidar_ = false;

//将ROS中的sensor_msgs型消息raw_pc2数据格式转化为PCL型消息raw_pc_数据结构
  pcl::fromROSMsg(raw_pc2_, raw_pc_);

//定义PCL型信息cloud_ground与cloud_cones
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground( new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cones( new pcl::PointCloud<pcl::PointXYZI>);

  // 预处理，分割地面点云
  preprocessing(raw_pc_, cloud_ground, cloud_cones);

  // 使用欧式聚类来获取锥桶位置
  ClusterProcessing(cloud_cones, 0.5);

  cluster_.header.frame_id = "/base_link";//该帧的id
  cluster_.header.stamp = raw_pc2_.header.stamp;//该帧的时间戳
  is_ok_flag_ = true;
}

void LidarCluster::preprocessing(
    pcl::PointCloud<pcl::PointXYZI> &raw,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones) {

  //定义PCL型信息，储存初步过滤信息
  pcl::PointCloud<pcl::PointXYZI> filtered;

  for (auto &iter : raw.points) {
    if (std::hypot(iter.x, iter.y) < sqrt(2) || iter.z > 0.7 ||
        iter.x < 0 || (std::hypot(iter.x, iter.y) > 7 && iter.z < 0.03))
      continue;
    filtered.points.push_back(iter);//初步过滤明显为非锥桶的点云信息
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//声明该分割模型的系数
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//申明存储模型的内点的索引
 
  pcl::SACSegmentation<pcl::PointXYZI> seg; // 创建一个分割模型seg

  seg.setOptimizeCoefficients(true);  //设置该估计模型的最优化参数因子
  // 以下为强制性参数设置
  seg.setModelType(pcl::SACMODEL_PLANE);//设置平面模型类型
  seg.setMethodType(pcl::SAC_RANSAC);//设置分割平面模型所用的随机参数估计方法
 
  seg.setDistanceThreshold(0.07);//设置最小的距离阈值0.07，该距离阀值决定了点被认为是局内点所必须满足的条件
  seg.setInputCloud(filtered.makeShared());//设置输入已经初步过滤过的点云信息
  seg.segment(*inliers, *coefficients);//引发分割实现，从余下的点云中分割最大平面组成部分，并将分割结果储存到点的索引inliers及存储平面模型的系数coefficients

  
  //分割分离地面点云
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(raw.makeShared());//设置输入原始点云数据
  extract.setIndices(inliers);//设置输入分割后的点索引并将其它点云数据分离为地面点云数据
  extract.filter(*cloud_ground);//将分离结果保存到cloud_ground中

  // 分割提取锥桶点云
  extract.setNegative(true);//提取非地面点云数据
  extract.filter(*cloud_cones);//将锥桶的提取结果保存到cloud_cones中
//将PCL型消息数据格式转化为ROS型消息数据格式
  pcl::toROSMsg(*cloud_ground, filter_ground_);
  pcl::toROSMsg(*cloud_cones, filter_cones_);
}

void LidarCluster::ClusterProcessing(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold) {

  cluster_.points.clear();//初始化

  //  为提取的搜索方法创建 KdTree 对象
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);//设置输入锥桶点云到搜索树中

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;//设置欧式聚类对象ec
  ec.setClusterTolerance(threshold);//设置近邻搜索的搜索半径为50cm
  ec.setMinClusterSize(2);//设置一个聚类需要的最少的点数目为2
  ec.setMaxClusterSize(200);//设置一个聚类需要的最大点数目为200
  ec.setSearchMethod(tree); //设置点云的搜索机制
  ec.setInputCloud(cloud);//输入锥桶点云
  ec.extract(cluster_indices);//引发聚类实现，从点云中提取聚类，并将锥桶点云索引保存在cluster_indices中

//遍历访问锥桶点云索引cluster_indices,输出锥桶空间位置坐标及锥桶边界大小
  for (const auto &iter : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cone(new pcl::PointCloud<pcl::PointXYZI>);//定义pcl类信息cone

//将所有cloud点云索引返回给cone
    for (auto it : iter.indices) {
      cone->points.push_back(cloud->points[it]);
    }
    //从points中的几何信息传给cone中的成员
    cone->width = cone->points.size();
    cone->height = 1;
    cone->is_dense = true;

    //定义四元数组代表质心，点云位置最大最小值
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::compute3DCentroid(*cone, centroid);//计算获得每个锥桶的质心位置
    pcl::getMinMax3D(*cone, min, max);//获得每个锥桶的最大最小点云值

   //计算锥桶边界大小
    float bound_x = std::fabs(max[0] - min[0]);
    float bound_y = std::fabs(max[1] - min[1]);
    float bound_z = std::fabs(max[2] - min[2]);

    // 基于锥桶大小和位置过滤有明显误差的锥桶并将最桶质心位置返回给cluster_
    if (bound_x < 0.5 && bound_y < 0.5 && bound_z < 0.4 && centroid[2] < 0.4) {
      geometry_msgs::Point32 tmp;
      tmp.x = centroid[0];
      tmp.y = centroid[1];
      tmp.z = centroid[2];
      cluster_.points.push_back(tmp);
    }
  }
}

} 
