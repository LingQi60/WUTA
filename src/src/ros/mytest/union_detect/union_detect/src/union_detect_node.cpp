//base-library
#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <typeinfo>
#include <vector>
#include <std_msgs/String.h>
//point-msg
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

// //self-msg
#include "union_detect_msgs/BoundingBox.h"
#include "union_detect_msgs/BoundingBoxes.h"
#include "fsd_common_msgs/Cone.h"
#include "fsd_common_msgs/Map.h"

//image dispose
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//namespace
using namespace std;
using namespace cv;
//self-structure
struct point3d {
    float x = 0;
    float y = 0;
    float z = 0 ;
    float m = 1;
};
struct point2d {
    double u = 0;
    double v = 0;
};
struct _3dbox {
    float x = 0;
    float y = 0;
    float z = 0;
    float w = 0;
    float h = 0;
    float l =0 ;
};
struct _2dbox {
    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    string cls = "";
};
struct _index {
		string cls = "";
		int x = 0;
		int y = 0;
		int z = 0;
	};

//global variable
int seq = 0;
cv::Mat images;
vector<_2dbox> m_Img2dBoxArr;
_2dbox m_Img2dBox;
vector<point2d> m_Point2dArr;
point2d m_Point2d;
vector<point3d> m_Point3dArr;
point3d m_point3d;
vector<_index> m_ResultIndexArr;
_index m_ResultIndex;
vector<_3dbox> m_lidar3dBoxArr;
_3dbox m_lidar3dBox;
vector<_2dbox> m_lidar2dBoxArr;
_2dbox m_lidar2dBox;
//subscriber and publisher
ros::Subscriber boundingboxesSubscriber_;
ros::Subscriber pointsSubscriber_;
ros::Subscriber cloud_pointsSubscriber_;
ros::Subscriber imagesSubscriber_;

ros::Subscriber lidar_bboxesSubscriber_;

ros::Publisher objectspublisher_;
ros::Publisher mappingpublisher_;

//the images result of yolo 
void imagescallback(const sensor_msgs::ImageConstPtr msg)
{
  // ROS_INFO("This is the black: imagescallback");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  images = cv_ptr->image;
  if(!images.empty())
  {
    cv_bridge::CvImage cvImage;
    cvImage.header.stamp = ros::Time::now();
    cvImage.header.frame_id = "point-image-mapping";
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    cvImage.image = images;
    mappingpublisher_.publish(*cvImage.toImageMsg());
  }
  return;
}

//the 2d boundingboxes result of yolo
void boundingboxcallback(const union_detect_msgs::BoundingBoxes &boundingBoxesResults_)
{
  // ROS_INFO("This is the black: boundingboxcallback");
  if(!boundingBoxesResults_.bounding_boxes.empty())
  {
    for(int i = 0; i < boundingBoxesResults_.bounding_boxes.size(); i++)
    {
        m_Img2dBox.cls = boundingBoxesResults_.bounding_boxes[i].Class;
        m_Img2dBox.x1 = boundingBoxesResults_.bounding_boxes[i].xmin;
        m_Img2dBox.x2 = boundingBoxesResults_.bounding_boxes[i].xmax;
        m_Img2dBox.y1 = boundingBoxesResults_.bounding_boxes[i].ymin;
        m_Img2dBox.y2 = boundingBoxesResults_.bounding_boxes[i].ymax;
        m_Img2dBoxArr.push_back(m_Img2dBox);
    }
    if(!m_Img2dBoxArr.empty())
    {
      ROS_INFO("The size of m_Img2dBoxArr is %d",m_Img2dBoxArr.size());
      ROS_INFO("");
    } 
  }
  return;
}

//the center point result of point cluster (---------unused------------)
void pointcallback(const sensor_msgs::PointCloud &cluster_)
{
  // ROS_INFO("This is the black: pointcallback");
  if(!cluster_.points.empty())
  {
    for(int i = 0; i < cluster_.points.size(); i++)
    {
      m_point3d.x=cluster_.points[i].x;
      m_point3d.y=cluster_.points[i].y;
      m_point3d.z=cluster_.points[i].z;
      m_point3d.m=1;
      m_Point3dArr.push_back(m_point3d);
    } 
    if(!m_Point3dArr.empty())
    {
      ROS_INFO("The size of m_Point3dArr is %d",m_Point3dArr.size());
      ROS_INFO("");
    }     
  }
  return;
}

void cloud_pointcallback(const sensor_msgs::PointCloud2 &cloud_msg)
{
  // ROS_INFO("This is the black: cloud_pointcallback");
  sensor_msgs::PointCloud clouddate;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg, clouddate);//sensor_msgs::PointCloud 和 sensor_msgs::PointCloud2之间的转换
  //ROS_INFO("***********************************************")
  if(!clouddate.points.empty())
  {
    for(int i = 0; i < clouddate.points.size(); i++)
    {
      m_point3d.x = clouddate.points[i].x;
      m_point3d.y = clouddate.points[i].y;
      m_point3d.z = clouddate.points[i].z;
      m_point3d.m = 1;
      m_Point3dArr.push_back(m_point3d);
    } 
    if(!m_Point3dArr.empty())
    {
      ROS_INFO("The size of m_Point3dArr is %d",m_Point3dArr.size());
      ROS_INFO("");
    }     
  }
  return;

}

//transform point-cloud into image
void point_Tr_velo_to_img()
{

  Mat Y;
  Mat X;


  Mat velo_to_cam0 = (Mat_<double>(4,4)<< 
    6.9974105811894338e-02, -9.9708431709214729e-01, -3.0438283866086047e-02,2.2150992184109702e-01   ,
    5.1349605453983482e-02, 3.4072919006499114e-02,-9.9809932081436514e-01,4.5005214293362100e-01,
    9.9622630086519315e-01,  6.8278113618224612e-02, 5.3584108326917068e-02,1.6927385845114373e+00, 
    0., 0., 0., 1.  );

  Mat cam_to_img0 = (Mat_<double>(3, 4) <<  
    1.2510499510020970e+03, 0., 1.0001407333516854e+03, 0.,
    0., 1.2324039059174520e+03, 5.1369443851930941e+02, 0.,
    0., 0., 1.,0.);

  // ROS_INFO("______________________________________");
  if (!m_Point3dArr.empty())
  {

    for(int i = 0; i < m_Point3dArr.size(); i++)
    {
      X = (Mat_<double>(4,1)<< m_Point3dArr[i].x, m_Point3dArr[i].y, m_Point3dArr[i].z, m_Point3dArr[i].m);
      Y = cam_to_img0 * velo_to_cam0 * X;
      m_Point2d.u = Y.at<double>(0,0)/Y.at<double>(2,0);
      m_Point2d.v = Y.at<double>(1,0)/Y.at<double>(2,0);
      if(m_Point2d.u > 0 && m_Point2d.u < 1920 && m_Point2d.v > 0 && m_Point2d.v < 1920)
      {
        // m_Point2dArr.push_back(m_Point2d);
        if(!images.empty())
        {
          circle(images, Point(int(m_Point2d.u ), int(m_Point2d.v)), 3, cv::Scalar(0, 255, 120), -1);
        }
      }
    }
    // if(!images.empty())
    // {
    //   cv_bridge::CvImage cvImage;
    //   cvImage.header.stamp = ros::Time::now();
    //   cvImage.header.frame_id = "velodyne";
    //   cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    //   cvImage.image = images;
    //   mappingpublisher_.publish(*cvImage.toImageMsg());
    // }
  }
  return;
}

//judge whether points in the 2d boundingboxes(---------unused------------)
void is_point_in_2dbbox()
{
  // ROS_INFO("This is the black: is_point_in_2dbbox");
  if(!m_Img2dBoxArr.empty() && !m_Point2dArr.empty() && !m_Point3dArr.empty())
  {
    for (int i = 0; i < m_Img2dBoxArr.size(); i++)
    {
      for (int j = 0; j < m_Point2dArr.size(); j++)
      {
        if (m_Point2dArr[j].u > m_Img2dBoxArr[i].x1
          && m_Point2dArr[j].u < m_Img2dBoxArr[i].x2
          && m_Point2dArr[j].v > m_Img2dBoxArr[i].y1
          && m_Point2dArr[j].v < m_Img2dBoxArr[i].y2)
        {
          m_ResultIndex.cls = m_Img2dBoxArr[i].cls;
          m_ResultIndex.x = m_Point3dArr[j].x;
          m_ResultIndex.y = m_Point3dArr[j].y;
          m_ResultIndex.z = m_Point3dArr[j].z;
          m_ResultIndexArr.push_back(m_ResultIndex);
        }
      }
    }
    // if(!m_ResultIndexArr.empty())
    // {
    //   ROS_INFO("The size of m_ResultIndexArr is %d",m_ResultIndexArr.size());
    //   ROS_INFO("");
    // }
  }
  return;
}

//the 3d boundingboxes result of point cluster
void lidar_bboxescallback(const jsk_recognition_msgs::BoundingBoxArray &bbox_array)
{
  if(!bbox_array.boxes.empty())
  {
    // ROS_INFO("This is the black: lidar_bboxescallback");
    for(int i = 0; i < bbox_array.boxes.size(); i++)
    {
      m_lidar3dBox.x = bbox_array.boxes[i].pose.position.x;
      m_lidar3dBox.y = bbox_array.boxes[i].pose.position.y;
      m_lidar3dBox.z = bbox_array.boxes[i].pose.position.z;
      m_lidar3dBox.l = bbox_array.boxes[i].dimensions.x;
      m_lidar3dBox.w = bbox_array.boxes[i].dimensions.y;
      m_lidar3dBox.h = bbox_array.boxes[i].dimensions.z;
      m_lidar3dBoxArr.push_back(m_lidar3dBox);
    }
    // if(!m_lidar3dBoxArr.empty())
    // {
    //   ROS_INFO("The size of m_lidar3dBoxArr is %d",m_lidar3dBoxArr.size());
    //   ROS_INFO("");
    // }
  }
  return;
}

//transform 3d boundingboxes into 2d boundingboxes of point cluster
void box_Tr_velo_to_img()
{
  // ROS_INFO("");
  // ROS_INFO("______________________________________");
  // ROS_INFO("This is the black: point_Tr_velo_to_img");
  Mat Y1;
  Mat Y2;
  Mat X1;
  Mat X2;

  // Mat velo_to_cam0 = (Mat_<double>(4,4)<< 
  // 3.8158970452755070e-02,-9.9919547021841981e-01 , -1.2341202087996384e-02,3.4333227555668117e-02,
  // 5.4152121511705176e-02, 1.4399786627030453e-02, -9.9842886270423770e-01, -9.0777367107424778e-01,
  // 9.9780330762619107e-01,3.7430715196039188e-02 , 5.4658035549639994e-02,-7.7453526167384212e-01, 
  // 0., 0., 0., 1. );

  // Mat cam_to_img0 = (Mat_<double>(3, 4) <<  
  // 1.1018550478406341e+03, 0., 1.0374826496205360e+03,0,
  // 0.,1.1072674653585802e+03, 5.8028032885303412e+02, 0,
  // 0., 0., 1. ,0);
  Mat velo_to_cam0 = (Mat_<double>(4,4)<< 
    6.9974105811894338e-02, -9.9708431709214729e-01, -3.0438283866086047e-02,2.2150992184109702e-01   ,
    5.1349605453983482e-02, 3.4072919006499114e-02,-9.9809932081436514e-01,4.5005214293362100e-01,
    9.9622630086519315e-01,  6.8278113618224612e-02, 5.3584108326917068e-02,1.6927385845114373e+00, 
    0., 0., 0., 1.  );

  Mat cam_to_img0 = (Mat_<double>(3, 4) <<  
    1.2510499510020970e+03, 0., 1.0001407333516854e+03, 0.,
    0., 1.2324039059174520e+03, 5.1369443851930941e+02, 0.,
    0., 0., 1.,0.);

  if (!m_lidar3dBoxArr.empty())
  {
    for(int i = 0; i < m_lidar3dBoxArr.size(); i++)
    {
      X1 = (Mat_<double>(4,1)<< m_lidar3dBoxArr[i].x + m_lidar3dBoxArr[i].l/2, 
                                m_lidar3dBoxArr[i].y + m_lidar3dBoxArr[i].w/2, 
                                m_lidar3dBoxArr[i].z + m_lidar3dBoxArr[i].h/2, 1);
      X2 = (Mat_<double>(4,1)<< m_lidar3dBoxArr[i].x - m_lidar3dBoxArr[i].l/2, 
                                m_lidar3dBoxArr[i].y - m_lidar3dBoxArr[i].w/2, 
                                m_lidar3dBoxArr[i].z - m_lidar3dBoxArr[i].h/2, 1);
      Y1 = cam_to_img0 * velo_to_cam0 * X1;
      Y2 = cam_to_img0 * velo_to_cam0 * X2;
      m_lidar2dBox.x1 = Y1.at<double>(0,0)/Y1.at<double>(2,0);
      m_lidar2dBox.y1 = Y1.at<double>(1,0)/Y1.at<double>(2,0);
      m_lidar2dBox.x2 = Y2.at<double>(0,0)/Y2.at<double>(2,0);
      m_lidar2dBox.y2 = Y2.at<double>(1,0)/Y2.at<double>(2,0);
      // ROS_INFO("m_lidar2dBox.x1 is %f",Y1.at<double>(0,0)/Y1.at<double>(2,0));
      // ROS_INFO("m_lidar2dBox.y1 is %f",m_lidar2dBox.y1);
      // ROS_INFO("m_lidar2dBox.x2 is %f",m_lidar2dBox.x2);
      // ROS_INFO("m_lidar2dBox.y2 is %f",m_lidar2dBox.y2);
      if(m_lidar2dBox.x1 > 0 && m_lidar2dBox.x1 < 1920 && m_lidar2dBox.y1 > 0 && m_lidar2dBox.y1 < 1920
      && m_lidar2dBox.x2 > 0 && m_lidar2dBox.x2 < 1920 && m_lidar2dBox.y2 > 0 && m_lidar2dBox.y2 < 1920)
      {
        m_lidar2dBoxArr.push_back(m_lidar2dBox);
        if(!images.empty())
        {
          rectangle(images,Point(int( m_lidar2dBox.x1),int( m_lidar2dBox.y1)),
           Point(int( m_lidar2dBox.x2),int( m_lidar2dBox.y2)),Scalar(0,255,255),2);
        }
      }
    }
    if(!images.empty())
    {
      cv_bridge::CvImage cvImage;
      cvImage.header.stamp = ros::Time::now();
      cvImage.header.frame_id = "point-image-mapping";
      cvImage.encoding = sensor_msgs::image_encodings::BGR8;
      cvImage.image = images;
      mappingpublisher_.publish(*cvImage.toImageMsg());
    }

  }
  return;
}

//calculate the iou between 2d boundingboxes of point cluster and the one of yolo
void iou_calculate()
{
  // ROS_INFO("This is the black: iou_calculate");
	float in_w = 0;
	float in_h = 0;
	float s_intersection = 0;
	float s_union = 0;
  float iou;
  float iou_threshold =0.1;
  if(!m_Img2dBoxArr.empty() && !m_lidar2dBoxArr.empty())
  {
    for(int i = 0; i < m_Img2dBoxArr.size(); i++)
    {
      for(int j = 0; j < m_lidar2dBoxArr.size(); j++)
      {
        in_w = float(min(m_Img2dBoxArr[i].x2, m_lidar2dBoxArr[j].x2) - max(m_Img2dBoxArr[i].x1, m_lidar2dBoxArr[j].x1));
        in_h = float(min(m_Img2dBoxArr[i].y2, m_lidar2dBoxArr[j].y2) - max(m_Img2dBoxArr[i].y1, m_lidar2dBoxArr[j].y1));
        s_intersection = in_w * in_h;
        // ROS_INFO("m_Img2dBoxArr[i].x2 is %f",m_Img2dBoxArr[i].x2);
        // ROS_INFO("m_lidar2dBoxArr[j].x2 is %f",m_lidar2dBoxArr[j].x2);
        // ROS_INFO("m_Img2dBoxArr[i].x1 is %f",m_Img2dBoxArr[i].x1);
        // ROS_INFO("m_lidar2dBoxArr[j].x1 is %f",m_lidar2dBoxArr[j].x1);
        // ROS_INFO("in_w is %f",in_w);
        // ROS_INFO("in_h is %f",in_h);
        ROS_INFO("s_intersection is %f",s_intersection);
        if (in_w < 0 || in_h < 0)
        {
          s_intersection = 0;
        }
        
        s_union = float((m_Img2dBoxArr[i].x2 - m_Img2dBoxArr[i].x1) * (m_Img2dBoxArr[i].y2 - m_Img2dBoxArr[i].y1)
                  + (m_lidar2dBoxArr[j].x2 - m_lidar2dBoxArr[j].x1) * (m_lidar2dBoxArr[j].y2 - m_lidar2dBoxArr[j].y1));
        ROS_INFO("s_union is %f",s_union);
        iou = s_intersection / s_union;
        ROS_INFO("iou is %f",iou);
        if(iou > iou_threshold)
        {
          m_ResultIndex.cls = m_Img2dBoxArr[i].cls;
          m_ResultIndex.x = m_Point3dArr[j].x;
          m_ResultIndex.y = m_Point3dArr[j].y;
          m_ResultIndex.z = m_Point3dArr[j].z;
          m_ResultIndexArr.push_back(m_ResultIndex);
        }
      }
    }
    if(!m_ResultIndexArr.empty())
    {
      ROS_INFO("The size of m_ResultIndexArr is %d",m_ResultIndexArr.size());
      ROS_INFO("");
    }
  }
  return;
}

//publish the result of union-detect
void object_publish() 
{	
  // ROS_INFO("This is the black: object_publish");
  fsd_common_msgs::Map map;
  fsd_common_msgs::Cone cone;
  map.cone_blue.clear();
  map.cone_red.clear();
  map.cone_unknow.clear();
  map.cone_yellow.clear();
  if(!m_ResultIndexArr.empty())
  {
    for(int i = 0; i < m_ResultIndexArr.size(); i++)
    {
      cone.color.data = m_ResultIndexArr[i].cls;
      cone.position.x = m_ResultIndexArr[i].x;
      cone.position.y = m_ResultIndexArr[i].y;
      cone.position.z = m_ResultIndexArr[i].z;
      ROS_INFO("The cone position is %f,%f,%f",cone.position.x cone.position.y cone.position.z);
      ROS_INFO("The cone color is %s",cone.color.data);
      if(m_ResultIndexArr[i].cls == "r")
      {
        map.cone_red.push_back(cone);
      }
      else if(m_ResultIndexArr[i].cls == "b")
      {
        map.cone_blue.push_back(cone);
      }
      else if(m_ResultIndexArr[i].cls == "y")
      {
        map.cone_yellow.push_back(cone);
      }
      else
      {
        map.cone_unknow.push_back(cone);
      }
    }
    map.header.stamp = ros::Time::now();
    map.header.frame_id = "map";
    map.header.seq = seq;
    objectspublisher_.publish(map);
    // ROS_INFO("The result of union-detect has been published");
    // ROS_INFO("");
  }
//  fsd_common_msgs::Map getLocalMap(const sensor_msgs::PointCloud2 &cones) {
//   fsd_common_msgs::Map map;
//   map.header = cones.header;
//   map.cone_red.clear();
//   map.cone_blue.clear();

//   pcl::PointCloud<pcl::PointXYZI> cloud;
//   pcl::fromROSMsg(cones, cloud);

//   geometry_msgs::Point tmp;
//   for (int i = 0; i < cloud.size(); i++) {
//     tmp.x = cloud[i].x;
//     tmp.y = cloud[i].y;
//     tmp.z = cloud[i].z;
//     if (cloud[i].intensity < 0.1 && cloud[i].intensity > 0.005) {
//       //red
//       // map.cone_red.push_back(getConeFromPoint(tmp, "r"));
//       map.cone_red.push_back(cone);
//     } else if (cloud[i].intensity > 0.9) {
//       //blue
//       // map.cone_blue.push_back(getConeFromPoint(tmp, "b"));
//       map.cone_red.push_back(cone);
//     }
//   }
//   return msg;//使用激光雷达反射强度确定锥桶的颜色和位置，可以作为某终冗余
// }
// sensor_msgs::PointCloud getLidarCluster(const sensor_msgs::PointCloud2 &cones) {
//   sensor_msgs::PointCloud msg;
//   msg.header = cones.header;

//   pcl::PointCloud<pcl::PointXYZI> cloud;
//   pcl::fromROSMsg(cones, cloud);

//   geometry_msgs::Point32 tmp;
//   for (int i = 0; i < cloud.size(); i++) {
//     if (cloud[i].intensity > 0) {
//       tmp.x = cloud[i].x;
//       tmp.y = cloud[i].y;
//       tmp.z = cloud[i].z;
//       msg.points.push_back(tmp);
//     }
//   }
//  return msg;//将激光雷达标准格式转换为此工程定义的格式

  return;
} 

//clear out all variables
void object_clear()
{
  // ROS_INFO("This is the black: object_clear");
  m_Img2dBoxArr.clear();
  m_Point3dArr.clear();
  m_Point2dArr.clear();
  m_lidar3dBoxArr.clear();
  m_lidar2dBoxArr.clear();
  m_ResultIndexArr.clear();
  // ROS_INFO("The variables have been cleaned");
  return;                                 
}

//the main funtion
void union_detector()
{
  point_Tr_velo_to_img();
  box_Tr_velo_to_img();
  iou_calculate();
  object_publish();
  return;
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "union_detect");
  ros::NodeHandle nodeHandle_("");


  std::string BoundingboxesTopicName;
  int BoundingBoxesQueueSize;

  std::string PointsTopicName;
  int pointsQueueSize;

  std::string cloud_PointsTopicName;
  int cloud_pointsQueueSize;

  std::string ImagesTopicName;
  int ImagesQueueSize;

  std::string lidar_bboxesTopicName;
  int lidar_bboxesQueueSize;

  std::string Object_ResultTopicName;
  int Object_ResultQueueSize;
  bool Object_ResultLatch;

  std::string MappingTopicName;
  int MappingQueueSize;
  bool MappingLatch;

  nodeHandle_.param("subscribers/boundingboxes/topic", BoundingboxesTopicName, std::string("/darknet_ros/bounding_boxes"));
  nodeHandle_.param("subscribers/boundingboxes/queue_size",  BoundingBoxesQueueSize, 1);

  nodeHandle_.param("subscribers/points/topic", PointsTopicName, std::string("/perception/lidar_cluster"));
  nodeHandle_.param("subscribers/points/queue_size",  pointsQueueSize, 1);

  nodeHandle_.param("subscribers/cloud_points/topic", cloud_PointsTopicName, std::string("/filtered_points_no_ground"));
  nodeHandle_.param("subscribers/cloud_points/queue_size",  cloud_pointsQueueSize, 10);

  // nodeHandle_.param("subscribers/images/topic", ImagesTopicName, std::string("/usb_cam/image_raw"));
    nodeHandle_.param("subscribers/images/topic", ImagesTopicName, std::string("/darknet_ros/detection_image"));
  nodeHandle_.param("subscribers/images/queue_size",  ImagesQueueSize, 1);

  nodeHandle_.param("subscribers/lidar_bboxes/topic", lidar_bboxesTopicName, std::string("/detected_bounding_boxs"));
  nodeHandle_.param("subscribers/lidar_bboxes/queue_size", lidar_bboxesQueueSize, 1);

  nodeHandle_.param("publishers/union_detect_result/topic", Object_ResultTopicName,
                    std::string("/map"));
  nodeHandle_.param("publishers/union_detect_result/queue_size", Object_ResultQueueSize, 1);
  nodeHandle_.param("publishers/union_detect_result/latch", Object_ResultLatch, true);

  nodeHandle_.param("publishers/mapping/topic", MappingTopicName, std::string("/union_detect/mapping"));
  nodeHandle_.param("publishers/mapping/queue_size", MappingQueueSize, 1);
  nodeHandle_.param("publishers/mapping/latch", MappingLatch, true);

  boundingboxesSubscriber_ = nodeHandle_.subscribe(BoundingboxesTopicName, BoundingBoxesQueueSize, boundingboxcallback);

  // -----unused---------
  // pointsSubscriber_ = nodeHandle_.subscribe(PointsTopicName,  pointsQueueSize, pointcallback);

  // ----point_cloud visualization----------
  cloud_pointsSubscriber_ = nodeHandle_.subscribe(cloud_PointsTopicName,  cloud_pointsQueueSize, cloud_pointcallback);

  imagesSubscriber_ = nodeHandle_.subscribe(ImagesTopicName,  ImagesQueueSize, imagescallback);

  lidar_bboxesSubscriber_ = nodeHandle_.subscribe( lidar_bboxesTopicName, lidar_bboxesQueueSize, lidar_bboxescallback);

  objectspublisher_ = nodeHandle_.advertise<fsd_common_msgs::Map>(Object_ResultTopicName,
                      Object_ResultQueueSize,Object_ResultLatch);

  mappingpublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(MappingTopicName,
                      MappingQueueSize,MappingLatch);

  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    ros::spinOnce();

    // if(!m_Img2dBoxArr.empty() && !m_lidar3dBoxArr.empty())
    // if(!m_lidar3dBoxArr.empty())
    // {
    union_detector();
    // }
    // point_Tr_velo_to_img();
    object_clear();
    seq++;
    loop_rate.sleep();
  }
  return 0;
}
