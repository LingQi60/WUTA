
#include "lidar_obstacle_detection/obstacle_detection.h"

using namespace ObstacleDetection_ns;
 #define _OPENMP
ObstacleDetection::ObstacleDetection(ros::NodeHandle node, ros::NodeHandle private_nh)
{

    cloud_sub = node.subscribe("/points_raw", 10, &ObstacleDetection::cloud_callback,this); //原始点云
    raw_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("raw_cloud_output", 1);
    cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);

    box_car_pub = node.advertise<jsk_recognition_msgs::BoundingBox>("object_box_car", 1,true);
    // box_pub = node.advertise<jsk_recognition_msgs::BoundingBoxArray>("object_box", 10);// 障碍物
    box_pub = node.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 10);
    box_pub_ab = node.advertise<jsk_recognition_msgs::BoundingBoxArray>("object_box_ab", 10); // 受到距离限制的障碍物
    speed_ratio_pub = node.advertise<std_msgs::Float64>("speed_ratio", 10);              // 速度限制

    private_nh.param<string>("frame_id", frame_id, "velodyne");

    private_nh.param<double>("car_length", car_length, 1.9);
    private_nh.param<double>("car_width", car_width, 1.0);
    private_nh.param<double>("car_hight", car_hight, 1.5);
    private_nh.param<double>("front_stop_distance", front_stop_distance, 1.0);
    private_nh.param<double>("edge_stop_distance", edge_stop_distance, 0.25);
    private_nh.param<double>("back_stop_distance", back_stop_distance, 0.1);


    private_nh.param<string>("lidar_type", lidar_type, "C16");

    //////////////////////////////
    #if 0
    path_dir = "/home/huazai/Git/Mayixia/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2";
    //Stream PCD
    pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    stream = pointProcessorI->streamPcd(path_dir);
    streamIterator = stream.begin();
    #endif

     // 发布车体体积
    box_car.pose.position.x = 0;
    box_car.pose.position.y =  0;
    box_car.pose.position.z = car_hight / 2;
    box_car.dimensions.x = car_length;
    box_car.dimensions.y = car_width;
    box_car.dimensions.z = car_hight;

    box_car.header.frame_id = "velodyne";
    box_car_pub.publish(box_car);

}

ObstacleDetection::~ObstacleDetection()
{
}

bool ObstacleDetection::speed_control(Box box_point,double speed_ratio[])
{

    speed_ratio[0] = speed_ratio[1]= speed_ratio[2] = 1.0;
    bool ret = false;

    // 前面车盖的角度
    double angle = atan2(car_width/2, car_length/2); //+  M_PI / 180 * 10;       //  M_PI / 180 * 10 表示加大一点前视角度
    double x_, y_, z_;

    x_ = abs(box_point.x_max) - abs(box_point.x_min) > 0 ? box_point.x_min : box_point.x_max;
    y_ = abs(box_point.y_max) - abs(box_point.y_min) > 0 ? box_point.y_min : box_point.y_max;
    z_ = abs(box_point.z_max) - abs(box_point.z_min) > 0 ? box_point.z_min : box_point.z_max;//该box离车体最近的点
    // box.dimensions.x = box_point.x_max - box_point.x_min;
    // box.dimensions.y = box_point.y_max - box_point.y_min;
    // box.dimensions.z = box_point.z_max - box_point.z_min;

    // 四周停止范围
    if (x_ > -(car_length / 2 + back_stop_distance) && y_ > -(car_width / 2 + edge_stop_distance) && 
        x_ < (car_length / 2 + front_stop_distance) && y_ < (car_width / 2 + edge_stop_distance)){
        speed_ratio[0] = 0;
        ret = true;
    } // 前方停止范围乘3倍进行减速
    if (x_ > -(car_length / 2 + back_stop_distance) && y_ > -(car_width / 2 + edge_stop_distance ) &&
             x_ < (car_length / 2 + front_stop_distance * 3) && y_ < (car_width / 2 + edge_stop_distance))
    {
        speed_ratio[1] = x_ - (car_length / 2 + front_stop_distance) / (front_stop_distance * 2); // 7 - 3 / 4, 减速比例 0~1
        //  speed_ratio[0] =  x_ - (car_length / 2)/  (front_stop_distance*3);  // 7 - 1 / 6                  , 减速比例 0.33~1

    } //侧方减速
    // if (x_ > -(car_length / 2 + back_stop_distance) && y_ > -(car_width / 2 + edge_stop_distance * 3) &&
    //          x_ < (car_length / 2 + front_stop_distance) && y_ < (car_width / 2 + edge_stop_distance * 3))
    // {
    //     speed_ratio[2] =  y_ - (car_width / 2 + edge_stop_distance) /  (edge_stop_distance * 2);
    //     // speed_ratio[0] = y_ - (car_width / 2) / (edge_stop_distance * 3);
    // }
    
#if 0
    if (-angle < atan2(y_,x_) && atan2(y_,x_) < angle)             // 前视角度   不能使用 1 < x < 2
    {
        if (((car_length / 2) + front_stop_distance) >= sqrt(pow(x_, 2) + pow(y_, 2)))
        {
            // 停止
            printf("--- 前方有障碍物\n");
            speed_ratio[0] = 0;
            ret = true;
        }
        else if (((car_length / 2) + front_stop_distance * 3) > sqrt(pow(x_, 2) + pow(y_, 2)))
        {
            // 减速
            //speed_ratio[0] = (sqrt(pow(x_, 2) + pow(y_, 2)) - (car_length / 2 + front_stop_distance)) / (front_stop_distance * 4); //控制速度的比例
            speed_ratio[0] = (sqrt(pow(x_, 2) + pow(y_, 2)) - (car_length / 2 )) / (front_stop_distance * 3);  // 100~33-0
        }else{
            //速度恢复
            speed_ratio[0] = 1;
        }
    }else if((-angle + M_PI) < (atan2(y_,x_) < 0?atan2(y_,x_)+M_PI*2:atan2(y_,x_)) && (atan2(y_,x_) < 0?atan2(y_,x_)+M_PI*2:atan2(y_,x_)) < (angle + M_PI))    // 后视角度 ，tan在第一、三象限为正，在第二、四为负
    {

        if(-(car_length / 2 + back_stop_distance) <= x_ && x_ <= (car_length / 2 + back_stop_distance))
        {
            printf("--- 后方有障碍物\n");
            speed_ratio[2] = 0;
            ret = true;
        }else if(-(car_length / 2 + back_stop_distance*3) < x_ && x_ < (car_length / 2 + back_stop_distance*3))
        {
            //speed_ratio[2] = (abs(x_) - (car_length / 2 + back_stop_distance)) / (back_stop_distance * 2); //控制速度的比例
            speed_ratio[2] = (abs(x_) - (car_length / 2 )) / (back_stop_distance * 3); 
        }else{
            speed_ratio[2] = 1;
        }
    }
    else // 侧边角度
    {

        if (-(car_width / 2 + edge_stop_distance) <= y_ && y_ <= (car_width / 2 + edge_stop_distance))
        {
            printf("--- 侧方有障碍物\n");
            speed_ratio[1] = 0;
            ret = true;
        }
        else if (-(car_width / 2 + edge_stop_distance * 3) < y_ && y_ < (car_width / 2 + edge_stop_distance * 3))
        {
            //speed_ratio[1] = (abs(y_) - (car_width / 2 + edge_stop_distance)) / (edge_stop_distance * 2); //控制速度的比例
            speed_ratio[1] = (abs(y_) - (car_width / 2))/ (edge_stop_distance * 3);
        }
        else
        {
            speed_ratio[1] = 1;
        }
    }
#endif
    return ret;
}

void ObstacleDetection::pub_speed_ratio(double min)
{
    //发布控制速度比例值
    std_msgs::Float64 mix_msg;
    min = floor(min * 100.000f + 0.5) / 100.000f;
    mix_msg.data = min;
    speed_ratio_pub.publish(mix_msg);
}

void ObstacleDetection::bounding_box(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters)
{
    auto startTime = chrono::high_resolution_clock::now();

    jsk_recognition_msgs::BoundingBox box;
    jsk_recognition_msgs::BoundingBoxArray boxs,boxs_ab;
    int i = 0;

    double min_speed=1.0;
    double min_speed_t;
    double speed_ratio[3];

    //for (int objectI = 0; objectI < visBBs.size(); objectI++)
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        Box box_point = pointProcessorI->BoundingBox(cluster); // 返回 最大和最小点的位置，即对角线的2点
        box.label = i + 1;
        #if 0
        //calculate bounding box
        double length_ = box_point.x_max - box_point.x_min;
        double width_ =  box_point.y_max - box_point.y_min;
        double height_ = box_point.z_max - box_point.z_min;

        box.pose.position.x = box_point.x_min + length_ / 2;
        box.pose.position.y = box_point.y_min + width_ / 2;
        box.pose.position.z = box_point.z_min + height_ / 2;
        box.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);   
        box.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        box.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);
        #else
        box.pose.position.x = (box_point.x_max + box_point.x_min) / 2;
        box.pose.position.y = (box_point.y_max + box_point.y_min) / 2;
        box.pose.position.z = (box_point.z_max + box_point.z_min) / 2;
        box.dimensions.x = box_point.x_max - box_point.x_min;
        box.dimensions.y = box_point.y_max - box_point.y_min;
        box.dimensions.z = box_point.z_max - box_point.z_min;
        #endif

        if (speed_control(box_point,speed_ratio) == true)
        {
            box.header.frame_id = frame_id;
            boxs_ab.boxes.push_back(box);
        }

        min_speed_t = speed_ratio[0] < speed_ratio[1] ? speed_ratio[0] : speed_ratio[1];     // 四周方向中选择一个最小值的发布
        min_speed_t = min_speed_t < speed_ratio[2] ? min_speed_t : speed_ratio[2];
        if(min_speed > min_speed_t){
            min_speed = min_speed_t;
        }

        box.header.frame_id = frame_id;
        //box.header.stamp = input_msg.header.stamp;
        boxs.boxes.push_back(box);
    }

    //boxs.header.stamp = input_msg.header.stamp;
    boxs.header.frame_id = frame_id;
    box_pub.publish(boxs);
    boxs_ab.header.frame_id = frame_id;
    box_pub_ab.publish(boxs_ab);

    pub_speed_ratio(min_speed);

    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double, milli> fp_ms = endTime - startTime;
    cout << fp_ms.count() << endl;
}

void ObstacleDetection::cloud_callback(const sensor_msgs::PointCloud2ConstPtr input)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr elevatedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Convert from ros msg to PCL::PointCloud data type
    pcl::fromROSMsg(*input, cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered ;
    if (lidar_type == "RSHELIOS" ){
        cloudFiltered = pointProcessorI->FilterCloud(cloud.makeShared(), 0.3, Eigen::Vector4f(-10, -5, -1.3, 1), Eigen::Vector4f(20, 6, 1, 1), Eigen::Vector4f(-(car_length / 2), -(car_width / 2), -car_hight, 1.0), Eigen::Vector4f(car_length / 2, car_width / 2, 0, 1.0));
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);

        cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500); // 0.53米内表示是同一个团族, 10<x<500的团集认为是一个障碍物
    }else if(lidar_type == "C16"){
        cloudFiltered = pointProcessorI->FilterCloud(cloud.makeShared(), 0.1, Eigen::Vector4f(-20, -10, -1.45, 1.0), Eigen::Vector4f(20, 10, 1, 1.0), Eigen::Vector4f(-(car_length / 2), -(car_width / 2), -car_hight, 1.0), Eigen::Vector4f(car_length / 2, car_width / 2, 0, 1.0));
        //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);
        cloudClusters = pointProcessorI->Clustering(cloudFiltered, 0.3, 3, 500); // 0.3米内表示是同一个团族, 3<x<500的团集认为是一个障碍物
    }

    int clusterId = 0;

    bounding_box(cloudClusters);
    // ---------------------------------------------------
    //发布处理后的点云
    pcl::toROSMsg(*cloudFiltered, output);
    output.header.frame_id = frame_id;
    output.header.seq = input->header.seq;
    output.header.stamp =  ros::Time::now();  //input->header.stamp;      //时间需要同步

    cloud_pub.publish(output);
}

// 循环发布话题
bool ObstacleDetection::pool()
{
    sensor_msgs::PointCloud2 output;

    //1 发布读取pcd文件的点云话题
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string()); // 加载pcd点云文件
    streamIterator++;
    if (streamIterator == stream.end())
        streamIterator = stream.begin();
    pcl::toROSMsg(*inputCloudI, output);
    output.header.frame_id = frame_id;
    raw_cloud_pub.publish(output);

    return true;
}
