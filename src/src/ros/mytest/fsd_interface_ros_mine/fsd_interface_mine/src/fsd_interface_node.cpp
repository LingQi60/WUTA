#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <typeinfo>
#include <vector>
#include <std_msgs/String.h>
#include <math.h>

#include "fsd_common_msgs/Cone.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "sensor_msgs/Imu.h"
using namespace std;
ros::Subscriber objectsSubscriber_;
ros::Publisher mappingPublisher_;
ros::Subscriber imuSubscriber_;
ros::Publisher statePublisher_;
int seq = 0;
double last_time_x = 0;
double last_time_y = 0;
double last_time_yaw = 0;
double last_time_x_velocity = 0;
double last_time_y_velocity = 0;
double last_time_yaw_velocity = 0;
double last_time_stamp = 0;

fsd_common_msgs::CarState getCar_state(const sensor_msgs::Imu &imu)
{
    float theta;
    theta = asin(2*(imu.orientation.w * imu.orientation.y - imu.orientation.z * imu.orientation.x));
    fsd_common_msgs::CarState carstate;

    if(seq == 0)
    {
        last_time_stamp = imu.header.stamp.toSec();
        
    }
    else
    {
        carstate.header = imu.header;
        carstate.car_state.theta = theta;
        carstate.car_state.x = imu.orientation.x;
        carstate.car_state.y = imu.orientation.y;

        carstate.car_state_dt.header = imu.header;
        carstate.car_state_dt.car_state_a.theta = (carstate.car_state.theta - last_time_yaw) /
         (imu.header.stamp.toSec() - last_time_stamp);
        carstate.car_state_dt.car_state_a.x = (carstate.car_state.x - last_time_x) /
         (imu.header.stamp.toSec() - last_time_stamp);
        carstate.car_state_dt.car_state_a.y = (carstate.car_state.y - last_time_y) / 
        (imu.header.stamp.toSec() - last_time_stamp);
        carstate.car_state_dt.car_state_dt.theta = (carstate.car_state_dt.car_state_a.theta - last_time_yaw_velocity) / 
        (imu.header.stamp.toSec() - last_time_stamp);
        carstate.car_state_dt.car_state_dt.x = (carstate.car_state_dt.car_state_a.x - last_time_x_velocity) / 
        (imu.header.stamp.toSec() - last_time_stamp);
        carstate.car_state_dt.car_state_dt.y = (carstate.car_state_dt.car_state_a.y - last_time_y_velocity) / 
        (imu.header.stamp.toSec() - last_time_stamp);

        // cout<<"\n"<<carstate.car_state_dt.car_state_a.theta;

        last_time_yaw  = carstate.car_state.theta;
        last_time_x = carstate.car_state.x;
        last_time_y = carstate.car_state.y; 
        last_time_yaw_velocity = carstate.car_state_dt.car_state_a.theta;
        last_time_x_velocity = carstate.car_state_dt.car_state_a.x;
        last_time_y_velocity = carstate.car_state_dt.car_state_a.y;
        last_time_stamp = imu.header.stamp.toSec();
    }

    return carstate;
}

void imucallback(const sensor_msgs::Imu &imu)
{
    
    statePublisher_.publish(getCar_state(imu));
}



int main(int argc, char** argv) 
{
    ros::init(argc, argv, "fsd_interface");
    ros::NodeHandle nodeHandle_("");

    std::string ObjectsTopicName;
    int ObjectsQueueSize;
    std::string MappingTopicName;
    int MappingQueueSize;
    bool MappingLatch;

    std::string ImuTopicName;
    int ImuQueueSize;
    std::string StateTopicName;
    int StateQueueSize;
    bool StateLatch;
    nodeHandle_.param("subscribers/imu/topic", ImuTopicName, std::string("/imu_raw"));
    nodeHandle_.param("subscribers/imu/queue_size", ImuQueueSize, 1);


    nodeHandle_.param("publishers/state/topic", StateTopicName, std::string("/estimation/slam/state"));
    nodeHandle_.param("publishers/state/queue_size", StateQueueSize, 1);
    nodeHandle_.param("publishers/state/latch", StateLatch, true);

    imuSubscriber_ = nodeHandle_.subscribe(ImuTopicName, ImuQueueSize, imucallback);
    statePublisher_  = nodeHandle_.advertise<fsd_common_msgs::CarState>(StateTopicName,
                        StateQueueSize,StateLatch);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        seq++;
        loop_rate.sleep();
    }
    return 0;
}

