#include <ros/ros.h>
#include "std_msgs/String.h"
// #include "union_detect_msgs/Object.h"
// #include "union_detect_msgs/Objects.h"
using namespace std;


// void callback(const union_detect_msgs::Objects &Objects_Result)
// {   

//     // cout<<"\nI am listener ,Objects_Result.object.at(0) = "<<Objects_Result.object.at(0);
// }
int main(int argc,char **argv)
{
    ros::init(argc,argv,"listener");
    ros::NodeHandle n;
//     ros::Subscriber sub = n.subscribe("/union_detect/union_detect_result",1,callback);
//     ros::spin();
    return 0;
}