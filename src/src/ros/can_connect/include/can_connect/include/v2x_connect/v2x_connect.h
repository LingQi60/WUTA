
#ifndef ASTAR_NAVI_H
#define ASTAR_NAVI_H

#include <iostream>
#include <vector>

#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include "can_box/socket_client.hpp"
// ros 头文件
#include <ros/ros.h>

#include "autoware_msgs/VehicleCmd.h"
#include <autoware_can_msgs/CANPacket.h>

class V2xConnect 
{
public:
  V2xConnect();
  ~V2xConnect();
  void run();

private:
  //1 ros 相关变量
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher  v2x_pub_;
  ros::Subscriber v2x_sub_;

  //2 ros callback
  void v2xWriteCallback(const autoware_can_msgs::CANPacket &msg);

  // 3 ros params 获取参数服务器值
  double update_rate_;
  double v2x_port_;
  std::string v2x_ip_;

  //4 ros msg variables
  // autoware_can_msgs::CANInfo v2x_msg_;


  //5 mist variables
  // using namespace std;
  int mode;
  int client_sock_;

  //5.1 struct 结构体

  //6 classes 组合
  //  AstarSearch astar_;
  ICanDevInferface *iv2x_ctrl;

  //7 fucntions
  bool socket_register();
  static void *getV2xValue(void *arg);
  // bool parseV2xValue(uint8_t *_v2x_data,SensorType &type);
  void publishV2xData(uint8_t *data);

  static void *recvV2xdata(void *arg);
};

#endif
