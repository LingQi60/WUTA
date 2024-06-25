
#ifndef ASTAR_NAVI_H
#define ASTAR_NAVI_H

#include <iostream>
#include <vector>

#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include "common/can_box_interface.hpp"
#include "can_box/can_socket.hpp"
// ros 头文件
#include <ros/ros.h>

#include "autoware_msgs/VehicleCmd.h"
#include <autoware_can_msgs/CANPacket.h>
#include <autoware_can_msgs/CANDataList.h>

class CanConnect
{
public:
  CanConnect();
  ~CanConnect();
  void run();

private:
  //1 ros 相关变量
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher  can_pub_;
  ros::Publisher  can_raw_pub_;
  ros::Subscriber can_sub_;

  //2 ros callback
  void vehicleCanWriteCallback(const autoware_can_msgs::CANPacket &msg);

  // 3 ros params 获取参数服务器值
  double update_rate_;
  double can_port_;

  //4 ros msg variables
  // autoware_can_msgs::CANInfo can_msg_;


  //5 mist variables
  // using namespace std;
  int mode;
  int client_sock_;

  //5.1 struct 结构体

  //6 classes 组合
  //  AstarSearch astar_;
  ICanDevInferface *ican_ctrl;

  //7 fucntions
  bool socket_register();
  static void *getCanValue(void *arg);
  // bool parseCanValue(uint8_t *_can_data,SensorType &type);
  void publishCanData(uint8_t *data);

  static void *recvCandata(void *arg);
};

#endif
