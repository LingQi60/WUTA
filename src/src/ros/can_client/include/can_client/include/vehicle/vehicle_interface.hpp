////////////////  1   ////////////////

#ifndef _VEHICLE_INTERFACE_H_
#define _VEHICLE_INTERFACE_H_

#include <cstdio>
#include <stdint.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_can_msgs/CANPacket.h>

class VehicleInferface
{
public:
    VehicleInferface(){};
    ~VehicleInferface(){};

    virtual void vehicle_cmd(autoware_msgs::VehicleCmd msg) = 0;
    virtual void vehicle_static(autoware_can_msgs::CANPacket msg) = 0;
    virtual void run() = 0;
};
#endif