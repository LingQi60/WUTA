#ifndef _SIMWULING_H_
#define _SIMWULING_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "vehicle/vehicle_interface.hpp"
#include "vehicle/wuling_lib/canbase.hpp"
// ros 头文件
#include <autoware_msgs/Gear.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_can_msgs/CANPacket.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <autoware_msgs/Vehicle_Ctrl_Status.h>
#include <queue>

class SimWulingVehicle : public VehicleInferface
{
public:
    SimWulingVehicle();
    ~SimWulingVehicle();

    void vehicle_cmd(autoware_msgs::VehicleCmd msg);
    void vehicle_static(autoware_can_msgs::CANPacket msg);
    void run();

private:
    // ros 相关变量
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher can_info_pub_, can_box_pub_, real_velocity_pub_, twist_pub_;
    ros::Subscriber current_velocity_sub_,spd_ratio_sub;

    autoware_can_msgs::CANInfo can_info_;
    autoware_msgs::VehicleCmd old_m;
    double maxsteerangle;
    double speed_ratio;
    bool m_aSafeDir[2];
    double add_speed_value;
    double speed_gain;
    int rotate_speed_;
    int _cnt;
    uint8_t turn_val,horn_val,beam_val;
    uint8_t set_mode;
    bool dev_is_ok;
    double base_brake_value;
    
    void callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr& msg);
    void callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    
    struct PidStatus
    {
        double SetSpeed;           //定义设定值
        double ActualSpeed;        //定义实际值
        double err;               //定义偏差值
        double err_next;           //定义上一个偏差值
        double err_last;           //定义最上前的偏差值
        double Kp,Ki,Kd;           //定义比例、积分、微分系数

    };


    struct PidStatus2
    {
        double SetSpeed;           //定义设定值
        double ActualSpeed;        //定义实际值
        double err;               //定义偏差值
        double err_next;           //定义上一个偏差值
        double err_last;           //定义最上前的偏差值
        double Kp,Ki,Kd;           //定义比例、积分、微分系数

    };

    std::queue<int> angle_;
    bool geer_dir;

    void fuzzy_PIDINT();
    double pid_break(double obj_spd,double current_spd);
    float Speed_FilterOut(float target);

    enum VehicleDataType
    {
        Mode_Cmd=0x100,
        Acc_Cmd = 0x101,
        Brake_Cmd = 0x102,
        Steer_Cmd = 0x103,
        Gear_Cmd = 0x104,

        Mode_fbk = 0x200,
        Acc_fbk = 0x201,
        Brake_fbk = 0x202,
        Steer_fbk = 0x203,
        Gear_fbk = 0x204,
        Speed_fbk = 0x109A,
    };

    enum GearShift
    {
        Parking = 1,
        Neutral = 3,
        Drive = 4,
        Reverse = 2,
    };

    bool setMode(int32_t en);
    void setAccPercent(int32_t value, bool en);
    void setBrakePercent(int32_t value, bool en);
    void setSteerPostion(int32_t value, bool en);
    void setGear(uint8_t valude, bool en);
    void setTurnLight(uint8_t valude);
    void setBeamLight(uint8_t valude);
    void setBeep(uint8_t valude);
    void setFPB(uint8_t valude, bool en);
    void Vehicle_Can_syne();


    void getMode(uint8_t *data);
    void getAcc( uint8_t *data);
    void getBrake( uint8_t *data);
    void getSteer(uint8_t *data);
    void getGear(uint8_t *value);
    void getSpeed(uint8_t *data);

    void getSpeed(uint16_t value);
    void getTurnLight(uint8_t value);
    void getBeamLight(uint8_t value);
    void getBeep(uint8_t value);
    bool check_ok(uint8_t *data);


    static void *set_auto_mode(void *arg);

    void increaseCNT();  //  计数函数
    uint8_t calculateCtrlMSGCheckSum(void* canFrame); //校验合函数
};

#endif