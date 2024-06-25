#ifndef _YUNLE_H_
#define _YUNLE_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "vehicle/vehicle_interface.hpp"
#include "vehicle/yunle_lib/canbase.hpp"
// ros 头文件
#include <autoware_msgs/Gear.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/Lane.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_can_msgs/CANPacket.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/BatteryState.h>
#include <autoware_msgs/Vehicle_Ctrl_Status.h>

class YunleVehicle : public VehicleInferface
{
public:
    YunleVehicle();
    ~YunleVehicle();


    void vehicle_cmd(autoware_msgs::VehicleCmd msg);
    void vehicle_static(autoware_can_msgs::CANPacket msg);
    void run();

private:
    // ros 相关变量
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher can_info_pub_, can_box_pub_, real_velocity_pub_;
    ros::Publisher batter_state_pub_;
    ros::Subscriber current_velocity_sub_, spd_ratio_sub, sub_final_waypoints_, sub_Trajectory_Cost, sub_GoalRemainingDistance;

    autoware_can_msgs::CANInfo can_info_;
    double maxsteerangle;
    double maxdistancetoavoid;
    double speed_ratio;
    double speed_slow[4]={1,-1,1,1};
    bool m_aSafeDir[2];
    double add_speed_value;
    double speed_gain;
    double base_brake_value;
    bool gear_reverse_on;
    bool backward_on;
    double back_distance;
    bool follow_on;
    int m_fRemainingDistance;

    int m_nTargetNum;
    int gear_mode;
    int dev_mode = 0;
	bool enable_recover;
    bool bench_debug_value;
    double stop_brake_increase;
    double normal_brake_increase;

    // uint8_t turn_val,horn_val,beam_val;
    // uint8_t set_mode;
    // bool dev_is_ok;
    // double base_brake_value;

    void callbackSpeedRatio(const autoware_msgs::Vehicle_Ctrl_StatusConstPtr &msg);
    void callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    void callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints);
    void callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr &msg);
    void callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg);


    void fuzzy_PIDINT();
    double pid_break(double obj_spd, double current_spd);
    float Speed_FilterOut(float target);
    bool node_state(void);//节点状态判断函数

    
    ros::V_string node_ultrasound;

    enum VehicleDataType
    {
        Gear_Cmd = 0xA1,
        Steer_Cmd = 0xA2,
        Drive_Cmd = 0xA3,
        Brake_Cmd = 0xA4,
        Parking_Cmd = 0xA5,
        Vcu_fbk = 0xC1,
        Bms_fbk = 0xC2,
        Enable_fbk = 0xC3,
        Err_fbk = 0xE1,
    };

    enum GearShift
    {
        Drive = 1,
        Neutral = 2,
        Reverse = 3,
    };

    enum CtrlMode
    {
        Normal = 1,
        Recover = 2,
        FollowingDistance = 3,
        Forward = 4,
    };
    int ctrl_mode;

    // 设置驾驶模式
    bool setMode(int32_t en);

    // 设置驱动百分比
    void setAccPercent(int32_t value, bool en);
    void setBrakePercent(int32_t value, bool en);

    // 设置转向角
    void setSteerPostion(int32_t value, bool en);
    void setSteerPostionWithLimit(int32_t value, int32_t limitAngle, int32_t limitSpdAngle);

    // 设置SCU数据
    void sendScuData(uint8_t Drive_Mode, uint8_t shiftLevel, float target, float steering, int16_t brake);
    // 设置杂项
    void setMist();
    // 设置档位
    void setGear(uint8_t valude, bool en);
    // 设置转向灯
    void setTurnLight(uint8_t valude);
    // 设置大灯
    void setBeamLight(uint8_t valude);
    // 设置喇叭
    void setBeep(uint8_t valude);
    // 设置手刹
    void setFPB(uint8_t valude, bool en);

    void getVcu_fbk(uint8_t *value);
    void getBms_fbk(uint8_t *value);
    void getEnable_fbk(uint8_t *value);
    void getErr_fbk(uint8_t *value);
    void getMist(uint8_t *value);
    bool check_ok(uint8_t *data);
    void getMode(uint8_t value);
    void getAcc(uint16_t value);
    void getBrake(uint16_t value);
    void getSteer(uint16_t value);
    void getGear(uint8_t value);
    void getTurnLight(uint8_t value);
    void getBeamLight(uint8_t value);
    void getBeep(uint8_t value);
    void getSpeed(uint16_t value);

};

#endif