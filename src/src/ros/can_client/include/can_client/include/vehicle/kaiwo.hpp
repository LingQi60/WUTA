#ifndef __KAIWO_H_
#define __KAIWO_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "vehicle/vehicle_interface.hpp"
#include "vehicle/kaiwo_lib/canbase.hpp"
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
#include <std_msgs/Bool.h>
#include <autoware_msgs/Vehicle_Ctrl_Status.h>
#include "vehicle/pid_controller.hpp"

#include <autoware_system_msgs/NodeStatus.h>

#include <numeric>


class KaiWo : public VehicleInferface
{
public:
    KaiWo();
    ~KaiWo();

    void vehicle_cmd(autoware_msgs::VehicleCmd msg);
    void vehicle_static(autoware_can_msgs::CANPacket msg);
    void run();

private:
    // ros 相关变量
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher can_info_pub_, can_box_pub_, real_velocity_pub_,goal_on_pub_;
    ros::Subscriber current_velocity_sub_;
    ros::Subscriber sub_current_behavior;
    ros::Subscriber sub_GoalRemainingDistance;
    ros::Subscriber brake_mode_sub_;

    enum STATE_TYPE {INITIAL_STATE, WAITING_STATE, FORWARD_STATE, STOPPING_STATE, EMERGENCY_STATE,
                    TRAFFIC_LIGHT_STOP_STATE,TRAFFIC_LIGHT_WAIT_STATE, STOP_SIGN_STOP_STATE, STOP_SIGN_WAIT_STATE, FOLLOW_STATE, 
                    LANE_CHANGE_STATE, OBSTACLE_AVOIDANCE_STATE, GOAL_STATE, FINISH_STATE, YIELDING_STATE,
                    BRANCH_LEFT_STATE, BRANCH_RIGHT_STATE,PULL_OVER};

    autoware_can_msgs::CANInfo can_info_;
    double maxsteerangle;
    double maxdistancetoavoid;
    double minFollowingDistance;
    double maxvelocity;
    double m_rollOuts_number;
    double speed_gain;
    uint8_t _cnt;

    uint8_t set_mode;
    bool dev_is_ok;
    bool follow_on;
    
    bool b_local_status;
    int m_brake_mode;

    int driver_state;
    int m_fRemainingDistance;

    void callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    void can_box_publish(int id, int len, uint8_t* data);
    void callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr & msg);
    void callbackGetGoalRemainingDistance(const std_msgs::Float32ConstPtr &msg);
    void brakeNodeCallback(const std_msgs::Float32::ConstPtr & msg);

    double m_object_speed;
    std::vector<float_t> steer_val;

    enum VehicleCanId
    {
        EPS_ID_W = 0x1801B0A0,
        SPEED_ID_W = 0x1803B0A0,
        CTR1_ID_W = 0x1805B0A0,
        CTR2_ID_W = 0x1807B0A0,

        EPS_ID_R = 0x1802A0B0,
        DRIVING_ID_R = 0x1804A0B0,
        STATE1_ID_R = 0x1806A0B0,
        STATE5_ID_R = 0x1808A0B0,
    };

    enum GearShift
    {
        PARK = 9,
        Neutral = 0,
        Drive = 1,
        Reverse = 2,
    };

    // ---------- set 
    uint8_t door_val;
    bool door_val_flag;
    // uint8_t clew_tone_type;
    bool clew_tone_flag;

    // int (根据can数据定义类型)
    struct SetVehicleData
    {
        uint8_t mode;
        int16_t acc;
        int16_t brake;
        int32_t steer;
        uint8_t gear;
        uint8_t turn_left;
        uint8_t turn_right;
        uint8_t epb;
        uint8_t beam;
        uint8_t beep;
        uint8_t door;
    };

    SetVehicleData m_vehicle_data;


	// 根据协议定义的封装数据
    void set_vehicle_data(autoware_msgs::VehicleCmd m_data);
    void set_auto_eps();
    void set_auto_speed();
    void set_auto_ctr_1();
    void set_auto_ctr_2();


	// 通用封装api
    void setMode(uint8_t value);
    void setAccPercent(int16_t value);
    void setBrakePercent(int16_t value);
    void setSteerPostion(int32_t value);
    void setGear(uint8_t value);
    void setTurnLight(uint8_t value_l, uint8_t value_r);
    void setBeamLight(uint8_t value);
    void setBeep(uint8_t value);
    void setEPB(uint8_t value);
    void setDoor(uint8_t value);
    void Vehicle_Can_syne();

    // ------------ get
    // double (根据msg封装类型)
    struct GetVehicleInfo
    {
        int32_t mode;
        int32_t brake;
        int32_t acc;
        double  steer;
        int32_t gear;
        int32_t epb;
        int32_t turn_left;
        int32_t turn_right;
        int32_t door;
        int32_t avas;
        int32_t beam;
        int32_t beep;
        double  speed;

    };
    GetVehicleInfo m_vehicle_info;
    control::PIDController* speed_pid_controller;
	
	// 根据协议定义的解析数据
    void get_vehicle_data(GetVehicleInfo vinfo);
    void get_vehicle_eps(uint8_t *data);
    void get_vehicle_driving(uint8_t *data);
    void get_vehicle_state_1(uint8_t *data);
    void get_vehicle_state_5(uint8_t *data);

	// 通用解析api
    void getMode(uint8_t value);
    void getAcc( int16_t value);
    void getBrake(int16_t value);
    void getSteer(int32_t value);
    void getGear(uint8_t value);

    void getEpb(uint8_t value);
    void getSpeed(int16_t value);
    void getTurnLight(uint8_t value_l,uint8_t value_r);
    void getBeamLight(uint8_t value);
    void getBeep(uint8_t value);
    void getAvas(uint8_t value);
    void getDoor(uint8_t value);
    void getDoorButton(uint8_t value);

    bool check_ok(uint8_t *data);
    
    static void *set_auto_mode(void *arg);

    void increaseCNT();  //  计数函数
    uint8_t calculateCtrlMSGCheckSum(void* canFrame); //校验和函数
    uint8_t calculateCtrlMSGCheckXor(void* canFrame); //校验异或函数

    double m_acceleration_cmd;
};

#endif