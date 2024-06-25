
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

#pragma once


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// This file must define:
// base monitor struct
// function signature for CRC calculation
// function signature for getting system tick value (100 us step)
// #include "canmonitorutil.h"

// def @vehicle_vin CAN Message (0)
#define vehicle_vin_IDE (0U)
#define vehicle_vin_DLC (8U)
#define vehicle_vin_CANID (0x0U)
typedef struct
{
} vehicle_vin_t;

// def @Gear_Shift_Cmd CAN Message (161)
#define Gear_Shift_Cmd_IDE (0U)
#define Gear_Shift_Cmd_DLC (8U)
#define Gear_Shift_Cmd_CANID (0xA1U)
typedef struct
{

  // 1 - "gear_autocontrol" 
  // 0 - "gear_manualcontrol" 

  uint8_t Gear_Enable_control;              //      Bits=08.  [ 0     , 1      ]  Unit:''     

  // 4 - "R" 
  // 3 - "D" 
  // 2 - "N" 
  // 1 - "P" 
  // 0 - "invalid" 

  uint8_t Gear_Shift_Req;                   //      Bits=08.  [ 0     , 5      ]  Unit:''     

  // 1 - "IPC_Mode" 
  // 0 - "ManMode" 

  uint8_t IPC_Mode_Shift;                   //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint8_t IPC_Stop_Eme;                     //      Bits=08.  [ 0     , 1      ]  Unit:''     
} Gear_Shift_Cmd_t;

// def @steering_control CAN Message (162)
#define steering_control_IDE (0U)
#define steering_control_DLC (8U)
#define steering_control_CANID (0xA2U)
// signal: @Steering_Pos_Req
#define steering_control_Steering_Pos_Req_CovFactor (0.0008545)
// conversion value to CAN signal
#define steering_control_Steering_Pos_Req_toS(x) ((int32_t)((x) / 0.0008545 + 32767))
// conversion value from CAN signal
#define steering_control_Steering_Pos_Req_fromS(x) ((x) * 0.0008545)

typedef struct
{

  uint8_t Steering_Enable_control;          //      Bits=08.  [ 0     , 1      ]  Unit:''     

  int32_t Steering_Pos_Req;                 //      Bits=16.  [ -28   , 27.9996575 ]  Unit:'deg'   Offset= -28       Factor= 0.0008545
} steering_control_t;

// def @Drive_control CAN Message (163)
#define Drive_control_IDE (0U)
#define Drive_control_DLC (8U)
#define Drive_control_CANID (0xA3U)
// signal: @Drive_Tq_Req
#define Drive_control_Drive_Tq_Req_CovFactor (0.001525)
// conversion value to CAN signal
#define Drive_control_Drive_Tq_Req_toS(x) ((uint16_t)((x) / 0.001525))
// conversion value from CAN signal
#define Drive_control_Drive_Tq_Req_fromS(x) ((x) * 0.001525)

typedef struct
{

  // 1 - "drive_auto" 
  // 0 - "drive_manual" 

  uint8_t Driven_Enable_control;            //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint16_t Drive_Tq_Req;                    //      Bits=16.  [ 0     , 99.940875 ]  Unit:'%'     Factor= 0.001525
} Drive_control_t;

// def @brake_control CAN Message (164)
#define brake_control_IDE (0U)
#define brake_control_DLC (8U)
#define brake_control_CANID (0xA4U)
// signal: @Brake_Tq_Req
#define brake_control_Brake_Tq_Req_CovFactor (0.001525)
// conversion value to CAN signal
#define brake_control_Brake_Tq_Req_toS(x) ((uint16_t)((x) / 0.001525))
// conversion value from CAN signal
#define brake_control_Brake_Tq_Req_fromS(x) ((x) * 0.001525)

typedef struct
{

  // 1 - "brake_auto" 
  // 0 - "brake_manual" 

  uint8_t Brake_Enable_control;             //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint16_t Brake_Tq_Req;                    //      Bits=16.  [ 0     , 100.26855 ]  Unit:'%'     Factor= 0.001525
} brake_control_t;

// def @Parking_control CAN Message (165)
#define Parking_control_IDE (0U)
#define Parking_control_DLC (8U)
#define Parking_control_CANID (0xA5U)
typedef struct
{

  // 1 - "Parking_autocontrol" 
  // 0 - "Parking_manualcontrol" 

  uint8_t Parking_Enable_control;           //      Bits=08.  [ 0     , 1      ]  Unit:''     

  // 1 - "parking_trigger" 
  // 0 - "release" 

  uint8_t Parking_Cmd;                      //      Bits=08.  [ 0     , 1      ]  Unit:''     
} Parking_control_t;

// def @VCU_FeedBack CAN Message (193)
#define VCU_FeedBack_IDE (0U)
#define VCU_FeedBack_DLC (8U)
#define VCU_FeedBack_CANID (0xC1U)
// signal: @Vehicle_Spd
#define VCU_FeedBack_Vehicle_Spd_CovFactor (0.1)
// conversion value to CAN signal
#define VCU_FeedBack_Vehicle_Spd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define VCU_FeedBack_Vehicle_Spd_fromS(x) ((x) * 0.1)

// signal: @Steering_Ang
#define VCU_FeedBack_Steering_Ang_CovFactor (0.0008545)
// conversion value to CAN signal
#define VCU_FeedBack_Steering_Ang_toS(x) ((int32_t)((x) / 0.0008545 + 32767))
// conversion value from CAN signal
#define VCU_FeedBack_Steering_Ang_fromS(x) ((x) * 0.0008545)

// signal: @Brake_Tq
#define VCU_FeedBack_Brake_Tq_CovFactor (0.19607)
// conversion value to CAN signal
#define VCU_FeedBack_Brake_Tq_toS(x) ((uint8_t)((x) / 0.19607))
// conversion value from CAN signal
#define VCU_FeedBack_Brake_Tq_fromS(x) ((x) * 0.19607)

// signal: @Brake_pedal_Position
#define VCU_FeedBack_Brake_pedal_Position_CovFactor (0.39215)
// conversion value to CAN signal
#define VCU_FeedBack_Brake_pedal_Position_toS(x) ((uint8_t)((x) / 0.39215))
// conversion value from CAN signal
#define VCU_FeedBack_Brake_pedal_Position_fromS(x) ((x) * 0.39215)

typedef struct
{

  uint16_t Vehicle_Spd;                     //      Bits=16.  [ 0     , 200    ]  Unit:'kph'   Factor= 0.1   

  int32_t Steering_Ang;                     //      Bits=16.  [ -28   , 27.9996575 ]  Unit:'deg'   Offset= -28       Factor= 0.0008545

  // 0 - "invalid" 
  // 4 - "R" 
  // 3 - "D" 
  // 2 - "N" 
  // 1 - "P" 

  uint8_t Gear_Pos;                         //      Bits=08.  [ 0     , 5      ]  Unit:''     

  uint8_t Brake_Tq;                         //      Bits=08.  [ 0     , 49.99785 ]  Unit:'nm'    Factor= 0.19607

  uint8_t Brake_pedal_Position;             //      Bits=08.  [ 0     , 0      ]  Unit:'%'     Factor= 0.39215

  uint8_t CheckSum_Byte;                    //      Bits=08.  [ 0     , 255    ]  Unit:''     
} VCU_FeedBack_t;

// def @BMS_FeedBack CAN Message (194)
#define BMS_FeedBack_IDE (0U)
#define BMS_FeedBack_DLC (8U)
#define BMS_FeedBack_CANID (0xC2U)
// signal: @Vol
#define BMS_FeedBack_Vol_CovFactor (0.1)
// conversion value to CAN signal
#define BMS_FeedBack_Vol_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define BMS_FeedBack_Vol_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t Soc;                              //      Bits=08.  [ 0     , 100    ]  Unit:'%'    

  uint16_t Vol;                             //      Bits=16.  [ 0     , 55     ]  Unit:'V'     Factor= 0.1   

  uint8_t Current;                          //      Bits=08.  [ 0     , 120    ]  Unit:'A'    

  uint8_t CheckSum_Byte1;                   //      Bits=08.  [ 0     , 255    ]  Unit:''     
} BMS_FeedBack_t;

// def @Enable_Fbk CAN Message (195)
#define Enable_Fbk_IDE (0U)
#define Enable_Fbk_DLC (8U)
#define Enable_Fbk_CANID (0xC3U)
typedef struct
{

  // 1 - "gear_autocontrol" 
  // 0 - "gear_manualcontrol" 

  uint8_t Gear_Enb_fbk;                     //      Bits=08.  [ 0     , 1      ]  Unit:''     

  // 1 - "steering_autocontrol" 
  // 0 - "steering_manualcontrol" 

  uint8_t Steering_Enb_fbk;                 //      Bits=08.  [ 0     , 1      ]  Unit:''     

  // 1 - "drive_auto " 
  // 0 - "drive_manual" 

  uint8_t Drive_Enb_fbk;                    //      Bits=08.  [ 0     , 1      ]  Unit:''     

  // 1 - "brake_auto " 
  // 0 - "brake_manual" 

  uint8_t Brake_Enb_fbk;                    //      Bits=08.  [ 0     , 1      ]  Unit:''     

  // 1 - "brake_auto " 
  // 0 - "brake_manual" 

  uint8_t Parking_Enb_fbk;                  //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint8_t RC_Takerover_Flg;                 //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint8_t CheckSum_Byte2;                   //      Bits=08.  [ 0     , 1      ]  Unit:''     
} Enable_Fbk_t;

// def @FourWheel_Spd_Fbk CAN Message (196)
#define FourWheel_Spd_Fbk_IDE (0U)
#define FourWheel_Spd_Fbk_DLC (8U)
#define FourWheel_Spd_Fbk_CANID (0xC4U)
// signal: @Fl_Wheel_Spd
#define FourWheel_Spd_Fbk_Fl_Wheel_Spd_CovFactor (2)
// conversion value to CAN signal
#define FourWheel_Spd_Fbk_Fl_Wheel_Spd_toS(x) ((uint8_t)((x) / 2))
// conversion value from CAN signal
#define FourWheel_Spd_Fbk_Fl_Wheel_Spd_fromS(x) ((x) * 2)

// signal: @Rl_Wheel_Spd
#define FourWheel_Spd_Fbk_Rl_Wheel_Spd_CovFactor (2)
// conversion value to CAN signal
#define FourWheel_Spd_Fbk_Rl_Wheel_Spd_toS(x) ((uint8_t)((x) / 2))
// conversion value from CAN signal
#define FourWheel_Spd_Fbk_Rl_Wheel_Spd_fromS(x) ((x) * 2)

// signal: @Fr_Wheel_Spd
#define FourWheel_Spd_Fbk_Fr_Wheel_Spd_CovFactor (2)
// conversion value to CAN signal
#define FourWheel_Spd_Fbk_Fr_Wheel_Spd_toS(x) ((uint8_t)((x) / 2))
// conversion value from CAN signal
#define FourWheel_Spd_Fbk_Fr_Wheel_Spd_fromS(x) ((x) * 2)

// signal: @Rr_Wheel_spd
#define FourWheel_Spd_Fbk_Rr_Wheel_spd_CovFactor (2)
// conversion value to CAN signal
#define FourWheel_Spd_Fbk_Rr_Wheel_spd_toS(x) ((uint8_t)((x) / 2))
// conversion value from CAN signal
#define FourWheel_Spd_Fbk_Rr_Wheel_spd_fromS(x) ((x) * 2)

typedef struct
{

  uint8_t Fl_Spd_Valid;                     //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint8_t Fl_Wheel_Spd;                     //      Bits=08.  [ 0     , 250    ]  Unit:''      Factor= 2     

  uint8_t Rl_Spd_Valid;                     //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint8_t Rl_Wheel_Spd;                     //      Bits=08.  [ 0     , 250    ]  Unit:''      Factor= 2     

  uint8_t Fr_Spd_Valid;                     //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint8_t Fr_Wheel_Spd;                     //      Bits=08.  [ 0     , 250    ]  Unit:''      Factor= 2     

  uint8_t Rr_Spd_Valid;                     //      Bits=08.  [ 0     , 1      ]  Unit:''     

  uint8_t Rr_Wheel_spd;                     //      Bits=08.  [ 0     , 250    ]  Unit:''      Factor= 2     
} FourWheel_Spd_Fbk_t;

// def @Motor_Spd_Fbk CAN Message (197)
#define Motor_Spd_Fbk_IDE (0U)
#define Motor_Spd_Fbk_DLC (8U)
#define Motor_Spd_Fbk_CANID (0xC5U)
// signal: @Motor_spd
#define Motor_Spd_Fbk_Motor_spd_CovFactor (1)
// conversion value to CAN signal
#define Motor_Spd_Fbk_Motor_spd_toS(x) ((int32_t)((x) + 10000))
// conversion value from CAN signal
#define Motor_Spd_Fbk_Motor_spd_fromS(x) ((x))

typedef struct
{

  int32_t Motor_spd;                        //      Bits=16.  [ -6000 , 6000   ]  Unit:''      Offset= -10000   

  uint8_t Rcontrol_Stop_Emcy;               //      Bits=08.  [ 0     , 1      ]  Unit:''     
} Motor_Spd_Fbk_t;

// def @Err_Fbk CAN Message (225)
#define Err_Fbk_IDE (0U)
#define Err_Fbk_DLC (8U)
#define Err_Fbk_CANID (0xE1U)
typedef struct
{

  // 2 - "??????????????" 
  // 1 - "??????????" 
  // 0 - "No Err" 

  uint8_t Error_code;                       //      Bits=08.  [ 0     , 3      ]  Unit:''     

  uint8_t Code_Num;                         //      Bits=08.  [ 0     , 5      ]  Unit:''     

  uint8_t Brake_Err;                        //      Bits=08.  [ 0     , 0      ]  Unit:''     

  uint8_t CheckSum_Byte3;                   //      Bits=08.  [ 0     , 0      ]  Unit:''     
} Err_Fbk_t;

uint32_t Unpack_vehicle_vin_CAN_yeyazhidong_20210(vehicle_vin_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_vehicle_vin_CAN_yeyazhidong_20210(const vehicle_vin_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Gear_Shift_Cmd_CAN_yeyazhidong_20210(Gear_Shift_Cmd_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Gear_Shift_Cmd_CAN_yeyazhidong_20210(const Gear_Shift_Cmd_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_steering_control_CAN_yeyazhidong_20210(steering_control_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_steering_control_CAN_yeyazhidong_20210(const steering_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Drive_control_CAN_yeyazhidong_20210(Drive_control_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Drive_control_CAN_yeyazhidong_20210(const Drive_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_brake_control_CAN_yeyazhidong_20210(brake_control_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_brake_control_CAN_yeyazhidong_20210(const brake_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Parking_control_CAN_yeyazhidong_20210(Parking_control_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Parking_control_CAN_yeyazhidong_20210(const Parking_control_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_VCU_FeedBack_CAN_yeyazhidong_20210(VCU_FeedBack_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_VCU_FeedBack_CAN_yeyazhidong_20210(const VCU_FeedBack_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_BMS_FeedBack_CAN_yeyazhidong_20210(BMS_FeedBack_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_BMS_FeedBack_CAN_yeyazhidong_20210(const BMS_FeedBack_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Enable_Fbk_CAN_yeyazhidong_20210(Enable_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Enable_Fbk_CAN_yeyazhidong_20210(const Enable_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_FourWheel_Spd_Fbk_CAN_yeyazhidong_20210(FourWheel_Spd_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_FourWheel_Spd_Fbk_CAN_yeyazhidong_20210(const FourWheel_Spd_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Motor_Spd_Fbk_CAN_yeyazhidong_20210(Motor_Spd_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Motor_Spd_Fbk_CAN_yeyazhidong_20210(const Motor_Spd_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Err_Fbk_CAN_yeyazhidong_20210(Err_Fbk_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Err_Fbk_CAN_yeyazhidong_20210(const Err_Fbk_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);

#ifdef __cplusplus
}
#endif

