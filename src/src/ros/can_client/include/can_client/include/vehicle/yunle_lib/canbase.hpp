
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



// def @CCU_Status CAN Message (81)
#define CCU_Status_IDE (0U)
#define CCU_Status_DLC (8U)
#define CCU_Status_CANID (0x51U)
// signal: @CCU_Steering_Wheel_Angle
#define CCU_Status_CCU_Steering_Wheel_Angle_CovFactor (0.1)
// conversion value to CAN signal
#define CCU_Status_CCU_Steering_Wheel_Angle_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define CCU_Status_CCU_Steering_Wheel_Angle_fromS(x) ((x) * 0.1)

// signal: @CCU_Vehicle_Speed
#define CCU_Status_CCU_Vehicle_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define CCU_Status_CCU_Vehicle_Speed_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define CCU_Status_CCU_Vehicle_Speed_fromS(x) ((x) * 0.1)

// signal: @Total_Odometer
#define CCU_Status_Total_Odometer_CovFactor (0.1)
// conversion value to CAN signal
#define CCU_Status_Total_Odometer_toS(x) ((uint32_t)((x) / 0.1))
// conversion value from CAN signal
#define CCU_Status_Total_Odometer_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t CCU_ShiftLevel_Sts;               //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t CCU_P_Sts;                        //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t CCU_Ignition_Sts;                 //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t Steering_Wheel_Direction;         //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint16_t CCU_Steering_Wheel_Angle;        //      Bits=12.  [ -204.8, 204.7  ]  Unit:''      Factor= 0.1   

  uint16_t CCU_Vehicle_Speed;               //      Bits=09.  [ 0     , 51     ]  Unit:'m/s'   Factor= 0.1   

  uint8_t CCU_Drive_Mode;                   //      Bits=03.  [ 0     , 3      ]  Unit:''     

  uint8_t CCU_ACC_Level;                    //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t CCU_Brake_Level;                  //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint32_t Total_Odometer;                  //      Bits=20.  [ 0     , 1048575 ]  Unit:''      Factor= 0.1   

  uint8_t Left_Turn_Light_Sts;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t Right_Turn_Light_Sts;             //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t Hazard_Light_Sts;                 //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t Position_Light_Sts;               //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t LowBeam_Sts;                      //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t HighBeam_Sts;                     //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t RearFog_Sts;                      //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t Horn_Sts;                         //      Bits=01.  [ 0     , 0      ]  Unit:''     
} CCU_Status_t;

// def @SAS_SM_input_vcu70 CAN Message (224)
#define SAS_SM_input_vcu70_IDE (0U)
#define SAS_SM_input_vcu70_DLC (8U)
#define SAS_SM_input_vcu70_CANID (0xE0U)
// signal: @SAS_Angle
#define SAS_SM_input_vcu70_SAS_Angle_CovFactor (0.1)
// conversion value to CAN signal
#define SAS_SM_input_vcu70_SAS_Angle_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define SAS_SM_input_vcu70_SAS_Angle_fromS(x) ((x) * 0.1)

// signal: @SAS_Angle_Speed
#define SAS_SM_input_vcu70_SAS_Angle_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define SAS_SM_input_vcu70_SAS_Angle_Speed_toS(x) ((int8_t)((x) / 0.1))
// conversion value from CAN signal
#define SAS_SM_input_vcu70_SAS_Angle_Speed_fromS(x) ((x) * 0.1)

typedef struct
{

  int16_t SAS_Angle;                        //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:'deg'   Factor= 0.1   

  int8_t SAS_Angle_Speed;                   //  [-] Bits=08.  [ 0     , 0      ]  Unit:'deg/s' Factor= 0.1   

  uint8_t SAS_Trim_Sts;                     //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t SAS_Calibration_Sts;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t SAS_Failure_Sts;                  //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t SAS_Message_Counter;              //      Bits=04.  [ 0     , 15     ]  Unit:''     

  uint8_t SAS_Checksum;                     //      Bits=04.  [ 0     , 0      ]  Unit:''     
} SAS_SM_input_vcu70_t;

// def @PadGateWay CAN Message (256)
#define PadGateWay_IDE (0U)
#define PadGateWay_DLC (8U)
#define PadGateWay_CANID (0x100U)
// signal: @GW_Steering_Wheel_Angle
#define PadGateWay_GW_Steering_Wheel_Angle_CovFactor (0.1)
// conversion value to CAN signal
#define PadGateWay_GW_Steering_Wheel_Angle_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define PadGateWay_GW_Steering_Wheel_Angle_fromS(x) ((x) * 0.1)

// signal: @GW_Target_Speed
#define PadGateWay_GW_Target_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define PadGateWay_GW_Target_Speed_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define PadGateWay_GW_Target_Speed_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t GW_ShiftLevel_Req;                //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_ACC_Level;                     //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Brake_Level;                   //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Drive_Mode_Req;                //      Bits=02.  [ 0     , 3      ]  Unit:''     

  int16_t GW_Steering_Wheel_Angle;          //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:''      Factor= 0.1   

  uint8_t GW_Left_Turn_Light_Req;           //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Right_Turn_Light_Req;          //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Hazard_Light_Req;              //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Position_Light_Req;            //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_LowBeam_Req;                   //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_HighBeam_Req;                  //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_RearFogLight_Req;              //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Horn_Req;                      //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint16_t GW_Target_Speed;                 //      Bits=10.  [ 0     , 102.3  ]  Unit:'kmph'  Factor= 0.1   

  uint8_t GW_Ebrake;                        //      Bits=01.  [ 0     , 1      ]  Unit:''     
} PadGateWay_t;

// def @SCU CAN Message (288)
#define SCU_IDE (0U)
#define SCU_DLC (8U)
#define SCU_CANID (0x120U)
// signal: @SCU_Steering_Wheel_Angle
#define SCU_SCU_Steering_Wheel_Angle_CovFactor (0.1)
// conversion value to CAN signal
#define SCU_SCU_Steering_Wheel_Angle_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define SCU_SCU_Steering_Wheel_Angle_fromS(x) ((x) * 0.1)

// signal: @SCU_Target_Speed
#define SCU_SCU_Target_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define SCU_SCU_Target_Speed_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define SCU_SCU_Target_Speed_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t SCU_ShiftLevel_Req;               //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t SCU_ACC_Mode;                     //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t SCU_Brake_Mode;                   //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t SCU_Drive_Mode_Req;               //      Bits=02.  [ 0     , 3      ]  Unit:''     

  int16_t SCU_Steering_Wheel_Angle;         //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:''      Factor= 0.1   

  uint16_t SCU_Target_Speed;                //      Bits=09.  [ 0     , 51     ]  Unit:'km/h'  Factor= 0.1   

  uint8_t SCU_Brk_En;                       //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t GW_Left_Turn_Light_Req;           //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Right_Turn_Light_Req;          //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Hazard_Light_Req;              //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Position_Light_Req;            //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_LowBeam_Req;                   //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_HighBeam_Req;                  //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_RearFogLight_Req;              //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint8_t GW_Horn_Req;                      //      Bits=02.  [ 0     , 3      ]  Unit:''     
  
  int16_t SCU_Brake_Coefficient;
  
} SCU_t;

// def @EPS_Status CAN Message (324)
#define EPS_Status_IDE (0U)
#define EPS_Status_DLC (8U)
#define EPS_Status_CANID (0x144U)
// signal: @EPS_Angle
#define EPS_Status_EPS_Angle_CovFactor (0.1)
// conversion value to CAN signal
#define EPS_Status_EPS_Angle_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define EPS_Status_EPS_Angle_fromS(x) ((x) * 0.1)

typedef struct
{

  int16_t EPS_Angle;                        //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:''      Factor= 0.1   

  uint8_t EPS_System_Status;                //      Bits=02.  [ 0     , 3      ]  Unit:''     

  uint16_t EPS_Fault_Code;                  //      Bits=16.  [ 0     , 65535  ]  Unit:''     
} EPS_Status_t;

// def @CCU_SAS_Info CAN Message (338)
#define CCU_SAS_Info_IDE (0U)
#define CCU_SAS_Info_DLC (8U)
#define CCU_SAS_Info_CANID (0x152U)
// signal: @SAS_Angle
#define CCU_SAS_Info_SAS_Angle_CovFactor (0.1)
// conversion value to CAN signal
#define CCU_SAS_Info_SAS_Angle_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define CCU_SAS_Info_SAS_Angle_fromS(x) ((x) * 0.1)

// signal: @SAS_Angle_Speed
#define CCU_SAS_Info_SAS_Angle_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define CCU_SAS_Info_SAS_Angle_Speed_toS(x) ((int8_t)((x) / 0.1))
// conversion value from CAN signal
#define CCU_SAS_Info_SAS_Angle_Speed_fromS(x) ((x) * 0.1)

typedef struct
{

  int16_t SAS_Angle;                        //  [-] Bits=16.  [ -3276.8, 3276.7 ]  Unit:''      Factor= 0.1   

  int8_t SAS_Angle_Speed;                   //  [-] Bits=08.  [ 0     , 0      ]  Unit:'deg/s' Factor= 0.1   

  uint8_t SAS_Failure_Sts;                  //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t SAS_Calibration_Sts;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t SAS_Trim_Sts;                     //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t SAS_Message_Counter;              //      Bits=04.  [ 0     , 15     ]  Unit:''     

  uint8_t SAS_Checksum;                     //      Bits=04.  [ 0     , 0      ]  Unit:''     
} CCU_SAS_Info_t;

// def @CCU_Req CAN Message (340)
#define CCU_Req_IDE (0U)
#define CCU_Req_DLC (8U)
#define CCU_Req_CANID (0x154U)
// signal: @CCU_IBC_HP_pressure
#define CCU_Req_CCU_IBC_HP_pressure_CovFactor (0.01)
// conversion value to CAN signal
#define CCU_Req_CCU_IBC_HP_pressure_toS(x) ((uint16_t)((x) / 0.01))
// conversion value from CAN signal
#define CCU_Req_CCU_IBC_HP_pressure_fromS(x) ((x) * 0.01)

// signal: @CCU_Break_Pressure_Req
#define CCU_Req_CCU_Break_Pressure_Req_CovFactor (0.01)
// conversion value to CAN signal
#define CCU_Req_CCU_Break_Pressure_Req_toS(x) ((uint16_t)((x) / 0.01))
// conversion value from CAN signal
#define CCU_Req_CCU_Break_Pressure_Req_fromS(x) ((x) * 0.01)

typedef struct
{

  uint16_t CCU_Torque_Req;                     //      Bits=16.  [ 0     , 80     ]  Unit:'Nm'   

  uint8_t CCU_Torque_Req_Invaild;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t CCU_Break_Pressure_Req_Val_ATM;      //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint16_t CCU_IBC_HP_pressure;                //      Bits=16.  [ 0     , 400    ]  Unit:'Mpa'   Factor= 0.01  

  uint16_t CCU_Break_Pressure_Req;             //      Bits=16.  [ 0     , 10     ]  Unit:'MPa'   Factor= 0.01  

  uint8_t CCU_Break_Pressure_Req_Val;          //      Bits=01.  [ 0     , 0      ]  Unit:''     
} CCU_Req_t;

// def @LHS1_Torque_Feedback CAN Message (376)
#define LHS1_Torque_Feedback_IDE (0U)
#define LHS1_Torque_Feedback_DLC (8U)
#define LHS1_Torque_Feedback_CANID (0x178U)
// signal: @Torque_Measured
#define LHS1_Torque_Feedback_Torque_Measured_CovFactor (0.0625)
// conversion value to CAN signal
#define LHS1_Torque_Feedback_Torque_Measured_toS(x) ((int16_t)((x) / 0.0625))
// conversion value from CAN signal
#define LHS1_Torque_Feedback_Torque_Measured_fromS(x) ((x) * 0.0625)

typedef struct
{

  // Torque calculated by DSP in inverter
  int16_t Torque_Measured;                  //  [-] Bits=16.  [ 0     , 0      ]  Unit:'Nm'    Factor= 0.0625

  // Measured motor speed by inverter
  int16_t Speed_Measured;                   //  [-] Bits=16.  [ 0     , 0      ]  Unit:'RPM'  

  // Net current flow measured by the inverter
  int16_t DC_Link_Current;                  //  [-] Bits=16.  [ -32768, 32767  ]  Unit:'A'    

  // Torque message sequence counter
  uint8_t SEQ_Torque;                       //      Bits=08.  [ 0     , 255    ]  Unit:''     

  // Torque message checksum
  uint8_t CS_Torque;                        //      Bits=08.  [ 0     , 255    ]  Unit:''     
} LHS1_Torque_Feedback_t;

// def @RHS1_Torque_Feedback CAN Message (392)
#define RHS1_Torque_Feedback_IDE (0U)
#define RHS1_Torque_Feedback_DLC (8U)
#define RHS1_Torque_Feedback_CANID (0x188U)
// signal: @Torque_Measured
#define RHS1_Torque_Feedback_Torque_Measured_CovFactor (0.0625)
// conversion value to CAN signal
#define RHS1_Torque_Feedback_Torque_Measured_toS(x) ((int16_t)((x) / 0.0625))
// conversion value from CAN signal
#define RHS1_Torque_Feedback_Torque_Measured_fromS(x) ((x) * 0.0625)

typedef struct
{

  // Torque calculated by DSP in inverter
  int16_t Torque_Measured;                  //  [-] Bits=16.  [ 0     , 0      ]  Unit:'Nm'    Factor= 0.0625

  // Measured motor speed by inverter
  int16_t Speed_Measured;                   //  [-] Bits=16.  [ 0     , 0      ]  Unit:'RPM'  

  // Net current flow measured by the inverter
  int16_t DC_Link_Current;                  //  [-] Bits=16.  [ -32768, 32767  ]  Unit:'A'    

  // Torque message sequence counter
  uint8_t SEQ_Torque;                       //      Bits=08.  [ 0     , 255    ]  Unit:''     

  // Torque message checksum
  uint8_t CS_Torque;                        //      Bits=08.  [ 0     , 255    ]  Unit:''     
} RHS1_Torque_Feedback_t;

// def @TBOX_Sts CAN Message (1666)
#define TBOX_Sts_IDE (0U)
#define TBOX_Sts_DLC (8U)
#define TBOX_Sts_CANID (0x682U)
typedef struct
{

  uint8_t TBOX_Ignition_Sts;                   //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_General_Fail_Status;            //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_WIFIAP_Work_Status;             //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_BlueTooth_work_Status;          //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_4G_Work_Status;                 //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_4G_Signal_Level;                //      Bits=08.  [ 0     , 100    ]  Unit:''     

  uint8_t TBOX_WIFI_Signal_Level;              //      Bits=08.  [ 0     , 100    ]  Unit:''     

  uint8_t TBOX_BlueTooth_Signal_Level;         //      Bits=08.  [ 0     , 100    ]  Unit:''     

  uint8_t TBOX_ICAN_Data_Link_Status;          //      Bits=02.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_AirCondition_Status;            //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_AirCleaner_Status;              //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_Cloud_Mirror_Status;            //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_Cameral_Controller_Status;      //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_ExtLamp_Controller_Status;      //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOX_Door_Controller_Status;         //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t TBOXI_Checksum;                      //      Bits=08.  [ 0     , 0      ]  Unit:''     
} TBOX_Sts_t;

// def @GPS_Sts_1 CAN Message (1669)
#define GPS_Sts_1_IDE (0U)
#define GPS_Sts_1_DLC (8U)
#define GPS_Sts_1_CANID (0x685U)
// signal: @GPS_Speed
#define GPS_Sts_1_GPS_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define GPS_Sts_1_GPS_Speed_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define GPS_Sts_1_GPS_Speed_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t GPS_Location_Sts;                 //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t GPS_Northern_or_Southern;         //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t GPS_East_or_West;                 //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint16_t GPS_Speed;                       //      Bits=16.  [ 0     , 200    ]  Unit:'km/h'  Factor= 0.1   

  uint16_t GPS_Direction;                   //      Bits=16.  [ 0     , 359    ]  Unit:''     

  uint8_t GPS_Num_Of_Satellite;             //      Bits=08.  [ 0     , 60     ]  Unit:''     

  uint8_t GPS_Checksum;                     //      Bits=08.  [ 0     , 0      ]  Unit:''     
} GPS_Sts_1_t;

// def @GPS_Sts_2 CAN Message (1670)
#define GPS_Sts_2_IDE (0U)
#define GPS_Sts_2_DLC (8U)
#define GPS_Sts_2_CANID (0x686U)
// signal: @GPS_Longitude
#define GPS_Sts_2_GPS_Longitude_CovFactor (1E-07)
// conversion value to CAN signal
#define GPS_Sts_2_GPS_Longitude_toS(x) ((uint32_t)((x) / 1E-07))
// conversion value from CAN signal
#define GPS_Sts_2_GPS_Longitude_fromS(x) ((x) * 1E-07)

// signal: @GPS_Latitude
#define GPS_Sts_2_GPS_Latitude_CovFactor (1E-07)
// conversion value to CAN signal
#define GPS_Sts_2_GPS_Latitude_toS(x) ((uint32_t)((x) / 1E-07))
// conversion value from CAN signal
#define GPS_Sts_2_GPS_Latitude_fromS(x) ((x) * 1E-07)

typedef struct
{

  uint32_t GPS_Longitude;                   //      Bits=32.  [ 0     , 90     ]  Unit:''      Factor= 1E-07 

  uint32_t GPS_Latitude;                    //      Bits=32.  [ 0     , 180    ]  Unit:''      Factor= 1E-07 
} GPS_Sts_2_t;

// def @Hardware_inpute CAN Message (1793)
#define Hardware_inpute_IDE (0U)
#define Hardware_inpute_DLC (8U)
#define Hardware_inpute_CANID (0x701U)
// signal: @hardware_input_Analog_Weight_V
#define Hardware_inpute_hardware_input_Analog_Weight_V_CovFactor (0.01)
// conversion value to CAN signal
#define Hardware_inpute_hardware_input_Analog_Weight_V_toS(x) ((int8_t)((x) / 0.01))
// conversion value from CAN signal
#define Hardware_inpute_hardware_input_Analog_Weight_V_fromS(x) ((x) * 0.01)

// signal: @GW_Target_Speed
#define Hardware_inpute_GW_Target_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define Hardware_inpute_GW_Target_Speed_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define Hardware_inpute_GW_Target_Speed_fromS(x) ((x) * 0.1)

// signal: @SCU_Target_Speed
#define Hardware_inpute_SCU_Target_Speed_CovFactor (0.1)
// conversion value to CAN signal
#define Hardware_inpute_SCU_Target_Speed_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define Hardware_inpute_SCU_Target_Speed_fromS(x) ((x) * 0.1)

// signal: @HarwareTargetSpd
#define Hardware_inpute_HarwareTargetSpd_CovFactor (0.1)
// conversion value to CAN signal
#define Hardware_inpute_HarwareTargetSpd_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define Hardware_inpute_HarwareTargetSpd_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t hardware_input_ignition;             //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t hardware_input_KeyOn;                //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t hardware_input_D;                    //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t hardware_input_N;                    //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t hardware_input_R;                    //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t hardware_input_Acc;                  //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t hardware_input_Dec;                  //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t hardware_Foot_brake_input;           //      Bits=01.  [ 0     , 1      ]  Unit:''     

  int8_t hardware_input_Analog_Weight_V;       //  [-] Bits=08.  [ 0     , 5.55   ]  Unit:'V'     Factor= 0.01  

  uint8_t HarwareGearCmd;                      //      Bits=04.  [ 0     , 15     ]  Unit:''     

  uint8_t DrvModFlag;                          //      Bits=04.  [ 0     , 15     ]  Unit:''     

  uint8_t gearcmd;                             //      Bits=04.  [ 0     , 15     ]  Unit:''     

  int16_t GW_Target_Speed;                     //  [-] Bits=10.  [ -51.2 , 51.1   ]  Unit:'kmph'  Factor= 0.1   

  int16_t SCU_Target_Speed;                    //  [-] Bits=10.  [ -51.2 , 51.1   ]  Unit:'m/s'   Factor= 0.1   

  int16_t HarwareTargetSpd;                    //  [-] Bits=10.  [ -51.2 , 51.1   ]  Unit:''      Factor= 0.1   
} Hardware_inpute_t;

// def @DebugEn CAN Message (1808)
#define DebugEn_IDE (0U)
#define DebugEn_DLC (8U)
#define DebugEn_CANID (0x710U)
typedef struct
{

  uint8_t BMSDebugEn;                       //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t VelDebugEn;                       //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t PIDDebugEn;                       //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t TqDebugEn;                        //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t ASDebugEn;                        //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t LightDebugEn;                     //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t SASDebugEn;                       //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t ChrgerDebugEn;                    //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t FaultResetCmd;                    //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t LTLedCmd;                         //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t RTLedCmd;                         //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t HalLedCmd;                        //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t PosLedCmd;                        //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t LBLedCmd;                         //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t HBLedCmd;                         //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t RFLedCmd;                         //      Bits=01.  [ 0     , 1      ]  Unit:''     

  uint8_t HornCmd;                          //      Bits=01.  [ 0     , 1      ]  Unit:''     
} DebugEn_t;

// def @ChrgASDebug CAN Message (1810)
#define ChrgASDebug_IDE (0U)
#define ChrgASDebug_DLC (8U)
#define ChrgASDebug_CANID (0x712U)
// signal: @ChrgVolCmd
#define ChrgASDebug_ChrgVolCmd_CovFactor (0.1)
// conversion value to CAN signal
#define ChrgASDebug_ChrgVolCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define ChrgASDebug_ChrgVolCmd_fromS(x) ((x) * 0.1)

// signal: @ChrgCurCmd
#define ChrgASDebug_ChrgCurCmd_CovFactor (0.1)
// conversion value to CAN signal
#define ChrgASDebug_ChrgCurCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define ChrgASDebug_ChrgCurCmd_fromS(x) ((x) * 0.1)

// signal: @ASFLHtCmd
#define ChrgASDebug_ASFLHtCmd_CovFactor (0.1)
// conversion value to CAN signal
#define ChrgASDebug_ASFLHtCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define ChrgASDebug_ASFLHtCmd_fromS(x) ((x) * 0.1)

// signal: @ASFRHtCmd
#define ChrgASDebug_ASFRHtCmd_CovFactor (0.1)
// conversion value to CAN signal
#define ChrgASDebug_ASFRHtCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define ChrgASDebug_ASFRHtCmd_fromS(x) ((x) * 0.1)

// signal: @ASRLHtCmd
#define ChrgASDebug_ASRLHtCmd_CovFactor (0.1)
// conversion value to CAN signal
#define ChrgASDebug_ASRLHtCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define ChrgASDebug_ASRLHtCmd_fromS(x) ((x) * 0.1)

// signal: @ASRRHtCmd
#define ChrgASDebug_ASRRHtCmd_CovFactor (0.1)
// conversion value to CAN signal
#define ChrgASDebug_ASRRHtCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define ChrgASDebug_ASRRHtCmd_fromS(x) ((x) * 0.1)

typedef struct
{

  uint8_t PreRelayClsCmd;                   //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t MainRelayClsCmd;                  //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint8_t ChrgEnCmd;                        //      Bits=01.  [ 0     , 0      ]  Unit:''     

  uint16_t ChrgVolCmd;                      //      Bits=10.  [ 0     , 0      ]  Unit:'V'     Factor= 0.1   

  uint16_t ChrgCurCmd;                      //      Bits=10.  [ 0     , 0      ]  Unit:'A'     Factor= 0.1   

  uint16_t ASFLHtCmd;                       //      Bits=10.  [ 0     , 0      ]  Unit:''      Factor= 0.1   

  uint16_t ASFRHtCmd;                       //      Bits=10.  [ 0     , 0      ]  Unit:''      Factor= 0.1   

  uint16_t ASRLHtCmd;                       //      Bits=10.  [ 0     , 0      ]  Unit:''      Factor= 0.1   

  uint16_t ASRRHtCmd;                       //      Bits=10.  [ 0     , 0      ]  Unit:''      Factor= 0.1   
} ChrgASDebug_t;

// def @DrvDebug CAN Message (1813)
#define DrvDebug_IDE (0U)
#define DrvDebug_DLC (8U)
#define DrvDebug_CANID (0x715U)
// signal: @VelCmd
#define DrvDebug_VelCmd_CovFactor (0.1)
// conversion value to CAN signal
#define DrvDebug_VelCmd_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define DrvDebug_VelCmd_fromS(x) ((x) * 0.1)

// signal: @SASCmd
#define DrvDebug_SASCmd_CovFactor (0.1)
// conversion value to CAN signal
#define DrvDebug_SASCmd_toS(x) ((int16_t)((x) / 0.1))
// conversion value from CAN signal
#define DrvDebug_SASCmd_fromS(x) ((x) * 0.1)

// signal: @VelKp
#define DrvDebug_VelKp_CovFactor (0.1)
// conversion value to CAN signal
#define DrvDebug_VelKp_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define DrvDebug_VelKp_fromS(x) ((x) * 0.1)

// signal: @VelKi
#define DrvDebug_VelKi_CovFactor (0.01)
// conversion value to CAN signal
#define DrvDebug_VelKi_toS(x) ((uint16_t)((x) / 0.01))
// conversion value from CAN signal
#define DrvDebug_VelKi_fromS(x) ((x) * 0.01)

// signal: @VelKd
#define DrvDebug_VelKd_CovFactor (0.01)
// conversion value to CAN signal
#define DrvDebug_VelKd_toS(x) ((uint16_t)((x) / 0.01))
// conversion value from CAN signal
#define DrvDebug_VelKd_fromS(x) ((x) * 0.01)

typedef struct
{

  int16_t VelCmd;                           //  [-] Bits=10.  [ -51.2 , 51.1   ]  Unit:'km/h'  Factor= 0.1   

  int16_t TqCmd;                            //  [-] Bits=10.  [ -512  , 511    ]  Unit:'Nm'   

  int16_t SASCmd;                           //  [-] Bits=12.  [ -204.8, 204.7  ]  Unit:'deg'   Factor= 0.1   

  uint16_t VelKp;                           //      Bits=10.  [ 0     , 102.3  ]  Unit:''      Factor= 0.1   

  uint16_t VelKi;                           //      Bits=10.  [ 0     , 10.23  ]  Unit:''      Factor= 0.01  

  uint16_t VelKd;                           //      Bits=10.  [ 0     , 10.23  ]  Unit:''      Factor= 0.01  
} DrvDebug_t;


// def @BMS_SOC CAN Message (395329537)
#define BMS_SOC_IDE (1U)
#define BMS_SOC_DLC (8U)
#define BMS_SOC_CANID (0x17904001U)
// signal: @BMS_Total_VolBat
#define BMS_SOC_BMS_Total_VolBat_CovFactor (0.1)
// conversion value to CAN signal
#define BMS_SOC_BMS_Total_VolBat_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define BMS_SOC_BMS_Total_VolBat_fromS(x) ((x) * 0.1)

// signal: @BMS_current_Vol
#define BMS_SOC_BMS_current_Vol_CovFactor (0.1)
// conversion value to CAN signal
#define BMS_SOC_BMS_current_Vol_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define BMS_SOC_BMS_current_Vol_fromS(x) ((x) * 0.1)

// signal: @BMS_current_Cur
#define BMS_SOC_BMS_current_Cur_CovFactor (0.1)
// conversion value to CAN signal
#define BMS_SOC_BMS_current_Cur_toS(x) ((int32_t)((x) / 0.1 + 30000))
// conversion value from CAN signal
#define BMS_SOC_BMS_current_Cur_fromS(x) ((x) * 0.1)

// signal: @BMS_SOC
#define BMS_SOC_BMS_SOC_CovFactor (0.1)
// conversion value to CAN signal
#define BMS_SOC_BMS_SOC_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define BMS_SOC_BMS_SOC_fromS(x) ((x) * 0.1)

typedef struct
{

  uint16_t BMS_Total_VolBat;                //      Bits=16.  [ 0     , 65535  ]  Unit:'V'     Factor= 0.1   

  uint16_t BMS_current_Vol;                 //      Bits=16.  [ 0     , 6553.5 ]  Unit:'V'     Factor= 0.1   

  int32_t BMS_current_Cur;                  //      Bits=16.  [ 3000  , 9553.5 ]  Unit:'A'     Offset= -3000     Factor= 0.1   

  uint16_t BMS_SOC;                         //      Bits=16.  [ 0     , 6553.5 ]  Unit:'%'     Factor= 0.1   
} BMS_SOC_t;

uint32_t Unpack_BMS_SOC_BMS_for_C(BMS_SOC_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_BMS_SOC_BMS_for_C(const BMS_SOC_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);



uint32_t Unpack_CCU_Status__YunleCAN(CCU_Status_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_CCU_Status__YunleCAN(const CCU_Status_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_SAS_SM_input_vcu70__YunleCAN(SAS_SM_input_vcu70_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_SAS_SM_input_vcu70__YunleCAN(const SAS_SM_input_vcu70_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_PadGateWay__YunleCAN(PadGateWay_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_PadGateWay__YunleCAN(const PadGateWay_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_SCU__YunleCAN(SCU_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_SCU__YunleCAN(const SCU_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_EPS_Status__YunleCAN(EPS_Status_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_EPS_Status__YunleCAN(const EPS_Status_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_CCU_SAS_Info__YunleCAN(CCU_SAS_Info_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_CCU_SAS_Info__YunleCAN(const CCU_SAS_Info_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_CCU_Req__YunleCAN(CCU_Req_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_CCU_Req__YunleCAN(const CCU_Req_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_LHS1_Torque_Feedback__YunleCAN(LHS1_Torque_Feedback_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_LHS1_Torque_Feedback__YunleCAN(const LHS1_Torque_Feedback_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_RHS1_Torque_Feedback__YunleCAN(RHS1_Torque_Feedback_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_RHS1_Torque_Feedback__YunleCAN(const RHS1_Torque_Feedback_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_TBOX_Sts__YunleCAN(TBOX_Sts_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_TBOX_Sts__YunleCAN(const TBOX_Sts_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_GPS_Sts_1__YunleCAN(GPS_Sts_1_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_GPS_Sts_1__YunleCAN(const GPS_Sts_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_GPS_Sts_2__YunleCAN(GPS_Sts_2_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_GPS_Sts_2__YunleCAN(const GPS_Sts_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_Hardware_inpute__YunleCAN(Hardware_inpute_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_Hardware_inpute__YunleCAN(const Hardware_inpute_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_DebugEn__YunleCAN(DebugEn_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_DebugEn__YunleCAN(const DebugEn_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_ChrgASDebug__YunleCAN(ChrgASDebug_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_ChrgASDebug__YunleCAN(const ChrgASDebug_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
uint32_t Unpack_DrvDebug__YunleCAN(DrvDebug_t* _m, const uint8_t* _d, uint8_t dlc_);
uint32_t Pack_DrvDebug__YunleCAN(const DrvDebug_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);

#ifdef __cplusplus
}
#endif

