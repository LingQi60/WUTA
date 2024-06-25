
// this file were genereted by coderdbc.com web service
// any questions - mailto:coderdbc@gmail.com

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

// This file must define:
// base monitor struct
// function signature for CRC calculation
// function signature for getting system tick value (100 us step)
// #include "canmonitorutil.h"

// def @MSG100 CAN Message (256)
#define MSG100_IDE (0U)
#define MSG100_DLC (8U)
#define MSG100_CANID (0x100U)
#define MSG100_CYC (10U)
    typedef struct
    {

        // ????????????????????0=??????1=??????
        uint8_t AutoCtrlEna; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ????????????
        uint8_t ModeCtrlCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t ModeCtrlCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG100_t;

// def @MSG101 CAN Message (257)
#define MSG101_IDE (0U)
#define MSG101_DLC (8U)
#define MSG101_CANID (0x101U)
#define MSG101_CYC (10U)
// signal: @AccPedCmd
#define MSG101_AccPedCmd_CovFactor (0.1)
// conversion value to CAN signal
#define MSG101_AccPedCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG101_AccPedCmd_fromS(x) ((x)*0.1)

// signal: @AccPedInv
#define MSG101_AccPedInv_CovFactor (0.1)
// conversion value to CAN signal
#define MSG101_AccPedInv_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG101_AccPedInv_fromS(x) ((x)*0.1)

    typedef struct
    {

        // ??????????????????????????0??100%??
        uint16_t AccPedCmd; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // -
        uint8_t AccTkoDis; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ??????????????????0=??????1=??????
        uint8_t AccCtrlEna; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ????????????????????????????0??100%??
        uint16_t AccPedInv; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // ????????????
        uint8_t AccCtrlCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t AccCtrlCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG101_t;

// def @MSG102 CAN Message (258)
#define MSG102_IDE (0U)
#define MSG102_DLC (8U)
#define MSG102_CANID (0x102U)
#define MSG102_CYC (10U)
// signal: @BrkPedCmd
#define MSG102_BrkPedCmd_CovFactor (0.1)
// conversion value to CAN signal
#define MSG102_BrkPedCmd_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG102_BrkPedCmd_fromS(x) ((x)*0.1)

// signal: @BrkPedInv
#define MSG102_BrkPedInv_CovFactor (0.1)
// conversion value to CAN signal
#define MSG102_BrkPedInv_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG102_BrkPedInv_fromS(x) ((x)*0.1)

    typedef struct
    {

        // ??????????????????????????0??100%??
        uint16_t BrkPedCmd; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // -
        uint8_t BrkTkoDis; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ??????????????????0=??????1=??????
        uint8_t BrkCtrlEna; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ????????????????????????????0??100%??
        uint16_t BrkPedInv; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // ????????????
        uint8_t BrkCtrlCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t BrkCtrlCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG102_t;

// def @MSG103 CAN Message (259)
#define MSG103_IDE (0U)
#define MSG103_DLC (8U)
#define MSG103_CANID (0x103U)
#define MSG103_CYC (10U)
// signal: @StrAngCmd
#define MSG103_StrAngCmd_CovFactor (0.1)
// conversion value to CAN signal
#define MSG103_StrAngCmd_toS(x) ((int16_t)((x) / 0.1 + 8000))
// conversion value from CAN signal
#define MSG103_StrAngCmd_fromS(x) ((x)*0.1)

// signal: @StrAngLimit
#define MSG103_StrAngLimit_CovFactor (0.1)
// conversion value to CAN signal
#define MSG103_StrAngLimit_toS(x) ((int16_t)((x) / 0.1 + 8000))
// conversion value from CAN signal
#define MSG103_StrAngLimit_fromS(x) ((x)*0.1)

    typedef struct
    {

        // ????????????????????????-470deg??470deg??
        int16_t StrAngCmd; //      Bits=14.  [ -540  , 540    ]  Unit:'deg'   Offset= -800      Factor= 0.1

        // ????????????????0=??????1=??????
        uint8_t StrCtrlEna; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ??????????????:??????????0deg??470deg??
        int16_t StrAngLimit; //      Bits=14.  [ -540  , 540    ]  Unit:'deg'   Offset= -800      Factor= 0.1

        // ????????????????????????????0??1020deg/s??
        uint16_t StrAngRateLimit; //      Bits=12.  [ 0     , 1020   ]  Unit:'deg/s'

        // ????????????
        uint8_t StrCtrlCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t StrCtrlCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG103_t;

// def @MSG104 CAN Message (260)
#define MSG104_IDE (0U)
#define MSG104_DLC (8U)
#define MSG104_CANID (0x104U)
#define MSG104_CYC (10U)
    typedef struct
    {

        // ??????????????2=R????3=N????4=D??????????????
        uint8_t GearCmd; //      Bits=04.  [ 0     , 255    ]  Unit:'-'

        // ??????????????0=??????1=??????
        uint8_t GearCtrlEna; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ????????????????0=??????????????1=??????????????2=??????????????3??????
        uint8_t TurnLightCmd; //      Bits=02.  [ 0     , 3      ]  Unit:'-'

        // ??????????????????0=????????1=??????????????????
        uint8_t BeamLightCmd; //      Bits=02.  [ 0     , 3      ]  Unit:'-'

        // ??????????????0??????????1????????
        uint8_t HornCmd; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        // ????????????
        uint8_t GearCtrlCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t GearCtrlCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG104_t;

// def @MSG200 CAN Message (512)
#define MSG200_IDE (0U)
#define MSG200_DLC (8U)
#define MSG200_CANID (0x200U)
#define MSG200_CYC (20U)
    typedef struct
    {

        // ??????????????????0=??????1=??????2=??????3??????
        uint8_t AutoCtrlStat; //      Bits=02.  [ 0     , 3      ]  Unit:'-'

        // ????????????
        uint8_t ModeStatCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t ModeStatCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG200_t;

// def @MSG201 CAN Message (513)
#define MSG201_IDE (0U)
#define MSG201_DLC (8U)
#define MSG201_CANID (0x201U)
#define MSG201_CYC (20U)
// signal: @AccPedAct
#define MSG201_AccPedAct_CovFactor (0.1)
// conversion value to CAN signal
#define MSG201_AccPedAct_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG201_AccPedAct_fromS(x) ((x)*0.1)

// signal: @AccPedExe
#define MSG201_AccPedExe_CovFactor (0.1)
// conversion value to CAN signal
#define MSG201_AccPedExe_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG201_AccPedExe_fromS(x) ((x)*0.1)

    typedef struct
    {

        // ????????????????????????????0??100%??
        uint16_t AccPedAct; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // ??????????????0=??????1=??????2=??????3??????
        uint8_t AccCtrlStat; //      Bits=02.  [ 0     , 3      ]  Unit:'-'

        // ??????????????????????????????0??100%??
        uint16_t AccPedCmd; //      Bits=10.  [ 0     , 255    ]  Unit:'-'

        // ????????????????????????????0??100%??
        uint16_t AccPedExe; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // ????????????
        uint8_t AccStatCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t AccStatCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG201_t;

// def @MSG202 CAN Message (514)
#define MSG202_IDE (0U)
#define MSG202_DLC (8U)
#define MSG202_CANID (0x202U)
#define MSG202_CYC (20U)
// signal: @BrkPedAct
#define MSG202_BrkPedAct_CovFactor (0.1)
// conversion value to CAN signal
#define MSG202_BrkPedAct_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG202_BrkPedAct_fromS(x) ((x)*0.1)

// signal: @BrkPedExe
#define MSG202_BrkPedExe_CovFactor (0.1)
// conversion value to CAN signal
#define MSG202_BrkPedExe_toS(x) ((uint16_t)((x) / 0.1))
// conversion value from CAN signal
#define MSG202_BrkPedExe_fromS(x) ((x)*0.1)

    typedef struct
    {

        // ????????????????????????0??100%??
        uint16_t BrkPedAct; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // ??????????????0=??????1=??????2=??????3??????
        uint8_t BrkCtrlStat; //      Bits=02.  [ 0     , 3      ]  Unit:'-'

        // ??????????????????????????????0??100%??
        uint16_t BrkPedCmd; //      Bits=10.  [ 0     , 255    ]  Unit:'-'

        // ??????????????????????????????0??100%??
        uint16_t BrkPedExe; //      Bits=10.  [ 0     , 100    ]  Unit:'%'     Factor= 0.1

        // ????????????
        uint8_t BrkStatCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t BrkStatCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG202_t;

// def @MSG203 CAN Message (515)
#define MSG203_IDE (0U)
#define MSG203_DLC (8U)
#define MSG203_CANID (0x203U)
#define MSG203_CYC (20U)
// signal: @StrAngAct
#define MSG203_StrAngAct_CovFactor (0.1)
// conversion value to CAN signal
#define MSG203_StrAngAct_toS(x) ((int16_t)((x) / 0.1 + 8000))
// conversion value from CAN signal
#define MSG203_StrAngAct_fromS(x) ((x)*0.1)

// signal: @StrAngCmd
#define MSG203_StrAngCmd_CovFactor (0.1)
// conversion value to CAN signal
#define MSG203_StrAngCmd_toS(x) ((int16_t)((x) / 0.1 + 8000))
// conversion value from CAN signal
#define MSG203_StrAngCmd_fromS(x) ((x)*0.1)

// signal: @StrTrqAct
#define MSG203_StrTrqAct_CovFactor (1)
// conversion value to CAN signal
#define MSG203_StrTrqAct_toS(x) ((int8_t)((x) + 64))
// conversion value from CAN signal
#define MSG203_StrTrqAct_fromS(x) ((x))

    typedef struct
    {

        // ??????????????????????????-470deg??470deg??
        int16_t StrAngAct; //      Bits=14.  [ -540  , 540    ]  Unit:'deg'   Offset= -800      Factor= 0.1

        // ??????????????0=??????1=??????2=??????3??????
        uint8_t StrCtrlStat; //      Bits=02.  [ 0     , 3      ]  Unit:'-'

        // ????????????????????????????-470deg??470deg??
        int16_t StrAngCmd; //      Bits=14.  [ -540  , 540    ]  Unit:'deg'   Offset= -800      Factor= 0.1

        // ??????????????????????????-64Nm??63Nm??
        int8_t StrTrqAct; //      Bits=07.  [ -64   , 63     ]  Unit:'Nm'    Offset= -64

        // ????????????
        uint8_t StrStatCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        // ????????????
        uint8_t StrStatCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG203_t;

// def @MSG204 CAN Message (516)
#define MSG204_IDE (0U)
#define MSG204_DLC (8U)
#define MSG204_CANID (0x204U)
#define MSG204_CYC (20U)
    typedef struct
    {

        uint8_t GearAct; //      Bits=04.  [ 0     , 5      ]  Unit:'-'

        uint8_t GearCmd; //      Bits=04.  [ 0     , 5      ]  Unit:'-'

        uint8_t TurnLightAct; //      Bits=02.  [ 0     , 2      ]  Unit:'-'

        uint8_t TurnLightCmd; //      Bits=02.  [ 0     , 2      ]  Unit:'-'

        uint8_t BeamLightAct; //      Bits=02.  [ 0     , 1      ]  Unit:'-'

        uint8_t BeamLightCmd; //      Bits=02.  [ 0     , 1      ]  Unit:'-'

        uint8_t HornStatus; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        uint16_t VehSpd;     // Bits=13

        uint8_t HornCmd; //      Bits=01.  [ 0     , 1      ]  Unit:'-'

        uint8_t GearStatCnt; //      Bits=04.  [ 0     , 15     ]  Unit:'-'

        uint8_t GearStatCks; //      Bits=08.  [ 0     , 255    ]  Unit:'-'
    } MSG204_t;

    uint32_t Unpack_MSG100_UserCAN2(MSG100_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG100_UserCAN2(const MSG100_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG101_UserCAN2(MSG101_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG101_UserCAN2(const MSG101_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG102_UserCAN2(MSG102_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG102_UserCAN2(const MSG102_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG103_UserCAN2(MSG103_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG103_UserCAN2(const MSG103_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG104_UserCAN2(MSG104_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG104_UserCAN2(const MSG104_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG200_UserCAN2(MSG200_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG200_UserCAN2(const MSG200_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG201_UserCAN2(MSG201_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG201_UserCAN2(const MSG201_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG202_UserCAN2(MSG202_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG202_UserCAN2(const MSG202_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG203_UserCAN2(MSG203_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG203_UserCAN2(const MSG203_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);
    uint32_t Unpack_MSG204_UserCAN2(MSG204_t *_m, const uint8_t *_d, uint8_t dlc_);
    uint32_t Pack_MSG204_UserCAN2(const MSG204_t *_m, uint8_t *_d, uint8_t *_len, uint8_t *_ide);

#ifdef __cplusplus
}
#endif
