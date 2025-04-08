#ifndef	_DI_CAN_PEAK_H_
#define	_DI_CAN_PEAK_H_

/******************************************************************************
*
* Copyright (C) 2023 - 2028 KETI, All rights reserved.
*                           (Korea Electronics Technology Institute)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running for Korean Government Project, or
* (b) that interact with KETI project/platform.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the KETI shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from KETI.
*
******************************************************************************/
/******************************************************************************/
/**
*
* @file di_gps.h
*
* @note
*
* DI CAN_PEAK Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "di.h"
#include "di_can.h"

/***************************** Definition ************************************/
#define DI_CAN_PEAK_TASK_MSG_KEY               (0x251531)

/***************************** Enum and Structure ****************************/
/**
* @details DI CAN_PEAK Status
* @param DI_CAN_PEAK_STATUS_E
*/
typedef enum {
    DI_CAN_PEAK_STATUS_DEINITIALIZED                = 0,
    DI_CAN_PEAK_STATUS_INITIALIZED                  = 1,
    DI_CAN_PEAK_STATUS_CLOSED                       = 2,
    DI_CAN_PEAK_STATUS_OPENED                       = 3,
    DI_CAN_PEAK_STATUS_STARTED                      = 4,
    DI_CAN_PEAK_STATUS_STOPPED                      = 5,
    DI_CAN_PEAK_STATUS_MAX                          = 255,
} DI_CAN_PEAK_STATUS_E;

/**
* @details DI_CAN_PEAK_EVENT_E
* @param DI_CAN_PEAK_EVENT_START
* @param DI_CAN_PEAK_EVENT_STOP
*/
typedef enum {
    eDI_CAN_PEAK_EVENT_UNKNOWN                    = 0x0000,
    eDI_CAN_PEAK_EVENT_START                      = 0x0001,
    eDI_CAN_PEAK_EVENT_STOP                       = 0x0002,
    eDI_CAN_PEAK_EVENT_UNDEFINED_1,
    eDI_CAN_PEAK_EVENT_UNDEFINED_2,
    eDI_CAN_PEAK_EVENT_MAX                        = 0xFFFF
} DI_CAN_PEAK_EVENT_E;

/**
 * @brief Control command signals for EPS, ACC, and AEB from CAN ID 0x156.
 *
 * This structure represents the control requests for electric power steering (EPS),
 * automated cruise control (ACC), and autonomous emergency braking (AEB) functions.
 *
 * @details
 * - Message ID : 0x156
 * - Baudrate   : 500 kbps
 * - Byte Order : Little-endian
 * - Tx Rate    : 10 ms
 * - Tx Timeout : 1000 ms
 *
 * Bit mapping (Little-endian):
 * - Bits  [0]      : EPS_En
 * - Bits  [2]      : Override_Ignore
 * - Bits  [8–15]   : EPS_Speed
 * - Bits  [16]     : ACC_En
 * - Bits  [22]     : AEB_En
 * - Bits  [40–47]  : AEB_decel_value
 * - Bits  [56–63]  : Aliv_Cnt
 * - Other bits     : Reserved
 *
 * @param bEpsEnable EPS mode enable request (Automated Power Steering Module).
 *        - Bit position : 0
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param bOverrideIgnore Whether to ignore driver override (steering, acceleration, brake).
 *        - Bit position : 2
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param ucEpsSpeed Steering angular speed.
 *        - Bit position : 8–15
 *        - Conversion   : `1.0 * value`
 *        - Range        : [0, 250]
 *        - Note         : Unit is not deg/s
 *
 * @param bAccEnable ACC mode enable request (Automated Speed Module).
 *        - Bit position : 16
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param bAebEnable AEB (Autonomous Emergency Braking) activation request.
 *        - Bit position : 22
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param fAebDecelValue AEB deceleration value (in Gs).
 *        - Bit position : 40–47
 *        - Conversion   : `0.01 * value`
 *        - Range        : [0.0, 1.0]
 *
 * @param ucAliveCnt Alive counter that increments with each CAN message transmission.
 *        - Bit position : 56–63
 *        - Conversion   : `1.0 * value`
 *        - Range        : [0, 255]
 *
 * @param unReserved Reserved field, unused.
 */
typedef struct DI_CAN_PEAK_MSG_ID_156_t {
    bool                        bEpsEnable;
    bool                        bOverrideIgnore;
    uint8_t                     ucEpsSpeed;
    bool                        bAccEnable;
    bool                        bAebEnable;
    float                       fAebDecelValue;
    uint8_t                     ucAliveCnt;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_156_T;


/**
 * @brief Command signals for steering and acceleration control from CAN ID 0x157.
 *
 * This structure represents the control commands transmitted via CAN message ID 0x157.
 * It includes steering angle control and acceleration/deceleration command values.
 *
 * @details
 * - Message ID : 0x157
 * - Baudrate   : 500 kbps[]
 * - Byte Order : Little-endian
 * - Tx Rate    : 10 ms
 * - Tx Timeout : None (depends on ID 0x156)
 *
 * Bit mapping (Little-endian):
 * - Bits [0–15]   : Steer_Cmd
 * - Bits [32–47]  : Accel_Dec_Cmd
 * - Other bits    : Reserved
 *
 * @param fSteerCmd Steering control angle command (in degrees).
 *        - Bit position : 0–15
 *        - Conversion   : `0.1 * value` (signed)
 *        - Range        : [-500.0, 500.0]
 *        - Example      : 0x0001 = 0.1°, 0xFFFF = -0.1°
 *
 * @param fAccCmd Speed control acceleration command (in m/s²).
 *        - Bit position : 32–47
 *        - Conversion   : `(value * 0.01) - 10.23`
 *        - Range        : [-3.0, 1.0]
 *
 * @param unReserved Reserved field, unused.
 */
typedef struct DI_CAN_PEAK_MSG_ID_157_t {
    float                       fSteerCmd;
    float                       fAccCmd;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_157_T;

/**
 * @brief Parsed EPS (Electric Power Steering) status and control data from CAN ID 0x710.
 *
 * This structure represents the decoded EPS-related signals from a CAN message with ID 0x710.
 * It includes steering enable status, control board state, override status, steering angle,
 * torque values, and an alive counter for system monitoring.
 *
 * @details
 * - Message ID : 0x710
 * - Baudrate   : 500 kbps
 * - Byte Order : Little-endian
 * - Tx Rate    : 20 ms
 * - Tx Timeout : 1000 ms
 *
 * Bit mapping (Little-endian):[]
 * - Bits  [0]      : EPS_En_Status
 * - Bits  [1–2]    : EPS_Control_Board_Status
 * - Bits  [3–6]    : EPS_Control_Status
 * - Bits  [13]     : Override_Status
 * - Bits [16–31]   : StrAng
 * - Bits [32–47]   : Str_Drv_Tq
 * - Bits [48–63]   : Str_Out_Tq
 * - Bits [56–63]   : EPS_Alive_Cnt (overlaps last byte)
 * - Other bits     : Reserved
 *
 * @param bEpsEnStatus EPS Manual/Auto mode enable status.
 *        - Bit position : 0
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param ucEpsCtrlBdStatus EPS control board state.
 *        - Bit position : 1–2
 *        - Value         : 0 = Fault, 1 = Initializing, 2 = Operating
 *        - Range         : [0, 2]
 *
 * @param ucEpsCtrlStatus EPS control status.
 *        - Bit position : 3–6
 *        - Value         :
 *            - 0 = None
 *            - 1 = Ready
 *            - 2 = All On
 *            - 3 = ACC On
 *            - 4 = Steer On
 *            - Other = Error
 *        - Range         : [0, 10]
 *
 * @param ucOverrideStatus Override detection flag.
 *        - Bit position : 13
 *        - Value         : 1 = Override occurred (lasts ~1 second)
 *        - Range         : [0, 1]
 *
 * @param fStrAng Steering angle sensor value (in degrees).
 *        - Bit position : 16–31
 *        - Conversion   : `0.1 * value (signed)`
 *        - Range        : [-500.0, 500.0]
 *
 * @param fStrDrvTq Driver-applied steering torque (in Nm).
 *        - Bit position : 32–47
 *        - Conversion   : `0.01 * (value - 2048)`
 *        - Range        : [-20.48, 20.47]
 *
 * @param fStrOutTq Output torque from the EPS system (in Nm).
 *        - Bit position : 48–63
 *        - Conversion   : `0.1 * (value - 2048)`
 *        - Range        : [-204.8, 204.7]
 *
 * @param ucEpsAliveCnt EPS alive count feedback.
 *        - Bit position : 56–63 (last byte)
 *        - Conversion   : `1 * value`
 *        - Range        : [0, 255]
 *
 * @param unReserved Reserved field, unused.
 */
typedef struct DI_CAN_PEAK_MSG_ID_710_t {
    bool                        bEpsEnStatus;
    uint8_t                     ucEpsCtrlBdStatus;
    uint8_t                     ucEpsCtrlStatus;
    uint8_t                     ucOverrideStatus;
    float                       fStrAng;
    float                       fStrDrvTq;
    float                       fStrOutTq;
    uint8_t                     ucEpsAliveCnt;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_710_T;

/**
 * @brief Parsed ACC control and status data from CAN ID 0x711.
 *
 * This structure represents the decoded signals from a CAN message with ID 0x711,
 * related to ACC (Adaptive Cruise Control) system status, vehicle speed, acceleration,
 * turn signals, and various system errors.
 *
 * @details
 * - Message ID : 0x711
 * - Baudrate   : 500 kbps
 * - Byte Order : Little-endian
 * - Tx Rate    : 10 ms
 * - Tx Timeout : 1000 ms
 *
 * Bit mapping (Little-endian):
 * - Bits  [0]     : ACC_En_Status
 * - Bits  [1–2]   : ACC_Control_Board_Status
 * - Bits  [3]     : ACC_USER_CAN_ERR
 * - Bits  [4]     : ACC_ERR
 * - Bits  [5]     : ACC_Veh_ERR
 * - Bits  [6–7]   : Reserved
 * - Bits  [8–15]  : Vehicle_speed
 * - Bits [16–31]  : LONG_ACCEL
 * - Bits [40]     : Turn_Right_En
 * - Bits [41]     : Hazard_En
 * - Bits [42]     : Turn_Left_En
 * - Bits [56–63]  : ACC_Alive_Cnt
 * - Other bits    : Reserved
 *
 * @param bAccEnStatus Manual/Auto mode status feedback.
 *        - Bit position : 0
 *        - Value         : 0x00 = Disabled, 0x01 = Enabled
 *        - Range         : [0, 1]
 *
 * @param ucAccCtrlBdStatus ACC control board status.
 *        - Bit position : 1–2
 *        - Value         : 0 = Fault, 1 = Initializing, 2 = Operating
 *        - Range         : [0, 2]
 *
 * @param ucAccVehErr ACC vehicle-level error flag.
 *        - Bit position : 5
 *        - Value         : 0 = Normal, 1 = Error
 *        - Range         : [0, 1]
 *
 * @param ucAccErr ACC actuator-level error flag.
 *        - Bit position : 4
 *        - Value         : 0 = Normal, 1 = Error
 *        - Range         : [0, 1]
 *
 * @param ucAccUserCanErr ACC user CAN error flag.
 *        - Bit position : 3
 *        - Value         : 0 = Normal, 1 = AEB triggered due to CAN timeout
 *        - Range         : [0, 1]
 *
 * @param ucVehicleSpeed Vehicle speed (in km/h).
 *        - Bit position : 8–15
 *        - Conversion   : `1 * value`
 *        - Range        : [0, 255]
 *
 * @param fLongAccel Longitudinal acceleration sensor value (in m/s²).
 *        - Bit position : 16–31
 *        - Conversion   : `0.01 * (value - 1023)`
 *        - Range        : [-10.23, 10.23]
 *
 * @param bTurnRightEn Right turn signal enabled.
 *        - Bit position : 40
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param bHazardEn Hazard light enabled.
 *        - Bit position : 41
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param bTurnLeftEn Left turn signal enabled.
 *        - Bit position : 42
 *        - Value         : 0 = Disabled, 1 = Enabled
 *        - Range         : [0, 1]
 *
 * @param ucAccAliveCnt Alive counter feedback.
 *        - Bit position : 56–63
 *        - Conversion   : `1 * value`
 *        - Range        : [0, 255]
 *
 * @param unReserved Reserved field, unused.
 */
typedef struct DI_CAN_PEAK_MSG_ID_711_t {
    bool                        bAccEnStatus;
    uint8_t                     ucAccCtrlBdStatus;
    uint8_t                     ucAccVehErr;
    uint8_t                     ucAccErr;
    uint8_t                     ucAccUserCanErr;
    uint8_t                     ucVehicleSpeed;
    float                       fLongAccel;
    bool                        bTurnRightEn;
    bool                        bHazardEn;
    bool                        bTurnLeftEn;
    uint8_t                     ucAccAliveCnt;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_711_T;

/**
 * @brief Parsed vehicle dynamics data from CAN ID 0x713.
 *
 * This structure represents the decoded signals related to vehicle dynamics from a CAN message with ID 0x713.
 * It includes lateral acceleration, yaw rate, and brake master cylinder pressure.
 *
 * @details
 * - Message ID : 0x713
 * - Baudrate   : 500 kbps
 * - Byte Order : Little-endian
 * - Tx Rate    : 10 ms
 * - Tx Timeout : None
 *
 * Bit mapping (Little-endian):
 * - Bits [0–15]    : LAT_ACCEL
 * - Bits [16–31]   : YAW_RATE
 * - Bits [32–47]   : BRK_CYLINDER
 * - Bits [48–63]   : Reserved
 *
 * @param fLatAccel Lateral acceleration sensor value (in m/s²).
 *        - Bit position : 0–15
 *        - Conversion   : `0.01 * (value - 1023)`
 *        - Range        : [-10.23, 10.23]
 *
 * @param fYawRate Yaw rate sensor value (in deg/s).
 *        - Bit position : 16–31
 *        - Conversion   : `0.01 * (value - 4095)`
 *        - Range        : [-40.95, 40.94]
 *
 * @param fBrkCylinder Brake master cylinder pressure (in bar).
 *        - Bit position : 32–47
 *        - Conversion   : `0.1 * value`
 *        - Range        : [0, 409.4]
 *
 * @param unReserved Reserved field, unused (bits 48–63).
[] */
typedef struct DI_CAN_PEAK_MSG_ID_713_t {
    float                       fLatAccel;
    float                       fYawRate;
    float                       fBrkCylinder;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_713_T;


/**
 * @brief Parsed object detection data from CAN ID 0x714.
 *
 * This structure represents the decoded object detection signals from a CAN message with ID 0x714.
 * It contains distance level, relative position, object distance, and relative speed.
 *
 * @details
 * - Message ID : 0x714
 * - Baudrate   : 500 kbps
 * - Byte Order : Little-endian
 * - Tx Rate    : 10 ms
 * - Tx Timeout : None
 *
 * Bit mapping (Little-endian):
 * - Bits [0–7]     : RAD_ObjState
 * - Bits [16–31]   : RAD_ObjLatPos
 * - Bits [32–47]   : RAD_ObjDist
 * - Bits [48–63]   : RAD_ObjRelSpd
 * - Bits [8–15], [64–127] : Reserved
 *
 * @param ucRadObjState Object distance level.
 *        - Bit position : 0–7
 *        - Conversion   : `1 * value`
 *        - Range        : [0, 4]
 *
 * @param fRadObjLatPos Lateral distance to the object (in meters).
 *        - Bit position : 16–31
 *        - Conversion   : `0.1 * (value - 20)`
 *        - Range        : [-20, 31.1]
 *
 * @param fRadObjDist Longitudinal distance to the object (in meters).
 *        - Bit position : 32–47
 *        - Conversion   : `0.2 * (value - 10)`
 *        - Range        : [-10, 204.7]
 *
 * @param fRadObjRelSpd Relative speed of the object (in m/s).
 *        - Bit position : 48–63
 *        - Conversion   : `0.1 * (value - 170)`
 *        - Range        : [-17.0, 23.95]
 *
 * @param unReserved Reserved field, unused (includes bits 8–15 and remaining unused bits).
 */
typedef struct DI_CAN_PEAK_MSG_ID_714_t {
    uint8_t                     ucRadObjState;
    float                       fRadObjLatPos;
    float                       fRadObjDist;
    float                       fRadObjRelSpd;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_714_T;


/**
 * @brief Parsed vehicle control signals from CAN ID 0x715.
 *
 * This structure represents the decoded signals from a CAN message with ID 0x715.
 * It contains accelerator pedal position, steering wheel angular velocity, and brake signal.
 *
 * @details
 * - Message ID : 0x715
 * - Baudrate   : 500 kbps
 * - Byte Order : Little-endian
 * - Tx Rate    : 10 ms
 * - Tx Timeout : None
 *
 * Bit mapping (Little-endian):
 * - Bits [0–7]     : AcceleratorPedalPosition
 * - Bits [8–15]    : SteeringWheelAngleRate
 * - Bits [16–17]   : BrakeActSignal
 * - Bits [18–63]   : Reserved
 *
 * @param fAccPedalPos Accelerator pedal position.
 *        - Bit position : 0–7
 *        - Conversion   : `0.3906 * value`
 *        - Range        : [0, 99.603]
 *
 * @param unSteeringWheelAngleRate Steering wheel angular velocity.
 *        - Bit position : 8–15
 *        - Conversion   : `4 * value`
 *        - Range        : [0, 1016]
 *
 * @param ucBrakeActSignal Brake activation signal.
 *        - Bit position : 16–17
 *        - Conversion   : `1 * value`
 *        - Range        : [0, 3]
 *
 * @param unReserved Reserved field (bits 18–63), unused.
 */
typedef struct DI_CAN_PEAK_MSG_ID_715_t {
    float                       fAccPedalPos;
    uint32_t                    unSteeringWheelAngleRate;
    uint8_t                     ucBrakeActSignal;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_715_T;


/**
* @details DI_CAN_PEAK_DATA_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_DATA_t {
    uint32_t                    unReserved;
} DI_CAN_PEAK_DATA_T;

/**
* @details DI_CAN_PEAK_EVENT_MSG_T
* @param eEventType
*/
typedef struct DI_CAN_PEAK_EVENT_MSG_t {
    DI_CAN_PEAK_EVENT_E          eEventType;
} DI_CAN_PEAK_EVENT_MSG_T;

/**
* @details DI_CAN_PEAK_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_t {
    DI_CAN_PEAK_STATUS_E        eDiCanPeakStatus;
    DI_CAN_PEAK_DATA_T          stDiCanPeakData;
    bool                        bLogLevel;
    bool                        bCanPeakNotAvailable;
    uint32_t                    unReserved;
} DI_CAN_PEAK_T;

/***************************** Function Protype ******************************/

int32_t DI_CAN_PEAK_Init(DI_CAN_PEAK_T *pstDiGps);
int32_t DI_CAN_PEAK_DeInit(DI_CAN_PEAK_T *pstDiGps);

int32_t DI_CAN_PEAK_SetLog(DI_CAN_PEAK_T *pstDiGps);
int32_t DI_CAN_PEAK_Get(DI_CAN_PEAK_T *pstDiGps);
int32_t DI_CAN_PEAK_SetNa(DI_CAN_PEAK_T *pstDiGps, bool bNotAvailable);

int32_t DI_CAN_PEAK_Open(DI_CAN_PEAK_T *pstDiGps);
int32_t DI_CAN_PEAK_Close(DI_CAN_PEAK_T *pstDiGps);
int32_t DI_CAN_PEAK_Start(DI_CAN_PEAK_T *pstDiGps);
int32_t DI_CAN_PEAK_Stop(DI_CAN_PEAK_T *pstDiGps);

void DI_CAN_PEAK_Status(DI_CAN_PEAK_T *pstDiGps);
#endif	/* _DI_CAN_PEAK_H_ */

