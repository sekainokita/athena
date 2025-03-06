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
* @details DI_CAN_PEAK_MSG_ID_156_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_MSG_ID_156_t {
    bool                        bEpsEnable;
    bool                        bOverrideIgnore;
    uint8_t                     unEpsSpeed;
    bool                        bAccEnable;
    bool                        bAebEnable;
    float                       fAebDecelValue;
    uint8_t                     unAliveCnt;
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_156_T;

/**
* @details DI_CAN_PEAK_MSG_ID_157_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_MSG_ID_157_t {
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_157_T;

/**
* @details DI_CAN_PEAK_MSG_ID_710_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_MSG_ID_710_t {
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_710_T;

/**
* @details DI_CAN_PEAK_MSG_ID_711_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_MSG_ID_711_t {
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_711_T;

/**
* @details DI_CAN_PEAK_MSG_ID_713_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_MSG_ID_713_t {
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_713_T;

/**
* @details DI_CAN_PEAK_MSG_ID_714_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_MSG_ID_714_t {
    uint32_t                    unReserved;
} DI_CAN_PEAK_MSG_ID_714_T;

/**
* @details DI_CAN_PEAK_MSG_ID_715_T
* @param unReserved
*/
typedef struct DI_CAN_PEAK_MSG_ID_715_t {
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

