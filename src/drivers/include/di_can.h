#ifndef	_DI_CAN_H_
#define	_DI_CAN_H_

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
* DI CAN Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "di.h"
#include "di_can_peak.h"

/***************************** Definition ************************************/
#define DI_CAN_TASK_MSG_KEY               (0x250531)

/***************************** Enum and Structure ****************************/
/**
* @details DI CAN Status
* @param DI_CAN_STATUS_E
*/
typedef enum {
    DI_CAN_STATUS_DEINITIALIZED                = 0,
    DI_CAN_STATUS_INITIALIZED                  = 1,
    DI_CAN_STATUS_CLOSED                       = 2,
    DI_CAN_STATUS_OPENED                       = 3,
    DI_CAN_STATUS_STARTED                      = 4,
    DI_CAN_STATUS_STOPPED                      = 5,
    DI_CAN_STATUS_MAX                          = 255,
} DI_CAN_STATUS_E;

/**
* @details DI_CAN_EVENT_E
* @param DI_CAN_EVENT_START
* @param DI_CAN_EVENT_STOP
*/
typedef enum {
    eDI_CAN_EVENT_UNKNOWN                    = 0x0000,
    eDI_CAN_EVENT_START                      = 0x0001,
    eDI_CAN_EVENT_STOP                       = 0x0002,
    eDI_CAN_EVENT_UNDEFINED_1,
    eDI_CAN_EVENT_UNDEFINED_2,
    eDI_CAN_EVENT_MAX                        = 0xFFFF
} DI_CAN_EVENT_E;


/**
* @details DI_CAN_DATA_T
* @param unReserved
*/
typedef struct DI_CAN_DATA_t {
    uint32_t                    unReserved;
} DI_CAN_DATA_T;

/**
* @details DI_CAN_EVENT_MSG_T
* @param eEventType
*/
typedef struct DI_CAN_EVENT_MSG_t {
    DI_CAN_EVENT_E          eEventType;
} DI_CAN_EVENT_MSG_T;

/**
* @details DI_CAN_T
* @param unReserved
*/
typedef struct DI_CAN_t {
    DI_CAN_STATUS_E             eDiCanStatus;
    DI_CAN_DATA_T               stDiCanData;
    bool                        bLogLevel;
    bool                        bCanNotAvailable;
    uint32_t                    unReserved;
} DI_CAN_T;

/***************************** Function Protype ******************************/

int32_t DI_CAN_Init(DI_CAN_T *pstDiGps);
int32_t DI_CAN_DeInit(DI_CAN_T *pstDiGps);

int32_t DI_CAN_SetLog(DI_CAN_T *pstDiGps);
int32_t DI_CAN_Get(DI_CAN_T *pstDiGps);
int32_t DI_CAN_SetNa(DI_CAN_T *pstDiGps, bool bNotAvailable);

int32_t DI_CAN_Open(DI_CAN_T *pstDiGps);
int32_t DI_CAN_Close(DI_CAN_T *pstDiGps);
int32_t DI_CAN_Start(DI_CAN_T *pstDiGps);
int32_t DI_CAN_Stop(DI_CAN_T *pstDiGps);

void DI_CAN_Status(DI_CAN_T *pstDiGps);
#endif	/* _DI_CAN_H_ */

