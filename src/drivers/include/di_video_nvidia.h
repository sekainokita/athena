#ifndef	_DI_VIDEO_NVIDIA_H_
#define	_DI_VIDEO_NVIDIA_H_

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
* DI VIDEO_NVIDIA Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "di.h"
#include "di_video.h"

/***************************** Definition ************************************/
#define DI_VIDEO_NVIDIA_TASK_MSG_KEY               (0x252531)

/***************************** Enum and Structure ****************************/
/**
* @details DI VIDEO_NVIDIA Status
* @param DI_VIDEO_NVIDIA_STATUS_E
*/
typedef enum {
    DI_VIDEO_NVIDIA_STATUS_DEINITIALIZED                = 0,
    DI_VIDEO_NVIDIA_STATUS_INITIALIZED                  = 1,
    DI_VIDEO_NVIDIA_STATUS_CLOSED                       = 2,
    DI_VIDEO_NVIDIA_STATUS_OPENED                       = 3,
    DI_VIDEO_NVIDIA_STATUS_STARTED                      = 4,
    DI_VIDEO_NVIDIA_STATUS_STOPPED                      = 5,
    DI_VIDEO_NVIDIA_STATUS_MAX                          = 255,
} DI_VIDEO_NVIDIA_STATUS_E;

/**
* @details DI_VIDEO_NVIDIA_EVENT_E
* @param DI_VIDEO_NVIDIA_EVENT_START
* @param DI_VIDEO_NVIDIA_EVENT_STOP
*/
typedef enum {
    eDI_VIDEO_NVIDIA_EVENT_UNKNOWN                    = 0x0000,
    eDI_VIDEO_NVIDIA_EVENT_START                      = 0x0001,
    eDI_VIDEO_NVIDIA_EVENT_STOP                       = 0x0002,
    eDI_VIDEO_NVIDIA_EVENT_UNDEFINED_1,
    eDI_VIDEO_NVIDIA_EVENT_UNDEFINED_2,
    eDI_VIDEO_NVIDIA_EVENT_MAX                        = 0xFFFF
} DI_VIDEO_NVIDIA_EVENT_E;


/**
* @details DI_VIDEO_NVIDIA_DATA_T
* @param unReserved
*/
typedef struct DI_VIDEO_NVIDIA_DATA_t {
    uint32_t                    unReserved;
} DI_VIDEO_NVIDIA_DATA_T;

/**
* @details DI_VIDEO_NVIDIA_EVENT_MSG_T
* @param eEventType
*/
typedef struct DI_VIDEO_NVIDIA_EVENT_MSG_t {
    DI_VIDEO_NVIDIA_EVENT_E          eEventType;
} DI_VIDEO_NVIDIA_EVENT_MSG_T;

/**
* @details DI_VIDEO_NVIDIA_T
* @param unReserved
*/
typedef struct DI_VIDEO_NVIDIA_t {
    DI_VIDEO_NVIDIA_STATUS_E        eDiVideoNvidiaStatus;
    DI_VIDEO_NVIDIA_DATA_T          stDiVideoNvidiaData;
    bool                            bLogLevel;
    bool                            bVideoNvidiaNotAvailable;
    uint32_t                        unReserved;
} DI_VIDEO_NVIDIA_T;

/***************************** Function Protype ******************************/

int32_t DI_VIDEO_NVIDIA_Init(DI_VIDEO_NVIDIA_T *pstDiGps);
int32_t DI_VIDEO_NVIDIA_DeInit(DI_VIDEO_NVIDIA_T *pstDiGps);

int32_t DI_VIDEO_NVIDIA_SetLog(DI_VIDEO_NVIDIA_T *pstDiGps);
int32_t DI_VIDEO_NVIDIA_Get(DI_VIDEO_NVIDIA_T *pstDiGps);
int32_t DI_VIDEO_NVIDIA_SetNa(DI_VIDEO_NVIDIA_T *pstDiGps, bool bNotAvailable);

int32_t DI_VIDEO_NVIDIA_Open(DI_VIDEO_NVIDIA_T *pstDiGps);
int32_t DI_VIDEO_NVIDIA_Close(DI_VIDEO_NVIDIA_T *pstDiGps);
int32_t DI_VIDEO_NVIDIA_Start(DI_VIDEO_NVIDIA_T *pstDiGps);
int32_t DI_VIDEO_NVIDIA_Stop(DI_VIDEO_NVIDIA_T *pstDiGps);

void DI_VIDEO_NVIDIA_Status(DI_VIDEO_NVIDIA_T *pstDiGps);
#endif	/* _DI_VIDEO_NVIDIA_H_ */


