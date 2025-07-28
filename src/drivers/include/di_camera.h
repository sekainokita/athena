#ifndef _DI_CAMERA_H_
#define _DI_CAMERA_H_

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
* @file di_camera.h
*
* @note
*
* DI Camera Header
*
******************************************************************************/

/***************************** Include ***************************************/
#include "type.h"
#include "di.h"

/***************************** Definition ************************************/
#define DI_CAMERA_TASK_MSG_KEY               (0x240532)
#define DI_CAMERA_MAX_FRAME_SIZE             (1920 * 1080 * 3)
#define DI_CAMERA_MAX_DEVICES                (4)
#define DI_CAMERA_DEFAULT_WIDTH              (1920)
#define DI_CAMERA_DEFAULT_HEIGHT             (1080)
#define DI_CAMERA_DEFAULT_FPS                (30)

/***************************** Enum and Structure ****************************/
/**
* @details DI Camera Status
* @param DI_CAMERA_STATUS_E
*/
typedef enum {
    DI_CAMERA_STATUS_DEINITIALIZED         = 0,
    DI_CAMERA_STATUS_INITIALIZED           = 1,
    DI_CAMERA_STATUS_CLOSED                = 2,
    DI_CAMERA_STATUS_OPENED                = 3,
    DI_CAMERA_STATUS_STARTED               = 4,
    DI_CAMERA_STATUS_STOPPED               = 5,
    DI_CAMERA_STATUS_MAX                   = 255,
} DI_CAMERA_STATUS_E;

/**
* @details DI_CAMERA_EVENT_E
* @param DI_CAMERA_EVENT_START
* @param DI_CAMERA_EVENT_STOP
*/
typedef enum {
    eDI_CAMERA_EVENT_UNKNOWN               = 0x0000,
    eDI_CAMERA_EVENT_START                 = 0x0001,
    eDI_CAMERA_EVENT_STOP                  = 0x0002,
    eDI_CAMERA_EVENT_CAPTURE               = 0x0003,
    eDI_CAMERA_EVENT_UNDEFINED_1,
    eDI_CAMERA_EVENT_UNDEFINED_2,
    eDI_CAMERA_EVENT_MAX                   = 0xFFFF
} DI_CAMERA_EVENT_E;

/**
* @details DI_CAMERA_FORMAT_E
* @param DI_CAMERA_FORMAT_YUV420
* @param DI_CAMERA_FORMAT_RGB888
* @param DI_CAMERA_FORMAT_H264
*/
typedef enum {
    eDI_CAMERA_FORMAT_UNKNOWN              = 0,
    eDI_CAMERA_FORMAT_YUV420               = 1,
    eDI_CAMERA_FORMAT_RGB888               = 2,
    eDI_CAMERA_FORMAT_H264                 = 3,
    eDI_CAMERA_FORMAT_MAX                  = 255,
} DI_CAMERA_FORMAT_E;

/**
* @details DI_CAMERA_FRAME_T
* @param unWidth
* @param unHeight
* @param unDataSize
* @param eFormat
* @param puchData
*/
typedef struct DI_CAMERA_FRAME_t {
    uint32_t                    unWidth;
    uint32_t                    unHeight;
    uint32_t                    unDataSize;
    uint32_t                    unTimestamp;
    uint32_t                    unFrameSeq;
    DI_CAMERA_FORMAT_E          eFormat;
    uint8_t                     *puchData;
} DI_CAMERA_FRAME_T;

/**
* @details DI_CAMERA_CONFIG_T
* @param unDeviceId
* @param unWidth
* @param unHeight
* @param unFps
* @param eFormat
*/
typedef struct DI_CAMERA_CONFIG_t {
    uint32_t                    unDeviceId;
    uint32_t                    unWidth;
    uint32_t                    unHeight;
    uint32_t                    unFps;
    DI_CAMERA_FORMAT_E          eFormat;
    bool                        bHardwareAccel;
} DI_CAMERA_CONFIG_T;

/**
* @details DI_CAMERA_EVENT_MSG_T
* @param eEventType
*/
typedef struct DI_CAMERA_EVENT_MSG_t {
    DI_CAMERA_EVENT_E           eEventType;
    uint32_t                    unDeviceId;
} DI_CAMERA_EVENT_MSG_T;

/**
* @details DI_CAMERA_T
* @param bLogLevel
* @param bCameraNotAvailable
* @param eDiCameraStatus
* @param stDiCameraConfig
* @param stDiCameraFrame
*/
typedef struct DI_CAMERA_t {
    bool                        bLogLevel;
    bool                        bCameraNotAvailable;
    DI_CAMERA_STATUS_E          eDiCameraStatus;
    DI_CAMERA_CONFIG_T          stDiCameraConfig;
    DI_CAMERA_FRAME_T           stDiCameraFrame;
    uint32_t                    unReserved;
} DI_CAMERA_T;

/***************************** Function Prototype *****************************/

int32_t DI_CAMERA_Init(DI_CAMERA_T *pstDiCamera);
int32_t DI_CAMERA_DeInit(DI_CAMERA_T *pstDiCamera);

int32_t DI_CAMERA_SetLog(DI_CAMERA_T *pstDiCamera);
int32_t DI_CAMERA_SetConfig(DI_CAMERA_T *pstDiCamera, DI_CAMERA_CONFIG_T *pstConfig);
int32_t DI_CAMERA_GetConfig(DI_CAMERA_T *pstDiCamera, DI_CAMERA_CONFIG_T *pstConfig);
int32_t DI_CAMERA_SetNa(DI_CAMERA_T *pstDiCamera, bool bNotAvailable);

int32_t DI_CAMERA_Open(DI_CAMERA_T *pstDiCamera);
int32_t DI_CAMERA_Close(DI_CAMERA_T *pstDiCamera);
int32_t DI_CAMERA_Start(DI_CAMERA_T *pstDiCamera);
int32_t DI_CAMERA_Stop(DI_CAMERA_T *pstDiCamera);

int32_t DI_CAMERA_GetFrame(DI_CAMERA_T *pstDiCamera, DI_CAMERA_FRAME_T *pstFrame);
int32_t DI_CAMERA_ReleaseFrame(DI_CAMERA_T *pstDiCamera, DI_CAMERA_FRAME_T *pstFrame);

void DI_CAMERA_Status(DI_CAMERA_T *pstDiCamera);

#if defined(CONFIG_VIDEO_STREAMING)
/* Ring buffer video streaming removed - now uses direct GStreamer pipelines */
/* int32_t DI_CAMERA_ConnectVideoStreaming(DI_CAMERA_T *pstDiCamera, DI_RING_BUFFER_T *pstRingBuffer); -- REMOVED */
/* int32_t DI_CAMERA_DisconnectVideoStreaming(DI_CAMERA_T *pstDiCamera); -- REMOVED */
#endif

#endif /* _DI_CAMERA_H_ */