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

/* Multi-camera configuration constants */
#define DI_VIDEO_CAMERA_MAX_COUNT                  6
#define DI_VIDEO_CAMERA_TCP_PORT_BASE              8554
#define DI_VIDEO_CAMERA_RTSP_PORT_BASE             8560
#define DI_VIDEO_CAMERA_PORT_OFFSET                1
#define DI_VIDEO_CAMERA_DEFAULT_AVAILABLE          FALSE

/* Port calculation macros */
#define DI_VIDEO_CAMERA_GET_TCP_PORT(id)           (DI_VIDEO_CAMERA_TCP_PORT_BASE + ((id) - 1) * DI_VIDEO_CAMERA_PORT_OFFSET)
#define DI_VIDEO_CAMERA_GET_RTSP_PORT(id)          (DI_VIDEO_CAMERA_RTSP_PORT_BASE + ((id) - 1) * DI_VIDEO_CAMERA_PORT_OFFSET)

/* Validation macros */
#define DI_VIDEO_CAMERA_IS_VALID_ID(id)            ((id) >= 1 && (id) <= DI_VIDEO_CAMERA_MAX_COUNT)

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

/**
* @details DI_VIDEO_CAMERA_CONFIG_T
* @param unMaxCameraCount maximum number of cameras
* @param unTcpPortBase base TCP port number
* @param unRtspPortBase base RTSP port number
* @param unPortOffset port offset between cameras
* @param bDefaultAvailable default camera availability status
*/
typedef struct DI_VIDEO_CAMERA_CONFIG_t {
    uint32_t                        unMaxCameraCount;
    uint32_t                        unTcpPortBase;
    uint32_t                        unRtspPortBase;
    uint32_t                        unPortOffset;
    bool                            bDefaultAvailable;
} DI_VIDEO_CAMERA_CONFIG_T;

/**
* @details DI_VIDEO_CAMERA_T
* @param unCameraId camera identifier (1-6)
* @param unTcpPort assigned TCP port
* @param unRtspPort assigned RTSP port
* @param bCameraAvailable camera availability status
* @param stDiVideoNvidia video nvidia instance
*/
typedef struct DI_VIDEO_CAMERA_t {
    uint32_t                        unCameraId;
    uint32_t                        unTcpPort;
    uint32_t                        unRtspPort;
    bool                            bCameraAvailable;
    DI_VIDEO_NVIDIA_T               stDiVideoNvidia;
} DI_VIDEO_CAMERA_T;

/**
* @details DI_VIDEO_PIPELINE_INFO_T - GStreamer pipeline status information
* @param unWidth current pipeline width
* @param unHeight current pipeline height  
* @param unFrameRate current frame rate
* @param unBitrate current bitrate
* @param achCodec codec name string
* @param achFormat format name string
* @param eGstState GStreamer pipeline state
* @param bIsActive pipeline active status
* @param ullFramesProcessed total processed frames (cumulative from GStreamer probes)
* @param ullDroppedFrames dropped frames count (cumulative from GStreamer probes)
* @param ullUdpBytes UDP bytes transmitted/received (cumulative from GStreamer probes)
* @param dRealFrameRate current measured frame rate (frames/sec)
* @param dCurrentByteRate current transfer rate (bytes/sec)
* @param unLatencyMs current latency in milliseconds
*/
typedef struct DI_VIDEO_PIPELINE_INFO_t {
    uint32_t unWidth;
    uint32_t unHeight;
    uint32_t unFrameRate;
    uint32_t unBitrate;
    char achCodec[32];
    char achFormat[32];
    uint32_t unGstState;                      /* GstState as uint32_t for header compatibility */
    bool bIsActive;
    uint64_t ullFramesProcessed;              /* Total frame count (cumulative) */
    uint64_t ullDroppedFrames;                /* Total dropped frame count (cumulative) */
    uint64_t ullUdpBytes;                     /* Total UDP bytes (cumulative) */
    double dRealFrameRate;                    /* Current frame rate (frames/sec) */
    double dCurrentByteRate;                  /* Current transfer rate (bytes/sec) */
    uint32_t unLatencyMs;
} DI_VIDEO_PIPELINE_INFO_T;

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

#if defined(CONFIG_VIDEO_STREAMING)
/* TCP streaming functions */
int32_t DI_VIDEO_NVIDIA_SetTcpConnection(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, const char *pchRemoteHost, int32_t nRemotePort, int32_t nLocalPort);
int32_t DI_VIDEO_NVIDIA_StartTxMode(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel);
int32_t DI_VIDEO_NVIDIA_StartRxMode(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel);
int32_t DI_VIDEO_NVIDIA_CheckTcpConnection(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia);

/* UDP protocol functions */
int32_t DI_VIDEO_NVIDIA_SetUdpProtocol(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, uint32_t unProtocol, const char *pchRemoteHost, uint32_t unUdpPort);

/* Pipeline monitoring functions */
int32_t DI_VIDEO_NVIDIA_GetPipelineInfo(DI_VIDEO_PIPELINE_INFO_T *pstTxInfo, DI_VIDEO_PIPELINE_INFO_T *pstRxInfo);
int32_t DI_VIDEO_NVIDIA_GetElementStats(const char *pchElementName, uint64_t *pullFrameCount, uint32_t *punLatency);
#endif

#endif	/* _DI_VIDEO_NVIDIA_H_ */


