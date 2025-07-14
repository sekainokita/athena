#ifndef _DI_CAMERA_NVIDIA_H_
#define _DI_CAMERA_NVIDIA_H_

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
* @file di_camera_nvidia.h
*
* @note
*
* DI Camera NVIDIA Header for V2X Video Streaming
*
******************************************************************************/

/***************************** Include ***************************************/
#include <stdint.h>
#include <stdbool.h>

/***************************** Definition ************************************/

/* Camera configuration */
#define DI_CAMERA_NVIDIA_DEFAULT_WIDTH        (1920)
#define DI_CAMERA_NVIDIA_DEFAULT_HEIGHT       (1080)
#define DI_CAMERA_NVIDIA_DEFAULT_FPS          (30)
#define DI_CAMERA_NVIDIA_MAX_DEVICES          (4)

/***************************** Enum and Structure ****************************/

/**
* @details DI Camera NVIDIA Status
*/
typedef enum {
    DI_CAMERA_NVIDIA_STATUS_UNKNOWN          = 0,
    DI_CAMERA_NVIDIA_STATUS_STOPPED          = 1,
    DI_CAMERA_NVIDIA_STATUS_RUNNING          = 2,
    DI_CAMERA_NVIDIA_STATUS_ERROR            = 3,
    DI_CAMERA_NVIDIA_STATUS_MAX              = 255
} DI_CAMERA_NVIDIA_STATUS_E;

/**
* @details DI Camera NVIDIA Configuration
*/
typedef struct DI_CAMERA_NVIDIA_CONFIG_t {
    uint32_t unDeviceIndex;                   /* Camera device index */
    uint32_t unWidth;                         /* Frame width */
    uint32_t unHeight;                        /* Frame height */
    uint32_t unFrameRate;                     /* Frame rate */
    uint32_t unBitrate;                       /* Bitrate for encoding */
    bool bAutoFocus;                          /* Auto focus enable */
    bool bAutoExposure;                       /* Auto exposure enable */
} DI_CAMERA_NVIDIA_CONFIG_T;

/**
* @details DI Camera NVIDIA Frame Data
*/
typedef struct DI_CAMERA_NVIDIA_FRAME_t {
    uint8_t *puchData;                        /* Frame data buffer */
    uint32_t unDataSize;                      /* Frame data size */
    uint32_t unWidth;                         /* Frame width */
    uint32_t unHeight;                        /* Frame height */
    uint32_t unFrameIndex;                    /* Frame index */
    uint64_t ullTimestamp;                    /* Frame timestamp */
} DI_CAMERA_NVIDIA_FRAME_T;

/**
* @details DI Camera NVIDIA Device Structure
*/
typedef struct DI_CAMERA_NVIDIA_t {
    uint32_t unDeviceIndex;                   /* Device index */
    DI_CAMERA_NVIDIA_STATUS_E eStatus;       /* Current status */
    DI_CAMERA_NVIDIA_CONFIG_T stConfig;      /* Configuration */
    bool bIsInitialized;                      /* Initialization flag */
    void *pvPrivateData;                      /* Private data */
} DI_CAMERA_NVIDIA_T;

/***************************** Function Prototype ****************************/

/* Camera initialization and cleanup */
int32_t DI_CAMERA_NVIDIA_Init(DI_CAMERA_NVIDIA_T *pstCameraDev);
int32_t DI_CAMERA_NVIDIA_DeInit(DI_CAMERA_NVIDIA_T *pstCameraDev);

/* Camera device control */
int32_t DI_CAMERA_NVIDIA_Open(DI_CAMERA_NVIDIA_T *pstCameraDev);
int32_t DI_CAMERA_NVIDIA_Close(DI_CAMERA_NVIDIA_T *pstCameraDev);

/* Camera streaming control */
int32_t DI_CAMERA_NVIDIA_Start(DI_CAMERA_NVIDIA_T *pstCameraDev);
int32_t DI_CAMERA_NVIDIA_Stop(DI_CAMERA_NVIDIA_T *pstCameraDev);

/* Frame operations */
int32_t DI_CAMERA_NVIDIA_GetFrame(DI_CAMERA_NVIDIA_T *pstCameraDev, 
                                  DI_CAMERA_NVIDIA_FRAME_T *pstFrame);
int32_t DI_CAMERA_NVIDIA_ReleaseFrame(DI_CAMERA_NVIDIA_T *pstCameraDev, 
                                      DI_CAMERA_NVIDIA_FRAME_T *pstFrame);

/* Camera configuration */
int32_t DI_CAMERA_NVIDIA_SetConfig(const DI_CAMERA_NVIDIA_CONFIG_T *pstConfig);
int32_t DI_CAMERA_NVIDIA_GetConfig(DI_CAMERA_NVIDIA_CONFIG_T *pstConfig);

/* Camera status */
int32_t DI_CAMERA_NVIDIA_GetStatus(DI_CAMERA_NVIDIA_STATUS_E *peStatus);

#endif /* _DI_CAMERA_NVIDIA_H_ */