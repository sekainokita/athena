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
* @file di_camera.c
*
* This file contains a data format design
*
* @note
*
* V2X DI Camera Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  25.01.08 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include "di.h"
#include "di_camera.h"

#if defined(CONFIG_CAMERA_NVIDIA)
#include "di_camera_nvidia.h"
#endif

#if defined(CONFIG_VIDEO_STREAMING)
#include "di_ring_buffer.h"
#include "di_error.h"
#include "di_memory_pool.h"
#endif

/***************************** Definition ************************************/
#define DI_CAMERA_DEFAULT_DEVICE_ID         (0)
#define DI_CAMERA_FRAME_BUFFER_CNT          (3)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiCameraLog = OFF;
static int32_t s_nDiCameraTaskMsgId = -1;
static key_t s_DiCameraTaskMsgKey = DI_CAMERA_TASK_MSG_KEY;
static pthread_t s_hDiCameraTask;
static pthread_mutex_t s_stDiCameraMutex = PTHREAD_MUTEX_INITIALIZER;

/* Frame buffer management */
static DI_CAMERA_FRAME_T s_astDiCameraFrames[DI_CAMERA_FRAME_BUFFER_CNT];
static uint32_t s_unFrameSeq = 0;

#if defined(CONFIG_CAMERA_NVIDIA)
static DI_CAMERA_NVIDIA_T s_stDiCameraDev;
#endif

#if defined(CONFIG_VIDEO_STREAMING)
/* Video streaming integration */
static DI_RING_BUFFER_T *s_pstCameraRingBuffer = NULL;
static bool s_bVideoStreamingEnabled = FALSE;
#endif

/***************************** Function  *************************************/

static void *P_DI_CAMERA_Task(void *pvArg)
{
    DI_CAMERA_EVENT_MSG_T stEventMsg;
    int32_t nRet = DI_ERROR;
    
    UNUSED(pvArg);
    
    memset(&stEventMsg, 0, sizeof(DI_CAMERA_EVENT_MSG_T));
    
    while (1)
    {
        if (msgrcv(s_nDiCameraTaskMsgId, &stEventMsg, sizeof(DI_CAMERA_EVENT_MSG_T), 0, MSG_NOERROR) == DI_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch (stEventMsg.eEventType)
            {
                case eDI_CAMERA_EVENT_START:
                {
                    PrintTrace("eDI_CAMERA_EVENT_START is received [DeviceId:%d]", stEventMsg.unDeviceId);
                    break;
                }
                
                case eDI_CAMERA_EVENT_STOP:
                {
                    PrintTrace("eDI_CAMERA_EVENT_STOP is received [DeviceId:%d]", stEventMsg.unDeviceId);
                    break;
                }
                
                case eDI_CAMERA_EVENT_CAPTURE:
                {
                    PrintTrace("eDI_CAMERA_EVENT_CAPTURE is received [DeviceId:%d]", stEventMsg.unDeviceId);
                    break;
                }
                
                default:
                {
                    PrintWarn("Unknown event type [%d]", stEventMsg.eEventType);
                    break;
                }
            }
        }
        
        usleep(1000);
    }
    
    return NULL;
}

static void P_DI_CAMERA_PrintMsgInfo(int32_t nMsqId)
{
    struct msqid_ds stMsgStat;
    
    PrintDebug("========== Camera Message Queue Information =============");
    
    if (msgctl(nMsqId, IPC_STAT, &stMsgStat) == DI_MSG_ERR)
    {
        PrintError("msgctl() is failed!");
    }
    else
    {
        PrintDebug("msg_lspid : %d", stMsgStat.msg_lspid);
        PrintDebug("msg_qnum : %ld", stMsgStat.msg_qnum);
        PrintDebug("msg_stime : %ld", stMsgStat.msg_stime);
    }
    
    PrintDebug("====================================================");
}

static int32_t P_DI_CAMERA_CreateTask(void)
{
    int32_t nRet = DI_ERROR;
    pthread_attr_t stAttr;
    
    pthread_attr_init(&stAttr);
    pthread_attr_setdetachstate(&stAttr, PTHREAD_CREATE_DETACHED);
    
    nRet = pthread_create(&s_hDiCameraTask, &stAttr, P_DI_CAMERA_Task, NULL);
    if (nRet != DI_OK)
    {
        PrintError("pthread_create() is failed! (P_DI_CAMERA_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DI_CAMERA_Task() is successfully created");
        nRet = DI_OK;
    }
    
    pthread_attr_destroy(&stAttr);
    
    return nRet;
}

static int32_t P_DI_CAMERA_Init(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        PrintWarn("bCameraNotAvailable[%d]", pstDiCamera->bCameraNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    /* Initialize frame buffers */
    memset(s_astDiCameraFrames, 0, sizeof(s_astDiCameraFrames));
    s_unFrameSeq = 0;
    
    /* Initialize device abstraction layer */
#if defined(CONFIG_CAMERA_NVIDIA)
    nRet = DI_CAMERA_NVIDIA_Init(&s_stDiCameraDev);
    if (nRet != DI_OK)
    {
        PrintError("DI_CAMERA_NVIDIA_Init() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
#else
    PrintWarn("None of camera devices are supported");
    nRet = DI_OK;
#endif
    
    /* Create message queue */
    s_nDiCameraTaskMsgId = msgget(s_DiCameraTaskMsgKey, IPC_CREAT | 0666);
    if (s_nDiCameraTaskMsgId == DI_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        nRet = DI_ERROR;
        goto EXIT;
    }
    else
    {
        P_DI_CAMERA_PrintMsgInfo(s_nDiCameraTaskMsgId);
    }
    
    /* Create task */
    nRet = P_DI_CAMERA_CreateTask();
    if (nRet != DI_OK)
    {
        PrintError("P_DI_CAMERA_CreateTask() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

static int32_t P_DI_CAMERA_DeInit(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        PrintWarn("bCameraNotAvailable[%d]", pstDiCamera->bCameraNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    /* Deinitialize device abstraction layer */
#if defined(CONFIG_CAMERA_NVIDIA)
    nRet = DI_CAMERA_NVIDIA_DeInit(&s_stDiCameraDev);
    if (nRet != DI_OK)
    {
        PrintError("DI_CAMERA_NVIDIA_DeInit() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    memset(&s_stDiCameraDev, 0, sizeof(DI_CAMERA_NVIDIA_T));
#else
    PrintWarn("None of camera devices are supported");
    nRet = DI_OK;
#endif
    
    /* Clean up frame buffers */
    memset(s_astDiCameraFrames, 0, sizeof(s_astDiCameraFrames));
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_Init(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    /* Set default configuration */
    pstDiCamera->stDiCameraConfig.unDeviceId = DI_CAMERA_DEFAULT_DEVICE_ID;
    pstDiCamera->stDiCameraConfig.unWidth = DI_CAMERA_DEFAULT_WIDTH;
    pstDiCamera->stDiCameraConfig.unHeight = DI_CAMERA_DEFAULT_HEIGHT;
    pstDiCamera->stDiCameraConfig.unFps = DI_CAMERA_DEFAULT_FPS;
    pstDiCamera->stDiCameraConfig.eFormat = eDI_CAMERA_FORMAT_YUV420;
    pstDiCamera->stDiCameraConfig.bHardwareAccel = TRUE;
    
    nRet = P_DI_CAMERA_Init(pstDiCamera);
    if (nRet != DI_OK)
    {
        PrintError("P_DI_CAMERA_Init() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_INITIALIZED;
    s_bDiCameraLog = pstDiCamera->bLogLevel;
    
    PrintTrace("DI_CAMERA is successfully initialized");
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_DeInit(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    nRet = P_DI_CAMERA_DeInit(pstDiCamera);
    if (nRet != DI_OK)
    {
        PrintError("P_DI_CAMERA_DeInit() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_DEINITIALIZED;
    
    PrintTrace("DI_CAMERA is successfully deinitialized");
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_SetLog(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    s_bDiCameraLog = pstDiCamera->bLogLevel;
    PrintTrace("SET:s_bDiCameraLog [%s]", s_bDiCameraLog == ON ? "ON" : "OFF");
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_SetConfig(DI_CAMERA_T *pstDiCamera, DI_CAMERA_CONFIG_T *pstConfig)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiCamera == NULL) || (pstConfig == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiCameraMutex);
    
    memcpy(&pstDiCamera->stDiCameraConfig, pstConfig, sizeof(DI_CAMERA_CONFIG_T));
    
    pthread_mutex_unlock(&s_stDiCameraMutex);
    
    PrintTrace("Camera config updated: DeviceId[%d] Resolution[%dx%d] FPS[%d] Format[%d]",
               pstConfig->unDeviceId, pstConfig->unWidth, pstConfig->unHeight, 
               pstConfig->unFps, pstConfig->eFormat);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_GetConfig(DI_CAMERA_T *pstDiCamera, DI_CAMERA_CONFIG_T *pstConfig)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiCamera == NULL) || (pstConfig == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiCameraMutex);
    
    memcpy(pstConfig, &pstDiCamera->stDiCameraConfig, sizeof(DI_CAMERA_CONFIG_T));
    
    pthread_mutex_unlock(&s_stDiCameraMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_SetNa(DI_CAMERA_T *pstDiCamera, bool bNotAvailable)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    pstDiCamera->bCameraNotAvailable = bNotAvailable;
    PrintTrace("Camera availability set to [%s]", bNotAvailable ? "NOT_AVAILABLE" : "AVAILABLE");
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_Open(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        PrintWarn("bCameraNotAvailable[%d]", pstDiCamera->bCameraNotAvailable);
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_OPENED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if ((pstDiCamera->eDiCameraStatus == DI_CAMERA_STATUS_INITIALIZED) || 
        (pstDiCamera->eDiCameraStatus == DI_CAMERA_STATUS_CLOSED))
    {
#if defined(CONFIG_CAMERA_NVIDIA)
        nRet = DI_CAMERA_NVIDIA_Open(&s_stDiCameraDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_CAMERA_NVIDIA_Open() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_OPENED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of CAMERA [%d]", pstDiCamera->eDiCameraStatus);
        
        if (pstDiCamera->eDiCameraStatus == DI_CAMERA_STATUS_OPENED)
        {
            PrintDebug("Already DI_CAMERA_STATUS_OPENED");
            nRet = DI_OK;
        }
    }
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_Close(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        PrintWarn("bCameraNotAvailable[%d]", pstDiCamera->bCameraNotAvailable);
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_CLOSED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiCamera->eDiCameraStatus == DI_CAMERA_STATUS_OPENED)
    {
#if defined(CONFIG_CAMERA_NVIDIA)
        nRet = DI_CAMERA_NVIDIA_Close(&s_stDiCameraDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_CAMERA_NVIDIA_Close() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_CLOSED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of CAMERA [%d]", pstDiCamera->eDiCameraStatus);
        
        if (pstDiCamera->eDiCameraStatus == DI_CAMERA_STATUS_CLOSED)
        {
            PrintDebug("Already DI_CAMERA_STATUS_CLOSED");
            nRet = DI_OK;
        }
    }
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_Start(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        PrintWarn("bCameraNotAvailable[%d]", pstDiCamera->bCameraNotAvailable);
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_STARTED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiCamera->eDiCameraStatus == DI_CAMERA_STATUS_OPENED)
    {
#if defined(CONFIG_CAMERA_NVIDIA)
        nRet = DI_CAMERA_NVIDIA_Start(&s_stDiCameraDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_CAMERA_NVIDIA_Start() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_STARTED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of CAMERA [%d]", pstDiCamera->eDiCameraStatus);
    }
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_Stop(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        PrintWarn("bCameraNotAvailable[%d]", pstDiCamera->bCameraNotAvailable);
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_STOPPED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiCamera->eDiCameraStatus == DI_CAMERA_STATUS_STARTED)
    {
#if defined(CONFIG_CAMERA_NVIDIA)
        nRet = DI_CAMERA_NVIDIA_Stop(&s_stDiCameraDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_CAMERA_NVIDIA_Stop() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiCamera->eDiCameraStatus = DI_CAMERA_STATUS_STOPPED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of CAMERA [%d]", pstDiCamera->eDiCameraStatus);
    }
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_GetFrame(DI_CAMERA_T *pstDiCamera, DI_CAMERA_FRAME_T *pstFrame)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiCamera == NULL) || (pstFrame == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        memset(pstFrame, 0, sizeof(DI_CAMERA_FRAME_T));
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiCamera->eDiCameraStatus >= DI_CAMERA_STATUS_STARTED)
    {
        pthread_mutex_lock(&s_stDiCameraMutex);
        
#if defined(CONFIG_CAMERA_NVIDIA)
        nRet = DI_CAMERA_NVIDIA_GetFrame(&s_stDiCameraDev, pstFrame);
        if (nRet != DI_OK)
        {
            PrintError("DI_CAMERA_NVIDIA_GetFrame() is failed! [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stDiCameraMutex);
            goto EXIT;
        }
#endif
        
        /* Update frame sequence number */
        pstFrame->unFrameSeq = ++s_unFrameSeq;
        pstFrame->unTimestamp = (uint32_t)time(NULL);
        
#if defined(CONFIG_VIDEO_STREAMING)
        /* Send frame to video streaming if enabled */
        if (s_bVideoStreamingEnabled && s_pstCameraRingBuffer != NULL && 
            pstFrame->puchData != NULL && pstFrame->unDataSize > 0)
        {
            int32_t nStreamRet = DI_RING_BUFFER_Write(s_pstCameraRingBuffer, 
                                                      pstFrame->puchData, 
                                                      pstFrame->unDataSize);
            if (nStreamRet != DI_OK)
            {
                PrintWarn("Failed to write frame to video streaming buffer [nRet:%d]", nStreamRet);
            }
        }
#endif
        
        pthread_mutex_unlock(&s_stDiCameraMutex);
        
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of CAMERA [%d]", pstDiCamera->eDiCameraStatus);
    }
    
EXIT:
    return nRet;
}

int32_t DI_CAMERA_ReleaseFrame(DI_CAMERA_T *pstDiCamera, DI_CAMERA_FRAME_T *pstFrame)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiCamera == NULL) || (pstFrame == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    if (pstDiCamera->bCameraNotAvailable == TRUE)
    {
        nRet = DI_OK;
        goto EXIT;
    }
    
#if defined(CONFIG_CAMERA_NVIDIA)
    nRet = DI_CAMERA_NVIDIA_ReleaseFrame(&s_stDiCameraDev, pstFrame);
    if (nRet != DI_OK)
    {
        PrintError("DI_CAMERA_NVIDIA_ReleaseFrame() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
#endif
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

void DI_CAMERA_Status(DI_CAMERA_T *pstDiCamera)
{
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        return;
    }
    
    PrintDebug("========== Camera Status Information =============");
    PrintDebug("Camera Status: %d", pstDiCamera->eDiCameraStatus);
    PrintDebug("Camera Available: %s", pstDiCamera->bCameraNotAvailable ? "NO" : "YES");
    PrintDebug("Device ID: %d", pstDiCamera->stDiCameraConfig.unDeviceId);
    PrintDebug("Resolution: %dx%d", pstDiCamera->stDiCameraConfig.unWidth, pstDiCamera->stDiCameraConfig.unHeight);
    PrintDebug("FPS: %d", pstDiCamera->stDiCameraConfig.unFps);
    PrintDebug("Format: %d", pstDiCamera->stDiCameraConfig.eFormat);
    PrintDebug("Hardware Acceleration: %s", pstDiCamera->stDiCameraConfig.bHardwareAccel ? "YES" : "NO");
    PrintDebug("Log Level: %s", s_bDiCameraLog == ON ? "ON" : "OFF");
    PrintDebug("Frame Sequence: %d", s_unFrameSeq);
#if defined(CONFIG_VIDEO_STREAMING)
    PrintDebug("Video Streaming: %s", s_bVideoStreamingEnabled ? "ENABLED" : "DISABLED");
#endif
    PrintDebug("================================================");
}

#if defined(CONFIG_VIDEO_STREAMING)
/*
 * Connect camera to video streaming ring buffer
 */
int32_t DI_CAMERA_ConnectVideoStreaming(DI_CAMERA_T *pstDiCamera, DI_RING_BUFFER_T *pstRingBuffer)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL || pstRingBuffer == NULL)
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiCameraMutex);
    
    s_pstCameraRingBuffer = pstRingBuffer;
    s_bVideoStreamingEnabled = TRUE;
    
    PrintTrace("Camera connected to video streaming ring buffer");
    
    pthread_mutex_unlock(&s_stDiCameraMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Disconnect camera from video streaming
 */
int32_t DI_CAMERA_DisconnectVideoStreaming(DI_CAMERA_T *pstDiCamera)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiCamera == NULL)
    {
        PrintError("pstDiCamera == NULL!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiCameraMutex);
    
    s_pstCameraRingBuffer = NULL;
    s_bVideoStreamingEnabled = FALSE;
    
    PrintTrace("Camera disconnected from video streaming");
    
    pthread_mutex_unlock(&s_stDiCameraMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}
#endif /* CONFIG_VIDEO_STREAMING */