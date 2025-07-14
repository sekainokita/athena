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
* @file di_display.c
*
* This file contains a data format design
*
* @note
*
* V2X DI Display Source File
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
#include "di_display.h"

#if defined(CONFIG_DISPLAY_NVIDIA)
#include "di_display_nvidia.h"
#endif

/***************************** Definition ************************************/
#define DI_DISPLAY_DEFAULT_DEVICE_ID         (0)
#define DI_DISPLAY_OVERLAY_MAX_COUNT         (16)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiDisplayLog = OFF;
static int32_t s_nDiDisplayTaskMsgId = -1;
static key_t s_DiDisplayTaskMsgKey = DI_DISPLAY_TASK_MSG_KEY;
static pthread_t s_hDiDisplayTask;
static pthread_mutex_t s_stDiDisplayMutex = PTHREAD_MUTEX_INITIALIZER;

/* Overlay management */
static DI_DISPLAY_OVERLAY_T s_astDiDisplayOverlays[DI_DISPLAY_OVERLAY_MAX_COUNT];
static uint32_t s_unOverlayCount = 0;

#if defined(CONFIG_DISPLAY_NVIDIA)
static DI_DISPLAY_NVIDIA_T s_stDiDisplayDev;
#endif

/***************************** Function  *************************************/

static void *P_DI_DISPLAY_Task(void *pvArg)
{
    DI_DISPLAY_EVENT_MSG_T stEventMsg;
    int32_t nRet = DI_ERROR;
    
    UNUSED(pvArg);
    
    memset(&stEventMsg, 0, sizeof(DI_DISPLAY_EVENT_MSG_T));
    
    while (1)
    {
        if (msgrcv(s_nDiDisplayTaskMsgId, &stEventMsg, sizeof(DI_DISPLAY_EVENT_MSG_T), 0, MSG_NOERROR) == DI_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch (stEventMsg.eEventType)
            {
                case eDI_DISPLAY_EVENT_START:
                {
                    PrintTrace("eDI_DISPLAY_EVENT_START is received [DeviceId:%d]", stEventMsg.unDeviceId);
                    break;
                }
                
                case eDI_DISPLAY_EVENT_STOP:
                {
                    PrintTrace("eDI_DISPLAY_EVENT_STOP is received [DeviceId:%d]", stEventMsg.unDeviceId);
                    break;
                }
                
                case eDI_DISPLAY_EVENT_RENDER:
                {
                    PrintTrace("eDI_DISPLAY_EVENT_RENDER is received [DeviceId:%d]", stEventMsg.unDeviceId);
                    break;
                }
                
                case eDI_DISPLAY_EVENT_OVERLAY:
                {
                    PrintTrace("eDI_DISPLAY_EVENT_OVERLAY is received [DeviceId:%d]", stEventMsg.unDeviceId);
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

static void P_DI_DISPLAY_PrintMsgInfo(int32_t nMsqId)
{
    struct msqid_ds stMsgStat;
    
    PrintDebug("========== Display Message Queue Information =============");
    
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
    
    PrintDebug("======================================================");
}

static int32_t P_DI_DISPLAY_CreateTask(void)
{
    int32_t nRet = DI_ERROR;
    pthread_attr_t stAttr;
    
    pthread_attr_init(&stAttr);
    pthread_attr_setdetachstate(&stAttr, PTHREAD_CREATE_DETACHED);
    
    nRet = pthread_create(&s_hDiDisplayTask, &stAttr, P_DI_DISPLAY_Task, NULL);
    if (nRet != DI_OK)
    {
        PrintError("pthread_create() is failed! (P_DI_DISPLAY_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DI_DISPLAY_Task() is successfully created");
        nRet = DI_OK;
    }
    
    pthread_attr_destroy(&stAttr);
    
    return nRet;
}

static int32_t P_DI_DISPLAY_Init(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        PrintWarn("bDisplayNotAvailable[%d]", pstDiDisplay->bDisplayNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    /* Initialize overlay management */
    memset(s_astDiDisplayOverlays, 0, sizeof(s_astDiDisplayOverlays));
    s_unOverlayCount = 0;
    
    /* Initialize device abstraction layer */
#if defined(CONFIG_DISPLAY_NVIDIA)
    nRet = DI_DISPLAY_NVIDIA_Init(&s_stDiDisplayDev);
    if (nRet != DI_OK)
    {
        PrintError("DI_DISPLAY_NVIDIA_Init() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
#else
    PrintWarn("None of display devices are supported");
    nRet = DI_OK;
#endif
    
    /* Create message queue */
    s_nDiDisplayTaskMsgId = msgget(s_DiDisplayTaskMsgKey, IPC_CREAT | 0666);
    if (s_nDiDisplayTaskMsgId == DI_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        nRet = DI_ERROR;
        goto EXIT;
    }
    else
    {
        P_DI_DISPLAY_PrintMsgInfo(s_nDiDisplayTaskMsgId);
    }
    
    /* Create task */
    nRet = P_DI_DISPLAY_CreateTask();
    if (nRet != DI_OK)
    {
        PrintError("P_DI_DISPLAY_CreateTask() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

static int32_t P_DI_DISPLAY_DeInit(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        PrintWarn("bDisplayNotAvailable[%d]", pstDiDisplay->bDisplayNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    /* Deinitialize device abstraction layer */
#if defined(CONFIG_DISPLAY_NVIDIA)
    nRet = DI_DISPLAY_NVIDIA_DeInit(&s_stDiDisplayDev);
    if (nRet != DI_OK)
    {
        PrintError("DI_DISPLAY_NVIDIA_DeInit() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    memset(&s_stDiDisplayDev, 0, sizeof(DI_DISPLAY_NVIDIA_T));
#else
    PrintWarn("None of display devices are supported");
    nRet = DI_OK;
#endif
    
    /* Clean up overlay management */
    memset(s_astDiDisplayOverlays, 0, sizeof(s_astDiDisplayOverlays));
    s_unOverlayCount = 0;
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_Init(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    /* Set default configuration */
    pstDiDisplay->stDiDisplayConfig.unDeviceId = DI_DISPLAY_DEFAULT_DEVICE_ID;
    pstDiDisplay->stDiDisplayConfig.unWidth = DI_DISPLAY_DEFAULT_WIDTH;
    pstDiDisplay->stDiDisplayConfig.unHeight = DI_DISPLAY_DEFAULT_HEIGHT;
    pstDiDisplay->stDiDisplayConfig.bFullScreen = TRUE;
    pstDiDisplay->stDiDisplayConfig.bHardwareAccel = TRUE;
    pstDiDisplay->stDiDisplayConfig.bOverlayEnabled = TRUE;
    
    nRet = P_DI_DISPLAY_Init(pstDiDisplay);
    if (nRet != DI_OK)
    {
        PrintError("P_DI_DISPLAY_Init() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_INITIALIZED;
    s_bDiDisplayLog = pstDiDisplay->bLogLevel;
    
    PrintTrace("DI_DISPLAY is successfully initialized");
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_DeInit(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    nRet = P_DI_DISPLAY_DeInit(pstDiDisplay);
    if (nRet != DI_OK)
    {
        PrintError("P_DI_DISPLAY_DeInit() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_DEINITIALIZED;
    
    PrintTrace("DI_DISPLAY is successfully deinitialized");
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_SetLog(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    s_bDiDisplayLog = pstDiDisplay->bLogLevel;
    PrintTrace("SET:s_bDiDisplayLog [%s]", s_bDiDisplayLog == ON ? "ON" : "OFF");
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_SetConfig(DI_DISPLAY_T *pstDiDisplay, DI_DISPLAY_CONFIG_T *pstConfig)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiDisplay == NULL) || (pstConfig == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiDisplayMutex);
    
    memcpy(&pstDiDisplay->stDiDisplayConfig, pstConfig, sizeof(DI_DISPLAY_CONFIG_T));
    
    pthread_mutex_unlock(&s_stDiDisplayMutex);
    
    PrintTrace("Display config updated: DeviceId[%d] Resolution[%dx%d] FullScreen[%s] HWAccel[%s] Overlay[%s]",
               pstConfig->unDeviceId, pstConfig->unWidth, pstConfig->unHeight,
               pstConfig->bFullScreen ? "YES" : "NO",
               pstConfig->bHardwareAccel ? "YES" : "NO",
               pstConfig->bOverlayEnabled ? "YES" : "NO");
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_GetConfig(DI_DISPLAY_T *pstDiDisplay, DI_DISPLAY_CONFIG_T *pstConfig)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiDisplay == NULL) || (pstConfig == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiDisplayMutex);
    
    memcpy(pstConfig, &pstDiDisplay->stDiDisplayConfig, sizeof(DI_DISPLAY_CONFIG_T));
    
    pthread_mutex_unlock(&s_stDiDisplayMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_SetNa(DI_DISPLAY_T *pstDiDisplay, bool bNotAvailable)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    pstDiDisplay->bDisplayNotAvailable = bNotAvailable;
    PrintTrace("Display availability set to [%s]", bNotAvailable ? "NOT_AVAILABLE" : "AVAILABLE");
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_Open(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        PrintWarn("bDisplayNotAvailable[%d]", pstDiDisplay->bDisplayNotAvailable);
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_OPENED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if ((pstDiDisplay->eDiDisplayStatus == DI_DISPLAY_STATUS_INITIALIZED) || 
        (pstDiDisplay->eDiDisplayStatus == DI_DISPLAY_STATUS_CLOSED))
    {
#if defined(CONFIG_DISPLAY_NVIDIA)
        nRet = DI_DISPLAY_NVIDIA_Open(&s_stDiDisplayDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_DISPLAY_NVIDIA_Open() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_OPENED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of DISPLAY [%d]", pstDiDisplay->eDiDisplayStatus);
        
        if (pstDiDisplay->eDiDisplayStatus == DI_DISPLAY_STATUS_OPENED)
        {
            PrintDebug("Already DI_DISPLAY_STATUS_OPENED");
            nRet = DI_OK;
        }
    }
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_Close(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        PrintWarn("bDisplayNotAvailable[%d]", pstDiDisplay->bDisplayNotAvailable);
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_CLOSED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiDisplay->eDiDisplayStatus == DI_DISPLAY_STATUS_OPENED)
    {
#if defined(CONFIG_DISPLAY_NVIDIA)
        nRet = DI_DISPLAY_NVIDIA_Close(&s_stDiDisplayDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_DISPLAY_NVIDIA_Close() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_CLOSED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of DISPLAY [%d]", pstDiDisplay->eDiDisplayStatus);
        
        if (pstDiDisplay->eDiDisplayStatus == DI_DISPLAY_STATUS_CLOSED)
        {
            PrintDebug("Already DI_DISPLAY_STATUS_CLOSED");
            nRet = DI_OK;
        }
    }
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_Start(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        PrintWarn("bDisplayNotAvailable[%d]", pstDiDisplay->bDisplayNotAvailable);
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_STARTED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiDisplay->eDiDisplayStatus == DI_DISPLAY_STATUS_OPENED)
    {
#if defined(CONFIG_DISPLAY_NVIDIA)
        nRet = DI_DISPLAY_NVIDIA_Start(&s_stDiDisplayDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_DISPLAY_NVIDIA_Start() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_STARTED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of DISPLAY [%d]", pstDiDisplay->eDiDisplayStatus);
    }
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_Stop(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        PrintWarn("bDisplayNotAvailable[%d]", pstDiDisplay->bDisplayNotAvailable);
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_STOPPED;
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiDisplay->eDiDisplayStatus == DI_DISPLAY_STATUS_STARTED)
    {
#if defined(CONFIG_DISPLAY_NVIDIA)
        nRet = DI_DISPLAY_NVIDIA_Stop(&s_stDiDisplayDev);
        if (nRet != DI_OK)
        {
            PrintError("DI_DISPLAY_NVIDIA_Stop() is failed! [nRet:%d]", nRet);
            goto EXIT;
        }
#endif
        pstDiDisplay->eDiDisplayStatus = DI_DISPLAY_STATUS_STOPPED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of DISPLAY [%d]", pstDiDisplay->eDiDisplayStatus);
    }
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_RenderFrame(DI_DISPLAY_T *pstDiDisplay, DI_CAMERA_FRAME_T *pstFrame)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiDisplay == NULL) || (pstFrame == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (pstDiDisplay->eDiDisplayStatus >= DI_DISPLAY_STATUS_STARTED)
    {
        pthread_mutex_lock(&s_stDiDisplayMutex);
        
#if defined(CONFIG_DISPLAY_NVIDIA)
        nRet = DI_DISPLAY_NVIDIA_RenderFrame(&s_stDiDisplayDev, pstFrame);
        if (nRet != DI_OK)
        {
            PrintError("DI_DISPLAY_NVIDIA_RenderFrame() is failed! [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stDiDisplayMutex);
            goto EXIT;
        }
#endif
        
        /* Apply overlays if enabled */
        if (pstDiDisplay->stDiDisplayConfig.bOverlayEnabled == TRUE)
        {
            /* Overlay rendering logic will be handled by hardware layer */
        }
        
        pthread_mutex_unlock(&s_stDiDisplayMutex);
        
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("Check the status of DISPLAY [%d]", pstDiDisplay->eDiDisplayStatus);
    }
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_SetOverlay(DI_DISPLAY_T *pstDiDisplay, DI_DISPLAY_OVERLAY_T *pstOverlay)
{
    int32_t nRet = DI_ERROR;
    
    if ((pstDiDisplay == NULL) || (pstOverlay == NULL))
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        nRet = DI_OK;
        goto EXIT;
    }
    
    if (s_unOverlayCount >= DI_DISPLAY_OVERLAY_MAX_COUNT)
    {
        PrintWarn("Maximum overlay count reached [%d]", DI_DISPLAY_OVERLAY_MAX_COUNT);
        nRet = DI_ERROR;
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiDisplayMutex);
    
    memcpy(&s_astDiDisplayOverlays[s_unOverlayCount], pstOverlay, sizeof(DI_DISPLAY_OVERLAY_T));
    s_unOverlayCount++;
    
    pthread_mutex_unlock(&s_stDiDisplayMutex);
    
    PrintTrace("Overlay added: Type[%d] Pos[%d,%d] Size[%dx%d]",
               pstOverlay->eType, pstOverlay->unPosX, pstOverlay->unPosY,
               pstOverlay->unWidth, pstOverlay->unHeight);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

int32_t DI_DISPLAY_ClearOverlay(DI_DISPLAY_T *pstDiDisplay)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        goto EXIT;
    }
    
    if (pstDiDisplay->bDisplayNotAvailable == TRUE)
    {
        nRet = DI_OK;
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stDiDisplayMutex);
    
    memset(s_astDiDisplayOverlays, 0, sizeof(s_astDiDisplayOverlays));
    s_unOverlayCount = 0;
    
    pthread_mutex_unlock(&s_stDiDisplayMutex);
    
    PrintTrace("All overlays cleared");
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

void DI_DISPLAY_Status(DI_DISPLAY_T *pstDiDisplay)
{
    if (pstDiDisplay == NULL)
    {
        PrintError("pstDiDisplay == NULL!");
        return;
    }
    
    PrintDebug("========== Display Status Information =============");
    PrintDebug("Display Status: %d", pstDiDisplay->eDiDisplayStatus);
    PrintDebug("Display Available: %s", pstDiDisplay->bDisplayNotAvailable ? "NO" : "YES");
    PrintDebug("Device ID: %d", pstDiDisplay->stDiDisplayConfig.unDeviceId);
    PrintDebug("Resolution: %dx%d", pstDiDisplay->stDiDisplayConfig.unWidth, pstDiDisplay->stDiDisplayConfig.unHeight);
    PrintDebug("Full Screen: %s", pstDiDisplay->stDiDisplayConfig.bFullScreen ? "YES" : "NO");
    PrintDebug("Hardware Acceleration: %s", pstDiDisplay->stDiDisplayConfig.bHardwareAccel ? "YES" : "NO");
    PrintDebug("Overlay Enabled: %s", pstDiDisplay->stDiDisplayConfig.bOverlayEnabled ? "YES" : "NO");
    PrintDebug("Overlay Count: %d", s_unOverlayCount);
    PrintDebug("Log Level: %s", s_bDiDisplayLog == ON ? "ON" : "OFF");
    PrintDebug("===================================================");
}