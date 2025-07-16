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
* @file di_video.c
*
* This file contains a data format design
*
* @note
*
* V2X DI VIDEO Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  23.06.29 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include <math.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "di.h"
#include "di_video.h"
#include "di_video_nvidia.h"

/***************************** Definition ************************************/

//#define CONFIG_DI_VIDEO_DEBUG     (1)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiVideoLog = OFF;

static int s_nDiVideoTaskMsgId;
static key_t s_DiVideoTaskMsgKey = DI_VIDEO_TASK_MSG_KEY;
static pthread_t sh_DiVideoTask;


#if defined(CONFIG_VIDEO_NVIDIA)
static DI_VIDEO_NVIDIA_T s_stDiVideoDev;
#endif

/***************************** Function  *************************************/
static void *P_DI_VIDEO_Task(void *arg)
{
    DI_VIDEO_EVENT_MSG_T stEventMsg;
    int32_t nRet = APP_ERROR;
    memset(&stEventMsg, 0, sizeof(DI_VIDEO_EVENT_MSG_T));

    UNUSED(arg);
    UNUSED(nRet);

    while (1)
    {
        if(msgrcv(s_nDiVideoTaskMsgId, &stEventMsg, sizeof(DI_VIDEO_EVENT_MSG_T), 0, MSG_NOERROR) == APP_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch(stEventMsg.eEventType)
            {
                case eDI_VIDEO_EVENT_START:
                {
                    PrintWarn("eDI_VIDEO_EVENT_START is received.");
                    break;
                }

                case eDI_VIDEO_EVENT_STOP:
                {
                    PrintWarn("eDI_VIDEO_EVENT_STOP is received.");
                    break;
                }

                default:
                    PrintWarn("unknown event type [%d]", stEventMsg.eEventType);
                    break;
            }
        }

        usleep(1000);
    }

    return NULL;
}

static void P_DI_VIDEO_PrintMsgInfo(int msqid)
{
    struct msqid_ds m_stat;

    PrintDebug("========== Messege Queue Infomation =============");

    if(msgctl(msqid, IPC_STAT, &m_stat) == DI_MSG_ERR)
    {
        PrintError("msgctl() is failed!!");
    }

    PrintDebug("msg_lspid : %d", m_stat.msg_lspid);
    PrintDebug("msg_qnum : %ld", m_stat.msg_qnum);
    PrintDebug("msg_stime : %ld", m_stat.msg_stime);

    PrintDebug("=================================================");
}

int32_t P_DI_VIDEO_CreateTask(void)
{
	int32_t nRet = APP_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_DiVideoTask, &attr, P_DI_VIDEO_Task, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_create() is failed!! (P_DI_VIDEO_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DI_VIDEO_Task() is successfully created.");
        nRet = APP_OK;
    }

	return nRet;
}

static int32_t P_DI_VIDEO_Init(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    if (pstDiVideo->bVideoNotAvailable == TRUE)
    {
        PrintWarn("bVideoNotAvailable[%d]", pstDiVideo->bVideoNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

#if defined(CONFIG_VIDEO_NVIDIA)
    nRet = DI_VIDEO_NVIDIA_Init(&s_stDiVideoDev);
    if(nRet != DI_OK)
    {
        PrintError("DI_VIDEO_NVIDIA_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
#else
    nRet = DI_OK;
    PrintWarn("None of VIDEO devices are supported.");
#endif

    if((s_nDiVideoTaskMsgId = msgget(s_DiVideoTaskMsgKey, IPC_CREAT|0666)) == DI_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DI_VIDEO_PrintMsgInfo(s_nDiVideoTaskMsgId);
    }

    nRet = P_DI_VIDEO_CreateTask();
    if (nRet != APP_OK)
    {
        PrintError("P_DI_VIDEO_CreateTask() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}
static int32_t P_DI_VIDEO_DeInit(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    if (pstDiVideo->bVideoNotAvailable == TRUE)
    {
        PrintWarn("bVideoNotAvailable[%d]", pstDiVideo->bVideoNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

#if defined(CONFIG_VIDEO_NVIDIA)
    nRet = DI_VIDEO_NVIDIA_DeInit(&s_stDiVideoDev);
    if(nRet != DI_OK)
    {
        PrintError("DI_VIDEO_NVIDIA_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    (void*)memset(&s_stDiVideoDev, 0x00, sizeof(DI_VIDEO_NVIDIA_T));
#else
    nRet = DI_OK;
    PrintWarn("None of VIDEO devices are supported.");
#endif

    return nRet;
}

int32_t DI_VIDEO_SetLog(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    s_bDiVideoLog = pstDiVideo->bLogLevel;
    PrintTrace("SET:s_bDiVideoLog [%s]", s_bDiVideoLog == ON ? "ON" : "OFF");

    nRet = DI_OK;

    return nRet;
}

int32_t DI_VIDEO_Get(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    if (pstDiVideo->bVideoNotAvailable == TRUE)
    {
        (void*)memset(&pstDiVideo->stDiVideoData, 0x00, sizeof(DI_VIDEO_DATA_T));

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiVideo->eDiVideoStatus >= DI_VIDEO_STATUS_OPENED)
    {
#if defined(CONFIG_VIDEO_NVIDIA)
        nRet = DI_VIDEO_NVIDIA_Get(&s_stDiVideoDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_VIDEO_NVIDIA_Get() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#endif
    }
    else
    {
        PrintWarn("check the status of VIDEO [%d]", pstDiVideo->eDiVideoStatus);
    }

    return nRet;
}

int32_t DI_VIDEO_Open(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    if (pstDiVideo->bVideoNotAvailable == TRUE)
    {
        PrintWarn("bVideoNotAvailable[%d]", pstDiVideo->bVideoNotAvailable);
        pstDiVideo->eDiVideoStatus = DI_VIDEO_STATUS_OPENED;

        nRet = DI_OK;
        return nRet;
    }

    if((pstDiVideo->eDiVideoStatus == DI_VIDEO_STATUS_INITIALIZED) || (pstDiVideo->eDiVideoStatus == DI_VIDEO_STATUS_CLOSED))
    {
#if defined(CONFIG_VIDEO_NVIDIA)
        nRet = DI_VIDEO_NVIDIA_Open(&s_stDiVideoDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_VIDEO_NVIDIA_Open() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#endif
        pstDiVideo->eDiVideoStatus = DI_VIDEO_STATUS_OPENED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("check the status of VIDEO [%d]", pstDiVideo->eDiVideoStatus);

        if(pstDiVideo->eDiVideoStatus == DI_VIDEO_STATUS_OPENED)
        {
            PrintDebug("already DI_VIDEO_STATUS_OPENED");
            nRet = DI_OK;
        }
    }

    return nRet;
}

int32_t DI_VIDEO_Close(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    if (pstDiVideo->bVideoNotAvailable == TRUE)
    {
        PrintWarn("bVideoNotAvailable[%d]", pstDiVideo->bVideoNotAvailable);
        pstDiVideo->eDiVideoStatus = DI_VIDEO_STATUS_CLOSED;

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiVideo->eDiVideoStatus == DI_VIDEO_STATUS_OPENED)
    {
#if defined(CONFIG_VIDEO_NVIDIA)
        nRet = DI_VIDEO_NVIDIA_Close(&s_stDiVideoDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_VIDEO_NVIDIA_Close() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#else
        nRet = DI_OK;
        PrintWarn("None of VIDEO devices are supported.");
#endif
        pstDiVideo->eDiVideoStatus = DI_VIDEO_STATUS_CLOSED;

    }
    else
    {
        PrintWarn("check the status of VIDEO [%d]", pstDiVideo->eDiVideoStatus);

        if(pstDiVideo->eDiVideoStatus == DI_VIDEO_STATUS_CLOSED)
        {
            PrintDebug("already DI_VIDEO_STATUS_CLOSED");
        }
    }

    return nRet;
}

int32_t DI_VIDEO_Start(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    if (pstDiVideo->bVideoNotAvailable == TRUE)
    {
        PrintWarn("bVideoNotAvailable[%d]", pstDiVideo->bVideoNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

    if(pstDiVideo->eDiVideoStatus == DI_VIDEO_STATUS_OPENED)
    {
#if defined(CONFIG_VIDEO_NVIDIA)
        nRet = DI_VIDEO_NVIDIA_Start(&s_stDiVideoDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_VIDEO_NVIDIA_Start() is failed! [nRet:%d]", nRet);
            return nRet;
        }
#endif
        pstDiVideo->eDiVideoStatus = DI_VIDEO_STATUS_STARTED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("check the status of VIDEO [%d]", pstDiVideo->eDiVideoStatus);
        
        if(pstDiVideo->eDiVideoStatus == DI_VIDEO_STATUS_STARTED)
        {
            PrintDebug("already DI_VIDEO_STATUS_STARTED");
            nRet = DI_OK;
        }
    }

    return nRet;
}

int32_t DI_VIDEO_Stop(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    return nRet;
}

void DI_VIDEO_Status(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    UNUSED(nRet);

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
    }
}

int32_t DI_VIDEO_Init(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    nRet = P_DI_VIDEO_Init(pstDiVideo);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiVideo->eDiVideoStatus = DI_VIDEO_STATUS_INITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    s_bDiVideoLog = pstDiVideo->bLogLevel;
    PrintDebug("s_bDiVideoLog [%s]", s_bDiVideoLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t DI_VIDEO_DeInit(DI_VIDEO_T *pstDiVideo)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideo == NULL)
    {
        PrintError("pstDiVideo == NULL!!");
        return nRet;
    }

    nRet = P_DI_VIDEO_DeInit(pstDiVideo);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiVideo->eDiVideoStatus = DI_VIDEO_STATUS_DEINITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}



