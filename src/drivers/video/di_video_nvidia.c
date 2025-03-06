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
* @file di_video_nvidia.c
*
* This file contains a data format design
*
* @note
*
* V2X DI VIDEO_NVIDIA Source File
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

//#define CONFIG_DI_VIDEO_NVIDIA_DEBUG     (1)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiVideoNvidiaLog = OFF;

static int s_nDiVideoNvidiaTaskMsgId;
static key_t s_DiVideoNvidiaTaskMsgKey = DI_VIDEO_NVIDIA_TASK_MSG_KEY;
static pthread_t sh_DiVideoNvidiaTask;

/***************************** Function  *************************************/
static void *P_DI_VIDEO_NVIDIA_Task(void *arg)
{
    DI_VIDEO_NVIDIA_EVENT_MSG_T stEventMsg;
    int32_t nRet = APP_ERROR;
    memset(&stEventMsg, 0, sizeof(DI_VIDEO_NVIDIA_EVENT_MSG_T));

    UNUSED(arg);
    UNUSED(nRet);

    while (1)
    {
        if(msgrcv(s_nDiVideoNvidiaTaskMsgId, &stEventMsg, sizeof(DI_VIDEO_NVIDIA_EVENT_MSG_T), 0, MSG_NOERROR) == APP_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch(stEventMsg.eEventType)
            {
                case eDI_VIDEO_NVIDIA_EVENT_START:
                {
                    PrintWarn("eDI_VIDEO_NVIDIA_EVENT_START is received.");
                    break;
                }

                case eDI_VIDEO_NVIDIA_EVENT_STOP:
                {
                    PrintWarn("eDI_VIDEO_NVIDIA_EVENT_STOP is received.");
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

static void P_DI_VIDEO_NVIDIA_PrintMsgInfo(int msqid)
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

int32_t P_DI_VIDEO_NVIDIA_CreateTask(void)
{
	int32_t nRet = APP_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_DiVideoNvidiaTask, &attr, P_DI_VIDEO_NVIDIA_Task, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_create() is failed!! (P_DI_VIDEO_NVIDIA_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DI_VIDEO_NVIDIA_Task() is successfully created.");
        nRet = APP_OK;
    }

	return nRet;
}

static int32_t P_DI_VIDEO_NVIDIA_Init(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

    if((s_nDiVideoNvidiaTaskMsgId = msgget(s_DiVideoNvidiaTaskMsgKey, IPC_CREAT|0666)) == DI_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DI_VIDEO_NVIDIA_PrintMsgInfo(s_nDiVideoNvidiaTaskMsgId);
    }

    nRet = P_DI_VIDEO_NVIDIA_CreateTask();
    if (nRet != APP_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_CreateTask() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}
static int32_t P_DI_VIDEO_NVIDIA_DeInit(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_SetLog(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    s_bDiVideoNvidiaLog = pstDiVideoNvidia->bLogLevel;
    PrintTrace("SET:s_bDiVideoNvidiaLog [%s]", s_bDiVideoNvidiaLog == ON ? "ON" : "OFF");

    nRet = DI_OK;

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Get(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        (void*)memset(&pstDiVideoNvidia->stDiVideoNvidiaData, 0x00, sizeof(DI_VIDEO_NVIDIA_DATA_T));

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiVideoNvidia->eDiVideoNvidiaStatus >= DI_VIDEO_NVIDIA_STATUS_OPENED)
    {
        /* TODO */
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Open(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_OPENED;

        nRet = DI_OK;
        return nRet;
    }

    if((pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_INITIALIZED) || (pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_CLOSED))
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_OPENED;
    }
    else
    {
        PrintWarn("check the status of VIDEO_NVIDIA [%d]", pstDiVideoNvidia->eDiVideoNvidiaStatus);

        if(pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_OPENED)
        {
            PrintDebug("already DI_VIDEO_NVIDIA_STATUS_OPENED");
        }
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Close(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_CLOSED;

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_OPENED)
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_CLOSED;

    }
    else
    {
        PrintWarn("check the status of VIDEO_NVIDIA [%d]", pstDiVideoNvidia->eDiVideoNvidiaStatus);

        if(pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_CLOSED)
        {
            PrintDebug("already DI_VIDEO_NVIDIA_STATUS_CLOSED");
        }
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Start(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Stop(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    return nRet;
}

void DI_VIDEO_NVIDIA_Status(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    UNUSED(nRet);

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
    }
}

int32_t DI_VIDEO_NVIDIA_Init(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    nRet = P_DI_VIDEO_NVIDIA_Init(pstDiVideoNvidia);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_INITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    s_bDiVideoNvidiaLog = pstDiVideoNvidia->bLogLevel;
    PrintDebug("s_bDiVideoNvidiaLog [%s]", s_bDiVideoNvidiaLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_DeInit(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    nRet = P_DI_VIDEO_NVIDIA_DeInit(pstDiVideoNvidia);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_DEINITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}



