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
* @file di_can_peak.c
*
* This file contains a data format design
*
* @note
*
* V2X DI CAN_PEAK Source File
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
#include "di_can.h"
#include "di_can_peak.h"

/***************************** Definition ************************************/

//#define CONFIG_DI_CAN_PEAK_DEBUG     (1)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiCanPeakLog = OFF;

static int s_nDiCanPeakTaskMsgId;
static key_t s_DiCanPeakTaskMsgKey = DI_CAN_PEAK_TASK_MSG_KEY;
static pthread_t sh_DiCanPeakTask;

/***************************** Function  *************************************/
static void *P_DI_CAN_PEAK_Task(void *arg)
{
    DI_CAN_PEAK_EVENT_MSG_T stEventMsg;
    int32_t nRet = APP_ERROR;
    memset(&stEventMsg, 0, sizeof(DI_CAN_PEAK_EVENT_MSG_T));

    UNUSED(arg);
    UNUSED(nRet);

    while (1)
    {
        if(msgrcv(s_nDiCanPeakTaskMsgId, &stEventMsg, sizeof(DI_CAN_PEAK_EVENT_MSG_T), 0, MSG_NOERROR) == APP_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch(stEventMsg.eEventType)
            {
                case eDI_CAN_PEAK_EVENT_START:
                {
                    PrintWarn("eDI_CAN_PEAK_EVENT_START is received.");
                    break;
                }

                case eDI_CAN_PEAK_EVENT_STOP:
                {
                    PrintWarn("eDI_CAN_PEAK_EVENT_STOP is received.");
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

static void P_DI_CAN_PEAK_PrintMsgInfo(int msqid)
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

int32_t P_DI_CAN_PEAK_CreateTask(void)
{
	int32_t nRet = APP_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_DiCanPeakTask, &attr, P_DI_CAN_PEAK_Task, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_create() is failed!! (P_DI_CAN_PEAK_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DI_CAN_PEAK_Task() is successfully created.");
        nRet = APP_OK;
    }

	return nRet;
}

static int32_t P_DI_CAN_PEAK_Init(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    if (pstDiCanPeak->bCanPeakNotAvailable == TRUE)
    {
        PrintWarn("bCanPeakNotAvailable[%d]", pstDiCanPeak->bCanPeakNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

    if((s_nDiCanPeakTaskMsgId = msgget(s_DiCanPeakTaskMsgKey, IPC_CREAT|0666)) == DI_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DI_CAN_PEAK_PrintMsgInfo(s_nDiCanPeakTaskMsgId);
    }

    nRet = P_DI_CAN_PEAK_CreateTask();
    if (nRet != APP_OK)
    {
        PrintError("P_DI_CAN_PEAK_CreateTask() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}
static int32_t P_DI_CAN_PEAK_DeInit(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    if (pstDiCanPeak->bCanPeakNotAvailable == TRUE)
    {
        PrintWarn("bCanPeakNotAvailable[%d]", pstDiCanPeak->bCanPeakNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

    return nRet;
}

int32_t DI_CAN_PEAK_SetLog(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    s_bDiCanPeakLog = pstDiCanPeak->bLogLevel;
    PrintTrace("SET:s_bDiCanPeakLog [%s]", s_bDiCanPeakLog == ON ? "ON" : "OFF");

    nRet = DI_OK;

    return nRet;
}

int32_t DI_CAN_PEAK_Get(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    if (pstDiCanPeak->bCanPeakNotAvailable == TRUE)
    {
        (void*)memset(&pstDiCanPeak->stDiCanPeakData, 0x00, sizeof(DI_CAN_PEAK_DATA_T));

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiCanPeak->eDiCanPeakStatus >= DI_CAN_PEAK_STATUS_OPENED)
    {
        /* TODO */
    }

    return nRet;
}

int32_t DI_CAN_PEAK_Open(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    if (pstDiCanPeak->bCanPeakNotAvailable == TRUE)
    {
        PrintWarn("bCanPeakNotAvailable[%d]", pstDiCanPeak->bCanPeakNotAvailable);
        pstDiCanPeak->eDiCanPeakStatus = DI_CAN_PEAK_STATUS_OPENED;

        nRet = DI_OK;
        return nRet;
    }

    if((pstDiCanPeak->eDiCanPeakStatus == DI_CAN_PEAK_STATUS_INITIALIZED) || (pstDiCanPeak->eDiCanPeakStatus == DI_CAN_PEAK_STATUS_CLOSED))
    {
        pstDiCanPeak->eDiCanPeakStatus = DI_CAN_PEAK_STATUS_OPENED;
    }
    else
    {
        PrintWarn("check the status of CAN_PEAK [%d]", pstDiCanPeak->eDiCanPeakStatus);

        if(pstDiCanPeak->eDiCanPeakStatus == DI_CAN_PEAK_STATUS_OPENED)
        {
            PrintDebug("already DI_CAN_PEAK_STATUS_OPENED");
        }
    }

    return nRet;
}

int32_t DI_CAN_PEAK_Close(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    if (pstDiCanPeak->bCanPeakNotAvailable == TRUE)
    {
        PrintWarn("bCanPeakNotAvailable[%d]", pstDiCanPeak->bCanPeakNotAvailable);
        pstDiCanPeak->eDiCanPeakStatus = DI_CAN_PEAK_STATUS_CLOSED;

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiCanPeak->eDiCanPeakStatus == DI_CAN_PEAK_STATUS_OPENED)
    {
        pstDiCanPeak->eDiCanPeakStatus = DI_CAN_PEAK_STATUS_CLOSED;

    }
    else
    {
        PrintWarn("check the status of CAN_PEAK [%d]", pstDiCanPeak->eDiCanPeakStatus);

        if(pstDiCanPeak->eDiCanPeakStatus == DI_CAN_PEAK_STATUS_CLOSED)
        {
            PrintDebug("already DI_CAN_PEAK_STATUS_CLOSED");
        }
    }

    return nRet;
}

int32_t DI_CAN_PEAK_Start(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DI_CAN_PEAK_Stop(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    return nRet;
}

void DI_CAN_PEAK_Status(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    UNUSED(nRet);

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
    }
}

int32_t DI_CAN_PEAK_Init(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    nRet = P_DI_CAN_PEAK_Init(pstDiCanPeak);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_CAN_PEAK_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiCanPeak->eDiCanPeakStatus = DI_CAN_PEAK_STATUS_INITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    s_bDiCanPeakLog = pstDiCanPeak->bLogLevel;
    PrintDebug("s_bDiCanPeakLog [%s]", s_bDiCanPeakLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t DI_CAN_PEAK_DeInit(DI_CAN_PEAK_T *pstDiCanPeak)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCanPeak == NULL)
    {
        PrintError("pstDiCanPeak == NULL!!");
        return nRet;
    }

    nRet = P_DI_CAN_PEAK_DeInit(pstDiCanPeak);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_CAN_PEAK_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiCanPeak->eDiCanPeakStatus = DI_CAN_PEAK_STATUS_DEINITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}


