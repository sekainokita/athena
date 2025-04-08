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
* @file di_can.c
*
* This file contains a data format design
*
* @note
*
* V2X DI CAN Source File
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

//#define CONFIG_DI_CAN_DEBUG     (1)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiCanLog = OFF;

static int s_nDiCanTaskMsgId;
static key_t s_DiCanTaskMsgKey = DI_CAN_TASK_MSG_KEY;
static pthread_t sh_DiCanTask;


#if defined(CONFIG_CAN_PEAK)
static DI_CAN_PEAK_T s_stDiCanDev;
#endif

/***************************** Function  *************************************/
static void *P_DI_CAN_Task(void *arg)
{
    DI_CAN_EVENT_MSG_T stEventMsg;
    int32_t nRet = APP_ERROR;
    memset(&stEventMsg, 0, sizeof(DI_CAN_EVENT_MSG_T));

    UNUSED(arg);
    UNUSED(nRet);

    while (1)
    {
        if(msgrcv(s_nDiCanTaskMsgId, &stEventMsg, sizeof(DI_CAN_EVENT_MSG_T), 0, MSG_NOERROR) == APP_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch(stEventMsg.eEventType)
            {
                case eDI_CAN_EVENT_START:
                {
                    PrintWarn("eDI_CAN_EVENT_START is received.");
                    break;
                }

                case eDI_CAN_EVENT_STOP:
                {
                    PrintWarn("eDI_CAN_EVENT_STOP is received.");
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

static void P_DI_CAN_PrintMsgInfo(int msqid)
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

int32_t P_DI_CAN_CreateTask(void)
{
	int32_t nRet = APP_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_DiCanTask, &attr, P_DI_CAN_Task, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_create() is failed!! (P_DI_CAN_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DI_CAN_Task() is successfully created.");
        nRet = APP_OK;
    }

	return nRet;
}

static int32_t P_DI_CAN_Init(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    if (pstDiCan->bCanNotAvailable == TRUE)
    {
        PrintWarn("bCanNotAvailable[%d]", pstDiCan->bCanNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

#if defined(CONFIG_CAN_PEAK)
    nRet = DI_CAN_PEAK_Init(&s_stDiCanDev);
    if(nRet != DI_OK)
    {
        PrintError("DI_CAN_PEAK_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
#else
    nRet = DI_OK;
    PrintWarn("None of CAN devices are supported.");
#endif

    if((s_nDiCanTaskMsgId = msgget(s_DiCanTaskMsgKey, IPC_CREAT|0666)) == DI_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DI_CAN_PrintMsgInfo(s_nDiCanTaskMsgId);
    }

    nRet = P_DI_CAN_CreateTask();
    if (nRet != APP_OK)
    {
        PrintError("P_DI_CAN_CreateTask() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}
static int32_t P_DI_CAN_DeInit(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    if (pstDiCan->bCanNotAvailable == TRUE)
    {
        PrintWarn("bCanNotAvailable[%d]", pstDiCan->bCanNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

#if defined(CONFIG_CAN_PEAK)
    nRet = DI_CAN_PEAK_DeInit(&s_stDiCanDev);
    if(nRet != DI_OK)
    {
        PrintError("DI_CAN_PEAK_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    (void*)memset(&s_stDiCanDev, 0x00, sizeof(DI_CAN_PEAK_T));
#else
    nRet = DI_OK;
    PrintWarn("None of CAN devices are supported.");
#endif

    return nRet;
}

int32_t DI_CAN_SetLog(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    s_bDiCanLog = pstDiCan->bLogLevel;
    PrintTrace("SET:s_bDiCanLog [%s]", s_bDiCanLog == ON ? "ON" : "OFF");

    nRet = DI_OK;

    return nRet;
}

int32_t DI_CAN_Get(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    if (pstDiCan->bCanNotAvailable == TRUE)
    {
        (void*)memset(&pstDiCan->stDiCanData, 0x00, sizeof(DI_CAN_DATA_T));

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiCan->eDiCanStatus >= DI_CAN_STATUS_OPENED)
    {
#if defined(CONFIG_CAN_PEAK)
        nRet = DI_CAN_PEAK_Get(&s_stDiCanDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_CAN_PEAK_Get() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#endif
    }
    else
    {
        PrintWarn("check the status of CAN [%d]", pstDiCan->eDiCanStatus);
    }

    return nRet;
}

int32_t DI_CAN_Open(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    if (pstDiCan->bCanNotAvailable == TRUE)
    {
        PrintWarn("bCanNotAvailable[%d]", pstDiCan->bCanNotAvailable);
        pstDiCan->eDiCanStatus = DI_CAN_STATUS_OPENED;

        nRet = DI_OK;
        return nRet;
    }

    if((pstDiCan->eDiCanStatus == DI_CAN_STATUS_INITIALIZED) || (pstDiCan->eDiCanStatus == DI_CAN_STATUS_CLOSED))
    {
#if defined(CONFIG_CAN_PEAK)
        nRet = DI_CAN_PEAK_Open(&s_stDiCanDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_CAN_PEAK_Open() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#else
        PrintError("CAN is not supported!");
#endif
        pstDiCan->eDiCanStatus = DI_CAN_STATUS_OPENED;
    }
    else
    {
        PrintWarn("check the status of CAN [%d]", pstDiCan->eDiCanStatus);

        if(pstDiCan->eDiCanStatus == DI_CAN_STATUS_OPENED)
        {
            PrintDebug("already DI_CAN_STATUS_OPENED");
        }
    }

    return nRet;
}

int32_t DI_CAN_Close(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    if (pstDiCan->bCanNotAvailable == TRUE)
    {
        PrintWarn("bCanNotAvailable[%d]", pstDiCan->bCanNotAvailable);
        pstDiCan->eDiCanStatus = DI_CAN_STATUS_CLOSED;

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiCan->eDiCanStatus == DI_CAN_STATUS_OPENED)
    {
#if defined(CONFIG_CAN_PEAK)
        nRet = DI_CAN_PEAK_Close(&s_stDiCanDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_CAN_PEAK_Close() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#else
        nRet = DI_OK;
        PrintWarn("None of CAN devices are supported.");
#endif
        pstDiCan->eDiCanStatus = DI_CAN_STATUS_CLOSED;

    }
    else
    {
        PrintWarn("check the status of CAN [%d]", pstDiCan->eDiCanStatus);

        if(pstDiCan->eDiCanStatus == DI_CAN_STATUS_CLOSED)
        {
            PrintDebug("already DI_CAN_STATUS_CLOSED");
        }
    }

    return nRet;
}

int32_t DI_CAN_Start(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DI_CAN_Stop(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    return nRet;
}

void DI_CAN_Status(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    UNUSED(nRet);

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
    }
}

int32_t DI_CAN_Init(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    nRet = P_DI_CAN_Init(pstDiCan);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_CAN_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiCan->eDiCanStatus = DI_CAN_STATUS_INITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    s_bDiCanLog = pstDiCan->bLogLevel;
    PrintDebug("s_bDiCanLog [%s]", s_bDiCanLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t DI_CAN_DeInit(DI_CAN_T *pstDiCan)
{
    int32_t nRet = DI_ERROR;

    if(pstDiCan == NULL)
    {
        PrintError("pstDiCan == NULL!!");
        return nRet;
    }

    nRet = P_DI_CAN_DeInit(pstDiCan);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_CAN_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiCan->eDiCanStatus = DI_CAN_STATUS_DEINITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}


