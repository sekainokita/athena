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
* @file time_manager.c
*
* This file contains a data format design
*
* @note
*
* V2X Data Format Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  23.03.22 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include "framework.h"
#include "time_manager.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

/***************************** Definition ************************************/

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
FILE* sh_pTimeMgrMsg;
static int s_nTimeTaskMsgId;
static key_t s_timeTaskMsgKey = FRAMEWORK_TIME_TASK_MSG_KEY;

static pthread_t sh_TimeMgrTask;

/***************************** Function  *************************************/

static void *P_TIME_MANAGER_Task(void *arg)
{
    TIME_MANAGER_EVENT_MSG_T stEventMsg;
    int32_t nRet = FRAMEWORK_ERROR;
    memset(&stEventMsg, 0, sizeof(TIME_MANAGER_EVENT_MSG_T));

    (void)arg;

    while (1)
    {
        if(msgrcv(s_nTimeTaskMsgId, &stEventMsg, sizeof(TIME_MANAGER_EVENT_MSG_T), 0, MSG_NOERROR) == FRAMEWORK_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            PrintError("TODO");
            nRet = FRAMEWORK_OK;
        }

        usleep(1000);
    }

    return NULL;
}

static void P_TIME_MANAGER_PrintMsgInfo(int msqid)
{

    struct msqid_ds m_stat;

    PrintDebug("========== Messege Queue Infomation =============");

    if(msgctl(msqid, IPC_STAT, &m_stat) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgctl() is failed!!");
    }

    PrintDebug("msg_lspid : %d", m_stat.msg_lspid);
    PrintDebug("msg_qnum : %ld", m_stat.msg_qnum);
    PrintDebug("msg_stime : %ld", m_stat.msg_stime);

    PrintDebug("=================================================");
}

int32_t P_TIME_MANAGER_CreateTask(void)
{
	int32_t nRet = FRAMEWORK_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_TimeMgrTask, &attr, P_TIME_MANAGER_Task, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_TIME_MANAGER_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_TIME_MANAGER_Task() is successfully created.");
        nRet = FRAMEWORK_OK;
    }

#if defined(CONFIG_PTHREAD_JOINABLE)
    nRet = pthread_join(sh_TimeMgrTask, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_TIME_MANAGER_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintDebug("P_TIME_MANAGER_Task() is successfully joined.");
        nRet = FRAMEWORK_OK;
    }
#endif
	return nRet;
}

static int32_t P_TIME_MANAGER_Init(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    if((s_nTimeTaskMsgId = msgget(s_timeTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_TIME_MANAGER_PrintMsgInfo(s_nTimeTaskMsgId);
    }

    nRet = P_TIME_MANAGER_CreateTask();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_TIME_MANAGER_CreateTask() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}
static int32_t P_TIME_MANAGER_DeInit(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t TIME_MANAGER_Open(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t TIME_MANAGER_Close(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t TIME_MANAGER_Start(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t TIME_MANAGER_Stop(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t TIME_MANAGER_Status(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t TIME_MANAGER_Init(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    nRet = P_TIME_MANAGER_Init(pstTimeManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_TIME_MANAGER_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

int32_t TIME_MANAGER_DeInit(TIME_MANAGER_T *pstTimeManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager == NULL!!");
        return nRet;
    }

    nRet = P_TIME_MANAGER_DeInit(pstTimeManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_TIME_MANAGER_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

