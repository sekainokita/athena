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
* @file db_manager.c
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
#include "db_manager.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

/***************************** Definition ************************************/
#define DB_MANAGER_DEFAULT_FILE_NAME "db_manager.log"
#define DB_MANAGER_THREAD_ID         (0x10)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
FILE* s_pDbManagerFd;
static key_t s_msgKey = FRAMEWORK_MSG_KEY;
static int s_nMsgId;

static DB_MANAGER_TASK_T *s_pThreadInfo = NULL;
static pthread_t *s_pThread = NULL;

/***************************** Function  *************************************/

static int32_t P_DB_MANAGER_Write(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if (s_pDbManagerFd != NULL)
    {
        char *pcPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
        memcpy(pcPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

        fprintf(s_pDbManagerFd, "eDeviceType[%d], ", pstEventMsg->pstDbV2x->eDeviceType);
        fprintf(s_pDbManagerFd, "eTeleCommType[%d], ", pstEventMsg->pstDbV2x->eTeleCommType);
        fprintf(s_pDbManagerFd, "unDeviceId[0x%x], ", pstEventMsg->pstDbV2x->unDeviceId);
        fprintf(s_pDbManagerFd, "ulTimeStamp[%ld], ", pstEventMsg->pstDbV2x->ulTimeStamp);
        fprintf(s_pDbManagerFd, "eServiceId[%d], ", pstEventMsg->pstDbV2x->eServiceId);
        fprintf(s_pDbManagerFd, "eActionType[%d], ", pstEventMsg->pstDbV2x->eActionType);
        fprintf(s_pDbManagerFd, "eRegionId[%d], ", pstEventMsg->pstDbV2x->eRegionId);
        fprintf(s_pDbManagerFd, "ePayloadType[%d], ", pstEventMsg->pstDbV2x->ePayloadType);
        fprintf(s_pDbManagerFd, "eCommId[%d], ", pstEventMsg->pstDbV2x->eCommId);
        fprintf(s_pDbManagerFd, "usDbVer[%d.%d], ", pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
        fprintf(s_pDbManagerFd, "usHwVer[0x%x], ", pstEventMsg->pstDbV2x->usHwVer);
        fprintf(s_pDbManagerFd, "usSwVer[0x%x], ", pstEventMsg->pstDbV2x->usSwVer);
        fprintf(s_pDbManagerFd, "ulPayloadLength[%d], ", pstEventMsg->pstDbV2x->ulPayloadLength);

        fprintf(s_pDbManagerFd, "cPayload[");
        for(int i = 0; i < (int)pstEventMsg->pstDbV2x->ulPayloadLength; i++)
        {
              fprintf(s_pDbManagerFd, "%d ", pcPayload[i]);
        }
        fprintf(s_pDbManagerFd, "], ");

        fprintf(s_pDbManagerFd, "ulPayloadCrc32[0x%x]", pstEventMsg->pstDbV2x->ulPacketCrc32);
        fprintf(s_pDbManagerFd, "\r\n");

        nRet = fflush(s_pDbManagerFd);
        if (nRet < 0)
        {
            PrintError("fflush() is failed! [unRet:%d]", nRet);
        }

        free(pcPayload);
    }
    else
    {
        PrintError("s_pDbManagerFd is NULL!!, check whethter s_pDbManagerFd is opened before.");
    }

    return nRet;
}

static void *P_DB_MANAGER_Task(void *arg)
{
    DB_MANAGER_TASK_T const *const stThreadInfo = (DB_MANAGER_TASK_T *)arg;
    int nMsgId;
    DB_MANAGER_EVENT_MSG_T stEventMsg;
    int32_t nRet = FRAMEWORK_ERROR;

    PrintDebug("stThreadInfo[0x%x]", stThreadInfo->nThreadId);

    if((nMsgId = msgget(s_msgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return NULL;
    }

    while (1)
    {
        if(msgrcv(nMsgId, &stEventMsg, sizeof(DB_MANAGER_EVENT_MSG_T), 0, MSG_NOERROR) == FRAMEWORK_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            if(stEventMsg.pstDbManagerWrite->eProc == DB_MANAGER_PROC_WRITE)
            {
                PrintDebug("DB_MANAGER_PROC_WRITE [%d]", stEventMsg.pstDbManagerWrite->eProc);

                switch(stEventMsg.pstDbManagerWrite->eFileType)
                {
                    case DB_MANAGER_FILE_TYPE_TXT:
                    {
                        PrintDebug("DB_MANAGER_FILE_TYPE_TXT [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        nRet = P_DB_MANAGER_Write(&stEventMsg);
                        if(nRet != FRAMEWORK_OK)
                        {
                            PrintError("P_DB_MANAGER_Write() is failed! [unRet:%d]", nRet);
                        }
                        break;
                    }
                    case DB_MANAGER_FILE_TYPE_CSV:
                    {
                        PrintDebug("DB_MANAGER_FILE_TYPE_CSV [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        PrintWarn("TODO");
                        break;
                    }
                    case DB_MANAGER_FILE_TYPE_SQLITE:
                    {
                        PrintDebug("DB_MANAGER_FILE_TYPE_SQLITE [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        PrintWarn("TODO");
                        break;
                    }
                    default:
                        PrintWarn("unknown file type [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        break;
                }
            }
            else if(stEventMsg.pstDbManagerWrite->eProc == DB_MANAGER_PROC_READ)
            {
                PrintDebug("DB_MANAGER_PROC_READ [%d]", stEventMsg.pstDbManagerWrite->eProc);
                PrintWarn("TODO");

            }
            else if(stEventMsg.pstDbManagerWrite->eProc == DB_MANAGER_PROC_CONVERT)
            {
                PrintDebug("DB_MANAGER_PROC_CONVERT [%d]", stEventMsg.pstDbManagerWrite->eProc);
                PrintWarn("TODO");
            }
            else
            {
                PrintWarn("unknown processing type [%d]", stEventMsg.pstDbManagerWrite->eProc);
            }
        }
    }

    return NULL;
}

static void P_DB_NABAGER_PrintMsgInfo(int msqid)
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

static int32_t P_DB_MANAGER_Init(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    int nThreads = sysconf(_SC_NPROCESSORS_ONLN);

    PrintDebug("nThreads[%d]", nThreads);

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    if((s_nMsgId = msgget(s_msgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DB_NABAGER_PrintMsgInfo(s_nMsgId);
        nRet = FRAMEWORK_OK;
    }

    pthread_t *s_pThread = (pthread_t *)malloc(nThreads * sizeof(pthread_t));
    DB_MANAGER_TASK_T *s_pThreadInfo = (DB_MANAGER_TASK_T *)malloc(nThreads * sizeof(DB_MANAGER_TASK_T));

    for (int i = 0; i < nThreads; ++i)
    {
        s_pThreadInfo[i].nThreads = nThreads;
        s_pThreadInfo[i].nThreadId = DB_MANAGER_THREAD_ID + i;
        s_pThreadInfo[i].nMsgId = s_nMsgId;
        if (pthread_create(&s_pThread[i], NULL, &P_DB_MANAGER_Task, &s_pThreadInfo[i]) != FRAMEWORK_OK)
        {
            PrintError("pthread_create() is failed!!");
        }
    }

#if defined(CONFIG_MULTI_THREAD)
    // Join all the threads
    for (int i = 0; i < nThreads; ++i)
    {
        pthread_join(s_pThreadInfo[i], NULL);
    }
#endif

    return FRAMEWORK_OK;
}
static int32_t P_DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    free(s_pThread);
    free(s_pThreadInfo);

    return nRet;
}

int32_t DB_MANAGER_Write(DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void *pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;
    DB_MANAGER_EVENT_MSG_T stEventMsg;

    if(pstDbManagerWrite == NULL)
    {
        PrintError("pstDbManagerWrite == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

    if(s_pDbManagerFd == NULL)
    {
        PrintError("s_pDbManagerFd == NULL!!, check DB_MANAGER_Open() is called.");
        return nRet;
    }

    stEventMsg.pstDbManagerWrite = pstDbManagerWrite;
    stEventMsg.pstDbV2x = pstDbV2x;
    stEventMsg.pPayload = pPayload;

    if(msgsnd(s_nMsgId, &stEventMsg, sizeof(DB_MANAGER_EVENT_MSG_T), IPC_NOWAIT) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgsnd() is failed!!");
        return nRet;
    }
    else
    {
        nRet = FRAMEWORK_OK;
    }

    return nRet;
}

int32_t DB_MANAGER_Read(DB_MANAGER_READ_T *pstDbManagerRead, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManagerRead == NULL)
    {
        PrintError("pstDbManagerRead == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

    if(s_pDbManagerFd == NULL)
    {
        PrintError("s_pDbManagerFd == NULL!!, check DB_MANAGER_Open() is called.");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Converter(DB_MANAGER_READ_T *pstDbManagerRead, DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManagerRead == NULL)
    {
        PrintError("pstDbManagerRead == NULL!!");
        return nRet;
    }

    if(pstDbManagerWrite == NULL)
    {
        PrintError("pstDbManagerWrite == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Open(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    switch(pstDbManager->eFileType)
    {
        case DB_MANAGER_FILE_TYPE_TXT:
            PrintDebug("DB_MANAGER_FILE_TYPE_TXT [%d]", pstDbManager->eFileType);

            if(s_pDbManagerFd == NULL)
            {
                s_pDbManagerFd = fopen(DB_MANAGER_DEFAULT_FILE_NAME, "a+");
                if(s_pDbManagerFd == NULL)
                {
                    PrintError("fopen() is failed!!");
                }
                else
                {
                    PrintTrace("DB_MANAGER_DEFAULT_FILE_NAME[%s] is opened.", DB_MANAGER_DEFAULT_FILE_NAME);
                    nRet = FRAMEWORK_OK;
                }
            }

            break;

        case DB_MANAGER_FILE_TYPE_CSV:
            PrintDebug("DB_MANAGER_FILE_TYPE_CSV [%d]", pstDbManager->eFileType);
            PrintWarn("TODO");
            break;

        case DB_MANAGER_FILE_TYPE_SQLITE:
            PrintDebug("DB_MANAGER_FILE_TYPE_SQLITE [%d]", pstDbManager->eFileType);
            PrintWarn("TODO");
            break;

        default:
            PrintWarn("unknown file type [%d]", pstDbManager->eFileType);
            break;

    }

    return nRet;
}

int32_t DB_MANAGER_Close(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    switch(pstDbManager->eFileType)
    {
        case DB_MANAGER_FILE_TYPE_TXT:
            PrintDebug("DB_MANAGER_FILE_TYPE_TXT [%d]", pstDbManager->eFileType);

            if(s_pDbManagerFd != NULL)
            {
                nRet = fflush(s_pDbManagerFd);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(s_pDbManagerFd);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("DB_MANAGER_DEFAULT_FILE_NAME[%s] is closed.", DB_MANAGER_DEFAULT_FILE_NAME);
                    s_pDbManagerFd = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }
            break;

        case DB_MANAGER_FILE_TYPE_CSV:
            PrintDebug("DB_MANAGER_FILE_TYPE_CSV [%d]", pstDbManager->eFileType);
            PrintWarn("TODO");
            break;

        case DB_MANAGER_FILE_TYPE_SQLITE:
            PrintDebug("DB_MANAGER_FILE_TYPE_SQLITE [%d]", pstDbManager->eFileType);
            PrintWarn("TODO");
            break;

        default:
            PrintWarn("unknown file type [%d]", pstDbManager->eFileType);
            break;

    }



    return nRet;
}

int32_t DB_MANAGER_Start(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Stop(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Status(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Init(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    nRet = P_DB_MANAGER_Init(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_DB_MANAGER_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

int32_t DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    nRet = P_DB_MANAGER_DeInit(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_DB_MANAGER_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

