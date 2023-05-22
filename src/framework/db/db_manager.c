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
#include "db_manager.h"

/***************************** Definition ************************************/
#define DB_MANAGER_DEFAULT_FILE_NAME "db_manager.log"

/***************************** Static Variable *******************************/
static mqd_t s_stMqdes;
FILE* s_pDbManagerFd;

/***************************** Function  *************************************/

DB_MANAGER_MSG_T *P_DB_MANAGER_MsgNew(int id, const char *text)
{
    DB_MANAGER_MSG_T *pstMsg = (DB_MANAGER_MSG_T *)malloc(sizeof(DB_MANAGER_MSG_T));
    pstMsg->text = text ? strdup(text) : NULL;
    pstMsg->id = id;

    return pstMsg;
}

int P_DB_MANAGER_MsgId(DB_MANAGER_MSG_T *pstMsg)
{
    return pstMsg->id;
}

char *P_DB_MANAGER_MsgText(DB_MANAGER_MSG_T *pstMsg)
{
    return pstMsg->text;
}

void P_DB_MANAGER_MsgFree(DB_MANAGER_MSG_T *pstMsg)
{
    free(pstMsg->text);
    free(pstMsg);
    pstMsg = NULL;
}

DB_MANAGER_ARCHIVE_T *P_DB_MANAGER_ArchiveNew(size_t size, const char *data)
{
    DB_MANAGER_ARCHIVE_T *pstAr = (DB_MANAGER_ARCHIVE_T *)malloc(sizeof(DB_MANAGER_ARCHIVE_T));
    pstAr->data = NULL;
    pstAr->size = size;

    if (size > 0)
    {
        memcpy(pstAr->data, data, size);
    }
    return pstAr;
}

size_t P_DB_MANAGER_ArchiveSize(DB_MANAGER_ARCHIVE_T *pstAr)
{
    return pstAr->size;
}

char *P_DB_MANAGER_ArchiveData(DB_MANAGER_ARCHIVE_T *pstAr)
{
    return pstAr->data;
}

void P_DB_MANAGER_ArchiveFree(DB_MANAGER_ARCHIVE_T *pstAr)
{
    free(pstAr->data);
    free(pstAr);
    pstAr = NULL;
}

DB_MANAGER_ARCHIVE_T *P_DB_MANAGER_MsgSerialize(const DB_MANAGER_MSG_T *pstMsg)
{
    DB_MANAGER_ARCHIVE_T *pstAr = P_DB_MANAGER_ArchiveNew(0, NULL);

    size_t len = strlen(pstMsg->text) + 1;
    pstAr->size = sizeof(int) + len * sizeof(char);

    pstAr->data = (char *)malloc(pstAr->size);
    size_t offset = 0;

    memcpy(pstAr->data + offset, &pstMsg->id, sizeof(int));
    offset += sizeof(int);

    memcpy(pstAr->data + offset, pstMsg->text, len * sizeof(char));

    return pstAr;
}

DB_MANAGER_MSG_T *P_DB_MANAGER_MsgUnSerialize(const char *buffer)
{
    DB_MANAGER_MSG_T *pstMsg = P_DB_MANAGER_MsgNew(0, NULL);

    size_t offset = 0;

    memcpy(&pstMsg->id, buffer + offset, sizeof(int));
    offset += sizeof(int);

    // I hope you serialized that null terminator
    pstMsg->text = strdup(buffer + offset);

    return pstMsg;
}

int P_DB_MANAGER_MsgSend(mqd_t mqdes, const DB_MANAGER_MSG_T *pstMsg, unsigned int msg_prio)
{
    DB_MANAGER_ARCHIVE_T *pstAr = P_DB_MANAGER_MsgSerialize(pstMsg);
    int ret = mq_send(mqdes, P_DB_MANAGER_ArchiveData(pstAr), P_DB_MANAGER_ArchiveSize(pstAr), msg_prio);
    P_DB_MANAGER_ArchiveFree(pstAr);

    return ret;
}

DB_MANAGER_MSG_T *P_DB_MANAGER_MsgReceive(mqd_t mqdes, char *buffer, size_t size)
{
    struct mq_attr mqstat;
    if (size == 0)
    {
        mq_getattr(mqdes, &mqstat);
        size = mqstat.mq_msgsize;
        buffer = (char *)malloc(size * sizeof(char));
    }

    DB_MANAGER_MSG_T *pstMsg = NULL;

    switch (mq_receive(mqdes, buffer, size, NULL))
    {
        case 0:
            // zero-length message means termination
            break;

        case -1:
            PrintError("mq_receive");
            break;

        default:
            pstMsg = P_DB_MANAGER_MsgUnSerialize(buffer);
            break;
    }

    if (size == 0)
    {
        free(buffer);
    }

    return pstMsg;
}

static void *P_DB_MANAGER_Task(void *arg)
{
    DB_MANAGER_TASK_T const *const tinfo = (DB_MANAGER_TASK_T *)arg;
    mqd_t mqdes = *(tinfo->pmqdes);

    struct mq_attr mqstat;
    mq_getattr(mqdes, &mqstat);
    char *buffer = (char *)malloc(mqstat.mq_msgsize * sizeof(char));

    DB_MANAGER_MSG_T *pstMsg;
    while (1)
    {
        pstMsg = P_DB_MANAGER_MsgReceive(mqdes, buffer, mqstat.mq_msgsize);
        if(pstMsg != NULL)
        {
            PrintDebug("tid[%d] len[%ld] id[%d], str[%s]", tinfo->tid, strlen(P_DB_MANAGER_MsgText(pstMsg)), P_DB_MANAGER_MsgId(pstMsg), P_DB_MANAGER_MsgText(pstMsg));
            usleep(10000); // Placeholder for long work
            P_DB_MANAGER_MsgFree(pstMsg);
        }
#if 0
        else
        {
            PrintWarn("Finish!");
            break;
        }
#endif
        continue;
    }

    free(buffer);

    return NULL;
}

static DB_MANAGER_TASK_T *s_pThreadInfo = NULL;
static pthread_t *s_pThread = NULL;
static const char s_cQueueName[] = "/db_manager_queue";

// Main loop
static int32_t P_DB_MANAGER_Init(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    int nthreads = sysconf(_SC_NPROCESSORS_ONLN);

    PrintDebug("nthreads[%d]", nthreads);

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    s_stMqdes = mq_open(s_cQueueName, O_CREAT | O_RDWR, 0644, NULL);
    if (s_stMqdes == (mqd_t)-1)
    {
        perror("mq_open");
    }

    pthread_t *s_pThread = (pthread_t *)malloc(nthreads * sizeof(pthread_t));
    DB_MANAGER_TASK_T *s_pThreadInfo = (DB_MANAGER_TASK_T *)malloc(nthreads * sizeof(DB_MANAGER_TASK_T));

    for (int i = 0; i < nthreads; ++i)
    {
        s_pThreadInfo[i].nthreads = nthreads;
        s_pThreadInfo[i].tid = i;
        s_pThreadInfo[i].pmqdes = &s_stMqdes;
        if (pthread_create(&s_pThread[i], NULL, &P_DB_MANAGER_Task, &s_pThreadInfo[i]) != 0)
        {
            PrintError("pthread_create() is failed!!");
        }
    }

#if defined(CONFIG_MULTI_THREAD)
    // Join all the threads
    for (int i = 0; i < nthreads; ++i)
    {
        pthread_join(s_pThreadInfo[i], NULL);
    }
#endif

    return FRAMEWORK_OK;
}
static int32_t P_DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    struct mq_attr attr;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    free(s_pThread);
    free(s_pThreadInfo);

    mq_getattr(s_stMqdes, &attr);
    assert(attr.mq_curmsgs == 0);

    mq_close(s_stMqdes);
    mq_unlink(s_cQueueName);

    return nRet;
}

int32_t DB_MANAGER_Write(DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

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

    for (int i = 0; i < 10; ++i)
    {
        char *text = NULL;
        nRet = asprintf(&text, "Hello mqdes %d", i);
        if (nRet < 0)
        {
            PrintError("asprintf() is failed!!");
        }

        DB_MANAGER_MSG_T *pstMsg = P_DB_MANAGER_MsgNew(10 - i, text);
        free(text);

        nRet = P_DB_MANAGER_MsgSend(s_stMqdes, pstMsg, 1);
        if (nRet != FRAMEWORK_OK)
        {
            PrintError("P_DB_MANAGER_MsgSend() is failed! [unRet:%d]", nRet);
        }

        P_DB_MANAGER_MsgFree(pstMsg);
    }

    {
        char *pcPayload = (char*)malloc(sizeof(char)*pstDbV2x->ulPayloadLength);
        memcpy(pcPayload, (char *)pPayload, pstDbV2x->ulPayloadLength);
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
            }
        }

        fprintf(s_pDbManagerFd, "eDeviceType[%d], ", pstDbV2x->eDeviceType);
        fprintf(s_pDbManagerFd, "eTeleCommType[%d], ", pstDbV2x->eTeleCommType);
        fprintf(s_pDbManagerFd, "unDeviceId[0x%x], ", pstDbV2x->unDeviceId);
        fprintf(s_pDbManagerFd, "ulTimeStamp[%ld], ", pstDbV2x->ulTimeStamp);
        fprintf(s_pDbManagerFd, "eServiceId[%d], ", pstDbV2x->eServiceId);
        fprintf(s_pDbManagerFd, "eActionType[%d], ", pstDbV2x->eActionType);
        fprintf(s_pDbManagerFd, "eRegionId[%d], ", pstDbV2x->eRegionId);
        fprintf(s_pDbManagerFd, "ePayloadType[%d], ", pstDbV2x->ePayloadType);
        fprintf(s_pDbManagerFd, "eCommId[%d], ", pstDbV2x->eCommId);
        fprintf(s_pDbManagerFd, "usDbVer[%d.%d], ", pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
        fprintf(s_pDbManagerFd, "usHwVer[0x%x], ", pstDbV2x->usHwVer);
        fprintf(s_pDbManagerFd, "usSwVer[0x%x], ", pstDbV2x->usSwVer);
        fprintf(s_pDbManagerFd, "ulPayloadLength[%d], ", pstDbV2x->ulPayloadLength);

        fprintf(s_pDbManagerFd, "cPayload[");
        for(int i = 0; i < (int)pstDbV2x->ulPayloadLength; i++)
        {
              fprintf(s_pDbManagerFd, "%d ", pcPayload[i]);
        }
        fprintf(s_pDbManagerFd, "], ");

        fprintf(s_pDbManagerFd, "ulPayloadCrc32[0x%x]", pstDbV2x->ulPacketCrc32);
        fprintf(s_pDbManagerFd, "\r\n");

        nRet = fflush(s_pDbManagerFd);
        if (nRet < 0)
        {
            PrintError("fflush() is failed! [unRet:%d]", nRet);
        }

        if(s_pDbManagerFd != NULL)
        {
            nRet = fclose(s_pDbManagerFd);
            if (nRet < 0)
            {
                PrintError("fclose() is failed! [unRet:%d]", nRet);
            }
            else
            {
                PrintTrace("DB_MANAGER_DEFAULT_FILE_NAME[%s] is closed.", DB_MANAGER_DEFAULT_FILE_NAME);
                s_pDbManagerFd = NULL;
            }
        }

        free(pcPayload);
    }

#if 0
    // Send empty message to all threads to signal termination
    for (int i = 0; i < nthreads; ++i)
    {
        if (mq_send(s_stMqdes, "", 0, 0) != 0)
        {
            PrintError("mq_send terminate"); // mq_send sets errno
        }
        else
        {
            PrintWarn("mq_sed() mq_send terminate, [i:%d]", i); // mq_send sets errno
        }
    }
#endif
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

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Close(DB_MANAGER_T *pstDbManager)
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

