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


/***************************** Static Variable *******************************/
static mqd_t s_stMqdes;


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
            PrintDebug("%d [%ld] %d, %s", tinfo->tid, strlen(P_DB_MANAGER_MsgText(pstMsg)), P_DB_MANAGER_MsgId(pstMsg), P_DB_MANAGER_MsgText(pstMsg));
            usleep(10000); // Placeholder for long work
            P_DB_MANAGER_MsgFree(pstMsg);
        }
        else
        {
            PrintWarn("Finish!");
            break;
        }
    }

    free(buffer);

    return NULL;
}

// Main loop
static uint32_t P_DB_MANAGER_Init(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;
    int nRet = FRAMEWORK_OK;

    // Get number of system threads
    const int nthreads = sysconf(_SC_NPROCESSORS_ONLN);
    // Open a new message queue
    const char queue_name[] = "/db_manager_queue";

    PrintDebug("nthreads[%d]", nthreads);

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    s_stMqdes = mq_open(queue_name, O_CREAT | O_RDWR, 0644, NULL);
    if (s_stMqdes == (mqd_t)-1)
    {
        perror("mq_open");
    }

    // Start threads
    pthread_t *thread = (pthread_t *)malloc(nthreads * sizeof(pthread_t));
    DB_MANAGER_TASK_T *tinfo = (DB_MANAGER_TASK_T *)malloc(nthreads * sizeof(DB_MANAGER_TASK_T));

    for (int i = 0; i < nthreads; ++i)
    {
        tinfo[i].tid = i;
        tinfo[i].pmqdes = &s_stMqdes;
        if (pthread_create(&thread[i], NULL, &P_DB_MANAGER_Task, &tinfo[i]) != 0)
        {
            PrintError("pthread_create() is failed!!");
        }
    }

    // Send messages to all threads via the queue
    for (int i = 0; i < 10; ++i)
    {
        char *text = NULL;
        nRet = asprintf(&text, "Hello mqdes %d\n", i);
        if (nRet < 0)
        {
            PrintError("asprintf() is failed!!");
        }

        DB_MANAGER_MSG_T *pstMsg = P_DB_MANAGER_MsgNew(10 - i, text);
        free(text);

        if (P_DB_MANAGER_MsgSend(s_stMqdes, pstMsg, 1) != 0)
        {
            PrintError("mq_send message");
        }

        P_DB_MANAGER_MsgFree(pstMsg);
    }

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

    // Join all the threads
    for (int i = 0; i < nthreads; ++i)
    {
        pthread_join(thread[i], NULL);
    }

    free(thread);
    free(tinfo);

    // Make sure there are no messages left in the queue
    struct mq_attr attr;
    mq_getattr(s_stMqdes, &attr);
    assert(attr.mq_curmsgs == 0);

    mq_close(s_stMqdes);
    mq_unlink(queue_name);

    return FRAMEWORK_OK;
}

uint32_t DB_MANAGER_Write(DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManagerWrite == NULL)
    {
        PrintError("pstDbManagerWrite == NULL!!");
        return unRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return unRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Read(DB_MANAGER_READ_T *pstDbManagerRead, DB_V2X_T *pstDbV2x, void* pPayload)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManagerRead == NULL)
    {
        PrintError("pstDbManagerRead == NULL!!");
        return unRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return unRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Converter(DB_MANAGER_READ_T *pstDbManagerRead, DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManagerRead == NULL)
    {
        PrintError("pstDbManagerRead == NULL!!");
        return unRet;
    }

    if(pstDbManagerWrite == NULL)
    {
        PrintError("pstDbManagerWrite == NULL!!");
        return unRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return unRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Open(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Close(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Start(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Stop(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Status(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    return unRet;
}

uint32_t DB_MANAGER_Init(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    PrintWarn("is successfully initialized.");

    P_DB_MANAGER_Init(pstDbManager);

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    unRet = FRAMEWORK_OK;

    return unRet;
}

uint32_t DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager)
{
    uint32_t unRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return unRet;
    }

    return unRet;
}

