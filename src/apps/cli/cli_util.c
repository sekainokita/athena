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
* @file cli_util.c
*
* This file contains a CLI design
*
* @note
*
* CLI Source File
*
*
******************************************************************************/

/***************************** Include ***************************************/
#include "cli.h"

/***************************** Definition ************************************/


/***************************** Static Variable *******************************/


/***************************** Function  *************************************/

void CLI_UTIL_Enqueue(CLI_UTIL_QUEUE_T *qb, CLI_UTIL_QUEUE_T *item)
{
    qb->q_prev->q_next = item;
    item->q_next = qb;
    item->q_prev = qb->q_prev;
    qb->q_prev = item;
}

void CLI_UTIL_Dequeue(CLI_UTIL_QUEUE_T *item)
{
    item->q_prev->q_next = item->q_next;
    item->q_next->q_prev = item->q_prev;
}

CLI_UTIL_QUEUE_T *CLI_UTIL_DequeueNext(CLI_UTIL_QUEUE_T *qb)
{
    if (qb->q_next == qb)
    {
        return NULL;
    }

    qb = qb->q_next;

    qb->q_prev->q_next = qb->q_next;
    qb->q_next->q_prev = qb->q_prev;

    return qb;
}

int CLI_UTIL_QueueMap(CLI_UTIL_QUEUE_T *qb, int (*func)(CLI_UTIL_QUEUE_T *, unsigned int, unsigned int), unsigned int a, unsigned int b)
{
    CLI_UTIL_QUEUE_T *qe;
    CLI_UTIL_QUEUE_T *nextq;
    int res;

    qe = qb;

    qe = qb->q_next;

    while (qe != qb)
    {
        nextq = qe->q_next;
        if ((res = (*func)(qe, a, b)))
        {
            return res;
        }
        qe = nextq;
    }

    return 0;
}

int CLI_UTIL_CountQueue(CLI_UTIL_QUEUE_T *qb)
{
    CLI_UTIL_QUEUE_T *qe;
    int res = 0;

    qe = qb;

    while (qe->q_next != qb)
    {
        qe = qe->q_next;
        res++;
    }

    return res;
}

int CLI_UTIL_FindQueue(CLI_UTIL_QUEUE_T *qb, CLI_UTIL_QUEUE_T *item)
{
    CLI_UTIL_QUEUE_T *q;
    int res = 1;

    q = qb->q_next;

    while (q != item)
    {
        if (q == qb)
        {
            return 0;
        }

        q = q->q_next;
        res++;
    }

    return res;
}

