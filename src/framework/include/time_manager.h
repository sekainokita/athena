#ifndef	_TIME_MANAGER_H_
#define	_TIME_MANAGER_H_

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
* @file time_manager.h
*
* @note
*
* TIME Manager Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "db_v2x.h"

/***************************** Definition ************************************/
#define TIME_MGR_TEST_TIMESTAMP              2023032314344766828

#define USLEEP_US                            (1)
#define USLEEP_MS                            (USLEEP_US*1000)
#define USLEEP_S                             (USLEEP_MS*USLEEP_MS)

/***************************** Enum and Structure ****************************/

/**
* @details TIME_MANAGER_SETTING_T
* @param unReserved
*/
typedef struct TIME_MANAGER_SETTING {
    uint32_t                     unReserved;
} TIME_MANAGER_SETTING_T;

/**
* @details TIME_MANAGER_EVENT_MSG_T
* @param pstTimeMgrSetting
*/
typedef struct TIME_MANAGER_EVENT_MSG {
    TIME_MANAGER_SETTING_T      *pstTimeMgrSetting;
} TIME_MANAGER_EVENT_MSG_T;

typedef struct TIME_MANAGER_LATENCY_t {
    uint64_t                    ulTime_s;
    uint64_t                    ulTime_ms;
} TIME_MANAGER_LATENCY_T;

/**
* @details TIME_MANAGER_T
* @param unReserved
*/
typedef struct TIME_MANAGER_t {
    TIME_MANAGER_LATENCY_T      stLatency;
    uint64_t                    ulTimeStamp;
    bool                        bLogLevel;
    uint32_t                    unReserved;
} TIME_MANAGER_T;

/***************************** Function Protype ******************************/

int32_t TIME_MANAGER_Init(TIME_MANAGER_T *pstTimeMgr);
int32_t TIME_MANAGER_DeInit(TIME_MANAGER_T *pstTimeMgr);

int32_t TIME_MANAGER_SetLog(TIME_MANAGER_T *pstTimeMgr);
int32_t TIME_MANAGER_Get(TIME_MANAGER_T *pstTimeMgr);

int32_t TIME_MANAGER_Open(TIME_MANAGER_T *pstTimeMgr);
int32_t TIME_MANAGER_Close(TIME_MANAGER_T *pstTimeMgr);
int32_t TIME_MANAGER_Start(TIME_MANAGER_T *pstTimeMgr);
int32_t TIME_MANAGER_Stop(TIME_MANAGER_T *pstTimeMgr);

void TIME_MANAGER_Status(TIME_MANAGER_T *pstTimeMgr);
int32_t TIME_MANAGER_Init(TIME_MANAGER_T *pstTimeMgr);
int32_t TIME_MANAGER_DeInit(TIME_MANAGER_T *pstTimeMgr);

void TIME_MANAGER_CheckLatencyTime(char *pStr, TIME_MANAGER_T *pstTimeMgr);
void TIME_MANAGER_CheckLatencyBegin(TIME_MANAGER_T *pstTimeMgr);
void TIME_MANAGER_CheckLatencyEnd(TIME_MANAGER_T *pstTimeMgr);

#endif	/* _TIME_MANAGER_H_ */

