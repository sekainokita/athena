#ifndef _FRAMEWORK_H_
#define _FRAMEWORK_H_

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
* @file framework.h
*
* @note
*
* V2X Framework Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "msg_manager.h"
#include "db_manager.h"
#include "time_manager.h"
#include "multi_msg_manager.h"
#include "multi_db_manager.h"

/***************************** Definition ************************************/
#define FRAMEWORK_DB_TASK_MSG_KEY               (0x840919)
#define FRAMEWORK_MSG_TX_TASK_MSG_KEY           (0x121202)
#define FRAMEWORK_MSG_RX_TASK_MSG_KEY           (0x220916)
#define FRAMEWORK_TIME_TASK_MSG_KEY             (0x230525)

//#define CONFIG_PTHREAD_JOINABLE                 (1)

/***************************** Enum and Structure ****************************/

typedef enum {
    FRAMEWORK_LOG_ALL                   = 0,
    FRAMEWORK_LOG_MGR_ALL               = 10,
    FRAMEWORK_LOG_MGR_MSG               = 11,
    FRAMEWORK_LOG_MGR_DB                = 12,
    FRAMEWORK_LOG_MGR_TIME              = 13,
    FRAMEWORK_LOG_SVC_ALL               = 100,
    FRAMEWORK_LOG_SVC_PLATOONING        = 101,
    FRAMEWORK_LOG_MAX                   = 0xFFFF
} FRAMEWORK_LOG_E;

typedef struct FRAMEWORK {
    FRAMEWORK_LOG_E eFrameworkLog;
    uint32_t unReserved;
} FRAMEWORK_T;


/***************************** Function Protype ******************************/
void FRAMEWORK_SetLog(FRAMEWORK_T *pstFramework, bool bOnOff);
int32_t FRAMEWORK_Init(FRAMEWORK_T *pstFramework);
int32_t FRAMEWORK_Multi_Init(FRAMEWORK_T * pstFrameworkMulti);
MSG_MANAGER_T* FRAMEWORK_GetMsgManagerInstance(void);
DB_MANAGER_T* FRAMEWORK_GetDbManagerInstance(void);
TIME_MANAGER_T* FRAMEWORK_GetTimeManagerInstance(void);
MULTI_MSG_MANAGER_T* FRAMEWORK_GetMultiMsgManagerInstance(void);
MULTI_DB_MANAGER_T* FRAMEWORK_GetMultiDbManagerInstance(void);

#endif  /* _FRAMEWORK_H_ */

