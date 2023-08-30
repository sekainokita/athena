#ifndef	_SVC_CP_H_
#define	_SVC_CP_H_

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
* @file svc_cp.h
*
* @note
*
* DB Manager Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "db_v2x.h"

/***************************** Definition ************************************/
#define SVC_CP_TASK_MSG_KEY               (0x230531)
#define SVC_CP_DEFAULT_ETH_DEV            "eth1"
/***************************** Enum and Structure ****************************/

/**
* @details SVC_CP_SETTING_T
* @param unReserved
*/
typedef struct SVC_CP_SETTING_t {
    uint32_t                     unReserved;
} SVC_CP_SETTING_T;

/**
* @details SVC_CP_EVENT_MSG_T
* @param pstTimeMgrSetting
*/
typedef struct SVC_CP_EVENT_MSG_t {
    SVC_CP_SETTING_T      *pstSvcCpSetting;
} SVC_CP_EVENT_MSG_T;

/**
* @details SVC_CP_T
* @param bLogLevel
* @param unReserved
*/
typedef struct SVC_CP_t {
    bool                    bLogLevel;
    DB_MANAGER_WRITE_T      stDbManagerWrite;
    MSG_MANAGER_TX_T        stMsgManagerTx;
    MSG_MANAGER_RX_T        stMsgManagerRx;
    DB_V2X_T                stDbV2x;
    char                    *pchIfaceName;
    uint32_t                unReserved;
} SVC_CP_T;

/***************************** Function Protype ******************************/

int32_t SVC_CP_SetLog(SVC_CP_T *pstSvcCp);

int32_t SVC_CP_Open(SVC_CP_T *pstSvcCp);
int32_t SVC_CP_Close(SVC_CP_T *pstSvcCp);
int32_t SVC_CP_Start(SVC_CP_T *pstSvcCp);
int32_t SVC_CP_Stop(SVC_CP_T *pstSvcCp);
int32_t SVC_CP_Status(SVC_CP_T *pstSvcCp);

void SVC_CP_ShowSettings(SVC_CP_T *pstSvcCp);

int32_t SVC_CP_SetSettings(SVC_CP_T *pstSvcCp);
int32_t SVC_CP_GetSettings(SVC_CP_T *pstSvcCp);

int32_t SVC_CP_Init(SVC_CP_T *pstSvcCp);
int32_t SVC_CP_DeInit(SVC_CP_T *pstSvcCp);

#endif	/* _SVC_CP_H_ */

