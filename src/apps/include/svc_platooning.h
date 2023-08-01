#ifndef	_SVC_PLATOONING_H_
#define	_SVC_PLATOONING_H_

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
* @file db_manager.h
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
#define SVC_PLATOONING_TASK_MSG_KEY               (0x230531)

/***************************** Enum and Structure ****************************/

/**
* @details SVC_PLATOONING_SETTING_T
* @param unReserved
*/
typedef struct SVC_PLATOONING_SETTING_t {
    uint32_t                     unReserved;
} SVC_PLATOONING_SETTING_T;

/**
* @details SVC_PLATOONING_EVENT_MSG_T
* @param pstTimeMgrSetting
*/
typedef struct SVC_PLATOONING_EVENT_MSG_t {
    SVC_PLATOONING_SETTING_T      *pstSvcPlatooningSetting;
} SVC_PLATOONING_EVENT_MSG_T;

/**
* @details SVC_PLATOONING_T
* @param bLogLevel
* @param unReserved
*/
typedef struct SVC_PLATOONING_t {
    bool                    bLogLevel;
    uint32_t                unReserved;
} SVC_PLATOONING_T;

/***************************** Function Protype ******************************/

int32_t SVC_PLATOONING_SetLog(SVC_PLATOONING_T *pstSvcPlatooning);

int32_t SVC_PLATOONING_Open(SVC_PLATOONING_T *pstSvcPlatooning);
int32_t SVC_PLATOONING_Close(SVC_PLATOONING_T *pstSvcPlatooning);
int32_t SVC_PLATOONING_Start(SVC_PLATOONING_T *pstSvcPlatooning);
int32_t SVC_PLATOONING_Stop(SVC_PLATOONING_T *pstSvcPlatooning);
int32_t SVC_PLATOONING_Status(SVC_PLATOONING_T *pstSvcPlatooning);

int32_t SVC_PLATOONING_Init(SVC_PLATOONING_T *pstSvcPlatooning);
int32_t SVC_PLATOONING_DeInit(SVC_PLATOONING_T *pstSvcPlatooning);

#endif	/* _SVC_PLATOONING_H_ */

