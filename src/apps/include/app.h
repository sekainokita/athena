#ifndef	_APP_H_
#define	_APP_H_

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
* @file app.h
*
* @note
*
* App Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "framework.h"
#include "di.h"
#include "svc_platooning.h"
#include "svc_cp.h"
#include "svc_mcp.h"

/***************************** Definition ************************************/
#define APP_VER     0x1

/***************************** Enum and Structure ****************************/
typedef struct APP {
    LOG_TYPE_E eLog;
    LOG_LEVEL_E eLogLevel;
    uint32_t unReserved;
} APP_T;

/***************************** Function Protype ******************************/
FRAMEWORK_T* APP_GetFrameworkInstance(void);
APP_T* APP_GetAppInstance(void);
DI_T* APP_GetDiInstance(void);
SVC_CP_T* APP_GetSvcCpInstance(void);
SVC_MCP_T* APP_GetSvcMCpInstance(void);
SVC_PLATOONING_T* APP_GetSvcPlatooningInstance(void);

int32_t APP_Init(APP_T *pstApp);

#endif	/* _APP_H_ */


