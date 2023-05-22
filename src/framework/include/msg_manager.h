#ifndef	_MSG_MANAGER_H_
#define	_MSG_MANAGER_H_

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
* @file msg_manager.h
*
* @note
*
* MSG Manager Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "db_v2x.h"

/***************************** Definition ************************************/


/***************************** Enum and Structure ****************************/
typedef struct MSG_MANAGER_WRITE {
    uint32_t unReserved;
} MSG_MANAGER_TX_T;

typedef struct MSG_MANAGER_READ {
    uint32_t unReserved;
} MSG_MANAGER_RX_T;

typedef struct MSG_MANAGER {
    uint32_t unReserved;
} MSG_MANAGER_T;

/***************************** Function Protype ******************************/

int32_t MSG_MANAGER_Write(MSG_MANAGER_TX_T *pstMsgManagerWrite, DB_V2X_T *pstDbV2x, void *pPayload);
int32_t MSG_MANAGER_Read(MSG_MANAGER_RX_T *pstMsgManagerRead, DB_V2X_T *pstDbV2x, void *pPayload);

int32_t MSG_MANAGER_Open(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Close(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Start(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Stop(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Status(MSG_MANAGER_T *pstMsgManager);

int32_t MSG_MANAGER_Init(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_DeInit(MSG_MANAGER_T *pstMsgManager);

#endif	/* _MSG_MANAGER_H_ */

