#ifndef	_DB_MANAGER_H_
#define	_DB_MANAGER_H_

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


/***************************** Enum and Structure ****************************/
typedef struct DB_MANAGER_MSG_t {
    int id;
    char *text;
} DB_MANAGER_MSG_T;

typedef struct DB_MANAGER_ARCHIVE_t {
    size_t size;
    char *data;
} DB_MANAGER_ARCHIVE_T;

typedef struct DB_MANAGER_TASK_t {
    int tid;
    mqd_t *pmqdes;
} DB_MANAGER_TASK_T;

typedef struct DB_MANAGER_WRITE {
    uint32_t unReserved;
} DB_MANAGER_WRITE_T;

typedef struct DB_MANAGER_READ {
    uint32_t unReserved;
} DB_MANAGER_READ_T;

typedef struct DB_MANAGER {
    uint32_t unReserved;
} DB_MANAGER_T;

/***************************** Function Protype ******************************/

uint32_t DB_MANAGER_Write(DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload);
uint32_t DB_MANAGER_Read(DB_MANAGER_READ_T *pstDbManagerRead, DB_V2X_T *pstDbV2x, void* pPayload);
uint32_t DB_MANAGER_Converter(DB_MANAGER_READ_T *pstDbManagerRead, DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload);

uint32_t DB_MANAGER_Open(DB_MANAGER_T *pstDbManager);
uint32_t DB_MANAGER_Close(DB_MANAGER_T *pstDbManager);
uint32_t DB_MANAGER_Start(DB_MANAGER_T *pstDbManager);
uint32_t DB_MANAGER_Stop(DB_MANAGER_T *pstDbManager);
uint32_t DB_MANAGER_Status(DB_MANAGER_T *pstDbManager);

uint32_t DB_MANAGER_Init(DB_MANAGER_T *pstDbManager);
uint32_t DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager);

#endif	/* _DB_MANAGER_H_ */

