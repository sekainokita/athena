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
#define CLI_DB_V2X_DEFAULT_DEVICE_ID              0x23040015
#define CLI_DB_V2X_DEFAULT_TIMESTAMP              2023032314344766828
#define CLI_DB_V2X_DEFAULT_HW_VER                 0x0001 // OBU rel. 230518
#define CLI_DB_V2X_DEFAULT_SW_VER                 0x0001 // OBU rel. 230523
#define CLI_DB_V2X_DEFAULT_PAYLOAD_LEN            1024

/***************************** Enum and Structure ****************************/

/**
* @details DB_MANAGER_FILE_TYPE_E
* @param DB_MANAGER_FILE_TYPE_TXT
* @param DB_MANAGER_FILE_TYPE_CSV
* @param DB_MANAGER_FILE_TYPE_SQLITE
*/
typedef enum {
    DB_MANAGER_FILE_TYPE_UNKNOWN                  = 0x0000,
    DB_MANAGER_FILE_TYPE_TXT                      = 0x0001,
    DB_MANAGER_FILE_TYPE_CSV                      = 0x0002,
    DB_MANAGER_FILE_TYPE_SQLITE                   = 0x0003,
    DB_MANAGER_FILE_TYPE_UNDEFINED_1,
    DB_MANAGER_FILE_TYPE_UNDEFINED_2,
    DB_MANAGER_FILE_TYPE_UNDEFINED_3,
    DB_MANAGER_FILE_TYPE_MAX                      = 0xFFFF
} DB_MANAGER_FILE_TYPE_E;

/**
* @details DB_MANAGER_COMM_MSG_TYPE_E
* @param DB_MANAGER_COMM_MSG_TYPE_TX
* @param DB_MANAGER_COMM_MSG_TYPE_RX
*/
typedef enum {
    DB_MANAGER_COMM_MSG_TYPE_UNKNOW               = 0x0000,
    DB_MANAGER_COMM_MSG_TYPE_TX                   = 0x0001,
    DB_MANAGER_COMM_MSG_TYPE_RX                   = 0x0002,
    DB_MANAGER_COMM_MSG_TYPE_MAX                  = 0xFFFF
} DB_MANAGER_COMM_MSG_TYPE_E;

/**
* @details DB_MANAGER_PROC_E
* @param DB_MANAGER_PROC_WRITE
* @param DB_MANAGER_PROC_CONVERT
*/
typedef enum {
    DB_MANAGER_PROC_UNKNOWN                        = 0x0000,
    DB_MANAGER_PROC_WRITE                          = 0x0001,
    DB_MANAGER_PROC_READ                           = 0x0002,
    DB_MANAGER_PROC_CONVERT                        = 0x0003,
    DB_MANAGER_PROC_MAX                            = 0xFFFF
} DB_MANAGER_PROC_E;

/**
* @details DB_MANAGER_WRITE_T
* @param eFileType
* @param eCommType
* @param eProc
* @param unReserved
*/
typedef struct DB_MANAGER_WRITE {
    DB_MANAGER_FILE_TYPE_E      eFileType;
    DB_MANAGER_COMM_MSG_TYPE_E  eCommMsgType;
    DB_MANAGER_PROC_E           eProc;
    uint32_t                    unReserved;
} DB_MANAGER_WRITE_T;

/**
* @details DB_MANAGER_READ_T
* @param eFileType
* @param eProc
*/
typedef struct DB_MANAGER_READ {
    DB_MANAGER_FILE_TYPE_E      eFileType;
    DB_MANAGER_PROC_E           eProc;
    uint32_t                    unReserved;
} DB_MANAGER_READ_T;

/**
* @details DB_MANAGER_EVENT_MSG_T
* @param pstDbManagerWrite
* @param pstDbV2x
* @param pPayload
*/
typedef struct DB_MANAGER_EVENT_MSG {
    DB_MANAGER_WRITE_T      *pstDbManagerWrite;
    DB_V2X_T                *pstDbV2x;
    void                    *pPayload;
} DB_MANAGER_EVENT_MSG_T;

/**
* @details DB_MANAGER_T
* @param eFileType
* @param unReserved
*/
typedef struct DB_MANAGER {
    DB_MANAGER_FILE_TYPE_E  eFileType;
    bool                    bLogLevel;
    uint32_t                unReserved;
} DB_MANAGER_T;

/***************************** Function Protype ******************************/

int32_t DB_MANAGER_Write(DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void *pPayload);
int32_t DB_MANAGER_Read(DB_MANAGER_READ_T *pstDbManagerRead, DB_V2X_T *pstDbV2x, void *pPayload);
int32_t DB_MANAGER_Converter(DB_MANAGER_READ_T *pstDbManagerRead, DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void *pPayload);

int32_t DB_MANAGER_SetLog(DB_MANAGER_T *pstDbManager);

int32_t DB_MANAGER_Open(DB_MANAGER_T *pstDbManager);
int32_t DB_MANAGER_Close(DB_MANAGER_T *pstDbManager);
int32_t DB_MANAGER_Start(DB_MANAGER_T *pstDbManager);
int32_t DB_MANAGER_Stop(DB_MANAGER_T *pstDbManager);
int32_t DB_MANAGER_Status(DB_MANAGER_T *pstDbManager);

int32_t DB_MANAGER_Init(DB_MANAGER_T *pstDbManager);
int32_t DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager);

#endif	/* _DB_MANAGER_H_ */

