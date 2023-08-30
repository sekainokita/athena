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
#include "framework.h"
#include "db_manager.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#if defined(CONFIG_SQLITE)
#include <sqlite3.h>
#endif
#include "db_v2x_status.h"

/***************************** Definition ************************************/
#define DB_MANAGER_TXT_TX_FILE     "db_v2x_tx_temp_writing.txt"
#define DB_MANAGER_TXT_RX_FILE     "db_v2x_rx_temp_writing.txt"

#define DB_MANAGER_CSV_TX_FILE     "db_v2x_tx_temp_writing.csv"
#define DB_MANAGER_CSV_RX_FILE     "db_v2x_rx_temp_writing.csv"

#if defined(CONFIG_SQLITE)
#define DB_MANAGER_SQL_TX_FILE     "db_v2x_tx_temp_writing.db"
#define DB_MANAGER_SQL_RX_FILE     "db_v2x_rx_temp_writing.db"
#endif

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
FILE* sh_pDbMgrTxMsg;
FILE* sh_pDbMgrRxMsg;
#if defined(CONFIG_SQLITE)
sqlite3* sh_pDbMgrTxSqlMsg;
sqlite3* sh_pDbMgrRxSqlMsg;
#endif

static int s_nDbTaskMsgId, s_nMsgTxTaskMsgId, s_nMsgRxTaskMsgId;
static key_t s_dbTaskMsgKey = FRAMEWORK_DB_TASK_MSG_KEY;
static key_t s_MsgTxTaskMsgKey = FRAMEWORK_MSG_TX_TASK_MSG_KEY;
static key_t s_MsgRxTaskMsgKey = FRAMEWORK_MSG_RX_TASK_MSG_KEY;

static pthread_t sh_DbMgrTask;

static bool s_bDbMgrLog = OFF;

/***************************** Function  *************************************/

#if defined(CONFIG_SQLITE)
static int32_t P_DB_MANAGER_WriteSqlite(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    char *ErrorMsg = NULL;
    int sql_TxStatus;
    int sql_RxStatus;
    const char* TxCreate = NULL;
    const char* RxCreate = NULL;
    char *InsertTxData = (char *)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
    if(InsertTxData == NULL)
        {
            PrintError("InsertTxData_malloc() is failed! [NULL]");
            return nRet;
        }
    char *InsertRxData = (char *)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
    if(InsertRxData == NULL)
        {
            PrintError("InsertRxData_malloc() is failed! [NULL]");
            return nRet;
        }

    switch (pstEventMsg->pstDbManagerWrite->eCommMsgType)
    {
        case DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pDbMgrTxSqlMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

                // Connect Database
                sql_TxStatus = sqlite3_open(DB_MANAGER_SQL_TX_FILE, &sh_pDbMgrTxSqlMsg);
                if (sql_TxStatus != SQLITE_OK)
                {
                    PrintError("Can't open Tx database : %s\n", sqlite3_errmsg(sh_pDbMgrTxSqlMsg));
                    sql_TxStatus = sqlite3_close(sh_pDbMgrTxSqlMsg);
                    if(sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pDbMgrTxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                }
                // Create Table
                if(TxCreate == NULL)
                {
                    TxCreate = "CREATE TABLE IF NOT EXISTS Txtable (eDeviceType INTEGER, eTeleCommType INTEGER, unDeviceId INTEGER, ulTimeStamp INTEGER,\
                    eServiceId INTEGER, eActionType INTEGER, eRegionId INTEGER, ePayloadType INTEGER, eCommId INTEGER, usDbVer INTEGER, usHwVer INTEGER, usSwVer INTEGER,\
                    ulPayloadLength INTEGER, unTotalpacketCrc32 INTEGER)";
                    sql_TxStatus = sqlite3_exec(sh_pDbMgrTxSqlMsg, TxCreate, 0, 0, &ErrorMsg);
                    if (sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't create Txtable : %s\n", ErrorMsg);
                        (void)sqlite3_free(ErrorMsg);
                        sql_TxStatus = sqlite3_close(sh_pDbMgrTxSqlMsg);
                        if(sql_TxStatus != SQLITE_OK)
                        {
                            PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pDbMgrTxSqlMsg));
                            nRet = FRAMEWORK_ERROR;
                            return nRet;
                        }
                    }
                }
                // Insert data (execpt cPayload)
                if (InsertTxData != NULL)
                {
                    sprintf(InsertTxData,\
                    "INSERT INTO Txtable(eDeviceType, eTeleCommType, unDeviceId, ulTimeStamp, eServiceId, eActionType, eRegionId, ePayloadType, eCommId, usDbVer, usHwVer,\
                    usSwVer, ulPayloadLength, unTotalpacketCrc32)\
                    VALUES (%d, %d, 0x%x, %ld, %d, %d, %d, %d, %d, %d.%d, 0x%x, 0x%x, %d, 0x%x)",\
                    pstEventMsg->pstDbV2x->eDeviceType,\
                    pstEventMsg->pstDbV2x->eTeleCommType,\
                    pstEventMsg->pstDbV2x->unDeviceId,\
                    pstEventMsg->pstDbV2x->ulTimeStamp,\
                    pstEventMsg->pstDbV2x->eServiceId,\
                    pstEventMsg->pstDbV2x->eActionType,\
                    pstEventMsg->pstDbV2x->eRegionId,\
                    pstEventMsg->pstDbV2x->ePayloadType,\
                    pstEventMsg->pstDbV2x->eCommId,\
                    pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK,\
                    pstEventMsg->pstDbV2x->usHwVer,\
                    pstEventMsg->pstDbV2x->usSwVer,\
                    pstEventMsg->pstDbV2x->ulPayloadLength,\
                    pstEventMsg->pstDbManagerWrite->unCrc32);
                }
                sql_TxStatus = sqlite3_exec(sh_pDbMgrTxSqlMsg, InsertTxData, 0, 0, &ErrorMsg);
                if (sql_TxStatus != SQLITE_OK)
                {
                    PrintError("Can't insert Txtable : %s\n", ErrorMsg);
                    (void)sqlite3_free(ErrorMsg);
                    sql_TxStatus = sqlite3_close(sh_pDbMgrTxSqlMsg);
                    if(sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pDbMgrTxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                }
                // Close Database Connection
                sql_TxStatus = sqlite3_close(sh_pDbMgrTxSqlMsg);
                if(sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pDbMgrTxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                nRet = FRAMEWORK_OK;
                return nRet;
            }
            else
            {
                PrintError("sh_pDbMgrTxSqlMsg is NULL!!, check whethter sh_pDbMgrTxSqlMsg is opened before.");
            }

            break;
        }
        case DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pDbMgrRxSqlMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

                // Connect Database
                sql_RxStatus = sqlite3_open(DB_MANAGER_SQL_RX_FILE, &sh_pDbMgrRxSqlMsg);
                if (sql_RxStatus != SQLITE_OK)
                {
                    PrintError("Can't open Rx database : %s\n", sqlite3_errmsg(sh_pDbMgrRxSqlMsg));
                    sql_RxStatus = sqlite3_close(sh_pDbMgrRxSqlMsg);
                    if(sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pDbMgrRxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                }
                // Create Table
                if(RxCreate == NULL)
                {
                    RxCreate = "CREATE TABLE IF NOT EXISTS Rxtable (eDeviceType INTEGER, eTeleCommType INTEGER, unDeviceId INTEGER, ulTimeStamp INTEGER,\
                    eServiceId INTEGER, eActionType INTEGER, eRegionId INTEGER, ePayloadType INTEGER, eCommId INTEGER, usDbVer INTEGER, usHwVer INTEGER, usSwVer INTEGER,\
                    ulPayloadLength INTEGER, unTotalpacketCrc32 INTEGER)";
                    sql_RxStatus = sqlite3_exec(sh_pDbMgrRxSqlMsg, RxCreate, 0, 0, &ErrorMsg);
                    if (sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't create Rxtable : %s\n", ErrorMsg);
                        (void)sqlite3_free(ErrorMsg);
                        sql_RxStatus = sqlite3_close(sh_pDbMgrRxSqlMsg);
                        if(sql_RxStatus != SQLITE_OK)
                        {
                            PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pDbMgrRxSqlMsg));
                            nRet = FRAMEWORK_ERROR;
                            return nRet;
                        }
                    }
                }
                // Insert data (execpt cPayload)
                if (InsertRxData != NULL)
                {
                    sprintf(InsertRxData,\
                    "INSERT INTO Rxtable(eDeviceType, eTeleCommType, unDeviceId, ulTimeStamp, eServiceId, eActionType, eRegionId, ePayloadType, eCommId, usDbVer, usHwVer,\
                    usSwVer, ulPayloadLength, unTotalpacketCrc32)\
                    VALUES (%d, %d, 0x%x, %ld, %d, %d, %d, %d, %d, %d.%d, 0x%x, 0x%x, %d, 0x%x)",\
                    pstEventMsg->pstDbV2x->eDeviceType,\
                    pstEventMsg->pstDbV2x->eTeleCommType,\
                    pstEventMsg->pstDbV2x->unDeviceId,\
                    pstEventMsg->pstDbV2x->ulTimeStamp,\
                    pstEventMsg->pstDbV2x->eServiceId,\
                    pstEventMsg->pstDbV2x->eActionType,\
                    pstEventMsg->pstDbV2x->eRegionId,\
                    pstEventMsg->pstDbV2x->ePayloadType,\
                    pstEventMsg->pstDbV2x->eCommId,\
                    pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK,\
                    pstEventMsg->pstDbV2x->usHwVer,\
                    pstEventMsg->pstDbV2x->usSwVer,\
                    pstEventMsg->pstDbV2x->ulPayloadLength,\
                    pstEventMsg->pstDbManagerWrite->unCrc32);
                }
                sql_RxStatus = sqlite3_exec(sh_pDbMgrRxSqlMsg, InsertRxData, 0, 0, &ErrorMsg);
                if (sql_RxStatus != SQLITE_OK)
                {
                    PrintError("Can't insert Rxtable : %s\n", ErrorMsg);
                    (void)sqlite3_free(ErrorMsg);
                    sql_RxStatus = sqlite3_close(sh_pDbMgrRxSqlMsg);
                    if(sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pDbMgrRxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                }
                // Close Database Connection
                sql_RxStatus = sqlite3_close(sh_pDbMgrRxSqlMsg);
                if(sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pDbMgrRxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                nRet = FRAMEWORK_OK;
                return nRet;
            }
            else
            {
                PrintError("sh_pDbMgrRxSqlMsg is NULL!!, check whethter sh_pDbMgrRxSqlMsg is opened before.");
            }

            break;
        }
        default:
            PrintError("unknown eCommMsgType [%d]", pstEventMsg->pstDbManagerWrite->eCommMsgType);
            break;
    }

    return nRet;
}

static int32_t P_DB_MANAGER_OpenSqlite(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    UNUSED(pstDbManager);
    int sql_TxStatus;
    int sql_RxStatus;

    if(sh_pDbMgrTxSqlMsg == NULL)
    {
        sql_TxStatus = sqlite3_open(DB_MANAGER_SQL_TX_FILE, &sh_pDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't open Tx database!!");
        }
        else
        {
            PrintTrace("DB_MANAGER_SQL_TX_FILE[%s] is opened.", DB_MANAGER_SQL_TX_FILE);
            nRet = FRAMEWORK_OK;
        }
        sql_TxStatus = sqlite3_close(sh_pDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pDbMgrTxSqlMsg));
        }
    }

    if(sh_pDbMgrRxSqlMsg == NULL)
    {
        sql_RxStatus = sqlite3_open(DB_MANAGER_SQL_RX_FILE, &sh_pDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't open Rx database!!");
        }
        else
        {
            PrintTrace("DB_MANAGER_SQL_RX_FILE[%s] is opened.", DB_MANAGER_SQL_RX_FILE);
            nRet = FRAMEWORK_OK;
        }
        sql_RxStatus = sqlite3_close(sh_pDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pDbMgrRxSqlMsg));
        }
    }

    if(sh_pDbMgrTxSqlMsg != NULL)
    {
        sql_TxStatus = sqlite3_open(DB_MANAGER_SQL_TX_FILE, &sh_pDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't open Tx database!!");
        }
        else
        {
            nRet = FRAMEWORK_OK;
        }
        sql_TxStatus = sqlite3_close(sh_pDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pDbMgrTxSqlMsg));
        }
    }

    if(sh_pDbMgrRxSqlMsg != NULL)
    {
        sql_RxStatus = sqlite3_open(DB_MANAGER_SQL_RX_FILE, &sh_pDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't open Rx database!!");
        }
        else
        {
            nRet = FRAMEWORK_OK;
        }
        sql_RxStatus = sqlite3_close(sh_pDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pDbMgrRxSqlMsg));
        }
    }

    return nRet;
}
#endif

static int32_t P_DB_MANAGER_WriteTxt(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;

    switch(pstEventMsg->pstDbManagerWrite->eCommMsgType)
    {
        case DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pDbMgrTxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pDbMgrTxMsg, "eDeviceType[%d], ", pstEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pDbMgrTxMsg, "eTeleCommType[%d], ", pstEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pDbMgrTxMsg, "unDeviceId[0x%x], ", pstEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pDbMgrTxMsg, "ulTimeStamp[%ld], ", pstEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pDbMgrTxMsg, "eServiceId[%d], ", pstEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pDbMgrTxMsg, "eActionType[%d], ", pstEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pDbMgrTxMsg, "eRegionId[%d], ", pstEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pDbMgrTxMsg, "ePayloadType[%d], ", pstEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pDbMgrTxMsg, "eCommId[%d], ", pstEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pDbMgrTxMsg, "usDbVer[%d.%d], ", pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pDbMgrTxMsg, "usHwVer[0x%x], ", pstEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pDbMgrTxMsg, "usSwVer[0x%x], ", pstEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pDbMgrTxMsg, "ulPayloadLength[%d], ", pstEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pDbMgrTxMsg, "cPayload[");
                for(int i = 0; i < (int)pstEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pDbMgrTxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pDbMgrTxMsg, "], ");

                fprintf(sh_pDbMgrTxMsg, "unTotalPacketCrc32[0x%x]", pstEventMsg->pstDbManagerWrite->unCrc32);
                fprintf(sh_pDbMgrTxMsg, "\r\n");

                nRet = fflush(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }
            }
            else
            {
                PrintError("sh_pDbMgrTxMsg is NULL!!, check whethter sh_pDbMgrTxMsg is opened before.");
            }

            break;
        }
        case DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pDbMgrRxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pDbMgrRxMsg, "eDeviceType[%d], ", pstEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pDbMgrRxMsg, "eTeleCommType[%d], ", pstEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pDbMgrRxMsg, "unDeviceId[0x%x], ", pstEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pDbMgrRxMsg, "ulTimeStamp[%ld], ", pstEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pDbMgrRxMsg, "eServiceId[%d], ", pstEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pDbMgrRxMsg, "eActionType[%d], ", pstEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pDbMgrRxMsg, "eRegionId[%d], ", pstEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pDbMgrRxMsg, "ePayloadType[%d], ", pstEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pDbMgrRxMsg, "eCommId[%d], ", pstEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pDbMgrRxMsg, "usDbVer[%d.%d], ", pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pDbMgrRxMsg, "usHwVer[0x%x], ", pstEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pDbMgrRxMsg, "usSwVer[0x%x], ", pstEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pDbMgrRxMsg, "ulPayloadLength[%d], ", pstEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pDbMgrRxMsg, "cPayload[");
                for(int i = 0; i < (int)pstEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pDbMgrRxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pDbMgrRxMsg, "], ");

                fprintf(sh_pDbMgrRxMsg, "unTotalPacketCrc32[0x%x]", pstEventMsg->pstDbManagerWrite->unCrc32);
                fprintf(sh_pDbMgrRxMsg, "\r\n");

                nRet = fflush(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }

                if(pstEventMsg->pPayload != NULL)
                {
                    if(s_bDbMgrLog == ON)
                    {
                        PrintDebug("free [%p] allocated at P_MSG_MANAGER_SendRxMsgToDbMgr()", pstEventMsg->pPayload);
                    }
                    free(pstEventMsg->pPayload);
                }
            }
            else
            {
                PrintError("sh_pDbMgrRxMsg is NULL!!, check whethter sh_pDbMgrRxMsg is opened before.");
            }

            break;
        }
        default:
            PrintError("unknown eCommMsgType [%d]", pstEventMsg->pstDbManagerWrite->eCommMsgType);
            break;
    }

    return nRet;
}

static int32_t P_DB_MANAGER_OpenCsv(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    UNUSED(pstDbManager);

    if(sh_pDbMgrTxMsg == NULL)
    {
        sh_pDbMgrTxMsg = fopen(DB_MANAGER_CSV_TX_FILE, "a+");
        if(sh_pDbMgrTxMsg == NULL)
        {
            PrintError("fopen() is failed!!");
        }
        else
        {
            PrintTrace("DB_MANAGER_TX_FILE[%s] is opened.", DB_MANAGER_CSV_TX_FILE);
            nRet = FRAMEWORK_OK;
        }
    }

    if(sh_pDbMgrRxMsg == NULL)
    {
        sh_pDbMgrRxMsg = fopen(DB_MANAGER_CSV_RX_FILE, "a+");
        if(sh_pDbMgrRxMsg == NULL)
        {
            PrintError("fopen() is failed!!");
        }
        else
        {
            PrintTrace("DB_MANAGER_RX_FILE[%s] is opened.", DB_MANAGER_CSV_RX_FILE);
            nRet = FRAMEWORK_OK;
        }
    }

    if (sh_pDbMgrTxMsg != NULL)
    {
        fprintf(sh_pDbMgrTxMsg, "eDeviceType,");
        fprintf(sh_pDbMgrTxMsg, "eTeleCommType,");
        fprintf(sh_pDbMgrTxMsg, "unDeviceId,");
        fprintf(sh_pDbMgrTxMsg, "ulTimeStamp,");
        fprintf(sh_pDbMgrTxMsg, "eServiceId,");
        fprintf(sh_pDbMgrTxMsg, "eActionType,");
        fprintf(sh_pDbMgrTxMsg, "eRegionId,");
        fprintf(sh_pDbMgrTxMsg, "ePayloadType,");
        fprintf(sh_pDbMgrTxMsg, "eCommId,");
        fprintf(sh_pDbMgrTxMsg, "usDbVer,");
        fprintf(sh_pDbMgrTxMsg, "usHwVer,");
        fprintf(sh_pDbMgrTxMsg, "usSwVer,");
        fprintf(sh_pDbMgrTxMsg, "ulPayloadLength,");
        fprintf(sh_pDbMgrTxMsg, "cPayload,");
        fprintf(sh_pDbMgrTxMsg, "unTotalPacketCrc32");
        fprintf(sh_pDbMgrTxMsg, "\r\n");
    }

    nRet = fflush(sh_pDbMgrTxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if (sh_pDbMgrRxMsg != NULL)
    {
        fprintf(sh_pDbMgrRxMsg, "eDeviceType,");
        fprintf(sh_pDbMgrRxMsg, "eTeleCommType,");
        fprintf(sh_pDbMgrRxMsg, "unDeviceId,");
        fprintf(sh_pDbMgrRxMsg, "ulTimeStamp,");
        fprintf(sh_pDbMgrRxMsg, "eServiceId,");
        fprintf(sh_pDbMgrRxMsg, "eActionType,");
        fprintf(sh_pDbMgrRxMsg, "eRegionId,");
        fprintf(sh_pDbMgrRxMsg, "ePayloadType,");
        fprintf(sh_pDbMgrRxMsg, "eCommId,");
        fprintf(sh_pDbMgrRxMsg, "usDbVer,");
        fprintf(sh_pDbMgrRxMsg, "usHwVer,");
        fprintf(sh_pDbMgrRxMsg, "usSwVer,");
        fprintf(sh_pDbMgrRxMsg, "ulPayloadLength,");
        fprintf(sh_pDbMgrRxMsg, "cPayload,");
        fprintf(sh_pDbMgrRxMsg, "unTotalPacketCrc32");
        fprintf(sh_pDbMgrRxMsg, "\r\n");
    }

    nRet = fflush(sh_pDbMgrRxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    return nRet;
}

static int32_t P_DB_MANAGER_WriteCsvPlatooningThroughput(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;

    switch(pstEventMsg->pstDbManagerWrite->eCommMsgType)
    {
        case DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pDbMgrTxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pDbMgrTxMsg, "0x%x,", pstEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pDbMgrTxMsg, "%ld,", pstEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pDbMgrTxMsg, "%d.%d,", pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pDbMgrTxMsg, "0x%x,", pstEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pDbMgrTxMsg, "0x%x,", pstEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->ulPayloadLength);

                for(int i = 0; i < (int)pstEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pDbMgrTxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pDbMgrTxMsg, ",");

                fprintf(sh_pDbMgrTxMsg, "0x%x", pstEventMsg->pstDbManagerWrite->unCrc32);
                fprintf(sh_pDbMgrTxMsg, "\r\n");

                nRet = fflush(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }
            }
            else
            {
                PrintError("sh_pDbMgrTxMsg is NULL!!, check whethter sh_pDbMgrTxMsg is opened before.");
            }

            break;
        }
        case DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pDbMgrRxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pDbMgrRxMsg, "0x%x,", pstEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pDbMgrRxMsg, "%ld,", pstEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pDbMgrRxMsg, "%d.%d,", pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pDbMgrRxMsg, "0x%x,", pstEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pDbMgrRxMsg, "0x%x,", pstEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->ulPayloadLength);

                for(int i = 0; i < (int)pstEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pDbMgrRxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pDbMgrRxMsg, ",");

                fprintf(sh_pDbMgrRxMsg, "0x%x", pstEventMsg->pstDbManagerWrite->unCrc32);
                fprintf(sh_pDbMgrRxMsg, "\r\n");

                nRet = fflush(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }

                if(pstEventMsg->pPayload != NULL)
                {
                    if(s_bDbMgrLog == ON)
                    {
                        PrintDebug("free [%p] allocated at P_MSG_MANAGER_SendRxMsgToDbMgr()", pstEventMsg->pPayload);
                    }
                    free(pstEventMsg->pPayload);
                }
            }
            else
            {
                PrintError("sh_pDbMgrRxMsg is NULL!!, check whethter sh_pDbMgrRxMsg is opened before.");
            }

            break;
        }
        default:
            PrintError("unknown eCommMsgType [%d]", pstEventMsg->pstDbManagerWrite->eCommMsgType);
            break;
    }

    return nRet;
}

static int32_t P_DB_MANAGER_WriteCsvPlatooning(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    UNUSED(pstEventMsg);
    PrintTrace("TODO : DB_V2X_PAYLOAD_TYPE_PLATOONING");
    nRet = FRAMEWORK_OK;

    return nRet;
}

static int32_t P_DB_MANAGER_WriteCsvV2xStatusTx(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    DB_V2X_STATUS_TX_T stDbV2xStatusTx;

    pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
    if(pchPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eDeviceType);
    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eTeleCommType);
    fprintf(sh_pDbMgrTxMsg, "0x%x,", pstEventMsg->pstDbV2x->unDeviceId);
    fprintf(sh_pDbMgrTxMsg, "%ld,", pstEventMsg->pstDbV2x->ulTimeStamp);
    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eServiceId);
    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eActionType);
    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eRegionId);
    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->ePayloadType);
    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->eCommId);
    fprintf(sh_pDbMgrTxMsg, "%d.%d,", pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
    fprintf(sh_pDbMgrTxMsg, "0x%x,", pstEventMsg->pstDbV2x->usHwVer);
    fprintf(sh_pDbMgrTxMsg, "0x%x,", pstEventMsg->pstDbV2x->usSwVer);
    fprintf(sh_pDbMgrTxMsg, "%d,", pstEventMsg->pstDbV2x->ulPayloadLength);

    memcpy(&stDbV2xStatusTx, pchPayload, sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pDbMgrTxMsg, "%ld,", stDbV2xStatusTx.ulTxTimeStampL1);
    fprintf(sh_pDbMgrTxMsg, "%ld,", stDbV2xStatusTx.ulTxTimeStampL2);
    fprintf(sh_pDbMgrTxMsg, "%ld,", stDbV2xStatusTx.ulTxTimeStampL3);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.unRxDeviceId);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.eChannel);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.sPower);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.eBandwidth);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.eChannel);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.usTxRatio);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.stTxPosition.nTxLatitude);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.stTxPosition.nTxLongitude);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.stTxPosition.nTxAttitude);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.unSeqNum);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.unContCnt);
    fprintf(sh_pDbMgrTxMsg, "%d,", stDbV2xStatusTx.unTxVehicleSpeed);

    fprintf(sh_pDbMgrTxMsg, "0x%x", pstEventMsg->pstDbManagerWrite->unCrc32);
    fprintf(sh_pDbMgrTxMsg, "\r\n");

    nRet = fflush(sh_pDbMgrTxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if(pchPayload != NULL)
    {
        free(pchPayload);
    }

    return nRet;
}

static int32_t P_DB_MANAGER_WriteCsvV2xStatusRx(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    DB_V2X_STATUS_TX_T stDbV2xStatusTx;

    pchPayload = (char*)malloc(sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);
    if(pchPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(pchPayload, (char *)pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eDeviceType);
    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eTeleCommType);
    fprintf(sh_pDbMgrRxMsg, "0x%x,", pstEventMsg->pstDbV2x->unDeviceId);
    fprintf(sh_pDbMgrRxMsg, "%ld,", pstEventMsg->pstDbV2x->ulTimeStamp);
    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eServiceId);
    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eActionType);
    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eRegionId);
    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->ePayloadType);
    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->eCommId);
    fprintf(sh_pDbMgrRxMsg, "%d.%d,", pstEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
    fprintf(sh_pDbMgrRxMsg, "0x%x,", pstEventMsg->pstDbV2x->usHwVer);
    fprintf(sh_pDbMgrRxMsg, "0x%x,", pstEventMsg->pstDbV2x->usSwVer);
    fprintf(sh_pDbMgrRxMsg, "%d,", pstEventMsg->pstDbV2x->ulPayloadLength);

    memcpy(&stDbV2xStatusTx, pchPayload, sizeof(char)*pstEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pDbMgrRxMsg, "%ld,", stDbV2xStatusTx.ulTxTimeStampL1);
    fprintf(sh_pDbMgrRxMsg, "%ld,", stDbV2xStatusTx.ulTxTimeStampL2);
    fprintf(sh_pDbMgrRxMsg, "%ld,", stDbV2xStatusTx.ulTxTimeStampL3);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.unRxDeviceId);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.eChannel);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.sPower);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.eBandwidth);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.eChannel);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.usTxRatio);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.stTxPosition.nTxLatitude);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.stTxPosition.nTxLongitude);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.stTxPosition.nTxAttitude);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.unSeqNum);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.unContCnt);
    fprintf(sh_pDbMgrRxMsg, "%d,", stDbV2xStatusTx.unTxVehicleSpeed);

    fprintf(sh_pDbMgrRxMsg, "0x%x", pstEventMsg->pstDbManagerWrite->unCrc32);
    fprintf(sh_pDbMgrRxMsg, "\r\n");

    nRet = fflush(sh_pDbMgrRxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if(pchPayload != NULL)
    {
        free(pchPayload);
    }

    if(pstEventMsg->pPayload != NULL)
    {
        if(s_bDbMgrLog == ON)
        {
            PrintDebug("free [%p] allocated at P_MSG_MANAGER_SendRxMsgToDbMgr()", pstEventMsg->pPayload);
        }
        free(pstEventMsg->pPayload);
    }

    return nRet;
}

static int32_t P_DB_MANAGER_WriteCsvV2xStatus(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    switch(pstEventMsg->pstDbManagerWrite->eCommMsgType)
    {
        case DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pDbMgrTxMsg != NULL)
            {
                nRet = P_DB_MANAGER_WriteCsvV2xStatusTx(pstEventMsg);
                if(nRet != FRAMEWORK_OK)
                {
                    PrintError("P_DB_MANAGER_WriteCsvV2xStatusTx() is failed! [unRet:%d]", nRet);
                }
            }
            else
            {
                PrintError("sh_pDbMgrTxMsg is NULL!!, check whethter sh_pDbMgrTxMsg is opened before.");
            }
            break;
        }
        case DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pDbMgrRxMsg != NULL)
            {
                nRet = P_DB_MANAGER_WriteCsvV2xStatusRx(pstEventMsg);
                if(nRet != FRAMEWORK_OK)
                {
                    PrintError("P_DB_MANAGER_WriteCsvV2xStatusRx() is failed! [unRet:%d]", nRet);
                }
            }
            else
            {
                PrintError("sh_pDbMgrRxMsg is NULL!!, check whethter sh_pDbMgrRxMsg is opened before.");
            }
            break;
        }
        default:
            PrintError("unknown eCommMsgType [%d]", pstEventMsg->pstDbManagerWrite->eCommMsgType);
            break;
    }
    nRet = FRAMEWORK_OK;

    return nRet;
}

static int32_t P_DB_MANAGER_WriteCsv(DB_MANAGER_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    switch(pstEventMsg->pstDbV2x->ePayloadType)
    {
        case DB_V2X_PAYLOAD_TYPE_V2X_STATUS:
        {
            nRet = P_DB_MANAGER_WriteCsvV2xStatus(pstEventMsg);
            if(nRet != FRAMEWORK_OK)
            {
                PrintError("P_DB_MANAGER_WriteCsvV2xStatus() is failed! [unRet:%d]", nRet);
            }
            break;
        }
        case DB_V2X_PAYLOAD_TYPE_PLATOONING:
        {
            nRet = P_DB_MANAGER_WriteCsvPlatooning(pstEventMsg);
            if(nRet != FRAMEWORK_OK)
            {
                PrintError("P_DB_MANAGER_WriteCsvPlatooning() is failed! [unRet:%d]", nRet);
            }
            break;
        }
        case DB_V2X_PAYLOAD_TYPE_PLATOONING_THROUGHPUT:
        {
            nRet = P_DB_MANAGER_WriteCsvPlatooningThroughput(pstEventMsg);
            if(nRet != FRAMEWORK_OK)
            {
                PrintError("P_DB_MANAGER_WriteCsvPlatooningThroughput() is failed! [unRet:%d]", nRet);
            }
            break;
        }
        default:
            PrintError("unknown ePayloadType [%d]", pstEventMsg->pstDbManagerWrite->eCommMsgType);
            break;
    }

    return nRet;
}

static void *P_DB_MANAGER_Task(void *arg)
{
    DB_MANAGER_EVENT_MSG_T stEventMsg;
    int32_t nRet = FRAMEWORK_ERROR;
    memset(&stEventMsg, 0, sizeof(DB_MANAGER_EVENT_MSG_T));

    (void)arg;

    while (1)
    {
        if(msgrcv(s_nDbTaskMsgId, &stEventMsg, sizeof(DB_MANAGER_EVENT_MSG_T), 0, MSG_NOERROR) == FRAMEWORK_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            if(stEventMsg.pstDbManagerWrite->eProc == DB_MANAGER_PROC_WRITE)
            {
                if (s_bDbMgrLog == ON)
                {
                    PrintDebug("DB_MANAGER_PROC_WRITE [%d]", stEventMsg.pstDbManagerWrite->eProc);
                }

                switch(stEventMsg.pstDbManagerWrite->eFileType)
                {
                    case DB_MANAGER_FILE_TYPE_TXT:
                    {
                        if (s_bDbMgrLog == ON)
                        {
                            PrintDebug("DB_MANAGER_FILE_TYPE_TXT [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        }

                        nRet = P_DB_MANAGER_WriteTxt(&stEventMsg);
                        if(nRet != FRAMEWORK_OK)
                        {
                            PrintError("P_DB_MANAGER_WriteTxt() is failed! [unRet:%d]", nRet);
                        }
                        break;
                    }
                    case DB_MANAGER_FILE_TYPE_CSV:
                    {
                        if (s_bDbMgrLog == ON)
                        {
                            PrintDebug("DB_MANAGER_FILE_TYPE_CSV [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        }

                        nRet = P_DB_MANAGER_WriteCsv(&stEventMsg);
                        if(nRet != FRAMEWORK_OK)
                        {
                            PrintError("P_DB_MANAGER_WriteCsv() is failed! [unRet:%d]", nRet);
                        }
                        break;
                    }
#if defined(CONFIG_SQLITE)
                    case DB_MANAGER_FILE_TYPE_SQLITE:
                    {
                        if (s_bDbMgrLog == ON)
                        {
                            PrintDebug("DB_MANAGER_FILE_TYPE_SQLITE [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        }

                        nRet = P_DB_MANAGER_WriteSqlite(&stEventMsg);
                        if(nRet != FRAMEWORK_OK)
                        {
                            PrintError("P_DB_MANAGER_WriteSqlite() is failed! [unRet:%d]", nRet);
                        }
                        break;
                    }
#endif
                    default:
                        PrintWarn("unknown file type [%d]", stEventMsg.pstDbManagerWrite->eFileType);
                        break;
                }
            }
            else if(stEventMsg.pstDbManagerWrite->eProc == DB_MANAGER_PROC_READ)
            {
                PrintDebug("DB_MANAGER_PROC_READ [%d]", stEventMsg.pstDbManagerWrite->eProc);
                PrintWarn("TODO");

            }
            else if(stEventMsg.pstDbManagerWrite->eProc == DB_MANAGER_PROC_CONVERT)
            {
                PrintDebug("DB_MANAGER_PROC_CONVERT [%d]", stEventMsg.pstDbManagerWrite->eProc);
                PrintWarn("TODO");
            }
            else
            {
                PrintWarn("unknown processing type [%d]", stEventMsg.pstDbManagerWrite->eProc);
            }
        }
    }

    return NULL;
}

static void P_DB_MANAGER_PrintMsgInfo(int msqid)
{

    struct msqid_ds m_stat;

    PrintDebug("========== Messege Queue Infomation =============");

    if(msgctl(msqid, IPC_STAT, &m_stat) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgctl() is failed!!");
    }

    PrintDebug("msg_lspid : %d", m_stat.msg_lspid);
    PrintDebug("msg_qnum : %ld", m_stat.msg_qnum);
    PrintDebug("msg_stime : %ld", m_stat.msg_stime);

    PrintDebug("=================================================");
}

int32_t P_DB_MANAGER_CreateTask(void)
{
	int32_t nRet = FRAMEWORK_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_DbMgrTask, &attr, P_DB_MANAGER_Task, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_DB_MANAGER_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DB_MANAGER_Task() is successfully created.");
        nRet = FRAMEWORK_OK;
    }

#if defined(CONFIG_PTHREAD_JOINABLE)
    nRet = pthread_join(sh_DbMgrTask, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_DB_MANAGER_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintDebug("P_DB_MANAGER_Task() is successfully joined.");
        nRet = FRAMEWORK_OK;
    }
#endif
	return nRet;
}

static int32_t P_DB_MANAGER_Init(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    if((s_nDbTaskMsgId = msgget(s_dbTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DB_MANAGER_PrintMsgInfo(s_nDbTaskMsgId);
    }

    if((s_nMsgTxTaskMsgId = msgget(s_MsgTxTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DB_MANAGER_PrintMsgInfo(s_nMsgTxTaskMsgId);
    }

    if((s_nMsgRxTaskMsgId = msgget(s_MsgRxTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_DB_MANAGER_PrintMsgInfo(s_nMsgRxTaskMsgId);
    }

    nRet = P_DB_MANAGER_CreateTask();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_DB_MANAGER_CreateTask() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}
static int32_t P_DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Write(DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void *pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;
    DB_MANAGER_EVENT_MSG_T stEventMsg;

    if(pstDbManagerWrite == NULL)
    {
        PrintError("pstDbManagerWrite == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

#if defined(CONFIG_SQLITE)
    if((pstDbManagerWrite->eFileType == DB_MANAGER_FILE_TYPE_SQLITE) && (sh_pDbMgrTxSqlMsg == NULL))
    {
        PrintError("sh_pDbMgrTxSqlMsg == NULL!!, check DB_MANAGER_Open() is called.");
        return nRet;
    }
#endif

    if((pstDbManagerWrite->eFileType == DB_MANAGER_FILE_TYPE_CSV) && (sh_pDbMgrTxMsg == NULL))
    {
        PrintError("sh_pDbMgrTxMsg == NULL!!, check DB_MANAGER_Open() is called.");
        return nRet;
    }

    stEventMsg.pstDbManagerWrite = pstDbManagerWrite;
    stEventMsg.pstDbV2x = pstDbV2x;
    stEventMsg.pPayload = pPayload;

    if(msgsnd(s_nDbTaskMsgId, &stEventMsg, sizeof(DB_MANAGER_EVENT_MSG_T), IPC_NOWAIT) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgsnd() is failed!!");
        return nRet;
    }
    else
    {
        nRet = FRAMEWORK_OK;
    }

    return nRet;
}

int32_t DB_MANAGER_Read(DB_MANAGER_READ_T *pstDbManagerRead, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManagerRead == NULL)
    {
        PrintError("pstDbManagerRead == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

#if defined(CONFIG_SQLITE)
    if(sh_pDbMgrTxSqlMsg == NULL)
    {
        PrintError("sh_pDbMgrTxSqlMsg == NULL!!, check DB_MANAGER_Open() is called.");
        return nRet;
    }
#endif

    if(sh_pDbMgrTxMsg == NULL)
    {
        PrintError("sh_pDbMgrTxMsg == NULL!!, check DB_MANAGER_Open() is called.");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Converter(DB_MANAGER_READ_T *pstDbManagerRead, DB_MANAGER_WRITE_T *pstDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManagerRead == NULL)
    {
        PrintError("pstDbManagerRead == NULL!!");
        return nRet;
    }

    if(pstDbManagerWrite == NULL)
    {
        PrintError("pstDbManagerWrite == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_SetLog(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    s_bDbMgrLog = pstDbManager->bLogLevel;
    PrintTrace("SET:s_bDbMgrLog [%s]", s_bDbMgrLog == ON ? "ON" : "OFF");

    nRet = FRAMEWORK_OK;

    return nRet;
}

int32_t DB_MANAGER_Open(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    switch(pstDbManager->eFileType)
    {
        case DB_MANAGER_FILE_TYPE_TXT:
            PrintDebug("DB_MANAGER_FILE_TYPE_TXT [%d]", pstDbManager->eFileType);

            if(sh_pDbMgrTxMsg == NULL)
            {
                sh_pDbMgrTxMsg = fopen(DB_MANAGER_TXT_TX_FILE, "a+");
                if(sh_pDbMgrTxMsg == NULL)
                {
                    PrintError("fopen() is failed!!");
                }
                else
                {
                    PrintTrace("DB_MANAGER_TX_FILE[%s] is opened.", DB_MANAGER_TXT_TX_FILE);
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pDbMgrRxMsg == NULL)
            {
                sh_pDbMgrRxMsg = fopen(DB_MANAGER_TXT_RX_FILE, "a+");
                if(sh_pDbMgrRxMsg == NULL)
                {
                    PrintError("fopen() is failed!!");
                }
                else
                {
                    PrintTrace("DB_MANAGER_RX_FILE[%s] is opened.", DB_MANAGER_TXT_RX_FILE);
                    nRet = FRAMEWORK_OK;
                }
            }

            break;

        case DB_MANAGER_FILE_TYPE_CSV:
            PrintDebug("DB_MANAGER_FILE_TYPE_CSV [%d]", pstDbManager->eFileType);

            nRet = P_DB_MANAGER_OpenCsv(pstDbManager);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("P_DB_MANAGER_OpenCsv() is failed! [nRet:%d]", nRet);
                return nRet;
            }
            break;

#if defined(CONFIG_SQLITE)
        case DB_MANAGER_FILE_TYPE_SQLITE:
            PrintDebug("DB_MANAGER_FILE_TYPE_SQLITE [%d]", pstDbManager->eFileType);

            nRet = P_DB_MANAGER_OpenSqlite(pstDbManager);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("P_DB_MANAGER_OpenSqlite() is failed! [nRet:%d]", nRet);
                return nRet;
            }
            break;
#endif

        default:
            PrintWarn("unknown file type [%d]", pstDbManager->eFileType);
            break;

    }

    return nRet;
}

int32_t DB_MANAGER_Close(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    switch(pstDbManager->eFileType)
    {
        case DB_MANAGER_FILE_TYPE_TXT:
            PrintDebug("DB_MANAGER_FILE_TYPE_TXT [%d]", pstDbManager->eFileType);

            if(sh_pDbMgrTxMsg != NULL)
            {
                nRet = fflush(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("DB_MANAGER_TX_FILE[%s] is closed.", DB_MANAGER_TXT_TX_FILE);
                    sh_pDbMgrTxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pDbMgrRxMsg != NULL)
            {
                nRet = fflush(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("DB_MANAGER_RX_FILE[%s] is closed.", DB_MANAGER_TXT_RX_FILE);
                    sh_pDbMgrRxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }
            break;

        case DB_MANAGER_FILE_TYPE_CSV:
            PrintDebug("DB_MANAGER_FILE_TYPE_CSV [%d]", pstDbManager->eFileType);
            if(sh_pDbMgrTxMsg != NULL)
            {
                nRet = fflush(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("DB_MANAGER_TX_FILE[%s] is closed.", DB_MANAGER_CSV_TX_FILE);
                    sh_pDbMgrTxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pDbMgrRxMsg != NULL)
            {
                nRet = fflush(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("DB_MANAGER_RX_FILE[%s] is closed.", DB_MANAGER_CSV_RX_FILE);
                    sh_pDbMgrRxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }
            break;

#if defined(CONFIG_SQLITE)
        case DB_MANAGER_FILE_TYPE_SQLITE:
            PrintDebug("DB_MANAGER_FILE_TYPE_SQLITE [%d]", pstDbManager->eFileType);
            if(sh_pDbMgrTxMsg != NULL)
            {
                nRet = fflush(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("DB_MANAGER_TX_FILE[%s] is closed.", DB_MANAGER_SQL_TX_FILE);
                    sh_pDbMgrTxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pDbMgrRxMsg != NULL)
            {
                nRet = fflush(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("DB_MANAGER_RX_FILE[%s] is closed.", DB_MANAGER_SQL_RX_FILE);
                    sh_pDbMgrRxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }
            break;
#endif

        default:
            PrintWarn("unknown file type [%d]", pstDbManager->eFileType);
            break;

    }



    return nRet;
}

int32_t DB_MANAGER_Start(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Stop(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Status(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DB_MANAGER_Init(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    nRet = P_DB_MANAGER_Init(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_DB_MANAGER_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    s_bDbMgrLog = pstDbManager->bLogLevel;
    PrintDebug("s_bDbMgrLog [%s]", s_bDbMgrLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t DB_MANAGER_DeInit(DB_MANAGER_T *pstDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        return nRet;
    }

    nRet = P_DB_MANAGER_DeInit(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_DB_MANAGER_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

