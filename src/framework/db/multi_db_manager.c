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
* @file multi_db_manager.c
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
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sqlite3.h>
#include "db_v2x_status.h"
#include "framework.h"
#include "multi_db_manager.h"
#include "di.h"
#include "app.h"
#include "svc_mcp.h"

/***************************** Definition ************************************/

#define MULTI_DB_MANAGER_DB_TEMP_PATH                "/tmp/"

#define MULTI_DB_MANAGER_TXT_TX_FILE                 MULTI_DB_MANAGER_DB_TEMP_PATH"db_v2x_tx_temp_writing.txt"
#define MULTI_DB_MANAGER_TXT_RX_FILE                 MULTI_DB_MANAGER_DB_TEMP_PATH"db_v2x_rx_temp_writing.txt"

#define MULTI_DB_MANAGER_CSV_TX_FILE                 MULTI_DB_MANAGER_DB_TEMP_PATH"db_v2x_tx_temp_writing.csv"
#define MULTI_DB_MANAGER_CSV_RX_FILE                 MULTI_DB_MANAGER_DB_TEMP_PATH"db_v2x_rx_temp_writing.csv"

#if defined(CONFIG_SQLITE)
#define MULTI_DB_MANAGER_SQL_TX_FILE                 MULTI_DB_MANAGER_DB_TEMP_PATH"db_v2x_tx_temp_writing.db"
#define MULTI_DB_MANAGER_SQL_RX_FILE                 MULTI_DB_MANAGER_DB_TEMP_PATH"db_v2x_rx_temp_writing.db"
#endif

#define MULTI_MSG_MGR_PDR_PER_CONVERT_RATE           (100000)
#define MULTI_MSG_MGR_MAX_CONT_CNT                   (100)
//#define DB_MGR_TEST                                (1)

#define MULTI_SVC_MCP_GPS_SPEED_CAL_CNT_MAX          (10)
#define MULTI_DB_MGR_TIME_US_TO_MS                   (1000)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
FILE* sh_pMultiDbMgrTxMsg;
FILE* sh_pMultiDbMgrRxMsg;
#if defined(CONFIG_SQLITE)
sqlite3* sh_pMultiDbMgrTxSqlMsg;
sqlite3* sh_pMultiDbMgrRxSqlMsg;
#endif

static int s_nMultiDbTaskMsgId, s_nMultiMsgTxTaskMsgId, s_nMultiMsgRxTaskMsgId;
static key_t s_MultidbTaskMsgKey = FRAMEWORK_DB_TASK_MSG_KEY;
static key_t s_MultiMsgTxTaskMsgKey = FRAMEWORK_MSG_TX_TASK_MSG_KEY;
static key_t s_MultiMsgRxTaskMsgKey = FRAMEWORK_MSG_RX_TASK_MSG_KEY;

static pthread_t sh_MultiDbMgrTask;

static bool s_bMultiDbMgrLog = OFF;

static MULTI_DB_MANAGER_V2X_STATUS_T s_stMultiDbV2xStatusRx;

static int32_t P_MULTI_DB_MANAGER_SetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus);
static int32_t P_MULTI_DB_MANAGER_GetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus);

static uint16_t s_usMultiGpsSpeedCalCnt = 0;
static uint32_t s_usMultiLastSpeedTx;
static uint32_t s_usMultiLastSpeedRx;

/***************************** Function  *************************************/

static int32_t P_MULTI_DB_MANAGER_PrintStatus(DB_V2X_STATUS_TX_T *pstDbV2xStatusTx, DB_V2X_STATUS_RX_T *pstDbV2xStatusRx)
{
    int32_t nRet = FRAMEWORK_ERROR;
    MULTI_DB_MANAGER_V2X_STATUS_T stMultiDbV2xStatus;

    if(pstDbV2xStatusTx == NULL)
    {
        PrintError("pstDbV2xStatusTx is NULL!");
        return nRet;
    }

    if(pstDbV2xStatusRx == NULL)
    {
        PrintError("pstDbV2xStatusRx is NULL!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }
    else
    {
        nRet = FRAMEWORK_OK;
        if(stMultiDbV2xStatus.unCurrentContCnt == MULTI_MSG_MGR_MAX_CONT_CNT)
        {
            PrintDebug("(Tx)unSeqNum[%d], (Rx)ulTotalPacketCnt[%ld], unPdr[%.3f], unPer[%.3f], ulTotalErrCnt[%ld]", pstDbV2xStatusTx->unSeqNum, pstDbV2xStatusRx->ulTotalPacketCnt, (float)(pstDbV2xStatusRx->unPdr/1000.0f), (float)(pstDbV2xStatusRx->unPer/1000.0f), pstDbV2xStatusRx->ulTotalErrCnt);
            PrintDebug("(Tx)unTxVehicleSpeed[%d], unTxVehicleHeading[%d], (Rx)unRxVehicleSpeed[%d], unRxVehicleHeading[%d]", pstDbV2xStatusTx->unTxVehicleSpeed, pstDbV2xStatusTx->unTxVehicleHeading, pstDbV2xStatusRx->unRxVehicleSpeed, pstDbV2xStatusRx->unRxVehicleHeading);
        }
    }
    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_PrintStatusPt(DB_V2X_PLATOONING_T *pstDbV2xPt)
{
    int32_t nRet = FRAMEWORK_ERROR;
    MULTI_DB_MANAGER_V2X_STATUS_T stMultiDbV2xStatus;

    if(pstDbV2xPt == NULL)
    {
        PrintError("pstDbV2xPt is NULL!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }
    else
    {
        nRet = FRAMEWORK_OK;
        if(stMultiDbV2xStatus.unCurrentContCnt == MULTI_MSG_MGR_MAX_CONT_CNT)
        {
            PrintDebug("eDbV2XPtType[%d], usV2xGroupId[%d]", pstDbV2xPt->eDbV2XPtType, pstDbV2xPt->usV2xGroupId);
        }
    }
    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_PrintStatusPtLv(DB_V2X_PLATOONING_LV_T *pstDbV2XPtLv)
{
    int32_t nRet = FRAMEWORK_ERROR;
    MULTI_DB_MANAGER_V2X_STATUS_T stMultiDbV2xStatus;

    if(pstDbV2XPtLv == NULL)
    {
        PrintError("pstDbV2XPtLv is NULL!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }
    else
    {
        nRet = FRAMEWORK_OK;
        if(stMultiDbV2xStatus.unCurrentContCnt == MULTI_MSG_MGR_MAX_CONT_CNT)
        {
            PrintDebug("eLvServiceId[%d], eLvMethodId[%d], unLvLength[%d], usLvClientId[%d], usLvSessionId[%d], ucLvProtocolVer[%d]", pstDbV2XPtLv->eLvServiceId, pstDbV2XPtLv->eLvMethodId, pstDbV2XPtLv->unLvLength, pstDbV2XPtLv->usLvClientId, pstDbV2XPtLv->usLvSessionId, pstDbV2XPtLv->ucLvProtocolVer);
            PrintDebug("ucLvInterfaceVer[%d], eLvMsgType[%d], ucLvReturnCode[%d], eLvVehicleType[%d]", pstDbV2XPtLv->ucLvInterfaceVer, pstDbV2XPtLv->eLvMsgType, pstDbV2XPtLv->ucLvReturnCode, pstDbV2XPtLv->eLvVehicleType);
            PrintDebug("szLvVehicleId[%s], szLvVehicleNum[%s], usLvMsgCount[%d], eLvMsgId[%d]", pstDbV2XPtLv->szLvVehicleId, pstDbV2XPtLv->szLvVehicleNum, pstDbV2XPtLv->usLvMsgCount, pstDbV2XPtLv->eLvMsgId);
            PrintDebug("nLvLatitude[%d], nLvLongitude[%d], usLvHeading[%d], usLvSpeed[%d]", pstDbV2XPtLv->nLvLatitude, pstDbV2XPtLv->nLvLongitude, pstDbV2XPtLv->usLvHeading, pstDbV2XPtLv->usLvSpeed);
            PrintDebug("szLvDriveLaneId[%s], eLvDriveStatus[%d], eLvChangeCode[%d]", pstDbV2XPtLv->szLvDriveLaneId, pstDbV2XPtLv->eLvDriveStatus, pstDbV2XPtLv->eLvChangeCode);
            PrintDebug("usLvPathId[%d], szLvLaneId[%s], eLvLanePlan[%d]", pstDbV2XPtLv->usLvPathId, pstDbV2XPtLv->szLvLaneId, pstDbV2XPtLv->eLvLanePlan);
            PrintDebug("eLvCrossway[%d], eLvLaneManeuver[%d]", pstDbV2XPtLv->eLvCrossway, pstDbV2XPtLv->eLvLaneManeuver);
            for (int i=0; i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN; i++)
            {
                PrintDebug("anLvLatitude[%d] = %d", i, pstDbV2XPtLv->stLvPathPlan.anLvLatitude[i]);
            }
            for (int i=0; i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN; i++)
            {
                PrintDebug("anLvLongitude[%d] = %d", i, pstDbV2XPtLv->stLvPathPlan.anLvLongitude[i]);
            }
            PrintDebug("unReserved1[%ld]", pstDbV2XPtLv->unReserved1);
        }
    }
    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_PrintStatusPtFv(DB_V2X_PLATOONING_FV_T *pstDbV2XPtFv)
{
    int32_t nRet = FRAMEWORK_ERROR;
    MULTI_DB_MANAGER_V2X_STATUS_T stMultiDbV2xStatus;

    if(pstDbV2XPtFv == NULL)
    {
        PrintError("pstDbV2XPtFv is NULL!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }
    else
    {
        nRet = FRAMEWORK_OK;
        if(stMultiDbV2xStatus.unCurrentContCnt == MULTI_MSG_MGR_MAX_CONT_CNT)
        {
            PrintDebug("eFvServiceId[%d], eFvMethodId[%d], unFvLength[%d], usFvClientId[%d], usFvSessionId[%d], ucFvProtocolVer[%d]", pstDbV2XPtFv->eFvServiceId, pstDbV2XPtFv->eFvMethodId, pstDbV2XPtFv->unFvLength, pstDbV2XPtFv->usFvClientId, pstDbV2XPtFv->usFvSessionId, pstDbV2XPtFv->ucFvProtocolVer);
            PrintDebug("ucFvInterfaceVer[%d], eFvMsgType[%d], ucFvReturnCode[%d], eFvVehicleType[%d]", pstDbV2XPtFv->ucFvInterfaceVer, pstDbV2XPtFv->eFvMsgType, pstDbV2XPtFv->ucFvReturnCode, pstDbV2XPtFv->eFvVehicleType);
            PrintDebug("szFvVehicleId[%s], szFvVehicleNum[%s], usFvMsgCount[%d], eFvMsgId[%d]", pstDbV2XPtFv->szFvVehicleId, pstDbV2XPtFv->szFvVehicleNum, pstDbV2XPtFv->usFvMsgCount, pstDbV2XPtFv->eFvMsgId);
            PrintDebug("nFvLatitude[%d], nFvLongitude[%d], usFvHeading[%d], usFvSpeed[%d]", pstDbV2XPtFv->nFvLatitude, pstDbV2XPtFv->nFvLongitude, pstDbV2XPtFv->usFvHeading, pstDbV2XPtFv->usFvSpeed);
            PrintDebug("szFvDriveLaneId[%s], eFvDriveStatus[%d], eFvChangeCode[%d]", pstDbV2XPtFv->szFvDriveLaneId, pstDbV2XPtFv->eFvDriveStatus, pstDbV2XPtFv->eFvChangeCode);
            for (int i=0; i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN; i++)
            {
                PrintDebug("anFvLatitude[%d] = %d", i, pstDbV2XPtFv->stFvPathPlan.anFvLatitude[i]);
            }
            for (int i=0; i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN; i++)
            {
                PrintDebug("anFvLongitude[%d] = %d", i, pstDbV2XPtFv->stFvPathPlan.anFvLongitude[i]);
            }
            PrintDebug("usFvRecommDistance[%d], usFvRecommSpeed[%d]", pstDbV2XPtFv->usFvRecommDistance, pstDbV2XPtFv->usFvRecommSpeed);
            PrintDebug("unReserved1[%ld], unReserved2[%ld], unReserved3[%ld], unReserved4[%ld], unReserved5[%d], unReserved6[%d]", pstDbV2XPtFv->unReserved1, pstDbV2XPtFv->unReserved2, pstDbV2XPtFv->unReserved3, pstDbV2XPtFv->unReserved4, pstDbV2XPtFv->unReserved5, pstDbV2XPtFv->unReserved6);
        }
    }
    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_UpdateStatus(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg, DB_V2X_STATUS_TX_T *pstDbV2xStatusTx, DB_V2X_STATUS_RX_T *pstDbV2xStatusRx)
{
    int32_t nRet = FRAMEWORK_ERROR;
    uint32_t nCurrSpeedTx;
    uint32_t nCurrSpeedRx;
    TIME_MANAGER_T *pstTimeManager;
    DI_T *pstDi;
    MULTI_DB_MANAGER_V2X_STATUS_T stMultiDbV2xStatus;
    float fTemp = 0.0f;
    double dRxlat, dRxLon, dTxLat, dTxLon, dDistMeter;
    double dHeading;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstEventMsg is NULL!");
        return nRet;
    }

    if(pstDbV2xStatusTx == NULL)
    {
        PrintError("pstDbV2xStatusTx is NULL!");
        return nRet;
    }

    if(pstDbV2xStatusRx == NULL)
    {
        PrintError("pstDbV2xStatusRx is NULL!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    if(pstMultiEventMsg->pstDbV2x->ulTimeStamp != stMultiDbV2xStatus.ulTxTimeStamp)
    {
        PrintError("MSG Rx Task Timestamp[%ld], DB Checked Tx Timestamp[%ld]", pstMultiEventMsg->pstDbV2x->ulTimeStamp, stMultiDbV2xStatus.ulTxTimeStamp);
    }

    pstTimeManager = FRAMEWORK_GetTimeManagerInstance();
    if(pstTimeManager == NULL)
    {
        PrintError("pstTimeManager is NULL!");
    }

    pstDbV2xStatusTx->stDbV2xDevL1.ulTimeStamp = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL1.ulTimeStamp;
    pstDbV2xStatusTx->stDbV2xDevL2.ulTimeStamp = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL2.ulTimeStamp;

    pstDbV2xStatusRx->stDbV2xDevL1.ulTimeStamp = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL1.ulTimeStamp;
    pstDbV2xStatusRx->stDbV2xDevL2.ulTimeStamp  = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL2.ulTimeStamp;

    nRet = TIME_MANAGER_Get(pstTimeManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nRet);
        pstDbV2xStatusRx->stDbV2xDevL3.ulTimeStamp = 0;
    }
    else
    {
        pstDbV2xStatusRx->stDbV2xDevL3.ulTimeStamp = pstTimeManager->ulTimeStamp;
    }

    pstDbV2xStatusRx->stDbV2xDevL1.ulLatency = pstDbV2xStatusRx->stDbV2xDevL1.ulTimeStamp - pstDbV2xStatusTx->stDbV2xDevL1.ulTimeStamp;
    pstDbV2xStatusRx->stDbV2xDevL2.ulLatency = pstDbV2xStatusRx->stDbV2xDevL2.ulTimeStamp - pstDbV2xStatusTx->stDbV2xDevL2.ulTimeStamp;
    pstDbV2xStatusRx->stDbV2xDevL3.ulLatency = pstDbV2xStatusRx->stDbV2xDevL3.ulTimeStamp - pstDbV2xStatusTx->stDbV2xDevL3.ulTimeStamp;

    if(s_bMultiDbMgrLog == TRUE)
    {
        /* L1 e.g. 2023-09-14:13:40:10.8606 - 2023-09-14-13:00:10.7814 = 792us */
        PrintTrace("Latency of Layer #1: Rx[%ld]-Tx[%ld]=[%ld]us", pstDbV2xStatusRx->stDbV2xDevL1.ulTimeStamp, pstDbV2xStatusTx->stDbV2xDevL1.ulTimeStamp, pstDbV2xStatusRx->stDbV2xDevL1.ulLatency);
        PrintTrace("Latency of Layer #2: Rx[%ld]-Tx[%ld]=[%ld]us", pstDbV2xStatusRx->stDbV2xDevL2.ulTimeStamp, pstDbV2xStatusTx->stDbV2xDevL2.ulTimeStamp, pstDbV2xStatusRx->stDbV2xDevL2.ulLatency);
        PrintTrace("Latency of Layer #3: Rx[%ld]-Tx[%ld]=[%ld]us", pstDbV2xStatusRx->stDbV2xDevL3.ulTimeStamp, pstDbV2xStatusTx->stDbV2xDevL3.ulTimeStamp, pstDbV2xStatusRx->stDbV2xDevL3.ulLatency);
    }

    pstDbV2xStatusRx->stDbV2xDevL1.unDevId = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL1.unDevId;
    pstDbV2xStatusRx->stDbV2xDevL2.unDevId = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL2.unDevId;
    pstDbV2xStatusRx->stDbV2xDevL3.unDevId = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL3.unDevId;

    pstDbV2xStatusRx->stDbV2xDevL1.usSwVer = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL1.usSwVer;
    pstDbV2xStatusRx->stDbV2xDevL2.usSwVer = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL2.usSwVer;
    pstDbV2xStatusRx->stDbV2xDevL3.usSwVer = pstMultiEventMsg->pstDbV2x->usSwVer;

    pstDbV2xStatusRx->stDbV2xDevL1.usHwVer = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL1.usHwVer;
    pstDbV2xStatusRx->stDbV2xDevL2.usHwVer = stMultiDbV2xStatus.stV2xStatusRx.stDbV2xDevL2.usHwVer;
    pstDbV2xStatusRx->stDbV2xDevL3.usHwVer = pstMultiEventMsg->pstDbV2x->usHwVer;

    pstDbV2xStatusTx->stDbV2xDevL1.unDevId = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL1.unDevId;
    pstDbV2xStatusTx->stDbV2xDevL2.unDevId = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL2.unDevId;
    pstDbV2xStatusTx->stDbV2xDevL3.unDevId = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL3.unDevId;

    pstDbV2xStatusTx->stDbV2xDevL1.usSwVer = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL1.usSwVer;
    pstDbV2xStatusTx->stDbV2xDevL2.usSwVer = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL2.usSwVer;

    pstDbV2xStatusTx->stDbV2xDevL1.usHwVer = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL1.usHwVer;
    pstDbV2xStatusTx->stDbV2xDevL2.usHwVer = stMultiDbV2xStatus.stV2xStatusTx.stDbV2xDevL2.usHwVer;

    pstDbV2xStatusTx->ucTxPwr = stMultiDbV2xStatus.stV2xStatusTx.ucTxPwr;
    pstDbV2xStatusTx->usTxFreq = stMultiDbV2xStatus.stV2xStatusTx.usTxFreq;
    pstDbV2xStatusTx->ucTxBw = stMultiDbV2xStatus.stV2xStatusTx.ucTxBw;
    pstDbV2xStatusTx->ucScs = stMultiDbV2xStatus.stV2xStatusTx.ucScs;
    pstDbV2xStatusTx->ucMcs = stMultiDbV2xStatus.stV2xStatusTx.ucMcs;

    pstDbV2xStatusRx->unTotalCommDevCnt = MULTI_DB_MGR_DEFAULT_COMM_DEV_CNT;
    pstDbV2xStatusRx->nRssi = stMultiDbV2xStatus.stV2xStatusRx.nRssi;
    pstDbV2xStatusRx->ucRcpi = stMultiDbV2xStatus.stV2xStatusRx.ucRcpi;
    pstDbV2xStatusRx->eRsvLevel = stMultiDbV2xStatus.stV2xStatusRx.eRsvLevel;

#if defined(CONFIG_GPS_OBU) || defined(CONFIG_GPS_RSU)
    UNUSED(pstDi);

    dRxlat = (double)stMultiDbV2xStatus.stV2xGpsInfoRx.nLatitudeNow / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    dRxLon = (double)stMultiDbV2xStatus.stV2xGpsInfoRx.nLongitudeNow / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;

    dTxLat = (double)stMultiDbV2xStatus.stV2xGpsInfoTx.nLatitudeNow / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    dTxLon = (double)stMultiDbV2xStatus.stV2xGpsInfoTx.nLatitudeNow / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;

    dDistMeter = DI_GPS_CalculateDistance(dRxlat, dRxLon, dTxLat, dTxLon);

    pstDbV2xStatusTx->stTxPosition.nTxLatitude = stMultiDbV2xStatus.stV2xGpsInfoTx.nLatitudeNow;
    pstDbV2xStatusTx->stTxPosition.nTxLongitude = stMultiDbV2xStatus.stV2xGpsInfoTx.nLongitudeNow;
    pstDbV2xStatusTx->stTxPosition.nTxAttitude = 0;

    pstDbV2xStatusRx->stRxPosition.unCommDistance = (uint32_t)(dDistMeter * SVC_MCP_GPS_VALUE_CONVERT);

    pstDbV2xStatusRx->stRxPosition.nRxLatitude = stMultiDbV2xStatus.stV2xGpsInfoRx.nLatitudeNow;
    pstDbV2xStatusRx->stRxPosition.nRxLongitude = stMultiDbV2xStatus.stV2xGpsInfoRx.nLongitudeNow;
    pstDbV2xStatusRx->stRxPosition.nRxAttitude = 0;

    stMultiDbV2xStatus.stV2xGpsInfoHeadingRx.nLatitudeNow = pstDbV2xStatusRx->stRxPosition.nRxLatitude;
    stMultiDbV2xStatus.stV2xGpsInfoHeadingRx.nLongitudeNow = pstDbV2xStatusRx->stRxPosition.nRxLongitude;
    stMultiDbV2xStatus.stV2xGpsInfoHeadingRx.ulTimeStampNow = pstTimeManager->ulTimeStamp;

    dHeading = DI_GPS_CalculateHeading(&stMultiDbV2xStatus.stV2xGpsInfoHeadingTx);
    if (dHeading < 0)
    {
        PrintError("DI_GPS_CalculateHeading() is failed! [dHeading:%lf]", dHeading);
    }

    pstDbV2xStatusRx->unRxVehicleHeading = (uint32_t)dHeading;

    stMultiDbV2xStatus.stV2xGpsInfoHeadingRx.nLatitudeLast = pstDbV2xStatusRx->stRxPosition.nRxLatitude;
    stMultiDbV2xStatus.stV2xGpsInfoHeadingRx.nLongitudeLast = pstDbV2xStatusRx->stRxPosition.nRxLongitude;
    stMultiDbV2xStatus.stV2xGpsInfoHeadingRx.ulTimeStampLast = pstTimeManager->ulTimeStamp;
#else
    pstDi = APP_GetDiInstance();
    if(pstDi == NULL)
    {
        PrintError("pstDi is NULL!");
    }

    /* Set the GPS values */
    nRet = DI_GPS_Get(&pstDi->stDiGps);
    if (nRet != DI_OK)
    {
        PrintError("DI_GPS_Get() is failed! [nRet:%d]", nRet);
    }

    dRxlat = (double)pstDi->stDiGps.stDiGpsData.fLatitude;
    dRxLon = (double)pstDi->stDiGps.stDiGpsData.fLongitude;

    dTxLat = (double)pstDbV2xStatusTx->stTxPosition.nTxLatitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    dTxLon = (double)pstDbV2xStatusTx->stTxPosition.nTxLongitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;

    dDistMeter = DI_GPS_CalculateDistance(dRxlat, dRxLon, dTxLat, dTxLon);
    pstDbV2xStatusRx->stRxPosition.unCommDistance = (uint32_t)(dDistMeter * SVC_MCP_GPS_VALUE_CONVERT);

    pstDbV2xStatusRx->stRxPosition.nRxLatitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fLatitude * SVC_MCP_GPS_VALUE_CONVERT);
    pstDbV2xStatusRx->stRxPosition.nRxLongitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fLongitude * SVC_MCP_GPS_VALUE_CONVERT);
    pstDbV2xStatusRx->stRxPosition.nRxAttitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fAltitude * SVC_MCP_GPS_VALUE_CONVERT);

    dHeading = DI_GPS_GetHeading(&pstDi->stDiGps);
    if (nRet != DI_OK)
    {
        PrintError("DI_GPS_GetHeading() is failed! [nRet:%d]", nRet);
    }

    pstDbV2xStatusRx->unRxVehicleHeading = (uint32_t)dHeading;
#endif

    if(stMultiDbV2xStatus.bFirstPacket == TRUE)
    {
        pstDbV2xStatusTx->unTxVehicleSpeed = MULTI_DB_MGR_DEFAULT_VEHICLE_SPEED;
        PrintWarn("stMultiDbV2xStatus.bFirstPacket's speed is the default value [%d]", pstDbV2xStatusTx->unTxVehicleSpeed);
        pstDbV2xStatusRx->unRxVehicleSpeed = MULTI_DB_MGR_DEFAULT_VEHICLE_SPEED;
        PrintWarn("stMultiDbV2xStatus.bFirstPacket's speed is the default value [%d]", pstDbV2xStatusRx->unRxVehicleSpeed);
    }
    else
    {
        if(s_usMultiGpsSpeedCalCnt == MULTI_SVC_MCP_GPS_SPEED_CAL_CNT_MAX)
        {
            stMultiDbV2xStatus.stV2xGpsInfoTx.nLatitudeNow = pstDbV2xStatusTx->stTxPosition.nTxLatitude;
            stMultiDbV2xStatus.stV2xGpsInfoTx.nLongitudeNow = pstDbV2xStatusTx->stTxPosition.nTxLongitude;
            stMultiDbV2xStatus.stV2xGpsInfoTx.ulTimeStampNow = pstTimeManager->ulTimeStamp;
            stMultiDbV2xStatus.stV2xGpsInfoRx.nLatitudeNow = pstDbV2xStatusRx->stRxPosition.nRxLatitude;
            stMultiDbV2xStatus.stV2xGpsInfoRx.nLongitudeNow = pstDbV2xStatusRx->stRxPosition.nRxLongitude;
            stMultiDbV2xStatus.stV2xGpsInfoRx.ulTimeStampNow = pstTimeManager->ulTimeStamp;

            nCurrSpeedTx = DI_GPS_CalculateSpeed(&stMultiDbV2xStatus.stV2xGpsInfoTx);
            nCurrSpeedRx = DI_GPS_CalculateSpeed(&stMultiDbV2xStatus.stV2xGpsInfoRx);
            if(nCurrSpeedTx == 0 && nCurrSpeedRx == 0)
            {
                stMultiDbV2xStatus.stV2xStatusTx.unTxVehicleSpeed = s_usMultiLastSpeedTx;
                stMultiDbV2xStatus.stV2xStatusRx.unRxVehicleSpeed = s_usMultiLastSpeedRx;
            }
            else if(nCurrSpeedTx == 0)
            {
                stMultiDbV2xStatus.stV2xStatusTx.unTxVehicleSpeed = s_usMultiLastSpeedTx;
                stMultiDbV2xStatus.stV2xStatusRx.unRxVehicleSpeed = nCurrSpeedRx;
            }
            else if(nCurrSpeedRx == 0)
            {
                stMultiDbV2xStatus.stV2xStatusTx.unTxVehicleSpeed = nCurrSpeedTx;
                stMultiDbV2xStatus.stV2xStatusRx.unRxVehicleSpeed = s_usMultiLastSpeedRx;
            }
            else
            {
                stMultiDbV2xStatus.stV2xStatusTx.unTxVehicleSpeed = nCurrSpeedTx;
                stMultiDbV2xStatus.stV2xStatusRx.unRxVehicleSpeed = nCurrSpeedRx;
            }

            s_usMultiLastSpeedTx = nCurrSpeedTx;
            s_usMultiLastSpeedRx = nCurrSpeedRx;

            s_usMultiGpsSpeedCalCnt = 0;
        }
    }

    stMultiDbV2xStatus.unCurrentContCnt = pstDbV2xStatusTx->unContCnt;

    if(stMultiDbV2xStatus.bFirstPacket == TRUE)
    {
        stMultiDbV2xStatus.unLastContCnt = pstDbV2xStatusTx->unContCnt;
        stMultiDbV2xStatus.bFirstPacket = FALSE;
        stMultiDbV2xStatus.stV2xStatusRx.ulTotalPacketCnt = pstDbV2xStatusTx->unSeqNum;
        PrintDebug("update unLastContCnt as the first packet's unCountCnt [%d], stMultiDbV2xStatus.bFirstPacket[%d]", stMultiDbV2xStatus.unLastContCnt, stMultiDbV2xStatus.bFirstPacket);
        PrintDebug("update ulTotalPacketCnt[%ld] as the unSeqNum[%d]", pstDbV2xStatusRx->ulTotalPacketCnt, pstDbV2xStatusTx->unSeqNum);
    }

    if(stMultiDbV2xStatus.unLastContCnt != stMultiDbV2xStatus.unCurrentContCnt)
    {
        PrintTrace("ContCnt does not be matched! [+1 increased unLastContCnt:%d], [unCurrentContCnt:%d]", stMultiDbV2xStatus.unLastContCnt, stMultiDbV2xStatus.unCurrentContCnt);
        stMultiDbV2xStatus.unContCntLoss++;
        stMultiDbV2xStatus.stV2xStatusRx.ulTotalErrCnt++;
        stMultiDbV2xStatus.stV2xStatusRx.ucErrIndicator = TRUE;
        PrintWarn("Increased unContCntLoss[%d], ulTotalErrCnt[%ld], set ucErrIndicator[%d]", stMultiDbV2xStatus.unContCntLoss, stMultiDbV2xStatus.stV2xStatusRx.ulTotalErrCnt, stMultiDbV2xStatus.stV2xStatusRx.ucErrIndicator);
    }

    if(s_bMultiDbMgrLog  == TRUE)
    {
        PrintDebug("[+1 increased unLastContCnt:%d] == [unCurrentContCnt:%d]", stMultiDbV2xStatus.unLastContCnt, stMultiDbV2xStatus.unCurrentContCnt);
    }

    pstDbV2xStatusRx->ucErrIndicator = stMultiDbV2xStatus.stV2xStatusRx.ucErrIndicator;
    pstDbV2xStatusRx->ulTotalPacketCnt = stMultiDbV2xStatus.stV2xStatusRx.ulTotalPacketCnt;

    stMultiDbV2xStatus.unLastContCnt = stMultiDbV2xStatus.unCurrentContCnt;
    stMultiDbV2xStatus.unLastContCnt++;
    if(stMultiDbV2xStatus.unLastContCnt > DB_V2X_STATUS_CONT_CNT_MAX)
    {
        stMultiDbV2xStatus.unLastContCnt = 1;
        if(s_bMultiDbMgrLog  == TRUE)
        {
            PrintWarn("Reset unLastContCnt as [%d]", stMultiDbV2xStatus.unLastContCnt);
        }
    }

    pstDbV2xStatusRx->ulTotalErrCnt = stMultiDbV2xStatus.stV2xStatusRx.ulTotalErrCnt;
    fTemp = (float)pstDbV2xStatusRx->ulTotalPacketCnt/(float)pstDbV2xStatusTx->unSeqNum;
    pstDbV2xStatusRx->unPdr = (uint32_t)(fTemp*MULTI_MSG_MGR_PDR_PER_CONVERT_RATE);
    pstDbV2xStatusRx->unPer = MULTI_MSG_MGR_PDR_PER_CONVERT_RATE - pstDbV2xStatusRx->unPdr;

    if(s_usMultiGpsSpeedCalCnt == 0)
    {
        stMultiDbV2xStatus.stV2xGpsInfoTx.nLatitudeLast = pstDbV2xStatusTx->stTxPosition.nTxLatitude;
        stMultiDbV2xStatus.stV2xGpsInfoTx.nLongitudeLast = pstDbV2xStatusTx->stTxPosition.nTxLongitude;
        stMultiDbV2xStatus.stV2xGpsInfoTx.ulTimeStampLast = pstTimeManager->ulTimeStamp;
        stMultiDbV2xStatus.stV2xGpsInfoRx.nLatitudeLast = pstDbV2xStatusRx->stRxPosition.nRxLatitude;
        stMultiDbV2xStatus.stV2xGpsInfoRx.nLongitudeLast = pstDbV2xStatusRx->stRxPosition.nRxLongitude;
        stMultiDbV2xStatus.stV2xGpsInfoRx.ulTimeStampLast = pstTimeManager->ulTimeStamp;
    }

    s_usMultiGpsSpeedCalCnt++;

    nRet = P_MULTI_DB_MANAGER_SetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

#if defined(CONFIG_SQLITE)
static int32_t P_MULTI_DB_MANAGER_WriteSqlite(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    char *ErrorMsg = NULL;
    int sql_TxStatus;
    int sql_RxStatus;
    const char* TxCreate = NULL;
    const char* RxCreate = NULL;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    char *InsertTxData = (char *)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
    if(InsertTxData == NULL)
        {
            PrintError("InsertTxData_malloc() is failed! [NULL]");
            return nRet;
        }
    char *InsertRxData = (char *)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
    if(InsertRxData == NULL)
        {
            PrintError("InsertRxData_malloc() is failed! [NULL]");
            return nRet;
        }

    switch (pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType)
    {
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pMultiDbMgrTxSqlMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                // Connect Database
                sql_TxStatus = sqlite3_open(MULTI_DB_MANAGER_SQL_TX_FILE, &sh_pMultiDbMgrTxSqlMsg);
                if (sql_TxStatus != SQLITE_OK)
                {
                    PrintError("Can't open Tx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrTxSqlMsg));
                    sql_TxStatus = sqlite3_close(sh_pMultiDbMgrTxSqlMsg);
                    if(sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrTxSqlMsg));
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
                    sql_TxStatus = sqlite3_exec(sh_pMultiDbMgrTxSqlMsg, TxCreate, 0, 0, &ErrorMsg);
                    if (sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't create Txtable : %s\n", ErrorMsg);
                        (void)sqlite3_free(ErrorMsg);
                        sql_TxStatus = sqlite3_close(sh_pMultiDbMgrTxSqlMsg);
                        if(sql_TxStatus != SQLITE_OK)
                        {
                            PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrTxSqlMsg));
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
                    VALUES (%d, %d, %d, %ld, %d, %d, %d, %d, %d, %d.%d, 0x%x, 0x%x, %d, 0x%x)",\
                    pstMultiEventMsg->pstDbV2x->eDeviceType,\
                    pstMultiEventMsg->pstDbV2x->eTeleCommType,\
                    pstMultiEventMsg->pstDbV2x->unDeviceId,\
                    pstMultiEventMsg->pstDbV2x->ulTimeStamp,\
                    pstMultiEventMsg->pstDbV2x->eServiceId,\
                    pstMultiEventMsg->pstDbV2x->eActionType,\
                    pstMultiEventMsg->pstDbV2x->eRegionId,\
                    pstMultiEventMsg->pstDbV2x->ePayloadType,\
                    pstMultiEventMsg->pstDbV2x->eCommId,\
                    pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK,\
                    pstMultiEventMsg->pstDbV2x->usHwVer,\
                    pstMultiEventMsg->pstDbV2x->usSwVer,\
                    pstMultiEventMsg->pstDbV2x->ulPayloadLength,\
                    pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
                }
                sql_TxStatus = sqlite3_exec(sh_pMultiDbMgrTxSqlMsg, InsertTxData, 0, 0, &ErrorMsg);
                if (sql_TxStatus != SQLITE_OK)
                {
                    PrintError("Can't insert Txtable : %s\n", ErrorMsg);
                    (void)sqlite3_free(ErrorMsg);
                    sql_TxStatus = sqlite3_close(sh_pMultiDbMgrTxSqlMsg);
                    if(sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrTxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                }
                // Close Database Connection
                sql_TxStatus = sqlite3_close(sh_pMultiDbMgrTxSqlMsg);
                if(sql_TxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrTxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                nRet = FRAMEWORK_OK;
                return nRet;
            }
            else
            {
                PrintError("sh_pMultiDbMgrTxSqlMsg is NULL!!, check whethter sh_pMultiDbMgrTxSqlMsg is opened before.");
            }

            break;
        }
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pMultiDbMgrRxSqlMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                // Connect Database
                sql_RxStatus = sqlite3_open(MULTI_DB_MANAGER_SQL_RX_FILE, &sh_pMultiDbMgrRxSqlMsg);
                if (sql_RxStatus != SQLITE_OK)
                {
                    PrintError("Can't open Rx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrRxSqlMsg));
                    sql_RxStatus = sqlite3_close(sh_pMultiDbMgrRxSqlMsg);
                    if(sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrRxSqlMsg));
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
                    sql_RxStatus = sqlite3_exec(sh_pMultiDbMgrRxSqlMsg, RxCreate, 0, 0, &ErrorMsg);
                    if (sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't create Rxtable : %s\n", ErrorMsg);
                        (void)sqlite3_free(ErrorMsg);
                        sql_RxStatus = sqlite3_close(sh_pMultiDbMgrRxSqlMsg);
                        if(sql_RxStatus != SQLITE_OK)
                        {
                            PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrRxSqlMsg));
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
                    VALUES (%d, %d, %d, %ld, %d, %d, %d, %d, %d, %d.%d, 0x%x, 0x%x, %d, 0x%x)",\
                    pstMultiEventMsg->pstDbV2x->eDeviceType,\
                    pstMultiEventMsg->pstDbV2x->eTeleCommType,\
                    pstMultiEventMsg->pstDbV2x->unDeviceId,\
                    pstMultiEventMsg->pstDbV2x->ulTimeStamp,\
                    pstMultiEventMsg->pstDbV2x->eServiceId,\
                    pstMultiEventMsg->pstDbV2x->eActionType,\
                    pstMultiEventMsg->pstDbV2x->eRegionId,\
                    pstMultiEventMsg->pstDbV2x->ePayloadType,\
                    pstMultiEventMsg->pstDbV2x->eCommId,\
                    pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK,\
                    pstMultiEventMsg->pstDbV2x->usHwVer,\
                    pstMultiEventMsg->pstDbV2x->usSwVer,\
                    pstMultiEventMsg->pstDbV2x->ulPayloadLength,\
                    pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
                }
                sql_RxStatus = sqlite3_exec(sh_pMultiDbMgrRxSqlMsg, InsertRxData, 0, 0, &ErrorMsg);
                if (sql_RxStatus != SQLITE_OK)
                {
                    PrintError("Can't insert Rxtable : %s\n", ErrorMsg);
                    (void)sqlite3_free(ErrorMsg);
                    sql_RxStatus = sqlite3_close(sh_pMultiDbMgrRxSqlMsg);
                    if(sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrRxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                }
                // Close Database Connection
                sql_RxStatus = sqlite3_close(sh_pMultiDbMgrRxSqlMsg);
                if(sql_RxStatus != SQLITE_OK)
                    {
                        PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrRxSqlMsg));
                        nRet = FRAMEWORK_ERROR;
                        return nRet;
                    }
                nRet = FRAMEWORK_OK;
                return nRet;
            }
            else
            {
                PrintError("sh_pMultiDbMgrRxSqlMsg is NULL!!, check whethter sh_pMultiDbMgrRxSqlMsg is opened before.");
            }

            break;
        }
        default:
            PrintError("unknown eMultiCommMsgType [%d]", pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType);
            break;
    }

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_OpenSqlite(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    int sql_TxStatus;
    int sql_RxStatus;

    UNUSED(pstMultiDbManager);

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager is NULL!");
        return nRet;
    }

    if(sh_pMultiDbMgrTxSqlMsg == NULL)
    {
        sql_TxStatus = sqlite3_open(MULTI_DB_MANAGER_SQL_TX_FILE, &sh_pMultiDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't open Tx database!!");
        }
        else
        {
            PrintTrace("MULTI_DB_MANAGER_SQL_TX_FILE[%s] is opened.", MULTI_DB_MANAGER_SQL_TX_FILE);
            nRet = FRAMEWORK_OK;
        }
        sql_TxStatus = sqlite3_close(sh_pMultiDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrTxSqlMsg));
        }
    }

    if(sh_pMultiDbMgrRxSqlMsg == NULL)
    {
        sql_RxStatus = sqlite3_open(MULTI_DB_MANAGER_SQL_RX_FILE, &sh_pMultiDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't open Rx database!!");
        }
        else
        {
            PrintTrace("MULTI_DB_MANAGER_SQL_RX_FILE[%s] is opened.", MULTI_DB_MANAGER_SQL_RX_FILE);
            nRet = FRAMEWORK_OK;
        }
        sql_RxStatus = sqlite3_close(sh_pMultiDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrRxSqlMsg));
        }
    }

    if(sh_pMultiDbMgrTxSqlMsg != NULL)
    {
        sql_TxStatus = sqlite3_open(MULTI_DB_MANAGER_SQL_TX_FILE, &sh_pMultiDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't open Tx database!!");
        }
        else
        {
            nRet = FRAMEWORK_OK;
        }
        sql_TxStatus = sqlite3_close(sh_pMultiDbMgrTxSqlMsg);
        if(sql_TxStatus != SQLITE_OK)
        {
            PrintError("Can't close Tx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrTxSqlMsg));
        }
    }

    if(sh_pMultiDbMgrRxSqlMsg != NULL)
    {
        sql_RxStatus = sqlite3_open(MULTI_DB_MANAGER_SQL_RX_FILE, &sh_pMultiDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't open Rx database!!");
        }
        else
        {
            nRet = FRAMEWORK_OK;
        }
        sql_RxStatus = sqlite3_close(sh_pMultiDbMgrRxSqlMsg);
        if(sql_RxStatus != SQLITE_OK)
        {
            PrintError("Can't close Rx database : %s\n", sqlite3_errmsg(sh_pMultiDbMgrRxSqlMsg));
        }
    }

    return nRet;
}
#endif

static int32_t P_MULTI_DB_MANAGER_WriteTxt(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    switch(pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType)
    {
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pMultiDbMgrTxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pMultiDbMgrTxMsg, "eDeviceType[%d], ", pstMultiEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pMultiDbMgrTxMsg, "eTeleCommType[%d], ", pstMultiEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pMultiDbMgrTxMsg, "unDeviceId[%d], ", pstMultiEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pMultiDbMgrTxMsg, "ulTimeStamp[%ld], ", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pMultiDbMgrTxMsg, "eServiceId[%d], ", pstMultiEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pMultiDbMgrTxMsg, "eActionType[%d], ", pstMultiEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pMultiDbMgrTxMsg, "eRegionId[%d], ", pstMultiEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pMultiDbMgrTxMsg, "ePayloadType[%d], ", pstMultiEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pMultiDbMgrTxMsg, "eCommId[%d], ", pstMultiEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pMultiDbMgrTxMsg, "usDbVer[%d.%d], ", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pMultiDbMgrTxMsg, "usHwVer[0x%x], ", pstMultiEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pMultiDbMgrTxMsg, "usSwVer[0x%x], ", pstMultiEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pMultiDbMgrTxMsg, "ulPayloadLength[%d], ", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pMultiDbMgrTxMsg, "cPayload[");
                for(int i = 0; i < (int)pstMultiEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pMultiDbMgrTxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pMultiDbMgrTxMsg, "], ");

                fprintf(sh_pMultiDbMgrTxMsg, "unTotalPacketCrc32[0x%x]", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
                fprintf(sh_pMultiDbMgrTxMsg, "\r\n");

                nRet = fflush(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }

                if(pstMultiEventMsg->pPayload != NULL)
                {
                    if(s_bMultiDbMgrLog == ON)
                    {
                        PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
                    }
                    free(pstMultiEventMsg->pPayload);
                }
            }
            else
            {
                PrintError("sh_pMultiDbMgrTxMsg is NULL!!, check whethter sh_pMultiDbMgrTxMsg is opened before.");
            }

            break;
        }
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pMultiDbMgrRxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pMultiDbMgrRxMsg, "eDeviceType[%d], ", pstMultiEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pMultiDbMgrRxMsg, "eTeleCommType[%d], ", pstMultiEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pMultiDbMgrRxMsg, "unDeviceId[%d], ", pstMultiEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pMultiDbMgrRxMsg, "ulTimeStamp[%ld], ", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pMultiDbMgrRxMsg, "eServiceId[%d], ", pstMultiEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pMultiDbMgrRxMsg, "eActionType[%d], ", pstMultiEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pMultiDbMgrRxMsg, "eRegionId[%d], ", pstMultiEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pMultiDbMgrRxMsg, "ePayloadType[%d], ", pstMultiEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pMultiDbMgrRxMsg, "eCommId[%d], ", pstMultiEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pMultiDbMgrRxMsg, "usDbVer[%d.%d], ", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pMultiDbMgrRxMsg, "usHwVer[0x%x], ", pstMultiEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pMultiDbMgrRxMsg, "usSwVer[0x%x], ", pstMultiEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pMultiDbMgrRxMsg, "ulPayloadLength[%d], ", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pMultiDbMgrRxMsg, "cPayload[");
                for(int i = 0; i < (int)pstMultiEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pMultiDbMgrRxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pMultiDbMgrRxMsg, "], ");

                fprintf(sh_pMultiDbMgrRxMsg, "unTotalPacketCrc32[0x%x]", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
                fprintf(sh_pMultiDbMgrRxMsg, "\r\n");

                nRet = fflush(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }

                if(pstMultiEventMsg->pPayload != NULL)
                {
                    if(s_bMultiDbMgrLog == ON)
                    {
                        PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
                    }
                    free(pstMultiEventMsg->pPayload);
                }
            }
            else
            {
                PrintError("sh_pMultiDbMgrRxMsg is NULL!!, check whethter sh_pMultiDbMgrRxMsg is opened before.");
            }

            break;
        }
        default:
            PrintError("unknown eMultiCommMsgType [%d]", pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType);
            break;
    }

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_OpenCsv(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager is NULL!");
        return nRet;
    }

    UNUSED(pstMultiDbManager);

    if(sh_pMultiDbMgrTxMsg == NULL)
    {
        sh_pMultiDbMgrTxMsg = fopen(MULTI_DB_MANAGER_CSV_TX_FILE, "a+");
        if(sh_pMultiDbMgrTxMsg == NULL)
        {
            PrintError("fopen() is failed!!");
        }
        else
        {
            PrintTrace("MULTI_DB_MANAGER_TX_FILE[%s] is opened.", MULTI_DB_MANAGER_CSV_TX_FILE);
            nRet = FRAMEWORK_OK;
        }
    }

    if(sh_pMultiDbMgrRxMsg == NULL)
    {
        sh_pMultiDbMgrRxMsg = fopen(MULTI_DB_MANAGER_CSV_RX_FILE, "a+");
        if(sh_pMultiDbMgrRxMsg == NULL)
        {
            PrintError("fopen() is failed!!");
        }
        else
        {
            PrintTrace("MULTI_DB_MANAGER_RX_FILE[%s] is opened.", MULTI_DB_MANAGER_CSV_RX_FILE);
            nRet = FRAMEWORK_OK;
        }
    }

    if (sh_pMultiDbMgrTxMsg != NULL)
    {
        fprintf(sh_pMultiDbMgrTxMsg, "eDeviceType,");
        fprintf(sh_pMultiDbMgrTxMsg, "eTeleCommType,");
        fprintf(sh_pMultiDbMgrTxMsg, "unDeviceId,");
        fprintf(sh_pMultiDbMgrTxMsg, "ulTimeStamp,");
        fprintf(sh_pMultiDbMgrTxMsg, "eServiceId,");
        fprintf(sh_pMultiDbMgrTxMsg, "eActionType,");
        fprintf(sh_pMultiDbMgrTxMsg, "eRegionId,");
        fprintf(sh_pMultiDbMgrTxMsg, "ePayloadType,");
        fprintf(sh_pMultiDbMgrTxMsg, "eCommId,");
        fprintf(sh_pMultiDbMgrTxMsg, "usDbVer,");
        fprintf(sh_pMultiDbMgrTxMsg, "usHwVer,");
        fprintf(sh_pMultiDbMgrTxMsg, "usSwVer,");
        fprintf(sh_pMultiDbMgrTxMsg, "ulPayloadLength,");

        if(pstMultiDbManager->eMultiSvcType == MULTI_DB_MANAGER_SVC_TYPE_V2X_STATUS)
        {
            fprintf(sh_pMultiDbMgrTxMsg, "ulTxTimeStampL1,");
            fprintf(sh_pMultiDbMgrTxMsg, "ulTxTimeStampL2,");
            fprintf(sh_pMultiDbMgrTxMsg, "ulTxTimeStampL3,");
            fprintf(sh_pMultiDbMgrTxMsg, "unRxTargetDeviceId,");
            fprintf(sh_pMultiDbMgrTxMsg, "usTxFreq,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucTxPwr,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucTxBw,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucScs,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucMcs,");
            fprintf(sh_pMultiDbMgrTxMsg, "usTxRatio,");
            fprintf(sh_pMultiDbMgrTxMsg, "nTxLatitude,");
            fprintf(sh_pMultiDbMgrTxMsg, "nTxLongitude,");
            fprintf(sh_pMultiDbMgrTxMsg, "nTxAttitude,");
            fprintf(sh_pMultiDbMgrTxMsg, "unSeqNum,");
            fprintf(sh_pMultiDbMgrTxMsg, "unContCnt,");
            fprintf(sh_pMultiDbMgrTxMsg, "unTxVehicleSpeed,");
            fprintf(sh_pMultiDbMgrTxMsg, "unTxVehicleHeading,");
            fprintf(sh_pMultiDbMgrTxMsg, "unTotalPacketCrc32");
        }
        else if(pstMultiDbManager->eMultiSvcType == MULTI_DB_MANAGER_SVC_TYPE_PLATOONING)
        {
            /* V2X Status */
            fprintf(sh_pMultiDbMgrTxMsg, "ulTxTimeStampL1,");
            fprintf(sh_pMultiDbMgrTxMsg, "ulTxTimeStampL2,");
            fprintf(sh_pMultiDbMgrTxMsg, "ulTxTimeStampL3,");
            fprintf(sh_pMultiDbMgrTxMsg, "unRxTargetDeviceId,");
            fprintf(sh_pMultiDbMgrTxMsg, "usTxFreq,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucTxPwr,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucTxBw,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucScs,");
            fprintf(sh_pMultiDbMgrTxMsg, "ucMcs,");
            fprintf(sh_pMultiDbMgrTxMsg, "usTxRatio,");
            fprintf(sh_pMultiDbMgrTxMsg, "nTxLatitude,");
            fprintf(sh_pMultiDbMgrTxMsg, "nTxLongitude,");
            fprintf(sh_pMultiDbMgrTxMsg, "nTxAttitude,");
            fprintf(sh_pMultiDbMgrTxMsg, "unSeqNum,");
            fprintf(sh_pMultiDbMgrTxMsg, "unContCnt,");
            fprintf(sh_pMultiDbMgrTxMsg, "unTxVehicleSpeed,");
            fprintf(sh_pMultiDbMgrTxMsg, "unTxVehicleHeading,");
            /* PLATOONING */
            fprintf(sh_pMultiDbMgrTxMsg, "eDbV2XPtType,");
            fprintf(sh_pMultiDbMgrTxMsg, "usV2xGroupId,");
            if (pstMultiDbManager->stDbV2xPt.eDbV2XPtType == eDB_V2X_PT_TYPE_LV)
            {
                fprintf(sh_pMultiDbMgrTxMsg, "eLvServiceId,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvMethodId,");
                fprintf(sh_pMultiDbMgrTxMsg, "unLvLength,");
                fprintf(sh_pMultiDbMgrTxMsg, "usLvClientId,");
                fprintf(sh_pMultiDbMgrTxMsg, "usLvSessionId,");
                fprintf(sh_pMultiDbMgrTxMsg, "ucLvProtocolVer,");
                fprintf(sh_pMultiDbMgrTxMsg, "ucLvInterfaceVer,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvMsgType,");
                fprintf(sh_pMultiDbMgrTxMsg, "ucLvReturnCode,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvVehicleType,");
                fprintf(sh_pMultiDbMgrTxMsg, "szLvVehicleId,");
                fprintf(sh_pMultiDbMgrTxMsg, "szLvVehicleNum,");
                fprintf(sh_pMultiDbMgrTxMsg, "usLvMsgCount,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvMsgId,");
                fprintf(sh_pMultiDbMgrTxMsg, "nLvLatitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "nLvLongitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "usLvHeading,");
                fprintf(sh_pMultiDbMgrTxMsg, "usLvSpeed,");
                fprintf(sh_pMultiDbMgrTxMsg, "szLvDriveLaneId,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvDriveStatus,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvChangeCode,");
                fprintf(sh_pMultiDbMgrTxMsg, "usLvPathId,");
                fprintf(sh_pMultiDbMgrTxMsg, "szLvLaneId,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvLanePlan,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvCrossway,");
                fprintf(sh_pMultiDbMgrTxMsg, "eLvLaneManeuver,");
                fprintf(sh_pMultiDbMgrTxMsg, "anLvLatitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "anLvLongitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "unReserved1,");
            }
            else if (pstMultiDbManager->stDbV2xPt.eDbV2XPtType == eDB_V2X_PT_TYPE_FV)
            {
                fprintf(sh_pMultiDbMgrTxMsg, "eFvServiceId,");
                fprintf(sh_pMultiDbMgrTxMsg, "eFvMethodId,");
                fprintf(sh_pMultiDbMgrTxMsg, "unFvLength,");
                fprintf(sh_pMultiDbMgrTxMsg, "usFvClientId,");
                fprintf(sh_pMultiDbMgrTxMsg, "usFvSessionId,");
                fprintf(sh_pMultiDbMgrTxMsg, "ucFvProtocolVer,");
                fprintf(sh_pMultiDbMgrTxMsg, "ucFvInterfaceVer,");
                fprintf(sh_pMultiDbMgrTxMsg, "eFvMsgType,");
                fprintf(sh_pMultiDbMgrTxMsg, "ucFvReturnCode,");
                fprintf(sh_pMultiDbMgrTxMsg, "eFvVehicleType,");
                fprintf(sh_pMultiDbMgrTxMsg, "szFvVehicleId,");
                fprintf(sh_pMultiDbMgrTxMsg, "szFvVehicleNum,");
                fprintf(sh_pMultiDbMgrTxMsg, "usFvMsgCount,");
                fprintf(sh_pMultiDbMgrTxMsg, "eFvMsgId,");
                fprintf(sh_pMultiDbMgrTxMsg, "nFvLatitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "nFvLongitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "usFvHeading,");
                fprintf(sh_pMultiDbMgrTxMsg, "usFvSpeed,");
                fprintf(sh_pMultiDbMgrTxMsg, "szFvDriveLaneId,");
                fprintf(sh_pMultiDbMgrTxMsg, "eFvDriveStatus,");
                fprintf(sh_pMultiDbMgrTxMsg, "eFvChangeCode,");
                fprintf(sh_pMultiDbMgrTxMsg, "anFvLatitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "anFvLongitude,");
                fprintf(sh_pMultiDbMgrTxMsg, "usFvRecommDistance,");
                fprintf(sh_pMultiDbMgrTxMsg, "usFvRecommSpeed,");
                fprintf(sh_pMultiDbMgrTxMsg, "unReserved1,");
                fprintf(sh_pMultiDbMgrTxMsg, "unReserved2,");
                fprintf(sh_pMultiDbMgrTxMsg, "unReserved3,");
                fprintf(sh_pMultiDbMgrTxMsg, "unReserved4,");
                fprintf(sh_pMultiDbMgrTxMsg, "unReserved5,");
                fprintf(sh_pMultiDbMgrTxMsg, "unReserved6,");
            }
            else
            {
                PrintError("unknown eDbV2XPtType[%d]", pstMultiDbManager->stDbV2xPt.eDbV2XPtType);
            }
            /* CRC */
            fprintf(sh_pMultiDbMgrTxMsg, "unTotalPacketCrc32");
        }
        else if(pstMultiDbManager->eMultiSvcType == MULTI_DB_MANAGER_SVC_TYPE_BASE)
        {
            fprintf(sh_pMultiDbMgrTxMsg, "cPayload,");
            fprintf(sh_pMultiDbMgrTxMsg, "unTotalPacketCrc32");
        }
        else
        {
            PrintError("unknown eMultiSvcType[nRet:%d]", pstMultiDbManager->eMultiSvcType);
            return nRet;
        }

        fprintf(sh_pMultiDbMgrTxMsg, "\r\n");
    }

    nRet = fflush(sh_pMultiDbMgrTxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if (sh_pMultiDbMgrRxMsg != NULL)
    {
        fprintf(sh_pMultiDbMgrRxMsg, "eDeviceType,");
        fprintf(sh_pMultiDbMgrRxMsg, "eTeleCommType,");
        fprintf(sh_pMultiDbMgrRxMsg, "unDeviceId,");
        fprintf(sh_pMultiDbMgrRxMsg, "ulTimeStamp,");
        fprintf(sh_pMultiDbMgrRxMsg, "eServiceId,");
        fprintf(sh_pMultiDbMgrRxMsg, "eActionType,");
        fprintf(sh_pMultiDbMgrRxMsg, "eRegionId,");
        fprintf(sh_pMultiDbMgrRxMsg, "ePayloadType,");
        fprintf(sh_pMultiDbMgrRxMsg, "eCommId,");
        fprintf(sh_pMultiDbMgrRxMsg, "usDbVer,");
        fprintf(sh_pMultiDbMgrRxMsg, "usHwVer,");
        fprintf(sh_pMultiDbMgrRxMsg, "usSwVer,");
        fprintf(sh_pMultiDbMgrRxMsg, "ulPayloadLength,");

        if(pstMultiDbManager->eMultiSvcType == MULTI_DB_MANAGER_SVC_TYPE_V2X_STATUS)
        {
            /* Tx */
            fprintf(sh_pMultiDbMgrRxMsg, "ulTxTimeStampL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTxTimeStampL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTxTimeStampL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxDeviceIdL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxDeviceIdL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxDeviceIdL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxSwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxSwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxSwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxHwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxHwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxHwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxTargetDeviceId,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxFreq,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucTxPwr,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucTxBw,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucScs,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucMcs,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxRatio,");
            fprintf(sh_pMultiDbMgrRxMsg, "nTxLatitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nTxLongitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nTxAttitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "unSeqNum,");
            fprintf(sh_pMultiDbMgrRxMsg, "unContCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxVehicleSpeed,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxVehicleHeading,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTotalPacketCrc32,");

            /* Rx */
            fprintf(sh_pMultiDbMgrRxMsg, "ulRxTimeStampL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulRxTimeStampL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulRxTimeStampL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulLatencyL1(us),");
            fprintf(sh_pMultiDbMgrRxMsg, "ulLatencyL2(us),");
            fprintf(sh_pMultiDbMgrRxMsg, "ulLatencyL3(us),");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxDeviceIdL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxDeviceIdL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxDeviceIdL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxSwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxSwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxSwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxHwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxHwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxHwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxVehicleSpeed,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxVehicleHeading,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTotalCommDevCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRssi,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucRcpi,");
            fprintf(sh_pMultiDbMgrRxMsg, "eRsvLevel,");
            fprintf(sh_pMultiDbMgrRxMsg, "usCommDistance,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRxLatitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRxLongitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRxAttitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucErrIndicator,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTotalPacketCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTotalErrCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "unPdr(percent),");
            fprintf(sh_pMultiDbMgrRxMsg, "unPer(percent)");
        }
        else if(pstMultiDbManager->eMultiSvcType == MULTI_DB_MANAGER_SVC_TYPE_PLATOONING)
        {
            /* PT Tx */
            fprintf(sh_pMultiDbMgrRxMsg, "ulTxTimeStampL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTxTimeStampL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTxTimeStampL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxDeviceIdL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxDeviceIdL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxDeviceIdL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxSwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxSwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxSwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxHwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxHwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxHwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxTargetDeviceId,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxFreq,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucTxPwr,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucTxBw,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucScs,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucMcs,");
            fprintf(sh_pMultiDbMgrRxMsg, "usTxRatio,");
            fprintf(sh_pMultiDbMgrRxMsg, "nTxLatitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nTxLongitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nTxAttitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "unSeqNum,");
            fprintf(sh_pMultiDbMgrRxMsg, "unContCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxVehicleSpeed,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTxVehicleHeading,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTotalPacketCrc32,");

            /* PT Rx */
            fprintf(sh_pMultiDbMgrRxMsg, "ulRxTimeStampL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulRxTimeStampL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulRxTimeStampL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulLatencyL1(us),");
            fprintf(sh_pMultiDbMgrRxMsg, "ulLatencyL2(us),");
            fprintf(sh_pMultiDbMgrRxMsg, "ulLatencyL3(us),");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxDeviceIdL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxDeviceIdL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxDeviceIdL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxSwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxSwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxSwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxHwVerL1,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxHwVerL2,");
            fprintf(sh_pMultiDbMgrRxMsg, "usRxHwVerL3,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxVehicleSpeed,");
            fprintf(sh_pMultiDbMgrRxMsg, "unRxVehicleHeading,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTotalCommDevCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRssi,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucRcpi,");
            fprintf(sh_pMultiDbMgrRxMsg, "eRsvLevel,");
            fprintf(sh_pMultiDbMgrRxMsg, "usCommDistance,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRxLatitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRxLongitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "nRxAttitude,");
            fprintf(sh_pMultiDbMgrRxMsg, "ucErrIndicator,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTotalPacketCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "ulTotalErrCnt,");
            fprintf(sh_pMultiDbMgrRxMsg, "unPdr(percent),");
            fprintf(sh_pMultiDbMgrRxMsg, "unPer(percent),");
            fprintf(sh_pMultiDbMgrRxMsg, "eDbV2XPtType,");
            fprintf(sh_pMultiDbMgrRxMsg, "usV2xGroupId,");

            if (pstMultiDbManager->stDbV2xPt.eDbV2XPtType == eDB_V2X_PT_TYPE_LV)
            {
                fprintf(sh_pMultiDbMgrRxMsg, "eFvServiceId,");
                fprintf(sh_pMultiDbMgrRxMsg, "eFvMethodId,");
                fprintf(sh_pMultiDbMgrRxMsg, "unFvLength,");
                fprintf(sh_pMultiDbMgrRxMsg, "usFvClientId,");
                fprintf(sh_pMultiDbMgrRxMsg, "usFvSessionId,");
                fprintf(sh_pMultiDbMgrRxMsg, "ucFvProtocolVer,");
                fprintf(sh_pMultiDbMgrRxMsg, "ucFvInterfaceVer,");
                fprintf(sh_pMultiDbMgrRxMsg, "eFvMsgType,");
                fprintf(sh_pMultiDbMgrRxMsg, "ucFvReturnCode,");
                fprintf(sh_pMultiDbMgrRxMsg, "eFvVehicleType,");
                fprintf(sh_pMultiDbMgrRxMsg, "szFvVehicleId,");
                fprintf(sh_pMultiDbMgrRxMsg, "szFvVehicleNum,");
                fprintf(sh_pMultiDbMgrRxMsg, "usFvMsgCount,");
                fprintf(sh_pMultiDbMgrRxMsg, "eFvMsgId,");
                fprintf(sh_pMultiDbMgrRxMsg, "nFvLatitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "nFvLongitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "usFvHeading,");
                fprintf(sh_pMultiDbMgrRxMsg, "usFvSpeed,");
                fprintf(sh_pMultiDbMgrRxMsg, "szFvDriveLaneId,");
                fprintf(sh_pMultiDbMgrRxMsg, "eFvDriveStatus,");
                fprintf(sh_pMultiDbMgrRxMsg, "anFvLatitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "eFvChangeCode,");
                fprintf(sh_pMultiDbMgrRxMsg, "anFvLongitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "usFvRecommDistance,");
                fprintf(sh_pMultiDbMgrRxMsg, "usFvRecommSpeed,");
                fprintf(sh_pMultiDbMgrRxMsg, "unReserved1,");
                fprintf(sh_pMultiDbMgrRxMsg, "unReserved2,");
                fprintf(sh_pMultiDbMgrRxMsg, "unReserved3,");
                fprintf(sh_pMultiDbMgrRxMsg, "unReserved4,");
                fprintf(sh_pMultiDbMgrRxMsg, "unReserved5,");
                fprintf(sh_pMultiDbMgrRxMsg, "unReserved6");
            }
            else if (pstMultiDbManager->stDbV2xPt.eDbV2XPtType == eDB_V2X_PT_TYPE_FV)
            {
                fprintf(sh_pMultiDbMgrRxMsg, "eLvServiceId,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvMethodId,");
                fprintf(sh_pMultiDbMgrRxMsg, "unLvLength,");
                fprintf(sh_pMultiDbMgrRxMsg, "usLvClientId,");
                fprintf(sh_pMultiDbMgrRxMsg, "usLvSessionId,");
                fprintf(sh_pMultiDbMgrRxMsg, "ucLvProtocolVer,");
                fprintf(sh_pMultiDbMgrRxMsg, "ucLvInterfaceVer,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvMsgType,");
                fprintf(sh_pMultiDbMgrRxMsg, "ucLvReturnCode,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvVehicleType,");
                fprintf(sh_pMultiDbMgrRxMsg, "szLvVehicleId,");
                fprintf(sh_pMultiDbMgrRxMsg, "szLvVehicleNum,");
                fprintf(sh_pMultiDbMgrRxMsg, "usLvMsgCount,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvMsgId,");
                fprintf(sh_pMultiDbMgrRxMsg, "nLvLatitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "nLvLongitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "usLvHeading,");
                fprintf(sh_pMultiDbMgrRxMsg, "usLvSpeed,");
                fprintf(sh_pMultiDbMgrRxMsg, "szLvDriveLaneId,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvDriveStatus,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvChangeCode,");
                fprintf(sh_pMultiDbMgrRxMsg, "usLvPathId,");
                fprintf(sh_pMultiDbMgrRxMsg, "szLvLaneId,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvLanePlan,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvCrossway,");
                fprintf(sh_pMultiDbMgrRxMsg, "eLvLaneManeuver,");
                fprintf(sh_pMultiDbMgrRxMsg, "anLvLatitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "anLvLongitude,");
                fprintf(sh_pMultiDbMgrRxMsg, "unReserved1");
            }
            else
            {
                PrintError("unknown eDbV2XPtType[%d]", pstMultiDbManager->stDbV2xPt.eDbV2XPtType);
            }
        }
        else if(pstMultiDbManager->eMultiSvcType == MULTI_DB_MANAGER_SVC_TYPE_BASE)
        {
            fprintf(sh_pMultiDbMgrRxMsg, "cPayload,");
            fprintf(sh_pMultiDbMgrRxMsg, "unTotalPacketCrc32");
        }
        else
        {
            PrintError("unknown eMultiSvcType[nRet:%d]", pstMultiDbManager->eMultiSvcType);
            return nRet;
        }

        fprintf(sh_pMultiDbMgrRxMsg, "\r\n");
    }

    nRet = fflush(sh_pMultiDbMgrRxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_WriteCsvPlatooningThroughput(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    switch(pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType)
    {
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pMultiDbMgrTxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pMultiDbMgrTxMsg, "%ld,", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pMultiDbMgrTxMsg, "%d.%d,", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pMultiDbMgrTxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pMultiDbMgrTxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                for(int i = 0; i < (int)pstMultiEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pMultiDbMgrTxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pMultiDbMgrTxMsg, ",");

                fprintf(sh_pMultiDbMgrTxMsg, "0x%x", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
                fprintf(sh_pMultiDbMgrTxMsg, "\r\n");

                nRet = fflush(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }

                if(pstMultiEventMsg->pPayload != NULL)
                {
                    if(s_bMultiDbMgrLog == ON)
                    {
                        PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
                    }
                    free(pstMultiEventMsg->pPayload);
                }
            }
            else
            {
                PrintError("sh_pMultiDbMgrTxMsg is NULL!!, check whethter sh_pMultiDbMgrTxMsg is opened before.");
            }

            break;
        }
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pMultiDbMgrRxMsg != NULL)
            {
                pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
                if(pchPayload == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                    return nRet;
                }

                memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eDeviceType);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eTeleCommType);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->unDeviceId);
                fprintf(sh_pMultiDbMgrRxMsg, "%ld,", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eServiceId);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eActionType);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eRegionId);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ePayloadType);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eCommId);
                fprintf(sh_pMultiDbMgrRxMsg, "%d.%d,", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
                fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usHwVer);
                fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usSwVer);
                fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

                for(int i = 0; i < (int)pstMultiEventMsg->pstDbV2x->ulPayloadLength; i++)
                {
                      fprintf(sh_pMultiDbMgrRxMsg, "%d ", pchPayload[i]);
                }
                fprintf(sh_pMultiDbMgrRxMsg, ",");

                fprintf(sh_pMultiDbMgrRxMsg, "0x%x", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
                fprintf(sh_pMultiDbMgrRxMsg, "\r\n");

                nRet = fflush(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                if(pchPayload != NULL)
                {
                    free(pchPayload);
                }

                if(pstMultiEventMsg->pPayload != NULL)
                {
                    if(s_bMultiDbMgrLog == ON)
                    {
                        PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
                    }
                    free(pstMultiEventMsg->pPayload);
                }
            }
            else
            {
                PrintError("sh_pDbMgrRxMsg is NULL!!, check whethter sh_pDbMgrRxMsg is opened before.");
            }

            break;
        }
        default:
            PrintError("unknown eCommMsgType [%d]", pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType);
            break;
    }

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_WriteCsvPlatooningTx(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    DB_V2X_STATUS_TX_T stDbV2xStatusTx;
    DB_V2X_PLATOONING_T stDbV2XPt;
    DB_V2X_PLATOONING_LV_T stDbV2XPtLv;
    DB_V2X_PLATOONING_FV_T stDbV2XPtFv;
    double dTemp;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    memset(&stDbV2xStatusTx, 0, sizeof(DB_V2X_STATUS_TX_T));
    memset(&stDbV2XPt, 0, sizeof(DB_V2X_PLATOONING_T));
    memset(&stDbV2XPtLv, 0, sizeof(DB_V2X_PLATOONING_LV_T));
    memset(&stDbV2XPtFv, 0, sizeof(DB_V2X_PLATOONING_FV_T));

    pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
    if(pchPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eDeviceType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eTeleCommType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->unDeviceId);
    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eServiceId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eActionType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eRegionId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ePayloadType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eCommId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d.%d,", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
    fprintf(sh_pMultiDbMgrTxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usHwVer);
    fprintf(sh_pMultiDbMgrTxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usSwVer);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    memcpy(&stDbV2xStatusTx, pchPayload, sizeof(DB_V2X_STATUS_TX_T));
    memcpy(&stDbV2XPt, pchPayload + sizeof(DB_V2X_STATUS_TX_T), sizeof(DB_V2X_PLATOONING_T));

    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unRxTargetDeviceId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.usTxFreq);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucTxPwr);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucTxBw);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucScs);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucMcs);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.usTxRatio);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLatitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrTxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLongitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrTxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxAttitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrTxMsg, "%lf,", dTemp);

    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unSeqNum);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unContCnt);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unTxVehicleSpeed);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unTxVehicleHeading);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPt.eDbV2XPtType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPt.usV2xGroupId);
    if (stDbV2XPt.eDbV2XPtType == eDB_V2X_PT_TYPE_LV)
    {
        memcpy(&stDbV2XPtLv, pchPayload + sizeof(DB_V2X_STATUS_TX_T) + sizeof(DB_V2X_PLATOONING_T), sizeof(DB_V2X_PLATOONING_LV_T));

        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvServiceId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvMethodId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.unLvLength);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.usLvClientId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.usLvSessionId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.ucLvProtocolVer);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.ucLvInterfaceVer);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvMsgType);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.ucLvReturnCode);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvVehicleType);
        fprintf(sh_pMultiDbMgrTxMsg, "%s,", stDbV2XPtLv.szLvVehicleId);
        fprintf(sh_pMultiDbMgrTxMsg, "%s,", stDbV2XPtLv.szLvVehicleNum);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.usLvMsgCount);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvMsgId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.nLvLatitude);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.nLvLongitude);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.usLvHeading);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.usLvSpeed);
        fprintf(sh_pMultiDbMgrTxMsg, "%s,", stDbV2XPtLv.szLvDriveLaneId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvDriveStatus);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvChangeCode);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.usLvPathId);
        fprintf(sh_pMultiDbMgrTxMsg, "%s,", stDbV2XPtLv.szLvLaneId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvLanePlan);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvCrossway);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtLv.eLvLaneManeuver);
        fprintf(sh_pMultiDbMgrTxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtLv.stLvPathPlan.anLvLatitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrTxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrTxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrTxMsg, "\",");
        fprintf(sh_pMultiDbMgrTxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtLv.stLvPathPlan.anLvLongitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrTxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrTxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrTxMsg, "\",");
        fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2XPtLv.unReserved1);
    }
    else if (stDbV2XPt.eDbV2XPtType == eDB_V2X_PT_TYPE_FV)
    {
        memcpy(&stDbV2XPtFv, pchPayload + sizeof(DB_V2X_STATUS_TX_T) + sizeof(DB_V2X_PLATOONING_T), sizeof(DB_V2X_PLATOONING_FV_T));

        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.eFvServiceId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.eFvMethodId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.unFvLength);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.usFvClientId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.usFvSessionId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.ucFvProtocolVer);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.ucFvInterfaceVer);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.eFvMsgType);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.ucFvReturnCode);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.eFvVehicleType);
        fprintf(sh_pMultiDbMgrTxMsg, "%s,", stDbV2XPtFv.szFvVehicleId);
        fprintf(sh_pMultiDbMgrTxMsg, "%s,", stDbV2XPtFv.szFvVehicleNum);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.usFvMsgCount);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.eFvMsgId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.nFvLatitude);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.nFvLongitude);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.usFvHeading);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.usFvSpeed);
        fprintf(sh_pMultiDbMgrTxMsg, "%s,", stDbV2XPtFv.szFvDriveLaneId);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.eFvDriveStatus);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.eFvChangeCode);
        fprintf(sh_pMultiDbMgrTxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtFv.stFvPathPlan.anFvLatitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrTxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrTxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrTxMsg, "\",");
        fprintf(sh_pMultiDbMgrTxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtFv.stFvPathPlan.anFvLongitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrTxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrTxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrTxMsg, "\",");
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.usFvRecommDistance);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.usFvRecommSpeed);
        fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2XPtFv.unReserved1);
        fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2XPtFv.unReserved2);
        fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2XPtFv.unReserved3);
        fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2XPtFv.unReserved4);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.unReserved5);
        fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2XPtFv.unReserved6);
    }
    else
    {
        PrintError("unknown eDbV2XPtType[%d]", stDbV2XPt.eDbV2XPtType);
    }
    fprintf(sh_pMultiDbMgrTxMsg, "0x%x", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
    fprintf(sh_pMultiDbMgrTxMsg, "\r\n");

    nRet = fflush(sh_pMultiDbMgrTxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if(pchPayload != NULL)
    {
        free(pchPayload);
    }

    if(pstMultiEventMsg->pPayload != NULL)
    {
        if(s_bMultiDbMgrLog == ON)
        {
            PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
        }
        free(pstMultiEventMsg->pPayload);
    }

    return nRet;

}

static int32_t P_MULTI_DB_MANAGER_WriteCsvPlatooningRx(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    DB_V2X_STATUS_TX_T stDbV2xStatusTx;
    DB_V2X_STATUS_RX_T stDbV2xStatusRx;
    MULTI_DB_MANAGER_V2X_STATUS_T stMultiDbV2xStatus;
    DB_V2X_PLATOONING_T stDbV2XPt;
    DB_V2X_PLATOONING_LV_T stDbV2XPtLv;
    DB_V2X_PLATOONING_FV_T stDbV2XPtFv;
    DI_T *pstDi;
    float fTemp;
    double dTemp;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    memset(&stDbV2xStatusTx, 0, sizeof(DB_V2X_STATUS_TX_T));
    memset(&stDbV2xStatusRx, 0, sizeof(DB_V2X_STATUS_RX_T));
    memset(&stDbV2XPt, 0, sizeof(DB_V2X_PLATOONING_T));
    memset(&stDbV2XPtLv, 0, sizeof(DB_V2X_PLATOONING_LV_T));
    memset(&stDbV2XPtFv, 0, sizeof(DB_V2X_PLATOONING_FV_T));

    pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
    if(pchPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eDeviceType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eTeleCommType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->unDeviceId);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eServiceId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eActionType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eRegionId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ePayloadType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eCommId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d.%d,", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    memcpy(&stDbV2xStatusTx, pchPayload, sizeof(DB_V2X_STATUS_TX_T));
    memcpy(&stDbV2XPt, pchPayload + sizeof(DB_V2X_STATUS_TX_T), sizeof(DB_V2X_PLATOONING_T));

    nRet = P_MULTI_DB_MANAGER_UpdateStatus(pstMultiEventMsg, &stDbV2xStatusTx, &stDbV2xStatusRx);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_UpdateStatus() is failed! [unRet:%d]", nRet);
    }

    nRet = P_MULTI_DB_MANAGER_PrintStatus(&stDbV2xStatusTx, &stDbV2xStatusRx);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_PrintStatus() is failed! [unRet:%d]", nRet);
    }

    nRet = P_MULTI_DB_MANAGER_PrintStatusPt(&stDbV2XPt);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_PrintStatusPt() is failed! [unRet:%d]", nRet);
    }

    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL1.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL2.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL3.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL1.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL2.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL3.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL1.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL2.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL3.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unRxTargetDeviceId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.usTxFreq);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucTxPwr);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucTxBw);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucScs);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucMcs);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.usTxRatio);

    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLatitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLongitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxAttitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unSeqNum);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unContCnt);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unTxVehicleSpeed);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unTxVehicleHeading);

    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);

    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL1.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL2.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL3.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL1.ulLatency);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL2.ulLatency);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL3.ulLatency);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL1.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL2.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL3.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL1.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL2.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL3.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL1.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL2.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL3.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.unRxVehicleSpeed);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.unRxVehicleHeading);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.unTotalCommDevCnt);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.nRssi);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.ucRcpi);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.eRsvLevel);

    pstDi = APP_GetDiInstance();
    if(pstDi == NULL)
    {
        PrintError("pstDi is NULL!");
    }

    /* Set the GPS values */
    nRet = DI_GPS_Get(&pstDi->stDiGps);
    if (nRet != DI_OK)
    {
        PrintError("DI_GPS_Get() is failed! [nRet:%d]", nRet);
    }
    dTemp = (double)stDbV2xStatusRx.stRxPosition.unCommDistance / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusRx.stRxPosition.nRxLatitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusRx.stRxPosition.nRxLongitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusRx.stRxPosition.nRxAttitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.ucErrIndicator);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.ulTotalPacketCnt);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.ulTotalErrCnt);
    fTemp = (float)stDbV2xStatusRx.unPdr / 1000.0f;
    fprintf(sh_pMultiDbMgrRxMsg, "%.3f,", fTemp);
    fTemp = (float)stDbV2xStatusRx.unPer / 1000.0f;
    fprintf(sh_pMultiDbMgrRxMsg, "%.3f,", fTemp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPt.eDbV2XPtType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPt.usV2xGroupId);

    if (stDbV2XPt.eDbV2XPtType == eDB_V2X_PT_TYPE_LV)
    {
        memcpy(&stDbV2XPtLv, pchPayload + sizeof(DB_V2X_STATUS_TX_T) + sizeof(DB_V2X_PLATOONING_T), sizeof(DB_V2X_PLATOONING_LV_T));

        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvServiceId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvMethodId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.unLvLength);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.usLvClientId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.usLvSessionId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.ucLvProtocolVer);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.ucLvInterfaceVer);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvMsgType);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.ucLvReturnCode);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvVehicleType);
        fprintf(sh_pMultiDbMgrRxMsg, "%s,", stDbV2XPtLv.szLvVehicleId);
        fprintf(sh_pMultiDbMgrRxMsg, "%s,", stDbV2XPtLv.szLvVehicleNum);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.usLvMsgCount);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvMsgId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.nLvLatitude);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.nLvLongitude);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.usLvHeading);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.usLvSpeed);
        fprintf(sh_pMultiDbMgrRxMsg, "%s,", stDbV2XPtLv.szLvDriveLaneId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvDriveStatus);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvChangeCode);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.usLvPathId);
        fprintf(sh_pMultiDbMgrRxMsg, "%s,", stDbV2XPtLv.szLvLaneId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvLanePlan);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvCrossway);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtLv.eLvLaneManeuver);
        fprintf(sh_pMultiDbMgrRxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtLv.stLvPathPlan.anLvLatitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrRxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrRxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrRxMsg, "\",");
        fprintf(sh_pMultiDbMgrRxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtLv.stLvPathPlan.anLvLongitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrRxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_LV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrRxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrRxMsg, "\",");
        fprintf(sh_pMultiDbMgrRxMsg, "%ld", stDbV2XPtLv.unReserved1);

        nRet = P_MULTI_DB_MANAGER_PrintStatusPtLv(&stDbV2XPtLv);
        if (nRet != FRAMEWORK_OK)
        {
            PrintError("P_MULTI_DB_MANAGER_PrintStatusPtLv() is failed! [unRet:%d]", nRet);
        }

    }
    else if (stDbV2XPt.eDbV2XPtType == eDB_V2X_PT_TYPE_FV)
    {
        memcpy(&stDbV2XPtFv, pchPayload + sizeof(DB_V2X_STATUS_TX_T) + sizeof(DB_V2X_PLATOONING_T), sizeof(DB_V2X_PLATOONING_FV_T));

        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.eFvServiceId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.eFvMethodId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.unFvLength);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.usFvClientId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.usFvSessionId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.ucFvProtocolVer);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.ucFvInterfaceVer);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.eFvMsgType);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.ucFvReturnCode);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.eFvVehicleType);
        fprintf(sh_pMultiDbMgrRxMsg, "%s,", stDbV2XPtFv.szFvVehicleId);
        fprintf(sh_pMultiDbMgrRxMsg, "%s,", stDbV2XPtFv.szFvVehicleNum);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.usFvMsgCount);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.eFvMsgId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.nFvLatitude);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.nFvLongitude);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.usFvHeading);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.usFvSpeed);
        fprintf(sh_pMultiDbMgrRxMsg, "%s,", stDbV2XPtFv.szFvDriveLaneId);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.eFvDriveStatus);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.eFvChangeCode);
        fprintf(sh_pMultiDbMgrRxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtFv.stFvPathPlan.anFvLatitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrRxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrRxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrRxMsg, "\",");
        fprintf(sh_pMultiDbMgrRxMsg, "\"");
        for (int i = 0; i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN; i++)
        {
            dTemp = (double)stDbV2XPtFv.stFvPathPlan.anFvLongitude[i] / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
            fprintf(sh_pMultiDbMgrRxMsg, "%lf", dTemp);
            if (i < DB_V2X_PT_FV_PATH_PLAN_MAX_LEN - 1)
            {
                fprintf(sh_pMultiDbMgrRxMsg, ",");
            }
        }
        fprintf(sh_pMultiDbMgrRxMsg, "\",");
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.usFvRecommDistance);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.usFvRecommSpeed);
        fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2XPtFv.unReserved1);
        fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2XPtFv.unReserved2);
        fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2XPtFv.unReserved3);
        fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2XPtFv.unReserved4);
        fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2XPtFv.unReserved5);
        fprintf(sh_pMultiDbMgrRxMsg, "%d", stDbV2XPtFv.unReserved6);

        nRet = P_MULTI_DB_MANAGER_PrintStatusPtFv(&stDbV2XPtFv);
        if (nRet != FRAMEWORK_OK)
        {
            PrintError("P_MULTI_DB_MANAGER_PrintStatusPtFv() is failed! [unRet:%d]", nRet);
        }
    }
    else
    {
        PrintError("unknown eDbV2XPtType[%d]", stDbV2XPt.eDbV2XPtType);
    }

    fprintf(sh_pMultiDbMgrRxMsg, "\r\n");

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    /* Reset the err indicator */
    stMultiDbV2xStatus.stV2xStatusRx.ucErrIndicator = FALSE;
    nRet = P_MULTI_DB_MANAGER_SetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    nRet = fflush(sh_pMultiDbMgrRxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if(pchPayload != NULL)
    {
        free(pchPayload);
    }

    if(pstMultiEventMsg->pPayload != NULL)
    {
        if(s_bMultiDbMgrLog == ON)
        {
            PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
        }
        free(pstMultiEventMsg->pPayload);
    }

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_WriteCsvPlatooning(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    switch(pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType)
    {
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pMultiDbMgrTxMsg != NULL)
            {
                nRet = P_MULTI_DB_MANAGER_WriteCsvPlatooningTx(pstMultiEventMsg);
                if(nRet != FRAMEWORK_OK)
                {
                    PrintError("P_MULTI_DB_MANAGER_WriteCsvPlatooningTx() is failed! [unRet:%d]", nRet);
                }
            }
            else
            {
                PrintError("sh_pMultiDbMgrTxMsg is NULL!!, check whethter sh_pMultiDbMgrTxMsg is opened before.");
            }
            break;
        }
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pMultiDbMgrRxMsg != NULL)
            {
                nRet = P_MULTI_DB_MANAGER_WriteCsvPlatooningRx(pstMultiEventMsg);
                if(nRet != FRAMEWORK_OK)
                {
                    PrintError("P_MULTI_DB_MANAGER_WriteCsvPlatooningRx() is failed! [unRet:%d]", nRet);
                }
            }
            else
            {
                PrintError("sh_pMultiDbMgrRxMsg is NULL!!, check whethter sh_pMultiDbMgrRxMsg is opened before.");
            }
            break;
        }
        default:
            PrintError("unknown eMultiCommMsgType [%d]", pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType);
            break;
    }
    nRet = FRAMEWORK_OK;

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_WriteCsvV2xStatusTx(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    DB_V2X_STATUS_TX_T stDbV2xStatusTx;
    double dTemp;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
    if(pchPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eDeviceType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eTeleCommType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->unDeviceId);
    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eServiceId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eActionType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eRegionId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ePayloadType);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eCommId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d.%d,", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
    fprintf(sh_pMultiDbMgrTxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usHwVer);
    fprintf(sh_pMultiDbMgrTxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usSwVer);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    memcpy(&stDbV2xStatusTx, pchPayload, sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unRxTargetDeviceId);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.usTxFreq);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucTxPwr);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucTxBw);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucScs);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.ucMcs);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.usTxRatio);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLatitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrTxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLongitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrTxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxAttitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrTxMsg, "%lf,", dTemp);

    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unSeqNum);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unContCnt);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unTxVehicleSpeed);
    fprintf(sh_pMultiDbMgrTxMsg, "%d,", stDbV2xStatusTx.unTxVehicleHeading);
    fprintf(sh_pMultiDbMgrTxMsg, "0x%x", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);
    fprintf(sh_pMultiDbMgrTxMsg, "\r\n");

    nRet = fflush(sh_pMultiDbMgrTxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if(pchPayload != NULL)
    {
        free(pchPayload);
    }

    if(pstMultiEventMsg->pPayload != NULL)
    {
        if(s_bMultiDbMgrLog == ON)
        {
            PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
        }
        free(pstMultiEventMsg->pPayload);
    }

    return nRet;
}

#if defined(DB_MGR_TEST)
static int32_t P_MULTI_DB_MANAGER_TestPacket(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus)
{
    int32_t nRet = FRAMEWORK_ERROR;

    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d.%d,", 0, 0);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", 0);
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiDbV2xStatus->stV2xStatusRx.ucErrIndicator);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", 0UL);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", pstMultiDbV2xStatus->stV2xStatusRx.ulTotalErrCnt);
    fprintf(sh_pMultiDbMgrRxMsg, "%.2f,", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "%.2f", 0.0f);
    fprintf(sh_pMultiDbMgrRxMsg, "\r\n");

    nRet = fflush(sh_pMultiDbMgrRxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    return nRet;
}
#endif

static int32_t P_MULTI_DB_MANAGER_WriteCsvV2xStatusRx(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char *pchPayload = NULL;
    DB_V2X_STATUS_TX_T stDbV2xStatusTx;
    DB_V2X_STATUS_RX_T stDbV2xStatusRx;
    MULTI_DB_MANAGER_V2X_STATUS_T stMultiDbV2xStatus;
    DI_T *pstDi;
    float fTemp;
    double dTemp;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    memset(&stDbV2xStatusTx, 0, sizeof(DB_V2X_STATUS_TX_T));
    memset(&stDbV2xStatusRx, 0, sizeof(DB_V2X_STATUS_RX_T));

    pchPayload = (char*)malloc(sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);
    if(pchPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(pchPayload, (char *)pstMultiEventMsg->pPayload, pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eDeviceType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eTeleCommType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->unDeviceId);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", pstMultiEventMsg->pstDbV2x->ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eServiceId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eActionType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eRegionId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ePayloadType);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->eCommId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d.%d,", pstMultiEventMsg->pstDbV2x->usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstMultiEventMsg->pstDbV2x->usDbVer & CLI_DB_V2X_MINOR_MASK);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstDbV2x->usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    memcpy(&stDbV2xStatusTx, pchPayload, sizeof(char)*pstMultiEventMsg->pstDbV2x->ulPayloadLength);

    nRet = P_MULTI_DB_MANAGER_UpdateStatus(pstMultiEventMsg, &stDbV2xStatusTx, &stDbV2xStatusRx);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_UpdateStatus() is failed! [unRet:%d]", nRet);
    }

    nRet = P_MULTI_DB_MANAGER_PrintStatus(&stDbV2xStatusTx, &stDbV2xStatusRx);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_PrintStatus() is failed! [unRet:%d]", nRet);
    }

    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL1.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL2.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL3.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL1.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL2.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL3.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL1.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL2.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.stDbV2xDevL3.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unRxTargetDeviceId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.usTxFreq);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucTxPwr);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucTxBw);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucScs);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.ucMcs);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.usTxRatio);

    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLatitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxLongitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusTx.stTxPosition.nTxAttitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unSeqNum);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unContCnt);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unTxVehicleSpeed);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusTx.unTxVehicleHeading);

    fprintf(sh_pMultiDbMgrRxMsg, "0x%x,", pstMultiEventMsg->pstMultiDbManagerWrite->unCrc32);

    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL1.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL2.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL3.ulTimeStamp);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL1.ulLatency);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL2.ulLatency);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.stDbV2xDevL3.ulLatency);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL1.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL2.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL3.unDevId);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL1.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL2.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL3.usSwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL1.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL2.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.stDbV2xDevL3.usHwVer);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.unRxVehicleSpeed);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.unRxVehicleHeading);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.unTotalCommDevCnt);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.nRssi);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.ucRcpi);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.eRsvLevel);

    pstDi = APP_GetDiInstance();
    if(pstDi == NULL)
    {
        PrintError("pstDi is NULL!");
    }

    /* Set the GPS values */
    nRet = DI_GPS_Get(&pstDi->stDiGps);
    if (nRet != DI_OK)
    {
        PrintError("DI_GPS_Get() is failed! [nRet:%d]", nRet);
    }
    dTemp = (double)stDbV2xStatusRx.stRxPosition.unCommDistance / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusRx.stRxPosition.nRxLatitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusRx.stRxPosition.nRxLongitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    dTemp = (double)stDbV2xStatusRx.stRxPosition.nRxAttitude / SVC_MCP_GPS_VALUE_CONVERT_DOUBLE;
    fprintf(sh_pMultiDbMgrRxMsg, "%lf,", dTemp);
    fprintf(sh_pMultiDbMgrRxMsg, "%d,", stDbV2xStatusRx.ucErrIndicator);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.ulTotalPacketCnt);
    fprintf(sh_pMultiDbMgrRxMsg, "%ld,", stDbV2xStatusRx.ulTotalErrCnt);
    fTemp = (float)stDbV2xStatusRx.unPdr / 1000.0f;
    fprintf(sh_pMultiDbMgrRxMsg, "%.3f,", fTemp);
    fTemp = (float)stDbV2xStatusRx.unPer / 1000.0f;
    fprintf(sh_pMultiDbMgrRxMsg, "%.3f", fTemp);

    fprintf(sh_pMultiDbMgrRxMsg, "\r\n");

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    /* Reset the err indicator */
    stMultiDbV2xStatus.stV2xStatusRx.ucErrIndicator = FALSE;
    nRet = P_MULTI_DB_MANAGER_SetV2xStatus(&stMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    nRet = fflush(sh_pMultiDbMgrRxMsg);
    if (nRet < 0)
    {
        PrintError("fflush() is failed! [unRet:%d]", nRet);
    }

    if(pchPayload != NULL)
    {
        free(pchPayload);
    }

    if(pstMultiEventMsg->pPayload != NULL)
    {
        if(s_bMultiDbMgrLog == ON)
        {
            PrintDebug("free [%p] allocated at P_MULTI_MSG_MANAGER_SendRxMsgToDbMgr()", pstMultiEventMsg->pPayload);
        }
        free(pstMultiEventMsg->pPayload);
    }

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_WriteCsvV2xStatus(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    switch(pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType)
    {
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_TX:
        {
            if (sh_pMultiDbMgrTxMsg != NULL)
            {
                nRet = P_MULTI_DB_MANAGER_WriteCsvV2xStatusTx(pstMultiEventMsg);
                if(nRet != FRAMEWORK_OK)
                {
                    PrintError("P_MULTI_DB_MANAGER_WriteCsvV2xStatusTx() is failed! [unRet:%d]", nRet);
                }
            }
            else
            {
                PrintError("sh_pMultiDbMgrTxMsg is NULL!!, check whethter sh_pDbMgrTxMsg is opened before.");
            }
            break;
        }
        case MULTI_DB_MANAGER_COMM_MSG_TYPE_RX:
        {
            if (sh_pMultiDbMgrRxMsg != NULL)
            {
                nRet = P_MULTI_DB_MANAGER_WriteCsvV2xStatusRx(pstMultiEventMsg);
                if(nRet != FRAMEWORK_OK)
                {
                    PrintError("P_MULTI_DB_MANAGER_WriteCsvV2xStatusRx() is failed! [unRet:%d]", nRet);
                }
            }
            else
            {
                PrintError("sh_pMultiDbMgrRxMsg is NULL!!, check whethter sh_pMultiDbMgrRxMsg is opened before.");
            }
            break;
        }
        default:
            PrintError("unknown eMultiCommMsgType [%d]", pstMultiEventMsg->pstMultiDbManagerWrite->eMultiCommMsgType);
            break;
    }
    nRet = FRAMEWORK_OK;

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_WriteCsv(MULTI_DB_MANAGER_EVENT_MSG_T *pstMultiEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiEventMsg == NULL)
    {
        PrintError("pstMultiEventMsg is NULL!");
        return nRet;
    }

    switch(pstMultiEventMsg->pstDbV2x->ePayloadType)
    {
        case DB_V2X_PAYLOAD_TYPE_V2X_STATUS:
        {
            nRet = P_MULTI_DB_MANAGER_WriteCsvV2xStatus(pstMultiEventMsg);
            if(nRet != FRAMEWORK_OK)
            {
                PrintError("P_MULTI_DB_MANAGER_WriteCsvV2xStatus() is failed! [unRet:%d]", nRet);
            }
            break;
        }
        case DB_V2X_PAYLOAD_TYPE_PLATOONING:
        {
            nRet = P_MULTI_DB_MANAGER_WriteCsvPlatooning(pstMultiEventMsg);
            if(nRet != FRAMEWORK_OK)
            {
                PrintError("P_MULTI_DB_MANAGER_WriteCsvPlatooning() is failed! [unRet:%d]", nRet);
            }
            break;
        }
        case DB_V2X_PAYLOAD_TYPE_PLATOONING_THROUGHPUT:
        {
            nRet = P_MULTI_DB_MANAGER_WriteCsvPlatooningThroughput(pstMultiEventMsg);
            if(nRet != FRAMEWORK_OK)
            {
                PrintError("P_MULTI_DB_MANAGER_WriteCsvPlatooningThroughput() is failed! [unRet:%d]", nRet);
            }
            break;
        }
        default:
            PrintError("unknown ePayloadType [%d]", pstMultiEventMsg->pstDbV2x->ePayloadType);
            break;
    }

    return nRet;
}

static void *P_MULTI_DB_MANAGER_Task(void *arg)
{
    MULTI_DB_MANAGER_EVENT_MSG_T stMultiEventMsg;
    int32_t nRet = FRAMEWORK_ERROR;
    memset(&stMultiEventMsg, 0, sizeof(MULTI_DB_MANAGER_EVENT_MSG_T));

    (void)arg;

    while (1)
    {
        if(msgrcv(s_nMultiDbTaskMsgId, &stMultiEventMsg, sizeof(MULTI_DB_MANAGER_EVENT_MSG_T), 0, MSG_NOERROR) == FRAMEWORK_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            if(stMultiEventMsg.pstMultiDbManagerWrite->eMultiProc == MULTI_DB_MANAGER_PROC_WRITE)
            {
                if (s_bMultiDbMgrLog == ON)
                {
                    PrintDebug("MULTI_DB_MANAGER_PROC_WRITE [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiProc);
                }

                switch(stMultiEventMsg.pstMultiDbManagerWrite->eMultiFileType)
                {
                    case MULTI_DB_MANAGER_FILE_TYPE_TXT:
                    {
                        if (s_bMultiDbMgrLog == ON)
                        {
                            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_TXT [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiFileType);
                        }

                        nRet = P_MULTI_DB_MANAGER_WriteTxt(&stMultiEventMsg);
                        if(nRet != FRAMEWORK_OK)
                        {
                            PrintError("P_MULTI_DB_MANAGER_WriteTxt() is failed! [unRet:%d]", nRet);
                        }
                        break;
                    }
                    case MULTI_DB_MANAGER_FILE_TYPE_CSV:
                    {
                        if (s_bMultiDbMgrLog == ON)
                        {
                            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_CSV [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiFileType);
                        }

                        nRet = P_MULTI_DB_MANAGER_WriteCsv(&stMultiEventMsg);
                        if(nRet != FRAMEWORK_OK)
                        {
                            PrintError("P_MULTI_DB_MANAGER_WriteCsv() is failed! [unRet:%d]", nRet);
                        }
                        break;
                    }
#if defined(CONFIG_SQLITE)
                    case MULTI_DB_MANAGER_FILE_TYPE_SQLITE:
                    {
                        if (s_bMultiDbMgrLog == ON)
                        {
                            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_SQLITE [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiFileType);
                        }

                        nRet = P_MULTI_DB_MANAGER_WriteSqlite(&stMultiEventMsg);
                        if(nRet != FRAMEWORK_OK)
                        {
                            PrintError("P_MULTI_DB_MANAGER_WriteSqlite() is failed! [unRet:%d]", nRet);
                        }
                        break;
                    }
#endif
                    default:
                        PrintWarn("unknown file type [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiFileType);
                        break;
                }
            }
            else if(stMultiEventMsg.pstMultiDbManagerWrite->eMultiProc == MULTI_DB_MANAGER_PROC_READ)
            {
                PrintDebug("MULTI_DB_MANAGER_PROC_READ [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiProc);
                PrintWarn("TODO");

            }
            else if(stMultiEventMsg.pstMultiDbManagerWrite->eMultiProc == MULTI_DB_MANAGER_PROC_CONVERT)
            {
                PrintDebug("MULTI_DB_MANAGER_PROC_CONVERT [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiProc);
                PrintWarn("TODO");
            }
            else
            {
                PrintWarn("unknown processing type [%d]", stMultiEventMsg.pstMultiDbManagerWrite->eMultiProc);
            }
        }
    }

    return NULL;
}

static void P_MULTI_DB_MANAGER_PrintMsgInfo(int msqid)
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

int32_t P_MULTI_DB_MANAGER_CreateTask(void)
{
	int32_t nRet = FRAMEWORK_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_MultiDbMgrTask, &attr, P_MULTI_DB_MANAGER_Task, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_MULTI_DB_MANAGER_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_MULTI_DB_MANAGER_Task() is successfully created.");
        nRet = FRAMEWORK_OK;
    }

#if defined(CONFIG_PTHREAD_JOINABLE)
    nRet = pthread_join(sh_MultiDbMgrTask, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_MULTI_DB_MANAGER_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintDebug("P_MULTI_DB_MANAGER_Task() is successfully joined.");
        nRet = FRAMEWORK_OK;
    }
#endif
    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_Init(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    if((s_nMultiDbTaskMsgId = msgget(s_MultidbTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_MULTI_DB_MANAGER_PrintMsgInfo(s_nMultiDbTaskMsgId);
    }

    if((s_nMultiMsgTxTaskMsgId = msgget(s_MultiMsgTxTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_MULTI_DB_MANAGER_PrintMsgInfo(s_nMultiMsgTxTaskMsgId);
    }

    if((s_nMultiMsgRxTaskMsgId = msgget(s_MultiMsgRxTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_MULTI_DB_MANAGER_PrintMsgInfo(s_nMultiMsgRxTaskMsgId);
    }

    nRet = P_MULTI_DB_MANAGER_CreateTask();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_CreateTask() is failed! [nRet:%d]", nRet);
    }

    memset(&s_stMultiDbV2xStatusRx, 0, sizeof(MULTI_DB_MANAGER_V2X_STATUS_T));
    memset(&s_stMultiDbV2xStatusRx.stV2xStatusRx, 0, sizeof(DB_V2X_STATUS_RX_T));
    memset(&s_stMultiDbV2xStatusRx.stV2xStatusRx.stRxPosition, 0, sizeof(DB_V2X_POSITION_RX_T));

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_DeInit(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    memset(&s_stMultiDbV2xStatusRx, 0, sizeof(MULTI_DB_MANAGER_V2X_STATUS_T));
    memset(&s_stMultiDbV2xStatusRx.stV2xStatusRx, 0, sizeof(DB_V2X_STATUS_RX_T));
    memset(&s_stMultiDbV2xStatusRx.stV2xStatusRx.stRxPosition, 0, sizeof(DB_V2X_POSITION_RX_T));

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_ResetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbV2xStatus == NULL)
    {
        PrintError("pstMultiDbV2xStatus is NULL!!");
    }

    memset(&s_stMultiDbV2xStatusRx, 0, sizeof(MULTI_DB_MANAGER_V2X_STATUS_T));
    memset(&s_stMultiDbV2xStatusRx.stV2xStatusRx, 0, sizeof(DB_V2X_STATUS_RX_T));
    memset(&s_stMultiDbV2xStatusRx.stV2xStatusRx.stRxPosition, 0, sizeof(DB_V2X_POSITION_RX_T));

    memcpy(pstMultiDbV2xStatus, &s_stMultiDbV2xStatusRx, sizeof(MULTI_DB_MANAGER_V2X_STATUS_T));

    nRet = FRAMEWORK_OK;

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_SetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbV2xStatus == NULL)
    {
        PrintError("pstMultiDbV2xStatus is NULL!!");
    }

    memcpy(&s_stMultiDbV2xStatusRx, pstMultiDbV2xStatus, sizeof(MULTI_DB_MANAGER_V2X_STATUS_T));
    nRet = FRAMEWORK_OK;

    return nRet;
}

static int32_t P_MULTI_DB_MANAGER_GetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbV2xStatus == NULL)
    {
        PrintError("pstMultiDbV2xStatus is NULL!!");
    }

    memcpy(pstMultiDbV2xStatus, &s_stMultiDbV2xStatusRx, sizeof(MULTI_DB_MANAGER_V2X_STATUS_T));
    nRet = FRAMEWORK_OK;

    return nRet;
}

int32_t MULTI_DB_MANAGER_SetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbV2xStatus == NULL)
    {
        PrintError("pstMultiDbV2xStatus == NULL!!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_SetV2xStatus(pstMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_SetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_GetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbV2xStatus == NULL)
    {
        PrintError("pstMultiDbV2xStatus == NULL!!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_GetV2xStatus(pstMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_ResetV2xStatus(MULTI_DB_MANAGER_V2X_STATUS_T *pstMultiDbV2xStatus)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbV2xStatus == NULL)
    {
        PrintError("pstMultiDbV2xStatus == NULL!!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_ResetV2xStatus(pstMultiDbV2xStatus);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_ResetV2xStatus() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}


int32_t MULTI_DB_MANAGER_Write(MULTI_DB_MANAGER_WRITE_T *pstMultiDbManagerWrite, DB_V2X_T *pstDbV2x, void *pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;
    MULTI_DB_MANAGER_EVENT_MSG_T stMultiEventMsg;

    if(pstMultiDbManagerWrite == NULL)
    {
        PrintError("pstMultiDbManagerWrite == NULL!!");
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
    if((pstMultiDbManagerWrite->eMultiFileType == MULTI_DB_MANAGER_FILE_TYPE_SQLITE) && (sh_pMultiDbMgrTxSqlMsg == NULL))
    {
        PrintError("sh_pMultiDbMgrTxSqlMsg == NULL!!, check MULTI_DB_MANAGER_Open() is called.");
        return nRet;
    }
#endif

    if((pstMultiDbManagerWrite->eMultiFileType == MULTI_DB_MANAGER_FILE_TYPE_CSV) && (sh_pMultiDbMgrTxMsg == NULL))
    {
        PrintError("sh_pMultiDbMgrTxMsg == NULL!!, check MULTI_DB_MANAGER_Open() is called.");
        return nRet;
    }

    stMultiEventMsg.pstMultiDbManagerWrite = pstMultiDbManagerWrite;
    stMultiEventMsg.pstDbV2x = pstDbV2x;
    stMultiEventMsg.pPayload = pPayload;

    if(msgsnd(s_nMultiDbTaskMsgId, &stMultiEventMsg, sizeof(MULTI_DB_MANAGER_EVENT_MSG_T), IPC_NOWAIT) == FRAMEWORK_MSG_ERR)
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

int32_t MULTI_DB_MANAGER_Read(MULTI_DB_MANAGER_READ_T *pstMultiDbManagerRead, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMultiDbManagerRead == NULL)
    {
        PrintError("pstMultiDbManagerRead == NULL!!");
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
    if(sh_pMultiDbMgrTxSqlMsg == NULL)
    {
        PrintError("sh_pMultiDbMgrTxSqlMsg == NULL!!, check MULTI_DB_MANAGER_Open() is called.");
        return nRet;
    }
#endif

    if(sh_pMultiDbMgrTxMsg == NULL)
    {
        PrintError("sh_pMultiDbMgrTxMsg == NULL!!, check MULTI_DB_MANAGER_Open() is called.");
        return nRet;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_Converter(MULTI_DB_MANAGER_READ_T *pstMultiDbManagerRead, MULTI_DB_MANAGER_WRITE_T *pstMultiDbManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMultiDbManagerRead == NULL)
    {
        PrintError("pstMultiDbManagerRead == NULL!!");
        return nRet;
    }

    if(pstMultiDbManagerWrite == NULL)
    {
        PrintError("pstMultiDbManagerWrite == NULL!!");
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

int32_t MULTI_DB_MANAGER_SetLog(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    s_bMultiDbMgrLog = pstMultiDbManager->bLogLevel;
    PrintTrace("SET:s_bMultiDbMgrLog [%s]", s_bMultiDbMgrLog == ON ? "ON" : "OFF");

    nRet = FRAMEWORK_OK;

    return nRet;
}

int32_t MULTI_DB_MANAGER_Open(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    switch(pstMultiDbManager->eMultiFileType)
    {
        case MULTI_DB_MANAGER_FILE_TYPE_TXT:
            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_TXT [%d]", pstMultiDbManager->eMultiFileType);

            if(sh_pMultiDbMgrTxMsg == NULL)
            {
                sh_pMultiDbMgrTxMsg = fopen(MULTI_DB_MANAGER_TXT_TX_FILE, "a+");
                if(sh_pMultiDbMgrTxMsg == NULL)
                {
                    PrintError("fopen() is failed!!");
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_TX_FILE[%s] is opened.", MULTI_DB_MANAGER_TXT_TX_FILE);
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pMultiDbMgrRxMsg == NULL)
            {
                sh_pMultiDbMgrRxMsg = fopen(MULTI_DB_MANAGER_TXT_RX_FILE, "a+");
                if(sh_pMultiDbMgrRxMsg == NULL)
                {
                    PrintError("fopen() is failed!!");
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_RX_FILE[%s] is opened.", MULTI_DB_MANAGER_TXT_RX_FILE);
                    nRet = FRAMEWORK_OK;
                }
            }

            break;

        case MULTI_DB_MANAGER_FILE_TYPE_CSV:
            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_CSV [%d]", pstMultiDbManager->eMultiFileType);

            nRet = P_MULTI_DB_MANAGER_OpenCsv(pstMultiDbManager);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("P_MULTI_DB_MANAGER_OpenCsv() is failed! [nRet:%d]", nRet);
                return nRet;
            }
            break;

#if defined(CONFIG_SQLITE)
        case MULTI_DB_MANAGER_FILE_TYPE_SQLITE:
            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_SQLITE [%d]", pstMultiDbManager->eMultiFileType);

            nRet = P_MULTI_DB_MANAGER_OpenSqlite(pstMultiDbManager);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("P_MULTI_DB_MANAGER_OpenSqlite() is failed! [nRet:%d]", nRet);
                return nRet;
            }
            break;
#endif

        default:
            PrintWarn("unknown file type [%d]", pstMultiDbManager->eMultiFileType);
            break;

    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_MakeDbFile(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char chFileName[MULTI_DB_MGR_FILE_MAX_LENGTH];
    char chSysCallStr[MULTI_DB_MGR_SYSTEM_CALL_MAX_LENGTH];
    int nCharCnt = 0;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    sprintf(chSysCallStr, "mkdir -p %s", MULTI_DB_V2X_FOLDER_DIR);
    nRet = system(chSysCallStr);
    if(nRet < FRAMEWORK_OK)
    {
        PrintError("system() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nCharCnt = sprintf(chFileName, "%s_", pstMultiDbManager->stMultiDbFile.pchTxRxType);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchDeviceType);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchDeviceId);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchStartTime);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchEndTime);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s.", pstMultiDbManager->stMultiDbFile.pchTotalTime);

    switch(pstMultiDbManager->eMultiFileType)
    {
        case MULTI_DB_MANAGER_FILE_TYPE_TXT:
            nCharCnt += sprintf(chFileName+nCharCnt, "%s", "txt");
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "cp %s %s/%s", MULTI_DB_MANAGER_TXT_TX_FILE, MULTI_DB_V2X_FOLDER_DIR, chFileName);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "cp %s %s/%s", MULTI_DB_MANAGER_TXT_RX_FILE, MULTI_DB_V2X_FOLDER_DIR, chFileName);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        case MULTI_DB_MANAGER_FILE_TYPE_CSV:
            nCharCnt += sprintf(chFileName+nCharCnt, "%s", "csv");
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "cp %s %s/%s", MULTI_DB_MANAGER_CSV_TX_FILE, MULTI_DB_V2X_FOLDER_DIR, chFileName);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "cp %s %s/%s", MULTI_DB_MANAGER_CSV_RX_FILE, MULTI_DB_V2X_FOLDER_DIR, chFileName);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        case MULTI_DB_MANAGER_FILE_TYPE_SQLITE:
            nCharCnt += sprintf(chFileName+nCharCnt, "%s", "db");
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "cp %s %s/%s", MULTI_DB_MANAGER_SQL_TX_FILE, MULTI_DB_V2X_FOLDER_DIR, chFileName);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "cp %s %s/%s", MULTI_DB_MANAGER_SQL_RX_FILE, MULTI_DB_V2X_FOLDER_DIR, chFileName);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        default:
            PrintError("unknown file type [%d]", pstMultiDbManager->eMultiFileType);
            break;
    }

    PrintTrace("[nCharCnt:%d][%s/%s] is successfully made!", nCharCnt, MULTI_DB_V2X_FOLDER_DIR, chFileName);
    PrintDebug("%s", chSysCallStr);

    nRet = system(chSysCallStr);
    if(nRet < FRAMEWORK_OK)
    {
        PrintError("system() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_UploadFile(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char chFileName[MULTI_DB_MGR_FILE_MAX_LENGTH];
    char chSysCallStr[MULTI_DB_MGR_SYSTEM_CALL_MAX_LENGTH];
    int nCharCnt = 0, nCharCmdCnt;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    nCharCnt = sprintf(chFileName, "%s_", pstMultiDbManager->stMultiDbFile.pchTxRxType);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchDeviceType);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchDeviceId);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchStartTime);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s_", pstMultiDbManager->stMultiDbFile.pchEndTime);
    nCharCnt += sprintf(chFileName+nCharCnt, "%s.", pstMultiDbManager->stMultiDbFile.pchTotalTime);

    switch(pstMultiDbManager->eMultiFileType)
    {
        case MULTI_DB_MANAGER_FILE_TYPE_TXT:
            nCharCnt += sprintf(chFileName+nCharCnt, "%s", "txt");
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                nCharCmdCnt = sprintf(chSysCallStr, "scp -P %d %s/%s keti@%s:%s", MULTI_DB_V2X_PORT, DB_V2X_FOLDER_DIR, chFileName, MULTI_DB_V2X_IP, DB_V2X_STORAGE);
                sprintf(chSysCallStr+nCharCmdCnt, "/%s/%s", MULTI_DB_V2X_YEAR, MULTI_DB_V2X_MONTH);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                nCharCmdCnt = sprintf(chSysCallStr, "scp -P %d %s/%s keti@%s:%s", MULTI_DB_V2X_PORT, MULTI_DB_V2X_FOLDER_DIR, chFileName, MULTI_DB_V2X_IP, MULTI_DB_V2X_STORAGE);
                sprintf(chSysCallStr+nCharCmdCnt, "/%s/%s", MULTI_DB_V2X_YEAR, MULTI_DB_V2X_MONTH);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        case MULTI_DB_MANAGER_FILE_TYPE_CSV:
            nCharCnt += sprintf(chFileName+nCharCnt, "%s", "csv");
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                nCharCmdCnt = sprintf(chSysCallStr, "scp -P %d %s/%s keti@%s:%s", MULTI_DB_V2X_PORT, MULTI_DB_V2X_FOLDER_DIR, chFileName, MULTI_DB_V2X_IP, MULTI_DB_V2X_STORAGE);
                sprintf(chSysCallStr+nCharCmdCnt, "/%s/%s", MULTI_DB_V2X_YEAR, MULTI_DB_V2X_MONTH);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                nCharCmdCnt = sprintf(chSysCallStr, "scp -P %d %s/%s keti@%s:%s", MULTI_DB_V2X_PORT, MULTI_DB_V2X_FOLDER_DIR, chFileName, MULTI_DB_V2X_IP, MULTI_DB_V2X_STORAGE);
                sprintf(chSysCallStr+nCharCmdCnt, "/%s/%s", MULTI_DB_V2X_YEAR, MULTI_DB_V2X_MONTH);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        case MULTI_DB_MANAGER_FILE_TYPE_SQLITE:
            nCharCnt += sprintf(chFileName+nCharCnt, "%s", "db");
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                nCharCmdCnt = sprintf(chSysCallStr, "scp -P %d %s/%s keti@%s:%s", MULTI_DB_V2X_PORT, MULTI_DB_V2X_FOLDER_DIR, chFileName, MULTI_DB_V2X_IP, MULTI_DB_V2X_STORAGE);
                sprintf(chSysCallStr+nCharCmdCnt, "/%s/%s", MULTI_DB_V2X_YEAR, MULTI_DB_V2X_MONTH);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                nCharCmdCnt = sprintf(chSysCallStr, "scp -P %d %s/%s keti@%s:%s", MULTI_DB_V2X_PORT, MULTI_DB_V2X_FOLDER_DIR, chFileName, MULTI_DB_V2X_IP, MULTI_DB_V2X_STORAGE);
                sprintf(chSysCallStr+nCharCmdCnt, "/%s/%s", MULTI_DB_V2X_YEAR, MULTI_DB_V2X_MONTH);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        default:
            PrintError("unknown file type [%d]", pstMultiDbManager->eMultiFileType);
            break;
    }

    PrintTrace("[scp:%s][%s/%s] is successfully send!", MULTI_DB_V2X_IP, MULTI_DB_V2X_FOLDER_DIR, chFileName);
    PrintDebug("%s", chSysCallStr);

    nRet = system(chSysCallStr);
    if(nRet < FRAMEWORK_OK)
    {
        PrintError("system() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_Close(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    switch(pstMultiDbManager->eMultiFileType)
    {
        case MULTI_DB_MANAGER_FILE_TYPE_TXT:
            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_TXT [%d]", pstMultiDbManager->eMultiFileType);

            if(sh_pMultiDbMgrTxMsg != NULL)
            {
                nRet = fflush(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_TX_FILE[%s] is closed.", MULTI_DB_MANAGER_TXT_TX_FILE);
                    sh_pMultiDbMgrTxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pMultiDbMgrRxMsg != NULL)
            {
                nRet = fflush(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_RX_FILE[%s] is closed.", MULTI_DB_MANAGER_TXT_RX_FILE);
                    sh_pMultiDbMgrRxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }
            break;

        case MULTI_DB_MANAGER_FILE_TYPE_CSV:
            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_CSV [%d]", pstMultiDbManager->eMultiFileType);
            if(sh_pMultiDbMgrTxMsg != NULL)
            {
                nRet = fflush(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_TX_FILE[%s] is closed.", MULTI_DB_MANAGER_CSV_TX_FILE);
                    sh_pMultiDbMgrTxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pMultiDbMgrRxMsg != NULL)
            {
                nRet = fflush(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_RX_FILE[%s] is closed.", MULTI_DB_MANAGER_CSV_RX_FILE);
                    sh_pMultiDbMgrRxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }
            break;

#if defined(CONFIG_SQLITE)
        case MULTI_DB_MANAGER_FILE_TYPE_SQLITE:
            PrintDebug("MULTI_DB_MANAGER_FILE_TYPE_SQLITE [%d]", pstMultiDbManager->eMultiFileType);
            if(sh_pMultiDbMgrTxMsg != NULL)
            {
                nRet = fflush(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pMultiDbMgrTxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_TX_FILE[%s] is closed.", MULTI_DB_MANAGER_SQL_TX_FILE);
                    sh_pMultiDbMgrTxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }

            if(sh_pMultiDbMgrRxMsg != NULL)
            {
                nRet = fflush(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fflush() is failed! [unRet:%d]", nRet);
                }

                nRet = fclose(sh_pMultiDbMgrRxMsg);
                if (nRet < 0)
                {
                    PrintError("fclose() is failed! [unRet:%d]", nRet);
                }
                else
                {
                    PrintTrace("MULTI_DB_MANAGER_RX_FILE[%s] is closed.", MULTI_DB_MANAGER_SQL_RX_FILE);
                    sh_pMultiDbMgrRxMsg = NULL;
                    nRet = FRAMEWORK_OK;
                }
            }
            break;
#endif

        default:
            PrintWarn("unknown file type [%d]", pstMultiDbManager->eMultiFileType);
            break;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_RemoveTempFile(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    char chSysCallStr[MULTI_DB_MGR_SYSTEM_CALL_MAX_LENGTH];

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    switch(pstMultiDbManager->eMultiFileType)
    {
        case MULTI_DB_MANAGER_FILE_TYPE_TXT:
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "rm -rf %s", MULTI_DB_MANAGER_TXT_TX_FILE);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "rm -rf %s", MULTI_DB_MANAGER_TXT_RX_FILE);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        case MULTI_DB_MANAGER_FILE_TYPE_CSV:
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "rm -rf %s", MULTI_DB_MANAGER_CSV_TX_FILE);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "rm -rf %s", MULTI_DB_MANAGER_CSV_RX_FILE);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        case MULTI_DB_MANAGER_FILE_TYPE_SQLITE:
            if(strcmp("Tx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "rm -rf %s", MULTI_DB_MANAGER_SQL_TX_FILE);
            }
            else if(strcmp("Rx", pstMultiDbManager->stMultiDbFile.pchTxRxType) == 0)
            {
                sprintf(chSysCallStr, "rm -rf %s", MULTI_DB_MANAGER_SQL_RX_FILE);
            }
            else
            {
                PrintError("unknown type [%s]", pstMultiDbManager->stMultiDbFile.pchTxRxType);
                return nRet;
            }
            break;

        default:
            PrintError("unknown file type [%d]", pstMultiDbManager->eMultiFileType);
            break;
    }

    PrintDebug("%s", chSysCallStr);

    nRet = system(chSysCallStr);
    if(nRet < FRAMEWORK_OK)
    {
        PrintError("system() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_Start(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_Stop(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_Status(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MULTI_DB_MANAGER_Init(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_Init(pstMultiDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    s_bMultiDbMgrLog = pstMultiDbManager->bLogLevel;
    PrintDebug("s_bMultiDbMgrLog [%s]", s_bMultiDbMgrLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t MULTI_DB_MANAGER_DeInit(MULTI_DB_MANAGER_T *pstMultiDbManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMultiDbManager == NULL)
    {
        PrintError("pstMultiDbManager == NULL!!");
        return nRet;
    }

    nRet = P_MULTI_DB_MANAGER_DeInit(pstMultiDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("P_MULTI_DB_MANAGER_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

