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
* @file svc_mcp.c
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
#include "svc_mcp.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include "app.h"
#include "di.h"

/***************************** Definition ************************************/
#define SVC_MCP_GPS_SPEED_CAL_CNT_MAX    (10)
#define SVC_MCP_SET_BUF_SIZE             (256)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
FILE* sh_pSvcMCpTxMsg;
FILE* sh_pSvcMCpRxMsg;

static int s_nSvcMCpTaskMsgId;

static key_t s_SvcMCpTaskMsgKey = SVC_MCP_TASK_MSG_KEY;

static pthread_t sh_SvcMCpTask;
static pthread_t sh_SvcMCpTaskTx;

static SVC_MCP_T s_stSvcMCp;
static DB_MANAGER_V2X_STATUS_T s_stDbV2xStatus;
static bool s_bFirstMsg = TRUE;

static uint16_t s_usGpsSpeedCalCnt = 0;
static uint32_t s_usLastSpeedTx;

static char s_chStrBufTxRxType[SVC_MCP_STR_BUF_LEN];
static char s_chStrBufDevType[SVC_MCP_STR_BUF_LEN];
static char s_chStrBufDevId[SVC_MCP_STR_BUF_LEN];
static char s_chStrBufStartTime[SVC_MCP_STR_BUF_LEN];
static char s_chStrBufEndTime[SVC_MCP_STR_BUF_LEN];
static char s_chStrBufTotalTime[SVC_MCP_STR_BUF_LEN];

static char s_chDeviceName[SVC_MCP_SET_BUF_SIZE] = DB_MGR_DEFAULT_COMM_DEV_ID;
static char s_chIfaceName[SVC_MCP_SET_BUF_SIZE] = SVC_MCP_DEFAULT_ETH_DEV;
static char s_chIpAddr[SVC_MCP_SET_BUF_SIZE] = SVC_MCP_DEFAULT_IP;

/***************************** Function  *************************************/

int32_t P_SVC_MCP_SetSettings(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp is NULL!!");
    }

    memcpy(&s_stSvcMCp, pstSvcMCp, sizeof(SVC_MCP_T));
    nRet = APP_OK;

    return nRet;
}

int32_t P_SVC_MCP_GetSettings(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp is NULL!!");
    }

    memcpy(pstSvcMCp, &s_stSvcMCp, sizeof(SVC_MCP_T));
    nRet = APP_OK;

    return nRet;
}

int32_t SVC_MCP_UpdateSettings(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;
    char chModelNameFile[MAX_MODEL_NAME_LEN] = {0};
    FILE *h_fdModelConf;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    snprintf(chModelNameFile, sizeof(chModelNameFile), "%s%s", CONFIG_MODEL_NAME, MODEL_NAME_FILE_SUFFIX);

    char chBuf[SVC_MCP_SET_BUF_SIZE] = {0};

    h_fdModelConf = fopen(chModelNameFile, "r");
    if (h_fdModelConf != NULL)
    {
        char chLine[MAX_MODEL_NAME_LEN] = {0};

        while (fgets(chLine, sizeof(chLine), h_fdModelConf) != NULL)
        {
            size_t len = strcspn(chLine, "\n"); // \n 위치 찾기
            chLine[len] = '\0'; // \n을 null-terminator로 대체

            if (strncmp(chLine, MODEL_PREFIX, MODEL_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + MODEL_PREFIX_LEN, sizeof(chBuf) - 1);
                PrintTrace("Model Name: %s", chBuf);
            }
            else if (strncmp(chLine, DEVICE_NAME_PREFIX, DEVICE_NAME_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + DEVICE_NAME_PREFIX_LEN, sizeof(chBuf) - 1);
                strncpy(s_chDeviceName, chBuf, SVC_MCP_SET_BUF_SIZE); // Static buffer 사용
                pstSvcMCp->pchDeviceName = s_chDeviceName;
                PrintTrace("Updated Device Name: %s", pstSvcMCp->pchDeviceName);
            }
            else if (strncmp(chLine, DEVICE_ID_PREFIX, DEVICE_ID_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + DEVICE_ID_PREFIX_LEN, sizeof(chBuf) - 1);
                pstSvcMCp->stDbV2x.unDeviceId = (uint32_t)atoi(chBuf);
                PrintTrace("Updated Device ID: %u", pstSvcMCp->stDbV2x.unDeviceId);
            }
            else if (strncmp(chLine, IFACE_NAME_PREFIX, IFACE_NAME_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + IFACE_NAME_PREFIX_LEN, sizeof(chBuf) - 1);
                strncpy(s_chIfaceName, chBuf, SVC_MCP_SET_BUF_SIZE); // Static buffer 사용
                pstSvcMCp->pchIfaceName = s_chIfaceName;
                PrintTrace("Updated Interface Name: %s", pstSvcMCp->pchIfaceName);
            }
            else if (strncmp(chLine, IP_ADDR_PREFIX, IP_ADDR_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + IP_ADDR_PREFIX_LEN, sizeof(chBuf) - 1);
                strncpy(s_chIpAddr, chBuf, SVC_MCP_SET_BUF_SIZE); // Static buffer 사용
                pstSvcMCp->pchIpAddr = s_chIpAddr;
                PrintTrace("Updated IP Address: %s", pstSvcMCp->pchIpAddr);
            }
            else if (strncmp(chLine, PORT_PREFIX, PORT_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + PORT_PREFIX_LEN, sizeof(chBuf) - 1);
                pstSvcMCp->unPort = (uint32_t)atoi(chBuf);
                PrintTrace("Updated Port: %d", pstSvcMCp->unPort);
            }
        }

        fclose(h_fdModelConf);
    }
    else
    {
        PrintWarn("chModelNameFile[%s] is not exist!", chModelNameFile);
    }

    nRet = APP_OK;

    PrintDebug("SVC_MCP_UpdateSettings() set is finished.[eth:%s,ip:%s,port:%d]", pstSvcMCp->pchIfaceName, pstSvcMCp->pchIpAddr, pstSvcMCp->unPort);

    return nRet;
}


int32_t P_SVC_MCP_SetDefaultSettings(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    pstSvcMCp->stDbManagerWrite.eFileType = DB_MANAGER_FILE_TYPE_CSV;
    pstSvcMCp->stDbManagerWrite.eCommMsgType = DB_MANAGER_COMM_MSG_TYPE_TX;
    pstSvcMCp->stDbManagerWrite.eProc = DB_MANAGER_PROC_WRITE;

    pstSvcMCp->stMsgManagerTx.ePayloadType = eMSG_MANAGER_PAYLOAD_TYPE_RAW;
    pstSvcMCp->stMsgManagerTx.eCommType = eMSG_MANAGER_COMM_TYPE_5GNRV2X;
    pstSvcMCp->stMsgManagerTx.eSignId = eMSG_MANAGER_SIGN_ID_UNSECURED;
    pstSvcMCp->stMsgManagerTx.eV2xFreq = eMSG_MANAGER_V2X_FREQ_5900;
    pstSvcMCp->stMsgManagerTx.ePriority = eMSG_MANAGER_PRIORITY_CV2X_PPPP_0;
    pstSvcMCp->stMsgManagerTx.eV2xDataRate = eMSG_MANAGER_V2X_DATA_RATE_6MBPS;
    pstSvcMCp->stMsgManagerTx.eV2xTimeSlot = eMSG_MANAGER_V2X_TIME_SLOT_CONTINUOUS;
    pstSvcMCp->stMsgManagerTx.unPsid = DB_V2X_PSID;
    pstSvcMCp->stMsgManagerTx.nTxPower = MSG_MANAGER_V2X_TX_POWER;
    pstSvcMCp->stMsgManagerTx.unTxCount = MSG_MANAGER_V2X_TX_COUNT;
    pstSvcMCp->stMsgManagerTx.unTxDelay = MSG_MANAGER_V2X_TX_DELAY;

    for(int i = 0; i < MSG_MANAGER_MAC_LENGTH; i++)
    {
        pstSvcMCp->stMsgManagerTx.uchPeerMacAddr[i] = 0xFF;
    }

    pstSvcMCp->stMsgManagerTx.unTransmitterProfileId = MSG_MANAGER_V2X_TX_PROFILE_ID;
    pstSvcMCp->stMsgManagerTx.unPeerL2Id = MSG_MANAGER_V2X_TX_PEER_L2_ID;

#if defined(CONFIG_OBU)
    pstSvcMCp->stDbV2x.eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
    PrintTrace("CONFIG_OBU is enabled, eDeviceType [%d]", pstSvcMCp->stDbV2x.eDeviceType);

    pstSvcMCp->pchIpAddr = SVC_MCP_DEFAULT_IP;
    pstSvcMCp->unPort = SVC_MCP_DEFAULT_PORT;
    pstSvcMCp->pchIfaceName = SVC_MCP_DEFAULT_ETH_DEV;
    pstSvcMCp->unPsid = SVC_MCP_V2V_PSID;
    pstSvcMCp->pchDeviceName = DB_MGR_DEFAULT_COMM_DEV_ID;
#elif defined(CONFIG_RSU)
    pstSvcMCp->stDbV2x.eDeviceType = DB_V2X_DEVICE_TYPE_RSU;
    PrintTrace("CONFIG_RSU is enabled, eDeviceType [%d]", pstSvcMCp->stDbV2x.eDeviceType);

    pstSvcMCp->pchIpAddr = SVC_MCP_DEFAULT_RSU_IP;
    pstSvcMCp->unPort = SVC_MCP_DEFAULT_RSU_PORT;
    pstSvcMCp->pchIfaceName = SVC_MCP_DEFAULT_RSU_ETH_DEV;
    pstSvcMCp->unPsid = SVC_MCP_I2V_PSID;
    pstSvcMCp->pchDeviceName = DB_MGR_DEFAULT_COMM_RSU_DEV_ID;
#else
    PrintError("check device type!!");
    return APP_ERROR;
#endif

    pstSvcMCp->stDbV2x.eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5_BROADCAST;
    pstSvcMCp->stDbV2x.unDeviceId = CLI_DB_V2X_DEFAULT_DEVICE_ID;
    pstSvcMCp->stDbV2x.eServiceId = DB_V2X_SERVICE_ID_PLATOONING;
    pstSvcMCp->stDbV2x.eActionType = DB_V2X_ACTION_TYPE_REQUEST;
    pstSvcMCp->stDbV2x.eRegionId = DB_V2X_REGION_ID_SEONGNAM;
    pstSvcMCp->stDbV2x.ePayloadType = DB_V2X_PAYLOAD_TYPE_V2X_STATUS;
    pstSvcMCp->stDbV2x.eCommId = DB_V2X_COMM_ID_V2V;
    pstSvcMCp->stDbV2x.usDbVer = (DB_V2X_VERSION_MAJOR << CLI_DB_V2X_MAJOR_SHIFT) | DB_V2X_VERSION_MINOR;
    pstSvcMCp->stDbV2x.usHwVer = CLI_DB_V2X_DEFAULT_HW_VER;
    pstSvcMCp->stDbV2x.usSwVer = CLI_DB_V2X_DEFAULT_SW_VER;


    pstSvcMCp->stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp = 0;
    pstSvcMCp->stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp = 0;
    pstSvcMCp->stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp = 0;
    pstSvcMCp->stDbV2xStatusTx.unRxTargetDeviceId = 0;
    pstSvcMCp->stDbV2xStatusTx.usTxFreq = MSG_MANAGER_V2X_TX_FREQ;
    pstSvcMCp->stDbV2xStatusTx.ucTxPwr = MSG_MANAGER_V2X_TX_POWER;
    pstSvcMCp->stDbV2xStatusTx.ucTxBw = MSG_MANAGER_V2X_TX_BW;
    pstSvcMCp->stDbV2xStatusTx.ucScs = 0;
    pstSvcMCp->stDbV2xStatusTx.ucMcs = 0;

    pstSvcMCp->stDbV2xStatusTx.usTxRatio = pstSvcMCp->stMsgManagerTx.unTxDelay;
    pstSvcMCp->stDbV2xStatusTx.stTxPosition.nTxLatitude = 0;
    pstSvcMCp->stDbV2xStatusTx.stTxPosition.nTxLongitude = 0;
    pstSvcMCp->stDbV2xStatusTx.stTxPosition.nTxAttitude = 0;

    pstSvcMCp->stDbV2xStatusTx.unSeqNum = 1;
    pstSvcMCp->stDbV2xStatusTx.unContCnt = 1;
    pstSvcMCp->stDbV2xStatusTx.unTxVehicleSpeed = DB_MGR_DEFAULT_VEHICLE_SPEED;
    pstSvcMCp->stDbV2xStatusTx.unTxVehicleHeading = 0;

    pstSvcMCp->ulDbStartTime = 0;
    pstSvcMCp->ulDbEndTime = 0;
    pstSvcMCp->unDbTotalWrittenTime = 0;

    nRet = APP_OK;

    PrintDebug("P_SVC_MCP_SetDefaultSettings() set is finished.[eth:%s,ip:%s,port:%d]", pstSvcMCp->pchIfaceName, pstSvcMCp->pchIpAddr, pstSvcMCp->unPort);

    return nRet;
}

static int32_t P_SVC_MCP_Start(SVC_MCP_EVENT_MSG_T *stEventMsg)
{
    int32_t nRet = APP_ERROR;
    TIME_MANAGER_T *pstTimeManager;
    UNUSED(stEventMsg);

    if ((s_stSvcMCp.eSvcMCpStatus == SVC_MCP_STATUS_STOP) || (s_stSvcMCp.eSvcMCpStatus == SVC_MCP_STATUS_IDLE))
    {
        s_stSvcMCp.eSvcMCpStatus = SVC_MCP_STATUS_START;

        pstTimeManager = FRAMEWORK_GetTimeManagerInstance();
        if(pstTimeManager == NULL)
        {
            PrintError("pstTimeManager is NULL!");
        }

        nRet = TIME_MANAGER_Get(pstTimeManager);
        if(nRet != FRAMEWORK_OK)
        {
            PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nRet);
        }
        else
        {
            PrintDebug("pstTimeManager->ulTimeStamp[%ld]", pstTimeManager->ulTimeStamp);
            s_stSvcMCp.ulDbStartTime = pstTimeManager->ulTimeStamp;
        }

        nRet = TIME_MANAGER_SetDbTxBegin(pstTimeManager);
        if(nRet != FRAMEWORK_OK)
        {
            PrintError("TIME_MANAGER_SetDbTxBegin() is failed! [nRet:%d]", nRet);
        }

        PrintTrace("eSvcMCpStatus[%d] STARTs NOW [Time:%ld]", s_stSvcMCp.eSvcMCpStatus, s_stSvcMCp.ulDbStartTime);

        if(s_stSvcMCp.eSvcMCpStatus == SVC_MCP_STATUS_START)
        {
            nRet = APP_OK;
        }
    }
    else
    {
        PrintWarn("unknown status type");
    }

    return nRet;
}

static int32_t P_SVC_MCP_Stop(SVC_MCP_EVENT_MSG_T *stEventMsg)
{
    int32_t nRet = APP_ERROR;
    TIME_MANAGER_T *pstTimeManager;
    UNUSED(stEventMsg);

    if(s_stSvcMCp.eSvcMCpStatus == SVC_MCP_STATUS_START)
    {
        s_stSvcMCp.eSvcMCpStatus = SVC_MCP_STATUS_STOP;

        pstTimeManager = FRAMEWORK_GetTimeManagerInstance();
        if(pstTimeManager == NULL)
        {
            PrintError("pstTimeManager is NULL!");
        }

        nRet = TIME_MANAGER_Get(pstTimeManager);
        if(nRet != FRAMEWORK_OK)
        {
            PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nRet);
        }
        else
        {
            PrintDebug("pstTimeManager->ulTimeStamp[%ld]", pstTimeManager->ulTimeStamp);
            s_stSvcMCp.ulDbEndTime = pstTimeManager->ulTimeStamp;
        }

        nRet = TIME_MANAGER_SetDbTxEnd(pstTimeManager);
        if(nRet != FRAMEWORK_OK)
        {
            PrintError("TIME_MANAGER_SetDbTxBegin() is failed! [nRet:%d]", nRet);
        }

        s_stSvcMCp.unDbTotalWrittenTime = pstTimeManager->unDbTxTotalTime;

        PrintTrace("eSvcMCpStatus[%d] STOPs NOW [Time:%ld], unDbTotalWrittenTime[%d seconds]", s_stSvcMCp.eSvcMCpStatus, s_stSvcMCp.ulDbEndTime, s_stSvcMCp.unDbTotalWrittenTime);

        if(s_stSvcMCp.eSvcMCpStatus == SVC_MCP_STATUS_STOP)
        {
            nRet = APP_OK;
        }
    }
    else
    {
        PrintWarn("unknown status type");
    }

    return nRet;

}

int32_t P_SVC_MCP_RestartDb(void)
{
    int32_t nRet = APP_ERROR;
    DB_MANAGER_T *pstDbManager;

    PrintWarn("Finish to write DB Files, start time[%ld], end time[%ld], total written time[%d]", s_stSvcMCp.ulDbStartTime, s_stSvcMCp.ulDbEndTime, s_stSvcMCp.unDbTotalWrittenTime);

    pstDbManager = FRAMEWORK_GetDbManagerInstance();
    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager is NULL!");
    }

    PrintDebug("Close the default temp DB file");
    nRet = DB_MANAGER_Close(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Close() is failed! [nRet:%d]", nRet);
    }

//    PrintWarn("Copy the default DB file as [%s]", );

    PrintDebug("Reopen the default temp DB file");
    pstDbManager->eFileType = s_stSvcMCp.stDbManagerWrite.eFileType;
    pstDbManager->eSvcType = DB_MANAGER_SVC_TYPE_V2X_STATUS;
    nRet = DB_MANAGER_Open(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Open() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("Restart to write the DB file");

    return nRet;
}

static void *P_SVC_MCP_TaskTx(void *arg)
{
    UNUSED(arg);
    TIME_MANAGER_T *pstTimeManager;
    char *pchPayload = NULL;
    int32_t nFrameWorkRet = FRAMEWORK_ERROR;
    bool bMsgTx = TRUE;
    DI_T *pstDi;
    int32_t nRet = APP_ERROR;
    uint32_t nCurrSpeed = 0;
    double dHeading;

    while (1)
    {
        if(s_stSvcMCp.eSvcMCpStatus == SVC_MCP_STATUS_START)
        {
            s_stSvcMCp.stDbV2x.ulPayloadLength = sizeof(s_stSvcMCp.stDbV2xStatusTx);

            pchPayload = (char*)malloc(sizeof(char)*s_stSvcMCp.stDbV2x.ulPayloadLength);
            if(pchPayload == NULL)
            {
                PrintError("malloc() is failed! [NULL]");
                break;
            }

            (void*)memset(pchPayload, 0x00, sizeof(sizeof(char)*s_stSvcMCp.stDbV2x.ulPayloadLength));

            pstTimeManager = FRAMEWORK_GetTimeManagerInstance();
            if(pstTimeManager == NULL)
            {
                PrintError("pstTimeManager is NULL!");
            }

            /* Set at the Rx Device by Using Ext Msg */
            s_stSvcMCp.stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp = 0;
            s_stSvcMCp.stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp = 0;

            nFrameWorkRet = TIME_MANAGER_Get(pstTimeManager);
            if(nFrameWorkRet != FRAMEWORK_OK)
            {
                PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nFrameWorkRet);
            }
            else
            {
                s_stSvcMCp.stDbV2x.ulTimeStamp = pstTimeManager->ulTimeStamp;

                /* Set the application timestamp before sending */
                s_stSvcMCp.stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp = pstTimeManager->ulTimeStamp;
            }

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

            s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLatitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fLatitude * SVC_MCP_GPS_VALUE_CONVERT);
            s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLongitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fLongitude * SVC_MCP_GPS_VALUE_CONVERT);
            s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxAttitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fAltitude * SVC_MCP_GPS_VALUE_CONVERT);

            memcpy(pchPayload, (char*)&s_stSvcMCp.stDbV2xStatusTx, sizeof(char)*s_stSvcMCp.stDbV2x.ulPayloadLength);

            if(s_bFirstMsg == TRUE)
            {
                s_stSvcMCp.stDbV2xStatusTx.unTxVehicleSpeed = DB_MGR_DEFAULT_VEHICLE_SPEED;
                s_bFirstMsg = FALSE;
                PrintWarn("stDbV2xStatus.bFirstPacket's speed is the default value [%d]", s_stSvcMCp.stDbV2xStatusTx.unTxVehicleSpeed);
            }
            else
            {
                if(s_usGpsSpeedCalCnt == SVC_MCP_GPS_SPEED_CAL_CNT_MAX)
                {
                    s_stDbV2xStatus.stV2xGpsInfoTx.nLatitudeNow = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
                    s_stDbV2xStatus.stV2xGpsInfoTx.nLongitudeNow = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
                    s_stDbV2xStatus.stV2xGpsInfoTx.ulTimeStampNow = pstTimeManager->ulTimeStamp;

                    nCurrSpeed = DI_GPS_CalculateSpeed(&s_stDbV2xStatus.stV2xGpsInfoTx);
                    if(nCurrSpeed == 0)
                    {
                        s_stSvcMCp.stDbV2xStatusTx.unTxVehicleSpeed = s_usLastSpeedTx;
                    }
                    else
                    {
                        s_stSvcMCp.stDbV2xStatusTx.unTxVehicleSpeed = nCurrSpeed;
                    }

                    s_usLastSpeedTx = nCurrSpeed;

                    s_usGpsSpeedCalCnt = 0;
                }
            }

            if(s_usGpsSpeedCalCnt == 0)
            {
                s_stDbV2xStatus.stV2xGpsInfoTx.nLatitudeLast = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
                s_stDbV2xStatus.stV2xGpsInfoTx.nLongitudeLast = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
                s_stDbV2xStatus.stV2xGpsInfoTx.ulTimeStampLast = pstTimeManager->ulTimeStamp;
            }

            s_usGpsSpeedCalCnt++;

#if defined(CONFIG_GPS_OBU) || defined(CONFIG_GPS_RSU)
			s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLatitudeNow = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
            s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLongitudeNow = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
            s_stDbV2xStatus.stV2xGpsInfoHeadingTx.ulTimeStampNow = pstTimeManager->ulTimeStamp;

			dHeading = DI_GPS_CalculateHeading(&s_stDbV2xStatus.stV2xGpsInfoHeadingTx);
            if (dHeading < 0)
            {
                PrintError("DI_GPS_CalculateHeading() is failed! [dHeading:%lf]", dHeading);
            }
#else
            dHeading = DI_GPS_GetHeading(&pstDi->stDiGps);
            if (dHeading < 0)
            {
                PrintError("DI_GPS_GetHeading() is failed! [dHeading:%lf]", dHeading);
            }
#endif
            s_stSvcMCp.stDbV2xStatusTx.unTxVehicleHeading = (uint32_t)dHeading;

			s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLatitudeLast = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
            s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLongitudeLast = s_stSvcMCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
            s_stDbV2xStatus.stV2xGpsInfoHeadingTx.ulTimeStampLast = pstTimeManager->ulTimeStamp;

            s_stSvcMCp.stDbV2x.ulReserved = 0;

            if(bMsgTx == TRUE)
            {
                nFrameWorkRet = MSG_MANAGER_Transmit(&s_stSvcMCp.stMsgManagerTx, &s_stSvcMCp.stDbV2x, (char*)pchPayload);
                if(nFrameWorkRet != FRAMEWORK_OK)
                {
                    PrintError("MSG_MANAGER_Transmit() is failed! [nRet:%d]", nFrameWorkRet);
                }
            }
            else
            {
                nFrameWorkRet = DB_MANAGER_Write(&s_stSvcMCp.stDbManagerWrite, &s_stSvcMCp.stDbV2x, (char*)pchPayload);
                if(nFrameWorkRet != FRAMEWORK_OK)
                {
                    PrintError("DB_MANAGER_Write() is failed! [nRet:%d]", nFrameWorkRet);
                }
            }

            /* free(pchPayload) is free at the P_MSG_MANAGER_SendTxMsg() */

            if(s_stSvcMCp.stDbV2xStatusTx.unSeqNum == DB_V2X_STATUS_SEQ_NUM_MAX)
            {
                /* Reset the sequence number */
                s_stSvcMCp.stDbV2xStatusTx.unSeqNum = 0;
            }

            if(s_stSvcMCp.stDbV2xStatusTx.unContCnt == DB_V2X_STATUS_CONT_CNT_MAX)
            {
                /* Reset the continuity counter */
                s_stSvcMCp.stDbV2xStatusTx.unContCnt = 0;
            }

            /* Increase the sequence number */
            s_stSvcMCp.stDbV2xStatusTx.unSeqNum++;

            /* Increase the continuity counter */
            s_stSvcMCp.stDbV2xStatusTx.unContCnt++;

            usleep((s_stSvcMCp.stMsgManagerTx.unTxDelay * USLEEP_MS));
        }
        else
        {
            if(s_stSvcMCp.bLogLevel == TRUE)
            {
                PrintError("s_stSvcMCp.eSvcMCpStatus [%d]", s_stSvcMCp.eSvcMCpStatus);
            }
            usleep(1000);
        }
    }

    return NULL;
}

static void *P_SVC_MCP_Task(void *arg)
{
    SVC_MCP_EVENT_MSG_T stEventMsg;
    int32_t nRet = APP_ERROR;

    UNUSED(arg);

    memset(&stEventMsg, 0, sizeof(SVC_MCP_EVENT_MSG_T));

    while (1)
    {
        if(msgrcv(s_nSvcMCpTaskMsgId, &stEventMsg, sizeof(SVC_MCP_EVENT_MSG_T), 0, MSG_NOERROR) == APP_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch(stEventMsg.eEventType)
            {
                case SVC_MCP_EVENT_START:
                {
                    nRet = P_SVC_MCP_Start(&stEventMsg);
                    if (nRet != APP_OK)
                    {
                        PrintError("SVC_MCP_Start() is failed! [unRet:%d]", nRet);
                    }
                    break;
                }

                case SVC_MCP_EVENT_STOP:
                {
                    nRet = P_SVC_MCP_Stop(&stEventMsg);
                    if (nRet != APP_OK)
                    {
                        PrintError("SVC_MCP_Stop() is failed! [unRet:%d]", nRet);
                    }
                    break;
                }

                default:
                    PrintWarn("unknown event type [%d]", stEventMsg.eEventType);
                    break;
            }
        }

        usleep(1000);
    }

    return NULL;
}

static void P_SVC_MCP_PrintMsgInfo(int msqid)
{

    struct msqid_ds m_stat;

    PrintDebug("========== Messege Queue Infomation =============");

    if(msgctl(msqid, IPC_STAT, &m_stat) == APP_MSG_ERR)
    {
        PrintError("msgctl() is failed!!");
    }

    PrintDebug("msg_lspid : %d", m_stat.msg_lspid);
    PrintDebug("msg_qnum : %ld", m_stat.msg_qnum);
    PrintDebug("msg_stime : %ld", m_stat.msg_stime);

    PrintDebug("=================================================");
}

int32_t P_SVC_MCP_CreateTask(void)
{
	int32_t nRet = APP_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_SvcMCpTask, &attr, P_SVC_MCP_Task, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_join() is failed!! (P_SVC_MCP_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_SVC_MCP_Task() is successfully created.");
        nRet = APP_OK;
    }

    nRet = pthread_create(&sh_SvcMCpTaskTx, &attr, P_SVC_MCP_TaskTx, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_join() is failed!! (P_SVC_MCP_TaskTx) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_SVC_MCP_TaskTx() is successfully created.");
        nRet = APP_OK;
    }

#if defined(CONFIG_PTHREAD_JOINABLE)
    nRet = pthread_join(sh_SvcMCpTask, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_join() is failed!! (P_SVC_MCP_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintDebug("P_SVC_MCP_Task() is successfully joined.");
        nRet = APP_OK;
    }
#endif
	return nRet;
}

static int32_t P_SVC_MCP_Init(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    if((s_nSvcMCpTaskMsgId = msgget(s_SvcMCpTaskMsgKey, IPC_CREAT|0666)) == APP_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_SVC_MCP_PrintMsgInfo(s_nSvcMCpTaskMsgId);
    }

    nRet = P_SVC_MCP_CreateTask();
    if (nRet != APP_OK)
    {
        PrintError("P_SVC_MCP_CreateTask() is failed! [nRet:%d]", nRet);
    }

    (void*)memset(&pstSvcMCp->stDbManagerWrite, 0x00, sizeof(DB_MANAGER_WRITE_T));
    (void*)memset(&pstSvcMCp->stMsgManagerTx, 0x00, sizeof(MSG_MANAGER_TX_T));
    (void*)memset(&pstSvcMCp->stMsgManagerRx, 0x00, sizeof(MSG_MANAGER_RX_T));
    (void*)memset(&pstSvcMCp->stDbV2x, 0x00, sizeof(DB_V2X_T));
    (void*)memset(&pstSvcMCp->stDbV2xStatusTx, 0x00, sizeof(DB_V2X_STATUS_TX_T));
    (void*)memset(&s_stSvcMCp, 0x00, sizeof(SVC_MCP_T));

    nRet = P_SVC_MCP_SetDefaultSettings(pstSvcMCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_MCP_SetDefaultSettings() is failed! [nRet:%d]", nRet);
    }

    nRet = P_SVC_MCP_SetSettings(pstSvcMCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_MCP_SetSettings() is failed! [nRet:%d]", nRet);
    }
    return nRet;
}

static int32_t P_SVC_MCP_DeInit(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    return nRet;
}

void SVC_MCP_ShowSettings(SVC_MCP_T *pstSvcMCp)
{
    PrintTrace("========================================================");
    PrintWarn("MSG V2X Tx Info>");
    PrintDebug(" ePayloadType[%d]", pstSvcMCp->stMsgManagerTx.ePayloadType);
    PrintDebug(" eCommType[%d]", pstSvcMCp->stMsgManagerTx.eCommType);
    PrintDebug(" eSignId[%d]", pstSvcMCp->stMsgManagerTx.eSignId);
    PrintDebug(" eV2xFreq[%d]", pstSvcMCp->stMsgManagerTx.eV2xFreq);
    PrintDebug(" ePriority[%d]", pstSvcMCp->stMsgManagerTx.ePriority);
    PrintDebug(" eV2xDataRate[%d]", pstSvcMCp->stMsgManagerTx.eV2xDataRate);
    PrintDebug(" eV2xTimeSlot[%d]", pstSvcMCp->stMsgManagerTx.eV2xTimeSlot);
    PrintDebug(" unPsid[%d]", pstSvcMCp->stMsgManagerTx.unPsid);
    PrintDebug(" nTxPower[%d]", pstSvcMCp->stMsgManagerTx.nTxPower);
    PrintDebug(" unTxCount[%d]", pstSvcMCp->stMsgManagerTx.unTxCount);
    PrintDebug(" unTxDelay[%d ms]", pstSvcMCp->stMsgManagerTx.unTxDelay);
    for(int i = 0; i < MSG_MANAGER_MAC_LENGTH; i++)
    {
        PrintDebug(" unTxCount[i:%d][0x%x]", i, pstSvcMCp->stMsgManagerTx.uchPeerMacAddr[i]);
    }

    PrintDebug(" unTransmitterProfileId[%d]", pstSvcMCp->stMsgManagerTx.unTransmitterProfileId);
    PrintDebug(" unPeerL2Id[%d]", pstSvcMCp->stMsgManagerTx.unPeerL2Id);

    PrintWarn("DB V2X Info>");
    PrintDebug(" eDeviceType[%d]", pstSvcMCp->stDbV2x.eDeviceType);
    PrintDebug(" eTeleCommType[%d]", pstSvcMCp->stDbV2x.eTeleCommType);
    PrintDebug(" unDeviceId[%d]", pstSvcMCp->stDbV2x.unDeviceId);
    PrintDebug(" eServiceId[%d]", pstSvcMCp->stDbV2x.eServiceId);
    PrintDebug(" eActionType[%d]", pstSvcMCp->stDbV2x.eActionType);
    PrintDebug(" eRegionId[%d]", pstSvcMCp->stDbV2x.eRegionId);
    PrintDebug(" ePayloadType[%d]", pstSvcMCp->stDbV2x.ePayloadType);
    PrintDebug(" eCommId[%d]", pstSvcMCp->stDbV2x.eCommId);
    PrintDebug(" usDbVer[%d.%d]", pstSvcMCp->stDbV2x.usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstSvcMCp->stDbV2x.usDbVer & CLI_DB_V2X_MINOR_MASK);
    PrintDebug(" usHwVer[0x%x]", pstSvcMCp->stDbV2x.usHwVer);
    PrintDebug(" usSwVer[0x%x]", pstSvcMCp->stDbV2x.usSwVer);

    PrintWarn("Device Info>");
    PrintDebug("Ethernet Interface [%s]", pstSvcMCp->pchIfaceName);
    PrintDebug("PSID [%d]", pstSvcMCp->unPsid);
    PrintDebug("pchIpAddr [%s]", pstSvcMCp->pchIpAddr);
    PrintDebug("unPort [%d]", pstSvcMCp->unPort);
    PrintDebug("pchDeviceName [%s]", pstSvcMCp->pchDeviceName);
    PrintDebug("ulDbStartTime [%ld]", pstSvcMCp->ulDbStartTime);
    PrintDebug("ulDbEndTime [%ld]", pstSvcMCp->ulDbEndTime);
    PrintDebug("unDbTotalWrittenTime [%d]", pstSvcMCp->unDbTotalWrittenTime);

    PrintWarn("V2X Status Tx Info>");
    PrintDebug(" stDbV2xDevL1.ulTimeStamp [%ld]", pstSvcMCp->stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp);
    PrintDebug(" stDbV2xDevL2.ulTimeStamp [%ld]", pstSvcMCp->stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp);
    PrintDebug(" stDbV2xDevL3.ulTimeStamp [%ld]", pstSvcMCp->stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp);
    PrintDebug(" unRxTargetDeviceId [%d]", pstSvcMCp->stDbV2xStatusTx.unRxTargetDeviceId);
    PrintDebug(" usTxFreq [%d]", pstSvcMCp->stDbV2xStatusTx.usTxFreq);
    PrintDebug(" ucTxPwr [%d]", pstSvcMCp->stDbV2xStatusTx.ucTxPwr);
    PrintDebug(" ucTxBw [%d]", pstSvcMCp->stDbV2xStatusTx.ucTxBw);
    PrintDebug(" ucScs [%d]", pstSvcMCp->stDbV2xStatusTx.ucScs);
    PrintDebug(" ucMcs [%d]", pstSvcMCp->stDbV2xStatusTx.ucMcs);
    PrintDebug(" usTxRatio [%d]", pstSvcMCp->stDbV2xStatusTx.usTxRatio);
    PrintDebug(" nTxLatitude [%d]", pstSvcMCp->stDbV2xStatusTx.stTxPosition.nTxLatitude);
    PrintDebug(" nTxLongitude [%d]", pstSvcMCp->stDbV2xStatusTx.stTxPosition.nTxLongitude);
    PrintDebug(" nTxAttitude [%d]", pstSvcMCp->stDbV2xStatusTx.stTxPosition.nTxAttitude);
    PrintDebug(" unSeqNum [%d]", pstSvcMCp->stDbV2xStatusTx.unSeqNum);
    PrintDebug(" unContCnt [%d]", pstSvcMCp->stDbV2xStatusTx.unContCnt);
    PrintDebug(" unTxVehicleSpeed [%d]", pstSvcMCp->stDbV2xStatusTx.unTxVehicleSpeed);
    PrintDebug(" unTxVehicleHeading [%d]", pstSvcMCp->stDbV2xStatusTx.unTxVehicleHeading);

    PrintTrace("========================================================");
}

int32_t SVC_MCP_GetSettings(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_MCP_GetSettings(pstSvcMCp);
    if (nRet != APP_OK)
    {
        PrintError("P_SVC_MCP_GetSettings() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

int32_t SVC_MCP_SetSettings(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_MCP_SetSettings(pstSvcMCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_MCP_SetSettings() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

int32_t SVC_MCP_SetLog(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    s_stSvcMCp.bLogLevel = pstSvcMCp->bLogLevel;
    PrintTrace("SET:s_stSvcMCp.bLogLevel [%s]", s_stSvcMCp.bLogLevel == ON ? "ON" : "OFF");

    nRet = APP_OK;

    return nRet;
}

int32_t SVC_MCP_Open(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;
    int32_t nFrameWorkRet = FRAMEWORK_ERROR;
    DB_MANAGER_T *pstDbManager;
    MSG_MANAGER_T *pstMsgManager;
    DI_T *pstDi;
    uint32_t nRetryCnt = 0;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    pstDbManager = FRAMEWORK_GetDbManagerInstance();
    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        nRet = APP_ERROR;
        return nRet;
    }

    pstDbManager->eFileType = pstSvcMCp->stDbManagerWrite.eFileType;
    pstDbManager->eSvcType = DB_MANAGER_SVC_TYPE_V2X_STATUS;

    nFrameWorkRet = DB_MANAGER_Open(pstDbManager);
    if(nFrameWorkRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Open() is failed! [nRet:%d]", nFrameWorkRet);
        return nRet;
    }

    pstMsgManager = FRAMEWORK_GetMsgManagerInstance();
    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        nRet = APP_ERROR;
        return nRet;
    }

    pstMsgManager->eDeviceType = pstSvcMCp->stDbV2x.eDeviceType;
    PrintDebug("eDeviceType[%d]", pstMsgManager->eDeviceType);
    pstMsgManager->pchIfaceName = pstSvcMCp->pchIfaceName;
    pstMsgManager->stExtMsgWsr.unPsid = pstSvcMCp->unPsid;
    pstMsgManager->pchIpAddr = pstSvcMCp->pchIpAddr;
    pstMsgManager->unPort = pstSvcMCp->unPort;

    nFrameWorkRet = MSG_MANAGER_Open(pstMsgManager);
    if(nFrameWorkRet != FRAMEWORK_OK)
    {
        PrintError("MSG_MANAGER_Open() is failed! [nRet:%d]", nFrameWorkRet);
        return nRet;
    }

    pstDi = APP_GetDiInstance();
    if (pstDi == NULL)
    {
        PrintError("APP_GetDiInstance() is failed! [nRet:%d]", nRet);
        nRet = APP_ERROR;
        return nRet;
    }

    for(nRetryCnt = 0; nRetryCnt < SVC_MCP_GPS_OPEN_RETRY_CNT; nRetryCnt++)
    {
        nRet = DI_GPS_Open(&pstDi->stDiGps);
        if (nRet != DI_OK)
        {
            PrintError("DI_GPS_Open() is failed! [nRet:%d], nRetryCnt[%d/%d], retry after [%d us]", nRet, nRetryCnt, SVC_MCP_GPS_OPEN_RETRY_CNT, SVC_MCP_GPS_OPEN_RETRY_DELAY * USLEEP_MS);
            usleep(SVC_MCP_GPS_OPEN_RETRY_DELAY * USLEEP_MS);
        }
        else
        {
            PrintTrace("DI_GPS_Open() is successfully opened");
            break;
        }
    }

    nRet = nFrameWorkRet;

    return nRet;
}

int32_t SVC_MCP_Close(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;
    DI_T *pstDi;
    DB_MANAGER_T *pstDbManager;
    MSG_MANAGER_T *pstMsgManager;
    char chTempDate[SVC_MCP_DATE_LEN+1];
    char chTempHour[SVC_MCP_HOUR_LEN+1];
    char chTempMin[SVC_MCP_MIN_LEN+1];
    char chTempSec[SVC_MCP_SEC_LEN+1];

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    pstDi = APP_GetDiInstance();
    if (pstDi == NULL)
    {
        PrintError("APP_GetDiInstance() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    pstMsgManager = FRAMEWORK_GetMsgManagerInstance();
    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        nRet = APP_ERROR;
        return nRet;
    }

    pstMsgManager->pchIfaceName = pstSvcMCp->pchIfaceName;
    pstMsgManager->stExtMsgWsr.unPsid = pstSvcMCp->unPsid;

    nRet = MSG_MANAGER_Close(pstMsgManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("MSG_MANAGER_Close() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = DI_GPS_Close(&pstDi->stDiGps);
    if (nRet != DI_OK)
    {
        PrintError("DI_GPS_Close() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    pstDbManager = FRAMEWORK_GetDbManagerInstance();
    if (pstDbManager == NULL)
    {
        PrintError("FRAMEWORK_GetDbManagerInstance() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = DB_MANAGER_Close(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Close() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    /* Tx */
    sprintf(s_chStrBufTxRxType, "%s", SVC_MCP_DB_TX);
    pstDbManager->stDbFile.pchTxRxType = s_chStrBufTxRxType;
    if(s_stSvcMCp.stDbV2x.eDeviceType == DB_V2X_DEVICE_TYPE_OBU)
    {

        sprintf(s_chStrBufDevType, "%s", SVC_MCP_DEV_OBU);
        PrintTrace("CONFIG_OBU is enabled, s_chStrBufDevType [%s]", s_chStrBufDevType);
    }
    else if(s_stSvcMCp.stDbV2x.eDeviceType == DB_V2X_DEVICE_TYPE_RSU)
    {
        sprintf(s_chStrBufDevType, "%s", SVC_MCP_DEV_RSU);
        PrintTrace("CONFIG_RSU is enabled, s_chStrBufDevType [%s]", s_chStrBufDevType);
    }
    else
    {
        sprintf(s_chStrBufDevType, "%s", SVC_MCP_DEV_UNKNOWN);
        PrintError("unknown device type[%d]", s_stSvcMCp.stDbV2x.eDeviceType);
    }
    pstDbManager->stDbFile.pchDeviceType = s_chStrBufDevType;

    sprintf(s_chStrBufDevId, "%s%s", DB_V2X_DEVICE_ID_PREFIX, s_stSvcMCp.pchDeviceName);
    pstDbManager->stDbFile.pchDeviceId = s_chStrBufDevId;

    sprintf(s_chStrBufStartTime, "%ld", s_stSvcMCp.ulDbStartTime);
    strncpy(chTempDate, s_chStrBufStartTime, SVC_MCP_DATE_LEN);
    chTempDate[SVC_MCP_DATE_LEN] = '\0';
    strncpy(chTempHour, s_chStrBufStartTime + SVC_MCP_DATE_LEN, SVC_MCP_HOUR_LEN);
    chTempHour[SVC_MCP_HOUR_LEN] = '\0';
    strncpy(chTempMin, s_chStrBufStartTime + SVC_MCP_DATE_LEN + SVC_MCP_HOUR_LEN, SVC_MCP_MIN_LEN);
    chTempMin[SVC_MCP_MIN_LEN] = '\0';
    strncpy(chTempSec, s_chStrBufStartTime + SVC_MCP_DATE_LEN + SVC_MCP_HOUR_LEN + SVC_MCP_MIN_LEN, SVC_MCP_SEC_LEN);
    chTempSec[SVC_MCP_SEC_LEN] = '\0';
    sprintf(s_chStrBufStartTime, "%s-%s-%s-%s", chTempDate, chTempHour, chTempMin, chTempSec);
    pstDbManager->stDbFile.pchStartTime = s_chStrBufStartTime;

    sprintf(s_chStrBufEndTime, "%ld", s_stSvcMCp.ulDbEndTime);
    strncpy(chTempDate, s_chStrBufEndTime, SVC_MCP_DATE_LEN);
    chTempDate[SVC_MCP_DATE_LEN] = '\0';
    strncpy(chTempHour, s_chStrBufEndTime + SVC_MCP_DATE_LEN, SVC_MCP_HOUR_LEN);
    chTempHour[SVC_MCP_HOUR_LEN] = '\0';
    strncpy(chTempMin, s_chStrBufEndTime + SVC_MCP_DATE_LEN + SVC_MCP_HOUR_LEN, SVC_MCP_MIN_LEN);
    chTempMin[SVC_MCP_MIN_LEN] = '\0';
    strncpy(chTempSec, s_chStrBufEndTime + SVC_MCP_DATE_LEN + SVC_MCP_HOUR_LEN + SVC_MCP_MIN_LEN, SVC_MCP_SEC_LEN);
    chTempSec[SVC_MCP_SEC_LEN] = '\0';

    sprintf(s_chStrBufEndTime, "%s-%s-%s-%s", chTempDate, chTempHour, chTempMin, chTempSec);
    pstDbManager->stDbFile.pchEndTime = s_chStrBufEndTime;

    sprintf(s_chStrBufTotalTime, "%d%s", s_stSvcMCp.unDbTotalWrittenTime, "secs");
    pstDbManager->stDbFile.pchTotalTime = s_chStrBufTotalTime;

    nRet = DB_MANAGER_MakeDbFile(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_MakeDbFile() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = DB_MANAGER_RemoveTempFile(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_RemoveTempFile() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = DB_MANAGER_UploadFile(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_UploadFile() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    /* Rx */
    sprintf(s_chStrBufTxRxType, "%s", SVC_MCP_DB_RX);
    pstDbManager->stDbFile.pchTxRxType = s_chStrBufTxRxType;

    nRet = DB_MANAGER_MakeDbFile(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_MakeDbFile() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = DB_MANAGER_RemoveTempFile(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_RemoveTempFile() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = DB_MANAGER_UploadFile(pstDbManager);
    if(nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_UploadFile() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    return nRet;
}

int32_t SVC_MCP_Start(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;
    SVC_MCP_EVENT_MSG_T stEventMsg;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    (void*)memset(&stEventMsg, 0x00, sizeof(SVC_MCP_EVENT_MSG_T));

    stEventMsg.eEventType = SVC_MCP_EVENT_START;

    nRet = msgsnd(s_nSvcMCpTaskMsgId, &stEventMsg, sizeof(SVC_MCP_EVENT_MSG_T), IPC_NOWAIT);
    if(nRet < 0)
    {
        PrintError("msgsnd() is failed! [nRet:%d]", nRet);
        return nRet;
    }
    else
    {
        nRet = APP_OK;
    }

    return nRet;
}

int32_t SVC_MCP_Stop(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;
    SVC_MCP_EVENT_MSG_T stEventMsg;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    (void*)memset(&stEventMsg, 0x00, sizeof(SVC_MCP_EVENT_MSG_T));

    stEventMsg.eEventType = SVC_MCP_EVENT_STOP;

    nRet = msgsnd(s_nSvcMCpTaskMsgId, &stEventMsg, sizeof(SVC_MCP_EVENT_MSG_T), IPC_NOWAIT);
    if(nRet < 0)
    {
        PrintError("msgsnd() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    else
    {
        nRet = APP_OK;
    }

    return nRet;
}

int32_t SVC_MCP_Status(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    PrintWarn("TODO");

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t SVC_MCP_Init(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_MCP_Init(pstSvcMCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_MCP_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    s_stSvcMCp.bLogLevel = pstSvcMCp->bLogLevel;
    PrintDebug("s_stSvcMCp.bLogLevel [%s]", s_stSvcMCp.bLogLevel == ON ? "ON" : "OFF");

    return nRet;
}

int32_t SVC_MCP_DeInit(SVC_MCP_T *pstSvcMCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcMCp == NULL)
    {
        PrintError("pstSvcMCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_MCP_DeInit(pstSvcMCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_MCP_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

