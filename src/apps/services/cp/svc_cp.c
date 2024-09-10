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
* @file svc_cp.c
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
#include "svc_cp.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include "app.h"
#include "di.h"

/***************************** Definition ************************************/
#define SVC_CP_GPS_SPEED_CAL_CNT_MAX    (10)
#define SVC_CP_SET_BUF_SIZE             (256)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
FILE* sh_pSvcCpTxMsg;
FILE* sh_pSvcCpRxMsg;

static int s_nSvcCpTaskMsgId;

static key_t s_SvcCpTaskMsgKey = SVC_CP_TASK_MSG_KEY;

static pthread_t sh_SvcCpTask;
static pthread_t sh_SvcCpTaskTx;

static SVC_CP_T s_stSvcCp;
static DB_MANAGER_V2X_STATUS_T s_stDbV2xStatus;
static bool s_bFirstMsg = TRUE;

static uint16_t s_usGpsSpeedCalCnt = 0;
static uint32_t s_usLastSpeedTx;

static char s_chStrBufTxRxType[SVC_CP_STR_BUF_LEN];
static char s_chStrBufDevType[SVC_CP_STR_BUF_LEN];
static char s_chStrBufDevId[SVC_CP_STR_BUF_LEN];
static char s_chStrBufStartTime[SVC_CP_STR_BUF_LEN];
static char s_chStrBufEndTime[SVC_CP_STR_BUF_LEN];
static char s_chStrBufTotalTime[SVC_CP_STR_BUF_LEN];

static char s_chDeviceName[SVC_CP_SET_BUF_SIZE] = DB_MGR_DEFAULT_COMM_DEV_ID;
static char s_chIfaceName[SVC_CP_SET_BUF_SIZE] = SVC_CP_DEFAULT_ETH_DEV;
static char s_chIpAddr[SVC_CP_SET_BUF_SIZE] = SVC_CP_DEFAULT_IP;

/***************************** Function  *************************************/

int32_t P_SVC_CP_SetSettings(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp is NULL!!");
    }

    memcpy(&s_stSvcCp, pstSvcCp, sizeof(SVC_CP_T));
    nRet = APP_OK;

    return nRet;
}

int32_t P_SVC_CP_GetSettings(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp is NULL!!");
    }

    memcpy(pstSvcCp, &s_stSvcCp, sizeof(SVC_CP_T));
    nRet = APP_OK;

    return nRet;
}

int32_t SVC_CP_UpdateSettings(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;
    char chModelNameFile[MAX_MODEL_NAME_LEN] = {0};
    FILE *h_fdModelConf;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    snprintf(chModelNameFile, sizeof(chModelNameFile), "%s%s", CONFIG_MODEL_NAME, MODEL_NAME_FILE_SUFFIX);

    char chBuf[SVC_CP_SET_BUF_SIZE] = {0};

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
                strncpy(s_chDeviceName, chBuf, SVC_CP_SET_BUF_SIZE); // Static buffer 사용
                pstSvcCp->pchDeviceName = s_chDeviceName;
                PrintTrace("Updated Device Name: %s", pstSvcCp->pchDeviceName);
            }
            else if (strncmp(chLine, DEVICE_ID_PREFIX, DEVICE_ID_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + DEVICE_ID_PREFIX_LEN, sizeof(chBuf) - 1);
                pstSvcCp->stDbV2x.unDeviceId = (uint32_t)atoi(chBuf);
                PrintTrace("Updated Device ID: %u", pstSvcCp->stDbV2x.unDeviceId);
            }
            else if (strncmp(chLine, IFACE_NAME_PREFIX, IFACE_NAME_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + IFACE_NAME_PREFIX_LEN, sizeof(chBuf) - 1);
                strncpy(s_chIfaceName, chBuf, SVC_CP_SET_BUF_SIZE); // Static buffer 사용
                pstSvcCp->pchIfaceName = s_chIfaceName;
                PrintTrace("Updated Interface Name: %s", pstSvcCp->pchIfaceName);
            }
            else if (strncmp(chLine, IP_ADDR_PREFIX, IP_ADDR_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + IP_ADDR_PREFIX_LEN, sizeof(chBuf) - 1);
                strncpy(s_chIpAddr, chBuf, SVC_CP_SET_BUF_SIZE); // Static buffer 사용
                pstSvcCp->pchIpAddr = s_chIpAddr;
                PrintTrace("Updated IP Address: %s", pstSvcCp->pchIpAddr);
            }
            else if (strncmp(chLine, PORT_PREFIX, PORT_PREFIX_LEN) == 0)
            {
                strncpy(chBuf, chLine + PORT_PREFIX_LEN, sizeof(chBuf) - 1);
                pstSvcCp->unPort = (uint32_t)atoi(chBuf);
                PrintTrace("Updated Port: %d", pstSvcCp->unPort);
            }
        }

        fclose(h_fdModelConf);
    }
    else
    {
        PrintWarn("chModelNameFile[%s] is not exist!", chModelNameFile);
    }

    nRet = APP_OK;

    PrintDebug("SVC_CP_UpdateSettings() set is finished.[eth:%s,ip:%s,port:%d]", pstSvcCp->pchIfaceName, pstSvcCp->pchIpAddr, pstSvcCp->unPort);

    return nRet;
}


int32_t P_SVC_CP_SetDefaultSettings(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    pstSvcCp->stDbManagerWrite.eFileType = DB_MANAGER_FILE_TYPE_CSV;
    pstSvcCp->stDbManagerWrite.eCommMsgType = DB_MANAGER_COMM_MSG_TYPE_TX;
    pstSvcCp->stDbManagerWrite.eProc = DB_MANAGER_PROC_WRITE;

    pstSvcCp->stMsgManagerTx.ePayloadType = eMSG_MANAGER_PAYLOAD_TYPE_RAW;
    pstSvcCp->stMsgManagerTx.eCommType = eMSG_MANAGER_COMM_TYPE_5GNRV2X;
    pstSvcCp->stMsgManagerTx.eSignId = eMSG_MANAGER_SIGN_ID_UNSECURED;
    pstSvcCp->stMsgManagerTx.eV2xFreq = eMSG_MANAGER_V2X_FREQ_5900;
    pstSvcCp->stMsgManagerTx.ePriority = eMSG_MANAGER_PRIORITY_CV2X_PPPP_0;
    pstSvcCp->stMsgManagerTx.eV2xDataRate = eMSG_MANAGER_V2X_DATA_RATE_6MBPS;
    pstSvcCp->stMsgManagerTx.eV2xTimeSlot = eMSG_MANAGER_V2X_TIME_SLOT_CONTINUOUS;
    pstSvcCp->stMsgManagerTx.unPsid = DB_V2X_PSID;
    pstSvcCp->stMsgManagerTx.nTxPower = MSG_MANAGER_V2X_TX_POWER;
    pstSvcCp->stMsgManagerTx.unTxCount = MSG_MANAGER_V2X_TX_COUNT;
    pstSvcCp->stMsgManagerTx.unTxDelay = MSG_MANAGER_V2X_TX_DELAY;

    for(int i = 0; i < MSG_MANAGER_MAC_LENGTH; i++)
    {
        pstSvcCp->stMsgManagerTx.uchPeerMacAddr[i] = 0xFF;
    }

    pstSvcCp->stMsgManagerTx.unTransmitterProfileId = MSG_MANAGER_V2X_TX_PROFILE_ID;
    pstSvcCp->stMsgManagerTx.unPeerL2Id = MSG_MANAGER_V2X_TX_PEER_L2_ID;

#if defined(CONFIG_OBU)
    pstSvcCp->stDbV2x.eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
    PrintTrace("CONFIG_OBU is enabled, eDeviceType [%d]", pstSvcCp->stDbV2x.eDeviceType);

    pstSvcCp->pchIpAddr = SVC_CP_DEFAULT_IP;
    pstSvcCp->unPort = SVC_CP_DEFAULT_PORT;
    pstSvcCp->pchIfaceName = SVC_CP_DEFAULT_ETH_DEV;
    pstSvcCp->unPsid = SVC_CP_V2V_PSID;
    pstSvcCp->pchDeviceName = DB_MGR_DEFAULT_COMM_DEV_ID;
#elif defined(CONFIG_RSU)
    pstSvcCp->stDbV2x.eDeviceType = DB_V2X_DEVICE_TYPE_RSU;
    PrintTrace("CONFIG_RSU is enabled, eDeviceType [%d]", pstSvcCp->stDbV2x.eDeviceType);

    pstSvcCp->pchIpAddr = SVC_CP_DEFAULT_RSU_IP;
    pstSvcCp->unPort = SVC_CP_DEFAULT_RSU_PORT;
    pstSvcCp->pchIfaceName = SVC_CP_DEFAULT_RSU_ETH_DEV;
    pstSvcCp->unPsid = SVC_CP_I2V_PSID;
    pstSvcCp->pchDeviceName = DB_MGR_DEFAULT_COMM_RSU_DEV_ID;
#else
    PrintError("check device type!!");
    return APP_ERROR;
#endif

    pstSvcCp->stDbV2x.eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5_BROADCAST;
    pstSvcCp->stDbV2x.unDeviceId = CLI_DB_V2X_DEFAULT_DEVICE_ID;
    pstSvcCp->stDbV2x.eServiceId = DB_V2X_SERVICE_ID_PLATOONING;
    pstSvcCp->stDbV2x.eActionType = DB_V2X_ACTION_TYPE_REQUEST;
    pstSvcCp->stDbV2x.eRegionId = DB_V2X_REGION_ID_SEONGNAM;
    pstSvcCp->stDbV2x.ePayloadType = DB_V2X_PAYLOAD_TYPE_V2X_STATUS;
    pstSvcCp->stDbV2x.eCommId = DB_V2X_COMM_ID_V2V;
    pstSvcCp->stDbV2x.usDbVer = (DB_V2X_VERSION_MAJOR << CLI_DB_V2X_MAJOR_SHIFT) | DB_V2X_VERSION_MINOR;
    pstSvcCp->stDbV2x.usHwVer = CLI_DB_V2X_DEFAULT_HW_VER;
    pstSvcCp->stDbV2x.usSwVer = CLI_DB_V2X_DEFAULT_SW_VER;


    pstSvcCp->stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp = 0;
    pstSvcCp->stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp = 0;
    pstSvcCp->stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp = 0;
    pstSvcCp->stDbV2xStatusTx.unRxTargetDeviceId = 0;
    pstSvcCp->stDbV2xStatusTx.usTxFreq = MSG_MANAGER_V2X_TX_FREQ;
    pstSvcCp->stDbV2xStatusTx.ucTxPwr = MSG_MANAGER_V2X_TX_POWER;
    pstSvcCp->stDbV2xStatusTx.ucTxBw = MSG_MANAGER_V2X_TX_BW;
    pstSvcCp->stDbV2xStatusTx.ucScs = 0;
    pstSvcCp->stDbV2xStatusTx.ucMcs = 0;

    pstSvcCp->stDbV2xStatusTx.usTxRatio = pstSvcCp->stMsgManagerTx.unTxDelay;
    pstSvcCp->stDbV2xStatusTx.stTxPosition.nTxLatitude = 0;
    pstSvcCp->stDbV2xStatusTx.stTxPosition.nTxLongitude = 0;
    pstSvcCp->stDbV2xStatusTx.stTxPosition.nTxAttitude = 0;

    pstSvcCp->stDbV2xStatusTx.unSeqNum = 1;
    pstSvcCp->stDbV2xStatusTx.unContCnt = 1;
    pstSvcCp->stDbV2xStatusTx.unTxVehicleSpeed = DB_MGR_DEFAULT_VEHICLE_SPEED;
    pstSvcCp->stDbV2xStatusTx.unTxVehicleHeading = 0;

    pstSvcCp->ulDbStartTime = 0;
    pstSvcCp->ulDbEndTime = 0;
    pstSvcCp->unDbTotalWrittenTime = 0;

    nRet = APP_OK;

    PrintDebug("P_SVC_CP_SetDefaultSettings() set is finished.[eth:%s,ip:%s,port:%d]", pstSvcCp->pchIfaceName, pstSvcCp->pchIpAddr, pstSvcCp->unPort);

    return nRet;
}

static int32_t P_SVC_CP_Start(SVC_CP_EVENT_MSG_T *stEventMsg)
{
    int32_t nRet = APP_ERROR;
    TIME_MANAGER_T *pstTimeManager;
    UNUSED(stEventMsg);

    if ((s_stSvcCp.eSvcCpStatus == SVC_CP_STATUS_STOP) || (s_stSvcCp.eSvcCpStatus == SVC_CP_STATUS_IDLE))
    {
        s_stSvcCp.eSvcCpStatus = SVC_CP_STATUS_START;

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
            s_stSvcCp.ulDbStartTime = pstTimeManager->ulTimeStamp;
        }

        nRet = TIME_MANAGER_SetDbTxBegin(pstTimeManager);
        if(nRet != FRAMEWORK_OK)
        {
            PrintError("TIME_MANAGER_SetDbTxBegin() is failed! [nRet:%d]", nRet);
        }

        PrintTrace("eSvcCpStatus[%d] STARTs NOW [Time:%ld]", s_stSvcCp.eSvcCpStatus, s_stSvcCp.ulDbStartTime);

        if(s_stSvcCp.eSvcCpStatus == SVC_CP_STATUS_START)
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

static int32_t P_SVC_CP_Stop(SVC_CP_EVENT_MSG_T *stEventMsg)
{
    int32_t nRet = APP_ERROR;
    TIME_MANAGER_T *pstTimeManager;
    UNUSED(stEventMsg);

    if(s_stSvcCp.eSvcCpStatus == SVC_CP_STATUS_START)
    {
        s_stSvcCp.eSvcCpStatus = SVC_CP_STATUS_STOP;

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
            s_stSvcCp.ulDbEndTime = pstTimeManager->ulTimeStamp;
        }

        nRet = TIME_MANAGER_SetDbTxEnd(pstTimeManager);
        if(nRet != FRAMEWORK_OK)
        {
            PrintError("TIME_MANAGER_SetDbTxBegin() is failed! [nRet:%d]", nRet);
        }

        s_stSvcCp.unDbTotalWrittenTime = pstTimeManager->unDbTxTotalTime;

        PrintTrace("eSvcCpStatus[%d] STOPs NOW [Time:%ld], unDbTotalWrittenTime[%d seconds]", s_stSvcCp.eSvcCpStatus, s_stSvcCp.ulDbEndTime, s_stSvcCp.unDbTotalWrittenTime);

        if(s_stSvcCp.eSvcCpStatus == SVC_CP_STATUS_STOP)
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

int32_t P_SVC_CP_RestartDb(void)
{
    int32_t nRet = APP_ERROR;
    DB_MANAGER_T *pstDbManager;

    PrintWarn("Finish to write DB Files, start time[%ld], end time[%ld], total written time[%d]", s_stSvcCp.ulDbStartTime, s_stSvcCp.ulDbEndTime, s_stSvcCp.unDbTotalWrittenTime);

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
    pstDbManager->eFileType = s_stSvcCp.stDbManagerWrite.eFileType;
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

static void *P_SVC_CP_TaskTx(void *arg)
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
        if(s_stSvcCp.eSvcCpStatus == SVC_CP_STATUS_START)
        {
            s_stSvcCp.stDbV2x.ulPayloadLength = sizeof(s_stSvcCp.stDbV2xStatusTx);

            pchPayload = (char*)malloc(sizeof(char)*s_stSvcCp.stDbV2x.ulPayloadLength);
            if(pchPayload == NULL)
            {
                PrintError("malloc() is failed! [NULL]");
                break;
            }

            (void*)memset(pchPayload, 0x00, sizeof(sizeof(char)*s_stSvcCp.stDbV2x.ulPayloadLength));

            pstTimeManager = FRAMEWORK_GetTimeManagerInstance();
            if(pstTimeManager == NULL)
            {
                PrintError("pstTimeManager is NULL!");
            }

            /* Set at the Rx Device by Using Ext Msg */
            s_stSvcCp.stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp = 0;
            s_stSvcCp.stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp = 0;

            nFrameWorkRet = TIME_MANAGER_Get(pstTimeManager);
            if(nFrameWorkRet != FRAMEWORK_OK)
            {
                PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nFrameWorkRet);
            }
            else
            {
                s_stSvcCp.stDbV2x.ulTimeStamp = pstTimeManager->ulTimeStamp;

                /* Set the application timestamp before sending */
                s_stSvcCp.stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp = pstTimeManager->ulTimeStamp;
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

            s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLatitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fLatitude * SVC_CP_GPS_VALUE_CONVERT);
            s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLongitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fLongitude * SVC_CP_GPS_VALUE_CONVERT);
            s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxAttitude = (int32_t)(pstDi->stDiGps.stDiGpsData.fAltitude * SVC_CP_GPS_VALUE_CONVERT);

            memcpy(pchPayload, (char*)&s_stSvcCp.stDbV2xStatusTx, sizeof(char)*s_stSvcCp.stDbV2x.ulPayloadLength);

            if(s_bFirstMsg == TRUE)
            {
                s_stSvcCp.stDbV2xStatusTx.unTxVehicleSpeed = DB_MGR_DEFAULT_VEHICLE_SPEED;
                s_bFirstMsg = FALSE;
                PrintWarn("stDbV2xStatus.bFirstPacket's speed is the default value [%d]", s_stSvcCp.stDbV2xStatusTx.unTxVehicleSpeed);
            }
            else
            {
                if(s_usGpsSpeedCalCnt == SVC_CP_GPS_SPEED_CAL_CNT_MAX)
                {
                    s_stDbV2xStatus.stV2xGpsInfoTx.nLatitudeNow = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
                    s_stDbV2xStatus.stV2xGpsInfoTx.nLongitudeNow = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
                    s_stDbV2xStatus.stV2xGpsInfoTx.ulTimeStampNow = pstTimeManager->ulTimeStamp;

                    nCurrSpeed = DI_GPS_CalculateSpeed(&s_stDbV2xStatus.stV2xGpsInfoTx);
                    if(nCurrSpeed == 0)
                    {
                        s_stSvcCp.stDbV2xStatusTx.unTxVehicleSpeed = s_usLastSpeedTx;
                    }
                    else
                    {
                        s_stSvcCp.stDbV2xStatusTx.unTxVehicleSpeed = nCurrSpeed;
                    }

                    s_usLastSpeedTx = nCurrSpeed;

                    s_usGpsSpeedCalCnt = 0;
                }
            }

            if(s_usGpsSpeedCalCnt == 0)
            {
                s_stDbV2xStatus.stV2xGpsInfoTx.nLatitudeLast = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
                s_stDbV2xStatus.stV2xGpsInfoTx.nLongitudeLast = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
                s_stDbV2xStatus.stV2xGpsInfoTx.ulTimeStampLast = pstTimeManager->ulTimeStamp;
            }

            s_usGpsSpeedCalCnt++;

#if defined(CONFIG_GPS_OBU) || defined(CONFIG_GPS_RSU)
			s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLatitudeNow = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
            s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLongitudeNow = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
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
            s_stSvcCp.stDbV2xStatusTx.unTxVehicleHeading = (uint32_t)dHeading;

			s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLatitudeLast = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLatitude;
            s_stDbV2xStatus.stV2xGpsInfoHeadingTx.nLongitudeLast = s_stSvcCp.stDbV2xStatusTx.stTxPosition.nTxLongitude;
            s_stDbV2xStatus.stV2xGpsInfoHeadingTx.ulTimeStampLast = pstTimeManager->ulTimeStamp;

            s_stSvcCp.stDbV2x.ulReserved = 0;

            if(bMsgTx == TRUE)
            {
                nFrameWorkRet = MSG_MANAGER_Transmit(&s_stSvcCp.stMsgManagerTx, &s_stSvcCp.stDbV2x, (char*)pchPayload);
                if(nFrameWorkRet != FRAMEWORK_OK)
                {
                    PrintError("MSG_MANAGER_Transmit() is failed! [nRet:%d]", nFrameWorkRet);
                }
            }
            else
            {
                nFrameWorkRet = DB_MANAGER_Write(&s_stSvcCp.stDbManagerWrite, &s_stSvcCp.stDbV2x, (char*)pchPayload);
                if(nFrameWorkRet != FRAMEWORK_OK)
                {
                    PrintError("DB_MANAGER_Write() is failed! [nRet:%d]", nFrameWorkRet);
                }
            }

            /* free(pchPayload) is free at the P_MSG_MANAGER_SendTxMsg() */

            if(s_stSvcCp.stDbV2xStatusTx.unSeqNum == DB_V2X_STATUS_SEQ_NUM_MAX)
            {
                /* Reset the sequence number */
                s_stSvcCp.stDbV2xStatusTx.unSeqNum = 0;
            }

            if(s_stSvcCp.stDbV2xStatusTx.unContCnt == DB_V2X_STATUS_CONT_CNT_MAX)
            {
                /* Reset the continuity counter */
                s_stSvcCp.stDbV2xStatusTx.unContCnt = 0;
            }

            /* Increase the sequence number */
            s_stSvcCp.stDbV2xStatusTx.unSeqNum++;

            /* Increase the continuity counter */
            s_stSvcCp.stDbV2xStatusTx.unContCnt++;

            usleep((s_stSvcCp.stMsgManagerTx.unTxDelay * USLEEP_MS));
        }
        else
        {
            if(s_stSvcCp.bLogLevel == TRUE)
            {
                PrintError("s_stSvcCp.eSvcCpStatus [%d]", s_stSvcCp.eSvcCpStatus);
            }
            usleep(1000);
        }
    }

    return NULL;
}

static void *P_SVC_CP_Task(void *arg)
{
    SVC_CP_EVENT_MSG_T stEventMsg;
    int32_t nRet = APP_ERROR;

    UNUSED(arg);

    memset(&stEventMsg, 0, sizeof(SVC_CP_EVENT_MSG_T));

    while (1)
    {
        if(msgrcv(s_nSvcCpTaskMsgId, &stEventMsg, sizeof(SVC_CP_EVENT_MSG_T), 0, MSG_NOERROR) == APP_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch(stEventMsg.eEventType)
            {
                case SVC_CP_EVENT_START:
                {
                    nRet = P_SVC_CP_Start(&stEventMsg);
                    if (nRet != APP_OK)
                    {
                        PrintError("SVC_CP_Start() is failed! [unRet:%d]", nRet);
                    }
                    break;
                }

                case SVC_CP_EVENT_STOP:
                {
                    nRet = P_SVC_CP_Stop(&stEventMsg);
                    if (nRet != APP_OK)
                    {
                        PrintError("SVC_CP_Stop() is failed! [unRet:%d]", nRet);
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

static void P_SVC_CP_PrintMsgInfo(int msqid)
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

int32_t P_SVC_CP_CreateTask(void)
{
	int32_t nRet = APP_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_SvcCpTask, &attr, P_SVC_CP_Task, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_join() is failed!! (P_SVC_CP_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_SVC_CP_Task() is successfully created.");
        nRet = APP_OK;
    }

    nRet = pthread_create(&sh_SvcCpTaskTx, &attr, P_SVC_CP_TaskTx, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_join() is failed!! (P_SVC_CP_TaskTx) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_SVC_CP_TaskTx() is successfully created.");
        nRet = APP_OK;
    }

#if defined(CONFIG_PTHREAD_JOINABLE)
    nRet = pthread_join(sh_SvcCpTask, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_join() is failed!! (P_SVC_CP_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintDebug("P_SVC_CP_Task() is successfully joined.");
        nRet = APP_OK;
    }
#endif
	return nRet;
}

static int32_t P_SVC_CP_Init(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    if((s_nSvcCpTaskMsgId = msgget(s_SvcCpTaskMsgKey, IPC_CREAT|0666)) == APP_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_SVC_CP_PrintMsgInfo(s_nSvcCpTaskMsgId);
    }

    nRet = P_SVC_CP_CreateTask();
    if (nRet != APP_OK)
    {
        PrintError("P_SVC_CP_CreateTask() is failed! [nRet:%d]", nRet);
    }

    (void*)memset(&pstSvcCp->stDbManagerWrite, 0x00, sizeof(DB_MANAGER_WRITE_T));
    (void*)memset(&pstSvcCp->stMsgManagerTx, 0x00, sizeof(MSG_MANAGER_TX_T));
    (void*)memset(&pstSvcCp->stMsgManagerRx, 0x00, sizeof(MSG_MANAGER_RX_T));
    (void*)memset(&pstSvcCp->stDbV2x, 0x00, sizeof(DB_V2X_T));
    (void*)memset(&pstSvcCp->stDbV2xStatusTx, 0x00, sizeof(DB_V2X_STATUS_TX_T));
    (void*)memset(&s_stSvcCp, 0x00, sizeof(SVC_CP_T));

    nRet = P_SVC_CP_SetDefaultSettings(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_CP_SetDefaultSettings() is failed! [nRet:%d]", nRet);
    }

    nRet = P_SVC_CP_SetSettings(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
    }
    return nRet;
}

static int32_t P_SVC_CP_DeInit(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    return nRet;
}

void SVC_CP_ShowSettings(SVC_CP_T *pstSvcCp)
{
    PrintTrace("========================================================");
    PrintWarn("MSG V2X Tx Info>");
    PrintDebug(" ePayloadType[%d]", pstSvcCp->stMsgManagerTx.ePayloadType);
    PrintDebug(" eCommType[%d]", pstSvcCp->stMsgManagerTx.eCommType);
    PrintDebug(" eSignId[%d]", pstSvcCp->stMsgManagerTx.eSignId);
    PrintDebug(" eV2xFreq[%d]", pstSvcCp->stMsgManagerTx.eV2xFreq);
    PrintDebug(" ePriority[%d]", pstSvcCp->stMsgManagerTx.ePriority);
    PrintDebug(" eV2xDataRate[%d]", pstSvcCp->stMsgManagerTx.eV2xDataRate);
    PrintDebug(" eV2xTimeSlot[%d]", pstSvcCp->stMsgManagerTx.eV2xTimeSlot);
    PrintDebug(" unPsid[%d]", pstSvcCp->stMsgManagerTx.unPsid);
    PrintDebug(" nTxPower[%d]", pstSvcCp->stMsgManagerTx.nTxPower);
    PrintDebug(" unTxCount[%d]", pstSvcCp->stMsgManagerTx.unTxCount);
    PrintDebug(" unTxDelay[%d ms]", pstSvcCp->stMsgManagerTx.unTxDelay);
    for(int i = 0; i < MSG_MANAGER_MAC_LENGTH; i++)
    {
        PrintDebug(" unTxCount[i:%d][0x%x]", i, pstSvcCp->stMsgManagerTx.uchPeerMacAddr[i]);
    }

    PrintDebug(" unTransmitterProfileId[%d]", pstSvcCp->stMsgManagerTx.unTransmitterProfileId);
    PrintDebug(" unPeerL2Id[%d]", pstSvcCp->stMsgManagerTx.unPeerL2Id);

    PrintWarn("DB V2X Info>");
    PrintDebug(" eDeviceType[%d]", pstSvcCp->stDbV2x.eDeviceType);
    PrintDebug(" eTeleCommType[%d]", pstSvcCp->stDbV2x.eTeleCommType);
    PrintDebug(" unDeviceId[%d]", pstSvcCp->stDbV2x.unDeviceId);
    PrintDebug(" eServiceId[%d]", pstSvcCp->stDbV2x.eServiceId);
    PrintDebug(" eActionType[%d]", pstSvcCp->stDbV2x.eActionType);
    PrintDebug(" eRegionId[%d]", pstSvcCp->stDbV2x.eRegionId);
    PrintDebug(" ePayloadType[%d]", pstSvcCp->stDbV2x.ePayloadType);
    PrintDebug(" eCommId[%d]", pstSvcCp->stDbV2x.eCommId);
    PrintDebug(" usDbVer[%d.%d]", pstSvcCp->stDbV2x.usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, pstSvcCp->stDbV2x.usDbVer & CLI_DB_V2X_MINOR_MASK);
    PrintDebug(" usHwVer[0x%x]", pstSvcCp->stDbV2x.usHwVer);
    PrintDebug(" usSwVer[0x%x]", pstSvcCp->stDbV2x.usSwVer);

    PrintWarn("Device Info>");
    PrintDebug("Ethernet Interface [%s]", pstSvcCp->pchIfaceName);
    PrintDebug("PSID [%d]", pstSvcCp->unPsid);
    PrintDebug("pchIpAddr [%s]", pstSvcCp->pchIpAddr);
    PrintDebug("unPort [%d]", pstSvcCp->unPort);
    PrintDebug("pchDeviceName [%s]", pstSvcCp->pchDeviceName);
    PrintDebug("ulDbStartTime [%ld]", pstSvcCp->ulDbStartTime);
    PrintDebug("ulDbEndTime [%ld]", pstSvcCp->ulDbEndTime);
    PrintDebug("unDbTotalWrittenTime [%d]", pstSvcCp->unDbTotalWrittenTime);

    PrintWarn("V2X Status Tx Info>");
    PrintDebug(" stDbV2xDevL1.ulTimeStamp [%ld]", pstSvcCp->stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp);
    PrintDebug(" stDbV2xDevL2.ulTimeStamp [%ld]", pstSvcCp->stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp);
    PrintDebug(" stDbV2xDevL3.ulTimeStamp [%ld]", pstSvcCp->stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp);
    PrintDebug(" unRxTargetDeviceId [%d]", pstSvcCp->stDbV2xStatusTx.unRxTargetDeviceId);
    PrintDebug(" usTxFreq [%d]", pstSvcCp->stDbV2xStatusTx.usTxFreq);
    PrintDebug(" ucTxPwr [%d]", pstSvcCp->stDbV2xStatusTx.ucTxPwr);
    PrintDebug(" ucTxBw [%d]", pstSvcCp->stDbV2xStatusTx.ucTxBw);
    PrintDebug(" ucScs [%d]", pstSvcCp->stDbV2xStatusTx.ucScs);
    PrintDebug(" ucMcs [%d]", pstSvcCp->stDbV2xStatusTx.ucMcs);
    PrintDebug(" usTxRatio [%d]", pstSvcCp->stDbV2xStatusTx.usTxRatio);
    PrintDebug(" nTxLatitude [%d]", pstSvcCp->stDbV2xStatusTx.stTxPosition.nTxLatitude);
    PrintDebug(" nTxLongitude [%d]", pstSvcCp->stDbV2xStatusTx.stTxPosition.nTxLongitude);
    PrintDebug(" nTxAttitude [%d]", pstSvcCp->stDbV2xStatusTx.stTxPosition.nTxAttitude);
    PrintDebug(" unSeqNum [%d]", pstSvcCp->stDbV2xStatusTx.unSeqNum);
    PrintDebug(" unContCnt [%d]", pstSvcCp->stDbV2xStatusTx.unContCnt);
    PrintDebug(" unTxVehicleSpeed [%d]", pstSvcCp->stDbV2xStatusTx.unTxVehicleSpeed);
    PrintDebug(" unTxVehicleHeading [%d]", pstSvcCp->stDbV2xStatusTx.unTxVehicleHeading);

    PrintTrace("========================================================");
}

int32_t SVC_CP_GetSettings(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_CP_GetSettings(pstSvcCp);
    if (nRet != APP_OK)
    {
        PrintError("P_SVC_CP_GetSettings() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

int32_t SVC_CP_SetSettings(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_CP_SetSettings(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

int32_t SVC_CP_SetLog(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    s_stSvcCp.bLogLevel = pstSvcCp->bLogLevel;
    PrintTrace("SET:s_stSvcCp.bLogLevel [%s]", s_stSvcCp.bLogLevel == ON ? "ON" : "OFF");

    nRet = APP_OK;

    return nRet;
}

int32_t SVC_CP_Open(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;
    int32_t nFrameWorkRet = FRAMEWORK_ERROR;
    DB_MANAGER_T *pstDbManager;
    MSG_MANAGER_T *pstMsgManager;
    DI_T *pstDi;
    uint32_t nRetryCnt = 0;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    pstDbManager = FRAMEWORK_GetDbManagerInstance();
    if(pstDbManager == NULL)
    {
        PrintError("pstDbManager == NULL!!");
        nRet = APP_ERROR;
        return nRet;
    }

    pstDbManager->eFileType = pstSvcCp->stDbManagerWrite.eFileType;
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

    pstMsgManager->eDeviceType = pstSvcCp->stDbV2x.eDeviceType;
    PrintDebug("eDeviceType[%d]", pstMsgManager->eDeviceType);
    pstMsgManager->pchIfaceName = pstSvcCp->pchIfaceName;
    pstMsgManager->stExtMsgWsr.unPsid = pstSvcCp->unPsid;
    pstMsgManager->pchIpAddr = pstSvcCp->pchIpAddr;
    pstMsgManager->unPort = pstSvcCp->unPort;

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

    for(nRetryCnt = 0; nRetryCnt < SVC_CP_GPS_OPEN_RETRY_CNT; nRetryCnt++)
    {
        nRet = DI_GPS_Open(&pstDi->stDiGps);
        if (nRet != DI_OK)
        {
            PrintError("DI_GPS_Open() is failed! [nRet:%d], nRetryCnt[%d/%d], retry after [%d us]", nRet, nRetryCnt, SVC_CP_GPS_OPEN_RETRY_CNT, SVC_CP_GPS_OPEN_RETRY_DELAY * USLEEP_MS);
            usleep(SVC_CP_GPS_OPEN_RETRY_DELAY * USLEEP_MS);
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

int32_t SVC_CP_Close(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;
    DI_T *pstDi;
    DB_MANAGER_T *pstDbManager;
    MSG_MANAGER_T *pstMsgManager;
    char chTempDate[SVC_CP_DATE_LEN+1];
    char chTempHour[SVC_CP_HOUR_LEN+1];
    char chTempMin[SVC_CP_MIN_LEN+1];
    char chTempSec[SVC_CP_SEC_LEN+1];

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
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

    pstMsgManager->pchIfaceName = pstSvcCp->pchIfaceName;
    pstMsgManager->stExtMsgWsr.unPsid = pstSvcCp->unPsid;

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
    sprintf(s_chStrBufTxRxType, "%s", SVC_CP_DB_TX);
    pstDbManager->stDbFile.pchTxRxType = s_chStrBufTxRxType;
    if(s_stSvcCp.stDbV2x.eDeviceType == DB_V2X_DEVICE_TYPE_OBU)
    {

        sprintf(s_chStrBufDevType, "%s", SVC_CP_DEV_OBU);
        PrintTrace("CONFIG_OBU is enabled, s_chStrBufDevType [%s]", s_chStrBufDevType);
    }
    else if(s_stSvcCp.stDbV2x.eDeviceType == DB_V2X_DEVICE_TYPE_RSU)
    {
        sprintf(s_chStrBufDevType, "%s", SVC_CP_DEV_RSU);
        PrintTrace("CONFIG_RSU is enabled, s_chStrBufDevType [%s]", s_chStrBufDevType);
    }
    else
    {
        sprintf(s_chStrBufDevType, "%s", SVC_CP_DEV_UNKNOWN);
        PrintError("unknown device type[%d]", s_stSvcCp.stDbV2x.eDeviceType);
    }
    pstDbManager->stDbFile.pchDeviceType = s_chStrBufDevType;

    sprintf(s_chStrBufDevId, "%s%s", DB_V2X_DEVICE_ID_PREFIX, s_stSvcCp.pchDeviceName);
    pstDbManager->stDbFile.pchDeviceId = s_chStrBufDevId;

    sprintf(s_chStrBufStartTime, "%ld", s_stSvcCp.ulDbStartTime);
    strncpy(chTempDate, s_chStrBufStartTime, SVC_CP_DATE_LEN);
    chTempDate[SVC_CP_DATE_LEN] = '\0';
    strncpy(chTempHour, s_chStrBufStartTime + SVC_CP_DATE_LEN, SVC_CP_HOUR_LEN);
    chTempHour[SVC_CP_HOUR_LEN] = '\0';
    strncpy(chTempMin, s_chStrBufStartTime + SVC_CP_DATE_LEN + SVC_CP_HOUR_LEN, SVC_CP_MIN_LEN);
    chTempMin[SVC_CP_MIN_LEN] = '\0';
    strncpy(chTempSec, s_chStrBufStartTime + SVC_CP_DATE_LEN + SVC_CP_HOUR_LEN + SVC_CP_MIN_LEN, SVC_CP_SEC_LEN);
    chTempSec[SVC_CP_SEC_LEN] = '\0';
    sprintf(s_chStrBufStartTime, "%s-%s-%s-%s", chTempDate, chTempHour, chTempMin, chTempSec);
    pstDbManager->stDbFile.pchStartTime = s_chStrBufStartTime;

    sprintf(s_chStrBufEndTime, "%ld", s_stSvcCp.ulDbEndTime);
    strncpy(chTempDate, s_chStrBufEndTime, SVC_CP_DATE_LEN);
    chTempDate[SVC_CP_DATE_LEN] = '\0';
    strncpy(chTempHour, s_chStrBufEndTime + SVC_CP_DATE_LEN, SVC_CP_HOUR_LEN);
    chTempHour[SVC_CP_HOUR_LEN] = '\0';
    strncpy(chTempMin, s_chStrBufEndTime + SVC_CP_DATE_LEN + SVC_CP_HOUR_LEN, SVC_CP_MIN_LEN);
    chTempMin[SVC_CP_MIN_LEN] = '\0';
    strncpy(chTempSec, s_chStrBufEndTime + SVC_CP_DATE_LEN + SVC_CP_HOUR_LEN + SVC_CP_MIN_LEN, SVC_CP_SEC_LEN);
    chTempSec[SVC_CP_SEC_LEN] = '\0';

    sprintf(s_chStrBufEndTime, "%s-%s-%s-%s", chTempDate, chTempHour, chTempMin, chTempSec);
    pstDbManager->stDbFile.pchEndTime = s_chStrBufEndTime;

    sprintf(s_chStrBufTotalTime, "%d%s", s_stSvcCp.unDbTotalWrittenTime, "secs");
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
    sprintf(s_chStrBufTxRxType, "%s", SVC_CP_DB_RX);
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

int32_t SVC_CP_Start(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;
    SVC_CP_EVENT_MSG_T stEventMsg;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    (void*)memset(&stEventMsg, 0x00, sizeof(SVC_CP_EVENT_MSG_T));

    stEventMsg.eEventType = SVC_CP_EVENT_START;

    nRet = msgsnd(s_nSvcCpTaskMsgId, &stEventMsg, sizeof(SVC_CP_EVENT_MSG_T), IPC_NOWAIT);
    if(nRet < 0)
    {
        PrintError("msgsnd() is failed!!, [nRet:%d]", nRet);
        return nRet;
    }
    else
    {
        nRet = APP_OK;
    }

    return nRet;
}

int32_t SVC_CP_Stop(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;
    SVC_CP_EVENT_MSG_T stEventMsg;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    (void*)memset(&stEventMsg, 0x00, sizeof(SVC_CP_EVENT_MSG_T));

    stEventMsg.eEventType = SVC_CP_EVENT_STOP;

    nRet = msgsnd(s_nSvcCpTaskMsgId, &stEventMsg, sizeof(SVC_CP_EVENT_MSG_T), IPC_NOWAIT);
    if(nRet < 0)
    {
        PrintError("msgsnd() is failed!!, [nRet:%d]", nRet);
        return nRet;
    }

    else
    {
        nRet = APP_OK;
    }

    return nRet;
}

int32_t SVC_CP_Status(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    PrintWarn("TODO");

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t SVC_CP_Init(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_CP_Init(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_CP_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    s_stSvcCp.bLogLevel = pstSvcCp->bLogLevel;
    PrintDebug("s_stSvcCp.bLogLevel [%s]", s_stSvcCp.bLogLevel == ON ? "ON" : "OFF");

    return nRet;
}

int32_t SVC_CP_DeInit(SVC_CP_T *pstSvcCp)
{
    int32_t nRet = APP_ERROR;

    if(pstSvcCp == NULL)
    {
        PrintError("pstSvcCp == NULL!!");
        return nRet;
    }

    nRet = P_SVC_CP_DeInit(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("P_SVC_CP_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

