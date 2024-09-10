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
* @file cli_cp.c
*
* @note
*
* CLI Communication Performance Source
*
******************************************************************************/


/***************************** Include ***************************************/
#include "cli.h"
#include "app.h"
#include "db_v2x.h"
#include "db_v2x_status.h"
#include "db_manager.h"
#include "framework.h"

/***************************** Definition ************************************/

/***************************** Static Variable *******************************/
static char s_chSetBufDevId[CLI_DB_V2X_DEFAULT_BUF_LEN];
static char s_chSetEth[CLI_DB_V2X_DEFAULT_BUF_LEN];
#if defined(CONFIG_EXT_DATA_FORMAT)
static char s_chSetIp[CLI_DB_V2X_DEFAULT_BUF_LEN];
#endif

/***************************** Function Protype ******************************/
void P_CLI_CP_WriteConfigToFile(FILE *h_fdModelConf, SVC_CP_T *pstSvcCp)
{
    fprintf(h_fdModelConf, "model=%s\n", CONFIG_MODEL_NAME);
    fprintf(h_fdModelConf, "pchDeviceName=%s\n", pstSvcCp->pchDeviceName);
    fprintf(h_fdModelConf, "unDeviceId=%u\n", pstSvcCp->stDbV2x.unDeviceId);
    fprintf(h_fdModelConf, "pchIfaceName=%s\n", pstSvcCp->pchIfaceName);
    fprintf(h_fdModelConf, "pchIpAddr=%s\n", pstSvcCp->pchIpAddr);
    fprintf(h_fdModelConf, "unPort=%d\n", pstSvcCp->unPort);
}

static int P_CLI_CP_SetV2xStatusScenario(CLI_CMDLINE_T *pstCmd)
{
    int32_t nRet = APP_OK;
    SVC_CP_T *pstSvcCp;
    char *pcCmd;
    char chModelNameFile[MAX_MODEL_NAME_LEN] = {0};
    FILE *h_fdModelConf;

    pstSvcCp = APP_GetSvcCpInstance();

    if(pstCmd == NULL)
    {
        PrintError("pstCmd == NULL!!");
        return CLI_CMD_Showusage(pstCmd);
    }

    nRet = SVC_CP_GetSettings(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
    if(pcCmd == NULL)
    {
        PrintError("pcCmd == NULL!!");
        return CLI_CMD_Showusage(pstCmd);
    }
    else if(strcmp(pcCmd, "dev") == 0)
    {
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
        if(pcCmd == NULL)
        {
            PrintError("pcCmd == NULL!!");
            return CLI_CMD_Showusage(pstCmd);
        }

        pstSvcCp->stDbV2x.unDeviceId = (uint32_t)atoi(pcCmd);
        sprintf(s_chSetBufDevId, "%s", pcCmd);
        pstSvcCp->pchDeviceName = s_chSetBufDevId;

        if(strcmp(pstSvcCp->pchDeviceName, DB_MGR_DEFAULT_COMM_DEV_ID) == 0)
        {
            PrintWarn("INSERT DEVICE ID is failed!");
            nRet = APP_ERROR;
        }
        else
        {
            PrintDebug("SET:unDeviceId[%d]", pstSvcCp->stDbV2x.unDeviceId);
            PrintDebug("SET:pchDeviceName[%s]", pstSvcCp->pchDeviceName);
        }

        nRet = SVC_CP_SetSettings(pstSvcCp);
        if(nRet != APP_OK)
        {
            PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
        }
    }
    else if(strcmp(pcCmd, "eth") == 0)
    {
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
        if(pcCmd == NULL)
        {
            PrintError("pcCmd == NULL!!");
            return CLI_CMD_Showusage(pstCmd);
        }

        sprintf(s_chSetEth, "%s", pcCmd);
        pstSvcCp->pchIfaceName = s_chSetEth;

        PrintDebug("SET:pchIfaceName[%s]", pstSvcCp->pchIfaceName);

        nRet = SVC_CP_SetSettings(pstSvcCp);
        if(nRet != APP_OK)
        {
            PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
        }

    }
    else if(strcmp(pcCmd, "txrat") == 0)
    {
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
        if(pcCmd == NULL)
        {
            PrintError("pcCmd == NULL!!");
            return CLI_CMD_Showusage(pstCmd);
        }

        pstSvcCp->stMsgManagerTx.unTxDelay = (uint16_t)atoi(pcCmd);
        pstSvcCp->stDbV2xStatusTx.usTxRatio = (uint16_t)atoi(pcCmd);

        PrintDebug("SET:unTxDelay[%d]", pstSvcCp->stMsgManagerTx.unTxDelay);
        PrintDebug("SET:usTxRatio[%d]", pstSvcCp->stDbV2xStatusTx.usTxRatio);

        nRet = SVC_CP_SetSettings(pstSvcCp);
        if(nRet != APP_OK)
        {
            PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
        }
    }
    else if(strcmp(pcCmd, "spd") == 0)
    {
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
        if(pcCmd == NULL)
        {
            PrintError("pcCmd == NULL!!");
            return CLI_CMD_Showusage(pstCmd);
        }

        pstSvcCp->stDbV2xStatusTx.unTxVehicleSpeed = (uint32_t)atoi(pcCmd);

        PrintDebug("SET:unTxVehicleSpeed[%d]", pstSvcCp->stDbV2xStatusTx.unTxVehicleSpeed);

        nRet = SVC_CP_SetSettings(pstSvcCp);
        if(nRet != APP_OK)
        {
            PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
        }
    }
    else if(strcmp(pcCmd, "rg") == 0)
    {
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
        if(pcCmd == NULL)
        {
            PrintError("pcCmd == NULL!!");
            return CLI_CMD_Showusage(pstCmd);
        }

        pstSvcCp->stDbV2x.eRegionId = (uint32_t)atoi(pcCmd);

        PrintDebug("SET:eRegionId[0x%x]", pstSvcCp->stDbV2x.eRegionId);

        nRet = SVC_CP_SetSettings(pstSvcCp);
        if(nRet != APP_OK)
        {
            PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
        }
    }
#if defined(CONFIG_EXT_DATA_FORMAT)
    else if(strcmp(pcCmd, "ip") == 0)
    {
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
        if(pcCmd == NULL)
        {
            PrintError("pcCmd == NULL!!");
            return CLI_CMD_Showusage(pstCmd);
        }

        sprintf(s_chSetIp, "%s", pcCmd);
        pstSvcCp->pchIpAddr = s_chSetIp;

        PrintDebug("Set:IP[%s]", pstSvcCp->pchIpAddr);

        nRet = SVC_CP_SetSettings(pstSvcCp);
        if(nRet != APP_OK)
        {
            PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
        }
    }
    else if(strcmp(pcCmd, "port") == 0)
    {
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
        if(pcCmd == NULL)
        {
            PrintError("pcCmd == NULL!!");
            return CLI_CMD_Showusage(pstCmd);
        }

        pstSvcCp->unPort = (uint32_t)atoi(pcCmd);

        PrintDebug("SET:port[%d]", pstSvcCp->unPort);

        nRet = SVC_CP_SetSettings(pstSvcCp);
        if(nRet != APP_OK)
        {
            PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
        }
    }
#endif
    else
    {
        PrintWarn("unknown set type");
        nRet = APP_ERROR;
    }

    snprintf(chModelNameFile, sizeof(chModelNameFile), "%s%s", CONFIG_MODEL_NAME, MODEL_NAME_FILE_SUFFIX);

    h_fdModelConf = fopen(chModelNameFile, "r+");
    if (h_fdModelConf == NULL)
    {
        h_fdModelConf = fopen(chModelNameFile, "w");
        if (h_fdModelConf == NULL)
        {
            PrintError("Failed to open or create file: %s", chModelNameFile);
            return APP_ERROR;
        }
        P_CLI_CP_WriteConfigToFile(h_fdModelConf, pstSvcCp);
    }
    else
    {
        char chExistingModelName[MAX_MODEL_NAME_LEN] = {0};
        if (fgets(chExistingModelName, sizeof(chExistingModelName), h_fdModelConf) != NULL)
        {
            if ((strncmp(chExistingModelName, MODEL_PREFIX, MODEL_PREFIX_LEN) != 0) || (strcmp(chExistingModelName + MODEL_PREFIX_LEN, CONFIG_MODEL_NAME) != 0))
            {
                h_fdModelConf = freopen(chModelNameFile, "w", h_fdModelConf);
                if (h_fdModelConf == NULL)
                {
                    PrintError("Failed to reopen file: %s", chModelNameFile);
                    return APP_ERROR;
                }
                P_CLI_CP_WriteConfigToFile(h_fdModelConf, pstSvcCp);
            }
        }
        else
        {
            // If we couldn't read the existing content, rewrite the file
            h_fdModelConf = freopen(chModelNameFile, "w", h_fdModelConf);
            if (h_fdModelConf == NULL)
            {
                PrintError("Failed to reopen file: %s", chModelNameFile);
                return APP_ERROR;
            }
            P_CLI_CP_WriteConfigToFile(h_fdModelConf, pstSvcCp);
        }
    }

    if (h_fdModelConf != NULL)
    {
        fclose(h_fdModelConf);
    }
    else
    {
        PrintError("h_fdModelConf is NULL!!");
    }

    return nRet;
}

static int P_CLI_CP_CheckV2xStatusScenario(void)
{
    int32_t nRet = APP_OK;
    SVC_CP_T *pstSvcCp;
    pstSvcCp = APP_GetSvcCpInstance();

    nRet = SVC_CP_GetSettings(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
    }

    (void)SVC_CP_ShowSettings(pstSvcCp);

    return nRet;
}

static int P_CLI_CP_ReadyV2xStatusScenario(void)
{
    int32_t nRet = APP_OK;
    SVC_CP_T *pstSvcCp;

    pstSvcCp = APP_GetSvcCpInstance();

    if (pstSvcCp == NULL)
    {
        PrintError("Failed to get service control point instance.");
        return APP_ERROR;
    }

    (void)SVC_CP_ShowSettings(pstSvcCp);

    nRet = SVC_CP_GetSettings(pstSvcCp);
    if (nRet != APP_OK)
    {
        PrintError("SVC_CP_GetSettings() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = SVC_CP_UpdateSettings(pstSvcCp);
    if (nRet != APP_OK)
    {
        PrintError("SVC_CP_UpdateSettings() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = SVC_CP_SetSettings(pstSvcCp);
    if (nRet != APP_OK)
    {
        PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
    }

    nRet = SVC_CP_Open(pstSvcCp);
    if (nRet != APP_OK)
    {
        PrintError("SVC_CP_Open() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    return nRet;
}


static int P_CLI_CP_StartV2xStatusScenario(void)
{
    int32_t nRet = APP_OK;
    SVC_CP_T *pstSvcCp;
    pstSvcCp = APP_GetSvcCpInstance();

    nRet = SVC_CP_Start(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("SVC_CP_Start() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

static int P_CLI_CP_StopV2xStatusScenario(void)
{
    int32_t nRet = APP_OK;
    SVC_CP_T *pstSvcCp;
    pstSvcCp = APP_GetSvcCpInstance();

    nRet = SVC_CP_Stop(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("SVC_CP_Start() is failed! [nRet:%d]", nRet);
    }

    usleep(SVC_CP_STOP_DELAY);

    nRet = SVC_CP_Close(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("SVC_CP_Close() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

static int P_CLI_CP_StartV2xStatus(bool bMsgTx, bool bLogOnOff)
{
    int32_t nRet = APP_OK;
    int nFrameWorkRet = FRAMEWORK_ERROR;
    TIME_MANAGER_T *pstTimeManager;
    DB_MANAGER_T *pstDbManager;
    MSG_MANAGER_T *pstMsgManager;
    SVC_CP_T *pstSvcCp;
    char *pchPayload = NULL;

    pstSvcCp = APP_GetSvcCpInstance();

    pstTimeManager = FRAMEWORK_GetTimeManagerInstance();

    (void)TIME_MANAGER_CheckLatencyBegin(pstTimeManager);

    pstDbManager = FRAMEWORK_GetDbManagerInstance();
    PrintDebug("pstDbManager[0x%p]", pstDbManager);

    pstDbManager->eFileType = DB_MANAGER_FILE_TYPE_CSV;
    pstDbManager->eSvcType = DB_MANAGER_SVC_TYPE_V2X_STATUS;

    nFrameWorkRet = DB_MANAGER_Open(pstDbManager);
    if(nFrameWorkRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Open() is failed! [nRet:%d]", nFrameWorkRet);
    }

    if(bMsgTx == TRUE)
    {
        pstMsgManager = FRAMEWORK_GetMsgManagerInstance();
        PrintDebug("pstMsgManager[0x%p]", pstMsgManager);

        pstMsgManager->eDeviceType = pstSvcCp->stDbV2x.eDeviceType;

        pstMsgManager->pchIfaceName = pstSvcCp->pchIfaceName;
        pstMsgManager->stExtMsgWsr.unPsid = pstSvcCp->unPsid;
        pstMsgManager->pchIpAddr = pstSvcCp->pchIpAddr;
        pstMsgManager->unPort = pstSvcCp->unPort;

        PrintTrace("pchIfaceName[%s], pchIpAddr[%s], unPort[%d]", pstMsgManager->pchIfaceName, pstMsgManager->pchIpAddr, pstMsgManager->unPort);
        PrintTrace("pchIfaceName[%s], unPsid[%d]", pstMsgManager->pchIfaceName, pstMsgManager->stExtMsgWsr.unPsid);

        nFrameWorkRet = MSG_MANAGER_Open(pstMsgManager);
        if(nFrameWorkRet != FRAMEWORK_OK)
        {
            PrintError("MSG_MANAGER_Open() is failed! [nRet:%d]", nFrameWorkRet);
        }
    }


    pstSvcCp->stDbV2x.ulPayloadLength = sizeof(pstSvcCp->stDbV2xStatusTx);

    pchPayload = (char*)malloc(sizeof(char)*pstSvcCp->stDbV2x.ulPayloadLength);
    if(pchPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    (void*)memset(pchPayload, 0x00, sizeof(sizeof(char)*pstSvcCp->stDbV2x.ulPayloadLength));

    nFrameWorkRet = TIME_MANAGER_Get(pstTimeManager);
    if(nFrameWorkRet != FRAMEWORK_OK)
    {
        PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nFrameWorkRet);
    }
    else
    {
        pstSvcCp->stDbV2x.ulTimeStamp = pstTimeManager->ulTimeStamp;

        pstSvcCp->stDbV2xStatusTx.stDbV2xDevL1.ulTimeStamp = 19840919;
        pstSvcCp->stDbV2xStatusTx.stDbV2xDevL2.ulTimeStamp = 19850501;
        pstSvcCp->stDbV2xStatusTx.stDbV2xDevL3.ulTimeStamp = pstTimeManager->ulTimeStamp;
    }

    memcpy(pchPayload, (char*)&pstSvcCp->stDbV2xStatusTx, sizeof(char)*pstSvcCp->stDbV2x.ulPayloadLength);

    pstSvcCp->stDbV2x.ulReserved = 0;

    if (bLogOnOff == TRUE)
    {
        (void)SVC_CP_ShowSettings(pstSvcCp);

        PrintTrace("========================================================");
        PrintDebug("ulTimeStamp[%ld]", pstSvcCp->stDbV2x.ulTimeStamp);
        PrintDebug("ulPayloadLength[%d]", pstSvcCp->stDbV2x.ulPayloadLength);
        PrintDebug("ulReserved[0x%x]", pstSvcCp->stDbV2x.ulReserved);
        PrintTrace("========================================================");
    }

    if(bMsgTx == TRUE)
    {
        nFrameWorkRet = MSG_MANAGER_Transmit(&pstSvcCp->stMsgManagerTx, &pstSvcCp->stDbV2x, (char*)pchPayload);
        if(nFrameWorkRet != FRAMEWORK_OK)
        {
            PrintError("MSG_MANAGER_Transmit() is failed! [nRet:%d]", nFrameWorkRet);
        }
        else
        {
            PrintDebug("Tx Success, Counts[%u], Delay[%d ms]", pstSvcCp->stMsgManagerTx.unTxCount, pstSvcCp->stMsgManagerTx.unTxDelay);
        }
    }
    else
    {
        nFrameWorkRet = DB_MANAGER_Write(&pstSvcCp->stDbManagerWrite, &pstSvcCp->stDbV2x, (char*)pchPayload);
        if(nFrameWorkRet != FRAMEWORK_OK)
        {
            PrintError("DB_MANAGER_Write() is failed! [nRet:%d]", nFrameWorkRet);
        }
    }

    (void)TIME_MANAGER_CheckLatencyEnd(pstTimeManager);
    (void)TIME_MANAGER_CheckLatencyTime("Tx Total Time", pstTimeManager);

    /* free(pchPayload) is free at the P_MSG_MANAGER_SendTxMsg() */

    return nRet;
}

static int P_CLI_CP(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    char *pcCmd;
    bool bMsgTx = FALSE, bLogOnOff = FALSE;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }
    else
    {
        if(IS_CMD(pcCmd, "test"))
        {
            for(int i = 0; i < CMD_MAX; i++)
            {
                pcCmd = CLI_CMD_GetArg(pstCmd, i);
                PrintDebug("pcCmd[idx:%d][value:%s]", i, pcCmd);
            }
        }
        else if(IS_CMD(pcCmd, "set"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if(pcCmd != NULL)
            {
                if(IS_CMD(pcCmd, "dev"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        nRet = P_CLI_CP_SetV2xStatusScenario(pstCmd);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_SetOptV2xStatusScenario() is failed![nRet:%d]", nRet);
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "eth"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        nRet = P_CLI_CP_SetV2xStatusScenario(pstCmd);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_SetOptV2xStatusScenario() is failed![nRet:%d]", nRet);
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "txrat"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        nRet = P_CLI_CP_SetV2xStatusScenario(pstCmd);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_SetOptV2xStatusScenario() is failed![nRet:%d]", nRet);
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "spd"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        nRet = P_CLI_CP_SetV2xStatusScenario(pstCmd);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_SetOptV2xStatusScenario() is failed![nRet:%d]", nRet);
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "rg"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        nRet = P_CLI_CP_SetV2xStatusScenario(pstCmd);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_SetOptV2xStatusScenario() is failed![nRet:%d]", nRet);
                        }
                    }
                }
#if defined(CONFIG_EXT_DATA_FORMAT)
                else if(IS_CMD(pcCmd, "ip"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        nRet = P_CLI_CP_SetV2xStatusScenario(pstCmd);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_SetOptV2xStatusScenario() is failed![nRet:%d]", nRet);
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "port"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        nRet = P_CLI_CP_SetV2xStatusScenario(pstCmd);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_SetOptV2xStatusScenario() is failed![nRet:%d]", nRet);
                        }
                    }
                }
#endif
                else
                {
                    return CLI_CMD_Showusage(pstCmd);
                }
            }
            else
            {
                return CLI_CMD_Showusage(pstCmd);
            }
        }
        else if(IS_CMD(pcCmd, "check") || IS_CMD(pcCmd, "get"))
        {
            nRet = P_CLI_CP_CheckV2xStatusScenario();
            if(nRet != APP_OK)
            {
                PrintError("P_CLI_CP_CheckV2xStatusScenario() is failed![nRet:%d]", nRet);
            }
        }
        else if(IS_CMD(pcCmd, "base"))
        {
            PrintTrace("Open DB");

            PrintTrace("Connect V2X Device (OBU/RSU)");

            PrintTrace("Setting Parameters");

            PrintTrace("Start V2X Tx/Rx Communication of Platooning");

            PrintTrace("Save DB");
        }
        else if(IS_CMD(pcCmd, "ready"))
        {
            nRet = P_CLI_CP_ReadyV2xStatusScenario();
            if(nRet != APP_OK)
            {
                PrintError("P_CLI_CP_ReadyV2xStatusScenario() is failed![nRet:%d]", nRet);
            }
        }
        else if(IS_CMD(pcCmd, "start"))
        {
            nRet = P_CLI_CP_StartV2xStatusScenario();
            if(nRet != APP_OK)
            {
                PrintError("P_CLI_CP_StartV2xStatusScenario() is failed![nRet:%d]", nRet);
            }
        }
        else if(IS_CMD(pcCmd, "stop"))
        {
            nRet = P_CLI_CP_StopV2xStatusScenario();
            if(nRet != APP_OK)
            {
                PrintError("P_CLI_CP_StopV2xStatusScenario() is failed![nRet:%d]", nRet);
            }
        }
        else if(IS_CMD(pcCmd, "status"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if (pcCmd != NULL)
            {
                if(IS_CMD(pcCmd, "start"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd != NULL)
                    {
                        bLogOnOff = TRUE;

                        if(IS_CMD(pcCmd, "msg"))
                        {
                            bMsgTx = TRUE;
                        }
                        else if(IS_CMD(pcCmd, "db"))
                        {
                            bMsgTx = FALSE;
                        }

                        PrintTrace("bMsgTx[%d], bLogOnOff[%d]", bMsgTx, bLogOnOff);

                        nRet = P_CLI_CP_StartV2xStatus(bMsgTx, bLogOnOff);
                        if(nRet != APP_OK)
                        {
                            PrintError("P_CLI_CP_StartV2xStatus() is failed![nRet:%d]", nRet);
                        }
                    }
                    else
                    {
                        return CLI_CMD_Showusage(pstCmd);
                    }
                }
                else
                {
                    return CLI_CMD_Showusage(pstCmd);
                }
            }
            else
            {
                return CLI_CMD_Showusage(pstCmd);
            }
        }
        else
        {
            return CLI_CMD_Showusage(pstCmd);
        }
    }

	return nRet;
}

int32_t CLI_CP_InitCmds(void)
{
    int32_t nRet = APP_ERROR;

    nRet = CLI_CMD_AddCmd("cp",
               P_CLI_CP,
               NULL,
               "help for Communication Performance commands",
               "cp [enter command]\n\n"
               "Without any parameters, the 'cp' show a description\n"
               "of available commands. For more details on a command, type and enter 'cp'\n"
               "and the command name.\n\n"
               "cp [OPTIONS]\n"
               "   test                         test cp command\n"
               "   set                          set Device ID (should be set cp ready first)\n"
               "   check                        check V2X scenario\n"
               "   base                         start a base Communication Performance scenario\n"
               "   ready                        ready V2X scenario\n"
               "   start                        start V2X scenario (should be set cp ready first)\n"
               "   stop                         stop V2X scenario\n"
               "   status start msg             start a test sample of V2X status data of msg tx\n"
               "   status start db              start a test sample of V2X status data of db\n"
               "   info                         get a status Communication Performance\n"
               "cp set [OPT] [PARAM]\n"
               "       dev   [id]               set device id Device ID\n"
               "       eth   [dev]              set ethernet i/f name\n"
               "       txrat [param ms]         set tx ratio\n"
               "       spd   [speed km/hrs]     set tx / rx vehicle speed\n"
               "       rg    [region id]        set region ID (1:SEOUL, 2:SEJONG, 3:BUSAN, 4:DAEGEON, 5:INCHEON\n"
               "                                               6:DAEGU, 7:DAEGU PG, 8:CHEONGJU, 9:SEONGNAM\n"
#if defined(CONFIG_EXT_DATA_FORMAT)
               "       ip    [ip]               set Ip (default ip : 192.168.1.11)\n"
               "       port  [port]             set port (default port : 47347)\n",
#endif
               "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

