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

/***************************** Function Protype ******************************/

static int P_CLI_CP_ReadyV2xStatusScenario(void)
{
    int32_t nRet = APP_OK;
    SVC_CP_T *pstSvcCp;
    pstSvcCp = APP_GetSvcCpInstance();

    (void)SVC_CP_ShowSettings(pstSvcCp);

    nRet = SVC_CP_Open(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("SVC_CP_Open() is failed! [nRet:%d]", nRet);
    }

    nRet = SVC_CP_SetSettings(pstSvcCp);
    if(nRet != APP_OK)
    {
        PrintError("SVC_CP_SetSettings() is failed! [nRet:%d]", nRet);
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

    nFrameWorkRet = DB_MANAGER_Open(pstDbManager);
    if(nFrameWorkRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Open() is failed! [nRet:%d]", nFrameWorkRet);
    }

    if(bMsgTx == TRUE)
    {
        pstMsgManager = FRAMEWORK_GetMsgManagerInstance();
        PrintDebug("pstMsgManager[0x%p]", pstMsgManager);

        pstMsgManager->pchIfaceName = pstSvcCp->pchIfaceName;

        PrintTrace("pstMsgManager->pchIfaceName[%s]", pstMsgManager->pchIfaceName);

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

        pstSvcCp->stDbV2xStatusTx.ulTxTimeStampL1 = 19840919;
        pstSvcCp->stDbV2xStatusTx.ulTxTimeStampL2 = 19850501;
        pstSvcCp->stDbV2xStatusTx.ulTxTimeStampL3 = pstTimeManager->ulTimeStamp;
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
        else if(IS_CMD(pcCmd, "sce"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if(pcCmd != NULL)
            {
                if(IS_CMD(pcCmd, "base"))
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
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd != NULL)
                    {
                        if(IS_CMD(pcCmd, "start"))
                        {
                            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_3);
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
               "cp test                   test cp command\n"
               "cp sce [OPTIONS]\n"
               "       base               start a base Communication Performance scenario\n"
               "       ready              ready V2X scenario\n"
               "       start              start V2X scenario (should be set cp sce ready first)\n"
               "       stop               stop V2X scenario\n"
               "       status start msg   start a test sample of V2X status data of msg tx\n"
               "       status start db    start a test sample of V2X status data of db\n"
               "cp info                   get a status Communication Performance\n",
               "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

