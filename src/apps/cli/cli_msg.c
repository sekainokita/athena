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
* @file cli_msg.c
*
* @note
*
* CLI MSG Source
*
******************************************************************************/


/***************************** Include ***************************************/
#include "cli.h"
#include "app.h"
#include "db_v2x.h"
#include "db_manager.h"
#include "msg_manager.h"
#include "framework.h"

/***************************** Definition ************************************/
#define SAMPLE_USAGE_STR "Usage: %s [OPTIONS]\n"                                                                  \
						 "  -o payload_type  : Raw, EncodedbyJ2735, eITSK00130, eKETI, eETRI (default : Raw)\n"   \
						 "  -p psid          : <decimal number> (default : 32)\n"                                 \
						 "  -y comm_type     : DSRC, LTEV2X, 5GNRV2X (default : 5GNRV2X)\n"                       \
						 "  -t tx_port       : tx port dbm (default : 20)\n"                                      \
						 "  -s signer_id     : UNSECURED, CERTIFICATE, DIGEST, ALTERNATE (default : UNSECURED)\n" \
						 "  -r priority      : 0~7 (default : 0)\n"                                               \
						 "  -c tx_count      : total tx count (default : 100)\n"                                  \
						 "  -d tx_delay      : msec delay (default : 100)\n"                                      \
						 "  -m total_time    : total exec second (default : 10)\n"                                \
						 "  -h help\n"


/***************************** Static Variable *******************************/


/***************************** Function Protype ******************************/
static int P_CLI_MSG_TcpRlogin(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    char *pcCmd;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return nRet;
}

static int P_CLI_MSG_TcpConnect(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    char *pcCmd;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }


    return nRet;
}

static int P_CLI_MSG_TcpListen(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    char *pcCmd;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return nRet;
}

static int P_CLI_MSG_TcpConsTest(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    char *pcCmd;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return nRet;
}

static int P_CLI_MSG_TcpTest(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    char *pcCmd;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return nRet;
}

static int P_CLI_MSG(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    int nFrameWorkRet = FRAMEWORK_ERROR;
    char *pcCmd;
    MSG_MANAGER_TX_T stMsgManagerTx;
    MSG_MANAGER_RX_T stMsgManagerRx;
    DB_V2X_T stDbV2x;
    char cPayload[CLI_DB_V2X_DEFAULT_PAYLOAD_LEN];

    (void*)memset(&stMsgManagerTx, 0x00, sizeof(MSG_MANAGER_TX_T));
    (void*)memset(&stMsgManagerRx, 0x00, sizeof(MSG_MANAGER_RX_T));
    (void*)memset(&stDbV2x, 0x00, sizeof(DB_V2X_T));
    (void*)memset(&cPayload, 0x00, sizeof(cPayload));

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
        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_0);
        if(IS_CMD(pcCmd, "test"))
        {
            for(int i = 0; i < CMD_MAX; i++)
            {
                pcCmd = CLI_CMD_GetArg(pstCmd, i);
                PrintDebug("pcCmd[idx:%d][value:%s]", i, pcCmd);
            }
        }
        else if(IS_CMD(pcCmd, "tx"))
        {
            TIME_MANAGER_T *pstTimeManager;
            uint32_t i = 0;

            stMsgManagerTx.unTxCount = 10;
            stMsgManagerTx.unTxDelay = 100;

            for (i = 0; i < stMsgManagerTx.unTxCount; i++)
            {
                stDbV2x.eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
                stDbV2x.eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5_BROADCAST;
                stDbV2x.unDeviceId = CLI_DB_V2X_DEFAULT_DEVICE_ID;

                pstTimeManager = FRAMEWORK_GetTimeManagerInstance();
                nFrameWorkRet = TIME_MANAGER_Get(pstTimeManager);
                if(nFrameWorkRet != FRAMEWORK_OK)
                {
                    PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nFrameWorkRet);
                }
                else
                {
                    PrintTrace("Get:Current a timestamp is [%ld]", pstTimeManager->ulTimeStamp);
                }
                stDbV2x.ulTimeStamp = pstTimeManager->ulTimeStamp;

                stDbV2x.eServiceId = DB_V2X_SERVICE_ID_PLATOONING;
                stDbV2x.eActionType = DB_V2X_ACTION_TYPE_REQUEST;
                stDbV2x.eRegionId = DB_V2X_REGION_ID_SEONGNAM;
                stDbV2x.ePayloadType = DB_V2X_PAYLOAD_TYPE_PLATOONING;
                stDbV2x.eCommId = DB_V2X_COMM_ID_V2V;
                stDbV2x.usDbVer = (DB_V2X_VERSION_MAJOR << CLI_DB_V2X_MAJOR_SHIFT) | DB_V2X_VERSION_MINOR;
                stDbV2x.usHwVer = CLI_DB_V2X_DEFAULT_HW_VER;
                stDbV2x.usSwVer = APP_VER;
                stDbV2x.ulPayloadLength = sizeof(cPayload);
                stDbV2x.ulPacketCrc32 = 0;

                PrintTrace("========================================================");
                PrintDebug("unTxCount[%d]", stMsgManagerTx.unTxCount);
                PrintDebug("unTxDelay[%d]", stMsgManagerTx.unTxDelay);

                PrintDebug("eDeviceType[%d]", stDbV2x.eDeviceType);
                PrintDebug("eTeleCommType[%d]", stDbV2x.eTeleCommType);
                PrintDebug("unDeviceId[0x%x]", stDbV2x.unDeviceId);
                PrintDebug("ulTimeStamp[%ld]", stDbV2x.ulTimeStamp);
                PrintDebug("eServiceId[%d]", stDbV2x.eServiceId);
                PrintDebug("eActionType[%d]", stDbV2x.eActionType);
                PrintDebug("eRegionId[%d]", stDbV2x.eRegionId);
                PrintDebug("ePayloadType[%d]", stDbV2x.ePayloadType);
                PrintDebug("eCommId[%d]", stDbV2x.eCommId);
                PrintDebug("usDbVer[%d.%d]", stDbV2x.usDbVer >> CLI_DB_V2X_MAJOR_SHIFT, stDbV2x.usDbVer & CLI_DB_V2X_MINOR_MASK);
                PrintDebug("usHwVer[0x%x]", stDbV2x.usHwVer);
                PrintDebug("usSwVer[0x%x]", stDbV2x.usSwVer);
                PrintDebug("ulPayloadLength[%d]", stDbV2x.ulPayloadLength);
                PrintDebug("cPayload");
                for(i = 0; i < (uint32_t)CLI_DB_V2X_DEFAULT_PAYLOAD_LEN; i++)
                {
                    cPayload[i] = rand();
                    printf("[%d:%d] ", i, cPayload[i]);
                }
                printf("\r\n");

                PrintDebug("ulPayloadCrc32[0x%x]", stDbV2x.ulPacketCrc32);
                PrintTrace("========================================================");

                nFrameWorkRet = MSG_MANAGER_Transmit(&stMsgManagerTx, &stDbV2x, (char*)&cPayload);
                if(nFrameWorkRet != FRAMEWORK_OK)
                {
                    PrintError("MSG_MANAGER_Transmit() is failed! [nRet:%d]", nFrameWorkRet);
                }
                else
                {
                    PrintDebug("Tx send success [%u/%u]", i + 1, stMsgManagerTx.unTxCount);
                }

                usleep((1000 * stMsgManagerTx.unTxDelay));
            }
        }
        else if(IS_CMD(pcCmd, "open"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if(pcCmd != NULL)
            {
                MSG_MANAGER_T *pstMsgManager;

                pstMsgManager = FRAMEWORK_GetMsgManagerInstance();
                PrintDebug("pstMsgManager[0x%p]", pstMsgManager);

                pstMsgManager->pchIfaceName = pcCmd;

                PrintTrace("pstMsgManager->pchIfaceName[%s]", pstMsgManager->pchIfaceName);

                nFrameWorkRet = MSG_MANAGER_Open(pstMsgManager);
                if(nFrameWorkRet != FRAMEWORK_OK)
                {
                    PrintError("MSG_MANAGER_Open() is failed! [nRet:%d]", nFrameWorkRet);
                }
            }
            else
            {
                PrintError("set an ethernet interface name, e.g. CLI> msg open eth1");
            }
        }
        else if(IS_CMD(pcCmd, "close"))
        {
            MSG_MANAGER_T *pstMsgManager;

            pstMsgManager = FRAMEWORK_GetMsgManagerInstance();
            PrintDebug("pstDbManager[0x%p]", pstMsgManager);

            nFrameWorkRet = MSG_MANAGER_Close(pstMsgManager);
            if(nFrameWorkRet != FRAMEWORK_OK)
            {
                PrintError("MSG_MANAGER_Close() is failed! [nRet:%d]", nFrameWorkRet);
            }
        }

        else
        {
            return CLI_CMD_Showusage(pstCmd);
        }
    }

	return nRet;
}


int32_t CLI_MSG_InitCmds(void)
{
    int32_t nRet = APP_ERROR;

    nRet = CLI_CMD_AddCmd("msg",
               P_CLI_MSG,
               NULL,
               "help for MSG commands",
               "msg [enter command]\n\n"
               "Without any parameters, the 'msg' show a description\n"
               "of available commands. For more details on a command, type and enter 'msg'\n"
               "and the command name."
               "and the command name.\n\n"
               "msg open [eth#]      open message protocol, connect TCP server, e.g. msg open eth1\n"
               "msg close            close message protocol\n"
               "msg tx               send v2x messages to v2x devices (set msg open before)\n",
               "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_CMD_AddCmd("tcp-rlogin",
	       P_CLI_MSG_TcpRlogin,
	       NULL,
	       "mini rlogin client.",
	       "rlogin hostname [username]\n\n"
	       "Connects to a remote system using the RLOGIN protocol.\n"
	       "The remote system must have appropriate permissions in place\n"
	       "(usually via the file '.rhosts') for CFE to connect.\n"
	       "To terminate the session, type\n"
	       "a tilde (~) character followed by a period (.)",
	       "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_CMD_AddCmd("tcp-connect",
	       P_CLI_MSG_TcpConnect,
	       NULL,
	       "TCP connection test.",
	       "tcp connect hostname [portnum]",
	       "-q;sink output, don't display on terminal|"
	       "-d;Send junk data to discard|"
	       "-nodelay;set nodelay option on socket|"
	       "-srcport=*;Specify the source port");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_CMD_AddCmd("tcp-listen",
	       P_CLI_MSG_TcpListen,
	       NULL,
	       "port listener.",
	       "tcp listen portnum",
	       "-q;sink output, don't display on terminal|"
	       "-d;Send junk data to discard|"
	       "-nodelay;set nodelay option on socket");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_CMD_AddCmd("tcp-constest",
	       P_CLI_MSG_TcpConsTest,
	       NULL,
	       "tcp console test.",
	       "tcp constest device",
	       "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_CMD_AddCmd("tcp-test",
	       P_CLI_MSG_TcpTest,
	       NULL,
	       "TCP test command.",
	       "ttcp -t [-options] host\n"
	       "ttcp -r [-options]\n\n",
	       "-t;Source a pattern to the network|"
	       "-r;Sink (discard) data from the network|"
	       "-D;Don't buffer TCP writes (TCP_NODELAY)|"
	       "-n=*;Number of buffers to send (-t only) (default 2048)|"
	       "-l=*;Size of buffer to send/receive (default 2048)|"
	       "-p=*;Port number to use (default 5001)");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

