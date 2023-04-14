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

/***************************** Definition ************************************/


/***************************** Static Variable *******************************/


/***************************** Function Protype ******************************/
static int P_CLI_MSG_TcpRlogin(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    uint32_t unRet = APP_OK;

    char *pcCmd;

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return unRet;
}

static int P_CLI_MSG_TcpConnect(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    uint32_t unRet = APP_OK;

    char *pcCmd;

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }


    return unRet;
}

static int P_CLI_MSG_TcpListen(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    uint32_t unRet = APP_OK;

    char *pcCmd;

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return unRet;
}

static int P_CLI_MSG_TcpConsTest(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    uint32_t unRet = APP_OK;

    char *pcCmd;

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return unRet;
}

static int P_CLI_MSG_TcpTest(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    uint32_t unRet = APP_OK;

    char *pcCmd;

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }

    return unRet;
}

static int P_CLI_MSG_Help(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    uint32_t unRet = APP_OK;

    char *pcCmd;

    pcCmd = CLI_CMD_GetArg(pstCmd, 0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }
    else
    {
        for(int i = 0; i < CMD_MAX; i++)
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, i);
            PrintDebug("pcCmd[idx:%d][value:%s]", i, pcCmd);
        }
    }

	return unRet;
}

uint32_t CLI_MSG_InitCmds(void)
{
    uint32_t unRet = APP_ERROR;

    unRet = CLI_CMD_AddCmd("msg",
               P_CLI_MSG_Help,
               NULL,
               "help for MSG commands",
               "msg [enter command]\n\n"
               "Without any parameters, the 'msg' show a description\n"
               "of available commands. For more details on a command, type and enter 'msg'\n"
               "and the command name.",
               "");
    if(unRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
    }

    unRet = CLI_CMD_AddCmd("tcp-rlogin",
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
    if(unRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
    }

    unRet = CLI_CMD_AddCmd("tcp-connect",
	       P_CLI_MSG_TcpConnect,
	       NULL,
	       "TCP connection test.",
	       "tcp connect hostname [portnum]",
	       "-q;sink output, don't display on terminal|"
	       "-d;Send junk data to discard|"
	       "-nodelay;set nodelay option on socket|"
	       "-srcport=*;Specify the source port");
    if(unRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
    }

    unRet = CLI_CMD_AddCmd("tcp-listen",
	       P_CLI_MSG_TcpListen,
	       NULL,
	       "port listener.",
	       "tcp listen portnum",
	       "-q;sink output, don't display on terminal|"
	       "-d;Send junk data to discard|"
	       "-nodelay;set nodelay option on socket");
    if(unRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
    }

    unRet = CLI_CMD_AddCmd("tcp-constest",
	       P_CLI_MSG_TcpConsTest,
	       NULL,
	       "tcp console test.",
	       "tcp constest device",
	       "");
    if(unRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
    }

    unRet = CLI_CMD_AddCmd("tcp-test",
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
    if(unRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
    }

    return unRet;
}

