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
* @file cli_db.c
*
* @note
*
* CLI DB Source
*
******************************************************************************/


/***************************** Include ***************************************/
#include "cli.h"
#include "db_manager.h"
/***************************** Definition ************************************/


/***************************** Static Variable *******************************/


/***************************** Function Protype ******************************/

static int P_CLI_DB_Help(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    int nFrameWorkRet = FRAMEWORK_ERROR;
    char *pcCmd;
    DB_MANAGER_WRITE_T stDbManagerWrite;
    DB_V2X_T stDbV2x;
    char cPayload[100];

    (void*)memset(&stDbManagerWrite, 0x00, sizeof(DB_MANAGER_WRITE_T));
    (void*)memset(&stDbV2x, 0x00, sizeof(DB_V2X_T));

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
        else if(IS_CMD(pcCmd, "v2x"))
        {
            nFrameWorkRet = DB_MANAGER_Write(&stDbManagerWrite, &stDbV2x, &cPayload);
            if(nFrameWorkRet != FRAMEWORK_OK)
            {
                PrintError("DB_MANAGER_Write() is failed! [nRet:%d]", nFrameWorkRet);
            }
        }
        else
        {
            return CLI_CMD_Showusage(pstCmd);
        }
    }

	return nRet;
}

int32_t CLI_DB_InitCmds(void)
{
    int32_t nRet = APP_ERROR;

    nRet = CLI_CMD_AddCmd("db",
               P_CLI_DB_Help,
               NULL,
               "help for DB commands",
               "db [enter command]\n\n"
               "Without any parameters, the 'db' show a description\n"
               "of available commands. For more details on a command, type and enter 'db'\n"
               "and the command name.\n\n"
               "db test    test db command\n"
               "db v2x     test db v2x sample command",
               "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

