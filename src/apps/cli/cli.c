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
* @file cli.c
*
* This file contains a CLI design
*
* @note
*
* CLI Source File
*
*
******************************************************************************/

/***************************** Include ***************************************/
#include "cli.h"

/***************************** Definition ************************************/


/***************************** Static Variable *******************************/
static int s_nInputString = 0;
static int S_nCliShow = 0;


/***************************** Function  *************************************/
static void P_CLI_StartLoop(void);
static int P_CLI_ExecuteCmd(CLI_UTIL_QUEUE_T *head);
static int P_CLI_ProcessCmd(CLI_UTIL_QUEUE_T *head);
static int P_CLI_StartCmd(char *str);
static int P_CLI_ParseInputString(char *prompt, char *str, int len);
static int P_CLI_ReadInputString(char *prompt, char *str, int maxlen);
static int P_CLI_ReadInputString(char *prompt, char *str, int maxlen);
static uint32_t P_CLI_InitCmd(void);

static int P_CLI_ExecuteCmd(CLI_UTIL_QUEUE_T *head)
{
    int res;
    CLI_CMDLINE_T cmd;

    res = CLI_CMD_CheckLookUp(head, &cmd);
    if (res == 0)
    {
        res = CLI_CMD_CheckValid(&cmd, cmd.switches);
        if (res != -1)
        {
            PrintError("Invalid switch: %s\n", CLI_CMD_CheckName(&cmd, res));
            return -8;
        }

        res = (*cmd.func)(&cmd, cmd.argc - cmd.argidx, &(cmd.argv[cmd.argidx]));
    }

    CLI_CMD_Free(&cmd);
    return res;
}

static int P_CLI_ProcessCmd(CLI_UTIL_QUEUE_T *head)
{
    CLI_UTIL_QUEUE_T cmdqueue;
    CLI_LIST_T *cmd;
    int status = APP_ERR_BLANK;
    int term;

    CLI_UTIL_InitQueue(&cmdqueue);

    while ((cmd = CLI_CMD_Read(head)))
    {

        if (cmd == NULL)
        {
            return APP_ERR_BLANK;
        }

        CLI_UTIL_Enqueue(&cmdqueue, (CLI_UTIL_QUEUE_T *)cmd);
    }

    while ((cmd = (CLI_LIST_T *)CLI_UTIL_DequeueNext(&(cmdqueue))))
    {
        status = P_CLI_ExecuteCmd(&(cmd->head));
        term = cmd->term;

        free(cmd);
        if (status == APP_ERR_BLANK)
        {
            continue;
        }

        if ((term == APP_CLI_CMD_AND) && (status != 0))
        {
            break;
        }

        if ((term == APP_CLI_CMD_OR) && (status == 0))
        {
            break;
        }
    }

    while ((cmd = (CLI_LIST_T *)CLI_UTIL_DequeueNext(&(cmdqueue))))
    {
        CLI_CMD_FreeTokens(&(cmd->head));
        free(cmd);
    }

    return status;
}

int P_CLI_StartCmd(char *str)
{
    CLI_UTIL_QUEUE_T cmd_list;
    int res;

    CLI_CMD_BuildList(&cmd_list, str);

    CLI_CMD_SetList(&cmd_list);

    res = P_CLI_ProcessCmd(&cmd_list);

    CLI_CMD_FreeTokens(&cmd_list);

    return res;
}

static int P_CLI_ParseInputString(char *prompt, char *str, int len)
{
    int reading = 1;
    char ch;
    int idx = 0;

    s_nInputString++;

    if (prompt && *prompt)
    {
        printf(COLOR_YELLOW);
        fputs(prompt, stdout);
        printf(COLOR_RESET);
    }

    while (reading)
    {
        if (S_nCliShow)
        {
            if (prompt && *prompt)
            {
                puts(prompt);
            }

            puts(str);
            S_nCliShow = false;
            continue;
        }

        ch = getchar();

        if (ch < 0)
        {
            break;
        }

        if (ch == 0)
        {
            continue;
        }

        switch (ch)
        {
        case 3: /* Ctrl-C */
            puts("^C\r\n");
            S_nCliShow = true;
            idx = 0;
            break;

        case 0x7f:
        case '\b':
            if (idx > 0)
            {
                idx--;
                puts("\b \b");
            }
            break;

        case 21: /* Ctrl-U */
            while (idx > 0)
            {
                idx--;
                puts("\b \b");
            }
            break;

        case '\r':
        case '\n':
            reading = 0;
            break;

        default:
            if (ch >= ' ')
            {
                if (idx < (len - 1))
                {
                    str[idx] = ch;
                    idx++;
                }
            }
            break;
        }
    }

    s_nInputString--;

    str[idx] = 0;
    return idx;
}

static int P_CLI_ReadInputString(char *prompt, char *str, int maxlen)
{
    str[0] = '\0';
    return P_CLI_ParseInputString(prompt, str, maxlen);
}

static void P_CLI_StartLoop(void)
{
    char buffer[300];
    int status;
    char *prompt;

    for (;;)
    {
        prompt = "CLI> ";
        P_CLI_ReadInputString(prompt, buffer, sizeof(buffer));

        status = P_CLI_StartCmd(buffer);
        if (status != APP_ERR_BLANK)
        {
#if defined(CONFIG_CLI_DEBUG)
            PrintDebug("status = %d", status);
#endif
        }
    }
}

static uint32_t P_CLI_InitCmd(void)
{
    uint32_t unRet = APP_ERROR;
    unRet = CLI_CMD_Init();
    if (unRet != APP_OK)
    {
        PrintError("CLI_Init() is failed! [unRet:%d]", unRet);
    }

    unRet = CLI_MSG_InitCmds();
    if (unRet != APP_OK)
    {
        PrintError("CLI_MSG_InitCmds() is failed! [unRet:%d]", unRet);
    }

    unRet = CLI_DB_InitCmds();
    if (unRet != APP_OK)
    {
        PrintError("CLI_DB_InitCmds() is failed! [unRet:%d]", unRet);
    }

    return unRet;
}

uint32_t CLI_Init(void)
{
    uint32_t unRet = APP_ERROR;

    PrintNotice("Init");

    unRet = P_CLI_InitCmd();
    if (unRet != APP_OK)
    {
        PrintError("P_CLI_InitCmd() is failed! [unRet:%d]", unRet);
        return unRet;
    }

    (void)P_CLI_StartLoop();

    return APP_OK;
}

