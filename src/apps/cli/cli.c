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
#include "termios.h"
#include "unistd.h"

/***************************** Definition ************************************/
#define CONFIG_CMD_HISTORY (1)
#if defined(CONFIG_CMD_HISTORY)
#define MAX_HISTORY 10
#define KEY_UP 65
#define KEY_DOWN 66
#endif

/***************************** Static Variable *******************************/
static int s_nInputString = 0;
static int S_nCliShow = 0;

#if defined(CONFIG_CMD_HISTORY)
static char *s_pszCmdHistory[MAX_HISTORY];
static int s_nCurrentHistorySize = 0;
static int s_nHistoryIdx = 0;
static struct termios tOriginalTermios;
#endif

/***************************** Function  *************************************/
static void P_CLI_StartLoop(void);
static int P_CLI_ExecuteCmd(CLI_UTIL_QUEUE_T *head);
static int P_CLI_ProcessCmd(CLI_UTIL_QUEUE_T *head);
static int P_CLI_StartCmd(char *str);
static int P_CLI_ParseInputString(char *prompt, char *str, int len);
#if defined(CONFIG_CMD_HISTORY)
#else
static int P_CLI_ReadInputString(char *prompt, char *str, int maxlen)
#endif
static int32_t P_CLI_InitCmd(void);

#if defined(CONFIG_CMD_HISTORY)
static int P_CLI_ParseInputString(char *pszPrompt, char *pszBuffer, int nMaxLen);
static void P_CLI_SaveCmdToHistory(const char *pszCmd);
static void P_CLI_ConfigureTerminal(void);
static void P_CLI_RestoreTerminal(void);
static void P_CLI_PrintHistoryList(void);
#endif

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

#if defined(CONFIG_CMD_HISTORY)
static int P_CLI_ParseInputString(char *pszPrompt, char *pszBuffer, int nMaxLen)
{
    int nIdx = 0;
    int nCh;
    int nRet;

    s_nInputString++;

    if (pszPrompt && *pszPrompt)
    {
        printf("\033[1;33m%s\033[0m", pszPrompt);
        fflush(stdout);
    }

    while (1)
    {
        nCh = getchar();

        if (nCh == 3)  /* Ctrl+C */
        {
            puts("^C\r\n");
            S_nCliShow = 1;
            nIdx = 0;
            break;
        }

        if (nCh == 21)  /* Ctrl+U */
        {
            while (nIdx > 0)
            {
                nIdx--;
                printf("\b \b");
            }
            continue;
        }

        if ((nCh == 0x7f) || (nCh == '\b'))
        {
            if (nIdx > 0)
            {
                nIdx--;
                printf("\b \b");
            }
            continue;
        }

        if ((nCh == '\r') || (nCh == '\n'))
        {
            pszBuffer[nIdx] = '\0';

            if (strcmp(pszBuffer, "exit") == 0)
            {
                printf("\nExiting CLI...\n");
                fflush(stdout);

                P_CLI_RestoreTerminal();
                nRet = system("clear");
                if (nRet != 0)
                {
                    PrintError("system() is failed! [nRet:%d]", nRet);
                }

                exit(0);
            }

            else if ((strcmp(pszBuffer, "z") == 0))
            {
                PrintTrace("Exit the CLI, kill the process");
                P_CLI_RestoreTerminal();
                fflush(stdout);
                exit(0);
            }

            if (nIdx > 0)
            {
                P_CLI_SaveCmdToHistory(pszBuffer);
                s_nHistoryIdx = s_nCurrentHistorySize;
            }

            s_nInputString--;
            printf("\n");
            return nIdx;
        }

        if (nCh == 27 && getchar() == '[')
        {
            switch (getchar())
            {
            case 'A':
                if (s_nHistoryIdx > 0)
                {
                    s_nHistoryIdx--;
                    strcpy(pszBuffer, s_pszCmdHistory[s_nHistoryIdx]);
                    printf("\033[2K\r\033[1;33m%s\033[0m\033[1;37m%s\033[0m", pszPrompt, pszBuffer);
                    nIdx = strlen(pszBuffer);
                }
                continue;

            case 'B':
                if (s_nHistoryIdx < s_nCurrentHistorySize - 1)
                {
                    s_nHistoryIdx++;
                    strcpy(pszBuffer, s_pszCmdHistory[s_nHistoryIdx]);
                    printf("\033[2K\r\033[1;33m%s\033[0m\033[1;37m%s\033[0m", pszPrompt, pszBuffer);
                    nIdx = strlen(pszBuffer);
                }
                continue;
            }
        }

        if (nCh >= ' ' && nIdx < (nMaxLen - 1))
        {
            pszBuffer[nIdx++] = nCh;
            putchar(nCh);
        }
    }

    s_nInputString--;
    pszBuffer[nIdx] = '\0';
    return nIdx;
}

static void P_CLI_StartLoop(void)
{
    char szBuffer[300];
    int nLen;
    char *pszPrompt = "CLI> ";

    while (1)
    {
        nLen = P_CLI_ParseInputString(pszPrompt, szBuffer, sizeof(szBuffer));

        if (nLen == 0)
        {
            continue;
        }

        if (strcmp(szBuffer, "history") == 0)
        {
            putchar('\n');
            P_CLI_PrintHistoryList();
            continue;
        }

        P_CLI_StartCmd(szBuffer);
    }
}

#else
static int P_CLI_ParseInputString(char *prompt, char *str, int len)
{
    int reading = 1;
    int ch;
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

        case 'z':
            PrintTrace("Exit the CLI, kill the process");
            exit(0);

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
#endif

static int32_t P_CLI_InitCmd(void)
{
    int32_t nRet = APP_ERROR;
    nRet = CLI_CMD_Init();
    if (nRet != APP_OK)
    {
        PrintError("CLI_Init() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_MSG_InitCmds();
    if (nRet != APP_OK)
    {
        PrintError("CLI_MSG_InitCmds() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_DB_InitCmds();
    if (nRet != APP_OK)
    {
        PrintError("CLI_DB_InitCmds() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_PLATOONING_InitCmds();
    if (nRet != APP_OK)
    {
        PrintError("CLI_PLATOONING_InitCmds() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_CP_InitCmds();
    if (nRet != APP_OK)
    {
        PrintError("CLI_CP_InitCmds() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_MCP_InitCmds();
    if (nRet != APP_OK)
    {
        PrintError("CLI_MCP_InitCmds() is failed! [nRet:%d]", nRet);
    }

    nRet = CLI_DI_InitCmds();
    if (nRet != APP_OK)
    {
        PrintError("CLI_DI_InitCmds() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

int32_t CLI_Init(void)
{
    int32_t nRet = APP_ERROR;

#if defined(CONFIG_CMD_HISTORY)
    P_CLI_ConfigureTerminal();
#endif

    PrintNotice("Init");

    nRet = P_CLI_InitCmd();
    if (nRet != APP_OK)
    {
        PrintError("P_CLI_InitCmd() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    (void)P_CLI_StartLoop();

    return APP_OK;
}

#if defined(CONFIG_CMD_HISTORY)
static void P_CLI_SaveCmdToHistory(const char *pszCmd)
{
    if (s_nCurrentHistorySize < MAX_HISTORY)
    {
        s_pszCmdHistory[s_nCurrentHistorySize++] = strdup(pszCmd);
    }

    else
    {
        free(s_pszCmdHistory[0]);

        for (int nIdx = 1; nIdx < MAX_HISTORY; nIdx++)
        {
            s_pszCmdHistory[nIdx - 1] = s_pszCmdHistory[nIdx];
        }

        s_pszCmdHistory[MAX_HISTORY - 1] = strdup(pszCmd);
    }

    s_nHistoryIdx = s_nCurrentHistorySize;
}

static void P_CLI_ConfigureTerminal(void)
{
    tcgetattr(STDIN_FILENO, &tOriginalTermios);
    struct termios newTermios = tOriginalTermios;
    newTermios.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newTermios);
}

static void P_CLI_RestoreTerminal(void)
{
    tcsetattr(STDIN_FILENO, TCSANOW, &tOriginalTermios);
    printf("\nTerminal restored.\n");
}

static void P_CLI_PrintHistoryList(void)
{
    printf("\033[1;37m     History list[command]\n");

    for(int nIdx = 0; nIdx < s_nCurrentHistorySize; nIdx++)
    {
        printf("     %d. %s\n", nIdx + 1, s_pszCmdHistory[nIdx]);
    }

    printf("\033[0m");
}
#endif