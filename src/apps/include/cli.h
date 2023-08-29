#ifndef	_CLI_H_
#define	_CLI_H_

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
* @file cli.h
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
#include "type.h"
#include "cli_util.h"
#include "cli_msg.h"
#include "cli_db.h"
#include "cli_platooning.h"
#include "cli_cp.h"
#include "cli_di.h"

/***************************** Definition ************************************/
//#define CONFIG_CLI_DEBUG (1)

#define isQuote(x) (((x) == '\'') || ((x) == '"'))
#define CLI_MAX_SWITCHES        (16)
#define CLI_MAX_TOKENS          (64)

#define CMD_0   0
#define CMD_1   1
#define CMD_2   2
#define CMD_3   3
#define CMD_4   4
#define CMD_5   5
#define CMD_6   6
#define CMD_7   7
#define CMD_8   8
#define CMD_9   9
#define CMD_MAX 10

#define IS_CMD(str1, str2) (strcmp((str1), (str2)) == 0)

/***************************** Enum and Structure ****************************/
typedef struct CLI_CMD_SW_t
{
    int swidx;
    char *swname;
    char *swvalue;
} CLI_CMD_SW_T;

typedef struct CLI_CMDLINE_t
{
    int argc;
    char *argv[CLI_MAX_TOKENS];
    int swc;
    CLI_CMD_SW_T swv[CLI_MAX_SWITCHES];
    int (*func)(struct CLI_CMDLINE_t *, int argc, char *argv[]);
    int argidx;
    char *ref;
    char *usage;
    char *switches;
} CLI_CMDLINE_T;

typedef struct CLI_LIST_t
{
    CLI_UTIL_QUEUE_T list;
    int term;
    char *ptr;
    CLI_UTIL_QUEUE_T head;
} CLI_LIST_T;

typedef struct CLI_CMD_TOKEN_t
{
    CLI_UTIL_QUEUE_T qb;
    char token;
} CLI_CMD_TOKEN_T;

typedef struct CLI_CMD_t
{
	struct CLI_CMD_t *sibling;
	struct CLI_CMD_t *child;
	char *cmdword;
	int (*func)(CLI_CMDLINE_T *, int argc, char *argv[]);
	void *ref;
	char *help;
	char *usage;
	char *switches;
} CLI_CMD_T;

/***************************** Function  *************************************/
char *CLI_CMD_CheckName(CLI_CMDLINE_T *cmd, int swidx);
void CLI_CMD_Free(CLI_CMDLINE_T *cmd);
int CLI_CMD_CheckValid(CLI_CMDLINE_T *cmd, char *validstr);
int32_t CLI_CMD_AddCmd(char *command,     int (*func)(CLI_CMDLINE_T *, int argc, char *argv[]), void *ref, char *help, char *usage, char *switches);
int CLI_CMD_CheckLookUp(CLI_UTIL_QUEUE_T *head, CLI_CMDLINE_T *cmd);
int32_t CLI_CMD_Init(void);
CLI_LIST_T *CLI_CMD_Read(CLI_UTIL_QUEUE_T *head);
void CLI_CMD_BuildList(CLI_UTIL_QUEUE_T *qb, char *buf);
void CLI_CMD_SetList(CLI_UTIL_QUEUE_T *qb);
void CLI_CMD_FreeTokens(CLI_UTIL_QUEUE_T *list);
void CLI_CMD_BuildCmdline(CLI_UTIL_QUEUE_T *head, CLI_CMDLINE_T *cmd);
char *CLI_CMD_GetArg(CLI_CMDLINE_T *pstCmd, int nArgNum);
int CLI_CMD_Showusage(CLI_CMDLINE_T *pstCmd);

int32_t CLI_Init(void);

#endif /* _CLI_H_ */

