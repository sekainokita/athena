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
* @file cli_cmd.c
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
CLI_CMD_T *sh_pstCliCmd;

/***************************** Static Variable *******************************/
static char *s_cTokenBreaks = " =\t\n\'\"&|;";
static char *s_cSpaceChars = " \t";

static char *P_CLI_CMD_RemoveQuotedArg(CLI_UTIL_QUEUE_T *head, CLI_CMD_TOKEN_T *t);

/***************************** Function  *************************************/

static char *P_CLI_CMD_Strdup(char *str)
{
	char *buf;

	buf = malloc(strlen(str) + 1);
	if (buf)
	{
		strcpy(buf, str);
	}

	return buf;
}

static inline int P_CLI_CMD_IsWhiteSpace(CLI_CMD_TOKEN_T *t)
{
	return (strchr(s_cSpaceChars, t->token) != NULL);
}

char *CLI_CMD_CheckName(CLI_CMDLINE_T *cmd, int swidx)
{
	if ((swidx < 0) || (swidx >= cmd->swc))
    {
		return NULL;
    }

	return cmd->swv[swidx].swname;
}

void CLI_CMD_Free(CLI_CMDLINE_T *cmd)
{
	int idx;

	for (idx = 0; idx < cmd->argc; idx++)
	{
		free(cmd->argv[idx]);
	}

	for (idx = 0; idx < cmd->swc; idx++)
	{
		free(cmd->swv[idx].swname);
	}

	cmd->argc = 0;
	cmd->swc = 0;
}

int CLI_CMD_CheckValid(CLI_CMDLINE_T *cmd, char *validstr)
{
	char *vdup;
	char *vptr;
	char *vnext;
	char atype;
	char *x;
	int idx;
	int valid;

	if (cmd->swc == 0)
    {
		return -1;
    }

	vdup = strdup(validstr);

	for (idx = 0; idx < cmd->swc; idx++)
	{
		vptr = vdup;

		vnext = vptr;
		valid = 0;

		while (vnext)
		{
			x = strchr(vptr, '|');
			if (x)
			{
				*x = '\0';
				vnext = x + 1;
			}
			else
			{
				vnext = NULL;
			}

			x = strchr(vptr, '=');
			if (x)
			{
				atype = *(x + 1);
				*x = 0;
			}
			else
			{
				if ((x = strchr(vptr, ';')))
					*x = 0;
				atype = 0;
			}

			if (strcmp(vptr, cmd->swv[idx].swname) == 0)
			{
				if ((atype == 0) && (cmd->swv[idx].swvalue == NULL))
				{
					valid = 1;
				}

				if ((atype != 0) && (cmd->swv[idx].swvalue != NULL))
				{
					valid = 1;
				}
				strcpy(vdup, validstr);
				break;
			}

			strcpy(vdup, validstr);
			vptr = vnext;
		}

		if (valid == 0)
		{
			free(vdup);
			return idx;
		}
	}

	free(vdup);

	return -1;
}

static CLI_CMD_T *CLI_CMD_FindWord(CLI_CMD_T *list, char *cmdword)
{
	if ((list != NULL) && (list->cmdword != NULL) && (cmdword != NULL))
	{
		while (list)
		{
			if (strcmp(cmdword, list->cmdword) == 0)
			{
				return list;
			}
			list = list->sibling;

			if (list->sibling == NULL)
			{
#if defined(CONFIG_CLI_DEBUG)
				PrintError("sibling is NULL!!");
#endif
				break;
			}
		}
	}
	else
	{
#if defined(CONFIG_CLI_DEBUG)
		PrintError("list[%p], list->cmdword[%p], cmdword[%p], NULL!!", list, list->cmdword, cmdword);
#endif
	}

	return NULL;
}

void CLI_CMD_BuildCmdline(CLI_UTIL_QUEUE_T *head, CLI_CMDLINE_T *cmd)
{
	CLI_CMD_TOKEN_T *t;
	CLI_CMD_TOKEN_T *next;

	memset(cmd, 0, sizeof(CLI_CMDLINE_T));

	t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(head);
	while (t != NULL)
	{
		if (P_CLI_CMD_IsWhiteSpace(t))
		{
			/* do nothing */
		}
		else if (t->token != '-')
		{
			if (cmd->argc < CLI_MAX_TOKENS)
			{
				cmd->argv[cmd->argc] = P_CLI_CMD_RemoveQuotedArg(head, t);
				cmd->argc++;
			}
		}
		else
		{
			if (cmd->swc < CLI_MAX_SWITCHES)
			{
				cmd->swv[cmd->swc].swname = P_CLI_CMD_Strdup(&(t->token));

				if (t->qb.q_next != head)
				{
					next = (CLI_CMD_TOKEN_T *)t->qb.q_next;
					if (next->token == '=')
					{
						free(t);
						t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(head);
						free(t);
						t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(head);
						if (t != NULL)
						{
							cmd->swv[cmd->swc].swvalue = P_CLI_CMD_RemoveQuotedArg(head, t);
						}
					}
					else
					{
						cmd->swv[cmd->swc].swvalue = NULL;
					}
				}

				cmd->swv[cmd->swc].swidx = cmd->argc;
				cmd->swc++;
			}
		}

		free(t);

		t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(head);
	}
}

uint32_t CLI_CMD_AddCmd(char *command, int (*func)(CLI_CMDLINE_T *, int argc, char *argv[]), void *ref, char *help, char *usage, char *switches)
{
    uint32_t unRet = APP_ERROR;
	CLI_CMD_T **list = &sh_pstCliCmd;
	CLI_CMD_T *cmd = NULL;
	CLI_UTIL_QUEUE_T tokens;
	CLI_UTIL_QUEUE_T *cur;
	CLI_CMD_TOKEN_T *t;

	CLI_CMD_BuildList(&tokens, command);
	cur = tokens.q_next;

	while (cur != &tokens)
	{
		t = (CLI_CMD_TOKEN_T *)cur;
		if (!P_CLI_CMD_IsWhiteSpace(t))
		{
			cmd = CLI_CMD_FindWord(*list, &(t->token));
			if (!cmd)
			{
				cmd = malloc(sizeof(CLI_CMD_T) + strlen(&(t->token)) + 1);
				memset(cmd, 0, sizeof(CLI_CMD_T));
				cmd->cmdword = (char *)(cmd + 1);
				strcpy(cmd->cmdword, &(t->token));
				cmd->sibling = *list;
				*list = cmd;
			}
			list = &(cmd->child);
		}
		cur = cur->q_next;
	}

	CLI_CMD_FreeTokens(&tokens);

	if (!cmd)
    {
        unRet = APP_ERROR;
		return unRet;
    }
    else
    {
        PrintDebug("[%s] is added.", cmd->cmdword);
        unRet = APP_OK;
    }

	cmd->func = func;
	cmd->usage = usage;
	cmd->ref = ref;
	cmd->help = help;
	cmd->switches = switches;

	return unRet;
}

static void P_CLI_CMD_DumpIndent(char *str, int amt)
{
	int idx;
	char *dupstr;
	char *end;
	char *ptr;

	dupstr = strdup(str);

	ptr = dupstr;

	while (*ptr)
	{
		for (idx = 0; idx < amt; idx++)
        {
			printf(" ");
        }

		end = strchr(ptr, '\n');

		if (end)
        {
			*end++ = '\0';
        }
		else
        {
			end = ptr + strlen(ptr);
        }

		PrintInfo("%s", ptr);
		ptr = end;
	}

	free(dupstr);
}

static void P_CLI_CMD_DumpSwitch(char *str)
{
	char *switches;
	char *end;
	char *ptr;
	char *semi;
	char *newline;

	switches = strdup(str);

	ptr = switches;

	while (*ptr)
	{
		end = strchr(ptr, '|');
		if (end)
        {
			*end++ = '\0';
        }
		else
        {
			end = ptr + strlen(ptr);
        }

		printf("     ");
		if ((semi = strchr(ptr, ';')))
		{
			*semi++ = '\0';
			newline = strchr(semi, '\n');
			if (newline)
            {
				*newline++ = '\0';
            }

			PrintInfo("%-12s %s", ptr, semi);
			if (newline)
            {
				P_CLI_CMD_DumpIndent(newline, 5 + 12 + 1);
            }
		}
		else
		{
			PrintError("%-12s (no information)", ptr);
		}

		ptr = end;
	}

	free(switches);
}

static void P_CLI_CMD_Dump(CLI_CMD_T *cmd, int level, char **words, int verbose)
{
	int idx;
	int len;

	while (cmd)
	{
		len = 0;
		words[level] = cmd->cmdword;
		if (cmd->func)
		{
			for (idx = 0; idx < level; idx++)
			{
				printf("%s ", words[idx]);
				len += strlen(words[idx]) + 1;
			}

			printf("%s", cmd->cmdword);
			len += strlen(cmd->cmdword);
			for (idx = len; idx < 20; idx++)
            {
				printf(" ");
            }

			PrintInfo("%s", cmd->help);

			if (verbose)
			{
				PrintInfo();
				P_CLI_CMD_DumpIndent(cmd->usage, 5);
				PrintInfo();
				P_CLI_CMD_DumpSwitch(cmd->switches);
				PrintInfo();
			}
		}
		P_CLI_CMD_Dump(cmd->child, level + 1, words, verbose);
		cmd = cmd->sibling;
	}
}

static void P_CLI_CMD_DumpCmds(int verbose)
{
	char *words[20];

	P_CLI_CMD_Dump(sh_pstCliCmd, 0, words, verbose);
}

static void P_CLI_CMD_ShowPossibleCmd(CLI_CMDLINE_T *cline, CLI_CMD_T *cmd)
{
	int i;

	if (cline->argidx == 0)
	{
		printf("Available commands: ");
	}
	else
	{
		printf("Available \"");
		for (i = 0; i < cline->argidx; i++)
		{
			printf("%s%s", (i == 0) ? "" : " ", cline->argv[i]);
		}
		printf("\" commands: ");
	}

	while (cmd)
	{
		printf("%s", cmd->cmdword);
		if (cmd->sibling)
        {
			printf(", ");
        }
		cmd = cmd->sibling;
	}

	PrintInfo();
}

static int P_CLI_CMD_Help(CLI_CMDLINE_T *cmd, int argc, char *argv[])
{
    int nRet = APP_ERROR;

	CLI_CMD_T **tab;
	CLI_CMD_T *cword;
	int idx;

    if(cmd == NULL)
    {
        PrintError("cmd == NULL!!");
        return nRet;
    }

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

	if (argc == 0)
	{
		PrintInfo("Available commands:\n");
		P_CLI_CMD_DumpCmds(0);
		PrintInfo();
		PrintInfo("For more information about a command, enter 'help command-name'");

        nRet = APP_OK;
	}
	else
	{
		idx = 0;
		tab = &sh_pstCliCmd;
		cword = NULL;

		for (;;)
		{
			cword = CLI_CMD_FindWord(*tab, argv[idx]);
			if (!cword)
            {
				break;
            }

			if (cword->func != NULL)
            {
				break;
            }

			idx++;
			tab = &(cword->child);

			if (idx >= argc)
            {
				break;
            }
		}

		if (cword == NULL)
		{
			PrintInfo("No help available for '%s'.\n", argv[idx]);
			PrintInfo("Type 'help' for a list of commands.");
			return -1;
		}

		if (!cword->func && (idx >= argc))
		{
			PrintInfo("No help available for '%s'.\n", cword->cmdword);
			PrintInfo("Type 'help' for a list of commands.");
			return -1;
		}

		PrintInfo("\n  SUMMARY");
		P_CLI_CMD_DumpIndent(cword->help, 5);
		PrintInfo("\n  USAGE");
		P_CLI_CMD_DumpIndent(cword->usage, 5);

		if (cword->switches && cword->switches[0])
		{
			PrintInfo("\n  OPTIONS");
			P_CLI_CMD_DumpSwitch(cword->switches);
		}

		PrintInfo();

        nRet = APP_OK;
	}

	return nRet;
}

static int P_CLI_CMD_Test(CLI_CMDLINE_T *cmd, int argc, char *argv[])
{
    int nRet = APP_ERROR;

    if(cmd == NULL)
    {
        PrintError("cmd == NULL!!");
        return nRet;
    }

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

	if (argc == 0)
	{
		PrintInfo("Test commands cmd_test:\n");
		PrintInfo("Test Commands");
        nRet = APP_OK;
	}

	return nRet;
}

uint32_t CLI_CMD_Init(void)
{
    uint32_t unRet = APP_ERROR;

	sh_pstCliCmd = malloc(sizeof(CLI_CMD_T)); /* Todo add handle free */

	unRet = CLI_CMD_AddCmd("help",
			   P_CLI_CMD_Help,
			   NULL,
			   "help for CLI commands",
			   "help [enter command]\n\n"
			   "Without any parameters, the 'help' show a description\n"
			   "of available commands. For more details on a command, type and enter 'help'\n"
			   "and the command name.",
			   "");
    if(unRet != APP_OK)
    {
#if defined(CONFIG_CLI_DEBUG)
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
#endif
    }

	unRet = CLI_CMD_AddCmd("test",
			   P_CLI_CMD_Test,
			   NULL,
			   "Test commands",
			   "Test [command 1]\n\n"
			   "Test [command 2]\n"
			   "Test [command 3]\n"
			   "Test [command 4]",
			   "");
    if(unRet != APP_OK)
    {
#if defined(CONFIG_CLI_DEBUG)
        PrintError("CLI_CMD_AddCmd() is failed! [unRet:%d]", unRet);
#endif
    }
#if defined(CONFIG_CLI_DEBUG)
#else
    unRet = APP_OK;
#endif
    return unRet;
}

int CLI_CMD_CheckLookUp(CLI_UTIL_QUEUE_T *head, CLI_CMDLINE_T *cmd)
{
	CLI_CMD_T **tab;
	CLI_CMD_T *cword;
	int idx;

	memset(cmd, 0, sizeof(CLI_CMDLINE_T));

	CLI_CMD_BuildCmdline(head, cmd);
	if (cmd->argc == 0)
    {
		return APP_ERR_BLANK;
    }

	idx = 0;
	tab = &sh_pstCliCmd;
	cword = NULL;

	for (;;)
	{
		cword = CLI_CMD_FindWord(*tab, cmd->argv[idx]);
		if (!cword)
		{
			break;
		}

		if (cword->func != NULL)
		{
			break;
		}

		idx++;

		tab = &(cword->child);

		if (idx >= cmd->argc)
		{
			break;
		}
	}

	cmd->argidx = idx;

	if (cword == NULL)
	{
		PrintWarn("Invalid command: \"%s\"\n", cmd->argv[idx]);
		P_CLI_CMD_ShowPossibleCmd(cmd, *tab);
		PrintInfo();
		return APP_ERR_INVALID;
	}

	if (!cword->func && (idx >= cmd->argc))
	{
		PrintWarn("Incomplete command: \"%s\"\n", cmd->argv[idx - 1]);
		P_CLI_CMD_ShowPossibleCmd(cmd, *tab);
		PrintInfo();
		return APP_ERR_AMBIGUOUS;
	}

	cmd->argidx++;
	cmd->ref = cword->ref;
	cmd->usage = cword->usage;
	cmd->switches = cword->switches;
	cmd->func = cword->func;

	return 0;
}

static void P_CLI_CMD_RemoveLeadingWhite(CLI_UTIL_QUEUE_T *head)
{
	CLI_CMD_TOKEN_T *t;

	while (!CLI_UTIL_IsEmpTyQueue(head))
	{
		t = (CLI_CMD_TOKEN_T *)CLI_UTIL_GetFirstQueue(head);
		if (P_CLI_CMD_IsWhiteSpace(t))
		{
			CLI_UTIL_Dequeue(&(t->qb));
			free(t);
		}
		else
        {
			break;
        }
	}
}

CLI_LIST_T *CLI_CMD_Read(CLI_UTIL_QUEUE_T *head)
{
	char *ptr;
	int insquote = false;
	int indquote = false;
	CLI_LIST_T *cmd;
	int term = APP_CLI_CMD_EOL;
	CLI_CMD_TOKEN_T *t;

	P_CLI_CMD_RemoveLeadingWhite(head);

	if (CLI_UTIL_IsEmpTyQueue(head))
    {
		return NULL;
    }

	cmd = (CLI_LIST_T *)malloc(sizeof(CLI_LIST_T));
	CLI_UTIL_InitQueue(&(cmd->head));

	while ((t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(head)))
	{
		ptr = &(t->token);

		if (!insquote && !indquote)
		{
			if ((*ptr == ';') || (*ptr == '\n'))
			{
				term = APP_CLI_CMD_SEMI;
				break;
			}
			if ((*ptr == '&') && (*(ptr + 1) == '&'))
			{
				term = APP_CLI_CMD_AND;
				break;
			}
			if ((*ptr == '|') && (*(ptr + 1) == '|'))
			{
				term = APP_CLI_CMD_OR;
				break;
			}
		}

		if (*ptr == '\'')
		{
			insquote = !insquote;
		}

		if (!insquote)
		{
			if (*ptr == '"')
			{
				indquote = !indquote;
			}
		}

		CLI_UTIL_Enqueue(&(cmd->head), &(t->qb));
	}

	cmd->term = term;

	if (term != APP_CLI_CMD_EOL)
	{
		free(t);
	}

	return cmd;
}

static CLI_CMD_TOKEN_T *P_CLI_CMD_MakeToken(char *str, int len)
{
	CLI_CMD_TOKEN_T *t = (CLI_CMD_TOKEN_T *)malloc(sizeof(CLI_CMD_TOKEN_T) + len);

	memcpy(&(t->token), str, len);
	(&(t->token))[len] = 0;

	return t;
}

void CLI_CMD_BuildList(CLI_UTIL_QUEUE_T *qb, char *buf)
{
	char *cur = buf, *start = NULL, *fin = NULL;
	CLI_CMD_TOKEN_T *t;

	CLI_UTIL_InitQueue(qb);

	start = cur;
	while (*cur != '\0')
	{
		if (*cur == '&' && *(cur + 1) != '&')
		{
			/* N/A */
		}
		else if (*cur == '|' && *(cur + 1) != '|')
		{
			/* N/A */
		}
		else if (((*cur == ' ') || (*cur == '\t')) && ((*(cur - 1) == ' ') || (*(cur - 1) == '\t')))
		{
			/* N/A */
		}
		else
		{
			if (strchr(s_cTokenBreaks, *cur))
			{
				if (cur != buf)
				{
					fin = cur;
					t = P_CLI_CMD_MakeToken(start, fin - start);
					CLI_UTIL_Enqueue(qb, &(t->qb));
					start = cur; /* Start new token */
				}
			}
			else
			{
				if ((cur > buf) && strchr(s_cTokenBreaks, *(cur - 1)))
				{
					fin = cur;
					t = P_CLI_CMD_MakeToken(start, fin - start);
					CLI_UTIL_Enqueue(qb, &(t->qb));
					start = cur; /* Start new token */
				}
				else
				{
                    /* N/A */
				}
			}
		}
		cur++;
	}

	fin = cur;

	if (fin - start > 0)
	{
		t = P_CLI_CMD_MakeToken(start, fin - start);
		CLI_UTIL_Enqueue(qb, &(t->qb));
	}

	return;
}

static int P_CLI_CMD_IsCommandSeparator(CLI_CMD_TOKEN_T *t)
{
	char *string = &(t->token);
	int sep = 0;

	switch (*string)
	{
	case ';':
		sep = 1;
		break;
	case '&':
		if (*(string + 1) == '&')
			sep = 1;
		break;
	case '|':
		if (*(string + 1) == '|')
			sep = 1;
	default:
		break;
	}

	return (sep);
}

static char *P_CLI_CMD_RemoveQuotedArg(CLI_UTIL_QUEUE_T *head, CLI_CMD_TOKEN_T *t)
{
	int dquote = 0;
	int squote = 0;
	CLI_UTIL_QUEUE_T qlist;
	CLI_UTIL_QUEUE_T *q;
	char *dest;
	int maxlen = 0;

	if (!isQuote(t->token))
	{
		dest = P_CLI_CMD_Strdup(&(t->token));
		return dest;
	}

	CLI_UTIL_InitQueue(&qlist);

	if (t->token == '"')
    {
		dquote = 1;
    }
	else
    {
		squote = 1;
    }

	t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(head);

	while (t != NULL)
	{
		if (squote && (t->token == '\''))
		{
			free(t);
			break;
		}

		if (dquote && !squote && (t->token == '\"'))
		{
			free(t);
			break;
		}
		CLI_UTIL_Enqueue(&qlist, (CLI_UTIL_QUEUE_T *)t);
		t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(head);
	}

	for (q = qlist.q_next; q != &qlist; q = q->q_next)
	{
		maxlen += strlen(&(((CLI_CMD_TOKEN_T *)q)->token));
	}

	dest = malloc(maxlen + 1);
	if (!dest)
		return NULL;

	*dest = '\0';

	while ((t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(&qlist)))
	{
		strcat(dest, &(t->token));
		free(t);
	}

	return dest;
}

void CLI_CMD_SetList(CLI_UTIL_QUEUE_T *qb)
{
	CLI_UTIL_QUEUE_T *q;
	CLI_UTIL_QUEUE_T newq;
	CLI_CMD_TOKEN_T *t;
	int alias_check = true;
	int insquote = false;

	CLI_UTIL_InitQueue(&newq);

	while ((t = (CLI_CMD_TOKEN_T *)CLI_UTIL_DequeueNext(qb)))
	{
		if (t->token == '\'')
		{
			alias_check = false;
			insquote = !insquote;
		}

		if (t)
		{
			CLI_UTIL_Enqueue(&newq, &(t->qb));
			alias_check = P_CLI_CMD_IsCommandSeparator(t);
		}
	}

	while ((q = CLI_UTIL_DequeueNext(&newq)))
	{
		CLI_UTIL_Enqueue(qb, q);
	}

    if(alias_check != true)
    {
        /* N/A */
    }

}

void CLI_CMD_FreeTokens(CLI_UTIL_QUEUE_T *list)
{
	CLI_UTIL_QUEUE_T *q;

	while ((q = CLI_UTIL_DequeueNext(list)))
	{
		free(q);
	}
}

char *CLI_CMD_GetArg(CLI_CMDLINE_T *pstCmd, int nArgNum)
{
    nArgNum += pstCmd->argidx;

    if ((nArgNum < 0) || (nArgNum >= pstCmd->argc))
    {
        return NULL;
    }

    return pstCmd->argv[nArgNum];
}

int CLI_CMD_Showusage(CLI_CMDLINE_T *pstCmd)
{
    PrintInfo();
    P_CLI_CMD_DumpIndent(pstCmd->usage, 5);
    PrintInfo();
    if (pstCmd->switches[0])
    {
    	P_CLI_CMD_DumpSwitch(pstCmd->switches);
        PrintInfo();
	}

    return APP_OK;
}

