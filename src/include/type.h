#ifndef	_TYPE_H_
#define	_TYPE_H_

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
* @file type.h
*
* @note
*
* Type Header
*
******************************************************************************/

/***************************** Include ***************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

/***************************** Definition ************************************/
#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"
#define COLOR_RESET   "\x1b[0m"

#define FRAMEWORK_OK        0
#define FRAMEWORK_ERROR     1

#define APP_OK              0
#define APP_ERROR           1
#define APP_ERR_INVALID     2
#define APP_ERR_AMBIGUOUS   3
#define APP_ERR_BLANK       4

#define APP_CLI_CMD_EOL     0
#define APP_CLI_CMD_SEMI    1
#define APP_CLI_CMD_AND     2
#define APP_CLI_CMD_OR      3

#define UNUSED(x)           (void)(x)

#define PrintError(fmt, args...) \
    do { \
        printf(COLOR_RED "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintNotice(fmt, args...) \
    do { \
        printf(COLOR_MAGENTA "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintWarn(fmt, args...) \
    do { \
        printf(COLOR_YELLOW "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintTrace(fmt, args...) \
    do { \
        printf(COLOR_BLUE "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintEnter(fmt, args...) \
    do { \
        printf(COLOR_GREEN "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintExit(fmt, args...) \
    do { \
        printf(COLOR_CYAN "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintDebug(fmt, args...) \
    do { \
        printf(COLOR_RESET "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintInfo(fmt, args...) \
    do { \
        printf(COLOR_RESET ""fmt"\n" COLOR_RESET , ##args); \
    } while (0)

/***************************** Enum and Structure ****************************/


/***************************** Function Protype ******************************/
#endif	/* _TYPE_H_ */
