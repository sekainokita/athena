#ifndef _TYPE_H_
#define _TYPE_H_

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
#include <assert.h>

/* POSIX C library */
#include <mqueue.h>
#include <pthread.h>
#include <unistd.h>

/***************************** Definition ************************************/
#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"
#define COLOR_RESET   "\x1b[0m"

#define FRAMEWORK_MSG_ERR   -1
#define FRAMEWORK_OK        0
#define FRAMEWORK_ERROR     1

#define PLATFORM_OK         0
#define PLATFORM_ERROR      1

#define DI_MSG_ERR          -1
#define DI_OK               0
#define DI_ERROR            1

#define APP_MSG_ERR         -1
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

#define TRUE                true
#define FALSE               false
#define ON                  true
#define OFF                 false

#define MODEL_PREFIX            "model="
#define DEVICE_NAME_PREFIX      "pchDeviceName="
#define DEVICE_ID_PREFIX        "unDeviceId="
#define IFACE_NAME_PREFIX       "pchIfaceName="
#define IP_ADDR_PREFIX          "pchIpAddr="
#define PORT_PREFIX             "unPort="

#define MODEL_PREFIX_LEN        (sizeof(MODEL_PREFIX) - 1)
#define DEVICE_NAME_PREFIX_LEN  (sizeof(DEVICE_NAME_PREFIX) - 1)
#define DEVICE_ID_PREFIX_LEN    (sizeof(DEVICE_ID_PREFIX) - 1)
#define IFACE_NAME_PREFIX_LEN   (sizeof(IFACE_NAME_PREFIX) - 1)
#define IP_ADDR_PREFIX_LEN      (sizeof(IP_ADDR_PREFIX) - 1)
#define PORT_PREFIX_LEN         (sizeof(PORT_PREFIX) - 1)

#define MODEL_NAME_FILE_SUFFIX  ".conf"
#define MAX_MODEL_NAME_LEN      256

#define PrintError(fmt, args...) \
    do { \
        printf(COLOR_RED "[%s][%d] """ fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintNotice(fmt, args...) \
    do { \
        printf(COLOR_MAGENTA "[%s][%d] """ fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintWarn(fmt, args...) \
    do { \
        printf(COLOR_YELLOW "[%s][%d] """ fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintTrace(fmt, args...) \
    do { \
        printf(COLOR_BLUE "[%s][%d] """ fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintEnter(fmt, args...) \
    do { \
        printf(COLOR_GREEN "[%s][%d] """ fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintExit(fmt, args...) \
    do { \
        printf(COLOR_CYAN "[%s][%d] """ fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintDebug(fmt, args...) \
    do { \
        printf(COLOR_RESET "[%s][%d] """ fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintInfo(fmt, args...) \
    do { \
        printf(COLOR_RESET ""fmt"\n" COLOR_RESET , ##args); \
    } while (0)

/***************************** Enum and Structure ****************************/
typedef enum {
    LOG_ALL                         = 0,
    LOG_FRAMEWORK_ALL               = 10,
    LOG_FRAMEWORK_MGR_ALL           = 11,
    LOG_FRAMEWORK_MGR_MSG           = 12,
    LOG_FRAMEWORK_MGR_MULTI_MSG     = 13,
    LOG_FRAMEWORK_MGR_DB            = 14,
    LOG_FRAMEWORK_MGR_MULTI_DB      = 15,
    LOG_FRAMEWORK_MGR_TIME          = 16,
    LOG_DI_ALL                      = 50,
    LOG_DI_GPS                      = 51,
    LOG_APP_ALL                     = 100,
    LOG_APP_SVC_ALL                 = 101,
    LOG_APP_SVC_CP                  = 102,
    LOG_APP_SVC_MULTI_CP            = 103,
    LOG_PLATFORM_ALL                = 150,
    LOG_MAX                         = 0xFFFF
} LOG_TYPE_E;

typedef enum {
    LOG_LEVEL_ALL                   = 0,
    LOG_LEVEL_DEBUG                 = 1,
    LOG_LEVEL_WARN                  = 2,
    LOG_LEVEL_ERROR                 = 3,
    LOG_LEVEL_FATAL                 = 4,
    LOG_LEVEL_NONE                  = 5, /* NONE : Log Off */
    LOG_LEVEL_MAX                   = 0xFFFF
} LOG_LEVEL_E;

/***************************** Function Protype ******************************/
#endif	/* _TYPE_H_ */
