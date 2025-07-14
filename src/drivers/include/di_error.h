#ifndef _DI_ERROR_H_
#define _DI_ERROR_H_

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
* @file di_error.h
*
* @note
*
* DI Error Handling Header for V2X Video Streaming
*
******************************************************************************/

/***************************** Include ***************************************/
#include <stdint.h>
#include <stdbool.h>

/***************************** Definition ************************************/

/* Common error codes */
#define DI_ERROR_OK                           (0)
#define DI_ERROR_GENERAL                      (-1)
#define DI_ERROR_INVALID_PARAM                (-2)
#define DI_ERROR_NULL_POINTER                 (-3)
#define DI_ERROR_MEMORY_ALLOC                 (-4)
#define DI_ERROR_TIMEOUT                      (-5)
#define DI_ERROR_NOT_INITIALIZED              (-6)
#define DI_ERROR_ALREADY_INITIALIZED          (-7)
#define DI_ERROR_NOT_SUPPORTED                (-8)
#define DI_ERROR_RESOURCE_BUSY                (-9)
#define DI_ERROR_RESOURCE_NOT_AVAILABLE       (-10)

/* Video streaming specific error codes */
#define DI_ERROR_STREAMING_BASE               (-100)
#define DI_ERROR_STREAMING_PIPELINE_CREATE    (DI_ERROR_STREAMING_BASE - 1)
#define DI_ERROR_STREAMING_PIPELINE_START     (DI_ERROR_STREAMING_BASE - 2)
#define DI_ERROR_STREAMING_PIPELINE_STOP      (DI_ERROR_STREAMING_BASE - 3)
#define DI_ERROR_STREAMING_ELEMENT_CREATE     (DI_ERROR_STREAMING_BASE - 4)
#define DI_ERROR_STREAMING_ELEMENT_LINK       (DI_ERROR_STREAMING_BASE - 5)
#define DI_ERROR_STREAMING_BUFFER_FULL        (DI_ERROR_STREAMING_BASE - 6)
#define DI_ERROR_STREAMING_BUFFER_EMPTY       (DI_ERROR_STREAMING_BASE - 7)
#define DI_ERROR_STREAMING_CODEC_ERROR        (DI_ERROR_STREAMING_BASE - 8)
#define DI_ERROR_STREAMING_NETWORK_ERROR      (DI_ERROR_STREAMING_BASE - 9)
#define DI_ERROR_STREAMING_FORMAT_ERROR       (DI_ERROR_STREAMING_BASE - 10)
#define DI_ERROR_STREAMING_ELEMENT_NOT_FOUND  (DI_ERROR_STREAMING_BASE - 11)
#define DI_ERROR_STREAMING_INIT_FAILED        (DI_ERROR_STREAMING_BASE - 12)
#define DI_ERROR_STREAMING_THREAD_CREATE      (DI_ERROR_STREAMING_BASE - 13)

/* Ring buffer specific error codes */
#define DI_ERROR_RING_BUFFER_BASE             (-200)
#define DI_ERROR_RING_BUFFER_OVERFLOW         (DI_ERROR_RING_BUFFER_BASE - 1)
#define DI_ERROR_RING_BUFFER_UNDERFLOW        (DI_ERROR_RING_BUFFER_BASE - 2)
#define DI_ERROR_RING_BUFFER_FULL             (DI_ERROR_RING_BUFFER_BASE - 3)
#define DI_ERROR_RING_BUFFER_EMPTY            (DI_ERROR_RING_BUFFER_BASE - 4)
#define DI_ERROR_RING_BUFFER_SIZE_INVALID     (DI_ERROR_RING_BUFFER_BASE - 5)

/* Network specific error codes */
#define DI_ERROR_NETWORK_BASE                 (-300)
#define DI_ERROR_NETWORK_CONNECTION_FAILED    (DI_ERROR_NETWORK_BASE - 1)
#define DI_ERROR_NETWORK_SEND_FAILED          (DI_ERROR_NETWORK_BASE - 2)
#define DI_ERROR_NETWORK_RECEIVE_FAILED       (DI_ERROR_NETWORK_BASE - 3)
#define DI_ERROR_NETWORK_TIMEOUT              (DI_ERROR_NETWORK_BASE - 4)
#define DI_ERROR_NETWORK_PACKET_LOSS          (DI_ERROR_NETWORK_BASE - 5)
#define DI_ERROR_NETWORK_CONGESTION           (DI_ERROR_NETWORK_BASE - 6)

/* Hardware specific error codes */
#define DI_ERROR_HARDWARE_BASE                (-400)
#define DI_ERROR_HARDWARE_CAMERA_NOT_FOUND    (DI_ERROR_HARDWARE_BASE - 1)
#define DI_ERROR_HARDWARE_CAMERA_INIT_FAILED  (DI_ERROR_HARDWARE_BASE - 2)
#define DI_ERROR_HARDWARE_DISPLAY_NOT_FOUND   (DI_ERROR_HARDWARE_BASE - 3)
#define DI_ERROR_HARDWARE_DISPLAY_INIT_FAILED (DI_ERROR_HARDWARE_BASE - 4)
#define DI_ERROR_HARDWARE_ENCODER_NOT_FOUND   (DI_ERROR_HARDWARE_BASE - 5)
#define DI_ERROR_HARDWARE_DECODER_NOT_FOUND   (DI_ERROR_HARDWARE_BASE - 6)
#define DI_ERROR_HARDWARE_GPU_ERROR           (DI_ERROR_HARDWARE_BASE - 7)

/***************************** Enum and Structure ****************************/

/**
* @details DI Error Severity Level
*/
typedef enum {
    DI_ERROR_SEVERITY_INFO                   = 0,
    DI_ERROR_SEVERITY_WARNING                = 1,
    DI_ERROR_SEVERITY_ERROR                  = 2,
    DI_ERROR_SEVERITY_CRITICAL               = 3,
    DI_ERROR_SEVERITY_FATAL                  = 4,
    DI_ERROR_SEVERITY_MAX                    = 255
} DI_ERROR_SEVERITY_E;

/**
* @details DI Error Context Information
*/
typedef struct DI_ERROR_CONTEXT_t {
    int32_t nErrorCode;                       /* Error code */
    DI_ERROR_SEVERITY_E eSeverity;            /* Error severity */
    const char *pchFileName;                  /* Source file name */
    const char *pchFunctionName;              /* Function name */
    uint32_t unLineNumber;                    /* Line number */
    uint32_t unTimestamp;                     /* Error timestamp */
    char achErrorMessage[256];                /* Error message */
    char achAdditionalInfo[512];              /* Additional information */
} DI_ERROR_CONTEXT_T;

/**
* @details DI Error Statistics
*/
typedef struct DI_ERROR_STATS_t {
    uint32_t unTotalErrorCount;               /* Total error count */
    uint32_t unInfoCount;                     /* Info message count */
    uint32_t unWarningCount;                  /* Warning count */
    uint32_t unErrorCount;                    /* Error count */
    uint32_t unCriticalCount;                 /* Critical error count */
    uint32_t unFatalCount;                    /* Fatal error count */
    uint32_t unLastErrorCode;                 /* Last error code */
    uint32_t unLastErrorTime;                 /* Last error timestamp */
} DI_ERROR_STATS_T;

/***************************** Function Prototype ****************************/

/* Error handling functions */
int32_t DI_ERROR_Init(void);
int32_t DI_ERROR_DeInit(void);
int32_t DI_ERROR_Report(int32_t nErrorCode, 
                        DI_ERROR_SEVERITY_E eSeverity,
                        const char *pchFileName,
                        const char *pchFunctionName,
                        uint32_t unLineNumber,
                        const char *pchFormat, ...);

/* Error information functions */
const char *DI_ERROR_GetErrorString(int32_t nErrorCode);
const char *DI_ERROR_GetSeverityString(DI_ERROR_SEVERITY_E eSeverity);
int32_t DI_ERROR_GetLastError(void);
int32_t DI_ERROR_GetErrorStats(DI_ERROR_STATS_T *pstStats);
int32_t DI_ERROR_ResetStats(void);

/* Error context functions */
int32_t DI_ERROR_GetContext(DI_ERROR_CONTEXT_T *pstContext);
int32_t DI_ERROR_SetContext(const DI_ERROR_CONTEXT_T *pstContext);

/* Utility macros */
#define DI_ERROR_REPORT_INFO(format, ...) \
    DI_ERROR_Report(DI_ERROR_OK, DI_ERROR_SEVERITY_INFO, \
                    __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)

#define DI_ERROR_REPORT_WARNING(code, format, ...) \
    DI_ERROR_Report(code, DI_ERROR_SEVERITY_WARNING, \
                    __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)

#define DI_ERROR_REPORT_ERROR(code, format, ...) \
    DI_ERROR_Report(code, DI_ERROR_SEVERITY_ERROR, \
                    __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)

#define DI_ERROR_REPORT_CRITICAL(code, format, ...) \
    DI_ERROR_Report(code, DI_ERROR_SEVERITY_CRITICAL, \
                    __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)

#define DI_ERROR_REPORT_FATAL(code, format, ...) \
    DI_ERROR_Report(code, DI_ERROR_SEVERITY_FATAL, \
                    __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)

/* Error checking macros */
#define DI_ERROR_CHECK_PARAM(param) \
    do { \
        if ((param) == NULL) { \
            DI_ERROR_REPORT_ERROR(DI_ERROR_NULL_POINTER, \
                                  "Parameter " #param " is NULL"); \
            return DI_ERROR_NULL_POINTER; \
        } \
    } while(0)

#define DI_ERROR_CHECK_RETURN(expr) \
    do { \
        int32_t nRet = (expr); \
        if (nRet != DI_ERROR_OK) { \
            DI_ERROR_REPORT_ERROR(nRet, \
                                  "Expression " #expr " failed with error %d", nRet); \
            return nRet; \
        } \
    } while(0)

#define DI_ERROR_CHECK_GOTO(expr, label) \
    do { \
        nRet = (expr); \
        if (nRet != DI_ERROR_OK) { \
            DI_ERROR_REPORT_ERROR(nRet, \
                                  "Expression " #expr " failed with error %d", nRet); \
            goto label; \
        } \
    } while(0)

#endif /* _DI_ERROR_H_ */