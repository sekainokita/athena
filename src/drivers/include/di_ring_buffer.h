#ifndef _DI_RING_BUFFER_H_
#define _DI_RING_BUFFER_H_

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
* @file di_ring_buffer.h
*
* @note
*
* DI Ring Buffer Header for V2X Video Streaming
*
******************************************************************************/

/***************************** Include ***************************************/
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "type.h"

/***************************** Definition ************************************/
#define DI_RING_BUFFER_DEFAULT_SIZE           (512 * 1024)       /* 512KB default size */
#define DI_RING_BUFFER_MAX_SIZE               (16 * 1024 * 1024) /* 16MB maximum size */
#define DI_RING_BUFFER_MIN_SIZE               (256 * 1024)       /* 256KB minimum size */
#define DI_RING_BUFFER_CHUNK_SIZE             (64 * 1024)        /* 64KB chunk size */
#define DI_RING_BUFFER_WATERMARK_HIGH         (70)               /* High watermark 70% */
#define DI_RING_BUFFER_WATERMARK_LOW          (10)               /* Low watermark 10% */

/* Ring buffer error codes */
#define DI_RING_BUFFER_OK                     (0)
#define DI_RING_BUFFER_ERROR                  (-1)
#define DI_RING_BUFFER_MEMORY_ERROR           (-2)
#define DI_RING_BUFFER_FULL_ERROR             (-3)
#define DI_RING_BUFFER_EMPTY_ERROR            (-4)
#define DI_RING_BUFFER_INVALID_PARAM_ERROR    (-5)
#define DI_RING_BUFFER_TIMEOUT_ERROR          (-6)
#define DI_RING_BUFFER_OVERFLOW_ERROR         (-7)
#define DI_RING_BUFFER_UNDERFLOW_ERROR        (-8)

/***************************** Enum and Structure ****************************/

/**
* @details DI Ring Buffer Status
*/
typedef enum {
    DI_RING_BUFFER_STATUS_UNINITIALIZED      = 0,
    DI_RING_BUFFER_STATUS_INITIALIZED        = 1,
    DI_RING_BUFFER_STATUS_RUNNING            = 2,
    DI_RING_BUFFER_STATUS_STOPPED            = 3,
    DI_RING_BUFFER_STATUS_ERROR              = 4,
    DI_RING_BUFFER_STATUS_MAX                = 255
} DI_RING_BUFFER_STATUS_E;

/**
* @details DI Ring Buffer Statistics
*/
typedef struct DI_RING_BUFFER_STATS_t {
    uint64_t ullTotalWriteCount;              /* Total write operations */
    uint64_t ullTotalReadCount;               /* Total read operations */
    uint64_t ullTotalWriteBytes;              /* Total bytes written */
    uint64_t ullTotalReadBytes;               /* Total bytes read */
    uint32_t unOverflowCount;                 /* Buffer overflow count */
    uint32_t unUnderflowCount;                /* Buffer underflow count */
    uint32_t unMaxUsedSize;                   /* Maximum used buffer size */
    uint32_t unCurrentUsedSize;               /* Current used buffer size */
    uint32_t unDroppedFrames;                 /* Dropped frame count */
} DI_RING_BUFFER_STATS_T;

/**
* @details DI Ring Buffer Configuration
*/
typedef struct DI_RING_BUFFER_CONFIG_t {
    uint32_t unBufferSize;                    /* Buffer total size */
    uint32_t unChunkSize;                     /* Chunk size for operations */
    uint32_t unWatermarkHigh;                 /* High watermark percentage */
    uint32_t unWatermarkLow;                  /* Low watermark percentage */
    uint32_t unTimeoutMs;                     /* Timeout in milliseconds */
    bool bDropOnOverflow;                     /* Drop oldest data on overflow */
    bool bBlockOnEmpty;                       /* Block when buffer is empty */
    bool bEnableStats;                        /* Enable statistics collection */
} DI_RING_BUFFER_CONFIG_T;

/**
* @details DI Ring Buffer Main Structure
*/
typedef struct DI_RING_BUFFER_t {
    /* Buffer memory management */
    uint8_t *puchBuffer;                      /* Buffer memory pointer */
    uint32_t unBufferSize;                    /* Total buffer size */
    uint32_t unReadPos;                       /* Read position */
    uint32_t unWritePos;                      /* Write position */
    uint32_t unDataSize;                      /* Current data size */
    
    /* Thread synchronization */
    pthread_mutex_t hMutex;                   /* Access control mutex */
    pthread_cond_t hCondNotEmpty;             /* Not empty condition */
    pthread_cond_t hCondNotFull;              /* Not full condition */
    
    /* Configuration and status */
    DI_RING_BUFFER_CONFIG_T stConfig;         /* Buffer configuration */
    DI_RING_BUFFER_STATUS_E eStatus;          /* Buffer status */
    DI_RING_BUFFER_STATS_T stStats;           /* Buffer statistics */
    
    /* Error handling */
    bool bErrorState;                         /* Error state flag */
    int32_t nLastError;                       /* Last error code */
    
    /* Logging */
    bool bLogLevel;                           /* Log level */
} DI_RING_BUFFER_T;

/***************************** Function Prototype ****************************/

/* Buffer lifecycle management */
int32_t DI_RING_BUFFER_Init(DI_RING_BUFFER_T *pstRingBuffer, 
                            DI_RING_BUFFER_CONFIG_T *pstConfig);
int32_t DI_RING_BUFFER_DeInit(DI_RING_BUFFER_T *pstRingBuffer);
int32_t DI_RING_BUFFER_Start(DI_RING_BUFFER_T *pstRingBuffer);
int32_t DI_RING_BUFFER_Stop(DI_RING_BUFFER_T *pstRingBuffer);

/* Buffer operations */
int32_t DI_RING_BUFFER_Write(DI_RING_BUFFER_T *pstRingBuffer, 
                             const uint8_t *puchData, 
                             uint32_t unDataSize);
int32_t DI_RING_BUFFER_Read(DI_RING_BUFFER_T *pstRingBuffer, 
                            uint8_t *puchData, 
                            uint32_t unBufferSize, 
                            uint32_t *punReadSize);
int32_t DI_RING_BUFFER_Peek(DI_RING_BUFFER_T *pstRingBuffer, 
                            uint8_t *puchData, 
                            uint32_t unBufferSize, 
                            uint32_t *punPeekSize);

/* Buffer status and control */
int32_t DI_RING_BUFFER_GetUsedSize(DI_RING_BUFFER_T *pstRingBuffer, 
                                   uint32_t *punUsedSize);
int32_t DI_RING_BUFFER_GetFreeSize(DI_RING_BUFFER_T *pstRingBuffer, 
                                   uint32_t *punFreeSize);
int32_t DI_RING_BUFFER_IsEmpty(DI_RING_BUFFER_T *pstRingBuffer, 
                               bool *pbIsEmpty);
int32_t DI_RING_BUFFER_IsFull(DI_RING_BUFFER_T *pstRingBuffer, 
                              bool *pbIsFull);
int32_t DI_RING_BUFFER_Clear(DI_RING_BUFFER_T *pstRingBuffer);

/* Statistics and monitoring */
int32_t DI_RING_BUFFER_GetStats(DI_RING_BUFFER_T *pstRingBuffer, 
                                DI_RING_BUFFER_STATS_T *pstStats);
int32_t DI_RING_BUFFER_ResetStats(DI_RING_BUFFER_T *pstRingBuffer);

/* Configuration and logging */
int32_t DI_RING_BUFFER_SetConfig(DI_RING_BUFFER_T *pstRingBuffer, 
                                 DI_RING_BUFFER_CONFIG_T *pstConfig);
int32_t DI_RING_BUFFER_GetConfig(DI_RING_BUFFER_T *pstRingBuffer, 
                                 DI_RING_BUFFER_CONFIG_T *pstConfig);
int32_t DI_RING_BUFFER_SetLog(DI_RING_BUFFER_T *pstRingBuffer, 
                              bool bLogLevel);

/* Utility functions */
void DI_RING_BUFFER_PrintStatus(DI_RING_BUFFER_T *pstRingBuffer);
int32_t DI_RING_BUFFER_GetDefaultConfig(DI_RING_BUFFER_CONFIG_T *pstConfig);

#endif /* _DI_RING_BUFFER_H_ */