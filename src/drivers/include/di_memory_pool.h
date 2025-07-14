#ifndef _DI_MEMORY_POOL_H_
#define _DI_MEMORY_POOL_H_

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
* @file di_memory_pool.h
*
* @note
*
* DI Memory Pool Header for V2X Video Streaming
*
******************************************************************************/

/***************************** Include ***************************************/
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "type.h"

/***************************** Definition ************************************/

/* Memory pool constants */
#define DI_MEMORY_POOL_DEFAULT_SIZE           (8 * 1024 * 1024)  /* 8MB default pool size */
#define DI_MEMORY_POOL_MIN_SIZE               (1 * 1024 * 1024)  /* 1MB minimum pool size */
#define DI_MEMORY_POOL_MAX_SIZE               (64 * 1024 * 1024) /* 64MB maximum pool size */
#define DI_MEMORY_POOL_DEFAULT_BLOCK_SIZE     (64 * 1024)        /* 64KB default block size */
#define DI_MEMORY_POOL_MIN_BLOCK_SIZE         (4 * 1024)         /* 4KB minimum block size */
#define DI_MEMORY_POOL_MAX_BLOCK_SIZE         (4 * 1024 * 1024)  /* 4MB maximum block size */
#define DI_MEMORY_POOL_MAX_BLOCKS             (1024)             /* Maximum number of blocks */

/* Memory pool error codes */
#define DI_MEMORY_POOL_OK                     (0)
#define DI_MEMORY_POOL_ERROR                  (-1)
#define DI_MEMORY_POOL_MEMORY_ERROR           (-2)
#define DI_MEMORY_POOL_INVALID_PARAM_ERROR    (-3)
#define DI_MEMORY_POOL_NO_FREE_BLOCK_ERROR    (-4)
#define DI_MEMORY_POOL_INVALID_BLOCK_ERROR    (-5)
#define DI_MEMORY_POOL_ALREADY_FREED_ERROR    (-6)
#define DI_MEMORY_POOL_CORRUPTION_ERROR       (-7)

/* Memory alignment */
#define DI_MEMORY_POOL_ALIGNMENT              (64)               /* 64-byte alignment */

/***************************** Enum and Structure ****************************/

/**
* @details DI Memory Pool Status
*/
typedef enum {
    DI_MEMORY_POOL_STATUS_UNINITIALIZED      = 0,
    DI_MEMORY_POOL_STATUS_INITIALIZED        = 1,
    DI_MEMORY_POOL_STATUS_ACTIVE             = 2,
    DI_MEMORY_POOL_STATUS_DESTROYED          = 3,
    DI_MEMORY_POOL_STATUS_ERROR              = 4,
    DI_MEMORY_POOL_STATUS_MAX                = 255
} DI_MEMORY_POOL_STATUS_E;

/**
* @details DI Memory Pool Block Header
*/
typedef struct DI_MEMORY_POOL_BLOCK_HEADER_t {
    uint32_t unMagicNumber;                   /* Magic number for corruption detection */
    uint32_t unBlockIndex;                    /* Block index */
    uint32_t unBlockSize;                     /* Block size */
    uint32_t unTimestamp;                     /* Allocation timestamp */
    bool bAllocated;                          /* Allocation status */
    uint8_t achPadding[3];                    /* Padding for alignment */
} DI_MEMORY_POOL_BLOCK_HEADER_T;

/**
* @details DI Memory Pool Statistics
*/
typedef struct DI_MEMORY_POOL_STATS_t {
    uint32_t unTotalAllocations;              /* Total allocation count */
    uint32_t unTotalFrees;                    /* Total free count */
    uint32_t unCurrentAllocations;            /* Current allocation count */
    uint32_t unPeakAllocations;               /* Peak allocation count */
    uint32_t unTotalAllocatedBytes;           /* Total allocated bytes */
    uint32_t unCurrentAllocatedBytes;         /* Current allocated bytes */
    uint32_t unPeakAllocatedBytes;            /* Peak allocated bytes */
    uint32_t unFragmentationCount;            /* Fragmentation count */
    uint32_t unCorruptionCount;               /* Corruption detection count */
    uint32_t unOutOfMemoryCount;              /* Out of memory count */
} DI_MEMORY_POOL_STATS_T;

/**
* @details DI Memory Pool Configuration
*/
typedef struct DI_MEMORY_POOL_CONFIG_t {
    uint32_t unPoolSize;                      /* Total pool size */
    uint32_t unBlockSize;                     /* Block size */
    uint32_t unAlignment;                     /* Memory alignment */
    bool bZeroInitialize;                     /* Zero initialize allocated memory */
    bool bCorruptionDetection;                /* Enable corruption detection */
    bool bThreadSafe;                         /* Enable thread safety */
    bool bEnableStats;                        /* Enable statistics */
} DI_MEMORY_POOL_CONFIG_T;

/**
* @details DI Memory Pool Main Structure
*/
typedef struct DI_MEMORY_POOL_t {
    /* Pool memory management */
    uint8_t *puchMemoryPool;                  /* Memory pool start address */
    uint32_t unPoolSize;                      /* Total pool size */
    uint32_t unBlockSize;                     /* Block size */
    uint32_t unBlockCount;                    /* Total block count */
    uint32_t unAlignment;                     /* Memory alignment */
    
    /* Block management */
    DI_MEMORY_POOL_BLOCK_HEADER_T *pstBlockHeaders; /* Block headers array */
    uint32_t *punFreeBlocks;                  /* Free block indices */
    uint32_t unFreeBlockCount;                /* Free block count */
    uint32_t unNextFreeIndex;                 /* Next free block index */
    
    /* Thread synchronization */
    pthread_mutex_t hMutex;                   /* Access control mutex */
    bool bThreadSafe;                         /* Thread safety flag */
    
    /* Configuration and status */
    DI_MEMORY_POOL_CONFIG_T stConfig;         /* Pool configuration */
    DI_MEMORY_POOL_STATUS_E eStatus;          /* Pool status */
    DI_MEMORY_POOL_STATS_T stStats;           /* Pool statistics */
    
    /* Error handling */
    bool bErrorState;                         /* Error state flag */
    int32_t nLastError;                       /* Last error code */
    
    /* Magic number for corruption detection */
    uint32_t unMagicNumber;                   /* Pool magic number */
    
    /* Logging */
    bool bLogLevel;                           /* Log level */
} DI_MEMORY_POOL_T;

/***************************** Function Prototype ****************************/

/* Memory pool lifecycle management */
int32_t DI_MEMORY_POOL_Init(DI_MEMORY_POOL_T *pstMemoryPool, 
                            DI_MEMORY_POOL_CONFIG_T *pstConfig);
int32_t DI_MEMORY_POOL_DeInit(DI_MEMORY_POOL_T *pstMemoryPool);

/* Memory allocation and deallocation */
int32_t DI_MEMORY_POOL_Alloc(DI_MEMORY_POOL_T *pstMemoryPool, 
                             uint8_t **ppuchMemory);
int32_t DI_MEMORY_POOL_Free(DI_MEMORY_POOL_T *pstMemoryPool, 
                            uint8_t *puchMemory);
int32_t DI_MEMORY_POOL_Realloc(DI_MEMORY_POOL_T *pstMemoryPool, 
                               uint8_t **ppuchMemory, 
                               uint32_t unNewSize);

/* Memory pool status and information */
int32_t DI_MEMORY_POOL_GetFreeBlockCount(DI_MEMORY_POOL_T *pstMemoryPool, 
                                         uint32_t *punFreeBlockCount);
int32_t DI_MEMORY_POOL_GetAllocatedBlockCount(DI_MEMORY_POOL_T *pstMemoryPool, 
                                              uint32_t *punAllocatedBlockCount);
int32_t DI_MEMORY_POOL_GetTotalSize(DI_MEMORY_POOL_T *pstMemoryPool, 
                                    uint32_t *punTotalSize);
int32_t DI_MEMORY_POOL_GetUsedSize(DI_MEMORY_POOL_T *pstMemoryPool, 
                                   uint32_t *punUsedSize);
int32_t DI_MEMORY_POOL_GetFreeSize(DI_MEMORY_POOL_T *pstMemoryPool, 
                                   uint32_t *punFreeSize);

/* Memory pool validation and debugging */
int32_t DI_MEMORY_POOL_ValidateBlock(DI_MEMORY_POOL_T *pstMemoryPool, 
                                     uint8_t *puchMemory);
int32_t DI_MEMORY_POOL_ValidatePool(DI_MEMORY_POOL_T *pstMemoryPool);
int32_t DI_MEMORY_POOL_CheckCorruption(DI_MEMORY_POOL_T *pstMemoryPool);

/* Statistics and monitoring */
int32_t DI_MEMORY_POOL_GetStats(DI_MEMORY_POOL_T *pstMemoryPool, 
                                DI_MEMORY_POOL_STATS_T *pstStats);
int32_t DI_MEMORY_POOL_ResetStats(DI_MEMORY_POOL_T *pstMemoryPool);

/* Configuration and logging */
int32_t DI_MEMORY_POOL_SetConfig(DI_MEMORY_POOL_T *pstMemoryPool, 
                                 DI_MEMORY_POOL_CONFIG_T *pstConfig);
int32_t DI_MEMORY_POOL_GetConfig(DI_MEMORY_POOL_T *pstMemoryPool, 
                                 DI_MEMORY_POOL_CONFIG_T *pstConfig);
int32_t DI_MEMORY_POOL_SetLog(DI_MEMORY_POOL_T *pstMemoryPool, 
                              bool bLogLevel);

/* Utility functions */
void DI_MEMORY_POOL_PrintStatus(DI_MEMORY_POOL_T *pstMemoryPool);
int32_t DI_MEMORY_POOL_GetDefaultConfig(DI_MEMORY_POOL_CONFIG_T *pstConfig);
uint32_t DI_MEMORY_POOL_CalculateBlockCount(uint32_t unPoolSize, 
                                           uint32_t unBlockSize);

/* Helper macros */
#define DI_MEMORY_POOL_ALIGN_SIZE(size, alignment) \
    (((size) + (alignment) - 1) & ~((alignment) - 1))

#define DI_MEMORY_POOL_IS_ALIGNED(ptr, alignment) \
    (((uintptr_t)(ptr) & ((alignment) - 1)) == 0)

#define DI_MEMORY_POOL_GET_BLOCK_INDEX(pool, ptr) \
    (((uint8_t*)(ptr) - (pool)->puchMemoryPool) / (pool)->unBlockSize)

#define DI_MEMORY_POOL_GET_BLOCK_PTR(pool, index) \
    ((pool)->puchMemoryPool + ((index) * (pool)->unBlockSize))

#endif /* _DI_MEMORY_POOL_H_ */