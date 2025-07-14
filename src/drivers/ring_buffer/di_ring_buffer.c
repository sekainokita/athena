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
* @file di_ring_buffer.c
*
* This file contains ring buffer implementation for V2X video streaming
*
* @note
*
* DI Ring Buffer Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  keti  25.01.09 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>

#include "di.h"
#include "di_ring_buffer.h"

/***************************** Definition ************************************/

/***************************** Static Variable *******************************/
static bool s_bDiRingBufferLog = FALSE;

/***************************** Static Function *******************************/

/* Calculate available space in ring buffer */
static uint32_t P_DI_RING_BUFFER_GetAvailableSpace(DI_RING_BUFFER_T *pstRingBuffer)
{
    uint32_t unAvailableSpace = 0;
    
    if (pstRingBuffer->unDataSize >= pstRingBuffer->unBufferSize)
    {
        unAvailableSpace = 0;
    }
    else
    {
        unAvailableSpace = pstRingBuffer->unBufferSize - pstRingBuffer->unDataSize;
    }
    
    return unAvailableSpace;
}

/* Update statistics */
static void P_DI_RING_BUFFER_UpdateStats(DI_RING_BUFFER_T *pstRingBuffer, 
                                         bool bWrite, 
                                         uint32_t unBytes)
{
    if (pstRingBuffer->stConfig.bEnableStats == FALSE)
    {
        return;
    }
    
    if (bWrite == TRUE)
    {
        pstRingBuffer->stStats.ullTotalWriteCount++;
        pstRingBuffer->stStats.ullTotalWriteBytes += unBytes;
    }
    else
    {
        pstRingBuffer->stStats.ullTotalReadCount++;
        pstRingBuffer->stStats.ullTotalReadBytes += unBytes;
    }
    
    /* Update current used size */
    pstRingBuffer->stStats.unCurrentUsedSize = pstRingBuffer->unDataSize;
    
    /* Update max used size */
    if (pstRingBuffer->unDataSize > pstRingBuffer->stStats.unMaxUsedSize)
    {
        pstRingBuffer->stStats.unMaxUsedSize = pstRingBuffer->unDataSize;
    }
}

/* Get timeout timespec */
static int32_t P_DI_RING_BUFFER_GetTimeoutSpec(uint32_t unTimeoutMs, 
                                               struct timespec *pstTimespec)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    struct timeval stCurrentTime;
    
    if (pstTimespec == NULL)
    {
        return nRet;
    }
    
    if (gettimeofday(&stCurrentTime, NULL) == 0)
    {
        pstTimespec->tv_sec = stCurrentTime.tv_sec + (unTimeoutMs / 1000);
        pstTimespec->tv_nsec = (stCurrentTime.tv_usec * 1000) + 
                               ((unTimeoutMs % 1000) * 1000000);
        
        if (pstTimespec->tv_nsec >= 1000000000)
        {
            pstTimespec->tv_sec++;
            pstTimespec->tv_nsec -= 1000000000;
        }
        
        nRet = DI_RING_BUFFER_OK;
    }
    
    return nRet;
}

/***************************** Function Implementation ***********************/

int32_t DI_RING_BUFFER_GetDefaultConfig(DI_RING_BUFFER_CONFIG_T *pstConfig)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstConfig == NULL)
    {
        PrintError("pstConfig == NULL");
        return nRet;
    }
    
    pstConfig->unBufferSize = DI_RING_BUFFER_DEFAULT_SIZE;
    pstConfig->unChunkSize = DI_RING_BUFFER_CHUNK_SIZE;
    pstConfig->unWatermarkHigh = DI_RING_BUFFER_WATERMARK_HIGH;
    pstConfig->unWatermarkLow = DI_RING_BUFFER_WATERMARK_LOW;
    pstConfig->unTimeoutMs = 1000;
    pstConfig->bDropOnOverflow = TRUE;
    pstConfig->bBlockOnEmpty = TRUE;
    pstConfig->bEnableStats = TRUE;
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_Init(DI_RING_BUFFER_T *pstRingBuffer, 
                            DI_RING_BUFFER_CONFIG_T *pstConfig)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    uint8_t *puchBuffer = NULL;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    if (pstConfig == NULL)
    {
        PrintError("pstConfig == NULL");
        return nRet;
    }
    
    /* Validate configuration */
    if ((pstConfig->unBufferSize < DI_RING_BUFFER_MIN_SIZE) ||
        (pstConfig->unBufferSize > DI_RING_BUFFER_MAX_SIZE))
    {
        PrintError("Invalid buffer size [%u]", pstConfig->unBufferSize);
        nRet = DI_RING_BUFFER_INVALID_PARAM_ERROR;
        return nRet;
    }
    
    /* Initialize structure */
    (void*)memset(pstRingBuffer, 0x00, sizeof(DI_RING_BUFFER_T));
    
    /* Allocate buffer memory */
    puchBuffer = (uint8_t*)malloc(pstConfig->unBufferSize);
    if (puchBuffer == NULL)
    {
        PrintError("Failed to allocate buffer memory [%u bytes]", 
                   pstConfig->unBufferSize);
        nRet = DI_RING_BUFFER_MEMORY_ERROR;
        return nRet;
    }
    
    /* Initialize mutex and condition variables */
    if (pthread_mutex_init(&pstRingBuffer->hMutex, NULL) != 0)
    {
        PrintError("Failed to initialize mutex");
        free(puchBuffer);
        nRet = DI_RING_BUFFER_ERROR;
        return nRet;
    }
    
    if (pthread_cond_init(&pstRingBuffer->hCondNotEmpty, NULL) != 0)
    {
        PrintError("Failed to initialize not empty condition");
        pthread_mutex_destroy(&pstRingBuffer->hMutex);
        free(puchBuffer);
        nRet = DI_RING_BUFFER_ERROR;
        return nRet;
    }
    
    if (pthread_cond_init(&pstRingBuffer->hCondNotFull, NULL) != 0)
    {
        PrintError("Failed to initialize not full condition");
        pthread_cond_destroy(&pstRingBuffer->hCondNotEmpty);
        pthread_mutex_destroy(&pstRingBuffer->hMutex);
        free(puchBuffer);
        nRet = DI_RING_BUFFER_ERROR;
        return nRet;
    }
    
    /* Set up ring buffer structure */
    pstRingBuffer->puchBuffer = puchBuffer;
    pstRingBuffer->unBufferSize = pstConfig->unBufferSize;
    pstRingBuffer->unReadPos = 0;
    pstRingBuffer->unWritePos = 0;
    pstRingBuffer->unDataSize = 0;
    pstRingBuffer->bErrorState = FALSE;
    pstRingBuffer->nLastError = DI_RING_BUFFER_OK;
    pstRingBuffer->bLogLevel = s_bDiRingBufferLog;
    
    /* Copy configuration */
    (void*)memcpy(&pstRingBuffer->stConfig, pstConfig, 
                  sizeof(DI_RING_BUFFER_CONFIG_T));
    
    /* Initialize statistics */
    (void*)memset(&pstRingBuffer->stStats, 0x00, 
                  sizeof(DI_RING_BUFFER_STATS_T));
    
    /* Set initial status */
    pstRingBuffer->eStatus = DI_RING_BUFFER_STATUS_INITIALIZED;
    
    if (s_bDiRingBufferLog == TRUE)
    {
        PrintTrace("Ring buffer initialized successfully [size:%u]", 
                   pstConfig->unBufferSize);
    }
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_DeInit(DI_RING_BUFFER_T *pstRingBuffer)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    /* Stop buffer operations */
    if (pstRingBuffer->eStatus == DI_RING_BUFFER_STATUS_RUNNING)
    {
        DI_RING_BUFFER_Stop(pstRingBuffer);
    }
    
    /* Destroy synchronization objects */
    pthread_cond_destroy(&pstRingBuffer->hCondNotFull);
    pthread_cond_destroy(&pstRingBuffer->hCondNotEmpty);
    pthread_mutex_destroy(&pstRingBuffer->hMutex);
    
    /* Free buffer memory */
    if (pstRingBuffer->puchBuffer != NULL)
    {
        free(pstRingBuffer->puchBuffer);
        pstRingBuffer->puchBuffer = NULL;
    }
    
    /* Reset structure */
    (void*)memset(pstRingBuffer, 0x00, sizeof(DI_RING_BUFFER_T));
    pstRingBuffer->eStatus = DI_RING_BUFFER_STATUS_UNINITIALIZED;
    
    if (s_bDiRingBufferLog == TRUE)
    {
        PrintTrace("Ring buffer deinitialized successfully");
    }
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_Start(DI_RING_BUFFER_T *pstRingBuffer)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    if (pstRingBuffer->eStatus != DI_RING_BUFFER_STATUS_INITIALIZED &&
        pstRingBuffer->eStatus != DI_RING_BUFFER_STATUS_STOPPED)
    {
        PrintError("Invalid ring buffer status [%d]", pstRingBuffer->eStatus);
        return nRet;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    
    pstRingBuffer->eStatus = DI_RING_BUFFER_STATUS_RUNNING;
    pstRingBuffer->bErrorState = FALSE;
    pstRingBuffer->nLastError = DI_RING_BUFFER_OK;
    
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    if (s_bDiRingBufferLog == TRUE)
    {
        PrintTrace("Ring buffer started successfully");
    }
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_Stop(DI_RING_BUFFER_T *pstRingBuffer)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    
    pstRingBuffer->eStatus = DI_RING_BUFFER_STATUS_STOPPED;
    
    /* Wake up all waiting threads */
    pthread_cond_broadcast(&pstRingBuffer->hCondNotEmpty);
    pthread_cond_broadcast(&pstRingBuffer->hCondNotFull);
    
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    if (s_bDiRingBufferLog == TRUE)
    {
        PrintTrace("Ring buffer stopped successfully");
    }
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_Write(DI_RING_BUFFER_T *pstRingBuffer, 
                             const uint8_t *puchData, 
                             uint32_t unDataSize)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    uint32_t unAvailableSpace = 0;
    uint32_t unFirstCopySize = 0;
    uint32_t unSecondCopySize = 0;
    struct timespec stTimeout;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    if (puchData == NULL)
    {
        PrintError("puchData == NULL");
        return nRet;
    }
    
    if (unDataSize == 0)
    {
        PrintError("unDataSize == 0");
        return nRet;
    }
    
    if (unDataSize > pstRingBuffer->unBufferSize)
    {
        PrintError("Data size too large [%u > %u]", 
                   unDataSize, pstRingBuffer->unBufferSize);
        return nRet;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    
    /* Check if buffer is running */
    if (pstRingBuffer->eStatus != DI_RING_BUFFER_STATUS_RUNNING)
    {
        PrintError("Ring buffer is not running [status:%d]", 
                   pstRingBuffer->eStatus);
        pthread_mutex_unlock(&pstRingBuffer->hMutex);
        return nRet;
    }
    
    /* Wait for space if buffer is full */
    while (TRUE)
    {
        unAvailableSpace = P_DI_RING_BUFFER_GetAvailableSpace(pstRingBuffer);
        
        if (unAvailableSpace >= unDataSize)
        {
            break;
        }
        
        /* Handle overflow */
        if (pstRingBuffer->stConfig.bDropOnOverflow == TRUE)
        {
            /* Drop oldest data to make space */
            uint32_t unDropSize = unDataSize - unAvailableSpace;
            
            pstRingBuffer->unReadPos = (pstRingBuffer->unReadPos + unDropSize) % 
                                       pstRingBuffer->unBufferSize;
            pstRingBuffer->unDataSize -= unDropSize;
            pstRingBuffer->stStats.unOverflowCount++;
            pstRingBuffer->stStats.unDroppedFrames++;
            
            if (s_bDiRingBufferLog == TRUE)
            {
                PrintWarn("Ring buffer overflow, dropped %u bytes", unDropSize);
            }
            
            break;
        }
        else
        {
            /* Wait for space with timeout */
            if (P_DI_RING_BUFFER_GetTimeoutSpec(pstRingBuffer->stConfig.unTimeoutMs, 
                                               &stTimeout) == DI_RING_BUFFER_OK)
            {
                if (pthread_cond_timedwait(&pstRingBuffer->hCondNotFull, 
                                          &pstRingBuffer->hMutex, 
                                          &stTimeout) != 0)
                {
                    PrintError("Write timeout");
                    pthread_mutex_unlock(&pstRingBuffer->hMutex);
                    return DI_RING_BUFFER_TIMEOUT_ERROR;
                }
            }
            else
            {
                pthread_cond_wait(&pstRingBuffer->hCondNotFull, 
                                 &pstRingBuffer->hMutex);
            }
            
            /* Check if buffer was stopped */
            if (pstRingBuffer->eStatus != DI_RING_BUFFER_STATUS_RUNNING)
            {
                pthread_mutex_unlock(&pstRingBuffer->hMutex);
                return DI_RING_BUFFER_ERROR;
            }
        }
    }
    
    /* Calculate copy sizes */
    if (pstRingBuffer->unWritePos + unDataSize <= pstRingBuffer->unBufferSize)
    {
        /* Single copy */
        unFirstCopySize = unDataSize;
        unSecondCopySize = 0;
    }
    else
    {
        /* Split copy */
        unFirstCopySize = pstRingBuffer->unBufferSize - pstRingBuffer->unWritePos;
        unSecondCopySize = unDataSize - unFirstCopySize;
    }
    
    /* Copy data */
    (void*)memcpy(&pstRingBuffer->puchBuffer[pstRingBuffer->unWritePos], 
                  puchData, unFirstCopySize);
    
    if (unSecondCopySize > 0)
    {
        (void*)memcpy(&pstRingBuffer->puchBuffer[0], 
                      &puchData[unFirstCopySize], unSecondCopySize);
    }
    
    /* Update positions */
    pstRingBuffer->unWritePos = (pstRingBuffer->unWritePos + unDataSize) % 
                                pstRingBuffer->unBufferSize;
    pstRingBuffer->unDataSize += unDataSize;
    
    /* Update statistics */
    P_DI_RING_BUFFER_UpdateStats(pstRingBuffer, TRUE, unDataSize);
    
    /* Signal waiting readers */
    pthread_cond_signal(&pstRingBuffer->hCondNotEmpty);
    
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    if (s_bDiRingBufferLog == TRUE)
    {
        PrintTrace("Ring buffer write completed [size:%u]", unDataSize);
    }
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_Read(DI_RING_BUFFER_T *pstRingBuffer, 
                            uint8_t *puchData, 
                            uint32_t unBufferSize, 
                            uint32_t *punReadSize)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    uint32_t unReadSize = 0;
    uint32_t unFirstCopySize = 0;
    uint32_t unSecondCopySize = 0;
    struct timespec stTimeout;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    if (puchData == NULL)
    {
        PrintError("puchData == NULL");
        return nRet;
    }
    
    if (punReadSize == NULL)
    {
        PrintError("punReadSize == NULL");
        return nRet;
    }
    
    *punReadSize = 0;
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    
    /* Check if buffer is running */
    if (pstRingBuffer->eStatus != DI_RING_BUFFER_STATUS_RUNNING)
    {
        PrintError("Ring buffer is not running [status:%d]", 
                   pstRingBuffer->eStatus);
        pthread_mutex_unlock(&pstRingBuffer->hMutex);
        return nRet;
    }
    
    /* Wait for data if buffer is empty */
    while (pstRingBuffer->unDataSize == 0)
    {
        if (pstRingBuffer->stConfig.bBlockOnEmpty == FALSE)
        {
            pthread_mutex_unlock(&pstRingBuffer->hMutex);
            return DI_RING_BUFFER_EMPTY_ERROR;
        }
        
        /* Wait for data with timeout */
        if (P_DI_RING_BUFFER_GetTimeoutSpec(pstRingBuffer->stConfig.unTimeoutMs, 
                                           &stTimeout) == DI_RING_BUFFER_OK)
        {
            if (pthread_cond_timedwait(&pstRingBuffer->hCondNotEmpty, 
                                      &pstRingBuffer->hMutex, 
                                      &stTimeout) != 0)
            {
                PrintError("Read timeout");
                pthread_mutex_unlock(&pstRingBuffer->hMutex);
                return DI_RING_BUFFER_TIMEOUT_ERROR;
            }
        }
        else
        {
            pthread_cond_wait(&pstRingBuffer->hCondNotEmpty, 
                             &pstRingBuffer->hMutex);
        }
        
        /* Check if buffer was stopped */
        if (pstRingBuffer->eStatus != DI_RING_BUFFER_STATUS_RUNNING)
        {
            pthread_mutex_unlock(&pstRingBuffer->hMutex);
            return DI_RING_BUFFER_ERROR;
        }
    }
    
    /* Determine read size */
    unReadSize = (pstRingBuffer->unDataSize < unBufferSize) ? 
                 pstRingBuffer->unDataSize : unBufferSize;
    
    /* Calculate copy sizes */
    if (pstRingBuffer->unReadPos + unReadSize <= pstRingBuffer->unBufferSize)
    {
        /* Single copy */
        unFirstCopySize = unReadSize;
        unSecondCopySize = 0;
    }
    else
    {
        /* Split copy */
        unFirstCopySize = pstRingBuffer->unBufferSize - pstRingBuffer->unReadPos;
        unSecondCopySize = unReadSize - unFirstCopySize;
    }
    
    /* Copy data */
    (void*)memcpy(puchData, 
                  &pstRingBuffer->puchBuffer[pstRingBuffer->unReadPos], 
                  unFirstCopySize);
    
    if (unSecondCopySize > 0)
    {
        (void*)memcpy(&puchData[unFirstCopySize], 
                      &pstRingBuffer->puchBuffer[0], 
                      unSecondCopySize);
    }
    
    /* Update positions */
    pstRingBuffer->unReadPos = (pstRingBuffer->unReadPos + unReadSize) % 
                               pstRingBuffer->unBufferSize;
    pstRingBuffer->unDataSize -= unReadSize;
    
    /* Update statistics */
    P_DI_RING_BUFFER_UpdateStats(pstRingBuffer, FALSE, unReadSize);
    
    /* Signal waiting writers */
    pthread_cond_signal(&pstRingBuffer->hCondNotFull);
    
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    *punReadSize = unReadSize;
    
    if (s_bDiRingBufferLog == TRUE)
    {
        PrintTrace("Ring buffer read completed [size:%u]", unReadSize);
    }
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_Clear(DI_RING_BUFFER_T *pstRingBuffer)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    
    pstRingBuffer->unReadPos = 0;
    pstRingBuffer->unWritePos = 0;
    pstRingBuffer->unDataSize = 0;
    
    /* Signal waiting threads */
    pthread_cond_broadcast(&pstRingBuffer->hCondNotEmpty);
    pthread_cond_broadcast(&pstRingBuffer->hCondNotFull);
    
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    if (s_bDiRingBufferLog == TRUE)
    {
        PrintTrace("Ring buffer cleared");
    }
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_GetUsedSize(DI_RING_BUFFER_T *pstRingBuffer, 
                                   uint32_t *punUsedSize)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    if (punUsedSize == NULL)
    {
        PrintError("punUsedSize == NULL");
        return nRet;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    *punUsedSize = pstRingBuffer->unDataSize;
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_GetFreeSize(DI_RING_BUFFER_T *pstRingBuffer, 
                                   uint32_t *punFreeSize)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    if (punFreeSize == NULL)
    {
        PrintError("punFreeSize == NULL");
        return nRet;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    *punFreeSize = P_DI_RING_BUFFER_GetAvailableSpace(pstRingBuffer);
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_GetStats(DI_RING_BUFFER_T *pstRingBuffer, 
                                DI_RING_BUFFER_STATS_T *pstStats)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    if (pstStats == NULL)
    {
        PrintError("pstStats == NULL");
        return nRet;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    (void*)memcpy(pstStats, &pstRingBuffer->stStats, 
                  sizeof(DI_RING_BUFFER_STATS_T));
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

int32_t DI_RING_BUFFER_SetLog(DI_RING_BUFFER_T *pstRingBuffer, 
                              bool bLogLevel)
{
    int32_t nRet = DI_RING_BUFFER_ERROR;
    
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return nRet;
    }
    
    s_bDiRingBufferLog = bLogLevel;
    pstRingBuffer->bLogLevel = bLogLevel;
    
    PrintTrace("Ring buffer log level set to [%s]", 
               bLogLevel == TRUE ? "ON" : "OFF");
    
    nRet = DI_RING_BUFFER_OK;
    
    return nRet;
}

void DI_RING_BUFFER_PrintStatus(DI_RING_BUFFER_T *pstRingBuffer)
{
    if (pstRingBuffer == NULL)
    {
        PrintError("pstRingBuffer == NULL");
        return;
    }
    
    pthread_mutex_lock(&pstRingBuffer->hMutex);
    
    PrintDebug("========== Ring Buffer Status ==========");
    PrintDebug("Buffer Size    : %u bytes", pstRingBuffer->unBufferSize);
    PrintDebug("Used Size      : %u bytes", pstRingBuffer->unDataSize);
    PrintDebug("Free Size      : %u bytes", 
               pstRingBuffer->unBufferSize - pstRingBuffer->unDataSize);
    PrintDebug("Read Position  : %u", pstRingBuffer->unReadPos);
    PrintDebug("Write Position : %u", pstRingBuffer->unWritePos);
    PrintDebug("Status         : %d", pstRingBuffer->eStatus);
    PrintDebug("Error State    : %s", 
               pstRingBuffer->bErrorState == TRUE ? "YES" : "NO");
    
    if (pstRingBuffer->stConfig.bEnableStats == TRUE)
    {
        PrintDebug("========== Ring Buffer Statistics ==========");
        PrintDebug("Total Writes   : %llu", pstRingBuffer->stStats.ullTotalWriteCount);
        PrintDebug("Total Reads    : %llu", pstRingBuffer->stStats.ullTotalReadCount);
        PrintDebug("Write Bytes    : %llu", pstRingBuffer->stStats.ullTotalWriteBytes);
        PrintDebug("Read Bytes     : %llu", pstRingBuffer->stStats.ullTotalReadBytes);
        PrintDebug("Overflow Count : %u", pstRingBuffer->stStats.unOverflowCount);
        PrintDebug("Underflow Count: %u", pstRingBuffer->stStats.unUnderflowCount);
        PrintDebug("Max Used Size  : %u bytes", pstRingBuffer->stStats.unMaxUsedSize);
        PrintDebug("Dropped Frames : %u", pstRingBuffer->stStats.unDroppedFrames);
    }
    
    PrintDebug("========================================");
    
    pthread_mutex_unlock(&pstRingBuffer->hMutex);
}