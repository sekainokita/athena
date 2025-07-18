/******************************************************************************
*
* Copyright (C) 2023 - 2028 KETI, All rights reserved.
*                           (Korea Electronics Technology Institute)
*
******************************************************************************/
/******************************************************************************
*
* @file svc_streaming.c
*
* This file contains V2X video streaming service implementation
*
* @note
*
* V2X Video Streaming Service Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  blank  25.07.12 First release
*
******************************************************************************/

/***************************** Include ****************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/msg.h>
#include <sys/ipc.h>

#include "framework.h"
#include "svc_streaming.h"

#if defined(CONFIG_VIDEO_STREAMING)
#include "di_camera.h"
#include "di_video.h"
#include "di_video_nvidia.h"
#include "di_ring_buffer.h"
#include "di_error.h"
#include "di_memory_pool.h"
#include "msg_manager.h"
#endif

/***************************** Definition ************************************/
#define SVC_STREAMING_TASK_MSG_KEY                  (0x240600)
#define SVC_STREAMING_DEFAULT_BITRATE               (2000000)
#define SVC_STREAMING_DEFAULT_GOP_SIZE              (30)
#define SVC_STREAMING_FRAME_TIMEOUT_MS              (5000)
#define SVC_STREAMING_STATS_UPDATE_INTERVAL_MS      (1000)

/***************************** Enum and Structure ****************************/

/* Event message types for streaming service */
typedef enum {
    SVC_STREAMING_EVENT_START = 0,
    SVC_STREAMING_EVENT_STOP,
    SVC_STREAMING_EVENT_CONFIG_CHANGE,
    SVC_STREAMING_EVENT_MAX
} SVC_STREAMING_EVENT_E;

/* Event message structure */
typedef struct {
    long mtype;
    SVC_STREAMING_EVENT_E eEventType;
    int32_t nData;
} SVC_STREAMING_EVENT_MSG_T;

/***************************** Static Variable *******************************/
static bool s_bSvcStreamingLog = OFF;
static int32_t s_nSvcStreamingTaskMsgId = -1;
static key_t s_SvcStreamingTaskMsgKey = SVC_STREAMING_TASK_MSG_KEY;
static pthread_t s_hSvcStreamingTask;
static pthread_mutex_t s_stSvcStreamingMutex = PTHREAD_MUTEX_INITIALIZER;

/* Global streaming state variables */
static SVC_STREAMING_CONFIG_T s_stCurrentConfig;
static SVC_STREAMING_STATS_T s_stCurrentStats;

#if defined(CONFIG_VIDEO_STREAMING)
/* Core components */
static DI_CAMERA_T s_stDiCamera;
static DI_VIDEO_T s_stDiVideo;
static DI_VIDEO_NVIDIA_T s_stDiVideoNvidia;
static DI_RING_BUFFER_T *s_pstTxRingBuffer = NULL;
static DI_RING_BUFFER_T *s_pstRxRingBuffer = NULL;
static DI_MEMORY_POOL_T *s_pstMemoryPool = NULL;

/* TCP connection settings */
static char s_achRemoteHost[256] = "192.168.1.100";
static int32_t s_nRemotePort = 8554;
static int32_t s_nLocalPort = 8554;

/* Service state */
static SVC_STREAMING_MODE_E s_eCurrentMode = SVC_STREAMING_MODE_TX;
static SVC_STREAMING_STATUS_E s_eCurrentStatus = SVC_STREAMING_STATUS_STOPPED;

/* Threading */
static pthread_t s_hTxThread;
static pthread_t s_hRxThread;
static pthread_t s_hStatsThread;
static bool s_bTxThreadRunning = FALSE;
static bool s_bRxThreadRunning = FALSE;
static bool s_bStatsThreadRunning = FALSE;
#endif

/***************************** Function  *************************************/

#if defined(CONFIG_VIDEO_STREAMING)
/*
 * Statistics collection thread
 */
static void *P_SVC_STREAMING_StatsThread(void *pvArg)
{
    struct timeval stCurrentTime;
    struct timeval stLastUpdateTime;
    
    UNUSED(pvArg);
    
    PrintTrace("Statistics thread started");
    
    gettimeofday(&stLastUpdateTime, NULL);
    
    while (s_bStatsThreadRunning)
    {
        gettimeofday(&stCurrentTime, NULL);
        
        /* Update statistics every second */
        if ((stCurrentTime.tv_sec - stLastUpdateTime.tv_sec) >= 1)
        {
            pthread_mutex_lock(&s_stSvcStreamingMutex);
            
            /* Update ring buffer statistics */
            if (s_pstTxRingBuffer != NULL)
            {
                DI_RING_BUFFER_STATS_T stTxStats;
                if (DI_RING_BUFFER_GetStats(s_pstTxRingBuffer, &stTxStats) == DI_OK)
                {
                    s_stCurrentStats.ullTotalFramesTx = stTxStats.ullTotalWriteCount;
                    s_stCurrentStats.unDroppedFramesTx = stTxStats.unOverflowCount;
                    s_stCurrentStats.unBufferUsage = stTxStats.unCurrentUsedSize;
                }
            }
            
            if (s_pstRxRingBuffer != NULL)
            {
                DI_RING_BUFFER_STATS_T stRxStats;
                if (DI_RING_BUFFER_GetStats(s_pstRxRingBuffer, &stRxStats) == DI_OK)
                {
                    s_stCurrentStats.ullTotalFramesRx = stRxStats.ullTotalReadCount;
                    s_stCurrentStats.unDroppedFramesRx = stRxStats.unUnderflowCount;
                    /* RX buffer usage handled above */
                }
            }
            
            /* Update network statistics */
            s_stCurrentStats.unNetworkLatency = 50; /* TODO: Implement actual measurement */
            s_stCurrentStats.unNetworkLatency = 50; /* TODO: Implement actual measurement */
            
            pthread_mutex_unlock(&s_stSvcStreamingMutex);
            
            stLastUpdateTime = stCurrentTime;
        }
        
        usleep(SVC_STREAMING_STATS_UPDATE_INTERVAL_MS * 1000);
    }
    
    PrintTrace("Statistics thread stopped");
    return NULL;
}

/*
 * TX thread for video streaming
 */
static void *P_SVC_STREAMING_TxThread(void *pvArg)
{
    DI_CAMERA_FRAME_T stFrame;
    int32_t nRet = DI_ERROR;
    
    UNUSED(pvArg);
    
    PrintTrace("TX thread started");
    
    while (s_bTxThreadRunning)
    {
        /* TX mode uses GStreamer v4l2src directly - no manual frame processing needed */
        /* This thread just monitors the TX streaming status */
        usleep(5000000); /* Check every 5 seconds */
    }
    
    PrintTrace("TX thread stopped");
    return NULL;
}

/*
 * RX thread for video streaming
 */
static void *P_SVC_STREAMING_RxThread(void *pvArg)
{
    int32_t nRet = DI_ERROR;
    
    UNUSED(pvArg);
    
    PrintTrace("RX thread started");
    
    while (s_bRxThreadRunning)
    {
        if (s_pstRxRingBuffer != NULL)
        {
            /* Read encoded frame from RX ring buffer */
            uint8_t achDataBuffer[1024 * 1024]; /* 1MB buffer */
            uint32_t unReadSize = 0;
            nRet = DI_RING_BUFFER_Read(s_pstRxRingBuffer, achDataBuffer, 1024 * 1024, &unReadSize);
            
            if (nRet == DI_OK && unReadSize > 0)
            {
                /* TODO: Send frame to video decoder pipeline */
                /* For now, just consume the data */
                PrintDebug("Received frame: %u bytes", unReadSize);
            }
            else
            {
                usleep(33000); /* ~30 FPS */
            }
        }
        else
        {
            usleep(100000); /* 100ms */
        }
    }
    
    PrintTrace("RX thread stopped");
    return NULL;
}

/*
 * Initialize ring buffers and memory pool
 */
static int32_t P_SVC_STREAMING_InitBuffers(void)
{
    int32_t nRet = DI_ERROR;
    DI_RING_BUFFER_CONFIG_T stRingBufferConfig;
    DI_MEMORY_POOL_CONFIG_T stMemoryPoolConfig;
    
    /* Initialize memory pool */
    memset(&stMemoryPoolConfig, 0, sizeof(DI_MEMORY_POOL_CONFIG_T));
    stMemoryPoolConfig.unPoolSize = DI_MEMORY_POOL_DEFAULT_SIZE;
    stMemoryPoolConfig.unBlockSize = DI_MEMORY_POOL_DEFAULT_BLOCK_SIZE;
    stMemoryPoolConfig.bCorruptionDetection = TRUE;
    
    s_pstMemoryPool = malloc(sizeof(DI_MEMORY_POOL_T));
    if (s_pstMemoryPool == NULL)
    {
        PrintError("Failed to allocate memory pool structure");
        nRet = DI_ERROR_MEMORY_ALLOC;
        goto EXIT;
    }
    
    nRet = DI_MEMORY_POOL_Init(s_pstMemoryPool, &stMemoryPoolConfig);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize memory pool [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Initialize TX ring buffer */
    memset(&stRingBufferConfig, 0, sizeof(DI_RING_BUFFER_CONFIG_T));
    stRingBufferConfig.unBufferSize = DI_RING_BUFFER_DEFAULT_SIZE;
    stRingBufferConfig.bDropOnOverflow = TRUE;
    stRingBufferConfig.bEnableStats = TRUE;
    
    s_pstTxRingBuffer = malloc(sizeof(DI_RING_BUFFER_T));
    if (s_pstTxRingBuffer == NULL)
    {
        PrintError("Failed to allocate TX ring buffer structure");
        nRet = DI_ERROR_MEMORY_ALLOC;
        goto EXIT;
    }
    
    nRet = DI_RING_BUFFER_Init(s_pstTxRingBuffer, &stRingBufferConfig);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize TX ring buffer [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Initialize RX ring buffer */
    s_pstRxRingBuffer = malloc(sizeof(DI_RING_BUFFER_T));
    if (s_pstRxRingBuffer == NULL)
    {
        PrintError("Failed to allocate RX ring buffer structure");
        nRet = DI_ERROR_MEMORY_ALLOC;
        goto EXIT;
    }
    
    nRet = DI_RING_BUFFER_Init(s_pstRxRingBuffer, &stRingBufferConfig);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize RX ring buffer [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("Streaming buffers initialized successfully");
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Initialize camera subsystem
 */
static int32_t P_SVC_STREAMING_InitCamera(void)
{
    int32_t nRet = DI_ERROR;
    DI_CAMERA_CONFIG_T stCameraConfig;
    
    /* Initialize camera */
    memset(&s_stDiCamera, 0, sizeof(DI_CAMERA_T));
    s_stDiCamera.bLogLevel = s_bSvcStreamingLog;
    
    nRet = DI_CAMERA_Init(&s_stDiCamera);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize camera [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Configure camera */
    memset(&stCameraConfig, 0, sizeof(DI_CAMERA_CONFIG_T));
    stCameraConfig.unDeviceId = 0;
    stCameraConfig.unWidth = s_stCurrentConfig.unWidth;
    stCameraConfig.unHeight = s_stCurrentConfig.unHeight;
    stCameraConfig.unFps = s_stCurrentConfig.unFrameRate;
    stCameraConfig.eFormat = eDI_CAMERA_FORMAT_YUV420;
    stCameraConfig.bHardwareAccel = s_stCurrentConfig.bHardwareAcceleration;
    
    nRet = DI_CAMERA_SetConfig(&s_stDiCamera, &stCameraConfig);
    if (nRet != DI_OK)
    {
        PrintError("Failed to configure camera [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Connect camera to TX ring buffer */
    nRet = DI_CAMERA_ConnectVideoStreaming(&s_stDiCamera, s_pstTxRingBuffer);
    if (nRet != DI_OK)
    {
        PrintError("Failed to connect camera to streaming [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("Camera subsystem initialized successfully");
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Initialize video subsystem
 */
static int32_t P_SVC_STREAMING_InitVideo(void)
{
    int32_t nRet = DI_ERROR;
    
    /* Initialize video */
    memset(&s_stDiVideo, 0, sizeof(DI_VIDEO_T));
    s_stDiVideo.bLogLevel = s_bSvcStreamingLog;
    
    nRet = DI_VIDEO_Init(&s_stDiVideo);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize video [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Initialize video nvidia for TCP streaming */
    memset(&s_stDiVideoNvidia, 0, sizeof(DI_VIDEO_NVIDIA_T));
    s_stDiVideoNvidia.bLogLevel = s_bSvcStreamingLog;
    
    nRet = DI_VIDEO_NVIDIA_Init(&s_stDiVideoNvidia);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize video nvidia [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Set default TCP connection */
    nRet = DI_VIDEO_NVIDIA_SetTcpConnection(&s_stDiVideoNvidia, s_achRemoteHost, s_nRemotePort, s_nLocalPort);
    if (nRet != DI_OK)
    {
        PrintError("Failed to set TCP connection [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("Video subsystem initialized successfully");
    nRet = DI_OK;
    
EXIT:
    return nRet;
}
#endif /* CONFIG_VIDEO_STREAMING */

/*
 * Main service task
 */
static void *P_SVC_STREAMING_Task(void *pvArg)
{
    SVC_STREAMING_EVENT_MSG_T stEventMsg;
    int32_t nRet = FRAMEWORK_ERROR;
    
    UNUSED(pvArg);
    
    memset(&stEventMsg, 0, sizeof(SVC_STREAMING_EVENT_MSG_T));
    
    while (1)
    {
        if (msgrcv(s_nSvcStreamingTaskMsgId, &stEventMsg, sizeof(SVC_STREAMING_EVENT_MSG_T), 0, MSG_NOERROR) == FRAMEWORK_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch (stEventMsg.eEventType)
            {
                case SVC_STREAMING_EVENT_START:
                {
                    PrintTrace("SVC_STREAMING_EVENT_START received");
                    /* TODO: Handle start event */
                    break;
                }
                
                case SVC_STREAMING_EVENT_STOP:
                {
                    PrintTrace("SVC_STREAMING_EVENT_STOP received");
                    /* TODO: Handle stop event */
                    break;
                }
                
                case SVC_STREAMING_EVENT_CONFIG_CHANGE:
                {
                    PrintTrace("SVC_STREAMING_EVENT_CONFIG_CHANGE received");
                    /* TODO: Handle config change event */
                    break;
                }
                
                default:
                {
                    PrintWarn("Unknown event type [%d]", stEventMsg.eEventType);
                    break;
                }
            }
        }
        
        usleep(1000);
    }
    
    return NULL;
}

/*
 * Create service task
 */
static int32_t P_SVC_STREAMING_CreateTask(void)
{
    int32_t nRet = FRAMEWORK_ERROR;
    pthread_attr_t stAttr;
    
    pthread_attr_init(&stAttr);
    pthread_attr_setdetachstate(&stAttr, PTHREAD_CREATE_DETACHED);
    
    nRet = pthread_create(&s_hSvcStreamingTask, &stAttr, P_SVC_STREAMING_Task, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_create() is failed! (P_SVC_STREAMING_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_SVC_STREAMING_Task() is successfully created");
        nRet = FRAMEWORK_OK;
    }
    
    pthread_attr_destroy(&stAttr);
    
    return nRet;
}

/*
 * Print message queue information
 */
static void P_SVC_STREAMING_PrintMsgInfo(int32_t nMsqId)
{
    struct msqid_ds stMsgStat;
    
    PrintDebug("========== Streaming Service Message Queue Information =============");
    
    if (msgctl(nMsqId, IPC_STAT, &stMsgStat) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgctl() is failed!");
    }
    else
    {
        PrintDebug("msg_lspid : %d", stMsgStat.msg_lspid);
        PrintDebug("msg_qnum : %ld", stMsgStat.msg_qnum);
        PrintDebug("msg_stime : %ld", stMsgStat.msg_stime);
    }
    
    PrintDebug("====================================================================");
}

/*
 * Initialize streaming service
 */
int32_t SVC_STREAMING_Init(SVC_STREAMING_T *pstSvcStreaming, SVC_STREAMING_CONFIG_T *pstConfig)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL || pstConfig == NULL)
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    /* Copy configuration */
    memcpy(&s_stCurrentConfig, pstConfig, sizeof(SVC_STREAMING_CONFIG_T));
    
    /* Initialize statistics */
    memset(&s_stCurrentStats, 0, sizeof(SVC_STREAMING_STATS_T));
    
#if defined(CONFIG_VIDEO_STREAMING)
    /* Initialize buffers */
    nRet = P_SVC_STREAMING_InitBuffers();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_SVC_STREAMING_InitBuffers() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Initialize camera */
    nRet = P_SVC_STREAMING_InitCamera();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_SVC_STREAMING_InitCamera() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Initialize video */
    nRet = P_SVC_STREAMING_InitVideo();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_SVC_STREAMING_InitVideo() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
#endif
    
    /* Create message queue */
    s_nSvcStreamingTaskMsgId = msgget(s_SvcStreamingTaskMsgKey, IPC_CREAT | 0666);
    if (s_nSvcStreamingTaskMsgId == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        nRet = FRAMEWORK_ERROR;
        goto EXIT;
    }
    else
    {
        P_SVC_STREAMING_PrintMsgInfo(s_nSvcStreamingTaskMsgId);
    }
    
    /* Create task */
    nRet = P_SVC_STREAMING_CreateTask();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_SVC_STREAMING_CreateTask() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    pstSvcStreaming->eStatus = SVC_STREAMING_STATUS_INITIALIZED;
    s_eCurrentStatus = SVC_STREAMING_STATUS_INITIALIZED;
    s_bSvcStreamingLog = pstSvcStreaming->bLogLevel;
    
    PrintTrace("SVC_STREAMING is successfully initialized");
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

/*
 * Deinitialize streaming service
 */
int32_t SVC_STREAMING_DeInit(SVC_STREAMING_T *pstSvcStreaming)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL)
    {
        PrintError("pstSvcStreaming == NULL!");
        goto EXIT;
    }
    
#if defined(CONFIG_VIDEO_STREAMING)
    /* Stop streaming if active */
    if (s_eCurrentStatus == SVC_STREAMING_STATUS_RUNNING)
    {
        SVC_STREAMING_Stop(pstSvcStreaming);
    }
    
    /* Deinitialize subsystems */
    DI_CAMERA_DeInit(&s_stDiCamera);
    DI_VIDEO_DeInit(&s_stDiVideo);
    
    /* Clean up buffers */
    if (s_pstTxRingBuffer != NULL)
    {
        DI_RING_BUFFER_DeInit(s_pstTxRingBuffer);
        free(s_pstTxRingBuffer);
        s_pstTxRingBuffer = NULL;
    }
    
    if (s_pstRxRingBuffer != NULL)
    {
        DI_RING_BUFFER_DeInit(s_pstRxRingBuffer);
        free(s_pstRxRingBuffer);
        s_pstRxRingBuffer = NULL;
    }
    
    if (s_pstMemoryPool != NULL)
    {
        DI_MEMORY_POOL_DeInit(s_pstMemoryPool);
        free(s_pstMemoryPool);
        s_pstMemoryPool = NULL;
    }
#endif
    
    pstSvcStreaming->eStatus = SVC_STREAMING_STATUS_UNINITIALIZED;
    
    PrintTrace("SVC_STREAMING is successfully deinitialized");
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

/*
 * Start streaming service
 */
int32_t SVC_STREAMING_Start(SVC_STREAMING_T *pstSvcStreaming, SVC_STREAMING_MODE_E eMode)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL)
    {
        PrintError("pstSvcStreaming == NULL!");
        goto EXIT;
    }
    
    if (eMode >= SVC_STREAMING_MODE_MAX)
    {
        PrintError("Invalid streaming mode [%d]", eMode);
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    
    if (s_eCurrentStatus == SVC_STREAMING_STATUS_RUNNING)
    {
        PrintWarn("Streaming service is already started");
        nRet = FRAMEWORK_OK;
        pthread_mutex_unlock(&s_stSvcStreamingMutex);
        goto EXIT;
    }
    
    s_eCurrentMode = eMode;
    
#if defined(CONFIG_VIDEO_STREAMING)
    /* Start camera */
    nRet = DI_CAMERA_Open(&s_stDiCamera);
    if (nRet != DI_OK)
    {
        PrintError("Failed to open camera [nRet:%d]", nRet);
        pthread_mutex_unlock(&s_stSvcStreamingMutex);
        goto EXIT;
    }
    
    nRet = DI_CAMERA_Start(&s_stDiCamera);
    if (nRet != DI_OK)
    {
        PrintError("Failed to start camera [nRet:%d]", nRet);
        pthread_mutex_unlock(&s_stSvcStreamingMutex);
        goto EXIT;
    }
    
    /* Start video */
    nRet = DI_VIDEO_Open(&s_stDiVideo);
    if (nRet != DI_OK)
    {
        PrintError("Failed to open video [nRet:%d]", nRet);
        pthread_mutex_unlock(&s_stSvcStreamingMutex);
        goto EXIT;
    }
    
    nRet = DI_VIDEO_Start(&s_stDiVideo);
    if (nRet != DI_OK)
    {
        PrintError("Failed to start video [nRet:%d]", nRet);
        pthread_mutex_unlock(&s_stSvcStreamingMutex);
        goto EXIT;
    }
    
    /* Start TCP streaming based on mode */
    if (eMode == SVC_STREAMING_MODE_TX || eMode == SVC_STREAMING_MODE_BOTH)
    {
        /* Start TX mode (Camera -> TCP Server) */
        nRet = DI_VIDEO_NVIDIA_StartTxMode(&s_stDiVideoNvidia, 
                                          s_stCurrentConfig.unWidth,
                                          s_stCurrentConfig.unHeight,
                                          s_stCurrentConfig.unFrameRate,
                                          s_stCurrentConfig.unBitrate,
                                          s_stCurrentConfig.unCodecType,
                                          s_stCurrentConfig.unFormatType,
                                          s_stCurrentConfig.unIFrameInterval,
                                          s_stCurrentConfig.unPresetLevel);
        if (nRet != DI_OK)
        {
            PrintError("Failed to start TX mode [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stSvcStreamingMutex);
            goto EXIT;
        }
        
        s_bTxThreadRunning = TRUE;
        if (pthread_create(&s_hTxThread, NULL, P_SVC_STREAMING_TxThread, NULL) != 0)
        {
            PrintError("Failed to create TX thread");
            s_bTxThreadRunning = FALSE;
            pthread_mutex_unlock(&s_stSvcStreamingMutex);
            nRet = FRAMEWORK_ERROR;
            goto EXIT;
        }
    }
    
    if (eMode == SVC_STREAMING_MODE_RX || eMode == SVC_STREAMING_MODE_BOTH)
    {
        /* Start RX mode (TCP Client -> Display) */
        nRet = DI_VIDEO_NVIDIA_StartRxMode(&s_stDiVideoNvidia);
        if (nRet != DI_OK)
        {
            PrintError("Failed to start RX mode [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stSvcStreamingMutex);
            goto EXIT;
        }
        
        s_bRxThreadRunning = TRUE;
        if (pthread_create(&s_hRxThread, NULL, P_SVC_STREAMING_RxThread, NULL) != 0)
        {
            PrintError("Failed to create RX thread");
            s_bRxThreadRunning = FALSE;
            pthread_mutex_unlock(&s_stSvcStreamingMutex);
            nRet = FRAMEWORK_ERROR;
            goto EXIT;
        }
    }
    
    /* Start statistics thread */
    s_bStatsThreadRunning = TRUE;
    if (pthread_create(&s_hStatsThread, NULL, P_SVC_STREAMING_StatsThread, NULL) != 0)
    {
        PrintError("Failed to create statistics thread");
        s_bStatsThreadRunning = FALSE;
        pthread_mutex_unlock(&s_stSvcStreamingMutex);
        nRet = FRAMEWORK_ERROR;
        goto EXIT;
    }
#endif
    
    s_eCurrentStatus = SVC_STREAMING_STATUS_RUNNING;
    pstSvcStreaming->eStatus = SVC_STREAMING_STATUS_RUNNING;
    
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    
    PrintTrace("SVC_STREAMING started successfully in mode [%d]", eMode);
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

/*
 * Stop streaming service
 */
int32_t SVC_STREAMING_Stop(SVC_STREAMING_T *pstSvcStreaming)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL)
    {
        PrintError("pstSvcStreaming == NULL!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    
    /* 상태에 관계없이 정리 작업 수행 (안전 장치) */
    bool bNeedCleanup = FALSE;
    if (s_eCurrentStatus == SVC_STREAMING_STATUS_RUNNING)
    {
        bNeedCleanup = TRUE;
        PrintTrace("Stopping running streaming service");
    }
    else
    {
        PrintWarn("Service not running - performing safety cleanup");
        bNeedCleanup = TRUE; /* 강제 정리 */
    }
    
    if (bNeedCleanup == TRUE)
    {
#if defined(CONFIG_VIDEO_STREAMING)
        /* 1. 먼저 하드웨어 자원 정리 */
        DI_VIDEO_NVIDIA_Stop(&s_stDiVideoNvidia);
        DI_VIDEO_Stop(&s_stDiVideo);
        DI_CAMERA_Stop(&s_stDiCamera);
        
        /* 2. 그 다음 스레드 종료 */
        s_bTxThreadRunning = FALSE;
        s_bRxThreadRunning = FALSE;
        s_bStatsThreadRunning = FALSE;
        
        /* 3. 최종 자원 해제 */
        DI_CAMERA_Close(&s_stDiCamera);
        DI_VIDEO_Close(&s_stDiVideo);
        DI_CAMERA_DisconnectVideoStreaming(&s_stDiCamera);
#endif
    }
    
    s_eCurrentStatus = SVC_STREAMING_STATUS_STOPPED;
    pstSvcStreaming->eStatus = SVC_STREAMING_STATUS_STOPPED;
    
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    
    PrintTrace("SVC_STREAMING stopped successfully");
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

/*
 * Set streaming configuration
 */
int32_t SVC_STREAMING_SetConfig(SVC_STREAMING_T *pstSvcStreaming, SVC_STREAMING_CONFIG_T *pstConfig)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL || pstConfig == NULL)
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    
    memcpy(&s_stCurrentConfig, pstConfig, sizeof(SVC_STREAMING_CONFIG_T));
    
    PrintTrace("Streaming configuration updated: Resolution[%dx%d] FPS[%d] Bitrate[%d]",
               pstConfig->unWidth, pstConfig->unHeight, pstConfig->unFrameRate, pstConfig->unBitrate);
    
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

/*
 * Get streaming configuration
 */
int32_t SVC_STREAMING_GetConfig(SVC_STREAMING_T *pstSvcStreaming, SVC_STREAMING_CONFIG_T *pstConfig)
{
    int32_t nRet = FRAMEWORK_ERROR;
    if (pstSvcStreaming == NULL || pstConfig == NULL)
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
#if defined(CONFIG_VIDEO_STREAMING)
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    memcpy(pstConfig, &s_stCurrentConfig, sizeof(SVC_STREAMING_CONFIG_T));
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    nRet = FRAMEWORK_OK;
#else
    memset(pstConfig, 0, sizeof(SVC_STREAMING_CONFIG_T));
    nRet = FRAMEWORK_OK;
#endif
EXIT:
    return nRet;
}

/*
 * Get streaming statistics
 */
int32_t SVC_STREAMING_GetStats(SVC_STREAMING_T *pstSvcStreaming, SVC_STREAMING_STATS_T *pstStats)
{
    int32_t nRet = FRAMEWORK_ERROR;
    if (pstSvcStreaming == NULL || pstStats == NULL)
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
#if defined(CONFIG_VIDEO_STREAMING)
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    memcpy(pstStats, &s_stCurrentStats, sizeof(SVC_STREAMING_STATS_T));
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    nRet = FRAMEWORK_OK;
#else
    memset(pstStats, 0, sizeof(SVC_STREAMING_STATS_T));
    nRet = FRAMEWORK_OK;
#endif
EXIT:
    return nRet;
}

/*
 * Set log level
 */
int32_t SVC_STREAMING_SetLog(SVC_STREAMING_T *pstSvcStreaming, bool bLogLevel)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL)
    {
        PrintError("pstSvcStreaming == NULL!");
        goto EXIT;
    }
    
    s_bSvcStreamingLog = bLogLevel;
    PrintTrace("SET:s_bSvcStreamingLog [%s]", s_bSvcStreamingLog == ON ? "ON" : "OFF");
    
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

/*
 * Display service status
 */
void SVC_STREAMING_Status(SVC_STREAMING_T *pstSvcStreaming)
{
    if (pstSvcStreaming == NULL)
    {
        PrintError("pstSvcStreaming == NULL!");
        return;
    }
    
    PrintDebug("========== Streaming Service Status Information =============");
    PrintDebug("Service Status: %d", pstSvcStreaming->eStatus);
#if defined(CONFIG_VIDEO_STREAMING)
    PrintDebug("Current Mode: %d", s_eCurrentMode);
#endif
#if defined(CONFIG_VIDEO_STREAMING)
    PrintDebug("Current Status: %d", s_eCurrentStatus);
#endif
    PrintDebug("Configuration:");
#if defined(CONFIG_VIDEO_STREAMING)
    PrintDebug("  Resolution: %dx%d", s_stCurrentConfig.unWidth, s_stCurrentConfig.unHeight);
    PrintDebug("  FPS: %d", s_stCurrentConfig.unFrameRate);
    PrintDebug("  Bitrate: %d", s_stCurrentConfig.unBitrate);
    PrintDebug("  Codec: %s", (s_stCurrentConfig.unCodecType == 0) ? "H.264" : 
                              (s_stCurrentConfig.unCodecType == 1) ? "H.265" : "MJPEG");
    PrintDebug("  Format: %s", (s_stCurrentConfig.unFormatType == 0) ? "YUYV" : 
                               (s_stCurrentConfig.unFormatType == 1) ? "MJPEG" : "NV12");
    PrintDebug("  I-Frame Interval: %d", s_stCurrentConfig.unIFrameInterval);
    PrintDebug("  Preset Level: %d", s_stCurrentConfig.unPresetLevel);
    PrintDebug("  Hardware Accel: %s", s_stCurrentConfig.bHardwareAcceleration ? "YES" : "NO");
#endif
    PrintDebug("Statistics:");
#if defined(CONFIG_VIDEO_STREAMING)
    PrintDebug("  TX Frames: %llu (Dropped: %d)", s_stCurrentStats.ullTotalFramesTx, s_stCurrentStats.unDroppedFramesTx);
    PrintDebug("  RX Frames: %llu (Dropped: %d)", s_stCurrentStats.ullTotalFramesRx, s_stCurrentStats.unDroppedFramesRx);
    PrintDebug("  Network Latency: %d ms", s_stCurrentStats.unNetworkLatency);
    PrintDebug("  Network Latency: %d ms", s_stCurrentStats.unNetworkLatency);
#endif
    PrintDebug("Log Level: %s", s_bSvcStreamingLog == ON ? "ON" : "OFF");
    PrintDebug("=============================================================");
}

#if defined(CONFIG_VIDEO_STREAMING)
/*
 * Set TCP connection parameters
 */
int32_t SVC_STREAMING_SetTcpConnection(SVC_STREAMING_T *pstSvcStreaming, 
                                       const char *pchRemoteHost, 
                                       int32_t nRemotePort, 
                                       int32_t nLocalPort)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL || pchRemoteHost == NULL)
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
    if (nRemotePort <= 0 || nRemotePort > 65535 || nLocalPort <= 0 || nLocalPort > 65535)
    {
        PrintError("Invalid port numbers! Remote[%d] Local[%d]", nRemotePort, nLocalPort);
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    
    /* Update TCP connection settings */
    strncpy(s_achRemoteHost, pchRemoteHost, sizeof(s_achRemoteHost) - 1);
    s_achRemoteHost[sizeof(s_achRemoteHost) - 1] = '\0';
    s_nRemotePort = nRemotePort;
    s_nLocalPort = nLocalPort;
    
    /* Set TCP connection in video nvidia */
    nRet = DI_VIDEO_NVIDIA_SetTcpConnection(&s_stDiVideoNvidia, s_achRemoteHost, s_nRemotePort, s_nLocalPort);
    if (nRet != DI_OK)
    {
        PrintError("Failed to set TCP connection in video nvidia [nRet:%d]", nRet);
        pthread_mutex_unlock(&s_stSvcStreamingMutex);
        goto EXIT;
    }
    
    PrintTrace("TCP connection set: Remote[%s:%d] Local[%d]", s_achRemoteHost, s_nRemotePort, s_nLocalPort);
    
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    
    nRet = FRAMEWORK_OK;
    
EXIT:
    return nRet;
}

/*
 * Check TCP connection status
 */
int32_t SVC_STREAMING_CheckTcpConnection(SVC_STREAMING_T *pstSvcStreaming)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL)
    {
        PrintError("pstSvcStreaming == NULL!");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    
    /* Check TCP connection in video nvidia */
    nRet = DI_VIDEO_NVIDIA_CheckTcpConnection(&s_stDiVideoNvidia);
    if (nRet != DI_OK)
    {
        PrintDebug("TCP connection check failed [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("TCP connection is active");
    }
    
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    
EXIT:
    return nRet;
}

/*
 * Print status wrapper function for CLI
 */
void SVC_STREAMING_PrintStatus(SVC_STREAMING_T *pstStreaming)
{
    if (pstStreaming != NULL)
    {
        SVC_STREAMING_Status(pstStreaming);
    }
}

#endif /* CONFIG_VIDEO_STREAMING */
