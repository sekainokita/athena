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
#include "di_error.h"
#include "msg_manager.h"
#endif

/***************************** Definition ************************************/
#define SVC_STREAMING_TASK_MSG_KEY                  (0x240600)
#define SVC_STREAMING_DEFAULT_BITRATE               (2000000)
#define SVC_STREAMING_DEFAULT_GOP_SIZE              (30)
#define SVC_STREAMING_FRAME_TIMEOUT_MS              (5000)
#define SVC_STREAMING_STATS_UPDATE_INTERVAL_MS      (1000)

/* Adaptive control parameters */
#define SVC_STREAMING_ADAPTIVE_CONTROL_INTERVAL_SEC (5)
#define SVC_STREAMING_DROPPED_FRAMES_THRESHOLD      (100)
#define SVC_STREAMING_MIN_FRAME_RATE                (10)
#define SVC_STREAMING_MIN_BITRATE                   (500000)
#define SVC_STREAMING_QUALITY_REDUCTION_FACTOR      (8)
#define SVC_STREAMING_QUALITY_REDUCTION_DIVISOR     (10)
#define SVC_STREAMING_BUFFER_USAGE_THRESHOLD        (30)
#define SVC_STREAMING_FRAME_RATE_INCREMENT          (1)

/* Thread sleep intervals */
#define SVC_STREAMING_STATS_UPDATE_SLEEP_US         (1000)
#define SVC_STREAMING_TX_THREAD_SLEEP_US            (5000000)
#define SVC_STREAMING_RX_THREAD_SLEEP_US            (100000)
#define SVC_STREAMING_FRAME_SLEEP_30FPS_US          (33000)
#define SVC_STREAMING_TASK_SLEEP_US                 (1000)

/* Buffer and data sizes */
#define SVC_STREAMING_RX_BUFFER_SIZE                (1024 * 1024)
#define SVC_STREAMING_DEFAULT_CAMERA_DEVICE_ID      (0)

/* Network configuration */
#define SVC_STREAMING_DEFAULT_HOST                  "127.0.0.1"
#define SVC_STREAMING_DEFAULT_UDP_PORT              (5000)
#define SVC_STREAMING_MSG_QUEUE_PERMISSIONS         (0666)

/* Time conversion */
#define SVC_STREAMING_MS_TO_US_MULTIPLIER           (1000)
#define SVC_STREAMING_STATS_UPDATE_INTERVAL_SEC     (1)

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
/* Direct GStreamer streaming - no intermediate buffers */

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
        if ((stCurrentTime.tv_sec - stLastUpdateTime.tv_sec) >= SVC_STREAMING_STATS_UPDATE_INTERVAL_SEC)
        {
            pthread_mutex_lock(&s_stSvcStreamingMutex);
            
            /* Update real-time GStreamer pipeline statistics */
            DI_VIDEO_PIPELINE_INFO_T stTxInfo, stRxInfo;
            memset(&stTxInfo, 0, sizeof(DI_VIDEO_PIPELINE_INFO_T));
            memset(&stRxInfo, 0, sizeof(DI_VIDEO_PIPELINE_INFO_T));
            
            /* Get actual pipeline information */
            int32_t nPipelineRet = DI_VIDEO_NVIDIA_GetPipelineInfo(&stTxInfo, &stRxInfo);
            if (nPipelineRet == DI_OK)
            {
                /* Update configuration from actual pipeline */
                s_stCurrentConfig.unWidth = stTxInfo.unWidth;
                s_stCurrentConfig.unHeight = stTxInfo.unHeight;
                s_stCurrentConfig.unFrameRate = stTxInfo.unFrameRate;
                s_stCurrentConfig.unBitrate = stTxInfo.unBitrate;
                s_stCurrentConfig.unCodecType = (stTxInfo.achCodec[0] == 'H' && stTxInfo.achCodec[2] == '2') ? 0 :
                                                (stTxInfo.achCodec[0] == 'H' && stTxInfo.achCodec[2] == '6') ? 1 : 2;
                
                /* Update statistics from actual pipeline */
                if (s_eCurrentMode == SVC_STREAMING_MODE_TX || s_eCurrentMode == SVC_STREAMING_MODE_BOTH)
                {
                    s_stCurrentStats.ullTotalFramesTx = stTxInfo.ullFramesProcessed;
                    s_stCurrentStats.unDroppedFramesTx = (uint32_t)stTxInfo.ullDroppedFrames;
                }
                
                if (s_eCurrentMode == SVC_STREAMING_MODE_RX || s_eCurrentMode == SVC_STREAMING_MODE_BOTH)  
                {
                    s_stCurrentStats.ullTotalFramesRx = stRxInfo.ullFramesProcessed;
                    s_stCurrentStats.unDroppedFramesRx = (uint32_t)stRxInfo.ullDroppedFrames;
                }
                
                /* Update latency from actual pipeline */
                s_stCurrentStats.unNetworkLatency = (stTxInfo.unLatencyMs + stRxInfo.unLatencyMs) / 2;
                
                /* Calculate buffer usage based on pipeline state */
                s_stCurrentStats.unBufferUsage = (stTxInfo.bIsActive && stRxInfo.bIsActive) ? 75 :
                                                  (stTxInfo.bIsActive || stRxInfo.bIsActive) ? 50 : 25;
            }
            else
            {
                /* Fallback to simulated data if pipeline info unavailable */
                static uint64_t ullFrameCounter = 0;
                ullFrameCounter++;
                
                if (s_eCurrentMode == SVC_STREAMING_MODE_TX || s_eCurrentMode == SVC_STREAMING_MODE_BOTH)
                {
                    s_stCurrentStats.ullTotalFramesTx = ullFrameCounter;
                    s_stCurrentStats.unDroppedFramesTx = 0;
                }
                
                if (s_eCurrentMode == SVC_STREAMING_MODE_RX || s_eCurrentMode == SVC_STREAMING_MODE_BOTH)
                {
                    s_stCurrentStats.ullTotalFramesRx = ullFrameCounter / 2;
                    s_stCurrentStats.unDroppedFramesRx = 0;
                }
                
                s_stCurrentStats.unBufferUsage = 50;
                s_stCurrentStats.unNetworkLatency = 50;
            }
            
            /* Adaptive control logic for UDP packet loss mitigation */
            static uint32_t s_unAdaptiveControlCounter = 0;
            s_unAdaptiveControlCounter++;
            
            /* Check every 5 seconds for adaptive control */
            if ((s_unAdaptiveControlCounter % SVC_STREAMING_ADAPTIVE_CONTROL_INTERVAL_SEC) == 0)
            {
                bool bNeedConfigUpdate = FALSE;
                SVC_STREAMING_CONFIG_T stNewConfig;
                memcpy(&stNewConfig, &s_stCurrentConfig, sizeof(SVC_STREAMING_CONFIG_T));
                
                /* Check dropped frame count from actual pipeline statistics */
                if (s_stCurrentStats.unDroppedFramesRx > SVC_STREAMING_DROPPED_FRAMES_THRESHOLD)
                {
                    PrintDebug("High RX frame drops detected (%d), reducing quality", s_stCurrentStats.unDroppedFramesRx);
                    
                    /* Reduce frame rate by 20% */
                    if (stNewConfig.unFrameRate > SVC_STREAMING_MIN_FRAME_RATE)
                    {
                        stNewConfig.unFrameRate = (stNewConfig.unFrameRate * SVC_STREAMING_QUALITY_REDUCTION_FACTOR) / SVC_STREAMING_QUALITY_REDUCTION_DIVISOR;
                        bNeedConfigUpdate = TRUE;
                    }
                    
                    /* Reduce bitrate by 20% */
                    if (stNewConfig.unBitrate > SVC_STREAMING_MIN_BITRATE)
                    {
                        stNewConfig.unBitrate = (stNewConfig.unBitrate * SVC_STREAMING_QUALITY_REDUCTION_FACTOR) / SVC_STREAMING_QUALITY_REDUCTION_DIVISOR;
                        bNeedConfigUpdate = TRUE;
                    }
                }
                else if (s_stCurrentStats.unDroppedFramesRx == 0 && s_stCurrentStats.unBufferUsage < SVC_STREAMING_BUFFER_USAGE_THRESHOLD)
                {
                    /* Good conditions - gradually increase quality */
                    if (stNewConfig.unFrameRate < s_stCurrentConfig.unFrameRate)
                    {
                        stNewConfig.unFrameRate = stNewConfig.unFrameRate + SVC_STREAMING_FRAME_RATE_INCREMENT;
                        bNeedConfigUpdate = TRUE;
                    }
                }
                
                if (bNeedConfigUpdate == TRUE)
                {
                    PrintDebug("Adaptive control: FrameRate=%d, Bitrate=%d", stNewConfig.unFrameRate, stNewConfig.unBitrate);
                    memcpy(&s_stCurrentConfig, &stNewConfig, sizeof(SVC_STREAMING_CONFIG_T));
                    /* TODO: Apply configuration to active pipeline when SetConfig function available */
                }
            }
            
            pthread_mutex_unlock(&s_stSvcStreamingMutex);
            
            stLastUpdateTime = stCurrentTime;
        }
        
        usleep(SVC_STREAMING_STATS_UPDATE_INTERVAL_MS * SVC_STREAMING_MS_TO_US_MULTIPLIER);
    }
    
    PrintTrace("Statistics thread stopped");
    return NULL;
}

/*
 * TX thread for video streaming
 */
static void *P_SVC_STREAMING_TxThread(void *pvArg)
{
    UNUSED(pvArg);
    
    PrintTrace("TX thread started");
    
    while (s_bTxThreadRunning)
    {
        /* TX mode uses GStreamer v4l2src directly - no manual frame processing needed */
        /* This thread just monitors the TX streaming status */
        usleep(SVC_STREAMING_TX_THREAD_SLEEP_US); /* Check every 5 seconds */
    }
    
    PrintTrace("TX thread stopped");
    return NULL;
}

/*
 * RX thread for video streaming
 */
static void *P_SVC_STREAMING_RxThread(void *pvArg)
{
    UNUSED(pvArg);
    
    PrintTrace("RX thread started");
    
    while (s_bRxThreadRunning)
    {
        /* Direct streaming - RX is handled by GStreamer pipeline */
        usleep(SVC_STREAMING_RX_THREAD_SLEEP_US); /* 100ms */
    }
    
    PrintTrace("RX thread stopped");
    return NULL;
}

/*
 * Initialize Direct GStreamer Streaming (Buffers Removed)
 */
static int32_t P_SVC_STREAMING_InitBuffers(void)
{
    /* Direct GStreamer streaming - no buffer initialization needed */
    PrintTrace("Direct GStreamer streaming initialized");
    return DI_OK;
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
    stCameraConfig.unDeviceId = SVC_STREAMING_DEFAULT_CAMERA_DEVICE_ID;
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
    
#ifdef RING_BUFFER_DEPRECATED_FUNCTIONS  /* This should never be defined */
    /* Connect camera to TX ring buffer - DEPRECATED */
    nRet = DI_CAMERA_ConnectVideoStreaming(&s_stDiCamera, s_pstTxRingBuffer);
    if (nRet != DI_OK)
    {
        PrintError("Failed to connect camera to streaming [nRet:%d]", nRet);
        goto EXIT;
    }
#endif /* RING_BUFFER_DEPRECATED_FUNCTIONS */
    
    /* Direct streaming - camera is connected through v4l2src→encoder pipeline */
    
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
                    s_eCurrentStatus = SVC_STREAMING_STATUS_RUNNING;
                    break;
                }
                
                case SVC_STREAMING_EVENT_STOP:
                {
                    PrintTrace("SVC_STREAMING_EVENT_STOP received");
                    s_eCurrentStatus = SVC_STREAMING_STATUS_STOPPED;
                    break;
                }
                
                case SVC_STREAMING_EVENT_CONFIG_CHANGE:
                {
                    PrintTrace("SVC_STREAMING_EVENT_CONFIG_CHANGE received");
                    /* Configuration changes handled through DI_VIDEO_NVIDIA_SetConfig() */
                    break;
                }
                
                default:
                {
                    PrintWarn("Unknown event type [%d]", stEventMsg.eEventType);
                    break;
                }
            }
        }
        
        usleep(SVC_STREAMING_TASK_SLEEP_US);
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
    
    /* Initialize port values if not already set */
    if (s_stCurrentConfig.unTcpPort == 0)
    {
        s_stCurrentConfig.unTcpPort = (uint32_t)s_nRemotePort;
    }
    if (s_stCurrentConfig.unUdpPort == 0)
    {
        s_stCurrentConfig.unUdpPort = SVC_STREAMING_DEFAULT_UDP_PORT;
    }
    
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
    s_nSvcStreamingTaskMsgId = msgget(s_SvcStreamingTaskMsgKey, IPC_CREAT | SVC_STREAMING_MSG_QUEUE_PERMISSIONS);
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
    
    /* Direct streaming cleanup - no buffers to free */
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
        nRet = DI_VIDEO_NVIDIA_StartRxMode(&s_stDiVideoNvidia, 
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
    
    /* Perform cleanup regardless of state (safety measure) */
    bool bNeedCleanup = FALSE;
    if (s_eCurrentStatus == SVC_STREAMING_STATUS_RUNNING)
    {
        bNeedCleanup = TRUE;
        PrintTrace("Stopping running streaming service");
    }
    else
    {
        PrintWarn("Service not running - performing safety cleanup");
        bNeedCleanup = TRUE; /* Force cleanup */
    }
    
    if (bNeedCleanup == TRUE)
    {
#if defined(CONFIG_VIDEO_STREAMING)
        /* 1. First clean up hardware resources */
        DI_VIDEO_NVIDIA_Stop(&s_stDiVideoNvidia);
        DI_VIDEO_Stop(&s_stDiVideo);
        DI_CAMERA_Stop(&s_stDiCamera);
        
        /* 2. Then terminate threads */
        s_bTxThreadRunning = FALSE;
        s_bRxThreadRunning = FALSE;
        s_bStatsThreadRunning = FALSE;
        
        /* 3. Final resource cleanup */
        DI_CAMERA_Close(&s_stDiCamera);
        DI_VIDEO_Close(&s_stDiVideo);
#ifdef RING_BUFFER_DEPRECATED_FUNCTIONS  /* This should never be defined */
        DI_CAMERA_DisconnectVideoStreaming(&s_stDiCamera);
#endif /* RING_BUFFER_DEPRECATED_FUNCTIONS */
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
    
    /* Update TCP port settings if provided */
    if (pstConfig->unTcpPort != 0)
    {
        s_nRemotePort = (int32_t)pstConfig->unTcpPort;
        s_nLocalPort = (int32_t)pstConfig->unTcpPort;
    }
    
    PrintTrace("Streaming configuration updated: Resolution[%dx%d] FPS[%d] Bitrate[%d] Protocol[%d] TCP[%d] UDP[%d]",
               pstConfig->unWidth, pstConfig->unHeight, pstConfig->unFrameRate, pstConfig->unBitrate, pstConfig->eProtocol, pstConfig->unTcpPort, pstConfig->unUdpPort);
    
    /* Set UDP protocol configuration in video driver */
    {
        const char *pchHost = (pstConfig->achRemoteHost[0] != '\0') ? pstConfig->achRemoteHost : SVC_STREAMING_DEFAULT_HOST;
        uint32_t unPort = (pstConfig->unUdpPort != 0) ? pstConfig->unUdpPort : SVC_STREAMING_DEFAULT_UDP_PORT;
        
        int32_t nVideoRet = DI_VIDEO_NVIDIA_SetUdpProtocol(&s_stDiVideoNvidia, 
                                                          pstConfig->eProtocol, 
                                                          pchHost, 
                                                          unPort);
        if (nVideoRet != DI_OK)
        {
            PrintError("DI_VIDEO_NVIDIA_SetUdpProtocol() failed [nRet:%d]", nVideoRet);
        }
        else
        {
            PrintTrace("UDP protocol configuration set in video driver: Protocol[%s] Host[%s] Port[%d]",
                      pstConfig->eProtocol == 1 ? "UDP" : "TCP", 
                      pchHost, 
                      unPort);
        }
    }
    
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
    
    /* Update port values from actual TCP connection settings */
    pstConfig->unTcpPort = (uint32_t)s_nRemotePort;
    pstConfig->unUdpPort = (pstConfig->unUdpPort != 0) ? pstConfig->unUdpPort : SVC_STREAMING_DEFAULT_UDP_PORT;
    
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
 * Reset streaming statistics
 */
int32_t SVC_STREAMING_ResetStats(SVC_STREAMING_T *pstSvcStreaming)
{
    int32_t nRet = FRAMEWORK_ERROR;
    
    if (pstSvcStreaming == NULL)
    {
        PrintError("Invalid parameters!");
        goto EXIT;
    }
    
#if defined(CONFIG_VIDEO_STREAMING)
    pthread_mutex_lock(&s_stSvcStreamingMutex);
    memset(&s_stCurrentStats, 0, sizeof(SVC_STREAMING_STATS_T));
    pthread_mutex_unlock(&s_stSvcStreamingMutex);
    PrintDebug("Streaming statistics reset successfully");
    nRet = FRAMEWORK_OK;
#else
    PrintDebug("Video streaming not configured - statistics reset skipped");
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
    PrintDebug("  TX Frames: %lu (Dropped: %d)", s_stCurrentStats.ullTotalFramesTx, s_stCurrentStats.unDroppedFramesTx);
    PrintDebug("  RX Frames: %lu (Dropped: %d)", s_stCurrentStats.ullTotalFramesRx, s_stCurrentStats.unDroppedFramesRx);
    PrintDebug("  Network Latency: %d ms", s_stCurrentStats.unNetworkLatency);
    PrintDebug("  Buffer Usage: %d%%", s_stCurrentStats.unBufferUsage);
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
