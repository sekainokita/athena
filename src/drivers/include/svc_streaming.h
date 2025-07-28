#ifndef _SVC_STREAMING_H_
#define _SVC_STREAMING_H_

/******************************************************************************
*
* Copyright (C) 2023 - 2028 KETI, All rights reserved.
*                           (Korea Electronics Technology Institute)
*
******************************************************************************/
/******************************************************************************
*
* @file svc_streaming.h
*
* @note
*
* SVC Streaming Header for V2X Video Streaming Service
*
******************************************************************************/

/***************************** Include ****************************************/
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "type.h"
/* #include "di_ring_buffer.h" -- REMOVED - Direct GStreamer streaming */

#ifdef CONFIG_GSTREAMER
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#endif

/***************************** Definition ************************************/

/* Streaming service constants */
#define SVC_STREAMING_MAX_PIPELINE_NAME       (64)
#define SVC_STREAMING_MAX_ELEMENT_NAME        (32)
#define SVC_STREAMING_DEFAULT_BITRATE         (2000000)    /* 2Mbps */
#define SVC_STREAMING_DEFAULT_FPS             (30)
#define SVC_STREAMING_DEFAULT_WIDTH           (1920)
#define SVC_STREAMING_DEFAULT_HEIGHT          (1080)
#define SVC_STREAMING_BUFFER_SIZE             (4 * 1024 * 1024) /* 4MB */

/* Streaming service error codes */
#define SVC_STREAMING_OK                      (0)
#define SVC_STREAMING_ERROR                   (-1)
#define SVC_STREAMING_PIPELINE_ERROR          (-2)
#define SVC_STREAMING_ELEMENT_ERROR           (-3)
#define SVC_STREAMING_BUFFER_ERROR            (-4)
#define SVC_STREAMING_NETWORK_ERROR           (-5)
#define SVC_STREAMING_TIMEOUT_ERROR           (-6)
#define SVC_STREAMING_INVALID_PARAM_ERROR     (-7)
#define SVC_STREAMING_NOT_INITIALIZED_ERROR   (-8)
#define SVC_STREAMING_ALREADY_RUNNING_ERROR   (-9)
#define SVC_STREAMING_NOT_RUNNING_ERROR       (-10)

/***************************** Enum and Structure ****************************/

/**
* @details SVC Streaming Status
*/
typedef enum {
    SVC_STREAMING_STATUS_UNINITIALIZED       = 0,
    SVC_STREAMING_STATUS_INITIALIZED         = 1,
    SVC_STREAMING_STATUS_PREPARED            = 2,
    SVC_STREAMING_STATUS_RUNNING             = 3,
    SVC_STREAMING_STATUS_PAUSED              = 4,
    SVC_STREAMING_STATUS_STOPPED             = 5,
    SVC_STREAMING_STATUS_ERROR               = 6,
    SVC_STREAMING_STATUS_MAX                 = 255
} SVC_STREAMING_STATUS_E;

/**
* @details SVC Streaming Mode
*/
typedef enum {
    SVC_STREAMING_MODE_TX                    = 0,
    SVC_STREAMING_MODE_RX                    = 1,
    SVC_STREAMING_MODE_BOTH                  = 2,
    SVC_STREAMING_MODE_MAX                   = 255
} SVC_STREAMING_MODE_E;

/**
* @details SVC Streaming Video Format
*/
typedef enum {
    SVC_STREAMING_FORMAT_H264                = 0,
    SVC_STREAMING_FORMAT_H265                = 1,
    SVC_STREAMING_FORMAT_VP8                 = 2,
    SVC_STREAMING_FORMAT_VP9                 = 3,
    SVC_STREAMING_FORMAT_MAX                 = 255
} SVC_STREAMING_FORMAT_E;

/**
* @details SVC Streaming Network Protocol
*/
typedef enum {
    SVC_STREAMING_PROTOCOL_TCP               = 0,
    SVC_STREAMING_PROTOCOL_UDP               = 1,
    SVC_STREAMING_PROTOCOL_MAX               = 255
} SVC_STREAMING_PROTOCOL_E;

/**
* @details SVC Streaming Configuration
*/
typedef struct SVC_STREAMING_CONFIG_t {
    SVC_STREAMING_MODE_E eMode;               /* Streaming mode */
    SVC_STREAMING_FORMAT_E eFormat;           /* Video format */
    SVC_STREAMING_PROTOCOL_E eProtocol;       /* Network protocol: TCP/UDP */
    uint32_t unWidth;                         /* Video width */
    uint32_t unHeight;                        /* Video height */
    uint32_t unFrameRate;                     /* Frame rate */
    uint32_t unBitrate;                       /* Bitrate */
    uint32_t unBufferSize;                    /* Buffer size */
    uint32_t unCodecType;                     /* Codec type: 0=H264, 1=H265, 2=MJPEG */
    uint32_t unFormatType;                    /* Camera format: 0=YUYV, 1=MJPEG, 2=NV12 */
    uint32_t unIFrameInterval;                /* I-frame interval (GOP size) */
    uint32_t unPresetLevel;                   /* Encoder preset level (0=fastest, 3=slowest) */
    uint32_t unTcpPort;                       /* TCP port for streaming */
    uint32_t unUdpPort;                       /* UDP port for streaming */
    bool bHardwareAcceleration;               /* Hardware acceleration */
    bool bEnableDisplay;                      /* Enable display output */
    bool bEnableRecord;                       /* Enable recording */
    bool bEnableRtsp;                         /* Enable RTSP server */
    bool bEnableStats;                        /* Enable statistics */
    char achCameraDevice[64];                 /* Camera device path */
    char achDisplayDevice[64];                /* Display device path */
    char achRecordPath[256];                  /* Recording file path */
    char achRemoteHost[64];                   /* Remote host for streaming */
} SVC_STREAMING_CONFIG_T;

/**
* @details SVC Streaming Statistics
*/
typedef struct SVC_STREAMING_STATS_t {
    uint64_t ullTotalFramesTx;                /* Total transmitted frames */
    uint64_t ullTotalFramesRx;                /* Total received frames */
    uint64_t ullTotalBytesTx;                 /* Total transmitted bytes */
    uint64_t ullTotalBytesRx;                 /* Total received bytes */
    uint32_t unDroppedFramesTx;               /* Dropped frames in TX */
    uint32_t unDroppedFramesRx;               /* Dropped frames in RX */
    uint32_t unCurrentBitrate;                /* Current bitrate */
    uint32_t unCurrentFrameRate;              /* Current frame rate */
    uint32_t unBufferUsage;                   /* Buffer usage percentage */
    uint32_t unNetworkLatency;                /* Network latency (ms) */
    uint32_t unEncodingLatency;               /* Encoding latency (ms) */
    uint32_t unDecodingLatency;               /* Decoding latency (ms) */
} SVC_STREAMING_STATS_T;

#ifdef CONFIG_GSTREAMER
/**
* @details SVC Streaming GStreamer Pipeline
*/
typedef struct SVC_STREAMING_PIPELINE_t {
    GstElement *pGstPipeline;                 /* Main pipeline */
    GstElement *pGstCameraSrc;                /* Camera source */
    GstElement *pGstEncoder;                  /* Video encoder */
    GstElement *pGstDecoder;                  /* Video decoder */
    GstElement *pGstTee;                      /* Tee element */
    GstElement *pGstDisplaySink;              /* Display sink */
    GstElement *pGstRecordSink;               /* Recording sink */
    GstElement *pGstRtspSink;                 /* RTSP sink */
    GstElement *pGstAppSrc;                   /* App source */
    GstElement *pGstAppSink;                  /* App sink */
    GstBus *pGstBus;                          /* Pipeline bus */
    guint unBusWatchId;                       /* Bus watch ID */
    bool bPipelineCreated;                    /* Pipeline creation flag */
} SVC_STREAMING_PIPELINE_T;
#endif

/**
* @details SVC Streaming Main Structure
*/
typedef struct SVC_STREAMING_t {
    /* Configuration and status */
    SVC_STREAMING_CONFIG_T stConfig;          /* Streaming configuration */
    SVC_STREAMING_STATUS_E eStatus;           /* Streaming status */
    SVC_STREAMING_STATS_T stStats;            /* Streaming statistics */
    
    /* Ring buffers removed - now using direct GStreamer pipeline streaming */
    /* DI_RING_BUFFER_T stTxRingBuffer; -- REMOVED */
    /* DI_RING_BUFFER_T stRxRingBuffer; -- REMOVED */
    
#ifdef CONFIG_GSTREAMER
    /* GStreamer pipeline */
    SVC_STREAMING_PIPELINE_T stPipeline;      /* GStreamer pipeline */
#endif
    
    /* Thread management */
    pthread_t hTxThread;                      /* TX thread */
    pthread_t hRxThread;                      /* RX thread */
    pthread_t hStatsThread;                   /* Statistics thread */
    bool bTxThreadRunning;                    /* TX thread running flag */
    bool bRxThreadRunning;                    /* RX thread running flag */
    bool bStatsThreadRunning;                 /* Stats thread running flag */
    
    /* Synchronization */
    pthread_mutex_t hMutex;                   /* Access control mutex */
    pthread_cond_t hCondition;                /* Condition variable */
    
    /* Error handling */
    bool bErrorState;                         /* Error state flag */
    int32_t nLastError;                       /* Last error code */
    
    /* Logging */
    bool bLogLevel;                           /* Log level */
} SVC_STREAMING_T;

/***************************** Function Prototype ****************************/

/* Service lifecycle management */
int32_t SVC_STREAMING_Init(SVC_STREAMING_T *pstStreaming, 
                           SVC_STREAMING_CONFIG_T *pstConfig);
int32_t SVC_STREAMING_DeInit(SVC_STREAMING_T *pstStreaming);
int32_t SVC_STREAMING_Start(SVC_STREAMING_T *pstStreaming, SVC_STREAMING_MODE_E eMode);
int32_t SVC_STREAMING_Stop(SVC_STREAMING_T *pstStreaming);
int32_t SVC_STREAMING_Pause(SVC_STREAMING_T *pstStreaming);
int32_t SVC_STREAMING_Resume(SVC_STREAMING_T *pstStreaming);

/* Data processing functions */
int32_t SVC_STREAMING_ProcessTxData(SVC_STREAMING_T *pstStreaming, 
                                    const uint8_t *puchData, 
                                    uint32_t unDataSize);
int32_t SVC_STREAMING_ProcessRxData(SVC_STREAMING_T *pstStreaming, 
                                    const uint8_t *puchData, 
                                    uint32_t unDataSize);

/* Configuration functions */
int32_t SVC_STREAMING_SetConfig(SVC_STREAMING_T *pstStreaming, 
                                SVC_STREAMING_CONFIG_T *pstConfig);
int32_t SVC_STREAMING_GetConfig(SVC_STREAMING_T *pstStreaming, 
                                SVC_STREAMING_CONFIG_T *pstConfig);
int32_t SVC_STREAMING_GetDefaultConfig(SVC_STREAMING_CONFIG_T *pstConfig);

/* Statistics and monitoring */
int32_t SVC_STREAMING_GetStats(SVC_STREAMING_T *pstStreaming, 
                               SVC_STREAMING_STATS_T *pstStats);
int32_t SVC_STREAMING_ResetStats(SVC_STREAMING_T *pstStreaming);

/* Status and information */
int32_t SVC_STREAMING_GetStatus(SVC_STREAMING_T *pstStreaming, 
                                SVC_STREAMING_STATUS_E *peStatus);
int32_t SVC_STREAMING_IsRunning(SVC_STREAMING_T *pstStreaming, 
                                bool *pbIsRunning);

/* Utility functions */
int32_t SVC_STREAMING_SetLog(SVC_STREAMING_T *pstStreaming, 
                            bool bLogLevel);
void SVC_STREAMING_PrintStatus(SVC_STREAMING_T *pstStreaming);

#if defined(CONFIG_VIDEO_STREAMING)
/* TCP streaming functions */
int32_t SVC_STREAMING_SetTcpConnection(SVC_STREAMING_T *pstStreaming, 
                                       const char *pchRemoteHost, 
                                       int32_t nRemotePort, 
                                       int32_t nLocalPort);
int32_t SVC_STREAMING_CheckTcpConnection(SVC_STREAMING_T *pstStreaming);
#endif

/* GStreamer specific functions */
#ifdef CONFIG_GSTREAMER
int32_t SVC_STREAMING_CreatePipeline(SVC_STREAMING_T *pstStreaming);
int32_t SVC_STREAMING_DestroyPipeline(SVC_STREAMING_T *pstStreaming);
int32_t SVC_STREAMING_StartPipeline(SVC_STREAMING_T *pstStreaming);
int32_t SVC_STREAMING_StopPipeline(SVC_STREAMING_T *pstStreaming);
#endif

/* Thread functions */
void *SVC_STREAMING_TxThread(void *pvArg);
void *SVC_STREAMING_RxThread(void *pvArg);
void *SVC_STREAMING_StatsThread(void *pvArg);


#endif /* _SVC_STREAMING_H_ */