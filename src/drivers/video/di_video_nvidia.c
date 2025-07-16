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
* @file di_video_nvidia.c
*
* This file contains a data format design
*
* @note
*
* V2X DI VIDEO_NVIDIA Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  23.06.29 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include <math.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "di.h"
#include "di_video.h"
#include "di_video_nvidia.h"
#include "svc_streaming.h"

#if defined(CONFIG_VIDEO_STREAMING)
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include "di_ring_buffer.h"
#include "di_error.h"
#include "di_memory_pool.h"
#endif

/***************************** Definition ************************************/

//#define CONFIG_DI_VIDEO_NVIDIA_DEBUG     (1)

#if defined(CONFIG_VIDEO_STREAMING)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_WIDTH    (1920)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_HEIGHT   (1080)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_FPS      (30)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_BITRATE  (6000000)
#define DI_VIDEO_NVIDIA_GST_FRAME_TIMEOUT_MS          (5000)
#define DI_VIDEO_NVIDIA_GST_BUFFER_SIZE               (1024 * 1024)
#endif

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiVideoNvidiaLog = OFF;

static int s_nDiVideoNvidiaTaskMsgId;
static key_t s_DiVideoNvidiaTaskMsgKey = DI_VIDEO_NVIDIA_TASK_MSG_KEY;
static pthread_t sh_DiVideoNvidiaTask;

#if defined(CONFIG_VIDEO_STREAMING)
/* GStreamer pipeline components */
static GstElement *s_hGstPipelineTx = NULL;
static GstElement *s_hGstPipelineRx = NULL;
static GstElement *s_hGstAppSrc = NULL;
static GstElement *s_hGstAppSink = NULL;
static GMainLoop *s_hMainLoop = NULL;
static pthread_t s_hGstMainLoopThread;
static bool s_bGstPipelineActive = FALSE;

/* Ring Buffer integration */
static DI_RING_BUFFER_T *s_pstRingBufferTx = NULL;
static DI_RING_BUFFER_T *s_pstRingBufferRx = NULL;
static DI_MEMORY_POOL_T *s_pstMemoryPool = NULL;
static pthread_mutex_t s_stGstMutex = PTHREAD_MUTEX_INITIALIZER;

/* TCP connection settings */
static char s_achRemoteHost[256] = "127.0.0.1";
static int32_t s_nRemotePort = 8554;
static int32_t s_nLocalPort = 8554;
#endif

/***************************** Function  *************************************/

#if defined(CONFIG_VIDEO_STREAMING)
/*
 * GStreamer Main Loop Thread
 */
static void *P_DI_VIDEO_NVIDIA_GstMainLoopThread(void *pvArg)
{
    UNUSED(pvArg);
    
    PrintTrace("GStreamer main loop started");
    
    if (s_hMainLoop != NULL)
    {
        g_main_loop_run(s_hMainLoop);
    }
    
    PrintTrace("GStreamer main loop stopped");
    return NULL;
}

/*
 * GStreamer Bus Message Handler
 */
static gboolean P_DI_VIDEO_NVIDIA_GstBusCall(GstBus *hBus, GstMessage *pstMsg, gpointer pvUserData)
{
    GMainLoop *hLoop = (GMainLoop *)pvUserData;
    
    UNUSED(hBus);
    
    switch (GST_MESSAGE_TYPE(pstMsg))
    {
        case GST_MESSAGE_EOS:
            PrintTrace("GStreamer EOS received");
            g_main_loop_quit(hLoop);
            break;
            
        case GST_MESSAGE_ERROR:
        {
            gchar *pchDebug = NULL;
            GError *pstError = NULL;
            
            gst_message_parse_error(pstMsg, &pstError, &pchDebug);
            PrintError("GStreamer error: %s", pstError->message);
            
            if (pchDebug != NULL)
            {
                PrintError("Debug info: %s", pchDebug);
                g_free(pchDebug);
            }
            
            g_error_free(pstError);
            g_main_loop_quit(hLoop);
            break;
        }
        
        case GST_MESSAGE_WARNING:
        {
            gchar *pchDebug = NULL;
            GError *pstError = NULL;
            
            gst_message_parse_warning(pstMsg, &pstError, &pchDebug);
            PrintWarn("GStreamer warning: %s", pstError->message);
            
            if (pchDebug != NULL)
            {
                PrintWarn("Debug info: %s", pchDebug);
                g_free(pchDebug);
            }
            
            g_error_free(pstError);
            break;
        }
        
        default:
            break;
    }
    
    return TRUE;
}

/*
 * AppSink New Sample Callback - RX Pipeline
 */
static GstFlowReturn P_DI_VIDEO_NVIDIA_AppSinkNewSample(GstAppSink *hAppSink, gpointer pvUserData)
{
    GstSample *pstSample = NULL;
    GstBuffer *pstBuffer = NULL;
    GstMapInfo stMapInfo;
    int32_t nRet = DI_ERROR;
    
    UNUSED(pvUserData);
    
    pstSample = gst_app_sink_pull_sample(hAppSink);
    if (pstSample == NULL)
    {
        PrintError("Failed to pull sample from appsink");
        return GST_FLOW_ERROR;
    }
    
    pstBuffer = gst_sample_get_buffer(pstSample);
    if (pstBuffer == NULL)
    {
        PrintError("Failed to get buffer from sample");
        gst_sample_unref(pstSample);
        return GST_FLOW_ERROR;
    }
    
    if (gst_buffer_map(pstBuffer, &stMapInfo, GST_MAP_READ) == FALSE)
    {
        PrintError("Failed to map buffer");
        gst_sample_unref(pstSample);
        return GST_FLOW_ERROR;
    }
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* Write decoded frame to RX ring buffer */
    if (s_pstRingBufferRx != NULL)
    {
        nRet = DI_RING_BUFFER_Write(s_pstRingBufferRx, stMapInfo.data, stMapInfo.size);
        if (nRet != DI_OK)
        {
            PrintError("Failed to write to RX ring buffer [nRet:%d]", nRet);
        }
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
    
    gst_buffer_unmap(pstBuffer, &stMapInfo);
    gst_sample_unref(pstSample);
    
    return GST_FLOW_OK;
}

/*
 * AppSrc Need Data Callback - TX Pipeline
 */
static void P_DI_VIDEO_NVIDIA_AppSrcNeedData(GstAppSrc *hAppSrc, guint unSize, gpointer pvUserData)
{
    GstBuffer *pstBuffer = NULL;
    GstMapInfo stMapInfo;
    uint8_t achDataBuffer[DI_VIDEO_NVIDIA_GST_BUFFER_SIZE];
    uint32_t unReadSize = 0;
    int32_t nRet = DI_ERROR;
    
    UNUSED(pvUserData);
    UNUSED(unSize);
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* Read camera frame from TX ring buffer */
    if (s_pstRingBufferTx != NULL)
    {
        nRet = DI_RING_BUFFER_Read(s_pstRingBufferTx, achDataBuffer, DI_VIDEO_NVIDIA_GST_BUFFER_SIZE, &unReadSize);
        if (nRet != DI_OK || unReadSize == 0)
        {
            pthread_mutex_unlock(&s_stGstMutex);
            return;
        }
        
        /* Create GStreamer buffer */
        pstBuffer = gst_buffer_new_allocate(NULL, unReadSize, NULL);
        if (pstBuffer == NULL)
        {
            PrintError("Failed to allocate GStreamer buffer");
            pthread_mutex_unlock(&s_stGstMutex);
            return;
        }
        
        if (gst_buffer_map(pstBuffer, &stMapInfo, GST_MAP_WRITE) == FALSE)
        {
            PrintError("Failed to map GStreamer buffer");
            gst_buffer_unref(pstBuffer);
            pthread_mutex_unlock(&s_stGstMutex);
            return;
        }
        
        memcpy(stMapInfo.data, achDataBuffer, unReadSize);
        gst_buffer_unmap(pstBuffer, &stMapInfo);
        
        /* Push buffer to appsrc */
        if (gst_app_src_push_buffer(hAppSrc, pstBuffer) != GST_FLOW_OK)
        {
            PrintError("Failed to push buffer to appsrc");
            gst_buffer_unref(pstBuffer);
        }
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
}

/*
 * Create TX Pipeline (Camera -> Encoder -> TCP)
 */
static int32_t P_DI_VIDEO_NVIDIA_CreateTxPipeline(uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate)
{
    int32_t nRet = DI_ERROR;
    GstBus *hBus = NULL;
    gchar *pchPipelineDesc = NULL;
    GError *pstError = NULL;
    
    /* Create TX pipeline description with Jetson hardware acceleration */
    pchPipelineDesc = g_strdup_printf(
        "v4l2src device=/dev/video1 do-timestamp=true ! "
        "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
        "nvvidconv ! "
        "video/x-raw(memory:NVMM),format=I420 ! "
        "nvv4l2h264enc bitrate=%d peak-bitrate=%d "
        "iframeinterval=15 insert-sps-pps=true preset-level=1 "
        "profile=4 control-rate=1 ! "
        "h264parse ! "
        "tcpserversink host=0.0.0.0 port=8554 sync=false",
        unWidth, unHeight, unFrameRate,
        unBitrate, unBitrate + 1000000
    );
    
    PrintTrace("TX Pipeline: %s", pchPipelineDesc);
    
    /* Parse and create pipeline */
    s_hGstPipelineTx = gst_parse_launch(pchPipelineDesc, &pstError);
    if (s_hGstPipelineTx == NULL)
    {
        PrintError("Failed to create TX pipeline: %s", pstError->message);
        g_error_free(pstError);
        nRet = DI_ERROR_STREAMING_PIPELINE_CREATE;
        goto EXIT;
    }
    
    /* TX pipeline uses v4l2src -> tcpserversink, no appsrc/appsink needed */
    PrintTrace("TX pipeline created successfully - using v4l2src directly");
    
    /* Set bus message handler */
    hBus = gst_pipeline_get_bus(GST_PIPELINE(s_hGstPipelineTx));
    gst_bus_add_watch(hBus, P_DI_VIDEO_NVIDIA_GstBusCall, s_hMainLoop);
    gst_object_unref(hBus);
    
    nRet = DI_OK;
    
EXIT:
    if (pchPipelineDesc != NULL)
    {
        g_free(pchPipelineDesc);
    }
    
    return nRet;
}

/*
 * Create RX Pipeline (TCP -> Decoder -> 3-way Output)
 */
static int32_t P_DI_VIDEO_NVIDIA_CreateRxPipeline(const char *pchRemoteHost, int32_t nRemotePort)
{
    int32_t nRet = DI_ERROR;
    GstBus *hBus = NULL;
    gchar *pchPipelineDesc = NULL;
    GError *pstError = NULL;
    
    /* Create RX pipeline description with Jetson hardware acceleration */
    pchPipelineDesc = g_strdup_printf(
        "tcpclientsrc host=%s port=%d ! "
        "h264parse ! "
        "nvv4l2decoder ! "
        "nvvidconv ! "
        "video/x-raw,format=I420,width=%d,height=%d ! "
        "tee name=t ! "
        "queue max-size-buffers=2 leaky=downstream ! nveglglessink sync=false force-aspect-ratio=true "
        "t. ! queue max-size-buffers=30 ! videoconvert ! x264enc bitrate=1000 ! mp4mux ! filesink location=/tmp/v2x_recorded.mp4 "
        "t. ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000",
        pchRemoteHost ? pchRemoteHost : "127.0.0.1",
        nRemotePort,
        DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_WIDTH,
        DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_HEIGHT
    );
    
    PrintTrace("RX Pipeline: %s", pchPipelineDesc);
    
    /* Parse and create pipeline */
    s_hGstPipelineRx = gst_parse_launch(pchPipelineDesc, &pstError);
    if (s_hGstPipelineRx == NULL)
    {
        PrintError("Failed to create RX pipeline: %s", pstError->message);
        g_error_free(pstError);
        nRet = DI_ERROR_STREAMING_PIPELINE_CREATE;
        goto EXIT;
    }
    
    /* RX pipeline uses tcpclientsrc -> decoder -> multiple outputs, no appsrc needed */
    PrintTrace("RX pipeline created successfully - using tcpclientsrc directly");
    
    /* Set bus message handler */
    hBus = gst_pipeline_get_bus(GST_PIPELINE(s_hGstPipelineRx));
    gst_bus_add_watch(hBus, P_DI_VIDEO_NVIDIA_GstBusCall, s_hMainLoop);
    gst_object_unref(hBus);
    
    nRet = DI_OK;
    
EXIT:
    if (pchPipelineDesc != NULL)
    {
        g_free(pchPipelineDesc);
    }
    
    return nRet;
}

/*
 * Initialize GStreamer System
 */
static int32_t P_DI_VIDEO_NVIDIA_GstInit(void)
{
    int32_t nRet = DI_ERROR;
    pthread_attr_t stAttr;
    GError *pstError = NULL;
    
    /* Initialize GStreamer */
    if (gst_init_check(NULL, NULL, &pstError) == FALSE)
    {
        PrintError("Failed to initialize GStreamer: %s", pstError->message);
        g_error_free(pstError);
        nRet = DI_ERROR_STREAMING_INIT_FAILED;
        goto EXIT;
    }
    
    /* Create main loop */
    s_hMainLoop = g_main_loop_new(NULL, FALSE);
    if (s_hMainLoop == NULL)
    {
        PrintError("Failed to create GStreamer main loop");
        nRet = DI_ERROR_STREAMING_INIT_FAILED;
        goto EXIT;
    }
    
    /* Create main loop thread */
    pthread_attr_init(&stAttr);
    pthread_attr_setdetachstate(&stAttr, PTHREAD_CREATE_DETACHED);
    
    if (pthread_create(&s_hGstMainLoopThread, &stAttr, P_DI_VIDEO_NVIDIA_GstMainLoopThread, NULL) != 0)
    {
        PrintError("Failed to create GStreamer main loop thread");
        nRet = DI_ERROR_STREAMING_THREAD_CREATE;
        goto EXIT;
    }
    
    pthread_attr_destroy(&stAttr);
    
    PrintTrace("GStreamer system initialized successfully");
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Initialize Ring Buffers and Memory Pool
 */
static int32_t P_DI_VIDEO_NVIDIA_InitBuffers(void)
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
    
    s_pstRingBufferTx = malloc(sizeof(DI_RING_BUFFER_T));
    if (s_pstRingBufferTx == NULL)
    {
        PrintError("Failed to allocate TX ring buffer structure");
        nRet = DI_ERROR_MEMORY_ALLOC;
        goto EXIT;
    }
    
    nRet = DI_RING_BUFFER_Init(s_pstRingBufferTx, &stRingBufferConfig);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize TX ring buffer [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Start TX ring buffer */
    nRet = DI_RING_BUFFER_Start(s_pstRingBufferTx);
    if (nRet != DI_OK)
    {
        PrintError("Failed to start TX ring buffer [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Initialize RX ring buffer */
    s_pstRingBufferRx = malloc(sizeof(DI_RING_BUFFER_T));
    if (s_pstRingBufferRx == NULL)
    {
        PrintError("Failed to allocate RX ring buffer structure");
        nRet = DI_ERROR_MEMORY_ALLOC;
        goto EXIT;
    }
    
    nRet = DI_RING_BUFFER_Init(s_pstRingBufferRx, &stRingBufferConfig);
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize RX ring buffer [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Start RX ring buffer */
    nRet = DI_RING_BUFFER_Start(s_pstRingBufferRx);
    if (nRet != DI_OK)
    {
        PrintError("Failed to start RX ring buffer [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("Ring buffers and memory pool initialized successfully");
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Set TCP connection parameters
 */
static int32_t P_DI_VIDEO_NVIDIA_SetTcpConnection(const char *pchRemoteHost, int32_t nRemotePort, int32_t nLocalPort)
{
    int32_t nRet = DI_ERROR;
    
    if (pchRemoteHost == NULL || nRemotePort <= 0 || nLocalPort <= 0)
    {
        PrintError("Invalid TCP connection parameters");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* Set remote host */
    strncpy(s_achRemoteHost, pchRemoteHost, sizeof(s_achRemoteHost) - 1);
    s_achRemoteHost[sizeof(s_achRemoteHost) - 1] = '\0';
    
    /* Set ports */
    s_nRemotePort = nRemotePort;
    s_nLocalPort = nLocalPort;
    
    PrintTrace("TCP connection set: Remote[%s:%d] Local[%d]", s_achRemoteHost, s_nRemotePort, s_nLocalPort);
    
    pthread_mutex_unlock(&s_stGstMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Start TX mode (Camera -> TCP Server)
 */
static int32_t P_DI_VIDEO_NVIDIA_StartTxMode(uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate)
{
    int32_t nRet = DI_ERROR;
    
    pthread_mutex_lock(&s_stGstMutex);
    
    if (s_bGstPipelineActive == FALSE)
    {
        /* Create TX pipeline with provided configuration */
        nRet = P_DI_VIDEO_NVIDIA_CreateTxPipeline(
            unWidth,
            unHeight,
            unFrameRate,
            unBitrate
        );
        if (nRet != DI_OK)
        {
            PrintError("Failed to create TX pipeline [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        /* Start TX pipeline */
        if (gst_element_set_state(s_hGstPipelineTx, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            PrintError("Failed to start TX pipeline");
            nRet = DI_ERROR_STREAMING_PIPELINE_START;
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        /* Wait for pipeline to reach PLAYING state */
        GstState stState, stPending;
        GstStateChangeReturn stRet = gst_element_get_state(s_hGstPipelineTx, &stState, &stPending, 5 * GST_SECOND);
        
        if (stRet == GST_STATE_CHANGE_SUCCESS && stState == GST_STATE_PLAYING)
        {
            s_bGstPipelineActive = TRUE;
            PrintTrace("TX pipeline started successfully - TCP server listening on port %d", s_nLocalPort);
        }
        else
        {
            PrintError("TX pipeline failed to reach PLAYING state");
            nRet = DI_ERROR_STREAMING_PIPELINE_START;
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Start RX mode (TCP Client -> Display)
 */
static int32_t P_DI_VIDEO_NVIDIA_StartRxMode(void)
{
    int32_t nRet = DI_ERROR;
    
    pthread_mutex_lock(&s_stGstMutex);
    
    if (s_bGstPipelineActive == FALSE)
    {
        /* Create RX pipeline */
        nRet = P_DI_VIDEO_NVIDIA_CreateRxPipeline(s_achRemoteHost, s_nRemotePort);
        if (nRet != DI_OK)
        {
            PrintError("Failed to create RX pipeline [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        /* Start RX pipeline */
        if (gst_element_set_state(s_hGstPipelineRx, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            PrintError("Failed to start RX pipeline");
            nRet = DI_ERROR_STREAMING_PIPELINE_START;
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        /* Wait for pipeline to reach PLAYING state */
        GstState stState, stPending;
        GstStateChangeReturn stRet = gst_element_get_state(s_hGstPipelineRx, &stState, &stPending, 10 * GST_SECOND);
        
        if (stRet == GST_STATE_CHANGE_SUCCESS && stState == GST_STATE_PLAYING)
        {
            s_bGstPipelineActive = TRUE;
            PrintTrace("RX pipeline started successfully - Connected to TCP server %s:%d", s_achRemoteHost, s_nRemotePort);
        }
        else
        {
            PrintWarn("RX pipeline may take longer to connect - State: %d", stState);
            s_bGstPipelineActive = TRUE; /* Set as active for async connection */
        }
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Check TCP connection status
 */
static int32_t P_DI_VIDEO_NVIDIA_CheckTcpConnection(void)
{
    int32_t nRet = DI_ERROR;
    GstState stState, stPending;
    
    pthread_mutex_lock(&s_stGstMutex);
    
    if (s_hGstPipelineTx != NULL)
    {
        gst_element_get_state(s_hGstPipelineTx, &stState, &stPending, 0);
        PrintTrace("TX Pipeline State: %s", gst_element_state_get_name(stState));
        
        if (stState == GST_STATE_PLAYING)
        {
            nRet = DI_OK;
        }
    }
    
    if (s_hGstPipelineRx != NULL)
    {
        gst_element_get_state(s_hGstPipelineRx, &stState, &stPending, 0);
        PrintTrace("RX Pipeline State: %s", gst_element_state_get_name(stState));
        
        if (stState == GST_STATE_PLAYING)
        {
            nRet = DI_OK;
        }
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
    
    return nRet;
}
#endif /* CONFIG_VIDEO_STREAMING */
static void *P_DI_VIDEO_NVIDIA_Task(void *arg)
{
    DI_VIDEO_NVIDIA_EVENT_MSG_T stEventMsg;
    int32_t nRet = APP_ERROR;
    memset(&stEventMsg, 0, sizeof(DI_VIDEO_NVIDIA_EVENT_MSG_T));

    UNUSED(arg);
    UNUSED(nRet);

    while (1)
    {
        if(msgrcv(s_nDiVideoNvidiaTaskMsgId, &stEventMsg, sizeof(DI_VIDEO_NVIDIA_EVENT_MSG_T), 0, MSG_NOERROR) == APP_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            switch(stEventMsg.eEventType)
            {
                case eDI_VIDEO_NVIDIA_EVENT_START:
                {
                    PrintWarn("eDI_VIDEO_NVIDIA_EVENT_START is received.");
                    break;
                }

                case eDI_VIDEO_NVIDIA_EVENT_STOP:
                {
                    PrintWarn("eDI_VIDEO_NVIDIA_EVENT_STOP is received.");
                    break;
                }

                default:
                    PrintWarn("unknown event type [%d]", stEventMsg.eEventType);
                    break;
            }
        }

        usleep(1000);
    }

    return NULL;
}

static void P_DI_VIDEO_NVIDIA_PrintMsgInfo(int msqid)
{
    struct msqid_ds m_stat;

    PrintDebug("========== Messege Queue Infomation =============");

    if(msgctl(msqid, IPC_STAT, &m_stat) == DI_MSG_ERR)
    {
        PrintError("msgctl() is failed!!");
    }

    PrintDebug("msg_lspid : %d", m_stat.msg_lspid);
    PrintDebug("msg_qnum : %ld", m_stat.msg_qnum);
    PrintDebug("msg_stime : %ld", m_stat.msg_stime);

    PrintDebug("=================================================");
}

int32_t P_DI_VIDEO_NVIDIA_CreateTask(void)
{
	int32_t nRet = APP_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_DiVideoNvidiaTask, &attr, P_DI_VIDEO_NVIDIA_Task, NULL);
    if (nRet != APP_OK)
    {
        PrintError("pthread_create() is failed!! (P_DI_VIDEO_NVIDIA_Task) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_DI_VIDEO_NVIDIA_Task() is successfully created.");
        nRet = APP_OK;
    }

	return nRet;
}

static int32_t P_DI_VIDEO_NVIDIA_Init(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        goto EXIT;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }

#if defined(CONFIG_VIDEO_STREAMING)
    /* Initialize GStreamer system */
    nRet = P_DI_VIDEO_NVIDIA_GstInit();
    if (nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_GstInit() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    /* Initialize Ring Buffers and Memory Pool */
    nRet = P_DI_VIDEO_NVIDIA_InitBuffers();
    if (nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_InitBuffers() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }
#endif

    if((s_nDiVideoNvidiaTaskMsgId = msgget(s_DiVideoNvidiaTaskMsgKey, IPC_CREAT|0666)) == DI_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        nRet = DI_ERROR;
        goto EXIT;
    }
    else
    {
        P_DI_VIDEO_NVIDIA_PrintMsgInfo(s_nDiVideoNvidiaTaskMsgId);
    }

    nRet = P_DI_VIDEO_NVIDIA_CreateTask();
    if (nRet != APP_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_CreateTask() is failed! [nRet:%d]", nRet);
        goto EXIT;
    }

    nRet = DI_OK;

EXIT:
    return nRet;
}
static int32_t P_DI_VIDEO_NVIDIA_DeInit(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        return nRet;
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_SetLog(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    s_bDiVideoNvidiaLog = pstDiVideoNvidia->bLogLevel;
    PrintTrace("SET:s_bDiVideoNvidiaLog [%s]", s_bDiVideoNvidiaLog == ON ? "ON" : "OFF");

    nRet = DI_OK;

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Get(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        (void*)memset(&pstDiVideoNvidia->stDiVideoNvidiaData, 0x00, sizeof(DI_VIDEO_NVIDIA_DATA_T));

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiVideoNvidia->eDiVideoNvidiaStatus >= DI_VIDEO_NVIDIA_STATUS_OPENED)
    {
        /* TODO */
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Open(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_OPENED;

        nRet = DI_OK;
        return nRet;
    }

    if((pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_INITIALIZED) || (pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_CLOSED))
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_OPENED;
        nRet = DI_OK;
    }
    else
    {
        PrintWarn("check the status of VIDEO_NVIDIA [%d]", pstDiVideoNvidia->eDiVideoNvidiaStatus);

        if(pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_OPENED)
        {
            PrintDebug("already DI_VIDEO_NVIDIA_STATUS_OPENED");
            nRet = DI_OK;
        }
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Close(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_CLOSED;

        nRet = DI_OK;
        return nRet;
    }

    if(pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_OPENED)
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_CLOSED;

    }
    else
    {
        PrintWarn("check the status of VIDEO_NVIDIA [%d]", pstDiVideoNvidia->eDiVideoNvidiaStatus);

        if(pstDiVideoNvidia->eDiVideoNvidiaStatus == DI_VIDEO_NVIDIA_STATUS_CLOSED)
        {
            PrintDebug("already DI_VIDEO_NVIDIA_STATUS_CLOSED");
        }
    }

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Start(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        goto EXIT;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }

#if defined(CONFIG_VIDEO_STREAMING)
    pthread_mutex_lock(&s_stGstMutex);
    
    if (s_bGstPipelineActive == FALSE)
    {
        /* Create TX pipeline with default configuration */
        nRet = P_DI_VIDEO_NVIDIA_CreateTxPipeline(
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_WIDTH,
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_HEIGHT,
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_FPS,
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_BITRATE
        );
        if (nRet != DI_OK)
        {
            PrintError("P_DI_VIDEO_NVIDIA_CreateTxPipeline() is failed! [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        /* Start TX pipeline */
        if (gst_element_set_state(s_hGstPipelineTx, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            PrintError("Failed to start TX pipeline");
            pthread_mutex_unlock(&s_stGstMutex);
            nRet = DI_ERROR_STREAMING_PIPELINE_START;
            goto EXIT;
        }
        
        s_bGstPipelineActive = TRUE;
        PrintTrace("TX pipeline started successfully");
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
#endif

    nRet = DI_OK;

EXIT:
    return nRet;
}

int32_t DI_VIDEO_NVIDIA_Stop(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        goto EXIT;
    }

    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }

#if defined(CONFIG_VIDEO_STREAMING)
    pthread_mutex_lock(&s_stGstMutex);
    
    if (s_bGstPipelineActive == TRUE)
    {
        /* Stop pipelines */
        if (s_hGstPipelineTx != NULL)
        {
            gst_element_set_state(s_hGstPipelineTx, GST_STATE_NULL);
            gst_object_unref(s_hGstPipelineTx);
            s_hGstPipelineTx = NULL;
        }
        
        if (s_hGstPipelineRx != NULL)
        {
            gst_element_set_state(s_hGstPipelineRx, GST_STATE_NULL);
            gst_object_unref(s_hGstPipelineRx);
            s_hGstPipelineRx = NULL;
        }
        
        s_hGstAppSrc = NULL;
        s_hGstAppSink = NULL;
        s_bGstPipelineActive = FALSE;
        
        PrintTrace("GStreamer pipelines stopped successfully");
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
#endif

    nRet = DI_OK;

EXIT:
    return nRet;
}

void DI_VIDEO_NVIDIA_Status(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    UNUSED(nRet);

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
    }
}

int32_t DI_VIDEO_NVIDIA_Init(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    nRet = P_DI_VIDEO_NVIDIA_Init(pstDiVideoNvidia);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_INITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    s_bDiVideoNvidiaLog = pstDiVideoNvidia->bLogLevel;
    PrintDebug("s_bDiVideoNvidiaLog [%s]", s_bDiVideoNvidiaLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t DI_VIDEO_NVIDIA_DeInit(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;

    if(pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!!");
        return nRet;
    }

    nRet = P_DI_VIDEO_NVIDIA_DeInit(pstDiVideoNvidia);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiVideoNvidia->eDiVideoNvidiaStatus = DI_VIDEO_NVIDIA_STATUS_DEINITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}

#if defined(CONFIG_VIDEO_STREAMING)
/*
 * Public function: Set TCP connection parameters
 */
int32_t DI_VIDEO_NVIDIA_SetTcpConnection(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, const char *pchRemoteHost, int32_t nRemotePort, int32_t nLocalPort)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!");
        goto EXIT;
    }
    
    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    nRet = P_DI_VIDEO_NVIDIA_SetTcpConnection(pchRemoteHost, nRemotePort, nLocalPort);
    if (nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_SetTcpConnection() failed [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("TCP connection parameters set successfully");
    
EXIT:
    return nRet;
}

/*
 * Public function: Start TX mode
 */
int32_t DI_VIDEO_NVIDIA_StartTxMode(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!");
        goto EXIT;
    }
    
    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    nRet = P_DI_VIDEO_NVIDIA_StartTxMode(unWidth, unHeight, unFrameRate, unBitrate);
    if (nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_StartTxMode() failed [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("TX mode started successfully");
    
EXIT:
    return nRet;
}

/*
 * Public function: Start RX mode
 */
int32_t DI_VIDEO_NVIDIA_StartRxMode(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!");
        goto EXIT;
    }
    
    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    nRet = P_DI_VIDEO_NVIDIA_StartRxMode();
    if (nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_StartRxMode() failed [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("RX mode started successfully");
    
EXIT:
    return nRet;
}

/*
 * Public function: Check TCP connection status
 */
int32_t DI_VIDEO_NVIDIA_CheckTcpConnection(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia)
{
    int32_t nRet = DI_ERROR;
    
    if (pstDiVideoNvidia == NULL)
    {
        PrintError("pstDiVideoNvidia == NULL!");
        goto EXIT;
    }
    
    if (pstDiVideoNvidia->bVideoNvidiaNotAvailable == TRUE)
    {
        PrintWarn("bVideoNvidiaNotAvailable[%d]", pstDiVideoNvidia->bVideoNvidiaNotAvailable);
        nRet = DI_OK;
        goto EXIT;
    }
    
    nRet = P_DI_VIDEO_NVIDIA_CheckTcpConnection();
    if (nRet != DI_OK)
    {
        PrintDebug("TCP connection check failed [nRet:%d]", nRet);
    }
    
EXIT:
    return nRet;
}
#endif /* CONFIG_VIDEO_STREAMING */



