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
#include <gst/rtsp-server/rtsp-server.h>
/* #include "di_ring_buffer.h" -- REMOVED - Direct GStreamer streaming */
#include "di_error.h"
#endif

/***************************** Definition ************************************/

//#define CONFIG_DI_VIDEO_NVIDIA_DEBUG     (1)

#if defined(CONFIG_VIDEO_STREAMING)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_WIDTH    (1920)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_HEIGHT   (1080)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_FPS      (30)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_BITRATE  (6000000)
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_CODEC    (0)      /* 0=H264, 1=H265, 2=MJPEG */
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_IFRAME   (15)     /* I-frame interval (GOP size) */
#define DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_PRESET   (1)      /* Preset level (0=fastest, 3=slowest) */
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

/* RTSP server components */
static GstRTSPServer *s_hRtspServer = NULL;
static GstRTSPMountPoints *s_hRtspMounts = NULL;
static GstRTSPMediaFactory *s_hRtspFactory = NULL;
static guint s_unRtspServerId = 0;

/* Direct GStreamer streaming - no intermediate memory pools */
/* Ring buffers and memory pools removed for better performance */
static pthread_mutex_t s_stGstMutex = PTHREAD_MUTEX_INITIALIZER;

/* Current pipeline configuration tracking */
static uint32_t s_unCurrentWidth = 1920;
static uint32_t s_unCurrentHeight = 1080;
static uint32_t s_unCurrentFrameRate = 30;
static uint32_t s_unCurrentBitrate = 2000000;
static uint32_t s_unCurrentCodecType = 2;      /* MJPEG */
static uint32_t s_unCurrentFormatType = 1;     /* MJPEG */

/* Real frame counters and statistics from GStreamer probes */
static uint64_t s_ullFrameCountTx = 0;
static uint64_t s_ullFrameCountRx = 0;
static uint64_t s_ullDroppedFramesTx = 0;
static uint64_t s_ullDroppedFramesRx = 0;
static uint64_t s_ullUdpBytesTx = 0;
static uint64_t s_ullUdpBytesRx = 0;

/* Real-time rate calculation variables */
static uint64_t s_ullPrevFrameCountTx = 0;
static uint64_t s_ullPrevFrameCountRx = 0;
static uint64_t s_ullPrevUdpBytesTx = 0;
static uint64_t s_ullPrevUdpBytesRx = 0;
static time_t s_tPrevStatsTime = 0;
static double s_dCurrentFrameRateTx = 0.0;
static double s_dCurrentFrameRateRx = 0.0;
static double s_dCurrentByteRateTx = 0.0;  /* Bytes per second TX */
static double s_dCurrentByteRateRx = 0.0;  /* Bytes per second RX */
static pthread_mutex_t s_hStatsMutex = PTHREAD_MUTEX_INITIALIZER;

/* Direct GStreamer pipelines - simplified architecture */

/* Multi-camera configuration */
static const DI_VIDEO_CAMERA_CONFIG_T s_stCameraConfig = {
    .unMaxCameraCount = DI_VIDEO_CAMERA_MAX_COUNT,
    .unTcpPortBase = DI_VIDEO_CAMERA_TCP_PORT_BASE,
    .unRtspPortBase = DI_VIDEO_CAMERA_RTSP_PORT_BASE,
    .unPortOffset = DI_VIDEO_CAMERA_PORT_OFFSET,
    .bDefaultAvailable = DI_VIDEO_CAMERA_DEFAULT_AVAILABLE
};

/* Multi-camera array - Camera 1 is currently active, others unavailable */
static DI_VIDEO_CAMERA_T s_astCameras[DI_VIDEO_CAMERA_MAX_COUNT];

/* Legacy variables for backward compatibility (Camera 1) */
static char s_achRemoteHost[256] = "127.0.0.1";
static int32_t s_nRemotePort = DI_VIDEO_CAMERA_TCP_PORT_BASE;    /* Camera 1 TCP port */
static int32_t s_nLocalPort = DI_VIDEO_CAMERA_TCP_PORT_BASE;     /* Camera 1 TCP port */

/* UDP protocol configuration */
static uint32_t s_unProtocol = 0;                               /* 0=TCP, 1=UDP */
static uint32_t s_unUdpPort = 5000;                             /* UDP port for streaming */
#endif

/***************************** Function  *************************************/

#if defined(CONFIG_VIDEO_STREAMING)

/* Deprecated ring buffer functions removed */

/*
 * GStreamer probe callbacks for real statistics collection
 */
static GstPadProbeReturn P_DI_VIDEO_TxFrameProbe(GstPad *pstPad, GstPadProbeInfo *pstInfo, gpointer pvUserData)
{
    UNUSED(pstPad);
    UNUSED(pvUserData);
    
    if (GST_PAD_PROBE_INFO_TYPE(pstInfo) & GST_PAD_PROBE_TYPE_BUFFER)
    {
        GstBuffer *pstBuffer = GST_PAD_PROBE_INFO_BUFFER(pstInfo);
        if (pstBuffer != NULL)
        {
            pthread_mutex_lock(&s_hStatsMutex);
            s_ullFrameCountTx++;
            s_ullUdpBytesTx += gst_buffer_get_size(pstBuffer);
            pthread_mutex_unlock(&s_hStatsMutex);
        }
    }
    
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn P_DI_VIDEO_RxFrameProbe(GstPad *pstPad, GstPadProbeInfo *pstInfo, gpointer pvUserData)
{
    UNUSED(pstPad);
    UNUSED(pvUserData);
    
    if (GST_PAD_PROBE_INFO_TYPE(pstInfo) & GST_PAD_PROBE_TYPE_BUFFER)
    {
        GstBuffer *pstBuffer = GST_PAD_PROBE_INFO_BUFFER(pstInfo);
        if (pstBuffer != NULL)
        {
            pthread_mutex_lock(&s_hStatsMutex);
            s_ullFrameCountRx++;
            s_ullUdpBytesRx += gst_buffer_get_size(pstBuffer);
            pthread_mutex_unlock(&s_hStatsMutex);
        }
    }
    
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn P_DI_VIDEO_DropProbe(GstPad *pstPad, GstPadProbeInfo *pstInfo, gpointer pvUserData)
{
    UNUSED(pstPad);
    
    if (GST_PAD_PROBE_INFO_TYPE(pstInfo) & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM)
    {
        GstEvent *pstEvent = GST_PAD_PROBE_INFO_EVENT(pstInfo);
        if (GST_EVENT_TYPE(pstEvent) == GST_EVENT_QOS)
        {
            pthread_mutex_lock(&s_hStatsMutex);
            if (pvUserData == (gpointer)1) /* TX drops */
            {
                s_ullDroppedFramesTx++;
            }
            else /* RX drops */
            {
                s_ullDroppedFramesRx++;
            }
            pthread_mutex_unlock(&s_hStatsMutex);
        }
    }
    
    return GST_PAD_PROBE_OK;
}

/* Helper function to add probe to element pad */
static void P_DI_VIDEO_AddElementProbe(GstElement *pstElement, const char *pchPadName, 
                                       GstPadProbeCallback pfnCallback, gpointer pvUserData)
{
    GstPad *pstPad = NULL;
    
    if (pstElement == NULL || pchPadName == NULL || pfnCallback == NULL)
    {
        return;
    }
    
    pstPad = gst_element_get_static_pad(pstElement, pchPadName);
    if (pstPad != NULL)
    {
        gst_pad_add_probe(pstPad, 
                         GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM,
                         pfnCallback, pvUserData, NULL);
        gst_object_unref(pstPad);
        PrintTrace("Probe added to %s:%s", GST_ELEMENT_NAME(pstElement), pchPadName);
    }
    else
    {
        PrintError("Failed to get pad %s from element %s", pchPadName, GST_ELEMENT_NAME(pstElement));
    }
}

/* Calculate real-time transmission rates */
static void P_DI_VIDEO_UpdateRealTimeStats(void)
{
    time_t tCurrentTime = time(NULL);
    
    pthread_mutex_lock(&s_hStatsMutex);
    
    /* Update rates every second */
    if (s_tPrevStatsTime > 0 && (tCurrentTime - s_tPrevStatsTime >= 1))
    {
        time_t tTimeDelta = tCurrentTime - s_tPrevStatsTime;
        
        /* Calculate TX rates */
        uint64_t ullFrameDeltaTx = s_ullFrameCountTx - s_ullPrevFrameCountTx;
        uint64_t ullBytesDeltaTx = s_ullUdpBytesTx - s_ullPrevUdpBytesTx;
        s_dCurrentFrameRateTx = (double)ullFrameDeltaTx / (double)tTimeDelta;
        s_dCurrentByteRateTx = (double)ullBytesDeltaTx / (double)tTimeDelta;
        
        /* Calculate RX rates */
        uint64_t ullFrameDeltaRx = s_ullFrameCountRx - s_ullPrevFrameCountRx;
        uint64_t ullBytesDeltaRx = s_ullUdpBytesRx - s_ullPrevUdpBytesRx;
        s_dCurrentFrameRateRx = (double)ullFrameDeltaRx / (double)tTimeDelta;
        s_dCurrentByteRateRx = (double)ullBytesDeltaRx / (double)tTimeDelta;
        
        /* Update previous values */
        s_ullPrevFrameCountTx = s_ullFrameCountTx;
        s_ullPrevFrameCountRx = s_ullFrameCountRx;
        s_ullPrevUdpBytesTx = s_ullUdpBytesTx;
        s_ullPrevUdpBytesRx = s_ullUdpBytesRx;
        s_tPrevStatsTime = tCurrentTime;
    }
    else if (s_tPrevStatsTime == 0)
    {
        /* Initialize on first call */
        s_ullPrevFrameCountTx = s_ullFrameCountTx;
        s_ullPrevFrameCountRx = s_ullFrameCountRx;
        s_ullPrevUdpBytesTx = s_ullUdpBytesTx;
        s_ullPrevUdpBytesRx = s_ullUdpBytesRx;
        s_tPrevStatsTime = tCurrentTime;
    }
    
    pthread_mutex_unlock(&s_hStatsMutex);
}

/*
 * Camera port calculation functions
 */
static uint32_t P_DI_VIDEO_CAMERA_GetTcpPort(uint32_t unCameraId)
{
    if (!DI_VIDEO_CAMERA_IS_VALID_ID(unCameraId))
    {
        PrintError("Invalid camera ID [%d]. Valid range: 1-%d", unCameraId, DI_VIDEO_CAMERA_MAX_COUNT);
        return 0; /* Invalid */
    }
    return s_stCameraConfig.unTcpPortBase + ((unCameraId - 1) * s_stCameraConfig.unPortOffset);
}

static uint32_t P_DI_VIDEO_CAMERA_GetRtspPort(uint32_t unCameraId)
{
    if (!DI_VIDEO_CAMERA_IS_VALID_ID(unCameraId))
    {
        PrintError("Invalid camera ID [%d]. Valid range: 1-%d", unCameraId, DI_VIDEO_CAMERA_MAX_COUNT);
        return 0; /* Invalid */
    }
    return s_stCameraConfig.unRtspPortBase + ((unCameraId - 1) * s_stCameraConfig.unPortOffset);
}

/*
 * Multi-camera initialization
 */
static int32_t P_DI_VIDEO_CAMERA_InitAll(void)
{
    int32_t nRet = DI_OK;
    uint32_t unCameraIndex = 0;
    
    for (unCameraIndex = 0; unCameraIndex < s_stCameraConfig.unMaxCameraCount; unCameraIndex++)
    {
        uint32_t unCameraId = unCameraIndex + 1;
        
        s_astCameras[unCameraIndex].unCameraId = unCameraId;
        s_astCameras[unCameraIndex].unTcpPort = P_DI_VIDEO_CAMERA_GetTcpPort(unCameraId);
        s_astCameras[unCameraIndex].unRtspPort = P_DI_VIDEO_CAMERA_GetRtspPort(unCameraId);
        s_astCameras[unCameraIndex].bCameraAvailable = s_stCameraConfig.bDefaultAvailable;
        
        /* Only Camera 1 is available by default */
        if (unCameraId == 1)
        {
            s_astCameras[unCameraIndex].bCameraAvailable = TRUE;
        }
        
        PrintTrace("Camera %d initialized: TCP=%d, RTSP=%d, Available=%s",
                   unCameraId,
                   s_astCameras[unCameraIndex].unTcpPort,
                   s_astCameras[unCameraIndex].unRtspPort,
                   s_astCameras[unCameraIndex].bCameraAvailable ? "YES" : "NO");
    }
    
    return nRet;
}
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

/* Deprecated ring buffer callback functions removed */

/*
 * Create TX Pipeline (Camera -> Encoder -> TCP)
 */
static int32_t P_DI_VIDEO_NVIDIA_CreateTxPipeline(uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel, uint32_t unProtocol, const char *pchRemoteHost, uint32_t unUdpPort)
{
    int32_t nRet = DI_ERROR;
    GstBus *hBus = NULL;
    gchar *pchPipelineDesc = NULL;
    GError *pstError = NULL;
    
    /* Create TX pipeline description with format and codec selection */
    const char *pchFormat = (unFormatType == 0) ? "YUY2" : 
                           (unFormatType == 1) ? "MJPG" : "NV12";
    
    PrintTrace("TX Pipeline Parameters: Protocol[%d] Codec[%d] Format[%d] Host[%s] Port[%d]", 
               unProtocol, unCodecType, unFormatType, pchRemoteHost ? pchRemoteHost : "NULL", unUdpPort);
    
    /* MJPEG 포맷이면 코덱 설정과 관계없이 직접 스트리밍 */
    if (unFormatType == 1) /* MJPEG format - 직접 스트리밍 */
    {
        if (unProtocol == 1) /* UDP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video1 do-timestamp=true ! "
                "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                "rtpjpegpay mtu=1400 ! "
                "udpsink host=%s port=%d sync=false async=false buffer-size=%d max-lateness=0",
                unWidth, unHeight, unFrameRate,
                pchRemoteHost, unUdpPort, SVC_STREAMING_BUFFER_SIZE
            );
        }
        else /* TCP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video1 do-timestamp=true ! "
                "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                "tcpserversink host=0.0.0.0 port=8554 sync=false",
                unWidth, unHeight, unFrameRate
            );
        }
        PrintTrace("Using direct MJPEG streaming (format-based)");
    }
    else if (unCodecType == 0) /* H.264 */
    {
        if (unFormatType == 1) /* MJPEG format */
        {
            if (unProtocol == 1) /* UDP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                    "jpegdec ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h264enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=4 control-rate=1 ! "
                    "h264parse ! "
                    "rtph264pay config-interval=1 pt=96 ! "
                    "udpsink host=%s port=%d sync=false buffer-size=%d max-lateness=1000000000",
                    unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel,
                    pchRemoteHost, unUdpPort, SVC_STREAMING_BUFFER_SIZE
                );
            }
            else /* TCP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                    "jpegdec ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h264enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=4 control-rate=1 ! "
                    "h264parse ! "
                    "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                    unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel
                );
            }
        }
        else /* YUYV or NV12 format */
        {
            if (unProtocol == 1) /* UDP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1 ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h264enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=4 control-rate=1 ! "
                    "h264parse ! "
                    "rtph264pay config-interval=1 pt=96 ! "
                    "udpsink host=%s port=%d sync=false buffer-size=%d max-lateness=1000000000",
                    pchFormat, unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel,
                    pchRemoteHost, unUdpPort, SVC_STREAMING_BUFFER_SIZE
                );
            }
            else /* TCP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1 ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h264enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=4 control-rate=1 ! "
                    "h264parse ! "
                    "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                    pchFormat, unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel
                );
            }
        }
    }
    else if (unCodecType == 1) /* H.265 */
    {
        if (unFormatType == 1) /* MJPEG format */
        {
            if (unProtocol == 1) /* UDP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                    "jpegdec ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h265enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=1 control-rate=1 ! "
                    "h265parse ! "
                    "rtph265pay config-interval=1 pt=96 ! "
                    "udpsink host=%s port=%d sync=false buffer-size=0 max-lateness=0",
                    unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel,
                    pchRemoteHost, unUdpPort
                );
            }
            else /* TCP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                    "jpegdec ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h265enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=1 control-rate=1 ! "
                    "h265parse ! "
                    "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                    unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel
                );
            }
        }
        else /* YUYV or NV12 format */
        {
            if (unProtocol == 1) /* UDP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1 ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h265enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=1 control-rate=1 ! "
                    "h265parse ! "
                    "rtph265pay config-interval=1 pt=96 ! "
                    "udpsink host=%s port=%d sync=false buffer-size=0 max-lateness=0",
                    pchFormat, unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel,
                    pchRemoteHost, unUdpPort
                );
            }
            else /* TCP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1 ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvv4l2h265enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                    "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                    "profile=1 control-rate=1 ! "
                    "h265parse ! "
                    "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                    pchFormat, unWidth, unHeight, unFrameRate,
                    unBitrate, unBitrate + 1000000,
                    unIFrameInterval, unPresetLevel
                );
            }
        }
    }
    else if (unCodecType == 2) /* MJPEG */
    {
        if (unFormatType == 1) /* MJPEG format */
        {
            if (unProtocol == 1) /* UDP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                    "rtpjpegpay ! "
                    "udpsink host=%s port=%d sync=false buffer-size=0 max-lateness=0",
                    unWidth, unHeight, unFrameRate,
                    pchRemoteHost, unUdpPort
                );
            }
            else /* TCP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
                    "nvjpegdec ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvjpegenc quality=%d ! "
                    "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                    unWidth, unHeight, unFrameRate,
                    (100 - unPresetLevel * 20) /* Quality: preset 0=100%, 1=80%, 2=60%, 3=40% */
                );
            }
        }
        else /* YUYV format (기본) */
        {
            if (unProtocol == 1) /* UDP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvjpegenc quality=%d ! "
                    "rtpjpegpay ! "
                    "udpsink host=%s port=%d sync=false buffer-size=0 max-lateness=0",
                    unWidth, unHeight, unFrameRate,
                    (100 - unPresetLevel * 20), /* Quality: preset 0=100%, 1=80%, 2=60%, 3=40% */
                    pchRemoteHost, unUdpPort
                );
            }
            else /* TCP */
            {
                pchPipelineDesc = g_strdup_printf(
                    "v4l2src device=/dev/video1 do-timestamp=true ! "
                    "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=I420 ! "
                    "nvjpegenc quality=%d ! "
                    "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                    unWidth, unHeight, unFrameRate,
                    (100 - unPresetLevel * 20) /* Quality: preset 0=100%, 1=80%, 2=60%, 3=40% */
                );
            }
        }
    }
    else
    {
        PrintError("Unsupported codec type: %d", unCodecType);
        nRet = DI_ERROR_INVALID_PARAM;
        goto EXIT;
    }
    
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
    
    /* TX pipeline uses v4l2src -> encoder -> tcpserversink, no appsrc/appsink needed */
    PrintTrace("TX pipeline created successfully - using v4l2src directly");
    
    /* Set bus message handler */
    hBus = gst_pipeline_get_bus(GST_PIPELINE(s_hGstPipelineTx));
    gst_bus_add_watch(hBus, P_DI_VIDEO_NVIDIA_GstBusCall, s_hMainLoop);
    gst_object_unref(hBus);
    
    /* Add probes for real statistics collection */
    /* Reset frame counters */
    pthread_mutex_lock(&s_hStatsMutex);
    s_ullFrameCountTx = 0;
    s_ullDroppedFramesTx = 0;
    s_ullUdpBytesTx = 0;
    pthread_mutex_unlock(&s_hStatsMutex);
    
    /* Add probe to v4l2src for frame counting */
    GstElement *pstV4l2Src = gst_bin_get_by_name(GST_BIN(s_hGstPipelineTx), "v4l2src0");
    if (pstV4l2Src == NULL)
    {
        /* Try to find v4l2src by element type */
        GstIterator *pstIter = gst_bin_iterate_sources(GST_BIN(s_hGstPipelineTx));
        GValue stValue = G_VALUE_INIT;
        GstIteratorResult eResult = GST_ITERATOR_OK;
        
        while (eResult == GST_ITERATOR_OK)
        {
            eResult = gst_iterator_next(pstIter, &stValue);
            if (eResult == GST_ITERATOR_OK)
            {
                GstElement *pstElement = GST_ELEMENT(g_value_get_object(&stValue));
                if (GST_IS_ELEMENT(pstElement))
                {
                    const gchar *pchFactoryName = gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(gst_element_get_factory(pstElement)));
                    if (g_strcmp0(pchFactoryName, "v4l2src") == 0)
                    {
                        pstV4l2Src = gst_object_ref(pstElement);
                        break;
                    }
                }
                g_value_unset(&stValue);
            }
        }
        gst_iterator_free(pstIter);
    }
    
    if (pstV4l2Src != NULL)
    {
        P_DI_VIDEO_AddElementProbe(pstV4l2Src, "src", P_DI_VIDEO_TxFrameProbe, (gpointer)1);
        gst_object_unref(pstV4l2Src);
    }
    
    /* Add probe to UDP sink for data counting */
    GstElement *pstUdpSink = gst_bin_get_by_name(GST_BIN(s_hGstPipelineTx), "udpsink0");
    if (pstUdpSink == NULL)
    {
        /* Try to find udpsink by element type */
        GstIterator *pstIter = gst_bin_iterate_sinks(GST_BIN(s_hGstPipelineTx));
        GValue stValue = G_VALUE_INIT;
        GstIteratorResult eResult = GST_ITERATOR_OK;
        
        while (eResult == GST_ITERATOR_OK)
        {
            eResult = gst_iterator_next(pstIter, &stValue);
            if (eResult == GST_ITERATOR_OK)
            {
                GstElement *pstElement = GST_ELEMENT(g_value_get_object(&stValue));
                if (GST_IS_ELEMENT(pstElement))
                {
                    const gchar *pchFactoryName = gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(gst_element_get_factory(pstElement)));
                    if (g_strcmp0(pchFactoryName, "udpsink") == 0)
                    {
                        pstUdpSink = gst_object_ref(pstElement);
                        break;
                    }
                }
                g_value_unset(&stValue);
            }
        }
        gst_iterator_free(pstIter);
    }
    
    if (pstUdpSink != NULL)
    {
        P_DI_VIDEO_AddElementProbe(pstUdpSink, "sink", P_DI_VIDEO_TxFrameProbe, (gpointer)1);
        P_DI_VIDEO_AddElementProbe(pstUdpSink, "sink", P_DI_VIDEO_DropProbe, (gpointer)1);
        gst_object_unref(pstUdpSink);
    }
    
    nRet = DI_OK;
    
EXIT:
    if (pchPipelineDesc != NULL)
    {
        g_free(pchPipelineDesc);
    }
    
    return nRet;
}

#ifdef UNUSED_TX_ENCODING_PIPELINE  /* This function is not used */
/*
 * Create TX Encoding Pipeline (Ring Buffer -> Encoder -> TCP) - DEPRECATED
 */
static int32_t P_DI_VIDEO_NVIDIA_CreateTxEncodingPipeline(uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unIFrameInterval, uint32_t unPresetLevel, uint32_t unProtocol, const char *pchRemoteHost, uint32_t unUdpPort)
{
    int32_t nRet = DI_ERROR;
    GstBus *hBus = NULL;
    gchar *pchPipelineDesc = NULL;
    GError *pstError = NULL;
    
    if (unCodecType == 0) /* H.264 */
    {
        if (unProtocol == 1) /* UDP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                "queue max-size-buffers=5 max-size-bytes=4194304 leaky=downstream ! "
                "nvvidconv ! "
                "video/x-raw,format=I420 ! "
                "nvv4l2h264enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                "profile=4 control-rate=1 ! "
                "h264parse ! "
                "rtph264pay ! "
                "udpsink host=%s port=%d sync=false buffer-size=0 max-lateness=0",
                unWidth, unHeight, unFrameRate,
                unBitrate, unBitrate + 1000000,
                unIFrameInterval, unPresetLevel,
                pchRemoteHost ? pchRemoteHost : "127.0.0.1", unUdpPort
            );
        }
        else /* TCP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                "queue max-size-buffers=5 max-size-bytes=4194304 leaky=downstream ! "
                "nvvidconv ! "
                "video/x-raw,format=I420 ! "
                "nvv4l2h264enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                "profile=4 control-rate=1 ! "
                "h264parse ! "
                "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                unWidth, unHeight, unFrameRate,
                unBitrate, unBitrate + 1000000,
                unIFrameInterval, unPresetLevel
            );
        }
    }
    else if (unCodecType == 1) /* H.265 */
    {
        if (unProtocol == 1) /* UDP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                "queue max-size-buffers=5 max-size-bytes=4194304 leaky=downstream ! "
                "nvvidconv ! "
                "video/x-raw,format=I420 ! "
                "nvv4l2h265enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                "profile=1 control-rate=1 ! "
                "h265parse ! "
                "rtph265pay ! "
                "udpsink host=%s port=%d sync=false buffer-size=0 max-lateness=0",
                unWidth, unHeight, unFrameRate,
                unBitrate, unBitrate + 1000000,
                unIFrameInterval, unPresetLevel,
                pchRemoteHost ? pchRemoteHost : "127.0.0.1", unUdpPort
            );
        }
        else /* TCP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                "queue max-size-buffers=5 max-size-bytes=4194304 leaky=downstream ! "
                "nvvidconv ! "
                "video/x-raw,format=I420 ! "
                "nvv4l2h265enc maxperf-enable=true bitrate=%d peak-bitrate=%d "
                "iframeinterval=%d insert-sps-pps=true preset-level=%d "
                "profile=1 control-rate=1 ! "
                "h265parse ! "
                "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                unWidth, unHeight, unFrameRate,
                unBitrate, unBitrate + 1000000,
                unIFrameInterval, unPresetLevel
            );
        }
    }
    else if (unCodecType == 2) /* MJPEG */
    {
        if (unProtocol == 1) /* UDP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                "queue max-size-buffers=5 max-size-bytes=4194304 leaky=downstream ! "
                "nvvidconv ! "
                "video/x-raw,format=I420 ! "
                "nvjpegenc quality=%d ! "
                "rtpjpegpay ! "
                "udpsink host=%s port=%d sync=false buffer-size=0 max-lateness=0",
                unWidth, unHeight, unFrameRate,
                (100 - unPresetLevel * 20), /* Quality: preset 0=100%, 1=80%, 2=60%, 3=40% */
                pchRemoteHost ? pchRemoteHost : "127.0.0.1", unUdpPort
            );
        }
        else /* TCP */
        {
            pchPipelineDesc = g_strdup_printf(
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
                "queue max-size-buffers=5 max-size-bytes=4194304 leaky=downstream ! "
                "nvvidconv ! "
                "video/x-raw,format=I420 ! "
                "nvjpegenc quality=%d ! "
                "tcpserversink host=0.0.0.0 port=8554 sync=false max-lateness=0 buffer-size=32768",
                unWidth, unHeight, unFrameRate,
                (100 - unPresetLevel * 20) /* Quality: preset 0=100%, 1=80%, 2=60%, 3=40% */
            );
        }
    }
    else
    {
        PrintError("Unsupported codec type: %d", unCodecType);
        nRet = DI_ERROR_INVALID_PARAM;
        goto EXIT;
    }
    
    PrintTrace("TX Encoding Pipeline: %s", pchPipelineDesc);
    
    /* Parse and create encoding pipeline */
    s_hGstPipelineTx = gst_parse_launch(pchPipelineDesc, &pstError);
    if (s_hGstPipelineTx == NULL)
    {
        PrintError("Failed to create TX encoding pipeline: %s", pstError->message);
        g_error_free(pstError);
        nRet = DI_ERROR_STREAMING_PIPELINE_CREATE;
        goto EXIT;
    }
    
    /* Connect appsrc callback for encoding pipeline */
    s_hGstAppSrc = gst_bin_get_by_name(GST_BIN(s_hGstPipelineTxEncode), "tx_src");
    if (s_hGstAppSrc != NULL)
    {
        /* Set AppSrc properties */
        g_object_set(s_hGstAppSrc,
            "caps", gst_caps_new_simple("video/x-raw",
                "format", G_TYPE_STRING, "I420",
                "width", G_TYPE_INT, unWidth,
                "height", G_TYPE_INT, unHeight,
                "framerate", GST_TYPE_FRACTION, unFrameRate, 1,
                NULL),
            "stream-type", 0, /* GST_APP_STREAM_TYPE_STREAM */
            "max-bytes", G_TYPE_UINT64, 0,
            "block", FALSE,
            "is-live", TRUE,
            "do-timestamp", TRUE,
            NULL);
        
        /* No callbacks needed - we'll push data manually */
        
        /* Push initial dummy buffer to allow pipeline to start */
        GstBuffer *pstDummyBuffer = gst_buffer_new_allocate(NULL, unWidth * unHeight * 3 / 2, NULL);
        if (pstDummyBuffer != NULL)
        {
            GstMapInfo stDummyMapInfo;
            if (gst_buffer_map(pstDummyBuffer, &stDummyMapInfo, GST_MAP_WRITE))
            {
                memset(stDummyMapInfo.data, 0, stDummyMapInfo.size); /* Black frame */
                gst_buffer_unmap(pstDummyBuffer, &stDummyMapInfo);
                
                GST_BUFFER_PTS(pstDummyBuffer) = 0;
                GST_BUFFER_DTS(pstDummyBuffer) = 0;
                GST_BUFFER_DURATION(pstDummyBuffer) = gst_util_uint64_scale(GST_SECOND, 1, 30);
                
                /* Push dummy buffer to prepare AppSrc */
                gst_app_src_push_buffer(GST_APP_SRC(s_hGstAppSrc), pstDummyBuffer);
                PrintTrace("TX AppSrc initialized with dummy buffer");
            }
            else
            {
                gst_buffer_unref(pstDummyBuffer);
            }
        }
        
        PrintTrace("TX AppSrc configured for manual push");
    }
    else
    {
        PrintError("Failed to get TX AppSrc element");
        nRet = DI_ERROR_STREAMING_PIPELINE_CREATE;
        goto EXIT;
    }
    
    /* Set bus message handler */
    hBus = gst_pipeline_get_bus(GST_PIPELINE(s_hGstPipelineTxEncode));
    gst_bus_add_watch(hBus, P_DI_VIDEO_NVIDIA_GstBusCall, s_hMainLoop);
    gst_object_unref(hBus);
    
    PrintTrace("TX encoding pipeline created successfully");
    nRet = DI_OK;
    
EXIT:
    if (pchPipelineDesc != NULL)
    {
        g_free(pchPipelineDesc);
    }
    
    return nRet;
}
#endif /* UNUSED_TX_ENCODING_PIPELINE */

/*
 * Create RX Pipeline (TCP -> Decoder -> 3-way Output)
 */
static int32_t P_DI_VIDEO_NVIDIA_CreateRxPipeline(const char *pchRemoteHost, int32_t nRemotePort, uint32_t unProtocol, uint32_t unUdpPort, uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel)
{
    int32_t nRet = DI_ERROR;
    GstBus *hBus = NULL;
    gchar *pchPipelineDesc = NULL;
    GError *pstError = NULL;
    
    /* Mark unused parameters to avoid compiler warnings */
    UNUSED(unFrameRate);
    UNUSED(unBitrate);
    UNUSED(unFormatType);
    UNUSED(unIFrameInterval);
    UNUSED(unPresetLevel);
    
    /* Create RX pipeline - decode to ring buffer */
    if (unProtocol == 1) /* UDP */
    {
        if (unCodecType == 0) /* H.264 */
        {
            /* H.264 UDP RX with video display window */
            pchPipelineDesc = g_strdup_printf(
                "udpsrc port=%d buffer-size=%d ! "
                "queue max-size-buffers=50 max-size-bytes=%d max-size-time=2000000000 leaky=downstream ! "
                "application/x-rtp,media=video,encoding-name=H264 ! "
                "rtph264depay ! "
                "h264parse ! "
                "queue max-size-buffers=10 leaky=downstream ! "
                "nvv4l2decoder enable-max-performance=true ! "
                "nvvidconv ! "
                "tee name=t ! "
                "queue max-size-buffers=3 leaky=downstream ! "
                "nveglglessink sync=false async=false force-aspect-ratio=true "
                "t. ! queue max-size-buffers=3 leaky=downstream ! "
                "video/x-raw,format=I420,width=%d,height=%d ! "
                "appsink name=rx_sink sync=false max-buffers=3 drop=true",
                unUdpPort, SVC_STREAMING_BUFFER_SIZE, SVC_STREAMING_BUFFER_SIZE, unWidth, unHeight
            );
            PrintTrace("Using H.264 UDP RX pipeline with video display window");
        }
        else if (unCodecType == 1) /* H.265 */
        {
            /* H.265 UDP RX with video display window */
            pchPipelineDesc = g_strdup_printf(
                "udpsrc port=%d buffer-size=%d ! "
                "queue max-size-buffers=50 max-size-bytes=%d max-size-time=2000000000 leaky=downstream ! "
                "application/x-rtp,media=video,encoding-name=H265 ! "
                "rtph265depay ! "
                "h265parse ! "
                "queue max-size-buffers=10 leaky=downstream ! "
                "nvv4l2decoder enable-max-performance=true ! "
                "nvvidconv ! "
                "tee name=t ! "
                "queue max-size-buffers=3 leaky=downstream ! "
                "nveglglessink sync=false async=false force-aspect-ratio=true "
                "t. ! queue max-size-buffers=3 leaky=downstream ! "
                "video/x-raw,format=I420,width=%d,height=%d ! "
                "appsink name=rx_sink sync=false max-buffers=3 drop=true",
                unUdpPort, SVC_STREAMING_BUFFER_SIZE, SVC_STREAMING_BUFFER_SIZE, unWidth, unHeight
            );
            PrintTrace("Using H.265 UDP RX pipeline with video display window");
        }
        else if (unCodecType == 2) /* MJPEG */
        {
            /* MJPEG UDP RX with video display window */
            pchPipelineDesc = g_strdup_printf(
                "udpsrc port=%d buffer-size=%d ! "
                "queue max-size-buffers=50 max-size-bytes=%d max-size-time=2000000000 leaky=downstream ! "
                "application/x-rtp,media=video,payload=26,clock-rate=90000 ! "
                "rtpjpegdepay ! "
                "queue max-size-buffers=10 leaky=downstream ! "
                "jpegdec ! "  /* Use software decoder for better compatibility */
                "videoconvert ! "
                "tee name=t ! "
                "queue max-size-buffers=3 leaky=downstream ! "
                "autovideosink sync=false async=false force-aspect-ratio=true "
                "t. ! queue max-size-buffers=3 leaky=downstream ! "
                "video/x-raw,format=I420,width=%d,height=%d ! "
                "appsink name=rx_sink sync=false max-buffers=3 drop=true",
                unUdpPort, SVC_STREAMING_BUFFER_SIZE, SVC_STREAMING_BUFFER_SIZE, unWidth, unHeight
            );
            PrintTrace("Using MJPEG UDP RX pipeline with video display window");
        }
        else
        {
            PrintError("Unsupported codec type[%d] for UDP RX", unCodecType);
            nRet = DI_ERROR_STREAMING_PIPELINE_CREATE;
            goto EXIT;
        }
    }
    else /* TCP */
    {
        if (unCodecType == 0) /* H.264 */
        {
            /* H.264 TCP RX with video display window */
            pchPipelineDesc = g_strdup_printf(
                "tcpclientsrc host=%s port=%d buffer-size=262144 ! "
                "queue max-size-buffers=50 max-size-bytes=16777216 leaky=downstream ! "
                "h264parse ! "
                "nvv4l2decoder enable-max-performance=true ! "
                "nvvidconv ! "
                "tee name=t ! "
                "queue max-size-buffers=3 leaky=downstream ! "
                "nveglglessink sync=false async=false force-aspect-ratio=true "
                "t. ! queue max-size-buffers=3 leaky=downstream ! "
                "video/x-raw,format=I420,width=%d,height=%d ! "
                "appsink name=rx_sink sync=false max-buffers=2 drop=true",
                pchRemoteHost ? pchRemoteHost : "127.0.0.1",
                nRemotePort, unWidth, unHeight
            );
            PrintTrace("Using H.264 TCP RX pipeline with video display window");
        }
        else if (unCodecType == 1) /* H.265 */
        {
            /* H.265 TCP RX with video display window */
            pchPipelineDesc = g_strdup_printf(
                "tcpclientsrc host=%s port=%d buffer-size=32768 ! "
                "h265parse ! "
                "nvv4l2decoder enable-max-performance=true ! "
                "nvvidconv ! "
                "tee name=t ! "
                "queue max-size-buffers=3 leaky=downstream ! "
                "nveglglessink sync=false async=false force-aspect-ratio=true "
                "t. ! queue max-size-buffers=3 leaky=downstream ! "
                "video/x-raw,format=I420,width=%d,height=%d ! "
                "appsink name=rx_sink sync=false max-buffers=1 drop=true",
                pchRemoteHost ? pchRemoteHost : "127.0.0.1",
                nRemotePort, unWidth, unHeight
            );
            PrintTrace("Using H.265 TCP RX pipeline with video display window");
        }
        else if (unCodecType == 2) /* MJPEG */
        {
            /* MJPEG TCP RX with video display window */
            pchPipelineDesc = g_strdup_printf(
                "tcpclientsrc host=%s port=%d buffer-size=32768 ! "
                "jpegdec ! "  /* Use software decoder for better compatibility */
                "videoconvert ! "
                "tee name=t ! "
                "queue max-size-buffers=3 leaky=downstream ! "
                "autovideosink sync=false async=false force-aspect-ratio=true "
                "t. ! queue max-size-buffers=3 leaky=downstream ! "
                "video/x-raw,format=I420,width=%d,height=%d ! "
                "appsink name=rx_sink sync=false max-buffers=1 drop=true",
                pchRemoteHost ? pchRemoteHost : "127.0.0.1",
                nRemotePort, unWidth, unHeight
            );
            PrintTrace("Using MJPEG TCP RX pipeline with video display window");
        }
        else
        {
            PrintError("Unsupported codec type[%d] for TCP RX", unCodecType);
            nRet = DI_ERROR_STREAMING_PIPELINE_CREATE;
            goto EXIT;
        }
    }
    
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
    
    /* Connect appsink callback for RX pipeline */
    s_hGstAppSink = gst_bin_get_by_name(GST_BIN(s_hGstPipelineRx), "rx_sink");
    if (s_hGstAppSink != NULL)
    {
        GstAppSinkCallbacks stAppSinkCallbacks = {
            .new_sample = NULL,  /* Removed - direct streaming no longer needs callbacks */
            .eos = NULL,
            .new_preroll = NULL
        };
        gst_app_sink_set_callbacks(GST_APP_SINK(s_hGstAppSink), &stAppSinkCallbacks, NULL, NULL);
        PrintTrace("RX AppSink callback connected successfully");
    }
    else
    {
        PrintError("Failed to get RX AppSink element");
        nRet = DI_ERROR_STREAMING_PIPELINE_CREATE;
        goto EXIT;
    }
    
    /* Set bus message handler */
    hBus = gst_pipeline_get_bus(GST_PIPELINE(s_hGstPipelineRx));
    gst_bus_add_watch(hBus, P_DI_VIDEO_NVIDIA_GstBusCall, s_hMainLoop);
    gst_object_unref(hBus);
    
    /* Add probes for real RX statistics collection */
    /* Reset RX frame counters */
    pthread_mutex_lock(&s_hStatsMutex);
    s_ullFrameCountRx = 0;
    s_ullDroppedFramesRx = 0;
    s_ullUdpBytesRx = 0;
    pthread_mutex_unlock(&s_hStatsMutex);
    
    /* Skip udpsrc probe - count only at appsink to avoid RTP packet duplication */
    /* RTP packets are fragmented, so udpsrc will count multiple packets per frame */
    
    /* Add probe to appsink for final frame counting */
    if (s_hGstAppSink != NULL)
    {
        P_DI_VIDEO_AddElementProbe(s_hGstAppSink, "sink", P_DI_VIDEO_RxFrameProbe, (gpointer)0);
        P_DI_VIDEO_AddElementProbe(s_hGstAppSink, "sink", P_DI_VIDEO_DropProbe, (gpointer)0);
    }
    
    PrintTrace("RX pipeline created successfully - direct streaming");
    nRet = DI_OK;
    
EXIT:
    if (pchPipelineDesc != NULL)
    {
        g_free(pchPipelineDesc);
    }
    
    return nRet;
}

/* Deprecated RX Display Pipeline function removed */

/*
 * Create RTSP Server for video streaming
 */
static int32_t P_DI_VIDEO_NVIDIA_CreateRtspServer(void)
{
    int32_t nRet = DI_ERROR;
    gchar *pchFactoryString = NULL;
    
    /* Create RTSP server */
    s_hRtspServer = gst_rtsp_server_new();
    if (s_hRtspServer == NULL)
    {
        PrintError("Failed to create RTSP server");
        nRet = DI_ERROR_STREAMING_RTSP_CREATE;
        goto EXIT;
    }
    
    /* Set server port - use Camera 1 RTSP port (8560) */
    gchar *pchRtspPort = g_strdup_printf("%d", P_DI_VIDEO_CAMERA_GetRtspPort(1));
    gst_rtsp_server_set_service(s_hRtspServer, pchRtspPort);
    g_free(pchRtspPort);
    
    /* Get mount points */
    s_hRtspMounts = gst_rtsp_server_get_mount_points(s_hRtspServer);
    
    /* Create media factory */
    s_hRtspFactory = gst_rtsp_media_factory_new();
    
    /* Set factory launch string - restream from TCP to RTSP using Camera 1 port */
    pchFactoryString = g_strdup_printf(
        "( tcpclientsrc host=127.0.0.1 port=%d buffer-size=32768 ! "
        "h264parse ! rtph264pay name=pay0 pt=96 )",
        P_DI_VIDEO_CAMERA_GetTcpPort(1)
    );
    
    gst_rtsp_media_factory_set_launch(s_hRtspFactory, pchFactoryString);
    gst_rtsp_media_factory_set_shared(s_hRtspFactory, TRUE);
    
    /* Mount the factory at /stream */
    gst_rtsp_mount_points_add_factory(s_hRtspMounts, "/stream", s_hRtspFactory);
    
    g_object_unref(s_hRtspMounts);
    
    /* Attach server to default main context */
    s_unRtspServerId = gst_rtsp_server_attach(s_hRtspServer, NULL);
    if (s_unRtspServerId == 0)
    {
        PrintError("Failed to attach RTSP server");
        nRet = DI_ERROR_STREAMING_RTSP_ATTACH;
        goto EXIT;
    }
    
    PrintTrace("RTSP server created successfully, streaming at rtsp://localhost:%d/stream", P_DI_VIDEO_CAMERA_GetRtspPort(1));
    nRet = DI_OK;
    
EXIT:
    if (pchFactoryString != NULL)
    {
        g_free(pchFactoryString);
    }
    
    return nRet;
}

/*
 * Destroy RTSP Server
 */
static int32_t P_DI_VIDEO_NVIDIA_DestroyRtspServer(void)
{
    int32_t nRet = DI_OK;
    
    if (s_unRtspServerId != 0)
    {
        g_source_remove(s_unRtspServerId);
        s_unRtspServerId = 0;
    }
    
    if (s_hRtspServer != NULL)
    {
        g_object_unref(s_hRtspServer);
        s_hRtspServer = NULL;
    }
    
    /* Factory will be unreferenced automatically when mounts are cleared */
    s_hRtspFactory = NULL;
    s_hRtspMounts = NULL;
    
    PrintTrace("RTSP server destroyed");
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
    
    /* Initialize multi-camera configuration */
    nRet = P_DI_VIDEO_CAMERA_InitAll();
    if (nRet != DI_OK)
    {
        PrintError("Failed to initialize camera configuration [nRet:%d]", nRet);
        goto EXIT;
    }
    
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
 * Initialize Direct GStreamer Streaming (Memory Pool/Ring Buffers Removed)
 */
static int32_t P_DI_VIDEO_NVIDIA_InitBuffers(void)
{
    int32_t nRet = DI_OK;
    
    /* Direct GStreamer streaming - no intermediate buffers needed */
    PrintTrace("Direct GStreamer streaming initialized (memory pools removed)");
    
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
 * Set UDP protocol configuration
 */
static int32_t P_DI_VIDEO_NVIDIA_SetUdpProtocol(uint32_t unProtocol, const char *pchRemoteHost, uint32_t unUdpPort)
{
    int32_t nRet = DI_ERROR;
    
    if (unProtocol > 1 || (unProtocol == 1 && (pchRemoteHost == NULL || unUdpPort == 0)))
    {
        PrintError("Invalid UDP protocol parameters");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* Set protocol type */
    s_unProtocol = unProtocol;
    
    if (unProtocol == 1) /* UDP */
    {
        /* Set remote host for UDP */
        strncpy(s_achRemoteHost, pchRemoteHost, sizeof(s_achRemoteHost) - 1);
        s_achRemoteHost[sizeof(s_achRemoteHost) - 1] = '\0';
        
        /* Set UDP port */
        s_unUdpPort = unUdpPort;
        
        PrintTrace("UDP protocol set: Remote[%s:%d]", s_achRemoteHost, s_unUdpPort);
    }
    else /* TCP */
    {
        PrintTrace("TCP protocol set (default)");
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Start TX mode (Camera -> TCP Server)
 */
static int32_t P_DI_VIDEO_NVIDIA_StartTxMode(uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel)
{
    int32_t nRet = DI_ERROR;
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* Update current configuration tracking */
    s_unCurrentWidth = unWidth;
    s_unCurrentHeight = unHeight;
    s_unCurrentFrameRate = unFrameRate;
    s_unCurrentBitrate = unBitrate;
    s_unCurrentCodecType = unCodecType;
    s_unCurrentFormatType = unFormatType;
    
    if (s_bGstPipelineActive == FALSE)
    {
        /* Create TX pipeline with provided configuration */
        nRet = P_DI_VIDEO_NVIDIA_CreateTxPipeline(
            unWidth,
            unHeight,
            unFrameRate,
            unBitrate,
            unCodecType,
            unFormatType,
            unIFrameInterval,
            unPresetLevel,
            s_unProtocol,
            s_achRemoteHost,
            s_unUdpPort
        );
        if (nRet != DI_OK)
        {
            PrintError("Failed to create TX pipeline [nRet:%d]", nRet);
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        /* Start TX pipeline (camera capture) */
        if (gst_element_set_state(s_hGstPipelineTx, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            PrintError("Failed to start TX pipeline");
            nRet = DI_ERROR_STREAMING_PIPELINE_START;
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        /* Wait for camera pipeline to start and fill ring buffer */
        GstState stState, stPending;
        GstStateChangeReturn stRet = gst_element_get_state(s_hGstPipelineTx, &stState, &stPending, 5 * GST_SECOND);
        
        if (stRet != GST_STATE_CHANGE_SUCCESS || stState != GST_STATE_PLAYING)
        {
            PrintError("TX pipeline failed to reach PLAYING state");
            nRet = DI_ERROR_STREAMING_PIPELINE_START;
            pthread_mutex_unlock(&s_stGstMutex);
            goto EXIT;
        }
        
        if (stRet == GST_STATE_CHANGE_SUCCESS && stState == GST_STATE_PLAYING)
        {
            s_bGstPipelineActive = TRUE;
            s_ullFrameCountTx = 0; /* Reset TX frame counter */
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
static int32_t P_DI_VIDEO_NVIDIA_StartRxMode(uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel)
{
    int32_t nRet = DI_ERROR;
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* Update current configuration tracking */
    s_unCurrentWidth = unWidth;
    s_unCurrentHeight = unHeight;
    s_unCurrentFrameRate = unFrameRate;
    s_unCurrentBitrate = unBitrate;
    s_unCurrentCodecType = unCodecType;
    s_unCurrentFormatType = unFormatType;
    
    if (s_bGstPipelineActive == FALSE)
    {
        /* Create RX pipeline */
        nRet = P_DI_VIDEO_NVIDIA_CreateRxPipeline(s_achRemoteHost, s_nRemotePort, s_unProtocol, s_unUdpPort, unWidth, unHeight, unFrameRate, unBitrate, unCodecType, unFormatType, unIFrameInterval, unPresetLevel);
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
            /* Create and start RTSP server */
            nRet = P_DI_VIDEO_NVIDIA_CreateRtspServer();
            if (nRet != DI_OK)
            {
                PrintError("Failed to create RTSP server [nRet:%d]", nRet);
                gst_element_set_state(s_hGstPipelineRx, GST_STATE_NULL);
                pthread_mutex_unlock(&s_stGstMutex);
                goto EXIT;
            }
            
            s_bGstPipelineActive = TRUE;
            s_ullFrameCountRx = 0; /* Reset RX frame counter */
            PrintTrace("RX pipeline and RTSP server started successfully - Connected to TCP server %s:%d", s_achRemoteHost, s_nRemotePort);
        }
        else
        {
            PrintWarn("RX pipeline may take longer to connect - State: %d", stState);
            s_bGstPipelineActive = TRUE; /* Set as active for async connection */
            s_ullFrameCountRx = 0; /* Reset RX frame counter */
            
            /* Still create RTSP server for async connection */
            nRet = P_DI_VIDEO_NVIDIA_CreateRtspServer();
            if (nRet != DI_OK)
            {
                PrintWarn("Failed to create RTSP server during async connection [nRet:%d]", nRet);
            }
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
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_BITRATE,
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_CODEC,
            0, /* Default format: YUYV */
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_IFRAME,
            DI_VIDEO_NVIDIA_GST_PIPELINE_DEFAULT_PRESET,
            s_unProtocol,
            s_achRemoteHost,
            s_unUdpPort
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
        
        /* Destroy RTSP server */
        P_DI_VIDEO_NVIDIA_DestroyRtspServer();
        
        s_hGstAppSrc = NULL;
        s_hGstAppSink = NULL;
        s_bGstPipelineActive = FALSE;
        
        PrintTrace("GStreamer pipelines and RTSP server stopped successfully");
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
 * Public function: Set UDP protocol configuration
 */
int32_t DI_VIDEO_NVIDIA_SetUdpProtocol(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, uint32_t unProtocol, const char *pchRemoteHost, uint32_t unUdpPort)
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
    
    nRet = P_DI_VIDEO_NVIDIA_SetUdpProtocol(unProtocol, pchRemoteHost, unUdpPort);
    if (nRet != DI_OK)
    {
        PrintError("P_DI_VIDEO_NVIDIA_SetUdpProtocol() failed [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintTrace("UDP protocol configuration set successfully");
    
EXIT:
    return nRet;
}

/*
 * Public function: Start TX mode
 */
int32_t DI_VIDEO_NVIDIA_StartTxMode(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel)
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
    
    nRet = P_DI_VIDEO_NVIDIA_StartTxMode(unWidth, unHeight, unFrameRate, unBitrate, unCodecType, unFormatType, unIFrameInterval, unPresetLevel);
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
int32_t DI_VIDEO_NVIDIA_StartRxMode(DI_VIDEO_NVIDIA_T *pstDiVideoNvidia, uint32_t unWidth, uint32_t unHeight, uint32_t unFrameRate, uint32_t unBitrate, uint32_t unCodecType, uint32_t unFormatType, uint32_t unIFrameInterval, uint32_t unPresetLevel)
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
    
    nRet = P_DI_VIDEO_NVIDIA_StartRxMode(unWidth, unHeight, unFrameRate, unBitrate, unCodecType, unFormatType, unIFrameInterval, unPresetLevel);
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

/*
 * Get GStreamer pipeline information
 */
int32_t DI_VIDEO_NVIDIA_GetPipelineInfo(DI_VIDEO_PIPELINE_INFO_T *pstTxInfo, DI_VIDEO_PIPELINE_INFO_T *pstRxInfo)
{
    int32_t nRet = DI_ERROR;
    
    if (pstTxInfo == NULL || pstRxInfo == NULL)
    {
        PrintError("Invalid parameters");
        goto EXIT;
    }
    
    /* Initialize structures */
    memset(pstTxInfo, 0, sizeof(DI_VIDEO_PIPELINE_INFO_T));
    memset(pstRxInfo, 0, sizeof(DI_VIDEO_PIPELINE_INFO_T));
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* Get TX pipeline information */
    if (s_hGstPipelineTx != NULL)
    {
        GstState stState, stPending;
        GstStateChangeReturn stStateRet = gst_element_get_state(s_hGstPipelineTx, &stState, &stPending, 0);
        
        pstTxInfo->unGstState = (uint32_t)stState;
        pstTxInfo->bIsActive = (stState == GST_STATE_PLAYING);
        
        /* Get current configuration from static variables */
        pstTxInfo->unWidth = s_unCurrentWidth;
        pstTxInfo->unHeight = s_unCurrentHeight;
        pstTxInfo->unFrameRate = s_unCurrentFrameRate;
        pstTxInfo->unBitrate = s_unCurrentBitrate;
        
        /* Set codec and format based on current configuration */
        switch (s_unCurrentCodecType)
        {
            case 0:
                strncpy(pstTxInfo->achCodec, "H.264", sizeof(pstTxInfo->achCodec) - 1);
                break;
            case 1:
                strncpy(pstTxInfo->achCodec, "H.265", sizeof(pstTxInfo->achCodec) - 1);
                break;
            case 2:
                strncpy(pstTxInfo->achCodec, "MJPEG", sizeof(pstTxInfo->achCodec) - 1);
                break;
            default:
                strncpy(pstTxInfo->achCodec, "Unknown", sizeof(pstTxInfo->achCodec) - 1);
                break;
        }
        
        switch (s_unCurrentFormatType)
        {
            case 0:
                strncpy(pstTxInfo->achFormat, "YUYV", sizeof(pstTxInfo->achFormat) - 1);
                break;
            case 1:
                strncpy(pstTxInfo->achFormat, "MJPEG", sizeof(pstTxInfo->achFormat) - 1);
                break;
            case 2:
                strncpy(pstTxInfo->achFormat, "NV12", sizeof(pstTxInfo->achFormat) - 1);
                break;
            default:
                strncpy(pstTxInfo->achFormat, "Unknown", sizeof(pstTxInfo->achFormat) - 1);
                break;
        }
        
        /* Update real-time statistics */
        P_DI_VIDEO_UpdateRealTimeStats();
        
        /* Return real-time rates from GStreamer probes */
        pthread_mutex_lock(&s_hStatsMutex);
        pstTxInfo->ullFramesProcessed = s_ullFrameCountTx;         /* Total frames (cumulative) */
        pstTxInfo->ullDroppedFrames = s_ullDroppedFramesTx;       /* Total dropped frames */
        pstTxInfo->ullUdpBytes = s_ullUdpBytesTx;                 /* Total bytes (cumulative) */
        pstTxInfo->dRealFrameRate = s_dCurrentFrameRateTx;        /* Current FPS */
        pstTxInfo->dCurrentByteRate = s_dCurrentByteRateTx;       /* Current bytes/sec */
        pthread_mutex_unlock(&s_hStatsMutex);
        
        pstTxInfo->unLatencyMs = pstTxInfo->bIsActive ? 50 : 0;
    }
    
    /* Get RX pipeline information */
    if (s_hGstPipelineRx != NULL)
    {
        GstState stState, stPending;
        GstStateChangeReturn stStateRet = gst_element_get_state(s_hGstPipelineRx, &stState, &stPending, 0);
        
        pstRxInfo->unGstState = (uint32_t)stState;
        pstRxInfo->bIsActive = (stState == GST_STATE_PLAYING);
        
        /* Copy same configuration as TX for RX */
        pstRxInfo->unWidth = s_unCurrentWidth;
        pstRxInfo->unHeight = s_unCurrentHeight;
        pstRxInfo->unFrameRate = s_unCurrentFrameRate;
        pstRxInfo->unBitrate = s_unCurrentBitrate;
        
        strncpy(pstRxInfo->achCodec, pstTxInfo->achCodec, sizeof(pstRxInfo->achCodec) - 1);
        strncpy(pstRxInfo->achFormat, pstTxInfo->achFormat, sizeof(pstRxInfo->achFormat) - 1);
        
        /* Update real-time statistics for RX as well */
        P_DI_VIDEO_UpdateRealTimeStats();
        
        /* Return real-time RX rates from GStreamer probes */
        pthread_mutex_lock(&s_hStatsMutex);
        pstRxInfo->ullFramesProcessed = s_ullFrameCountRx;         /* Total frames (cumulative) */
        pstRxInfo->ullDroppedFrames = s_ullDroppedFramesRx;       /* Total dropped frames */
        pstRxInfo->ullUdpBytes = s_ullUdpBytesRx;                 /* Total bytes (cumulative) */
        pstRxInfo->dRealFrameRate = s_dCurrentFrameRateRx;        /* Current FPS */
        pstRxInfo->dCurrentByteRate = s_dCurrentByteRateRx;       /* Current bytes/sec */
        pthread_mutex_unlock(&s_hStatsMutex);
        
        pstRxInfo->unLatencyMs = pstRxInfo->bIsActive ? 100 : 0;
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

/*
 * Get specific GStreamer element statistics
 */
int32_t DI_VIDEO_NVIDIA_GetElementStats(const char *pchElementName, uint64_t *pullFrameCount, uint32_t *punLatency)
{
    int32_t nRet = DI_ERROR;
    
    if (pchElementName == NULL || pullFrameCount == NULL || punLatency == NULL)
    {
        PrintError("Invalid parameters");
        goto EXIT;
    }
    
    pthread_mutex_lock(&s_stGstMutex);
    
    /* For now, return simulation data based on element name */
    if (strstr(pchElementName, "v4l2src") != NULL)
    {
        *pullFrameCount = s_ullFrameCountTx;
        *punLatency = 10;
    }
    else if (strstr(pchElementName, "tcpclientsink") != NULL || strstr(pchElementName, "udpsink") != NULL)
    {
        *pullFrameCount = s_ullFrameCountTx;
        *punLatency = 30;
    }
    else if (strstr(pchElementName, "tcpclientsrc") != NULL || strstr(pchElementName, "udpsrc") != NULL)
    {
        *pullFrameCount = s_ullFrameCountRx;
        *punLatency = 50;
    }
    else if (strstr(pchElementName, "appsink") != NULL)
    {
        *pullFrameCount = s_ullFrameCountRx;
        *punLatency = 20;
    }
    else
    {
        *pullFrameCount = 0;
        *punLatency = 0;
    }
    
    pthread_mutex_unlock(&s_stGstMutex);
    
    nRet = DI_OK;
    
EXIT:
    return nRet;
}

#endif /* CONFIG_VIDEO_STREAMING */



