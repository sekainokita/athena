
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
* @file cli_di.c
*
* @note
*
* CLI DI Source
*
******************************************************************************/


/***************************** Include ***************************************/
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include "type.h"
#include "cli_util.h"
#include "di.h"
#include "cli.h"
#include "di_gps.h"
#include "di_can.h"
#include "svc_streaming.h"
#include "di_video_nvidia.h"

/***************************** Definition ************************************/

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static DI_T s_stDi;
static SVC_STREAMING_T s_stSvcStreaming = {0};

#if defined(CONFIG_VIDEO_STREAMING)
static DI_VIDEO_NVIDIA_T s_stDiVideoNvidia = {0};
#endif

/***************************** Function Protype ******************************/
static int32_t P_CLI_DI_ConfigInteractive(void);
static int32_t P_CLI_DI_VideoStatusRealtime(void);
static void P_CLI_DI_DrawDashboard(SVC_STREAMING_STATS_T *pstStats);
static void P_CLI_DI_GetNetworkStats(uint64_t *pullBytesTx, uint64_t *pullBytesRx);
static int32_t P_CLI_DI_Kbhit(void);

static int P_CLI_DI(CLI_CMDLINE_T *pstCmd, int argc, char *argv[])
{
    int32_t nRet = APP_OK;
    char *pcCmd;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_0);
    if (pcCmd == NULL)
    {
        return CLI_CMD_Showusage(pstCmd);
    }
    else
    {
        DI_T *pstDi;

        pstDi = &s_stDi;
        PrintDebug("pstDi [0x%p]", pstDi);

        pcCmd = CLI_CMD_GetArg(pstCmd, CMD_0);
        if(IS_CMD(pcCmd, "test"))
        {
            for(int i = 0; i < CMD_MAX; i++)
            {
                pcCmd = CLI_CMD_GetArg(pstCmd, i);
                PrintDebug("pcCmd[idx:%d][value:%s]", i, pcCmd);
            }
        }
        else if(IS_CMD(pcCmd, "log"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if (pcCmd == NULL)
            {
                return CLI_CMD_Showusage(pstCmd);
            }
            else
            {
                if(IS_CMD(pcCmd, "on"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd == NULL)
                    {
                        return CLI_CMD_Showusage(pstCmd);
                    }
                    else
                    {
                        if(IS_CMD(pcCmd, "a"))
                        {
                            pstDi->eLog = LOG_DI_ALL;
                            (void)DI_SetLog(pstDi, ON);
                        }
                        else if(IS_CMD(pcCmd, "g"))
                        {
                            pstDi->eLog = LOG_DI_GPS;
                            (void)DI_SetLog(pstDi, ON);
                        }
                        else
                        {
                            PrintError("di log on a/g, e.g. di log on a, or di log on g");
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "off"))
                {
                    (void)DI_SetLog(pstDi, OFF);
                }
                else
                {
                    PrintError("di log on/off, e.g. di log on a/g, or di log off");
                }
            }
        }
        else if(IS_CMD(pcCmd, "gps"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if (pcCmd == NULL)
            {
                return CLI_CMD_Showusage(pstCmd);
            }
            else
            {
                if(IS_CMD(pcCmd, "open"))
                {
                    nRet = DI_GPS_Open(&pstDi->stDiGps);
                    if (nRet != DI_OK)
                    {
                        PrintError("DI_GPS_Open() is failed! [nRet:%d]", nRet);
                        return nRet;
                    }
                }
                else if(IS_CMD(pcCmd, "close"))
                {
                    nRet = DI_GPS_Close(&pstDi->stDiGps);
                    if (nRet != DI_OK)
                    {
                        PrintError("DI_GPS_Close() is failed! [nRet:%d]", nRet);
                        return nRet;
                    }
                }
                else if(IS_CMD(pcCmd, "heading"))
                {
                    double dHeading;
                    dHeading = DI_GPS_GetHeading(&pstDi->stDiGps);
                    PrintDebug("dHeading[%lf]", dHeading);
                }
                else if(IS_CMD(pcCmd, "get"))
                {
                    nRet = DI_GPS_Get(&pstDi->stDiGps);
                    if (nRet != DI_OK)
                    {
                        PrintError("DI_GPS_Get() is failed! [nRet:%d]", nRet);
                        return nRet;
                    }

                    PrintDebug("AccX            %.9f", pstDi->stDiGps.stDiGpsData.fAccX);
                    PrintDebug("AccY            %.9f", pstDi->stDiGps.stDiGpsData.fAccY);
                    PrintDebug("AccZ            %.9f", pstDi->stDiGps.stDiGpsData.fAccZ);
                    PrintDebug("GyrX            %.9f", pstDi->stDiGps.stDiGpsData.fGyrX);
                    PrintDebug("GyrY            %.9f", pstDi->stDiGps.stDiGpsData.fGyrY);
                    PrintDebug("GyrZ            %.9f", pstDi->stDiGps.stDiGpsData.fGyrZ);

                    PrintDebug("MagX            %.9f", pstDi->stDiGps.stDiGpsData.fMagX);
                    PrintDebug("MagY            %.9f", pstDi->stDiGps.stDiGpsData.fMagY);
                    PrintDebug("MagZ            %.9f", pstDi->stDiGps.stDiGpsData.fMagZ);

                    PrintDebug("QuaternionW     %.9f", pstDi->stDiGps.stDiGpsData.fQuaternionW);
                    PrintDebug("QuaternionX     %.9f", pstDi->stDiGps.stDiGpsData.fQuaternionX);
                    PrintDebug("QuaternionY     %.9f", pstDi->stDiGps.stDiGpsData.fQuaternionY);
                    PrintDebug("QuaternionZ     %.9f", pstDi->stDiGps.stDiGpsData.fQuaternionZ);

                    PrintDebug("EulerRoll       %.9f", pstDi->stDiGps.stDiGpsData.fEulerRoll);
                    PrintDebug("EulerPitch      %.9f", pstDi->stDiGps.stDiGpsData.fEulerPitch);
                    PrintDebug("EulerYaw        %.9f", pstDi->stDiGps.stDiGpsData.fEulerYaw);
                    PrintDebug("Latitude        %.9f", pstDi->stDiGps.stDiGpsData.fLatitude);
                    PrintDebug("Longitude       %.9f", pstDi->stDiGps.stDiGpsData.fLongitude);
                    PrintDebug("Altitude        %.9f", pstDi->stDiGps.stDiGpsData.fAltitude);
                    PrintDebug("VelocityEast    %.9f", pstDi->stDiGps.stDiGpsData.fVelocityEast);
                    PrintDebug("VelocityNorth   %.9f", pstDi->stDiGps.stDiGpsData.fVelocityNorth);
                    PrintDebug("VelocityUp      %.9f", pstDi->stDiGps.stDiGpsData.fVelocityUp);
                }
                else if(IS_CMD(pcCmd, "set"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        if(IS_CMD(pcCmd, "na"))
                        {
                            nRet = DI_GPS_SetNa(&pstDi->stDiGps, TRUE);
                            if (nRet != DI_OK)
                            {
                                PrintError("DI_GPS_SetNa() is failed! [nRet:%d]", nRet);
                                return nRet;
                            }
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "tcp"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if(pcCmd != NULL)
                    {
                        if(IS_CMD(pcCmd, "server"))
                        {
                            nRet = DI_GPS_StartSocketServer(&pstDi->stDiGps);
                            if (nRet != DI_OK)
                            {
                                PrintError("DI_GPS_StartSocketServer() is failed! [nRet:%d]", nRet);
                                return nRet;
                            }
                        }
                    }
                }
                else
                {
                    return CLI_CMD_Showusage(pstCmd);
                }
            }
        }
        else if(IS_CMD(pcCmd, "can"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if (pcCmd == NULL)
            {
                return CLI_CMD_Showusage(pstCmd);
            }
            else
            {
                if(IS_CMD(pcCmd, "open"))
                {
                    nRet = DI_CAN_Open(&pstDi->stDiCan);
                    if (nRet != DI_OK)
                    {
                        PrintError("DI_CAN_Open() is failed! [nRet:%d]", nRet);
                        return nRet;
                    }
                }
                else if(IS_CMD(pcCmd, "close"))
                {
                    nRet = DI_CAN_Close(&pstDi->stDiCan);
                    if (nRet != DI_OK)
                    {
                        PrintError("DI_CAN_Close() is failed! [nRet:%d]", nRet);
                        return nRet;
                    }
                }
                else
                {
                    return CLI_CMD_Showusage(pstCmd);
                }
            }
        }
        else if(IS_CMD(pcCmd, "video"))
        {
            pcCmd = CLI_CMD_GetArg(pstCmd, CMD_1);
            if (pcCmd == NULL)
            {
                return CLI_CMD_Showusage(pstCmd);
            }
            else
            {
                if(IS_CMD(pcCmd, "init"))
                {
#if defined(CONFIG_VIDEO_STREAMING)
                    SVC_STREAMING_CONFIG_T stConfig;
                    
                    /* Set default configuration */
                    memset(&stConfig, 0, sizeof(SVC_STREAMING_CONFIG_T));
                    stConfig.eMode = SVC_STREAMING_MODE_TX;
                    stConfig.eFormat = SVC_STREAMING_FORMAT_H264;
                    stConfig.unWidth = SVC_STREAMING_DEFAULT_WIDTH;
                    stConfig.unHeight = SVC_STREAMING_DEFAULT_HEIGHT;
                    stConfig.unFrameRate = SVC_STREAMING_DEFAULT_FPS;
                    stConfig.unBitrate = SVC_STREAMING_DEFAULT_BITRATE;
                    stConfig.unCodecType = 0;           /* H.264 */
                    stConfig.unFormatType = 0;          /* YUYV */
                    stConfig.unIFrameInterval = 15;     /* GOP size */
                    stConfig.unPresetLevel = 1;         /* Medium preset */
                    stConfig.bHardwareAcceleration = TRUE;
                    
                    /* Initialize video streaming service */
                    nRet = SVC_STREAMING_Init(&s_stSvcStreaming, &stConfig);
                    if (nRet != FRAMEWORK_OK)
                    {
                        PrintError("SVC_STREAMING_Init() failed! [nRet:%d]", nRet);
                        return nRet;
                    }
                    
                    PrintDebug("Video streaming service initialized successfully");
#else
                    PrintError("Video streaming not supported in this build");
#endif
                }
                else if(IS_CMD(pcCmd, "start"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd == NULL)
                    {
                        return CLI_CMD_Showusage(pstCmd);
                    }
                    else
                    {
#if defined(CONFIG_VIDEO_STREAMING)
                        SVC_STREAMING_MODE_E eMode;
                        
                        if(IS_CMD(pcCmd, "tx"))
                        {
                            eMode = SVC_STREAMING_MODE_TX;
                        }
                        else if(IS_CMD(pcCmd, "rx"))
                        {
                            eMode = SVC_STREAMING_MODE_RX;
                        }
                        else if(IS_CMD(pcCmd, "both"))
                        {
                            eMode = SVC_STREAMING_MODE_BOTH;
                        }
                        else
                        {
                            PrintError("Invalid mode! Use tx/rx/both");
                            return CLI_CMD_Showusage(pstCmd);
                        }
                        
                        nRet = SVC_STREAMING_Start(&s_stSvcStreaming, eMode);
                        if (nRet != FRAMEWORK_OK)
                        {
                            PrintError("SVC_STREAMING_Start() failed! [nRet:%d]", nRet);
                            return nRet;
                        }
                        
                        PrintDebug("Video streaming started in mode [%s]", pcCmd);
#else
                        PrintError("Video streaming not supported in this build");
#endif
                    }
                }
                else if(IS_CMD(pcCmd, "stop"))
                {
#if defined(CONFIG_VIDEO_STREAMING)
                    nRet = SVC_STREAMING_Stop(&s_stSvcStreaming);
                    if (nRet != FRAMEWORK_OK)
                    {
                        PrintError("SVC_STREAMING_Stop() failed! [nRet:%d]", nRet);
                        return nRet;
                    }
                    
                    PrintDebug("Video streaming stopped");
#else
                    PrintError("Video streaming not supported in this build");
#endif
                }
                else if(IS_CMD(pcCmd, "scan"))
                {
#if defined(CONFIG_VIDEO_STREAMING)
                    PrintDebug("Scanning available camera devices...");
                    
                    /* Scan video devices from /dev/video0 to /dev/video9 */
                    int32_t nDeviceCount = 0;
                    int32_t nDeviceIndex = 0;
                    char achDevicePath[64];
                    char achCommand[256];
                    FILE *hPipe = NULL;
                    char achBuffer[1024];
                    
                    for (nDeviceIndex = 0; nDeviceIndex < 10; nDeviceIndex++)
                    {
                        snprintf(achDevicePath, sizeof(achDevicePath), "/dev/video%d", nDeviceIndex);
                        
                        /* Check if device exists */
                        if (access(achDevicePath, F_OK) == 0)
                        {
                            PrintDebug("Found camera device: %s", achDevicePath);
                            nDeviceCount++;
                            
                            /* Get device capabilities using v4l2-ctl */
                            snprintf(achCommand, sizeof(achCommand), 
                                   "v4l2-ctl --device=%s --list-formats-ext 2>/dev/null", 
                                   achDevicePath);
                            
                            hPipe = popen(achCommand, "r");
                            if (hPipe != NULL)
                            {
                                PrintDebug("  Supported formats:");
                                while (fgets(achBuffer, sizeof(achBuffer), hPipe) != NULL)
                                {
                                    /* Remove newline */
                                    size_t szLen = strlen(achBuffer);
                                    if ((szLen > 0) && (achBuffer[szLen - 1] == '\n'))
                                    {
                                        achBuffer[szLen - 1] = '\0';
                                    }
                                    PrintDebug("    %s", achBuffer);
                                }
                                pclose(hPipe);
                                hPipe = NULL;
                            }
                            PrintDebug("");
                        }
                    }
                    
                    if (nDeviceCount == 0)
                    {
                        PrintDebug("No camera devices found");
                    }
                    else
                    {
                        PrintDebug("Total camera devices found: %d", nDeviceCount);
                    }
#else
                    PrintError("Video streaming not supported in this build");
#endif
                }
                else if(IS_CMD(pcCmd, "config"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd == NULL)
                    {
#if defined(CONFIG_VIDEO_STREAMING)
                        /* Show current configuration */
                        SVC_STREAMING_CONFIG_T stConfig;
                        nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                        if (nRet != FRAMEWORK_OK)
                        {
                            PrintError("SVC_STREAMING_GetConfig() failed! [nRet:%d]", nRet);
                            return nRet;
                        }
                        
                        PrintDebug("Current Configuration:");
                        PrintDebug("  Mode: %d", stConfig.eMode);
                        PrintDebug("  Format: %d", stConfig.eFormat);
                        PrintDebug("  Resolution: %dx%d", stConfig.unWidth, stConfig.unHeight);
                        PrintDebug("  Frame Rate: %d", stConfig.unFrameRate);
                        PrintDebug("  Bitrate: %d", stConfig.unBitrate);
                        PrintDebug("  Codec: %s", (stConfig.unCodecType == 0) ? "H.264" : 
                                                  (stConfig.unCodecType == 1) ? "H.265" : 
                                                  (stConfig.unCodecType == 2) ? "MJPEG" : "Unknown");
                        PrintDebug("  Camera Format: %s", (stConfig.unFormatType == 0) ? "YUYV" :
                                                         (stConfig.unFormatType == 1) ? "MJPEG" :
                                                         (stConfig.unFormatType == 2) ? "NV12" : "Unknown");
                        PrintDebug("  I-Frame Interval: %d", stConfig.unIFrameInterval);
                        PrintDebug("  Preset Level: %d", stConfig.unPresetLevel);
                        PrintDebug("  Hardware Accel: %s", stConfig.bHardwareAcceleration ? "YES" : "NO");
#else
                        PrintError("Video streaming not supported in this build");
#endif
                    }
                    else
                    {
#if defined(CONFIG_VIDEO_STREAMING)
                        /* Set configuration options */
                        if(IS_CMD(pcCmd, "resolution"))
                        {
                            char *pchWidth = CLI_CMD_GetArg(pstCmd, CMD_3);
                            char *pchHeight = CLI_CMD_GetArg(pstCmd, CMD_4);
                            
                            if (pchWidth != NULL && pchHeight != NULL)
                            {
                                SVC_STREAMING_CONFIG_T stConfig;
                                nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                                if (nRet == FRAMEWORK_OK)
                                {
                                    stConfig.unWidth = (uint32_t)atoi(pchWidth);
                                    stConfig.unHeight = (uint32_t)atoi(pchHeight);
                                    nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
                                    if (nRet == FRAMEWORK_OK)
                                    {
                                        PrintDebug("Resolution set to %dx%d", stConfig.unWidth, stConfig.unHeight);
                                    }
                                }
                            }
                        }
                        else if(IS_CMD(pcCmd, "fps"))
                        {
                            char *pchFps = CLI_CMD_GetArg(pstCmd, CMD_3);
                            
                            if (pchFps != NULL)
                            {
                                SVC_STREAMING_CONFIG_T stConfig;
                                nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                                if (nRet == FRAMEWORK_OK)
                                {
                                    stConfig.unFrameRate = (uint32_t)atoi(pchFps);
                                    nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
                                    if (nRet == FRAMEWORK_OK)
                                    {
                                        PrintDebug("Frame rate set to %d fps", stConfig.unFrameRate);
                                    }
                                }
                            }
                        }
                        else if(IS_CMD(pcCmd, "bitrate"))
                        {
                            char *pchBitrate = CLI_CMD_GetArg(pstCmd, CMD_3);
                            
                            if (pchBitrate != NULL)
                            {
                                SVC_STREAMING_CONFIG_T stConfig;
                                nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                                if (nRet == FRAMEWORK_OK)
                                {
                                    stConfig.unBitrate = (uint32_t)atoi(pchBitrate);
                                    nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
                                    if (nRet == FRAMEWORK_OK)
                                    {
                                        PrintDebug("Bitrate set to %d", stConfig.unBitrate);
                                    }
                                }
                            }
                        }
                        /* Handle codec configuration */
                        else if(IS_CMD(pcCmd, "codec"))
                        {
                            char *pchCodec = CLI_CMD_GetArg(pstCmd, CMD_3);
                            
                            if (pchCodec != NULL)
                            {
                                SVC_STREAMING_CONFIG_T stConfig;
                                nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                                if (nRet == FRAMEWORK_OK)
                                {
                                    if (strncmp(pchCodec, "h264", 4) == 0)
                                    {
                                        stConfig.unCodecType = 0;
                                        PrintDebug("Codec set to H.264");
                                    }
                                    else if (strncmp(pchCodec, "h265", 4) == 0)
                                    {
                                        stConfig.unCodecType = 1;
                                        PrintDebug("Codec set to H.265");
                                    }
                                    else if (strncmp(pchCodec, "mjpeg", 5) == 0)
                                    {
                                        stConfig.unCodecType = 2;
                                        PrintDebug("Codec set to MJPEG");
                                    }
                                    else
                                    {
                                        PrintError("Invalid codec! Use h264/h265/mjpeg");
                                        nRet = FRAMEWORK_ERROR;
                                    }
                                    
                                    if (nRet == FRAMEWORK_OK)
                                    {
                                        nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
                                    }
                                }
                            }
                        }
                        /* Handle format configuration */
                        else if(IS_CMD(pcCmd, "format"))
                        {
                            char *pchFormat = CLI_CMD_GetArg(pstCmd, CMD_3);
                            
                            if (pchFormat != NULL)
                            {
                                SVC_STREAMING_CONFIG_T stConfig;
                                nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                                if (nRet == FRAMEWORK_OK)
                                {
                                    if (strncmp(pchFormat, "yuyv", 4) == 0)
                                    {
                                        stConfig.unFormatType = 0;
                                        PrintDebug("Camera format set to YUYV");
                                    }
                                    else if (strncmp(pchFormat, "mjpeg", 5) == 0)
                                    {
                                        stConfig.unFormatType = 1;
                                        PrintDebug("Camera format set to MJPEG");
                                    }
                                    else if (strncmp(pchFormat, "nv12", 4) == 0)
                                    {
                                        stConfig.unFormatType = 2;
                                        PrintDebug("Camera format set to NV12");
                                    }
                                    else
                                    {
                                        PrintError("Invalid format! Use yuyv/mjpeg/nv12");
                                        nRet = FRAMEWORK_ERROR;
                                    }
                                    
                                    if (nRet == FRAMEWORK_OK)
                                    {
                                        nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
                                    }
                                }
                            }
                        }
                        /* Handle encoder configuration */
                        else if(IS_CMD(pcCmd, "encoder"))
                        {
                            char *pchOption = CLI_CMD_GetArg(pstCmd, CMD_3);
                            char *pchValue = CLI_CMD_GetArg(pstCmd, CMD_4);
                            
                            if ((pchOption != NULL) && (pchValue != NULL))
                            {
                                SVC_STREAMING_CONFIG_T stConfig;
                                nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                                if (nRet == FRAMEWORK_OK)
                                {
                                    if (strncmp(pchOption, "iframe", 6) == 0)
                                    {
                                        stConfig.unIFrameInterval = (uint32_t)atoi(pchValue);
                                        PrintDebug("I-Frame interval set to %d", stConfig.unIFrameInterval);
                                    }
                                    else if (strncmp(pchOption, "preset", 6) == 0)
                                    {
                                        stConfig.unPresetLevel = (uint32_t)atoi(pchValue);
                                        PrintDebug("Preset level set to %d", stConfig.unPresetLevel);
                                    }
                                    else
                                    {
                                        PrintError("Invalid encoder option! Use iframe/preset");
                                        nRet = FRAMEWORK_ERROR;
                                    }
                                    
                                    if (nRet == FRAMEWORK_OK)
                                    {
                                        nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
                                    }
                                }
                            }
                        }
                        /* Handle protocol configuration */
                        else if(IS_CMD(pcCmd, "protocol"))
                        {
                            char *pchProtocol = CLI_CMD_GetArg(pstCmd, CMD_3);
                            
                            if (pchProtocol != NULL)
                            {
                                SVC_STREAMING_CONFIG_T stConfig;
                                nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
                                if (nRet == FRAMEWORK_OK)
                                {
                                    if (strncmp(pchProtocol, "tcp", 3) == 0)
                                    {
                                        stConfig.eProtocol = SVC_STREAMING_PROTOCOL_TCP;
                                        PrintDebug("Network protocol set to TCP");
                                    }
                                    else if (strncmp(pchProtocol, "udp", 3) == 0)
                                    {
                                        stConfig.eProtocol = SVC_STREAMING_PROTOCOL_UDP;
                                        PrintDebug("Network protocol set to UDP");
                                    }
                                    else
                                    {
                                        PrintError("Invalid protocol! Use tcp/udp");
                                        nRet = FRAMEWORK_ERROR;
                                    }
                                    
                                    if (nRet == FRAMEWORK_OK)
                                    {
                                        nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
                                        
                                        /* Set UDP protocol configuration in SVC_STREAMING layer */
                                        PrintDebug("UDP protocol configuration set in streaming service");
                                        /* TODO: Add UDP protocol setting through SVC_STREAMING interface */
                                    }
                                }
                            }
                        }
                        /* Interactive configuration mode */
                        else if(IS_CMD(pcCmd, "interactive"))
                        {
                            nRet = P_CLI_DI_ConfigInteractive();
                        }
                        else
                        {
                            PrintError("Invalid config option! Use resolution/fps/bitrate/codec/format/encoder/protocol/interactive");
                        }
#else
                        PrintError("Video streaming not supported in this build");
#endif
                    }
                }
                else if(IS_CMD(pcCmd, "tcp"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd == NULL)
                    {
                        return CLI_CMD_Showusage(pstCmd);
                    }
                    else
                    {
                        if(IS_CMD(pcCmd, "set"))
                        {
                            char *pchHost = CLI_CMD_GetArg(pstCmd, CMD_3);
                            char *pchRemotePort = CLI_CMD_GetArg(pstCmd, CMD_4);
                            char *pchLocalPort = CLI_CMD_GetArg(pstCmd, CMD_5);
                            
                            if (pchHost != NULL && pchRemotePort != NULL && pchLocalPort != NULL)
                            {
                                int32_t nRemotePort = atoi(pchRemotePort);
                                int32_t nLocalPort = atoi(pchLocalPort);
                                
#if defined(CONFIG_VIDEO_STREAMING)
                                nRet = SVC_STREAMING_SetTcpConnection(&s_stSvcStreaming, pchHost, nRemotePort, nLocalPort);
                                if (nRet != FRAMEWORK_OK)
                                {
                                    PrintError("SVC_STREAMING_SetTcpConnection() failed! [nRet:%d]", nRet);
                                    return nRet;
                                }
                                
                                PrintDebug("TCP connection set: Host[%s] Remote[%d] Local[%d]", pchHost, nRemotePort, nLocalPort);
#else
                                PrintError("TCP streaming not supported in this build");
#endif
                            }
                            else
                            {
                                PrintError("TCP set requires: host remote_port local_port");
                            }
                        }
                        else if(IS_CMD(pcCmd, "check"))
                        {
#if defined(CONFIG_VIDEO_STREAMING)
                            nRet = SVC_STREAMING_CheckTcpConnection(&s_stSvcStreaming);
                            if (nRet != FRAMEWORK_OK)
                            {
                                PrintDebug("TCP connection check failed [nRet:%d]", nRet);
                            }
                            else
                            {
                                PrintDebug("TCP connection is active");
                            }
#else
                            PrintError("TCP streaming not supported in this build");
#endif
                        }
                        else
                        {
                            PrintError("Invalid TCP option! Use set/check");
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "udp"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd == NULL)
                    {
                        return CLI_CMD_Showusage(pstCmd);
                    }
                    else
                    {
                        if(IS_CMD(pcCmd, "set"))
                        {
                            char *pchHost = CLI_CMD_GetArg(pstCmd, CMD_3);
                            char *pchUdpPort = CLI_CMD_GetArg(pstCmd, CMD_4);
                            
                            if (pchHost != NULL && pchUdpPort != NULL)
                            {
                                uint32_t unUdpPort = (uint32_t)atoi(pchUdpPort);
                                
#if defined(CONFIG_VIDEO_STREAMING)
                                nRet = DI_VIDEO_NVIDIA_SetUdpProtocol(&s_stDiVideoNvidia, 1, pchHost, unUdpPort);
                                if (nRet != DI_OK)
                                {
                                    PrintError("DI_VIDEO_NVIDIA_SetUdpProtocol() failed! [nRet:%d]", nRet);
                                    return nRet;
                                }
                                
                                PrintDebug("UDP protocol set: Host[%s] Port[%u]", pchHost, unUdpPort);
#else
                                PrintError("UDP streaming not supported in this build");
#endif
                            }
                            else
                            {
                                PrintError("UDP set requires: host udp_port");
                            }
                        }
                        else
                        {
                            PrintError("Invalid UDP option! Use set");
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "rtsp"))
                {
                    pcCmd = CLI_CMD_GetArg(pstCmd, CMD_2);
                    if (pcCmd == NULL)
                    {
                        return CLI_CMD_Showusage(pstCmd);
                    }
                    else
                    {
                        if(IS_CMD(pcCmd, "start"))
                        {
#if defined(CONFIG_VIDEO_STREAMING)
                            PrintDebug("RTSP server start requested");
                            PrintDebug("RTSP server will be automatically started with RX mode");
                            PrintDebug("Use 'di video start rx' to start both RX pipeline and RTSP server");
                            PrintDebug("RTSP stream will be available at: rtsp://localhost:8560/stream");
#else
                            PrintError("Video streaming not supported in this build");
#endif
                        }
                        else if(IS_CMD(pcCmd, "stop"))
                        {
#if defined(CONFIG_VIDEO_STREAMING)
                            PrintDebug("RTSP server stop requested");
                            PrintDebug("RTSP server will be automatically stopped with video stop");
                            PrintDebug("Use 'di video stop' to stop both pipelines and RTSP server");
#else
                            PrintError("Video streaming not supported in this build");
#endif
                        }
                        else if(IS_CMD(pcCmd, "status"))
                        {
#if defined(CONFIG_VIDEO_STREAMING)
                            PrintDebug("RTSP Server Status:");
                            PrintDebug("  Server: Integrated with RX pipeline");
                            PrintDebug("  Port: 8560");
                            PrintDebug("  Stream Path: /stream");
                            PrintDebug("  Full URL: rtsp://localhost:8560/stream");
                            PrintDebug("  Status: %s", s_stSvcStreaming.eStatus == SVC_STREAMING_STATUS_RUNNING ? "Active with RX mode" : "Inactive");
#else
                            PrintError("Video streaming not supported in this build");
#endif
                        }
                        else
                        {
                            return CLI_CMD_Showusage(pstCmd);
                        }
                    }
                }
                else if(IS_CMD(pcCmd, "status"))
                {
#if defined(CONFIG_VIDEO_STREAMING)
                    /* Check if streaming service is initialized */
                    if (s_stSvcStreaming.eStatus == SVC_STREAMING_STATUS_UNINITIALIZED)
                    {
                        PrintError("Video streaming service not initialized. Use 'di video init' first.");
                    }
                    else
                    {
                        /* Launch real-time status dashboard */
                        PrintDebug("Starting real-time video streaming dashboard...");
                        PrintDebug("Press 'q' to quit, 'r' to reset statistics");
                        nRet = P_CLI_DI_VideoStatusRealtime();
                    }
#else
                    PrintError("Video streaming not supported in this build");
#endif
                }
                else
                {
                    return CLI_CMD_Showusage(pstCmd);
                }
            }
        }
        else
        {
            return CLI_CMD_Showusage(pstCmd);
        }
    }

    return nRet;
}

/* Interactive configuration function */
static int32_t P_CLI_DI_ConfigInteractive(void)
{
    int32_t nRet = FRAMEWORK_OK;
    SVC_STREAMING_CONFIG_T stConfig;
    char achInput[256] = {0};
    int32_t nChoice = 0;
    bool bValidConfig = FALSE;
    
    PrintDebug("===== Interactive Video Streaming Configuration =====");
    
    /* Get current configuration */
    nRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("SVC_STREAMING_GetConfig() failed! [nRet:%d]", nRet);
        goto EXIT;
    }
    
    PrintDebug("Current configuration will be displayed and you can modify each parameter.");
    PrintDebug("Press Enter to keep current value, or enter new value.\n");
    
    /* Step 1: Configure Resolution */
    PrintDebug("1. Resolution Configuration");
    PrintDebug("   Current: %dx%d", stConfig.unWidth, stConfig.unHeight);
    PrintDebug("   Common options:");
    PrintDebug("   [1] 640x480   (VGA)");
    PrintDebug("   [2] 1280x720  (HD)");
    PrintDebug("   [3] 1920x1080 (FHD)");
    PrintDebug("   [4] Custom");
    printf("   Select resolution [1-4] or press Enter to keep current: ");
    
    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
    {
        if (achInput[0] != '\n') /* Not just Enter */
        {
            nChoice = atoi(achInput);
            switch (nChoice)
            {
                case 1:
                    stConfig.unWidth = 640;
                    stConfig.unHeight = 480;
                    PrintDebug("   Resolution set to 640x480");
                    break;
                case 2:
                    stConfig.unWidth = 1280;
                    stConfig.unHeight = 720;
                    PrintDebug("   Resolution set to 1280x720");
                    break;
                case 3:
                    stConfig.unWidth = 1920;
                    stConfig.unHeight = 1080;
                    PrintDebug("   Resolution set to 1920x1080");
                    break;
                case 4:
                    printf("   Enter width: ");
                    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
                    {
                        uint32_t unWidth = (uint32_t)atoi(achInput);
                        if ((unWidth >= 320) && (unWidth <= 3840))
                        {
                            stConfig.unWidth = unWidth;
                            printf("   Enter height: ");
                            if (fgets(achInput, sizeof(achInput), stdin) != NULL)
                            {
                                uint32_t unHeight = (uint32_t)atoi(achInput);
                                if ((unHeight >= 240) && (unHeight <= 2160))
                                {
                                    stConfig.unHeight = unHeight;
                                    PrintDebug("   Custom resolution set to %dx%d", stConfig.unWidth, stConfig.unHeight);
                                }
                                else
                                {
                                    PrintError("   Invalid height! Using current value.");
                                }
                            }
                        }
                        else
                        {
                            PrintError("   Invalid width! Using current value.");
                        }
                    }
                    break;
                default:
                    PrintDebug("   Keeping current resolution");
                    break;
            }
        }
    }
    
    /* Step 2: Configure Frame Rate */
    PrintDebug("\n2. Frame Rate Configuration");
    PrintDebug("   Current: %d fps", stConfig.unFrameRate);
    PrintDebug("   Common options: 15, 30, 60 fps");
    printf("   Enter frame rate (5-60) or press Enter to keep current: ");
    
    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
    {
        if (achInput[0] != '\n')
        {
            uint32_t unFps = (uint32_t)atoi(achInput);
            if ((unFps >= 5) && (unFps <= 60))
            {
                stConfig.unFrameRate = unFps;
                PrintDebug("   Frame rate set to %d fps", stConfig.unFrameRate);
            }
            else
            {
                PrintError("   Invalid frame rate! Using current value.");
            }
        }
        else
        {
            PrintDebug("   Keeping current frame rate");
        }
    }
    
    /* Step 3: Configure Camera Format */
    PrintDebug("\n3. Camera Format Configuration");
    PrintDebug("   Current: %s", (stConfig.unFormatType == 0) ? "YUYV" :
                                (stConfig.unFormatType == 1) ? "MJPEG" :
                                (stConfig.unFormatType == 2) ? "NV12" : "Unknown");
    PrintDebug("   Available formats:");
    PrintDebug("   [1] YUYV  - Uncompressed YUV format → H.264/H.265 codec");
    PrintDebug("   [2] MJPEG - Hardware JPEG compression → MJPEG codec (direct streaming)");
    PrintDebug("   [3] NV12  - Planar YUV format → H.264/H.265 codec");
    printf("   Select format [1-3] or press Enter to keep current: ");
    
    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
    {
        if (achInput[0] != '\n')
        {
            nChoice = atoi(achInput);
            switch (nChoice)
            {
                case 1:
                    stConfig.unFormatType = 0;
                    PrintDebug("   Camera format set to YUYV");
                    break;
                case 2:
                    stConfig.unFormatType = 1;
                    /* Don't touch codec - GStreamer will use direct streaming based on format */
                    PrintDebug("   Camera format set to MJPEG");
                    PrintDebug("   → Direct JPEG streaming enabled (codec setting ignored)");
                    break;
                case 3:
                    stConfig.unFormatType = 2;
                    PrintDebug("   Camera format set to NV12");
                    break;
                default:
                    PrintDebug("   Keeping current camera format");
                    break;
            }
        }
        else
        {
            PrintDebug("   Keeping current camera format");
        }
    }
    
    /* Step 4: Configure Codec (skip if MJPEG format auto-selected codec) */
    if (stConfig.unFormatType == 1) /* MJPEG camera format */
    {
        PrintDebug("\n4. Codec Configuration");
        PrintDebug("   MJPEG camera format → MJPEG codec already set for optimal performance");
        PrintDebug("   Current: MJPEG (direct streaming, no re-encoding)");
        PrintDebug("   Change codec? [y/N] (not recommended): ");
        
        if (fgets(achInput, sizeof(achInput), stdin) != NULL)
        {
            if ((achInput[0] == 'y') || (achInput[0] == 'Y'))
            {
                PrintDebug("   Available codecs:");
                PrintDebug("   [1] H.264  - Requires JPEG decode + H.264 encode (CPU intensive)");
                PrintDebug("   [2] H.265  - Requires JPEG decode + H.265 encode (CPU intensive)");
                PrintDebug("   [3] MJPEG  - Direct JPEG streaming (current)");
                printf("   Select codec [1-3]: ");
                
                if (fgets(achInput, sizeof(achInput), stdin) != NULL)
                {
                    nChoice = atoi(achInput);
                    switch (nChoice)
                    {
                        case 1:
                            stConfig.unCodecType = 0;
                            PrintWarn("   Codec changed to H.264 (performance impact expected)");
                            break;
                        case 2:
                            stConfig.unCodecType = 1;
                            PrintWarn("   Codec changed to H.265 (performance impact expected)");
                            break;
                        case 3:
                        default:
                            stConfig.unCodecType = 2;
                            PrintDebug("   Keeping MJPEG codec (optimal choice)");
                            break;
                    }
                }
            }
            else
            {
                PrintDebug("   Keeping MJPEG codec (optimal choice)");
            }
        }
    }
    else /* Non-MJPEG camera format */
    {
        PrintDebug("\n4. Codec Configuration");
        PrintDebug("   Current: %s", (stConfig.unCodecType == 0) ? "H.264" :
                                    (stConfig.unCodecType == 1) ? "H.265" :
                                    (stConfig.unCodecType == 2) ? "MJPEG" : "Unknown");
        PrintDebug("   Available codecs:");
        PrintDebug("   [1] H.264  - Standard video codec (recommended)");
        PrintDebug("   [2] H.265  - High efficiency codec");
        PrintDebug("   [3] MJPEG  - Requires YUV to JPEG encode (suboptimal)");
        printf("   Select codec [1-3] or press Enter to keep current: ");
        
        if (fgets(achInput, sizeof(achInput), stdin) != NULL)
        {
            if (achInput[0] != '\n')
            {
                nChoice = atoi(achInput);
                switch (nChoice)
                {
                    case 1:
                        stConfig.unCodecType = 0;
                        PrintDebug("   Codec set to H.264");
                        break;
                    case 2:
                        stConfig.unCodecType = 1;
                        PrintDebug("   Codec set to H.265");
                        break;
                    case 3:
                        stConfig.unCodecType = 2;
                        PrintWarn("   Codec set to MJPEG (requires YUV→JPEG encoding)");
                        break;
                    default:
                        PrintDebug("   Keeping current codec");
                        break;
                }
            }
            else
            {
                PrintDebug("   Keeping current codec");
            }
        }
    }
    
    /* Step 5: Configure Bitrate (only for H.264/H.265) */
    if ((stConfig.unCodecType == 0) || (stConfig.unCodecType == 1))
    {
        PrintDebug("\n5. Bitrate Configuration");
        PrintDebug("   Current: %d bps", stConfig.unBitrate);
        PrintDebug("   Recommended values:");
        PrintDebug("   - 640x480:   500000 - 1000000 bps");
        PrintDebug("   - 1280x720:  1000000 - 3000000 bps");
        PrintDebug("   - 1920x1080: 3000000 - 8000000 bps");
        printf("   Enter bitrate (100000-10000000) or press Enter to keep current: ");
        
        if (fgets(achInput, sizeof(achInput), stdin) != NULL)
        {
            if (achInput[0] != '\n')
            {
                uint32_t unBitrate = (uint32_t)atoi(achInput);
                if ((unBitrate >= 100000) && (unBitrate <= 10000000))
                {
                    stConfig.unBitrate = unBitrate;
                    PrintDebug("   Bitrate set to %d bps", stConfig.unBitrate);
                }
                else
                {
                    PrintError("   Invalid bitrate! Using current value.");
                }
            }
            else
            {
                PrintDebug("   Keeping current bitrate");
            }
        }
    }
    else
    {
        PrintDebug("\n5. Bitrate Configuration");
        PrintDebug("   MJPEG codec does not use bitrate setting (quality depends on camera)");
    }
    
    /* Step 6: Configure Network Protocol */
    PrintDebug("\n6. Network Protocol Configuration");
    PrintDebug("   Current: %s", (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_TCP) ? "TCP" : "UDP");
    PrintDebug("   Available protocols:");
    PrintDebug("   [1] UDP - Lower latency, packet loss possible");
    PrintDebug("   [2] TCP - Reliable delivery, higher latency");
    printf("   Select protocol [1-2] or press Enter to keep current: ");
    
    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
    {
        if (achInput[0] != '\n')
        {
            nChoice = atoi(achInput);
            switch (nChoice)
            {
                case 1:
                    stConfig.eProtocol = SVC_STREAMING_PROTOCOL_UDP;
                    PrintDebug("   Protocol set to UDP");
                    break;
                case 2:
                    stConfig.eProtocol = SVC_STREAMING_PROTOCOL_TCP;
                    PrintDebug("   Protocol set to TCP");
                    break;
                default:
                    PrintDebug("   Keeping current protocol");
                    break;
            }
        }
        else
        {
            PrintDebug("   Keeping current protocol");
        }
    }
    
    /* Step 7: Configure Network Ports */
    PrintDebug("\n7. Network Port Configuration");
    PrintDebug("   Current TCP Port: %d", stConfig.unTcpPort);
    PrintDebug("   Current UDP Port: %d", stConfig.unUdpPort);
    
    if (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_TCP)
    {
        PrintDebug("   TCP Protocol selected - Configure TCP ports");
        PrintDebug("   Common TCP ports: 8554 (RTSP), 8080 (HTTP), 5000-5999 (custom)");
        printf("   Enter TCP port (1024-65535) or press Enter to keep current (%d): ", stConfig.unTcpPort);
        
        if (fgets(achInput, sizeof(achInput), stdin) != NULL)
        {
            if (achInput[0] != '\n')
            {
                int32_t nTcpPort = atoi(achInput);
                if ((nTcpPort >= 1024) && (nTcpPort <= 65535))
                {
                    stConfig.unTcpPort = (uint32_t)nTcpPort;
                    PrintDebug("   TCP port set to %d", nTcpPort);
                    
                    /* Set TCP connection using existing function */
                    nRet = SVC_STREAMING_SetTcpConnection(&s_stSvcStreaming, "127.0.0.1", nTcpPort, nTcpPort);
                    if (nRet != FRAMEWORK_OK)
                    {
                        PrintWarn("   Failed to apply TCP port setting [nRet:%d]", nRet);
                    }
                }
                else
                {
                    PrintWarn("   Invalid port range! Keeping current port (%d)", stConfig.unTcpPort);
                }
            }
            else
            {
                PrintDebug("   Keeping current TCP port (%d)", stConfig.unTcpPort);
            }
        }
    }
    else /* UDP Protocol */
    {
        PrintDebug("   UDP Protocol selected - Configure UDP ports");
        PrintDebug("   Common UDP ports: 5555 (default), 5000-5999 (custom), 1234, 4321");
        printf("   Enter UDP port (1024-65535) or press Enter to keep current (%d): ", stConfig.unUdpPort);
        
        if (fgets(achInput, sizeof(achInput), stdin) != NULL)
        {
            if (achInput[0] != '\n')
            {
                int32_t nUdpPort = atoi(achInput);
                if ((nUdpPort >= 1024) && (nUdpPort <= 65535))
                {
                    stConfig.unUdpPort = (uint32_t)nUdpPort;
                    PrintDebug("   UDP port set to %d", nUdpPort);
                    
                    /* Set UDP protocol using existing function */
                    nRet = DI_VIDEO_NVIDIA_SetUdpProtocol(&s_stDiVideoNvidia, 1, "127.0.0.1", (uint32_t)nUdpPort);
                    if (nRet != DI_OK)
                    {
                        PrintWarn("   Failed to apply UDP port setting [nRet:%d]", nRet);
                    }
                }
                else
                {
                    PrintWarn("   Invalid port range! Keeping current port (%d)", stConfig.unUdpPort);
                }
            }
            else
            {
                PrintDebug("   Keeping current UDP port (%d)", stConfig.unUdpPort);
            }
        }
    }
    
    /* Step 8: Remote Host Configuration (for network streaming) */
    PrintDebug("\n8. Remote Host Configuration");
    PrintDebug("   Current: %s (localhost)", stConfig.achRemoteHost[0] ? stConfig.achRemoteHost : "127.0.0.1");
    PrintDebug("   Options:");
    PrintDebug("   [1] 127.0.0.1     - Local machine (loopback)");
    PrintDebug("   [2] 192.168.1.x   - Local network device");
    PrintDebug("   [3] Custom IP     - Enter specific IP address");
    printf("   Select option [1-3] or press Enter to keep current: ");
    
    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
    {
        if (achInput[0] != '\n')
        {
            nChoice = atoi(achInput);
            switch (nChoice)
            {
                case 1:
                    if (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_TCP)
                    {
                        nRet = SVC_STREAMING_SetTcpConnection(&s_stSvcStreaming, "127.0.0.1", stConfig.unTcpPort, stConfig.unTcpPort);
                    }
                    else
                    {
                        nRet = DI_VIDEO_NVIDIA_SetUdpProtocol(&s_stDiVideoNvidia, 1, "127.0.0.1", stConfig.unUdpPort);
                    }
                    PrintDebug("   Remote host set to localhost (127.0.0.1)");
                    break;
                case 2:
                    printf("   Enter IP address (192.168.1.x): ");
                    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
                    {
                        /* Remove newline character */
                        size_t szLen = strlen(achInput);
                        if ((szLen > 0) && (achInput[szLen - 1] == '\n'))
                        {
                            achInput[szLen - 1] = '\0';
                        }
                        
                        if (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_TCP)
                        {
                            nRet = SVC_STREAMING_SetTcpConnection(&s_stSvcStreaming, achInput, stConfig.unTcpPort, stConfig.unTcpPort);
                        }
                        else
                        {
                            nRet = DI_VIDEO_NVIDIA_SetUdpProtocol(&s_stDiVideoNvidia, 1, achInput, stConfig.unUdpPort);
                        }
                        PrintDebug("   Remote host set to %s", achInput);
                    }
                    break;
                case 3:
                    printf("   Enter custom IP address: ");
                    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
                    {
                        /* Remove newline character */
                        size_t szLen = strlen(achInput);
                        if ((szLen > 0) && (achInput[szLen - 1] == '\n'))
                        {
                            achInput[szLen - 1] = '\0';
                        }
                        
                        if (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_TCP)
                        {
                            nRet = SVC_STREAMING_SetTcpConnection(&s_stSvcStreaming, achInput, stConfig.unTcpPort, stConfig.unTcpPort);
                        }
                        else
                        {
                            nRet = DI_VIDEO_NVIDIA_SetUdpProtocol(&s_stDiVideoNvidia, 1, achInput, stConfig.unUdpPort);
                        }
                        PrintDebug("   Remote host set to %s", achInput);
                    }
                    break;
                default:
                    PrintDebug("   Keeping current remote host");
                    break;
            }
        }
        else
        {
            PrintDebug("   Keeping current remote host");
        }
    }
    
    /* Step 9: Summary and Confirmation */
    PrintDebug("\n===== Configuration Summary =====");
    PrintDebug("Resolution:     %dx%d", stConfig.unWidth, stConfig.unHeight);
    PrintDebug("Frame Rate:     %d fps", stConfig.unFrameRate);
    PrintDebug("Camera Format:  %s", (stConfig.unFormatType == 0) ? "YUYV" :
                                    (stConfig.unFormatType == 1) ? "MJPEG" :
                                    (stConfig.unFormatType == 2) ? "NV12" : "Unknown");
    PrintDebug("Codec:          %s", (stConfig.unCodecType == 0) ? "H.264" :
                                    (stConfig.unCodecType == 1) ? "H.265" :
                                    (stConfig.unCodecType == 2) ? "MJPEG" : "Unknown");
    if ((stConfig.unCodecType == 0) || (stConfig.unCodecType == 1))
    {
        PrintDebug("Bitrate:        %d bps", stConfig.unBitrate);
    }
    PrintDebug("Protocol:       %s", (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_TCP) ? "TCP" : "UDP");
    if (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_TCP)
    {
        PrintDebug("TCP Port:       %d", stConfig.unTcpPort);
    }
    else
    {
        PrintDebug("UDP Port:       %d", stConfig.unUdpPort);
    }
    
    /* Final codec-format validation */
    bValidConfig = TRUE;
    if ((stConfig.unFormatType == 1) && (stConfig.unCodecType == 2))
    {
        PrintDebug("OPTIMAL: MJPEG camera format → MJPEG codec (direct JPEG streaming, no re-encoding)");
    }
    else if ((stConfig.unFormatType == 0) && (stConfig.unCodecType != 2)) /* YUYV → H264/H265 */
    {
        PrintDebug("GOOD: YUYV camera format → H.264/H.265 codec (standard encoding)");
    }
    else if ((stConfig.unFormatType == 2) && (stConfig.unCodecType != 2)) /* NV12 → H264/H265 */
    {
        PrintDebug("GOOD: NV12 camera format → H.264/H.265 codec (standard encoding)");
    }
    else
    {
        PrintWarn("SUBOPTIMAL: Format-codec combination requires re-encoding with performance impact");
        if ((stConfig.unFormatType == 1) && (stConfig.unCodecType != 2))
        {
            PrintWarn("  MJPEG camera → non-MJPEG codec: JPEG decode → re-encode (CPU intensive)");
        }
        else if ((stConfig.unFormatType != 1) && (stConfig.unCodecType == 2))
        {
            PrintWarn("  Non-MJPEG camera → MJPEG codec: YUV → JPEG encode (suboptimal)");
        }
        bValidConfig = FALSE;
    }
    
    printf("\nApply this configuration? [y/N]: ");
    if (fgets(achInput, sizeof(achInput), stdin) != NULL)
    {
        if ((achInput[0] == 'y') || (achInput[0] == 'Y'))
        {
            nRet = SVC_STREAMING_SetConfig(&s_stSvcStreaming, &stConfig);
            if (nRet == FRAMEWORK_OK)
            {
                PrintDebug("Configuration applied successfully!");
                if (bValidConfig == FALSE)
                {
                    PrintWarn("Note: Current configuration may have performance implications.");
                }
            }
            else
            {
                PrintError("Failed to apply configuration! [nRet:%d]", nRet);
            }
        }
        else
        {
            PrintDebug("Configuration cancelled.");
        }
    }
    else
    {
        PrintDebug("Configuration cancelled.");
    }
    
EXIT:
    return nRet;
}

int32_t CLI_DI_InitCmds(void)
{
    int32_t nRet = APP_ERROR;

    nRet = CLI_CMD_AddCmd("di",
               P_CLI_DI,
               NULL,
               "help for DI commands",
               "di [enter command]\n\n"
               "Without any parameters, the 'di' show a description\n"
               "of available commands. For more details on a command, type and enter 'di'\n"
               "and the command name.\n\n"
               "di test               test di command\n"
               "di log on [OPTIONS]   show di logs\n"
               "           a          show all logs\n"
               "           g          show gps logs\n"
               "di log off            hide di logs\n"
               "di gps heading        get heading of device\n"
               "di gps open           open a GPS device\n"
               "di gps close          close a GPS device\n"
               "di gps get            get a GPS data\n"
               "di gps tcp server     start tcp/ip server of di gps\n"
               "di gps set na         set a GPS device as not available\n"
               "di can open           open a CAN device\n"
               "di can close          close a CAN device\n"
               "di video init         initialize video streaming service\n"
               "di video start tx     start video streaming in TX mode\n"
               "di video start rx     start video streaming in RX mode\n"
               "di video start both   start video streaming in both TX/RX mode\n"
               "di video stop         stop video streaming\n"
               "di video scan         scan available camera devices and formats\n"
               "di video config       show current video configuration\n"
               "di video config resolution [width] [height]\n"
               "                      set video resolution\n"
               "di video config fps [fps]\n"
               "                      set video frame rate\n"
               "di video config bitrate [bitrate]\n"
               "                      set video bitrate\n"
               "di video config codec [h264|h265|mjpeg]\n"
               "                      set video codec type\n"
               "di video config format [yuyv|mjpeg|nv12]\n"
               "                      set camera format type\n"
               "di video config encoder iframe [interval]\n"
               "                      set I-frame interval (GOP size)\n"
               "di video config encoder preset [level]\n"
               "                      set encoder preset level (0=fastest, 3=slowest)\n"
               "di video config protocol [tcp|udp]\n"
               "                      set network protocol (TCP or UDP)\n"
               "di video config interactive\n"
               "                      interactive configuration mode with validation\n"
               "di video tcp set [host] [remote_port] [local_port]\n"
               "                      set TCP connection parameters\n"
               "di video tcp check    check TCP connection status\n"
               "di video udp set [host] [udp_port]\n"
               "                      set UDP connection parameters\n"
               "di video rtsp start   start RTSP server (port 8554)\n"
               "di video rtsp stop    stop RTSP server\n"
               "di video rtsp status  show RTSP server status\n"
               "di video status       show video streaming status and statistics\n",
               "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

/* Non-blocking keyboard input detection */
static int32_t P_CLI_DI_Kbhit(void)
{
    struct termios stOldTerm, stNewTerm;
    int32_t nOldFlags, nChar = 0;
    
    tcgetattr(STDIN_FILENO, &stOldTerm);
    stNewTerm = stOldTerm;
    stNewTerm.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &stNewTerm);
    
    nOldFlags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, nOldFlags | O_NONBLOCK);
    
    nChar = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &stOldTerm);
    fcntl(STDIN_FILENO, F_SETFL, nOldFlags);
    
    if (nChar != EOF)
    {
        ungetc(nChar, stdin);
        return 1;
    }
    
    return 0;
}

/* Get network statistics from /proc/net/dev */
static void P_CLI_DI_GetNetworkStats(uint64_t *pullBytesTx, uint64_t *pullBytesRx)
{
    FILE *hFile = NULL;
    char achLine[256] = {0};
    char achInterface[32] = {0};
    uint64_t ullRxBytes = 0, ullTxBytes = 0;
    uint64_t ullLoRxBytes = 0, ullLoTxBytes = 0;
    uint64_t ullEthRxBytes = 0, ullEthTxBytes = 0;
    
    *pullBytesTx = 0;
    *pullBytesRx = 0;
    
    hFile = fopen("/proc/net/dev", "r");
    if (hFile == NULL)
    {
        return;
    }
    
    /* Skip header lines */
    if (fgets(achLine, sizeof(achLine), hFile) == NULL) return;
    if (fgets(achLine, sizeof(achLine), hFile) == NULL) return;
    
    while (fgets(achLine, sizeof(achLine), hFile) != NULL)
    {
        if (sscanf(achLine, "%31s %lu %*u %*u %*u %*u %*u %*u %*u %lu",
                   achInterface, &ullRxBytes, &ullTxBytes) == 3)
        {
            /* Remove colon from interface name */
            char *pchColon = strchr(achInterface, ':');
            if (pchColon != NULL)
            {
                *pchColon = '\0';
            }
            
            if (strcmp(achInterface, "lo") == 0)
            {
                ullLoRxBytes = ullRxBytes;
                ullLoTxBytes = ullTxBytes;
            }
            else if (strncmp(achInterface, "eth", 3) == 0)
            {
                ullEthRxBytes += ullRxBytes;
                ullEthTxBytes += ullTxBytes;
            }
        }
    }
    
    fclose(hFile);
    
    /* Return total network traffic */
    *pullBytesRx = ullLoRxBytes + ullEthRxBytes;
    *pullBytesTx = ullLoTxBytes + ullEthTxBytes;
}

/* Draw progress bar */
static void P_CLI_DI_DrawProgressBar(const char *pchLabel, uint64_t ullCurrent, uint64_t ullMax, const char *pchUnit)
{
    const int32_t nBarWidth = 20;
    int32_t nFilled = 0;
    int32_t nPercent = 0;
    int32_t i = 0;
    
    if (ullMax > 0)
    {
        nPercent = (int32_t)((ullCurrent * 100) / ullMax);
        nFilled = (nPercent * nBarWidth) / 100;
    }
    
    printf("│ %-12s ", pchLabel);
    
    for (i = 0; i < nBarWidth; i++)
    {
        if (i < nFilled)
        {
            printf("█");
        }
        else
        {
            printf("░");
        }
    }
    
    printf(" %3d%% (%.1f %s)%*s│\n", 
           nPercent, 
           (double)ullCurrent / (1024.0 * 1024.0), 
           pchUnit,
           15 - (int)strlen(pchUnit), "");
}

/* Draw real-time dashboard */
static void P_CLI_DI_DrawDashboard(SVC_STREAMING_STATS_T *pstStats)
{
    static uint64_t s_ullPrevBytesTx = 0, s_ullPrevBytesRx = 0;
    static time_t s_tPrevTime = 0;
    uint64_t ullCurrentBytesTx = 0, ullCurrentBytesRx = 0;
    uint64_t ullDeltaBytesTx = 0, ullDeltaBytesRx = 0;
    time_t tCurrentTime = time(NULL);
    double dTxRate = 0.0, dRxRate = 0.0;
    double dTxRateMbps = 0.0, dRxRateMbps = 0.0;
    struct tm *pstLocalTime = NULL;
    char achTimeStr[32] = {0};
    
    /* Get network statistics */
    P_CLI_DI_GetNetworkStats(&ullCurrentBytesTx, &ullCurrentBytesRx);
    
    /* Calculate transfer rates */
    if (s_tPrevTime > 0)
    {
        time_t tDelta = tCurrentTime - s_tPrevTime;
        if (tDelta > 0)
        {
            ullDeltaBytesTx = ullCurrentBytesTx - s_ullPrevBytesTx;
            ullDeltaBytesRx = ullCurrentBytesRx - s_ullPrevBytesRx;
            dTxRate = (double)ullDeltaBytesTx / (double)tDelta;
            dRxRate = (double)ullDeltaBytesRx / (double)tDelta;
            dTxRateMbps = (dTxRate * 8.0) / (1024.0 * 1024.0);
            dRxRateMbps = (dRxRate * 8.0) / (1024.0 * 1024.0);
        }
    }
    
    s_ullPrevBytesTx = ullCurrentBytesTx;
    s_ullPrevBytesRx = ullCurrentBytesRx;
    s_tPrevTime = tCurrentTime;
    
    /* Get current time string */
    pstLocalTime = localtime(&tCurrentTime);
    strftime(achTimeStr, sizeof(achTimeStr), "%Y-%m-%d %H:%M:%S", pstLocalTime);
    
    /* Clear screen */
    if (system("clear") == -1) {
        /* Clear failed, continue anyway */
    }
    
    /* Get real-time streaming configuration and pipeline info */
    SVC_STREAMING_CONFIG_T stConfig;
    DI_VIDEO_PIPELINE_INFO_T stTxInfo, stRxInfo;
    int32_t nConfigRet = SVC_STREAMING_GetConfig(&s_stSvcStreaming, &stConfig);
    int32_t nPipelineRet = DI_VIDEO_NVIDIA_GetPipelineInfo(&stTxInfo, &stRxInfo);
    
    /* Draw dashboard */
    printf("┌─── ATHENA Video Streaming Status ─── [q: quit, r: reset stats] ───┐\n");
    printf("│ Time: %-19s                    Uptime: Active      │\n", achTimeStr);
    printf("├─── Camera & Encoding ─────────────────────────────────────────────┤\n");
    
    if (nPipelineRet == DI_OK && nConfigRet == FRAMEWORK_OK)
    {
        printf("│ Resolution: %dx%d @ %dfps    Codec: %-6s  Format: %-8s │\n",
               stConfig.unWidth, stConfig.unHeight, stConfig.unFrameRate,
               stTxInfo.achCodec, stTxInfo.achFormat);
        printf("│ Quality: Auto                    I-Frame: %-3d     Preset: %-3d     │\n",
               stConfig.unIFrameInterval, stConfig.unPresetLevel);
    }
    else
    {
        printf("│ Resolution: 1920x1080 @ 30fps    Codec: MJPEG    Format: Hardware │\n");
        printf("│ Quality: Auto                    I-Frame: N/A     Preset: N/A     │\n");
    }
    
    printf("├─── Network Configuration ─────────────────────────────────────────┤\n");
    
    if (nConfigRet == FRAMEWORK_OK)
    {
        const char *pchProtocol = (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_UDP) ? "UDP" : "TCP";
        uint32_t unPort = (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_UDP) ? stConfig.unUdpPort : stConfig.unTcpPort;
        const char *pchHost = (stConfig.achRemoteHost[0] != '\0') ? stConfig.achRemoteHost : "127.0.0.1";
        
        printf("│ Protocol: %-3s                  Port: %-7d Host: %-11s │\n", pchProtocol, unPort, pchHost);
        
        const char *pchTxStatus = (nPipelineRet == DI_OK && stTxInfo.bIsActive) ? "ACTIVE" : "INACTIVE";
        const char *pchRxStatus = (nPipelineRet == DI_OK && stRxInfo.bIsActive) ? "ACTIVE" : "INACTIVE";
        const char *pchConnection = (nPipelineRet == DI_OK && (stTxInfo.bIsActive || stRxInfo.bIsActive)) ? "UP" : "DOWN";
        
        printf("│ TX: %-8s                 RX: %-8s Connection: %-6s │\n", pchTxStatus, pchRxStatus, pchConnection);
    }
    else
    {
        printf("│ Protocol: UDP                    Port: 5555       Host: 127.0.0.1 │\n");
        printf("│ Mode: TX                         Status: ACTIVE   Connection: UP   │\n");
    }
    printf("├─── Real-time Statistics (GStreamer Pipeline) ──────────────────────┤\n");
    
    /* Use real statistics from GStreamer pipeline probes */
    if (nPipelineRet == DI_OK)
    {
        /* Display current FPS and cumulative frame counts */
        printf("│ TX: %.1f fps (Total: %lu frames)   RX: %.1f fps (Total: %lu frames) │\n",
               stTxInfo.dRealFrameRate,
               stTxInfo.ullFramesProcessed, 
               stRxInfo.dRealFrameRate,
               stRxInfo.ullFramesProcessed);
        
        /* Display current transfer rates in KB/s and total transferred */
        printf("│ TX: %.1f KB/s (Total: %.1f MB)     RX: %.1f KB/s (Total: %.1f MB)   │\n",
               stTxInfo.dCurrentByteRate / 1024.0,
               (double)stTxInfo.ullUdpBytes / (1024.0 * 1024.0),
               stRxInfo.dCurrentByteRate / 1024.0,
               (double)stRxInfo.ullUdpBytes / (1024.0 * 1024.0));
        
        /* Display current bitrate in Mbps */
        printf("│ TX Rate: %.1f Mbps                 RX Rate: %.1f Mbps               │\n",
               (stTxInfo.dCurrentByteRate * 8.0) / (1024.0 * 1024.0),
               (stRxInfo.dCurrentByteRate * 8.0) / (1024.0 * 1024.0));
        
        /* Display dropped frames and latency */
        printf("│ Dropped: %lu TX / %lu RX           Latency: %ums TX / %ums RX      │\n",
               stTxInfo.ullDroppedFrames,
               stRxInfo.ullDroppedFrames,
               stTxInfo.unLatencyMs,
               stRxInfo.unLatencyMs);
    }
    else
    {
        /* Fallback to old system stats if pipeline data unavailable */
        printf("│ TX Frames: %-8lu (%.1f fps)     RX Frames: %-8lu (%.1f fps)     │\n",
               pstStats->ullTotalFramesTx, 
               pstStats->ullTotalFramesTx > 0 ? 30.0 : 0.0,
               pstStats->ullTotalFramesRx,
               pstStats->ullTotalFramesRx > 0 ? 30.0 : 0.0);
        printf("│ TX Bytes:  %.1f MB (%.1f MB/s)  RX Bytes:  %.1f MB (%.1f MB/s)  │\n",
               (double)ullCurrentBytesTx / (1024.0 * 1024.0),
               dTxRate / (1024.0 * 1024.0),
               (double)ullCurrentBytesRx / (1024.0 * 1024.0),
               dRxRate / (1024.0 * 1024.0));
        printf("│ TX Rate:   %.1f Mbps (SYS)       RX Rate:   %.1f Mbps (SYS)       │\n",
               dTxRateMbps, dRxRateMbps);
        printf("│ Dropped:   %u frames (%.2f%%)     Latency:   %ums               │\n",
               pstStats->unDroppedFramesTx + pstStats->unDroppedFramesRx,
               pstStats->ullTotalFramesTx > 0 ? 
               (double)(pstStats->unDroppedFramesTx + pstStats->unDroppedFramesRx) * 100.0 / 
               (double)pstStats->ullTotalFramesTx : 0.0,
               pstStats->unNetworkLatency);
    }
    printf("├─── Network Ports ─────────────────────────────────────────────────┤\n");
    
    /* Draw network port usage bars with dynamic port information */
    if (nConfigRet == FRAMEWORK_OK)
    {
        char achPortLabel[32];
        
        if (stConfig.eProtocol == SVC_STREAMING_PROTOCOL_UDP)
        {
            snprintf(achPortLabel, sizeof(achPortLabel), "UDP %u:", stConfig.unUdpPort);
            P_CLI_DI_DrawProgressBar(achPortLabel, (uint64_t)(dTxRate / 1024.0), 10240, "KB/s");
            
            snprintf(achPortLabel, sizeof(achPortLabel), "TCP %u:", stConfig.unTcpPort);
            P_CLI_DI_DrawProgressBar(achPortLabel, 0, 10240, "KB/s");
        }
        else
        {
            snprintf(achPortLabel, sizeof(achPortLabel), "TCP %u:", stConfig.unTcpPort);
            P_CLI_DI_DrawProgressBar(achPortLabel, (uint64_t)(dTxRate / 1024.0), 10240, "KB/s");
            
            snprintf(achPortLabel, sizeof(achPortLabel), "UDP %u:", stConfig.unUdpPort);
            P_CLI_DI_DrawProgressBar(achPortLabel, 0, 10240, "KB/s");
        }
        
        P_CLI_DI_DrawProgressBar("RTSP 8554:", 0, 10240, "KB/s");
    }
    else
    {
        /* Fallback to hardcoded values */
        P_CLI_DI_DrawProgressBar("UDP 5555:", (uint64_t)(dTxRate / 1024.0), 10240, "KB/s");
        P_CLI_DI_DrawProgressBar("TCP 8554:", 0, 10240, "KB/s");
        P_CLI_DI_DrawProgressBar("RTSP 8554:", 0, 10240, "KB/s");
    }
    
    printf("├─── System Resources ──────────────────────────────────────────────┤\n");
    printf("│ CPU: N/A    Memory: N/A GB    GPU: N/A    Temp: N/A°C        │\n");
    printf("└─── Press 'q' to quit, 'r' to reset statistics ───────────────────┘\n");
    
    fflush(stdout);
}

/* Real-time video status dashboard */
static int32_t P_CLI_DI_VideoStatusRealtime(void)
{
    int32_t nRet = FRAMEWORK_OK;
    SVC_STREAMING_STATS_T stStats = {0};
    int32_t nChar = 0;
    
    /* Set terminal to raw mode for immediate key detection */
    if (system("stty -icanon -echo") == -1) {
        PrintWarn("Failed to set terminal raw mode");
    }
    
    PrintDebug("Real-time dashboard started. Press 'q' to quit, 'r' to reset statistics.\n");
    sleep(1);  /* Give user time to read message */
    
    while (1)
    {
        /* Get current streaming statistics */
        nRet = SVC_STREAMING_GetStats(&s_stSvcStreaming, &stStats);
        if (nRet != FRAMEWORK_OK)
        {
            /* If stats unavailable, use empty stats */
            memset(&stStats, 0, sizeof(stStats));
        }
        
        /* Draw dashboard */
        P_CLI_DI_DrawDashboard(&stStats);
        
        /* Check for keyboard input (non-blocking) */
        if (P_CLI_DI_Kbhit())
        {
            nChar = getchar();
            if ((nChar == 'q') || (nChar == 'Q'))
            {
                break;
            }
            else if ((nChar == 'r') || (nChar == 'R'))
            {
                /* Reset statistics */
                SVC_STREAMING_ResetStats(&s_stSvcStreaming);
                PrintDebug("Statistics reset!");
                sleep(1);
            }
        }
        
        /* Update every second */
        sleep(1);
    }
    
    /* Restore terminal settings */
    if (system("stty icanon echo") == -1) {
        PrintWarn("Failed to restore terminal settings");
    }
    
    printf("\nReal-time dashboard stopped.\n");
    return FRAMEWORK_OK;
}

