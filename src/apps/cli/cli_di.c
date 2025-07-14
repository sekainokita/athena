
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
#include "type.h"
#include "cli_util.h"
#include "di.h"
#include "cli.h"
#include "di_gps.h"
#include "di_can.h"
#include "svc_streaming.h"

/***************************** Definition ************************************/

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static DI_T s_stDi;
static SVC_STREAMING_T s_stSvcStreaming = {0};

/***************************** Function Protype ******************************/

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
                        else
                        {
                            PrintError("Invalid config option! Use resolution/fps/bitrate");
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
                        /* Use the existing status function */
                        SVC_STREAMING_PrintStatus(&s_stSvcStreaming);
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
               "di video config       show current video configuration\n"
               "di video config resolution [width] [height]\n"
               "                      set video resolution\n"
               "di video config fps [fps]\n"
               "                      set video frame rate\n"
               "di video config bitrate [bitrate]\n"
               "                      set video bitrate\n"
               "di video tcp set [host] [remote_port] [local_port]\n"
               "                      set TCP connection parameters\n"
               "di video tcp check    check TCP connection status\n"
               "di video status       show video streaming status and statistics\n",
               "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

