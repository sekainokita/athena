
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
#include "cli.h"
#include "app.h"
#include "di.h"

/***************************** Definition ************************************/

/***************************** Static Variable *******************************/


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

        pstDi = APP_GetDiInstance();
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
               "di log off            hide di logs"
               "di gps heading        get heading of device\n"
               "di gps open           open a GPS device\n"
               "di gps close          close a GPS device\n"
               "di gps get            get a GPS data\n"
               "di gps tcp server     start tcp/ip server of di gps\n"
               "di gps set na         set a GPS device as not available\n",
               "");
    if(nRet != APP_OK)
    {
        PrintError("CLI_CMD_AddCmd() is failed! [nRet:%d]", nRet);
    }

    return nRet;
}

