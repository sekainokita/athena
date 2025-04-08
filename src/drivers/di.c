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
* @file di.c
*
* This file contains a data format design
*
* @note
*
* V2X Data Format Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  23.03.22 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include "di.h"

/***************************** Definition ************************************/

/***************************** Static Variable *******************************/

/***************************** Function  *************************************/

void DI_SetLog(DI_T *pstDi, bool bOnOff)
{
    int32_t nRet = DI_ERROR;

    switch(pstDi->eLog)
    {
        case LOG_DI_ALL:
            if (bOnOff == ON)
            {
                pstDi->stDiGps.bLogLevel = ON;
            }
            else
            {
                pstDi->stDiGps.bLogLevel = OFF;
            }

            nRet = DI_GPS_SetLog(&pstDi->stDiGps);
            if (nRet != DI_OK)
            {
                PrintError("DI_GPS_SetLog() is failed! [nRet:%d]", nRet);
                return;
            }
            PrintDebug("LOG_DI_ALL [%d], bOnOff [%d]", nRet, bOnOff);
            break;

        case LOG_DI_GPS:
            if (bOnOff == ON)
            {
                pstDi->stDiGps.bLogLevel = ON;
            }
            else
            {
                pstDi->stDiGps.bLogLevel = OFF;
            }

            nRet = DI_GPS_SetLog(&pstDi->stDiGps);
            if (nRet != DI_OK)
            {
                PrintError("DI_GPS_SetLog() is failed! [nRet:%d]", nRet);
                return;
            }
            PrintDebug("LOG_DI_GPS [%d], bOnOff [%d]", nRet, bOnOff);
            break;

        default:
            PrintError("Unknown Log Type [%d]", pstDi->eLog);
            break;
    }
}


int32_t DI_Init(DI_T *pstDi)
{
    int32_t nRet = DI_ERROR;

    if(pstDi == NULL)
    {
        PrintError("pstDi == NULL!!");
        return nRet;
    }
    (void*)memset(&pstDi->stDiGps, 0x00, sizeof(DI_GPS_T));

    nRet = DI_GPS_Init(&pstDi->stDiGps);
    if(nRet != DI_OK)
    {
        PrintError("DI_GPS_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    nRet = DI_CAN_Init(&pstDi->stDiCan);
    if(nRet != DI_OK)
    {
        PrintError("DI_CAN_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    nRet = DI_VIDEO_Init(&pstDi->stDiVideo);
    if(nRet != DI_OK)
    {
        PrintError("DI_VIDEO_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }

#if defined(CONFIG_GPS_OBU) || defined(CONFIG_GPS_RSU)
    PrintWarn("Does not support GPS module on the b'd, use GPS values of OBUs, or RSUs");
    nRet = DI_GPS_SetNa(&pstDi->stDiGps, TRUE);
    if (nRet != DI_OK)
    {
        PrintError("DI_GPS_SetNa() is failed! [nRet:%d]", nRet);
        return nRet;
    }
#endif

    PrintWarn("is successfully initialized.");

    return nRet;
}

