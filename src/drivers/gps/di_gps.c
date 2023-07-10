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
* @file di_gps.c
*
* This file contains a data format design
*
* @note
*
* V2X DI GPS Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  23.06.29 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include "di.h"
#include "di_gps.h"
#include "di_gps_xsens.h"

/***************************** Definition ************************************/

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiGpsLog = OFF;
static DI_GPS_XSENS_T s_stDiGpsDev;

/***************************** Function  *************************************/

static int32_t P_DI_GPS_Init(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

#if defined(CONFIG_GPS_XSENS)
    (void*)memset(&s_stDiGpsDev, 0x00, sizeof(DI_GPS_XSENS_T));

    nRet = DI_GPS_XSENS_Init(&s_stDiGpsDev);
    if(nRet != DI_OK)
    {
        PrintError("DI_GPS_XSENS_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
#endif

    return nRet;
}
static int32_t P_DI_GPS_DeInit(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

#if defined(CONFIG_GPS_XSENS)
    nRet = DI_GPS_XSENS_DeInit(&s_stDiGpsDev);
    if(nRet != DI_OK)
    {
        PrintError("DI_GPS_XSENS_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    (void*)memset(&s_stDiGpsDev, 0x00, sizeof(DI_GPS_XSENS_T));
#endif

    return nRet;
}

int32_t DI_GPS_SetLog(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    s_bDiGpsLog = pstDiGps->bLogLevel;
    PrintTrace("SET:s_bDiGpsLog [%s]", s_bDiGpsLog == ON ? "ON" : "OFF");

    nRet = DI_OK;

    return nRet;
}

int32_t DI_GPS_Get(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    if(pstDiGps->eDiGpsStatus >= DI_GPS_STATUS_OPENED)
    {
#if defined(CONFIG_GPS_XSENS)
        nRet = DI_GPS_XSENS_Get(&s_stDiGpsDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_GPS_XSENS_Get() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#endif
    }
    else
    {
        PrintWarn("check the status of GPS [%d]", pstDiGps->eDiGpsStatus);
    }

    return nRet;
}


int32_t DI_GPS_Open(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    if((pstDiGps->eDiGpsStatus == DI_GPS_STATUS_INITIALIZED) || (pstDiGps->eDiGpsStatus == DI_GPS_STATUS_CLOSED))
    {
#if defined(CONFIG_GPS_XSENS)
        nRet = DI_GPS_XSENS_Open(&s_stDiGpsDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_GPS_XSENS_Open() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#endif
        pstDiGps->eDiGpsStatus = DI_GPS_STATUS_OPENED;
    }
    else
    {
        PrintWarn("check the status of GPS [%d]", pstDiGps->eDiGpsStatus);

        if(pstDiGps->eDiGpsStatus == DI_GPS_STATUS_OPENED)
        {
            PrintDebug("already DI_GPS_STATUS_OPENED");
        }
    }

    return nRet;
}

int32_t DI_GPS_Close(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    if(pstDiGps->eDiGpsStatus == DI_GPS_STATUS_OPENED)
    {
#if defined(CONFIG_GPS_XSENS)
        nRet = DI_GPS_XSENS_Close(&s_stDiGpsDev);
        if(nRet != DI_OK)
        {
            PrintError("DI_GPS_XSENS_Close() is failed! [unRet:%d]", nRet);
            return nRet;
        }
#endif
        pstDiGps->eDiGpsStatus = DI_GPS_STATUS_CLOSED;

    }
    else
    {
        PrintWarn("check the status of GPS [%d]", pstDiGps->eDiGpsStatus);

        if(pstDiGps->eDiGpsStatus == DI_GPS_STATUS_CLOSED)
        {
            PrintDebug("already DI_GPS_STATUS_CLOSED");
        }
    }

    return nRet;
}

int32_t DI_GPS_Start(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t DI_GPS_Stop(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    return nRet;
}

void DI_GPS_Status(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    PrintWarn("TODO");

    UNUSED(nRet);

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
    }
}

int32_t DI_GPS_Init(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    nRet = P_DI_GPS_Init(pstDiGps);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_GPS_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiGps->eDiGpsStatus = DI_GPS_STATUS_INITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    s_bDiGpsLog = pstDiGps->bLogLevel;
    PrintDebug("s_bDiGpsLog [%s]", s_bDiGpsLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t DI_GPS_DeInit(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    nRet = P_DI_GPS_DeInit(pstDiGps);
    if(nRet != DI_OK)
    {
        PrintError("P_DI_GPS_DeInit() is failed! [unRet:%d]", nRet);
        return nRet;
    }
    else
    {
        pstDiGps->eDiGpsStatus = DI_GPS_STATUS_DEINITIALIZED;
        PrintWarn("is successfully initialized.");
    }

    return nRet;
}


