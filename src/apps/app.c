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
* @file app.c
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
#include <stdio.h>
#include "app.h"
#include "cli.h"
/***************************** Definition ************************************/


/***************************** Static Variable *******************************/

static FRAMEWORK_T s_stFramework;
static APP_T s_stApp;
static DI_T s_stDi;
static SVC_PLATOONING_T s_stSvcPlatooning;
static SVC_CP_T s_stSvcCp;

/***************************** Function  *************************************/

int32_t APP_Init(APP_T *pstApp)
{
    int32_t nRet = APP_ERROR;

    if(pstApp == NULL)
    {
        PrintError("pstApp == NULL!!");
        return nRet;
    }

    PrintNotice("Init");

    nRet = SVC_PLATOONING_Init(&s_stSvcPlatooning);
    if (nRet != APP_OK)
    {
        PrintError("SVC_PLATOONING_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    nRet = SVC_CP_Init(&s_stSvcCp);
    if (nRet != APP_OK)
    {
        PrintError("SVC_CP_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    nRet = CLI_Init();
    if (nRet != APP_OK)
    {
        PrintError("CLI_Init() is failed! [unRet:%d]", nRet);
        return nRet;
    }

    return nRet;
}

FRAMEWORK_T* APP_GetFrameworkInstance(void)
{
    return &s_stFramework;
}

APP_T* APP_GetAppInstance(void)
{
    return &s_stApp;
}

DI_T* APP_GetDiInstance(void)
{
    return &s_stDi;
}

SVC_CP_T* APP_GetSvcCpInstance(void)
{
    return &s_stSvcCp;
}

SVC_PLATOONING_T* APP_GetSvcPlatooningInstance(void)
{
    return &s_stSvcPlatooning;
}

int main(int argc, char *argv[])
{
    int32_t nRet = APP_ERROR;

    UNUSED(argc);

    if(argv == NULL)
    {
        PrintError("argv == NULL!!");
        return nRet;
    }

    PrintDebug("Start the main");

    (void*)memset(&s_stFramework, 0x00, sizeof(FRAMEWORK_T));
    (void*)memset(&s_stApp, 0x00, sizeof(APP_T));
    (void*)memset(&s_stSvcPlatooning, 0x00, sizeof(SVC_PLATOONING_T));
    (void*)memset(&s_stSvcCp, 0x00, sizeof(SVC_CP_T));

    PrintDebug("pstFramework [0x%p]", &s_stFramework);
    PrintDebug("s_stApp [0x%p]", &s_stApp);

    nRet = DI_Init(&s_stDi);
    if (nRet != DI_OK)
    {
        PrintError("DI_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = FRAMEWORK_Init(&s_stFramework);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("FRAMEWORK_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    nRet = APP_Init(&s_stApp);
    if (nRet != APP_OK)
    {
        PrintError("APP_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    return nRet;
}

