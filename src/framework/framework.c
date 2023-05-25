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
* @file framework.c
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
#include "framework.h"

/***************************** Definition ************************************/


/***************************** Static Variable *******************************/
static MSG_MANAGER_T s_stMsgManager;
static DB_MANAGER_T s_stDbManager;
static TIME_MANAGER_T s_stTimeManager;

/***************************** Function  *************************************/

int32_t FRAMEWORK_Init(FRAMEWORK_T *pstFramework)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstFramework == NULL)
    {
        PrintError("pstFramework == NULL!!");
        return nRet;
    }

    (void*)memset(&s_stMsgManager, 0x00, sizeof(MSG_MANAGER_T));
    (void*)memset(&s_stDbManager, 0x00, sizeof(DB_MANAGER_T));
    (void*)memset(&s_stTimeManager, 0x00, sizeof(TIME_MANAGER_T));

    PrintWarn("is successfully initialized.");

    nRet = MSG_MANAGER_Init(&s_stMsgManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("MSG_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("MSG_MANAGER_Init() is successfully initialized, s_stMsgManager[0x%p]", &s_stMsgManager);

    nRet = DB_MANAGER_Init(&s_stDbManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("DB_MANAGER_Init() is successfully initialized, s_stDbManager[0x%p]", &s_stDbManager);

    nRet = TIME_MANAGER_Init(&s_stTimeManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("TIME_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("TIME_MANAGER_Init() is successfully initialized, s_stTimeManager[0x%p]", &s_stTimeManager);

    return nRet;
}

MSG_MANAGER_T* FRAMEWORK_GetMsgManagerInstance(void)
{
    return &s_stMsgManager;
}

DB_MANAGER_T* FRAMEWORK_GetDbManagerInstance(void)
{
    return &s_stDbManager;
}

DB_MANAGER_T* FRAMEWORK_GetTimeManagerInstance(void)
{
    return &s_stTimeManager;
}

