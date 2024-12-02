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
#if defined(CONFIG_MULTI_DEV)
static MULTI_MSG_MANAGER_T s_stMultiMsgManager;
static MULTI_DB_MANAGER_T s_stMultiDbManager;
#else
static MSG_MANAGER_T s_stMsgManager;
static DB_MANAGER_T s_stDbManager;
#endif
static TIME_MANAGER_T s_stTimeManager;

/***************************** Function  *************************************/

void FRAMEWORK_SetLog(FRAMEWORK_T *pstFramework, bool bOnOff)
{
    int32_t nRet = FRAMEWORK_ERROR;

#if defined(CONFIG_MULTI_DEV)
    switch(pstFramework->eFrameworkLog)
        {
            case FRAMEWORK_LOG_ALL:
                s_stTimeManager.bLogLevel = bOnOff;
                nRet = TIME_MANAGER_SetLog(&s_stTimeManager);
                if (nRet != FRAMEWORK_OK)
                {
                    PrintError("TIME_MANAGER_SetLog() is failed! [nRet:%d]", nRet);
                }

                s_stMultiMsgManager.bLogLevel = bOnOff;
                nRet = MULTI_MSG_MANAGER_SetLog(&s_stMultiMsgManager);
                if (nRet != FRAMEWORK_OK)
                {
                    PrintError("MULTI_MSG_MANAGER_SetLog() is failed! [nRet:%d]", nRet);
                }

                s_stMultiDbManager.bLogLevel = bOnOff;
                nRet = MULTI_DB_MANAGER_SetLog(&s_stMultiDbManager);
                if (nRet != FRAMEWORK_OK)
                {
                    PrintError("MULTI_DB_MANAGER_SetLog() is failed! [nRet:%d]", nRet);
                }

#else
    switch(pstFramework->eFrameworkLog)
    {
        case FRAMEWORK_LOG_ALL:
            s_stTimeManager.bLogLevel = bOnOff;
            nRet = TIME_MANAGER_SetLog(&s_stTimeManager);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("TIME_MANAGER_SetLog() is failed! [nRet:%d]", nRet);
            }

            s_stMsgManager.bLogLevel = bOnOff;
            nRet = MSG_MANAGER_SetLog(&s_stMsgManager);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("MSG_MANAGER_SetLog() is failed! [nRet:%d]", nRet);
            }

            s_stDbManager.bLogLevel = bOnOff;
            nRet = DB_MANAGER_SetLog(&s_stDbManager);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("DB_MANAGER_SetLog() is failed! [nRet:%d]", nRet);
            }
#endif
            break;

        default:
            PrintError("Unknown Log Type [%d]", pstFramework->eFrameworkLog);
            break;
    }
}

int32_t FRAMEWORK_Init(FRAMEWORK_T *pstFramework)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstFramework == NULL)
    {
        PrintError("pstFramework == NULL!!");
        return nRet;
    }
#if defined(CONFIG_MULTI_DEV)
    (void*)memset(&s_stMultiMsgManager, 0x00, sizeof(MULTI_MSG_MANAGER_T));
    (void*)memset(&s_stMultiDbManager, 0x00, sizeof(MULTI_DB_MANAGER_T));
    (void*)memset(&s_stMultiDbManager.stMultiDbFile, 0x00, sizeof(MULTI_DB_MANAGER_FILE_T));
    (void*)memset(&s_stTimeManager, 0x00, sizeof(TIME_MANAGER_T));

    PrintWarn("is successfully initialized.");

    s_stTimeManager.bLogLevel = OFF;

    nRet = TIME_MANAGER_Init(&s_stTimeManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("TIME_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("TIME_MANAGER_Init() is successfully initialized, s_stTimeManager[0x%p]", &s_stTimeManager);

    (void)TIME_MANAGER_CheckLatencyBegin(&s_stTimeManager);

    s_stMultiMsgManager.bLogLevel = OFF;

    nRet = MULTI_MSG_MANAGER_Init(&s_stMultiMsgManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("MULTI_MSG_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("MULTI_MSG_MANAGER_Init() is successfully initialized, s_stMultiMsgManager[0x%p]", &s_stMultiMsgManager);

    s_stMultiDbManager.bLogLevel = OFF;

    nRet = MULTI_DB_MANAGER_Init(&s_stMultiDbManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("MULTI_DB_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("MULTI_DB_MANAGER_Init() is successfully initialized, s_stDbManager[0x%p]", &s_stMultiDbManager);
#else
    (void*)memset(&s_stMsgManager, 0x00, sizeof(MSG_MANAGER_T));
    (void*)memset(&s_stDbManager, 0x00, sizeof(DB_MANAGER_T));
    (void*)memset(&s_stDbManager.stDbFile, 0x00, sizeof(DB_MANAGER_FILE_T));
    (void*)memset(&s_stTimeManager, 0x00, sizeof(TIME_MANAGER_T));

    PrintWarn("is successfully initialized.");

    s_stTimeManager.bLogLevel = OFF;

    nRet = TIME_MANAGER_Init(&s_stTimeManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("TIME_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("TIME_MANAGER_Init() is successfully initialized, s_stTimeManager[0x%p]", &s_stTimeManager);

    (void)TIME_MANAGER_CheckLatencyBegin(&s_stTimeManager);

    s_stMsgManager.bLogLevel = OFF;

    nRet = MSG_MANAGER_Init(&s_stMsgManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("MSG_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("MSG_MANAGER_Init() is successfully initialized, s_stMsgManager[0x%p]", &s_stMsgManager);

    s_stDbManager.bLogLevel = OFF;

    nRet = DB_MANAGER_Init(&s_stDbManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("DB_MANAGER_Init() is failed! [nRet:%d]", nRet);
        return nRet;
    }

    PrintDebug("DB_MANAGER_Init() is successfully initialized, s_stDbManager[0x%p]", &s_stDbManager);
#endif
    (void)TIME_MANAGER_CheckLatencyEnd(&s_stTimeManager);
    (void)TIME_MANAGER_CheckLatencyTime("Framework Init Time", &s_stTimeManager);

    return nRet;
}

#if defined(CONFIG_MULTI_DEV)
MULTI_MSG_MANAGER_T* FRAMEWORK_GetMultiMsgManagerInstance(void)
{
    return &s_stMultiMsgManager;
}

MULTI_DB_MANAGER_T* FRAMEWORK_GetMultiDbManagerInstance(void)
{
    return &s_stMultiDbManager;
}
#else
MSG_MANAGER_T* FRAMEWORK_GetMsgManagerInstance(void)
{
    return &s_stMsgManager;
}

DB_MANAGER_T* FRAMEWORK_GetDbManagerInstance(void)
{
    return &s_stDbManager;
}
#endif

TIME_MANAGER_T* FRAMEWORK_GetTimeManagerInstance(void)
{
    return &s_stTimeManager;
}

