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
* @file msg_manager.c
*
* This file contains a data format design
*
* @note
*
* V2X Data Format Message Source File
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1.00  bman  23.04.07 First release
*
******************************************************************************/

/***************************** Include ***************************************/
#include "msg_manager.h"

/***************************** Definition ************************************/


/***************************** Static Variable *******************************/


/***************************** Function  *************************************/
int32_t MSG_MANAGER_Write(MSG_MANAGER_TX_T *pstMsgManagerWrite, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManagerWrite == NULL)
    {
        PrintError("pstMsgManagerWrite == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MSG_MANAGER_Read(MSG_MANAGER_RX_T *pstMsgManagerRead, DB_V2X_T *pstDbV2x, void* pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManagerRead == NULL)
    {
        PrintError("pstMsgManagerRead == NULL!!");
        return nRet;
    }

    if(pstDbV2x == NULL)
    {
        PrintError("pstDbV2x == NULL!!");
        return nRet;
    }

    if(pPayload == NULL)
    {
        PrintError("pPayload == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MSG_MANAGER_Open(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MSG_MANAGER_Close(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MSG_MANAGER_Start(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MSG_MANAGER_Stop(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MSG_MANAGER_Status(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    return nRet;
}

int32_t MSG_MANAGER_Init(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    nRet = FRAMEWORK_OK;

    return nRet;
}

int32_t MSG_MANAGER_DeInit(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    return nRet;
}


