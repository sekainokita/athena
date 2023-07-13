#ifndef	_DI_H_
#define	_DI_H_

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
* @file di.h
*
* @note
*
* V2X DI Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "config.h"
#include "di_gps.h"

/***************************** Definition ************************************/

/***************************** Enum and Structure ****************************/

typedef enum {
    DI_LOG_ALL                   = 0,
    DI_LOG_GPS_ALL               = 10,
    DI_LOG_CAMERA                = 11,
    DI_LOG_VIDEO                 = 12,
    DI_LOG_MAX                   = 0xFFFF
} DI_LOG_E;

typedef struct DI_t {
    DI_LOG_E eDiLog;
    DI_GPS_T stDiGps;
    uint32_t unReserved;
} DI_T;

/***************************** Function Protype ******************************/
void DI_SetLog(DI_T *pstDi, bool bOnOff);
int32_t DI_Init(DI_T *pstDi);

#endif	/* _DI_H_ */

