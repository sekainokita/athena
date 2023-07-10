#ifndef	_DI_GPS_XSENS_H_
#define	_DI_GPS_XSENS_H_

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
* @file di_gps.h
*
* @note
*
* DI GPS Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "config.h"
#include "di.h"

/***************************** Definition ************************************/

/***************************** Enum and Structure ****************************/

/**
* @details DI_GPS_XSENS_T
* @param unReserved
*/
typedef struct DI_GPS_XSENS_t {
    bool                        bLogLevel;
    uint32_t                    unReserved;
} DI_GPS_XSENS_T;

/***************************** Function Protype ******************************/
#ifdef __cplusplus
    extern "C" {
#endif

int32_t DI_GPS_XSENS_Init(DI_GPS_XSENS_T *pstDiGpsXsens);
int32_t DI_GPS_XSENS_DeInit(DI_GPS_XSENS_T *pstDiGpsXsens);

int32_t DI_GPS_XSENS_SetLog(DI_GPS_XSENS_T *pstDiGpsXsens);
int32_t DI_GPS_XSENS_Get(DI_GPS_XSENS_T *pstDiGpsXsens);

int32_t DI_GPS_XSENS_Open(DI_GPS_XSENS_T *pstDiGpsXsens);
int32_t DI_GPS_XSENS_Close(DI_GPS_XSENS_T *pstDiGpsXsens);
int32_t DI_GPS_XSENS_Start(DI_GPS_XSENS_T *pstDiGpsXsens);
int32_t DI_GPS_XSENS_Stop(DI_GPS_XSENS_T *pstDiGpsXsens);

void DI_GPS_XSENS_Status(DI_GPS_XSENS_T *pstDiGpsXsens);
int32_t DI_GPS_XSENS_Init(DI_GPS_XSENS_T *pstDiGpsXsens);
int32_t DI_GPS_XSENS_DeInit(DI_GPS_XSENS_T *pstDiGpsXsens);

#ifdef __cplusplus
   }
#endif

#endif	/* _DI_GPS_XSENS_H_ */

