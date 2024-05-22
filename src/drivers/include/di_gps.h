#ifndef	_DI_GPS_H_
#define	_DI_GPS_H_

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
#include "di.h"
#include "di_gps_xsens.h"
#include "db_v2x_status.h"

/***************************** Definition ************************************/

/***************************** Enum and Structure ****************************/
/**
* @details DI GPS Status
* @param DI_GPS_STATUS_E
*/
typedef enum {
    DI_GPS_STATUS_DEINITIALIZED                = 0,
    DI_GPS_STATUS_INITIALIZED                  = 1,
    DI_GPS_STATUS_CLOSED                       = 2,
    DI_GPS_STATUS_OPENED                       = 3,
    DI_GPS_STATUS_STARTED                      = 4,
    DI_GPS_STATUS_STOPPED                      = 5,
    DI_GPS_STATUS_MAX                          = 255,
} DI_GPS_STATUS_E;

/**
* @details DI_GPS_DATA_T
* @param unReserved
*/
typedef struct DI_GPS_DATA_t {
    float                       fAccX;
    float                       fAccY;
    float                       fAccZ;
    float                       fGyrX;
    float                       fGyrY;
    float                       fGyrZ;
    float                       fMagX;
    float                       fMagY;
    float                       fMagZ;
    float                       fQuaternionW;
    float                       fQuaternionX;
    float                       fQuaternionY;
    float                       fQuaternionZ;
    float                       fEulerRoll;
    float                       fEulerPitch;
    float                       fEulerYaw;
    float                       fLatitude;
    float                       fLongitude;
    float                       fAltitude;
    float                       fVelocityEast;
    float                       fVelocityNorth;
    float                       fVelocityUp;
} DI_GPS_DATA_T;

/**
* @details DI_GPS_T
* @param unReserved
*/
typedef struct DI_GPS_t {
    bool                        bLogLevel;
    bool                        bGpsNotAvailable;
    DI_GPS_STATUS_E             eDiGpsStatus;
    DI_GPS_DATA_T               stDiGpsData;
    uint32_t                    unReserved;
} DI_GPS_T;

/***************************** Function Protype ******************************/

int32_t DI_GPS_Init(DI_GPS_T *pstDiGps);
int32_t DI_GPS_DeInit(DI_GPS_T *pstDiGps);

int32_t DI_GPS_SetLog(DI_GPS_T *pstDiGps);
int32_t DI_GPS_Get(DI_GPS_T *pstDiGps);
int32_t DI_GPS_SetNa(DI_GPS_T *pstDiGps, bool bNotAvailable);

int32_t DI_GPS_Open(DI_GPS_T *pstDiGps);
int32_t DI_GPS_Close(DI_GPS_T *pstDiGps);
int32_t DI_GPS_Start(DI_GPS_T *pstDiGps);
int32_t DI_GPS_Stop(DI_GPS_T *pstDiGps);

double DI_GPS_GetHeading(DI_GPS_T *pstDiGps);
double DI_GPS_CalculateHeading(DB_V2X_GPS_INFO_T *pstV2xGpsInfo);
double DI_GPS_CalculateDistance(double dRxLat, double dRxLon, double dTxLat, double dTxLon);
uint16_t DI_GPS_CalculateSpeed(DB_V2X_GPS_INFO_T *pstV2xGpsInfo);

void DI_GPS_Status(DI_GPS_T *pstDiGps);
int32_t DI_GPS_Init(DI_GPS_T *pstDiGps);
int32_t DI_GPS_DeInit(DI_GPS_T *pstDiGps);
#endif	/* _DI_GPS_H_ */

