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
#include <math.h>

/***************************** Definition ************************************/
#define PI                      (3.14159265358979323846)
#define DI_GPS_MIN_MAX          (60)
#define DI_GPS_SEC_MAX          (60)
#define DI_GPS_MS               (1000)
#define DI_GPS_CONVERT_KM       (1000)
#define DI_GPS_NS               (100) /* Cut x10, it is deleted when Tx because of the length of data size */
#define DI_GPS_STRAIGHT_ANGLES  (180.0)
#define DI_GPS_FULL_ANGLES      (360.0)

/***************************** Enum and Structure ****************************/

/***************************** Static Variable *******************************/
static bool s_bDiGpsLog = OFF;
static DI_GPS_XSENS_T s_stDiGpsDev;
static bool s_bLogOnOff = FALSE;

/***************************** Function  *************************************/

static inline double P_DI_GPS_ConvertDegreeToRadian(double dTheta)
{
    return (dTheta * M_PI) / DI_GPS_STRAIGHT_ANGLES;
}

static inline double P_DI_GPS_ConvertRadianToDegree(double dTheta){
    return (dTheta * DI_GPS_STRAIGHT_ANGLES) / M_PI;
}

static int32_t P_DI_GPS_Init(DI_GPS_T *pstDiGps)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    if (pstDiGps->bGpsNotAvailable == TRUE)
    {
        PrintWarn("bGpsNotAvailable[%d]", pstDiGps->bGpsNotAvailable);
        nRet = DI_OK;
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
#else
    nRet = DI_OK;
    PrintWarn("None of GPS devices are supported.");
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

    if (pstDiGps->bGpsNotAvailable == TRUE)
    {
        PrintWarn("bGpsNotAvailable[%d]", pstDiGps->bGpsNotAvailable);
        nRet = DI_OK;
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
#else
    nRet = DI_OK;
    PrintWarn("None of GPS devices are supported.");
#endif

    return nRet;
}

double P_DI_GPS_SetToRadians(double dDegree)
{
    double dRadian = dDegree * PI / 180;

    return dRadian;
}

static double P_DI_GPS_CalculateDistance(double dRxLat, double dRxLon, double dTxLat, double dTxLon)
{
    /* Refenrece Codes : https://github.com/janantala/GPS-distance/blob/master/c/distance.c */

    double a = 6378137, b = 6356752.314245, f = 1 / 298.257223563;
    double L = P_DI_GPS_SetToRadians(dTxLon - dRxLon);

    double U1 = atan((1 - f) * tan(P_DI_GPS_SetToRadians(dRxLat)));
    double U2 = atan((1 - f) * tan(P_DI_GPS_SetToRadians(dTxLat)));
    double sinU1 = sin(U1), cosU1 = cos(U1);
    double sinU2 = sin(U2), cosU2 = cos(U2);
    double cosSqAlpha;
    double sinSigma;
    double cos2SigmaM;
    double cosSigma;
    double sigma;
    double lambda = L, lambdaP, iterLimit = 100;
    double uSq, A, B, deltaSigma, dDistanceMeters;

    if(s_bLogOnOff == TRUE)
    {
        PrintDebug("Rx Lat[%lf], lon[%lf] : Tx Lat[%lf], lon[%lf]", dRxLat, dRxLon, dTxLat, dTxLon);
    }

    do
    {
        double sinLambda = sin(lambda), cosLambda = cos(lambda);

        sinSigma = sqrt((cosU2 * sinLambda)
                    * (cosU2 * sinLambda)
                        + (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
                            * (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
                        );

        if (sinSigma == 0)
        {
            return 0;
        }

        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = atan2(sinSigma, cosSigma);
        double sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1 - sinAlpha * sinAlpha;
        cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;

        double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
        lambdaP = lambda;
        lambda = L + (1 - C) * f * sinAlpha
                *   (sigma + C * sinSigma
                    *   (cos2SigmaM + C * cosSigma
                        *   (-1 + 2 * cos2SigmaM * cos2SigmaM)
                        )
                    );
    } while (fabs(lambda - lambdaP) > 1e-12 && --iterLimit > 0);

    if (iterLimit == 0)
    {
        return 0;
    }

    uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    A = 1 + uSq / 16384
            * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    deltaSigma =
                B * sinSigma
                    * (cos2SigmaM + B / 4
                        * (cosSigma
                            * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM
                                * (-3 + 4 * sinSigma * sinSigma)
                                    * (-3 + 4 * cos2SigmaM * cos2SigmaM)));

    dDistanceMeters = b * A * (sigma - deltaSigma);

    if(s_bLogOnOff == TRUE)
    {
        PrintDebug("Distance [%lf] meters", dDistanceMeters);
    }

    return dDistanceMeters;
}

double DI_GPS_CalculateHeading(DB_V2X_GPS_INFO_T *pstV2xGpsInfo)
{
    double dHeading = 0.0f;
    double dLat1, dLat2, dLon1, dLon2;

    if(pstV2xGpsInfo == NULL)
    {
        PrintError("pstV2xSpeed == NULL!!");
        return 0.0f;
    }

    dLat1 = pstV2xGpsInfo->nLatitudeNow;
    dLon1 = pstV2xGpsInfo->nLongitudeNow;

    dLat2 = pstV2xGpsInfo->nLatitudeLast;
    dLon2 = pstV2xGpsInfo->nLongitudeLast;

    dLat1 = P_DI_GPS_ConvertDegreeToRadian(dLat1);
    dLon1 = P_DI_GPS_ConvertDegreeToRadian(dLon1);
    dLat2 = P_DI_GPS_ConvertDegreeToRadian(dLat2);
    dLon2 = P_DI_GPS_ConvertDegreeToRadian(dLon2);

    double dTempLon = dLon2 - dLon1;
    double X = cos(dLat2) * sin(dTempLon);
    double Y = cos(dLat1) * sin(dLat2) - sin(dLat1) * cos(dLat2) * cos(dTempLon);

    dHeading = atan2(X,Y);

    dHeading = P_DI_GPS_ConvertRadianToDegree(dHeading);
    if (dHeading < 0)
    {
        /* a uniform heading of >=0 and <360 */
        dHeading = 360.0 + dHeading;
    }

    return dHeading;
}

uint16_t DI_GPS_CalculateSpeed(DB_V2X_GPS_INFO_T *pstV2xGpsInfo)
{
    double dDistMeter;
    double dDistKiloMeter;
    double dSpeed;
    uint16_t usSpeed;
    double dDiffTimeNs;
    double dDiffTimeMs;
    double dDiffTimeS;
    double dDiffTimeM;
    double dDiffTimeH;

    if(pstV2xGpsInfo == NULL)
    {
        PrintError("pstV2xSpeed == NULL!!");
        return 0;
    }

    dDistMeter = (P_DI_GPS_CalculateDistance((double)pstV2xGpsInfo->nLatitudeNow / DB_V2X_GPS_VALUE_CONVERT_DOUBLE, (double)pstV2xGpsInfo->nLongitudeNow / DB_V2X_GPS_VALUE_CONVERT_DOUBLE,
        (double)pstV2xGpsInfo->nLatitudeLast / DB_V2X_GPS_VALUE_CONVERT_DOUBLE, (double)pstV2xGpsInfo->nLongitudeLast / DB_V2X_GPS_VALUE_CONVERT_DOUBLE));

    dDistKiloMeter = dDistMeter / DI_GPS_CONVERT_KM;

    dDiffTimeNs = (pstV2xGpsInfo->ulTimeStampNow - pstV2xGpsInfo->ulTimeStampLast);
    dDiffTimeMs = dDiffTimeNs / DI_GPS_NS;
    dDiffTimeS = dDiffTimeMs / DI_GPS_MS;
    dDiffTimeM = dDiffTimeS / DI_GPS_SEC_MAX;
    dDiffTimeH = dDiffTimeM / DI_GPS_MIN_MAX;

    dSpeed = dDistKiloMeter / dDiffTimeH;
    usSpeed = (uint16_t)dSpeed;

    if(s_bDiGpsLog == TRUE)
    {
        PrintDebug("%lfm/%lfs=%lfm/s, %lfkm/%lfh=%lfkm/h, usSpeed[%d]km/h", dDistMeter, dDiffTimeS, dDistMeter/dDiffTimeS, dDistKiloMeter, dDiffTimeH, dSpeed, usSpeed);
    }

    return usSpeed;
}

double DI_GPS_CalculateDistance(double dRxLat, double dRxLon, double dTxLat, double dTxLon)
{
    /* Refenrece Codes : https://github.com/janantala/GPS-distance/blob/master/c/distance.c */

    double a = 6378137, b = 6356752.314245, f = 1 / 298.257223563;
    double L = P_DI_GPS_SetToRadians(dTxLon - dRxLon);

    double U1 = atan((1 - f) * tan(P_DI_GPS_SetToRadians(dRxLat)));
    double U2 = atan((1 - f) * tan(P_DI_GPS_SetToRadians(dTxLat)));
    double sinU1 = sin(U1), cosU1 = cos(U1);
    double sinU2 = sin(U2), cosU2 = cos(U2);
    double cosSqAlpha;
    double sinSigma;
    double cos2SigmaM;
    double cosSigma;
    double sigma;
    double lambda = L, lambdaP, iterLimit = 100;
    double uSq, A, B, deltaSigma, dDistanceMeters;

    if(s_bLogOnOff == TRUE)
    {
        PrintDebug("Rx Lat[%lf], lon[%lf] : Tx Lat[%lf], lon[%lf]", dRxLat, dRxLon, dTxLat, dTxLon);
    }

    do
    {
        double sinLambda = sin(lambda), cosLambda = cos(lambda);

        sinSigma = sqrt((cosU2 * sinLambda)
                    * (cosU2 * sinLambda)
                        + (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
                            * (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
                        );

        if (sinSigma == 0)
        {
            return 0;
        }

        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = atan2(sinSigma, cosSigma);
        double sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1 - sinAlpha * sinAlpha;
        cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;

        double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
        lambdaP = lambda;
        lambda = L + (1 - C) * f * sinAlpha
                *   (sigma + C * sinSigma
                    *   (cos2SigmaM + C * cosSigma
                        *   (-1 + 2 * cos2SigmaM * cos2SigmaM)
                        )
                    );
    } while (fabs(lambda - lambdaP) > 1e-12 && --iterLimit > 0);

    if (iterLimit == 0)
    {
        return 0;
    }

    uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    A = 1 + uSq / 16384
            * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    deltaSigma =
                B * sinSigma
                    * (cos2SigmaM + B / 4
                        * (cosSigma
                            * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM
                                * (-3 + 4 * sinSigma * sinSigma)
                                    * (-3 + 4 * cos2SigmaM * cos2SigmaM)));

    dDistanceMeters = b * A * (sigma - deltaSigma);

    if(s_bLogOnOff == TRUE)
    {
        PrintDebug("Distance [%lf] meters", dDistanceMeters);
    }

    return dDistanceMeters;
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

    if (pstDiGps->bGpsNotAvailable == TRUE)
    {
        (void*)memset(&pstDiGps->stDiGpsData, 0x00, sizeof(DI_GPS_DATA_T));

        nRet = DI_OK;
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
        pstDiGps->stDiGpsData.fAccX = s_stDiGpsDev.fAccX;
        pstDiGps->stDiGpsData.fAccY = s_stDiGpsDev.fAccY;
        pstDiGps->stDiGpsData.fAccZ = s_stDiGpsDev.fAccZ;

        pstDiGps->stDiGpsData.fGyrX = s_stDiGpsDev.fGyrX;
        pstDiGps->stDiGpsData.fGyrY = s_stDiGpsDev.fGyrY;
        pstDiGps->stDiGpsData.fGyrZ = s_stDiGpsDev.fGyrZ;

        pstDiGps->stDiGpsData.fMagX = s_stDiGpsDev.fMagX;
        pstDiGps->stDiGpsData.fMagY = s_stDiGpsDev.fMagY;
        pstDiGps->stDiGpsData.fMagZ = s_stDiGpsDev.fMagZ;

        pstDiGps->stDiGpsData.fQuaternionW = s_stDiGpsDev.fQuaternionW;
        pstDiGps->stDiGpsData.fQuaternionX = s_stDiGpsDev.fQuaternionX;
        pstDiGps->stDiGpsData.fQuaternionY = s_stDiGpsDev.fQuaternionY;
        pstDiGps->stDiGpsData.fQuaternionZ = s_stDiGpsDev.fQuaternionZ;

        pstDiGps->stDiGpsData.fEulerRoll = s_stDiGpsDev.fEulerRoll;
        pstDiGps->stDiGpsData.fEulerPitch = s_stDiGpsDev.fEulerPitch;
        pstDiGps->stDiGpsData.fEulerYaw = s_stDiGpsDev.fEulerYaw;
        pstDiGps->stDiGpsData.fLatitude = s_stDiGpsDev.fLatitude;
        pstDiGps->stDiGpsData.fLongitude = s_stDiGpsDev.fLongitude;
        pstDiGps->stDiGpsData.fAltitude = s_stDiGpsDev.fAltitude;
        pstDiGps->stDiGpsData.fVelocityEast = s_stDiGpsDev.fVelocityEast;
        pstDiGps->stDiGpsData.fVelocityNorth = s_stDiGpsDev.fVelocityNorth;
        pstDiGps->stDiGpsData.fVelocityUp = s_stDiGpsDev.fVelocityUp;
    }
    else
    {
        PrintWarn("check the status of GPS [%d]", pstDiGps->eDiGpsStatus);
    }

    return nRet;
}

int32_t DI_GPS_SetNa(DI_GPS_T *pstDiGps, bool bNotAvailable)
{
    int32_t nRet = DI_ERROR;

    if(pstDiGps == NULL)
    {
        PrintError("pstDiGps == NULL!!");
        return nRet;
    }

    pstDiGps->bGpsNotAvailable = bNotAvailable;

    PrintDebug("bNotAvailable[%d] => pstDiGps->bGpsNotAvailable[%d]", bNotAvailable, pstDiGps->bGpsNotAvailable);
    nRet = DI_OK;

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

    if (pstDiGps->bGpsNotAvailable == TRUE)
    {
        PrintWarn("bGpsNotAvailable[%d]", pstDiGps->bGpsNotAvailable);
        pstDiGps->eDiGpsStatus = DI_GPS_STATUS_OPENED;

        nRet = DI_OK;
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
#else
        nRet = DI_OK;
        PrintWarn("None of GPS devices are supported.");
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

    if (pstDiGps->bGpsNotAvailable == TRUE)
    {
        PrintWarn("bGpsNotAvailable[%d]", pstDiGps->bGpsNotAvailable);
        pstDiGps->eDiGpsStatus = DI_GPS_STATUS_CLOSED;

        nRet = DI_OK;
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
#else
        nRet = DI_OK;
        PrintWarn("None of GPS devices are supported.");
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


