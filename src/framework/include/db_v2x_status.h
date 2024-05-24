#ifndef	_DB_V2X_STATUS_H_
#define	_DB_V2X_STATUS_H_

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
* @file db_v2x_status.h
*
* This file contains a data format design of V2X status
*
* @note
*
* V2X Data Format Header of Status
*
* MODIFICATION HISTORY:
* Ver   Who  Date     Changes
* ----- ---- -------- ----------------------------------------------------
* 1  bman  23.06.19 First draft
*
******************************************************************************/


/***************************** Include ***************************************/
#include <stdint.h>

/***************************** Definition ************************************/

#define DB_V2X_STATUS_SEQ_NUM_MAX               (4294967295) /* maximum value of uint32 */
#define DB_V2X_STATUS_CONT_CNT_MAX              (100)
#define DB_V2X_GPS_VALUE_CONVERT                (1000000)
#define DB_V2X_GPS_VALUE_CONVERT_DOUBLE         (1000000.0f)

/***************************** Enum and Structure ****************************/
/**
* @details DB V2X Receive Level
* @param DB_V2X_STATUS_RSV_LEVEL_E
*/
typedef enum {
    DB_V2X_STATUS_RSV_LEVEL_UNKNOWN             = 255,
    DB_V2X_STATUS_RSV_LEVEL_NO_SIGNAL           = 0,
    DB_V2X_STATUS_RSV_LEVEL_1                   = 1,
    DB_V2X_STATUS_RSV_LEVEL_2                   = 2,
    DB_V2X_STATUS_RSV_LEVEL_3                   = 3,
    DB_V2X_STATUS_RSV_LEVEL_4                   = 4,
    DB_V2X_STATUS_RSV_LEVEL_MAX                 = 255
} DB_V2X_STATUS_RSV_LEVEL_E;

/**
* @details DB V2X Channel of Communication Numbers
* @param CH172, 5.855 - 5.865GHz (Non-safety applications)
* @param CH174, 5.865 - 5.875GHz (Non-safety applications)
* @param CH176, 5.875 - 5.885GHz (Safety and traffic applications)
* @param CH178, 5.885 - 5.895GHz (Safety and traffic applications)
* @param CH180, 5.895 - 5.905GHz (Safety and traffic applications)
* @param CH182, 5.905 - 5.915GHz (Safety and traffic applications)
* @param CH184, 5.915 - 5.925GHz (Safety and traffic applications)
*/
typedef enum {
    DB_V2X_STATUS_CHANNEL_UNKNOWN               = 0,
    DB_V2X_STATUS_CHANNEL_5_855_5_865           = 172,
    DB_V2X_STATUS_CHANNEL_5_865_5_875           = 174,
    DB_V2X_STATUS_CHANNEL_5_875_5_885           = 176,
    DB_V2X_STATUS_CHANNEL_5_885_5_895           = 178,
    DB_V2X_STATUS_CHANNEL_5_895_5_905           = 180,
    DB_V2X_STATUS_CHANNEL_5_905_5_915           = 182,
    DB_V2X_STATUS_CHANNEL_5_915_5_925           = 184,
    DB_V2X_STATUS_CHANNEL_MAX                   = 255
} DB_V2X_STATUS_CHANNEL_E;

/**
* @details DB V2X Bandwidth
* @param DB_V2X_STATUS_BANDWIDTH_E
*/
typedef enum {
    DB_V2X_STATUS_BANDWIDTH_UNKNOWN             = 0,
    DB_V2X_STATUS_BANDWIDTH_10MHZ               = 1,
    DB_V2X_STATUS_BANDWIDTH_20MHZ               = 2,
    DB_V2X_STATUS_BANDWIDTH_30MHZ               = 3,
    DB_V2X_STATUS_BANDWIDTH_MAX                 = 255
} DB_V2X_STATUS_BANDWIDTH_E;

/**
* @details GPS Info
* @param ulTimeStampNow                         The current timestamp
* @param nLatitudeNow                           The current latitude
* @param nLongitudeNow                          The current longitude
* @param ulTimeStampLast                        The last timestamp
* @param nLatitudeLast                          The last latitude
* @param nLongitudeLast                         The last longitude
*/
typedef struct DB_V2X_STATUS_GPS_INFO_t {
    uint64_t                                    ulTimeStampNow;
    int32_t                                     nLatitudeNow;
    int32_t                                     nLongitudeNow;
    uint64_t                                    ulTimeStampLast;
    int32_t                                     nLatitudeLast;
    int32_t                                     nLongitudeLast;
} __attribute__((__packed__)) DB_V2X_GPS_INFO_T;

/**
* @details DB V2X Struct of Rx Position
* @param usCommDistance                         The distance to transmission device (based on GPS signal of communication device)
* @param nRxLatitude                            (The Latitude of the Rx device) units of 1/10 micro degree, Providing a range of ± 90 degrees (1 degree = 1,000,000 micro degree)
* @param nRxLongitude                           (The Longitude of the Rx device) units of 1/10 micro degree, Providing a range of ± 180 degrees (1 degree = 1,000,000 micro degree)
* @param nRxAttitude                            (The Attitude of the Rx device)
*/
typedef struct DB_V2X_POSITION_RX_t {
    uint32_t                                    unCommDistance;
    int32_t                                     nRxLatitude;
    int32_t                                     nRxLongitude;
    int32_t                                     nRxAttitude;
} __attribute__((__packed__)) DB_V2X_POSITION_RX_T;

/**
* @details DB_V2X_DEV_INFO_T
* @param unDevId                                the device ID
* @param usHwVer                                the HW version
* @param usSwVer                                the SW version
* @param ulTimeStamp                            the timestamp, value at which the communication device transmitted the message (need to change to future UTF time, C-ITS communication time compatibility)
* @param ulLatency                              the latency
*/
typedef struct DB_V2X_DEV_INFO_t {
    uint32_t                                    unDevId;
    uint16_t                                    usHwVer;
    uint16_t                                    usSwVer;
    uint64_t                                    ulTimeStamp;
    uint64_t                                    ulLatency;
} __attribute__((__packed__)) DB_V2X_DEV_INFO_T;

/**
* @details DB V2X Struct of Rx Status
* @param stDbV2xDevL1                           The system layer 1 device info
* @param stDbV2xDevL2                           The system layer 2 device info
* @param stDbV2xDevL3                           The system layer 3 device info
* @param unRxVehicleSpeed                       Rx vehicle speed (Rx vehicle speed (if available), if not available, set the experimental value manually, default 60km/h)
* @param unRxVehicleHeading                     Rx vehicle heading (0~360, 0(S), 90(E), -90(W), N(+-180))
* @param unTotalCommDevCnt                      The total number of devices that are currently simultaneously connected and transmitting/receiving (the count number of all device IDs currently received and stored by the Rx device)
* @param nRssi                                  Receive Signal Strength Indication, The total signal strength of the antenna at the time the message was received
* @param ucRcpi                                 RCPI
* @param eRsvLevel                              Receive signal level, the level of the received signal strength of the mobile communication at the time the message is received (antenna signal)
* @param stRxPosition                           see DB_V2X_POSITION_RX_T
* @param ucErrIndicator                         If an error occurs in the sequence in the Tx Continuity Counter transmitted to Tx, an error is displayed (ok : 0, error : 1)
* @param ulTotalPacketCnt                       Total packets counts received from the same device
* @param ulTotalErrCnt                          The number of txContinuityCounter error packets among packets received from the same device
* @param unPdr                                  Packet Delivery Ratio (PDR)
* @param unPer                                  Packet Error Ratio (PER)
*/
typedef struct DB_V2X_STATUS_RX_t {
    DB_V2X_DEV_INFO_T                           stDbV2xDevL1;
    DB_V2X_DEV_INFO_T                           stDbV2xDevL2;
    DB_V2X_DEV_INFO_T                           stDbV2xDevL3;
    uint16_t                                    unRxVehicleSpeed;
    uint32_t                                    unRxVehicleHeading;
    uint32_t                                    unTotalCommDevCnt;
    uint16_t                                    nRssi;
    uint8_t                                     ucRcpi;
    DB_V2X_STATUS_RSV_LEVEL_E                   eRsvLevel;
    DB_V2X_POSITION_RX_T                        stRxPosition;
    uint8_t                                     ucErrIndicator;
    uint64_t                                    ulTotalPacketCnt;
    uint64_t                                    ulTotalErrCnt;
    uint32_t                                    unPdr;
    uint32_t                                    unPer;
} __attribute__((__packed__)) DB_V2X_STATUS_RX_T;

/**
* @details DB V2X Struct of Tx Position
* @param nTxLatitude                            (The Latitude of the Tx device) units of 1/10 micro degree, Providing a range of ± 90 degrees (1 degree = 1,000,000 micro degree)
* @param nTxLongitude                           (The Longitude of the Tx device) units of 1/10 micro degree, Providing a range of ± 180 degrees (1 degree = 1,000,000 micro degree)
* @param nTxLongitude                           (The Attitude of the Tx device) (Optional, if it is available)
*/
typedef struct DB_V2X_POSITION_TX_t {
    int32_t                                     nTxLatitude;
    int32_t                                     nTxLongitude;
    int32_t                                     nTxAttitude;
} __attribute__((__packed__)) DB_V2X_POSITION_TX_T;


/**
* @details DB V2X Struct of Tx Status
* @param stDbV2xDevL1                           The system layer 1 device info
* @param stDbV2xDevL2                           The system layer 2 device info
* @param stDbV2xDevL3                           The system layer 3 device info
* @param unRxTargetDeviceId                     Unique Serial Number of the device to which you want to send the message; there can be more than one transfer destination
* @param usTxFreq                               Transmission frequency
* @param ucTxPwr                                Transmission strength (dBm)
* @param ucTxBw                                 Communication frequency bandwidth, Communication frequency bandwidth status at the time the message is received
* @param ucScs                                  SCS
* @param ucMcs                                  MCS
* @param usTxRatio                              Tx packet trasmisstion ratio (frequency) (10ms, 100ms, etc)
* @param stTxPosition                           see DB_V2X_POSITION_TX_T
* @param unSeqNum                               sequence number (Sequential transmission of values ​​from 0 to N)
* @param unContCnt                              continuity counter Sequential transmission of values ​​from 0 to 100, The Rx device checks and stores the values ​​of 0 to 100 transmitted from Tx sequentially transmitted Tx Continuity Counter (sequence number)
* @param unTxVehicleSpeed                       Tx vehicle speed (Tx vehicle speed (if available), if not available, set the experimental value manually, default 60km/h)
* @param unTxVehicleHeading                     Tx vehicle heading (0~360, 0(S), 90(E), -90(W), N(+-180))
*/
typedef struct DB_V2X_STATUS_TX_t {
    uint64_t                                    ulReserved0;
    uint64_t                                    ulReserved1;
    DB_V2X_DEV_INFO_T                           stDbV2xDevL1;
    DB_V2X_DEV_INFO_T                           stDbV2xDevL2;
    DB_V2X_DEV_INFO_T                           stDbV2xDevL3;
    uint32_t                                    unRxTargetDeviceId;
    uint8_t                                     ucTxPwr;
    uint16_t                                    usTxFreq;
    uint8_t                                     ucTxBw;
    uint8_t                                     ucScs;
    uint8_t                                     ucMcs;
    uint16_t                                    usTxRatio;
    DB_V2X_POSITION_TX_T                        stTxPosition;
    uint32_t                                    unSeqNum;
    uint32_t                                    unContCnt;
    uint32_t                                    unTxVehicleSpeed;
    uint32_t                                    unTxVehicleHeading;
} __attribute__((__packed__)) DB_V2X_STATUS_TX_T;
/***************************** Function Protype ******************************/

#endif	/* _DB_V2X_STATUS_H_ */

