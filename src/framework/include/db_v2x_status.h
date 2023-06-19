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

/***************************** Enum and Structure ****************************/
/**
* @details DB V2X Receive Level
* @param DB_V2X_STATUS_RSV_LEVEL_E
*/
typedef enum {
    DB_V2X_STATUS_RSV_LEVEL_UNKNOWN             = DB_V2X_STATUS_RSV_LEVEL_MAX,
    DB_V2X_STATUS_RSV_LEVEL_NO_SIGNAL           = 0,
    DB_V2X_STATUS_RSV_LEVEL_1                   = 1,
    DB_V2X_STATUS_RSV_LEVEL_2                   = 2,
    DB_V2X_STATUS_RSV_LEVEL_3                   = 3,
    DB_V2X_STATUS_RSV_LEVEL_4                   = 4,
    DB_V2X_STATUS_RSV_LEVEL_MAX                 = 255
} DB_V2X_STATUS_RSV_LEVEL_E;

/**
* @details DB V2X Channel
* @param DB_V2X_STATUS_CHANNEL_E
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
* @details DB V2X Struct of Rx Position
* @param usCommDistance                         The distance to transmission device (based on GPS signal of communication device)
* @param nRxLatitude                            (The Latitude of the Rx device) units of 1/10 micro degree, Providing a range of ± 90 degrees (1 degree = 1,000,000 micro degree)
* @param nRxLongitude                           (The Longitude of the Rx device) units of 1/10 micro degree, Providing a range of ± 180 degrees (1 degree = 1,000,000 micro degree)
*/
typedef struct DB_V2X_POSITION_RX_t {
    uint16_t                                    usCommDistance;
    int32_t                                     nRxLatitude;
    int32_t                                     nRxLongitude;
} DB_V2X_POSITION_RX_T;

/**
* @details DB V2X Struct of Rx Status
* @param ulRxTimeStamp                          The system time value at which the communication device received the message (need to change to future UTF time, C-ITS communication time compatibility)
* @param unTxDeviceId                           The unique ID (Serial Number) of the transfer device
* @param ulRxDeviceTimeStamp                    The system time value at which the OBU/RSU device received the message
* @param ulLatency                              rTimestamp - tTimestamp = latency on communication modem device, status of synchronization to GPS Pulse Per Second (PPS)
* @param unPdr                                  Packet delivery ratio (PDR)
* @param usRssi                                 Receive Signal Strength Indication, The total signal strength of the antenna at the time the message was received
* @param eRsvLevel                              Receive signal level, the level of the received signal strength of the mobile communication at the time the message is received (antenna signal)
* @param stRxPosition                           see DB_V2X_POSITION_RX_T
*/
typedef struct DB_V2X_STATUS_RX_t {
    uint64_t                                    ulRxTimeStamp;
    uint32_t                                    unTxDeviceId;
    uint64_t                                    ulRxDeviceTimeStamp;
    uint64_t                                    ulLatency;
    uint32_t                                    unPdr;
    uint16_t                                    usRssi;
    DB_V2X_STATUS_RSV_LEVEL_E                   eRsvLevel;
    DB_V2X_POSITION_RX_T                        stRxPosition;
} DB_V2X_STATUS_RX_T;

/**
* @details DB V2X Struct of Tx Position
* @param nTxLatitude                            (The Latitude of the Tx device) units of 1/10 micro degree, Providing a range of ± 90 degrees (1 degree = 1,000,000 micro degree)
* @param nTxLongitude                           (The Longitude of the Rx device) units of 1/10 micro degree, Providing a range of ± 180 degrees (1 degree = 1,000,000 micro degree)
*/
typedef struct DB_V2X_POSITION_TX_t {
    int32_t                                     nTxLatitude;
    int32_t                                     nTxLongitude;
} DB_V2X_POSITION_TX_T;

/**
* @details DB V2X Struct of Tx Status
* @param ulTxTimeStamp                          The system time value at which the communication device transmitted the message (need to change to future UTF time, C-ITS communication time compatibility)
* @param unRxDeviceId                           Unique Serial Number of the device to which you want to send the message; there can be more than one transfer destination
* @param eChannel                               Transmission channel number
* @param sPower                                 Transmission strength (dBm)
* @param eBandwidth                             Communication frequency bandwidth, Communication frequency bandwidth status at the time the message is received
* @param stTxPosition                           see DB_V2X_POSITION_TX_T
*/
typedef struct DB_V2X_STATUS_TX_t {
    uint64_t                                    ulTxTimeStamp;
    uint32_t                                    unRxDeviceId;
    DB_V2X_STATUS_CHANNEL_E                     eChannel;
    int16_t                                     sPower;
    DB_V2X_STATUS_BANDWIDTH_E                   eBandwidth;
    DB_V2X_POSITION_TX_T                        stTxPosition;
} DB_V2X_STATUS_TX_T;
/***************************** Function Protype ******************************/

#endif	/* _DB_V2X_STATUS_H_ */

