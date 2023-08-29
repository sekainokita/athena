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
* @details DB V2X Struct of Rx Position
* @param usCommDistance                         The distance to transmission device (based on GPS signal of communication device)
* @param nRxLatitude                            (The Latitude of the Rx device) units of 1/10 micro degree, Providing a range of ± 90 degrees (1 degree = 1,000,000 micro degree)
* @param nRxLongitude                           (The Longitude of the Rx device) units of 1/10 micro degree, Providing a range of ± 180 degrees (1 degree = 1,000,000 micro degree)
* @param nRxAttitude                            (The Attitude of the Rx device)

*/
typedef struct DB_V2X_POSITION_RX_t {
    uint16_t                                    usCommDistance;
    int32_t                                     nRxLatitude;
    int32_t                                     nRxLongitude;
    int32_t                                     nRxAttitude;
} __attribute__((__packed__)) DB_V2X_POSITION_RX_T;

/**
* @details DB V2X Struct of Rx Status
* @param ulRxTimeStampL1                        The system layer 1 time value at which the communication device transmitted the message (need to change to future UTF time, C-ITS communication time compatibility)
* @param ulRxTimeStampL2                        The system layer 2 time value at which the OBU/RSU/Control Units transmitted the message (Optional, if the devices are separated from model IP)
* @param ulRxTimeStampL2                        The system layer 3 time value at which the application device transmitted the message (Optional, if the devices are separated OBU/RSU)
* @param ulAvgLatencyL1                         [L1] rTimestamp - tTimestamp = latency, synchronized with GPS PPS (Pulse Per Second)
* @param ulAvgLatencyL2                         [L2] rTimestamp - tTimestamp = latency, synchronized with GPS PPS (Pulse Per Second)
* @param ulAvgLatencyL3                         [L3] rTimestamp - tTimestamp = latency, synchronized with GPS PPS (Pulse Per Second)
* @param unTxDeviceId                           The unique ID (Serial Number) of the transfer device
* @param unRxVehicleSpeed                       Rx vehicle speed (Rx vehicle speed (if available), if not available, set the experimental value manually, default 60km/h)
* @param unTotalCommDevCnt                      The total number of devices that are currently simultaneously connected and transmitting/receiving (the count number of all device IDs currently received and stored by the Rx device)
* @param usRssi                                 Receive Signal Strength Indication, The total signal strength of the antenna at the time the message was received
* @param eRsvLevel                              Receive signal level, the level of the received signal strength of the mobile communication at the time the message is received (antenna signal)
* @param stRxPosition                           see DB_V2X_POSITION_RX_T
* @param ucErrIndicator                         If an error occurs in the sequence in the Tx Continuity Counter transmitted to Tx, an error is displayed (ok : 0, error : 1)
* @param ulTotalPacketCnt                       Total packets counts received from the same device
* @param ulTotalErrCnt                          The number of txContinuityCounter error packets among packets received from the same device
* @param unPdr                                  Packet Delivery Ratio (PDR)
* @param unPer                                  Packet Error Ratio (PER)
*/
typedef struct DB_V2X_STATUS_RX_t {
    uint64_t                                    ulRxTimeStampL1;
    uint64_t                                    ulRxTimeStampL2;
    uint64_t                                    ulRxTimeStampL3;
    uint64_t                                    ulAvgLatencyL1;
    uint64_t                                    ulAvgLatencyL2;
    uint64_t                                    ulAvgLatencyL3;
    uint32_t                                    unTxDeviceId;
    uint16_t                                    unRxVehicleSpeed;
    uint32_t                                    unTotalCommDevCnt;
    uint16_t                                    usRssi;
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
* @param ulTxTimeStampL1                        The system layer 1 time value at which the communication device transmitted the message (need to change to future UTF time, C-ITS communication time compatibility)
* @param ulTxTimeStampL2                        The system layer 2 time value at which the OBU/RSU/Control Units transmitted the message (Optional, if the devices are separated from model IP)
* @param ulTxTimeStampL2                        The system layer 3 time value at which the application device transmitted the message (Optional, if the devices are separated OBU/RSU)
* @param unRxDeviceId                           Unique Serial Number of the device to which you want to send the message; there can be more than one transfer destination
* @param eChannel                               Transmission channel number
* @param sPower                                 Transmission strength (dBm)
* @param eBandwidth                             Communication frequency bandwidth, Communication frequency bandwidth status at the time the message is received
* @param usTxRatio                              Tx packet trasmisstion ratio (frequency) (10ms, 100ms, etc)
* @param stTxPosition                           see DB_V2X_POSITION_TX_T
* @param unSeqNum                               sequence number (Sequential transmission of values ​​from 0 to N)
* @param unContCnt                              continuity counter Sequential transmission of values ​​from 0 to 100, The Rx device checks and stores the values ​​of 0 to 100 transmitted from Tx sequentially transmitted Tx Continuity Counter (sequence number)
* @param unTxVehicleSpeed                       Tx vehicle speed (Tx vehicle speed (if available), if not available, set the experimental value manually, default 60km/h)
*/
typedef struct DB_V2X_STATUS_TX_t {
    uint64_t                                    ulTxTimeStampL1;
    uint64_t                                    ulTxTimeStampL2;
    uint64_t                                    ulTxTimeStampL3;
    uint32_t                                    unRxDeviceId;
    DB_V2X_STATUS_CHANNEL_E                     eChannel;
    int16_t                                     sPower;
    DB_V2X_STATUS_BANDWIDTH_E                   eBandwidth;
    uint16_t                                    usTxRatio;
    DB_V2X_POSITION_TX_T                        stTxPosition;
    uint32_t                                    unSeqNum;
    uint32_t                                    unContCnt;
    uint32_t                                    unTxVehicleSpeed;
} __attribute__((__packed__)) DB_V2X_STATUS_TX_T;
/***************************** Function Protype ******************************/

#endif	/* _DB_V2X_STATUS_H_ */

