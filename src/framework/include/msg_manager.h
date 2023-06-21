#ifndef	_MSG_MANAGER_H_
#define	_MSG_MANAGER_H_

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
* @file msg_manager.h
*
* @note
*
* MSG Manager Header
*
******************************************************************************/


/***************************** Include ***************************************/
#include "type.h"
#include "config.h"
#include "db_v2x.h"

/***************************** Definition ************************************/
#define MSG_MANAGER_MAC_LENGTH          (6)

 /**
* @details V2X TX POWER
* @param MSG_MANAGER_V2X_TX_POWER       DSRC (0 ~ 23, Korea 20), C-V2X (-30~23)
*/
#define MSG_MANAGER_V2X_TX_POWER        (20)

#define MSG_MANAGER_V2X_TX_PROFILE_ID   (100)
#define MSG_MANAGER_V2X_TX_PEER_L2_ID   (0)

/***************************** Enum and Structure ****************************/

/**
* @details MSG_MANAGER_TX_PAYLOAD_TYPE_E
* @param MSG_MANAGER_TX_PAYLOAD_TYPE_RAW
* @param MSG_MANAGER_TX_PAYLOAD_TYPE_ENCODED_J2735   ASN.1 encoded J2735 Msg
* @param MSG_MANAGER_TX_PAYLOAD_TYPE_ITSK00130       ITSK-00130
* @param MSG_MANAGER_TX_PAYLOAD_TYPE_5GNRV2X         5G-NR-V2X data frame
* @param MSG_MANAGER_TX_PAYLOAD_TYPE_5GNRV2X_SS      5G-NR-V2X sensor sharing data frame
*/
typedef enum
{
	MSG_MANAGER_PAYLOAD_TYPE_RAW = 0,
	MSG_MANAGER_PAYLOAD_TYPE_ENCODED_J2735 = 1,
	MSG_MANAGER_PAYLOAD_TYPE_ITSK00130 = 2,
	MSG_MANAGER_PAYLOAD_TYPE_5GNRV2X = 3,
	MSG_MANAGER_PAYLOAD_TYPE_5GNRV2X_SS = 4,
	MSG_MANAGER_PAYLOAD_TYPE_MAX = 0xFF,
} MSG_MANAGER_PAYLOAD_TYPE_E;

typedef enum
{
    MSG_MANAGER_COMM_TYPE_UNKNOWN = 0,
    MSG_MANAGER_COMM_TYPE_DSRC,
    MSG_MANAGER_COMM_TYPE_LTEV2X,
    MSG_MANAGER_COMM_TYPE_5GNRV2X,
    MSG_MANAGER_COMM_TYPE_MAX = 0xFF
} MSG_MANAGER_COMM_TYPE_E;

typedef enum
{
	MSG_MANAGER_SIGN_ID_UNSECURED = 0,
	MSG_MANAGER_SIGN_ID_CERTIFICATE,
	MSG_MANAGER_SIGN_ID_DIGEST,
	MSG_MANAGER_SIGN_ID_ALTERNATE,
    MSG_MANAGER_SIGN_ID_MAX = 0xFF
} MSG_MANAGER_SIGN_ID_E;

typedef enum
{
	MSG_MANAGER_PRIORITY_CV2X_PPPP_MIN = 0,      /* CV2X */
	MSG_MANAGER_PRIORITY_CV2X_PPPP_0 = 0,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_1 = 1,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_2 = 2,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_3 = 3,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_4 = 4,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_5 = 5,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_6 = 6,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_7 = 7,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_MAX = 7,
	MSG_MANAGER_PRIORITY_CV2X_PPPP_INVALID = 8,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_MIN = 0, /* DSRC */
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_00 = 0,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_01 = 1,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_02 = 2,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_03 = 3,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_04 = 4,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_05 = 5,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_06 = 6,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_07 = 7,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_MAX = 7,
	MSG_MANAGER_PRIORITY_DSRC_USER_PRIO_INVALID = 8
} MSG_MANAGER_PRIORITY_E;

typedef enum
{
	MSG_MANAGER_V2X_FREQ_5860 = 5860, /* 172 ch */
	MSG_MANAGER_V2X_FREQ_5870 = 5870, /* 174 ch */
	MSG_MANAGER_V2X_FREQ_5880 = 5880, /* 176 ch */
	MSG_MANAGER_V2X_FREQ_5890 = 5890, /* 178 ch */
	MSG_MANAGER_V2X_FREQ_5900 = 5900, /* 180 ch */
	MSG_MANAGER_V2X_FREQ_5910 = 5910, /* 182 ch */
	MSG_MANAGER_V2X_FREQ_5920 = 5920, /* 184 ch */
	MSG_MANAGER_V2X_FREQ_Min = MSG_MANAGER_V2X_FREQ_5860,
	MSG_MANAGER_V2X_FREQ_Max = MSG_MANAGER_V2X_FREQ_5920
} MSG_MANAGER_V2X_FREQ_E;

typedef enum
{
	MSG_MANAGER_V2X_DATA_RATE_NA = 0,
	MSG_MANAGER_V2X_DATA_RATE_3MBPS = 0x06,
	MSG_MANAGER_V2X_DATA_RATE_4_5MBPS = 0x09,
	MSG_MANAGER_V2X_DATA_RATE_6MBPS = 0x0C,
	MSG_MANAGER_V2X_DATA_RATE_9MBPS = 0x12,
	MSG_MANAGER_V2X_DATA_RATE_12MBPS = 0x18,
	MSG_MANAGER_V2X_DATA_RATE_18MBPS = 0x24,
	MSG_MANAGER_V2X_DATA_RATE_24MBPS = 0x30,
	MSG_MANAGER_V2X_DATA_RATE_27MBPS = 0x36
} MSG_MANAGER_V2X_DATA_RATE_E;

/**
* @details MSG_MANAGER_V2X_TIME_SLOT_E (1609.3 - 7.3.2.2)
*/
typedef enum
{
	MSG_MANAGER_V2X_TIME_SLOT_0 = 0,
	MSG_MANAGER_V2X_TIME_SLOT_1,
	MSG_MANAGER_V2X_TIME_SLOT_CONTINUOUS,
	MSG_MANAGER_V2X_TIME_SLOT_MAX,
} MSG_MANAGER_V2X_TIME_SLOT_E;

/**
* @details MSG_MANAGER_TX_T
* @param ePayloadType
* @param eCommType
* @param eSignId
* @param eV2xFreq
* @param eV2xDataRate
* @param eV2xTimeSlot
* @param unPsid
* @param nTxPower
* @param unTxCount
* @param uchPeerMacAddr
* @param unTransmitterProfileId
* @param unPeerL2Id
* @param unReserved
*/
typedef struct MSG_MANAGER_TX_t {
    MSG_MANAGER_PAYLOAD_TYPE_E      ePayloadType;
    MSG_MANAGER_COMM_TYPE_E         eCommType;
    MSG_MANAGER_SIGN_ID_E           eSignId;
    MSG_MANAGER_V2X_FREQ_E          eV2xFreq;
    MSG_MANAGER_PRIORITY_E          ePriority;
    MSG_MANAGER_V2X_DATA_RATE_E     eV2xDataRate;
    MSG_MANAGER_V2X_TIME_SLOT_E     eV2xTimeSlot;
    uint32_t                        unPsid;
    int8_t                          nTxPower;
    uint32_t                        unTxCount;
    uint32_t                        unTxDelay;
    uint8_t                         uchPeerMacAddr[MSG_MANAGER_MAC_LENGTH];
    uint32_t                        unTransmitterProfileId;
    uint32_t                        unPeerL2Id;
    uint32_t                        unReserved;
} MSG_MANAGER_TX_T;

typedef struct MSG_MANAGER_RX_t {
    uint32_t                        unDelayTime;
    uint32_t                        unReserved;
} MSG_MANAGER_RX_T;

/**
* @details MSG_MANAGER_TX_EVENT_MSG_T
* @param pstMsgManagerTx
* @param pstDbV2x
* @param pPayload
*/
typedef struct MSG_MANAGER_TX_EVENT_MSG_t {
    MSG_MANAGER_TX_T                *pstMsgManagerTx;
    DB_V2X_T                        *pstDbV2x;
    void                            *pPayload;
} MSG_MANAGER_TX_EVENT_MSG_T;

/**
* @details MSG_MANAGER_RX_EVENT_MSG_T
* @param pstMsgManagerRx
* @param pstDbV2x
* @param pPayload
*/
typedef struct MSG_MANAGER_RX_EVENT_MSG_t {
    MSG_MANAGER_RX_T                *pstMsgManagerRx;
    DB_V2X_T                        *pstDbV2x;
    void                            *pPayload;
} MSG_MANAGER_RX_EVENT_MSG_T;

typedef struct MSG_MANAGER_t {
    char                            *pchIfaceName;
    bool                            bLogLevel;
    uint32_t                        unReserved;
} MSG_MANAGER_T;

/***************************** Function Protype ******************************/

int32_t MSG_MANAGER_Transmit(MSG_MANAGER_TX_T *pstMsgMgrTx, DB_V2X_T *pstDbV2x, void *pPayload);
int32_t MSG_MANAGER_Receive(MSG_MANAGER_RX_T *pstMsgMgrRx, DB_V2X_T *pstDbV2x, void *pPayload);

int32_t MSG_MANAGER_SetLog(MSG_MANAGER_T *pstMsgManager);

int32_t MSG_MANAGER_Open(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Close(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Start(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Stop(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_Status(MSG_MANAGER_T *pstMsgManager);

int32_t MSG_MANAGER_Init(MSG_MANAGER_T *pstMsgManager);
int32_t MSG_MANAGER_DeInit(MSG_MANAGER_T *pstMsgManager);

#endif	/* _MSG_MANAGER_H_ */

