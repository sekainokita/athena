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
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "framework.h"
#include "msg_manager.h"
#include "db_manager.h"
#include "time_manager.h"

#include "v2x_defs.h"
#include "v2x_ext_type.h"

#include "cli_util.h"

/***************************** Definition ************************************/
#define SAMPLE_V2X_API_VER 0x0001
#define SAMPLE_V2X_IP_ADDR "192.168.1.11"
#define SAMPLE_V2X_MSG_LEN 100
#define SAMPLE_V2X_PORT_ADDR 47347

//#define CONFIG_TEMP_OBU_TEST (1)

#ifdef WORDS_BIGENDIAN
#define htonll(x)   (x)
#define ntohll(x)   (x)
#else
#define htonll(x)   ((((uint64_t)htonl(x)) << 32) + htonl(x >> 32))
#define ntohll(x)   ((((uint64_t)ntohl(x)) << 32) + ntohl(x >> 32))
#endif

/***************************** Static Variable *******************************/
static int32_t s_nSocketHandle = -1;
static int s_nDbTaskMsgId, s_nMsgTxTaskMsgId, s_nMsgRxTaskMsgId;
static key_t s_dbTaskMsgKey = FRAMEWORK_DB_TASK_MSG_KEY;
static key_t s_MsgTxTaskMsgKey = FRAMEWORK_MSG_TX_TASK_MSG_KEY;
static key_t s_MsgRxTaskMsgKey = FRAMEWORK_MSG_RX_TASK_MSG_KEY;

static pthread_t sh_msgMgrTxTask;
static pthread_t sh_msgMgrRxTask;

static bool s_bMsgMgrLog = OFF;
static bool s_bFirstPacket = FALSE;

/***************************** Function  *************************************/

/////////////////////////////////////////////////////////////////////////////////////////
/* Global Variable Value */
V2xAction_t e_action_g = eV2xAction_ADD;
V2xPayloadType_t e_payload_type_g = eRaw;
V2xPsid_t psid_g = 5271;
V2XCommType_t e_comm_type_g = eV2XCommType_5GNRV2X;
V2xPowerDbm_t tx_power_g = 20;
V2xSignerId_t e_signer_id_g = eV2xSignerId_UNSECURED;
V2xMsgPriority_t e_priority_g = eV2xPriority_CV2X_PPPP_0;
uint32_t tx_cnt_g = 100;
uint32_t tx_delay_g = 100;
V2xFrequency_t freq_g = 5900;
V2xDataRate_t e_data_rate_g = eV2xDataRate_6MBPS;
V2xTimeSlot_t e_time_slot_g = eV2xTimeSlot_Continuous;
uint8_t peer_mac_addr_g[MAC_EUI48_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t transmitter_profile_id_g = 100;
uint32_t peer_l2id_g = 0;

#if defined(CONFIG_TEMP_OBU_TEST)
uint32_t delay_time_sec_g = 10;
#endif

static void P_MSG_NABAGER_PrintMsgInfo(int msqid)
{

    struct msqid_ds m_stat;

    PrintDebug("========== Messege Queue Infomation =============");

    if(msgctl(msqid, IPC_STAT, &m_stat) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgctl() is failed!!");
    }

    PrintDebug("msg_lspid : %d", m_stat.msg_lspid);
    PrintDebug("msg_qnum : %ld", m_stat.msg_qnum);
    PrintDebug("msg_stime : %ld", m_stat.msg_stime);

    PrintDebug("=================================================");
}

static int32_t P_MSG_MANAGER_ConnectV2XDevice(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;
    int32_t nSocketHandle = -1;
    int32_t nFlags = 0;

    nSocketHandle = socket(AF_INET, SOCK_STREAM, 0);
    if (nSocketHandle < 0)
    {
        PrintError("socket() is failed!!");
        nRet = FRAMEWORK_ERROR;
        return nRet;
    }

    PrintDebug("pstMsgManager->pchIfaceName[%s]", pstMsgManager->pchIfaceName);

    nRet = setsockopt(nSocketHandle, SOL_SOCKET, SO_BINDTODEVICE, pstMsgManager->pchIfaceName, strlen(pstMsgManager->pchIfaceName));
    if (nRet < 0)
    {
        PrintError("setsockopt() is failed");
        return nRet;
    }

    struct sockaddr_in server_addr =
    {
        .sin_family = AF_INET,
        .sin_addr.s_addr = inet_addr(SAMPLE_V2X_IP_ADDR),
        .sin_port = htons(SAMPLE_V2X_PORT_ADDR)
    };

    nRet = connect(nSocketHandle, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (nRet < 0)
    {
        PrintError("connect() is failed");
        return nRet;
    }

    /* Change to NON-BLOCK socket */
    nFlags = fcntl(nSocketHandle, F_GETFL, 0);
    if (nFlags == -1)
    {
        PrintError("fcntl() is F_GETFL failed");
        return nRet;
    }

    nFlags |= O_NONBLOCK;
    nRet = fcntl(nSocketHandle, F_SETFL, nFlags);
    if (nRet < 0)
    {
        PrintError("fcntl() is F_SETFL failed");
        return nRet;
    }

    s_nSocketHandle = nSocketHandle;

    PrintTrace("Connection of V2X Device is successed! [s_nSocketHandle:0x%x]", s_nSocketHandle);

    nRet = FRAMEWORK_OK;

    return nRet;
}

static int32_t P_MSG_MANAGER_DisconnectV2XDevice(void)
{
    int32_t nRet = FRAMEWORK_ERROR;

	if (s_nSocketHandle >= 0)
	{
		close(s_nSocketHandle);
        s_nSocketHandle = -1;
        PrintTrace("s_nSocketHandle is closed, s_nSocketHandle[%d]", s_nSocketHandle);
        nRet = FRAMEWORK_OK;
	}
    else
    {
        PrintError("s_nSocketHandle is not available!!");
    }

    return nRet;
}

int32_t P_MSG_MANAGER_SetV2xWsrSetting(void)
{
	int32_t nRet = FRAMEWORK_ERROR;

	// Prepare the Ext_WSReq_t structure
	Ext_WSReq_t ws_req;
	memset(&ws_req, 0, sizeof(ws_req));
	ws_req.magic_num = htons(MAGIC_WSREQ);
	ws_req.ver = htons(SAMPLE_V2X_API_VER);
	ws_req.e_action = eV2xAction_ADD;
	ws_req.e_payload_type = e_payload_type_g;
	ws_req.psid = htonl(psid_g);

	printf("\nWSM Service REQ>>\n"
		   "  magic_num        : 0x%04X\n"
		   "  ver              : 0x%04X\n"
		   "  e_action         : %d\n"
		   "  e_payload_type   : %d\n"
		   "  psid             : %u\n",
		   ntohs(ws_req.magic_num),
		   ntohs(ws_req.ver),
		   ws_req.e_action,
		   ws_req.e_payload_type,
		   ntohl(ws_req.psid));

	// Send the request
	ssize_t n = send(s_nSocketHandle, &ws_req, sizeof(ws_req), 0);
	if (n < 0)
	{
		PrintError("send() is failed!!");
		return nRet;
	}
	else if (n != sizeof(ws_req))
	{
		PrintError("send() sent a different number of bytes than expected!");
		return nRet;
	}

	// Wait for the response
	Ext_WSResp_t ws_resp;
	memset(&ws_resp, 0, sizeof(ws_resp));
	n = -1;

	while (n <= 0)
	{
		n = recv(s_nSocketHandle, &ws_resp, sizeof(ws_resp), 0);
		if (n < 0)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			{
				PrintError("recv() is failed!!");
				break;
			}
		}
		else if (n == 0)
		{
			PrintError("recv()'connection is closed by peer!!");
		}
		else if (n != sizeof(ws_resp))
		{
			PrintError("recv() is received a different number of bytes than expected!!");
		}
        else
        {
            nRet = FRAMEWORK_OK;
            PrintTrace("recv() is success to get ws_resp");
        }

		usleep(1000);
	}

	PrintDebug("\nWSM Service RESP>>\n"
		   "  magic_num      : 0x%04X\n"
		   "  ver            : 0x%04X\n"
		   "  e_action       : %d\n"
		   "  is_confirmed   : %d\n"
		   "  psid           : %u\n",
		   ntohs(ws_resp.magic_num),
		   ntohs(ws_resp.ver),
		   ws_resp.e_action,
		   ws_resp.is_confirmed,
		   ntohl(ws_resp.psid));

	return nRet;
}

static int32_t P_MSG_MANAGER_SendTxMsgToDbMgr(MSG_MANAGER_TX_EVENT_MSG_T *pstEventMsg, uint32_t unCrc32)
{
    int32_t nRet = FRAMEWORK_ERROR;
    DB_MANAGER_WRITE_T stDbManagerWrite;
    DB_MANAGER_EVENT_MSG_T stEventMsg;
    DB_MANAGER_T *pstDbManager;

    (void*)memset(&stDbManagerWrite, 0x00, sizeof(DB_MANAGER_WRITE_T));

    pstDbManager = FRAMEWORK_GetDbManagerInstance();
    if(pstDbManager == NULL)
    {
        PrintError("FRAMEWORK_GetDbManagerInstance() is failed!! pstDbManager is NULL");
        return nRet;
    }

    stDbManagerWrite.eFileType = pstDbManager->eFileType;
    stDbManagerWrite.eCommMsgType = DB_MANAGER_COMM_MSG_TYPE_TX;
    stDbManagerWrite.eProc = DB_MANAGER_PROC_WRITE;
    stDbManagerWrite.unCrc32 = unCrc32;

    stEventMsg.pstDbManagerWrite = &stDbManagerWrite;
    stEventMsg.pstDbV2x = pstEventMsg->pstDbV2x;

    /* free at P_DB_MANAGER_WriteXXX() */
    stEventMsg.pPayload = malloc(pstEventMsg->pstDbV2x->ulPayloadLength);
    if(stEventMsg.pPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(stEventMsg.pPayload, pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

    if(msgsnd(s_nDbTaskMsgId, &stEventMsg, sizeof(DB_MANAGER_EVENT_MSG_T), IPC_NOWAIT) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgsnd() is failed!!");
        return nRet;
    }
    else
    {
        nRet = FRAMEWORK_OK;
    }

	return nRet;
}

static int32_t P_MSG_MANAGER_SendTxMsg(MSG_MANAGER_TX_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    uint32_t unDbV2xPacketLength = sizeof(DB_V2X_T) + pstEventMsg->pstDbV2x->ulPayloadLength + sizeof(pstEventMsg->pstDbV2x->ulReserved);
    uint32_t unDbV2xCrcCalcuatedLength = sizeof(DB_V2X_T) + pstEventMsg->pstDbV2x->ulPayloadLength;
    uint32_t unV2xTxPduLength = sizeof(Ext_V2X_TxPDU_t) + unDbV2xPacketLength;
    ssize_t nRetSendSize = 0;
    uint32_t ulTempDbV2xTotalPacketCrc32 = 0, ulDbV2xTotalPacketCrc32 = 0;
    TIME_MANAGER_T *pstTimeManager;

    Ext_V2X_TxPDU_t *pstV2xTxPdu = NULL;
    DB_V2X_T *pstDbV2x = NULL;
    uint8_t *pchDbV2xCrc = NULL;

    pstV2xTxPdu = malloc(unV2xTxPduLength);
    if(pstV2xTxPdu == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memset(pstV2xTxPdu, 0, sizeof(Ext_V2X_TxPDU_t));

    pstV2xTxPdu->ver = htons(SAMPLE_V2X_API_VER);
    pstV2xTxPdu->e_payload_type = e_payload_type_g;
    pstV2xTxPdu->psid = htonl(psid_g);
    pstV2xTxPdu->tx_power = tx_power_g;
    pstV2xTxPdu->e_signer_id = e_signer_id_g;
    pstV2xTxPdu->e_priority = e_priority_g;

    if ((e_comm_type_g == eV2XCommType_LTEV2X) || (e_comm_type_g == eV2XCommType_5GNRV2X))
    {
        pstV2xTxPdu->magic_num = htons(MAGIC_CV2X_TX_PDU);
        pstV2xTxPdu->u.config_cv2x.transmitter_profile_id = htonl(transmitter_profile_id_g);
        pstV2xTxPdu->u.config_cv2x.peer_l2id = htonl(peer_l2id_g);
    }
    else if (e_comm_type_g == eV2XCommType_DSRC)
    {
        pstV2xTxPdu->magic_num = htons(MAGIC_DSRC_TX_PDU);
        pstV2xTxPdu->u.config_wave.freq = htons(freq_g);
        pstV2xTxPdu->u.config_wave.e_data_rate = htons(e_data_rate_g);
        pstV2xTxPdu->u.config_wave.e_time_slot = e_time_slot_g;
        memcpy(pstV2xTxPdu->u.config_wave.peer_mac_addr, peer_mac_addr_g, MAC_EUI48_LEN);
    }

    pstV2xTxPdu->v2x_msg.length = htons(unDbV2xPacketLength);

    pstDbV2x = malloc(unDbV2xPacketLength);
    if(pstDbV2x == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memset(pstDbV2x, 0, unDbV2xPacketLength);

    pstDbV2x->eDeviceType = htons(pstEventMsg->pstDbV2x->eDeviceType);
    pstDbV2x->eTeleCommType = htons(pstEventMsg->pstDbV2x->eTeleCommType);
    pstDbV2x->unDeviceId = htonl(pstEventMsg->pstDbV2x->unDeviceId);
    pstDbV2x->ulTimeStamp = htonll(pstEventMsg->pstDbV2x->ulTimeStamp);
    pstDbV2x->eServiceId = htons(pstEventMsg->pstDbV2x->eServiceId);
    pstDbV2x->eActionType = htons(pstEventMsg->pstDbV2x->eActionType);
    pstDbV2x->eRegionId = htons(pstEventMsg->pstDbV2x->eRegionId);
    pstDbV2x->ePayloadType = htons(pstEventMsg->pstDbV2x->ePayloadType);
    pstDbV2x->eCommId = htons(pstEventMsg->pstDbV2x->eCommId);
    pstDbV2x->usDbVer = htons(pstEventMsg->pstDbV2x->usDbVer);
    pstDbV2x->usHwVer = htons(pstEventMsg->pstDbV2x->usHwVer);
    pstDbV2x->usSwVer = htons(pstEventMsg->pstDbV2x->usSwVer);
    pstDbV2x->ulPayloadLength = htonl(pstEventMsg->pstDbV2x->ulPayloadLength);
    pstDbV2x->ulReserved = htonl(pstEventMsg->pstDbV2x->ulReserved);

    pchDbV2xCrc = malloc(unDbV2xPacketLength);
    if(pchDbV2xCrc == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }
    memset(pchDbV2xCrc, 0, unDbV2xCrcCalcuatedLength);
    memcpy(pchDbV2xCrc, pstDbV2x, unDbV2xPacketLength);
    memcpy(pchDbV2xCrc + sizeof(DB_V2X_T), pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

    ulTempDbV2xTotalPacketCrc32 = CLI_UTIL_GetCrc32((uint8_t*)pchDbV2xCrc, sizeof(DB_V2X_T) + pstEventMsg->pstDbV2x->ulPayloadLength);
    ulDbV2xTotalPacketCrc32 = htonl(ulTempDbV2xTotalPacketCrc32);

    memcpy(pstV2xTxPdu->v2x_msg.data, pstDbV2x, unDbV2xPacketLength);
    memcpy(pstV2xTxPdu->v2x_msg.data + sizeof(DB_V2X_T), pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);
    memcpy(pstV2xTxPdu->v2x_msg.data + sizeof(DB_V2X_T) + pstEventMsg->pstDbV2x->ulPayloadLength, &ulDbV2xTotalPacketCrc32, sizeof(uint32_t));

    /* free the allocated payload */
    if(pstEventMsg->pPayload != NULL)
    {
        if (pstEventMsg->pstDbV2x->ePayloadType != DB_V2X_PAYLOAD_TYPE_PLATOONING_THROUGHPUT)
        {
            free(pstEventMsg->pPayload);
        }
    }

    if(s_bMsgMgrLog == ON)
    {
        pstTimeManager = FRAMEWORK_GetTimeManagerInstance();
        if(pstTimeManager == NULL)
        {
            PrintError("pstTimeManager is NULL!");
        }

        nRet = TIME_MANAGER_Get(pstTimeManager);
        if(nRet != FRAMEWORK_OK)
        {
            PrintError("TIME_MANAGER_Get() is failed! [nRet:%d]", nRet);
        }
        else
        {
            /* The average delay between svc and send() is about 10 us, so use the timestamp of svc */
            PrintDebug("[%ld]-[%ld]=[%ld]", pstTimeManager->ulTimeStamp, pstEventMsg->pstDbV2x->ulTimeStamp, pstTimeManager->ulTimeStamp-pstEventMsg->pstDbV2x->ulTimeStamp);
        }

        printf("\nV2X TX PDU>>\n"
        "  magic_num        : 0x%04X\n"
        "  ver              : 0x%04X\n"
        "  e_payload_type   : %d\n"
        "  psid             : %u\n"
        "  tx_power         : %d\n"
        "  e_signer_id      : %d\n"
        "  e_priority       : %d\n",
        ntohs(pstV2xTxPdu->magic_num),
        ntohs(pstV2xTxPdu->ver),
        pstV2xTxPdu->e_payload_type,
        ntohl(pstV2xTxPdu->psid),
        pstV2xTxPdu->tx_power,
        pstV2xTxPdu->e_signer_id,
        pstV2xTxPdu->e_priority);

        if (e_comm_type_g == eV2XCommType_LTEV2X || e_comm_type_g == eV2XCommType_5GNRV2X)
        {
            printf("  u.config_cv2x.transmitter_profile_id : %u\n"
            "  u.config_cv2x.peer_l2id              : %u\n",
            ntohl(pstV2xTxPdu->u.config_cv2x.transmitter_profile_id),
            ntohl(pstV2xTxPdu->u.config_cv2x.peer_l2id));
        }
        else if (e_comm_type_g == eV2XCommType_DSRC)
        {
            printf("  u.config_wave.freq                  : %d\n"
            "  u.config_wave.e_data_rate           : %d\n"
            "  u.config_wave.e_time_slot           : %d\n"
            "  u.config_wave.peer_mac_addr         : %s\n",
            ntohs(pstV2xTxPdu->u.config_wave.freq),
            ntohs(pstV2xTxPdu->u.config_wave.e_data_rate),
            pstV2xTxPdu->u.config_wave.e_time_slot,
            pstV2xTxPdu->u.config_wave.peer_mac_addr);
        }
    }

    nRetSendSize = send(s_nSocketHandle, pstV2xTxPdu, unV2xTxPduLength, 0);
    if (nRetSendSize < 0)
    {
        PrintError("send() is failed!!");
        nRet = FRAMEWORK_ERROR;
        return nRet;
    }
    else if (nRetSendSize != unV2xTxPduLength)
    {
        PrintError("send() sent a different number of bytes than expected!!");
        nRet = FRAMEWORK_ERROR;
        return nRet;
    }
    else
    {
        if(s_bMsgMgrLog == ON)
        {
            PrintDebug("tx send success (%ld bytes)", nRetSendSize);
        }
    }

    nRet = P_MSG_MANAGER_SendTxMsgToDbMgr(pstEventMsg, ntohl(ulDbV2xTotalPacketCrc32));
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_SendTxMsgToDbMgr() is faild! [nRet:%d]", nRet);
        return nRet;
    }

    if(pstV2xTxPdu != NULL)
    {
        free(pstV2xTxPdu);
    }

    if(pstDbV2x != NULL)
    {
        free(pstDbV2x);
    }

    if(pchDbV2xCrc != NULL)
    {
        free(pchDbV2xCrc);
    }

    return nRet;
}

static int32_t P_MSG_MANAGER_SendRxMsgToDbMgr(MSG_MANAGER_RX_EVENT_MSG_T *pstEventMsg, uint32_t unCrc32)
{
    int32_t nRet = FRAMEWORK_ERROR;
    DB_MANAGER_WRITE_T stDbManagerWrite;
    DB_MANAGER_EVENT_MSG_T stEventMsg;
    DB_MANAGER_T *pstDbManager;

    (void*)memset(&stDbManagerWrite, 0x00, sizeof(DB_MANAGER_WRITE_T));

    pstDbManager = FRAMEWORK_GetDbManagerInstance();
    if(pstDbManager == NULL)
    {
        PrintError("FRAMEWORK_GetDbManagerInstance() is failed!! pstDbManager is NULL");
        return nRet;
    }

    stDbManagerWrite.eFileType = pstDbManager->eFileType;
    stDbManagerWrite.eCommMsgType = DB_MANAGER_COMM_MSG_TYPE_RX;
    stDbManagerWrite.eProc = DB_MANAGER_PROC_WRITE;
    stDbManagerWrite.unCrc32 = unCrc32;

    stEventMsg.pstDbManagerWrite = &stDbManagerWrite;
    stEventMsg.pstDbV2x = pstEventMsg->pstDbV2x;

    /* free at P_DB_MANAGER_WriteXXX() */
    stEventMsg.pPayload = malloc(pstEventMsg->pstDbV2x->ulPayloadLength);
    if(stEventMsg.pPayload == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return nRet;
    }

    memcpy(stEventMsg.pPayload, pstEventMsg->pPayload, pstEventMsg->pstDbV2x->ulPayloadLength);

    if(msgsnd(s_nDbTaskMsgId, &stEventMsg, sizeof(DB_MANAGER_EVENT_MSG_T), IPC_NOWAIT) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgsnd() is failed!!");
        return nRet;
    }
    else
    {
        nRet = FRAMEWORK_OK;
    }

	return nRet;
}

static int32_t P_MSG_MANAGER_ReceiveRxMsg(MSG_MANAGER_RX_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    uint8_t buf[4096] = {0};
    int nRecvLen = -1;
    Ext_V2X_RxPDU_t *pstV2xRxPdu = NULL;
    DB_V2X_T *pstDbV2x = NULL;
    uint32_t ulDbV2xTotalPacketCrc32 = 0, ulCompDbV2xTotalPacketCrc32 = 0, ulTempDbV2xTotalPacketCrc32 = 0;
    DB_MANAGER_V2X_STATUS_T stDbV2xStatus;

    while (1)
    {
        nRecvLen = recv(s_nSocketHandle, buf, sizeof(buf), 0);
        if (nRecvLen < 0)
        {
            if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
            {
                if(s_bMsgMgrLog == ON)
                {
                    PrintError("recv() is failed!!");
                }

                usleep(1000*1000);
            }
            else
            {
                usleep(10*1000);
                continue;
            }
        }
        else if (nRecvLen == 0)
        {
            PrintError("recv()'s connection is closed by peer!!");
            break;
        }
        else
        {
            if(s_bMsgMgrLog == ON)
            {
                PrintDebug("recv() is success, nRecvLen[%u]", nRecvLen);
            }

            pstV2xRxPdu = malloc(nRecvLen);
            if(pstV2xRxPdu == NULL)
            {
                PrintError("malloc() is failed! [NULL]");
            }
            else
            {
                memset(pstV2xRxPdu, 0, sizeof(Ext_V2X_RxPDU_t));
                memcpy(pstV2xRxPdu, buf, nRecvLen);

                pstDbV2x = malloc(pstV2xRxPdu->v2x_msg.length);
                if(pstDbV2x == NULL)
                {
                    PrintError("malloc() is failed! [NULL]");
                }
                else
                {
                    memset(pstDbV2x, 0, pstV2xRxPdu->v2x_msg.length);
                    memcpy(pstDbV2x, pstV2xRxPdu->v2x_msg.data, sizeof(DB_V2X_T));

                    pstEventMsg->pPayload = malloc(pstEventMsg->pstDbV2x->ulPayloadLength);
                    if(pstEventMsg->pPayload == NULL)
                    {
                        PrintError("malloc() is failed! [NULL]");
                    }
                    else
                    {
                        memcpy(pstEventMsg->pPayload, pstV2xRxPdu->v2x_msg.data + sizeof(DB_V2X_T), pstEventMsg->pstDbV2x->ulPayloadLength);

                        memcpy(&ulTempDbV2xTotalPacketCrc32, pstV2xRxPdu->v2x_msg.data + sizeof(DB_V2X_T) + pstEventMsg->pstDbV2x->ulPayloadLength, sizeof(uint32_t));
                        ulDbV2xTotalPacketCrc32 = ntohl(ulTempDbV2xTotalPacketCrc32);

                        ulCompDbV2xTotalPacketCrc32 = CLI_UTIL_GetCrc32((uint8_t*)&pstV2xRxPdu->v2x_msg.data[0], sizeof(DB_V2X_T) + pstEventMsg->pstDbV2x->ulPayloadLength);
                        if(ulDbV2xTotalPacketCrc32 != ulCompDbV2xTotalPacketCrc32)
                        {
                            if(s_bFirstPacket == TRUE)
                            {
                                PrintWarn("Successfully received the first packet from the other device, set s_bFirstPacket [%d]", s_bFirstPacket);
                            }
                            else
                            {
                                PrintError("CRC32 does not matched!! check Get:ulDbV2xTotalPacketCrc32[0x%x] != Calculate:ulCompDbV2xTotalPacketCrc32[0x%x]", ulDbV2xTotalPacketCrc32, ulCompDbV2xTotalPacketCrc32);
                                nRet = DB_MANAGER_GetV2xStatus(&stDbV2xStatus);
                                if(nRet != FRAMEWORK_OK)
                                {
                                    PrintError("DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
                                }

                                stDbV2xStatus.stV2xStatusRx.ucErrIndicator = TRUE;
                                stDbV2xStatus.stV2xStatusRx.ulTotalErrCnt++;
                                stDbV2xStatus.bCrc32ErrPkt = TRUE;
                                PrintWarn("increase ulTotalErrCnt [from %ld to %ld], [bCrc32ErrPkt:%d]", (stDbV2xStatus.stV2xStatusRx.ulTotalErrCnt-1), stDbV2xStatus.stV2xStatusRx.ulTotalErrCnt, stDbV2xStatus.bCrc32ErrPkt);

                                nRet = DB_MANAGER_SetV2xStatus(&stDbV2xStatus);
                                if(nRet != FRAMEWORK_OK)
                                {
                                    PrintError("DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
                                }
                            }
                        }

                        if(s_bFirstPacket == TRUE)
                        {
                            s_bFirstPacket = FALSE;
                        }
                        else
                        {
                            nRet = DB_MANAGER_GetV2xStatus(&stDbV2xStatus);
                            if(nRet != FRAMEWORK_OK)
                            {
                                PrintError("DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
                            }

                            stDbV2xStatus.stV2xStatusRx.ulTotalPacketCnt++;

                            nRet = DB_MANAGER_SetV2xStatus(&stDbV2xStatus);
                            if(nRet != FRAMEWORK_OK)
                            {
                                PrintError("DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
                            }
                        }

                        if(s_bMsgMgrLog == ON)
                        {
                            printf("\nV2X RX PDU>>\n"
                            "  magic_num        : 0x%04X\n"
                            "  ver              : 0x%04X\n"
                            "  e_v2x_comm_type   : %d\n"
                            "  e_payload_type   : %d\n"
                            "  psid             : %u\n"
                            "  freq             : %d\n"
                            "  rssi             : %d\n"
                            "  reserved1        : %d\n"
                            "  peer_l2id        : %d\n"
                            "  crc              : %d\n"
                            "  v2x_msg.length   : %d\n",
                            ntohs(pstV2xRxPdu->magic_num),
                            ntohs(pstV2xRxPdu->ver),
                            pstV2xRxPdu->e_v2x_comm_type,
                            pstV2xRxPdu->e_payload_type,
                            ntohl(pstV2xRxPdu->psid),
                            pstV2xRxPdu->freq,
                            pstV2xRxPdu->rssi,
                            pstV2xRxPdu->reserved1,
                            pstV2xRxPdu->u.peer_l2id,
                            pstV2xRxPdu->crc,
                            ntohs(pstV2xRxPdu->v2x_msg.length));

                            PrintDebug("db_v2x_tmp_p->eDeviceType[%d]", ntohs(pstDbV2x->eDeviceType));
                            PrintDebug("db_v2x_tmp_p->eTeleCommType[%d]", ntohs(pstDbV2x->eTeleCommType));
                            PrintDebug("db_v2x_tmp_p->unDeviceId[0x%x]", ntohl(pstDbV2x->unDeviceId));
                            PrintDebug("db_v2x_tmp_p->ulTimeStamp[%ld]", ntohll(pstDbV2x->ulTimeStamp));
                            PrintDebug("db_v2x_tmp_p->eServiceId[%d]", ntohs(pstDbV2x->eServiceId));
                            PrintDebug("db_v2x_tmp_p->eActionType[%d]", ntohs(pstDbV2x->eActionType));
                            PrintDebug("db_v2x_tmp_p->eRegionId[%d]", ntohs(pstDbV2x->eRegionId));
                            PrintDebug("db_v2x_tmp_p->ePayloadType[%d]", ntohs(pstDbV2x->ePayloadType));
                            PrintDebug("db_v2x_tmp_p->eCommId[%d]", ntohs(pstDbV2x->eCommId));
                            PrintDebug("db_v2x_tmp_p->usDbVer[%d.%d]", ntohs(pstDbV2x->usDbVer) >> CLI_DB_V2X_MAJOR_SHIFT, ntohs(pstDbV2x->usDbVer) & CLI_DB_V2X_MINOR_MASK);
                            PrintDebug("db_v2x_tmp_p->usHwVer[%d]", ntohs(pstDbV2x->usHwVer));
                            PrintDebug("db_v2x_tmp_p->usSwVer[%d]", ntohs(pstDbV2x->usSwVer));
                            PrintDebug("db_v2x_tmp_p->ulPayloadLength[%d]", ntohl(pstDbV2x->ulPayloadLength));
                            PrintDebug("db_v2x_tmp_p->ulReserved[0x%x]", ntohl(pstDbV2x->ulReserved));

                            PrintDebug("received CRC:ulDbV2xTotalPacketCrc32[0x%x]", ulDbV2xTotalPacketCrc32);
                            PrintDebug("calcuated CRC:ulCompDbV2xTotalPacketCrc32[0x%x]", ulCompDbV2xTotalPacketCrc32);

                            if(ulDbV2xTotalPacketCrc32 == ulCompDbV2xTotalPacketCrc32)
                            {
                                PrintTrace("CRC32 is matched!");
                            }
                        }

                        if(pstEventMsg == NULL)
                        {
                            PrintError("pstEventMsg is NULL");
                        }
                        else
                        {
                            pstEventMsg->pstDbV2x->eDeviceType = ntohs(pstDbV2x->eDeviceType);
                            pstEventMsg->pstDbV2x->eTeleCommType = ntohs(pstDbV2x->eTeleCommType);
                            pstEventMsg->pstDbV2x->unDeviceId = ntohl(pstDbV2x->unDeviceId);
                            pstEventMsg->pstDbV2x->ulTimeStamp = ntohll(pstDbV2x->ulTimeStamp);
                            pstEventMsg->pstDbV2x->eServiceId = ntohs(pstDbV2x->eServiceId);
                            pstEventMsg->pstDbV2x->eActionType = ntohs(pstDbV2x->eActionType);
                            pstEventMsg->pstDbV2x->eRegionId = ntohs(pstDbV2x->eRegionId);
                            pstEventMsg->pstDbV2x->ePayloadType = ntohs(pstDbV2x->ePayloadType);
                            pstEventMsg->pstDbV2x->eCommId = ntohs(pstDbV2x->eCommId);
                            pstEventMsg->pstDbV2x->usDbVer = ntohs(pstDbV2x->usDbVer);
                            pstEventMsg->pstDbV2x->usHwVer = ntohs(pstDbV2x->usHwVer);
                            pstEventMsg->pstDbV2x->usSwVer = ntohs(pstDbV2x->usSwVer);
                            pstEventMsg->pstDbV2x->ulPayloadLength = ntohl(pstDbV2x->ulPayloadLength);
                            pstEventMsg->pstDbV2x->ulReserved = ntohl(pstDbV2x->ulReserved);

                            nRet = DB_MANAGER_GetV2xStatus(&stDbV2xStatus);
                            if(nRet != FRAMEWORK_OK)
                            {
                                PrintError("DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
                            }

                            stDbV2xStatus.ulTxTimeStamp = pstEventMsg->pstDbV2x->ulTimeStamp;

                            nRet = DB_MANAGER_SetV2xStatus(&stDbV2xStatus);
                            if(nRet != FRAMEWORK_OK)
                            {
                                PrintError("DB_MANAGER_GetV2xStatus() is failed! [nRet:%d]", nRet);
                            }

                            nRet = P_MSG_MANAGER_SendRxMsgToDbMgr(pstEventMsg, ulDbV2xTotalPacketCrc32);
                            if (nRet != FRAMEWORK_OK)
                            {
                                PrintError("P_MSG_MANAGER_SendTxMsgToDbMgr() is faild! [nRet:%d]", nRet);
                            }
                        }
                    }
                }
            }

            if(pstV2xRxPdu != NULL)
            {
                free(pstV2xRxPdu);
            }

            if(pstDbV2x != NULL)
            {
                free(pstDbV2x);
            }

            if(pstEventMsg->pPayload != NULL)
            {
                free(pstEventMsg->pPayload);
            }
        }
    }

    return nRet;
}

static void *P_MSG_MANAGER_TxTask(void *arg)
{
    MSG_MANAGER_TX_EVENT_MSG_T stEventMsg;
    int32_t nRet = FRAMEWORK_ERROR;

    UNUSED(arg);

    memset(&stEventMsg, 0, sizeof(MSG_MANAGER_TX_EVENT_MSG_T));

    while (1)
    {
        if(msgrcv(s_nMsgTxTaskMsgId, &stEventMsg, sizeof(MSG_MANAGER_TX_EVENT_MSG_T), 0, MSG_NOERROR) == FRAMEWORK_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            nRet = P_MSG_MANAGER_SendTxMsg(&stEventMsg);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("P_MSG_MANAGER_SendTxMsg() is faild! [nRet:%d]", nRet);
            }
        }
    }

    return NULL;
}


static void *P_MSG_MANAGER_RxTask(void *arg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    MSG_MANAGER_RX_EVENT_MSG_T stEventMsg;
    MSG_MANAGER_RX_T           stMsgManagerRx;
    DB_V2X_T                   stDbV2x;

    UNUSED(arg);

    (void*)memset(&stEventMsg, 0x00, sizeof(MSG_MANAGER_RX_EVENT_MSG_T));
    (void*)memset(&stMsgManagerRx, 0x00, sizeof(MSG_MANAGER_RX_T));
    (void*)memset(&stDbV2x, 0x00, sizeof(DB_V2X_T));

    stEventMsg.pstMsgManagerRx = &stMsgManagerRx;
    stEventMsg.pstDbV2x = &stDbV2x;

    nRet = P_MSG_MANAGER_ReceiveRxMsg(&stEventMsg);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_ReceiveRxMsg() is faild! [nRet:%d]", nRet);
    }

    return NULL;
}

#if defined(CONFIG_TEMP_OBU_TEST)
void *MSG_MANAGER_TxTask(void *arg)
{
    (void)arg;

    // Prepare the Ext_WSReq_t structure
    int db_v2x_tmp_size = sizeof(DB_V2X_T) + SAMPLE_V2X_MSG_LEN;
    int v2x_tx_pdu_size = sizeof(Ext_V2X_TxPDU_t) + db_v2x_tmp_size;

    Ext_V2X_TxPDU_t *v2x_tx_pdu_p = NULL;
    DB_V2X_T *db_v2x_tmp_p = NULL;
    MSG_MANAGER_TX_EVENT_MSG_T stEventMsg;

    v2x_tx_pdu_p = malloc(v2x_tx_pdu_size);
    if(v2x_tx_pdu_p == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return NULL;
    }

    memset(&stEventMsg, 0, sizeof(MSG_MANAGER_TX_T));
    memset(v2x_tx_pdu_p, 0, sizeof(Ext_V2X_TxPDU_t));

    v2x_tx_pdu_p->ver = htons(SAMPLE_V2X_API_VER);
    v2x_tx_pdu_p->e_payload_type = e_payload_type_g;
    v2x_tx_pdu_p->psid = htonl(psid_g);
    v2x_tx_pdu_p->tx_power = tx_power_g;
    v2x_tx_pdu_p->e_signer_id = e_signer_id_g;
    v2x_tx_pdu_p->e_priority = e_priority_g;

    if (e_comm_type_g == eV2XCommType_LTEV2X || e_comm_type_g == eV2XCommType_5GNRV2X)
    {
        v2x_tx_pdu_p->magic_num = htons(MAGIC_CV2X_TX_PDU);
        v2x_tx_pdu_p->u.config_cv2x.transmitter_profile_id = htonl(transmitter_profile_id_g);
        v2x_tx_pdu_p->u.config_cv2x.peer_l2id = htonl(peer_l2id_g);
    }
    else if (e_comm_type_g == eV2XCommType_DSRC)
    {
        v2x_tx_pdu_p->magic_num = htons(MAGIC_DSRC_TX_PDU);
        v2x_tx_pdu_p->u.config_wave.freq = htons(freq_g);
        v2x_tx_pdu_p->u.config_wave.e_data_rate = htons(e_data_rate_g);
        v2x_tx_pdu_p->u.config_wave.e_time_slot = e_time_slot_g;
        memcpy(v2x_tx_pdu_p->u.config_wave.peer_mac_addr, peer_mac_addr_g, MAC_EUI48_LEN);
    }

    // Payload = KETI Format
    v2x_tx_pdu_p->v2x_msg.length = htons(db_v2x_tmp_size);

    db_v2x_tmp_p = malloc(db_v2x_tmp_size);
    if(db_v2x_tmp_p == NULL)
    {
        PrintError("malloc() is failed! [NULL]");
        return NULL;
    }

    memset(db_v2x_tmp_p, 0, db_v2x_tmp_size);

    db_v2x_tmp_p->eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
    db_v2x_tmp_p->eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5;
    db_v2x_tmp_p->unDeviceId = 0;
    db_v2x_tmp_p->ulTimeStamp = 0ULL;
    db_v2x_tmp_p->eServiceId = DB_V2X_SERVICE_ID_PLATOONING;
    db_v2x_tmp_p->eActionType = DB_V2X_ACTION_TYPE_REQUEST;
    db_v2x_tmp_p->eRegionId = DB_V2X_REGION_ID_SEOUL;
    db_v2x_tmp_p->ePayloadType = DB_V2X_PAYLOAD_TYPE_SAE_J2735_BSM;
    db_v2x_tmp_p->eCommId = DB_V2X_COMM_ID_V2V;
    db_v2x_tmp_p->usDbVer = 0;
    db_v2x_tmp_p->usHwVer = 0;
    db_v2x_tmp_p->usSwVer = 0;
    db_v2x_tmp_p->ulPayloadLength = SAMPLE_V2X_MSG_LEN;
    db_v2x_tmp_p->ulReserved = 0;

    memcpy(v2x_tx_pdu_p->v2x_msg.data, db_v2x_tmp_p, db_v2x_tmp_size);

    printf("\nV2X TX PDU>>\n"
    "  magic_num        : 0x%04X\n"
    "  ver              : 0x%04X\n"
    "  e_payload_type   : %d\n"
    "  psid             : %u\n"
    "  tx_power         : %d\n"
    "  e_signer_id      : %d\n"
    "  e_priority       : %d\n",
    ntohs(v2x_tx_pdu_p->magic_num),
    ntohs(v2x_tx_pdu_p->ver),
    v2x_tx_pdu_p->e_payload_type,
    ntohl(v2x_tx_pdu_p->psid),
    v2x_tx_pdu_p->tx_power,
    v2x_tx_pdu_p->e_signer_id,
    v2x_tx_pdu_p->e_priority);

    if (e_comm_type_g == eV2XCommType_LTEV2X || e_comm_type_g == eV2XCommType_5GNRV2X)
    {
        printf("  u.config_cv2x.transmitter_profile_id : %u\n"
        "  u.config_cv2x.peer_l2id              : %u\n",
        ntohl(v2x_tx_pdu_p->u.config_cv2x.transmitter_profile_id),
        ntohl(v2x_tx_pdu_p->u.config_cv2x.peer_l2id));
    }
    else if (e_comm_type_g == eV2XCommType_DSRC)
    {
        printf("  u.config_wave.freq                  : %d\n"
        "  u.config_wave.e_data_rate           : %d\n"
        "  u.config_wave.e_time_slot           : %d\n"
        "  u.config_wave.peer_mac_addr         : %s\n",
        ntohs(v2x_tx_pdu_p->u.config_wave.freq),
        ntohs(v2x_tx_pdu_p->u.config_wave.e_data_rate),
        v2x_tx_pdu_p->u.config_wave.e_time_slot,
        v2x_tx_pdu_p->u.config_wave.peer_mac_addr);
    }

    uint32_t i;
    ssize_t n;

    for (i = 0; i < tx_cnt_g; i++)
    {
        n = send(s_nSocketHandle, v2x_tx_pdu_p, v2x_tx_pdu_size, 0);
        if (n < 0)
        {
            PrintError("send() is failed!!");
            break;
        }
        else if (n != v2x_tx_pdu_size)
        {
            PrintError("send() sent a different number of bytes than expected!!");
            break;
        }
        else
        {
            PrintDebug("tx send success (%ld bytes) : [%u/%u]", n, i + 1, tx_cnt_g);
        }

        P_MSG_MANAGER_SendTxMsgToDbMgr(&stEventMsg);

        usleep((1000 * tx_delay_g));
    }

    free(v2x_tx_pdu_p);
    free(db_v2x_tmp_p);

    return NULL;
}

void *MSG_MANAGER_RxTask(void *arg)
{
	(void)arg;

	uint8_t buf[4096] = {0};
	int nRecvLen = -1;
	time_t start_time = time(NULL);

	while (1)
	{
		time_t current_time = time(NULL);
		if (current_time - start_time >= delay_time_sec_g)
		{
			break;
		}

		nRecvLen = recv(s_nSocketHandle, buf, sizeof(buf), 0);
		if (nRecvLen < 0)
		{
			if ((errno != EAGAIN) && (errno != EWOULDBLOCK)
			{
				PrintError("recv() is failed!!");
				break;
			}
			else
			{
				usleep(10000);
				continue;
			}
		}
		else if (nRecvLen == 0)
		{
			PrintError("recv()'s connection is closed by peer!!");
			break;
		}
		else
		{
			PrintDebug("recv() is success : nRecvLen[%u]", nRecvLen);
		}
	}

	return NULL;
}
#endif

int32_t P_MSG_MANAGER_CreateTask(void)
{
	int32_t nRet = FRAMEWORK_ERROR;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    nRet = pthread_create(&sh_msgMgrTxTask, &attr, P_MSG_MANAGER_TxTask, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_create() is failed!! (P_MSG_MANAGER_TxTask) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_MSG_MANAGER_TxTask() is successfully created.");
        nRet = FRAMEWORK_OK;
    }

    nRet = pthread_create(&sh_msgMgrRxTask, &attr, P_MSG_MANAGER_RxTask, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_create() is failed!! (P_MSG_MANAGER_RxTask) [nRet:%d]", nRet);
    }
    else
    {
        PrintTrace("P_MSG_MANAGER_RxTask() is successfully created.");
        nRet = FRAMEWORK_OK;
    }

#if defined(CONFIG_PTHREAD_JOINABLE)
    nRet = pthread_join(sh_msgMgrTxTask, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_MSG_MANAGER_TxTask) [nRet:%d]", nRet);
    }
    else
    {
        PrintDebug("P_MSG_MANAGER_TxTask() is successfully joined.");
        nRet = FRAMEWORK_OK;
    }

    nRet = pthread_join(sh_msgMgrRxTask, NULL);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("pthread_join() is failed!! (P_MSG_MANAGER_RxTask) [nRet:%d]", nRet);
    }
    else
    {
        PrintDebug("P_MSG_MANAGER_RxTask() is successfully joined.");
        nRet = FRAMEWORK_OK;
    }
#endif
	return nRet;
}

#if defined(CONFIG_TEMP_OBU_TEST)
int32_t P_MSG_MANAGER_CreateObuTask(void)
{
	int32_t nRet = FRAMEWORK_ERROR;
	pthread_t h_TxTask;
	pthread_t h_RxTask;

	void *pTxTaskRet;
	void *pRxTaskRet;

	pthread_create(&h_TxTask, NULL, MSG_MANAGER_TxTask, NULL);
	pthread_create(&h_RxTask, NULL, MSG_MANAGER_RxTask, NULL);

	pthread_join(h_TxTask, &pTxTaskRet);
	pthread_join(h_RxTask, &pRxTaskRet);

    nRet = FRAMEWORK_OK;

	return nRet;
}
#endif

int32_t MSG_MANAGER_Transmit(MSG_MANAGER_TX_T *pstMsgMgrTx, DB_V2X_T *pstDbV2x, void *pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;
    MSG_MANAGER_TX_EVENT_MSG_T stEventMsg;

    if(pstMsgMgrTx == NULL)
    {
        PrintError("pstMsgMgrTx == NULL!!");
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

    stEventMsg.pstMsgManagerTx = pstMsgMgrTx;
    stEventMsg.pstDbV2x = pstDbV2x;
    stEventMsg.pPayload = pPayload;

    if(msgsnd(s_nMsgTxTaskMsgId, &stEventMsg, sizeof(DB_MANAGER_EVENT_MSG_T), IPC_NOWAIT) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgsnd() is failed!!");
        return nRet;
    }
    else
    {
        nRet = FRAMEWORK_OK;
    }

    return nRet;
}

int32_t MSG_MANAGER_Receive(MSG_MANAGER_RX_T *pstMsgMgrRx, DB_V2X_T *pstDbV2x, void *pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

    if(pstMsgMgrRx == NULL)
    {
        PrintError("pstMsgMgrRx == NULL!!");
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

int32_t MSG_MANAGER_SetLog(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    s_bMsgMgrLog = pstMsgManager->bLogLevel;
    PrintTrace("SET:s_bMsgMgrLog [%s]", s_bMsgMgrLog == ON ? "ON" : "OFF");

    nRet = FRAMEWORK_OK;

    return nRet;
}

int32_t MSG_MANAGER_Open(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    nRet = P_MSG_MANAGER_ConnectV2XDevice(pstMsgManager);
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_ConnectV2XDevice() is failed!!, nRet[%d]", nRet);
        return nRet;
    }

	nRet = P_MSG_MANAGER_SetV2xWsrSetting();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_SetV2xWsrSetting() is failed!!, nRet[%d]", nRet);
        return nRet;
    }

#if defined(CONFIG_TEMP_OBU_TEST)
    nRet = P_MSG_MANAGER_CreateObuTask();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_CreateObuTask() is failed!!, nRet[%d]", nRet);
        return nRet;
    }
#endif

    s_bFirstPacket = TRUE;

    return nRet;
}

int32_t MSG_MANAGER_Close(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    if(s_nSocketHandle != 0)
    {
        close(s_nSocketHandle);
        s_nSocketHandle = 0;
        PrintTrace("Close Connection of V2X Device is successed! [s_nSocketHandle:0x%x]", s_nSocketHandle);
        nRet = FRAMEWORK_OK;
    }
    else
    {
        PrintError("Disconnected [s_nSocketHandle:0x%x]", s_nSocketHandle);
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

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    if((s_nDbTaskMsgId = msgget(s_dbTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_MSG_NABAGER_PrintMsgInfo(s_nDbTaskMsgId);
        nRet = FRAMEWORK_OK;
    }

    if((s_nMsgTxTaskMsgId = msgget(s_MsgTxTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_MSG_NABAGER_PrintMsgInfo(s_nMsgTxTaskMsgId);
        nRet = FRAMEWORK_OK;
    }

    if((s_nMsgRxTaskMsgId = msgget(s_MsgRxTaskMsgKey, IPC_CREAT|0666)) == FRAMEWORK_MSG_ERR)
    {
        PrintError("msgget() is failed!");
        return nRet;
    }
    else
    {
        P_MSG_NABAGER_PrintMsgInfo(s_nMsgRxTaskMsgId);
        nRet = FRAMEWORK_OK;
    }

    nRet = P_MSG_MANAGER_CreateTask();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_CreateTask() is failed!!, nRet[%d]", nRet);
        return nRet;
    }

    s_bMsgMgrLog = pstMsgManager->bLogLevel;
    PrintDebug("s_bMsgMgrLog [%s]", s_bMsgMgrLog == ON ? "ON" : "OFF");

    return nRet;
}

int32_t MSG_MANAGER_DeInit(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    if(pstMsgManager == NULL)
    {
        PrintError("pstMsgManager == NULL!!");
        return nRet;
    }

    nRet = P_MSG_MANAGER_DisconnectV2XDevice();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_ConnectV2XDevice() is failed!!, nRet[%d]", nRet);
        return nRet;
    }

    return nRet;
}

