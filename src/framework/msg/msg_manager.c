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

#include "v2x_defs.h"
#include "v2x_ext_type.h"

/***************************** Definition ************************************/
#define SAMPLE_V2X_API_VER 0x0001
#define SAMPLE_V2X_IP_ADDR "192.168.1.11"
#define SAMPLE_V2X_MSG_LEN 100
#define SAMPLE_V2X_PORT_ADDR 47347

/***************************** Static Variable *******************************/
static int32_t s_nSocketHandle = -1;
static int s_nDbTaskMsgId, s_nMsgTxTaskMsgId, s_nMsgRxTaskMsgId;
static key_t s_dbTaskMsgKey = FRAMEWORK_DB_TASK_MSG_KEY;
static key_t s_MsgTxTaskMsgKey = FRAMEWORK_MSG_TX_TASK_MSG_KEY;
static key_t s_MsgRxTaskMsgKey = FRAMEWORK_MSG_RX_TASK_MSG_KEY;

static pthread_t sh_msgMgrTxTask;
static pthread_t sh_msgMgrRxTask;

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

uint32_t delay_time_sec_g = 10;


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

static int32_t P_MSG_MANAGER_SendTxMsgToDbMgr(MSG_MANAGER_TX_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;
    DB_MANAGER_WRITE_T stDbManagerWrite;
    DB_MANAGER_EVENT_MSG_T stEventMsg;
    DB_V2X_T stDbV2x;
    char cPayload[CLI_DB_V2X_DEFAULT_PAYLOAD_LEN];

    (void*)memset(&stDbManagerWrite, 0x00, sizeof(DB_MANAGER_WRITE_T));
    (void*)memset(&stDbV2x, 0x00, sizeof(DB_V2X_T));
    (void*)memset(&cPayload, 0x00, sizeof(cPayload));

    UNUSED(pstEventMsg);

    stDbManagerWrite.eFileType = DB_MANAGER_FILE_TYPE_TXT;
    stDbManagerWrite.eCommMsgType = DB_MANAGER_COMM_MSG_TYPE_TX;
    stDbManagerWrite.eProc = DB_MANAGER_PROC_WRITE;

    stDbV2x.eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
    stDbV2x.eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5_BROADCAST;
    stDbV2x.unDeviceId = CLI_DB_V2X_DEFAULT_DEVICE_ID;
    stDbV2x.ulTimeStamp = CLI_DB_V2X_DEFAULT_TIMESTAMP;
    stDbV2x.eServiceId = DB_V2X_SERVICE_ID_PLATOONING;
    stDbV2x.eActionType = DB_V2X_ACTION_TYPE_REQUEST;
    stDbV2x.eRegionId = DB_V2X_REGION_ID_SEONGNAM;
    stDbV2x.ePayloadType = DB_V2X_PAYLOAD_TYPE_PLATOONING;
    stDbV2x.eCommId = DB_V2X_COMM_ID_V2V;
    stDbV2x.usDbVer = (DB_V2X_VERSION_MAJOR << CLI_DB_V2X_MAJOR_SHIFT) | DB_V2X_VERSION_MINOR;
    stDbV2x.usHwVer = CLI_DB_V2X_DEFAULT_HW_VER;
    stDbV2x.usSwVer = 0;
    stDbV2x.ulPayloadLength = sizeof(cPayload);
    stDbV2x.ulPacketCrc32 = 0;
    for(int i=0; i < CLI_DB_V2X_DEFAULT_PAYLOAD_LEN; i++)
    {
        cPayload[i] = rand();
        printf("%d ", cPayload[i]);
    }

    stEventMsg.pstDbManagerWrite = &stDbManagerWrite;
    stEventMsg.pstDbV2x = &stDbV2x;
    stEventMsg.pPayload = (char*)&cPayload;

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
    int db_v2x_tmp_size = sizeof(DB_V2X_T) + SAMPLE_V2X_MSG_LEN;
    int v2x_tx_pdu_size = sizeof(Ext_V2X_TxPDU_t) + db_v2x_tmp_size;
    uint32_t i;
    ssize_t n;

    Ext_V2X_TxPDU_t *v2x_tx_pdu_p = NULL;
    DB_V2X_T *db_v2x_tmp_p = NULL;
    MSG_MANAGER_TX_EVENT_MSG_T stEventMsg;

    v2x_tx_pdu_p = malloc(v2x_tx_pdu_size);

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
    db_v2x_tmp_p->ulPacketCrc32 = 0;

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

    for (i = 0; i < pstEventMsg->pstMsgManagerTx->unTxCount; i++)
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
            PrintDebug("tx send success (%ld bytes) : [%u/%u]", n, i + 1, pstEventMsg->pstMsgManagerTx->unTxCount);
        }

        P_MSG_MANAGER_SendTxMsgToDbMgr(&stEventMsg);

        usleep((1000 * pstEventMsg->pstMsgManagerTx->unTxDelay));
    }

    free(v2x_tx_pdu_p);
    free(db_v2x_tmp_p);

    return nRet;
}

static int32_t P_MSG_MANAGER_RxSendTxMsgToDbMgr(MSG_MANAGER_RX_EVENT_MSG_T *pstEventMsg)
{
    int32_t nRet = FRAMEWORK_ERROR;

    uint8_t buf[4096] = {0};
    int n = -1;
    time_t start_time = time(NULL);

    UNUSED(pstEventMsg);

    while (1)
    {
        time_t current_time = time(NULL);
        if (current_time - start_time >= delay_time_sec_g)
        {
            break;
        }

        n = recv(s_nSocketHandle, buf, sizeof(buf), 0);
        if (n < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
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
        else if (n == 0)
        {
            PrintError("recv()'s connection is closed by peer!!");
            break;
        }
        else
        {
            PrintDebug("recv() is success : len[%u]", n);
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
    MSG_MANAGER_RX_EVENT_MSG_T stEventMsg;
    int32_t nRet = FRAMEWORK_ERROR;

    UNUSED(arg);

    memset(&stEventMsg, 0, sizeof(MSG_MANAGER_RX_EVENT_MSG_T));

    while (1)
    {
        if(msgrcv(s_nMsgRxTaskMsgId, &stEventMsg, sizeof(MSG_MANAGER_RX_EVENT_MSG_T), 0, MSG_NOERROR) == FRAMEWORK_MSG_ERR)
        {
            PrintError("msgrcv() is failed!");
        }
        else
        {
            nRet = P_MSG_MANAGER_RxSendTxMsgToDbMgr(&stEventMsg);
            if (nRet != FRAMEWORK_OK)
            {
                PrintError("P_MSG_MANAGER_RxSendTxMsgToDbMgr() is faild! [nRet:%d]", nRet);
            }
        }
    }

    return NULL;
}

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
    db_v2x_tmp_p->ulPacketCrc32 = 0;

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
	int n = -1;
	time_t start_time = time(NULL);

	while (1)
	{
		time_t current_time = time(NULL);
		if (current_time - start_time >= delay_time_sec_g)
		{
			break;
		}

		n = recv(s_nSocketHandle, buf, sizeof(buf), 0);
		if (n < 0)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
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
		else if (n == 0)
		{
			PrintError("recv()'s connection is closed by peer!!");
			break;
		}
		else
		{
			PrintDebug("recv() is success : len[%u]", n);
		}
	}

	return NULL;
}

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

int32_t MSG_MANAGER_Transmit(MSG_MANAGER_TX_T *pstMsgMgrTx, DB_V2X_T *pstDbV2x, void *pPayload)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

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
    MSG_MANAGER_TX_EVENT_MSG_T stEventMsg;

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

int32_t MSG_MANAGER_Open(MSG_MANAGER_T *pstMsgManager)
{
    int32_t nRet = FRAMEWORK_ERROR;

    PrintWarn("TODO");

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

#if 1
    nRet = P_MSG_MANAGER_CreateObuTask();
    if (nRet != FRAMEWORK_OK)
    {
        PrintError("P_MSG_MANAGER_CreateObuTask() is failed!!, nRet[%d]", nRet);
        return nRet;
    }
#endif
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

