#ifndef _NR_V2X_INTERFACE_H_
#define _NR_V2X_INTERFACE_H_
#pragma pack(1) //padding data size delete

#include "v2x_app_ext.h"

#define SIZE_V2X_APP_EXT_HEADER				(sizeof(V2x_App_Hdr))
#define SIZE_TX_HEADER						(sizeof(V2x_App_TxMsg))
#define SIZE_RX_HEADER						(sizeof(V2x_App_RxMsg))
#define SIZE_EXT_HEADER						(sizeof(V2x_App_TLVC))
#define SIZE_WSC_DATA						(sizeof(V2x_App_WSC_Add_Crc) + SIZE_V2X_APP_EXT_HEADER)
#define SIZE_WSR_DATA						(sizeof(V2x_App_WSR_Add_Crc) + SIZE_V2X_APP_EXT_HEADER)

#define SIZE_MAGIC_NUMBER_OF_HEADER			(sizeof(uint32_t))
#define SIZE_LENTH_OF_HEADER				(sizeof(uint16_t))
#define SIZE_LEN_PSID_OF_PAYLOAD			(sizeof(uint32_t))
#define SIZE_CRC16_OF_TAIL					(sizeof(uint16_t))
#define SIZE_V2X_HEADER_WITHOUT_MAGIC		(SIZE_V2X_APP_EXT_HEADER - 4)
#define SIZE_HDR_LEN_EXCEPT_DATA			(10)		// hdr.seq(2) + hdr.payloadid(2) + payload.psid(4) + tail.crc(2)
#define SIZE_CRC_LEN_EXCEPT_DATA			(10)		// hdr.len(2) + hdr.seq(2) + hdr.payloadid(2) + payload.psid(4)

#define MAX_PSID_VALUE						(270549119)
#define MAX_DATA_SIZE						(8999)			// raw 메시지 크기
#define MAX_TX_PAYLOAD_TO_MODULE			(MAX_DATA_SIZE)
#define MAX_RX_PAYLOAD_BY_MODULE			(MAX_DATA_SIZE)

// OBU로 보내는 TX 패킷
#define MAX_TX_PACKET_TO_OBU				(MAX_DATA_SIZE + SIZE_V2X_APP_EXT_HEADER + SIZE_TX_HEADER + SIZE_CRC16_OF_TAIL)
// OBU에서 받는 RX 패킷
#define MAX_RX_PACKET_BY_OBU				(MAX_DATA_SIZE + SIZE_V2X_APP_EXT_HEADER + SIZE_RX_HEADER + SIZE_CRC16_OF_TAIL)

// length는 seq부터 crc까지의 길이 (전체 받는 길이에서 magic과 lenth크기만큼 제외)
#define MAX_RX_LEN_FROM_DEV					(MAX_RX_PACKET_BY_DEVICE - SIZE_MAGIC_NUMBER_OF_HEADER - SIZE_LENTH_OF_HEADER)

typedef enum
{
	CMD_WSM_SERVICE_REQ	= 1,
	CMD_SEND_DATA,
	CMD_SEND_TEST_BSM,
	CMD_SEND_TEST_PVD,
	CMD_SEND_TEST_EXTENSIBLE_V2V,
	CMD_SEND_TEST_EXTENSIBLE_V2I,
	CMD_SEND_TEST_EXTENSIBLE_V2V_ADD_SEQUENCE,
	CMD_SEND_TEST_EXTENSIBLE_V2I_ADD_SEQUENCE,
	CMD_SEND_TEST_EXTENSIBLE_V2_V2V,
	CMD_SEND_TEST_EXTENSIBLE_V2_V2I,
	CMD_SEND_TEST_EXTENSIBLE_V2_V2V_ADD_SEQUENCE,
	CMD_SEND_TEST_EXTENSIBLE_V2_V2I_ADD_SEQUENCE,
	CMD_SEND_TEST_CREATE_DB,
    CMD_SEND_TEST_FTP_CONN_INFO_RESP,
	CMD_MAX
} eNrV2xCmd;

#endif //_NR_V2X_INTERFACE_H_
