/* PC1 side UDP Tx implementation */

#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/time.h>
#include <stdbool.h>
#include <stdarg.h>
#include <pthread.h>
#include <poll.h>
#include <errno.h>

#include "nr_v2x_interface.h"
#include "crc16.h"

#define DEBUG_LV				2
#define DEST_PORT				47347

typedef enum{
	DEBUG_MSG_LV_NONE = 0,
	DEBUG_MSG_LV_LOW = 1,
	DEBUG_MSG_LV_MID = 2,
	DEBUG_MSG_LV_HIGH = 3
}DEBUG_MSG_LV;

static pthread_t pCmdThread;
static bool testMenuPrintStatus = true;
static int choiceNum = -1;
static bool state = true;
static unsigned msgcnt = 0;

static char *__bar ="---------------------------------------------------------------";

const char* getDateTimeStr(void)
{
	static char buf[32];
	struct timeval tv;
	struct tm *ptm;
	int    Y, M, D, h, m, s, mcs;

	gettimeofday(&tv, NULL);
	ptm = localtime(&tv.tv_sec);

	Y   = ptm->tm_year + 1900;
	M   = ptm->tm_mon + 1;
	D   = ptm->tm_mday;
	h   = ptm->tm_hour;
	m   = ptm->tm_min;
	s   = ptm->tm_sec;
	mcs = tv.tv_usec;

	sprintf(buf, "%04d-%02d-%02d.%02d:%02d:%02d.%06d",Y,M,D,h,m,s,mcs);

	return buf;
}


void Test_Msg_Print(char* format, ...)
{
	va_list arg;
	printf("\n#### V2X Message: ");
	va_start(arg, format);
	vprintf(format,arg);
	va_end( arg);

    {
        char szBuf[1024] = {0, };

        va_list lpStart;
        va_start(lpStart, format);
        vsprintf(szBuf, format, lpStart);
        va_end(lpStart);    
    }
}


void Debug_Msg_Print(int msgLv, char* format, ...)
{
	va_list arg;

    if(msgLv <= DEBUG_LV )
	{
		printf("$ [DT=%s] - DEBUG[%d]: ", getDateTimeStr(), msgLv);

		va_start(arg, format);
		vprintf(format,arg);
		va_end( arg);		
		
		printf("\n");

		{
			char szBuf[1024] = {0, };
            
			va_list lpStart;
			va_start(lpStart, format);
			vsprintf(szBuf, format, lpStart);
			va_end(lpStart);
		}
	}
}

void Debug_Msg_Print_Data(int msgLv, unsigned char* data, int len)
{
    int rep;
	char buf[256], hex_str[5];

    if(msgLv <= DEBUG_LV)
    {
		printf("\n\t (Len : 0x%X(%d) bytes)", len, len);
		printf("\n\t========================================================");
		printf("\n\t Hex.   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
		printf("\n\t--------------------------------------------------------\n");
#if 0
		for(rep = 0 ; rep < len ; rep++)
		{
			if(rep % 16 == 0) printf("\n\t %03X- : ", rep/16);
			printf("%02X ", data[rep]);
		}
#else
		for(rep = 0 ; rep < len ; rep++)
		{
			if(rep % 16 == 0)
			{
				if (rep == 0)
					sprintf(buf, "\t %03X- : ", rep/16);
				else
				{
					printf("%s\n", buf);
					sprintf(buf, "\t %03X- : ", rep/16);
				}
			}
			
			sprintf(hex_str, "%02X ", data[rep]);
			strcat(buf, hex_str);
		}
		printf("%s\n", buf);
#endif
		printf("\t========================================================");
		printf("\n\n");
    }
}

/**
  * @name	ReadNTcp
  * @brief	TCP연결 후 요청하는 길이만큼 읽어서 전달
  * @param	const int sock : 소켓
  			char *buf : 받을 데이터 넣을 버퍼
  			uint32_t len : 읽어올 데이터 길이
  * @return 0 : close 필요, -1 : 에러, -2 : 알수없는 에러, 0> : 성공(읽은 크기)
 **/
int ReadNTcp(const int sock, uint8_t *buf, uint32_t len)
{
	int size, readn = 0, eagain = 0;
	uint32_t remain = len;
	struct pollfd poll_fd;

	while(remain > 0)
	{
		size = read(sock, (char *)&buf[readn], remain);

		if(size > 0)
		{
			readn += size;
			remain -= size;
		}
		else if(size == 0)
			return 0;
		else
		{
			if(errno == EAGAIN)
			{
				if(eagain > 100)
				{
					return -1;
				}

				eagain++;
				usleep(0);
				continue;
			}
			else if(errno == ECONNRESET)
			{
				return 0;
			}
			else
				return -2;
		}

		if (remain > 0)
		{
			poll_fd.fd = sock;
			poll_fd.events = POLLIN;
			if (!poll(&poll_fd, 1, 5000))
			{
				perror("data leak\n");
				return -3;		// No data
			}
		}
	} // while

	return readn;
}

/**
  * @name	WriteNTcp
  * @brief	TCP연결 후 요청하는 길이만큼 전송
  * @param	const int sock : 소켓
  			char *buf : 쓸 데이터 넣은 버퍼
  			uint32_t len : 쓸 데이터 길이
  * @return 0 : close 필요, -1 : 에러, -2 : 알수없는 에러, 0> : 성공(읽은 크기)
 **/
int WriteNTcp(const int sock, uint8_t *buf, uint32_t len)
{
	int size, readn = 0, eagain = 0;
	uint32_t remain = len;

	while(remain > 0)
	{
		size = write(sock, (char *)&buf[readn], remain);

		if(size > 0)
		{
			readn += size;
			remain -= size;
		}
		else if(size == 0)
		{
			return 0;
		}
		else
		{
			if(errno == EAGAIN)
			{
				if(eagain > 100)
				{
					return -1;
				}

				eagain++;
				usleep(0);
				continue;
			}
			else if(errno == ECONNRESET)
				return 0;
			else
				return -2;
		}
	} // while

	return readn;
}

/**
  * @name   AddExtStatusData
  * @brief  Extensible 메시지에 Status Data 추가하는 함수
  * @param  TLVC_Overall *p_overall : Overall 위치 포이터
  * @return int : 성공 - 0, 실패 - 음수
 **/
int AddExtStatusData(TLVC_Overall *p_overall, int tx_rx)
{
	TLVC_STATUS_CommUnit *p_status;
	struct timeval now;
	struct tm *tm;
	uint64_t keti_time;
	uint16_t package_len = ntohs(p_overall->len_package);

	p_status  = (TLVC_STATUS_CommUnit *)((uint8_t*)p_overall + sizeof(TLVC_Overall) + package_len);

	p_overall->num_package++;
	package_len += sizeof(TLVC_STATUS_CommUnit);
	p_overall->len_package = htons(package_len);
	p_overall->crc = htons(CalcCRC16((uint8_t*)p_overall, sizeof(TLVC_Overall) - 2));	// TLVC 중 CRC만 제외

	p_status->type = htonl(EM_PT_STATUS);
	p_status->len = htons(sizeof(TLVC_STATUS_CommUnit) - 6);
	p_status->dev_type = eStatusDevType_Obu;
	p_status->tx_rx = tx_rx;
	p_status->dev_id = htonl(1);
	p_status->hw_ver = htons(2);
	p_status->sw_ver = htons(3);
	gettimeofday(&now, NULL);
	now.tv_sec = now.tv_sec + (3600 * 9);			// UTC -> KST
	tm = localtime(&now.tv_sec);
	keti_time = (uint64_t)(tm->tm_year+1900) * 1000000000000000 +
				(uint64_t)(tm->tm_mon+1)     * 10000000000000 +
				(uint64_t)tm->tm_mday        * 100000000000 +
				(uint64_t)tm->tm_hour        * 1000000000 +
				(uint64_t)tm->tm_min         * 10000000 +
				(uint64_t)tm->tm_sec         * 100000 +
				(uint64_t)now.tv_usec        / 10;

	p_status->timestamp = htobe64(keti_time);
	p_status->crc = htons(CalcCRC16((uint8_t*)p_status, sizeof(TLVC_STATUS_CommUnit) - 2));	// TLVC 중 CRC만 제외

	return 0;
}

bool Test_App_Main(int fd)
{
	bool rtnVal = true, flag_send_fin = false;
	int re = 0, n, send_len, i, cnt = 1, period = 100;
	char buf[MAX_TX_PACKET_TO_OBU];

	memset(buf,0, sizeof(buf));	
	V2x_App_Hdr* hdr = (V2x_App_Hdr*)buf;
	choiceNum = -1;

	printf("\n%s", __bar);

	if (testMenuPrintStatus == true)
	{
		Test_Msg_Print("Please select menu ");
		Test_Msg_Print("< V2X Message Set >-------------------");
		Test_Msg_Print("[%d] WSR", CMD_WSM_SERVICE_REQ);
		Test_Msg_Print("[%d] Send Data", CMD_SEND_DATA);
		Test_Msg_Print("[%d] Send Test BSM", CMD_SEND_TEST_BSM);
		Test_Msg_Print("[%d] Send Test PVD", CMD_SEND_TEST_PVD);
		Test_Msg_Print("[%d] Send V2V Test Extensible MSG", CMD_SEND_TEST_EXTENSIBLE_V2V);
		Test_Msg_Print("[%d] Send V2I Test Extensible MSG", CMD_SEND_TEST_EXTENSIBLE_V2I);
		Test_Msg_Print("[%d] Send V2V Test Extensible MSG with SEQ", CMD_SEND_TEST_EXTENSIBLE_V2V_ADD_SEQUENCE);
		Test_Msg_Print("[%d] Send V2I Test Extensible MSG with SEQ", CMD_SEND_TEST_EXTENSIBLE_V2I_ADD_SEQUENCE);

		Test_Msg_Print("< EXIT >-------------------------------");
		Test_Msg_Print("[0] Exit");
	}
	Test_Msg_Print("Enter your choice : \n");

	re = scanf("%d", &choiceNum);
	printf("\n%s", __bar);
	Test_Msg_Print("choiceTest Number: %d\n\n", choiceNum);

	if ((choiceNum > CMD_MAX) || (choiceNum < 0))
		return false;

 	int rand_m= 0;
	int char_size = 0;
	
	switch (choiceNum)
	{
	  case 0:
		rtnVal = false;
		break;
	  case CMD_WSM_SERVICE_REQ:
	  {
		int psid = -1, action = -1;
		V2x_App_WSR* wsr = (V2x_App_WSR*)hdr->data;
		
		Debug_Msg_Print(DEBUG_MSG_LV_MID," >> WSR len: %d",(int) sizeof(SIZE_WSR_DATA));
		memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
		hdr->len = htons(SIZE_WSR_DATA-6);
		hdr->seq = 0;
		hdr->payload_id = htons(ePayloadId_WsmServiceReq);
		Test_Msg_Print("Enter Action(add:0, del:1) : \n");
		re = scanf("%d", &action);
		Test_Msg_Print("Enter psid : \n");
		re = scanf("%d", &psid);
		if (action < 0 || action > 1 || psid < 0)
		{
			perror("Action or PSID Error\n");
			return true;
		}
		wsr->action = action;
		wsr->psid = htonl(psid);
		send_len = SIZE_WSR_DATA;

		printf("action = %s, psid = %u\n", (action==0)?"ADD":"DEL", psid);
		break;
	  }

	  case CMD_SEND_DATA:
	  {
		int size = -1, psid = -1, i;
		uint16_t *crc16;
		uint16_t calc_crc16;
		V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;

		Debug_Msg_Print(DEBUG_MSG_LV_LOW," >> Send DATA");
		Test_Msg_Print("Enter psid : \n");
		re = scanf("%d", &psid);
		Test_Msg_Print("Enter Send Data Size(except header and crc) : \n");
		re = scanf("%d", &size);
		if (size < 0 || size > MAX_DATA_SIZE || psid < 0)
		{
			perror("Size or PSID Error\n");
			return true;
		}
		
		memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
		hdr->len = htons(size + 10);		// seq 2byte, payload id 2byte, psid 4byte, crc16 2byte
		hdr->seq = 0;
		hdr->payload_id = htons(ePayloadId_TxMsg);
		tx_msg->psid = htonl(psid);
		for (i=0; i<size; i++)
		{
			tx_msg->data[i] = rand()%255;
		}
		send_len = SIZE_V2X_APP_EXT_HEADER + size + SIZE_CRC16_OF_HEADER + 4;

		crc16 = (uint16_t*)&buf[send_len-2];
		calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, size + 10);
		*crc16 = htons(calc_crc16);
		break;
	  }

	  case CMD_SEND_TEST_BSM:
	  {
		int size = -1, psid = -1, i;
		uint16_t *crc16;
		uint16_t calc_crc16;
		V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;
		char test_bsm[] = {0x00, 0x14, 0x80, 0x98, 0x46, 0x82, 0x82, 0xC3,
							0x03, 0x4C, 0x0E, 0x65, 0xE2, 0xC0, 0x00, 0x5B,
							0x87, 0xA8, 0xFF, 0x88, 0x1C, 0xE3, 0x6E, 0xAB,
							0xE5, 0x01, 0x33, 0xA6, 0x94, 0xBA, 0xA8, 0xC8,
							0xFC, 0xD9, 0x82, 0xBB, 0x78, 0xC4, 0x4B, 0x11,
							0x32, 0x01, 0x59, 0xF0, 0xE0, 0x00, 0x1B, 0xFF,
							0xFB, 0xF2, 0x0F, 0x1C, 0xF1, 0xD4, 0xC3, 0x49,
							0x6B, 0xF1, 0xAA, 0xBF, 0x6D, 0xE9, 0x35, 0x00,
							0x20, 0x72, 0xE1, 0x1D, 0x07, 0xD0, 0x78, 0xCA,
							0x00, 0x8C, 0x1F, 0xF2, 0x66, 0x11, 0x3F, 0x0D,
							0x41, 0x61, 0xA8, 0x43, 0x20, 0x9C, 0x38, 0x7F,
							0x51, 0x96, 0xA0, 0x44, 0x1E, 0x5F, 0x86, 0xA0,
							0xB0, 0xD4, 0x21, 0x90, 0x4E, 0x1C, 0x3F, 0xA8,
							0xCB, 0x50, 0x22, 0x0F, 0x2F, 0xC3, 0x50, 0x58,
							0x6A, 0x10, 0xC8, 0x27, 0x0E, 0x1F, 0xD4, 0x65,
							0xA8, 0x11, 0x07, 0x94, 0x03, 0x68, 0xC2, 0xAA,
							0x00, 0x11, 0x06, 0x40, 0x4A, 0x02, 0x80, 0x03,
							0xC1, 0x3D, 0x84, 0x7D, 0x6C, 0xC4, 0x04, 0x04,
							0x04, 0x00, 0x00, 0x81, 0xD8, 0x00, 0x00, 0x78,
							0x40, 0x07, 0x68, 0x00};

		Test_Msg_Print("Enter count : \n");
		re = scanf("%d", &cnt);
		if (cnt > 1)
		{
			Test_Msg_Print("Enter Period(ms) : \n");
			re = scanf("%d", &period);
		}
		else
			cnt = 1;

		psid = 82050;
		size = sizeof(test_bsm);
		memcpy(tx_msg->data, test_bsm, size);

		memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
		hdr->len = htons(size + 10);		// seq 2byte, payload id 2byte, psid 4byte, crc16 2byte
		hdr->seq = 0;
		hdr->payload_id = htons(ePayloadId_TxMsg);
		tx_msg->psid = htonl(psid);
		send_len = SIZE_V2X_APP_EXT_HEADER + size + SIZE_CRC16_OF_HEADER + 4;
		crc16 = (uint16_t*)&buf[send_len-2];
		calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, size + 10);
		*crc16 = htons(calc_crc16);

		Debug_Msg_Print(DEBUG_MSG_LV_LOW," >> Send Test BSM DATA");
		break;
	  }

	  case CMD_SEND_TEST_PVD:
	  {
		int size = -1, psid = -1, i;
		uint16_t *crc16;
		uint16_t calc_crc16;
		V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;
		char test_pvd[] = {0x00, 0x1A, 0x82, 0x1F, 0x70, 0x00, 0x02, 0x00,
							0x05, 0x7E, 0xA4, 0x5E, 0x9D, 0x34, 0xE6, 0xDF,
							0xCD, 0x6C, 0x39, 0x35, 0x29, 0xAD, 0x9F, 0x0A,
							0xAA, 0xD6, 0x0C, 0x18, 0x31, 0x52, 0x2B, 0xA3,
							0xA3, 0x4B, 0x33, 0x7B, 0x99, 0x6A, 0xB2, 0x4A,
							0x74, 0x22, 0xF4, 0xE9, 0xA7, 0x36, 0xFE, 0x6B,
							0x77, 0xF7, 0xDD, 0x97, 0x94, 0x3D, 0xF9, 0x32,
							0x80, 0x40, 0x80, 0xC1, 0x08, 0x00, 0x7F, 0xFF,
							0x7E, 0x64, 0x0B, 0xDE, 0xAF, 0xC8, 0x69, 0x16,
							0xE1, 0xEA, 0x40, 0xD2, 0xF1, 0x60, 0x01, 0xC4,
							0x19, 0x52, 0xC0, 0x71, 0xF4, 0x0A, 0x14, 0x01,
							0x2C, 0x00, 0x00, 0x00, 0xFF, 0x02, 0x00, 0x14,
							0x80, 0xCE, 0xD0, 0x21, 0x6F, 0xFF, 0xEF, 0xCC,
							0x81, 0x7B, 0xD5, 0xF9, 0x0D, 0x22, 0xDC, 0x3D,
							0x48, 0x1A, 0x5E, 0x2C, 0x00, 0x38, 0x83, 0x2A,
							0x58, 0x0E, 0x3E, 0x81, 0x42, 0x80, 0x25, 0x80,
							0x00, 0x00, 0x1F, 0x0E, 0x55, 0x49, 0xBF, 0xFF,
							0xBF, 0x20, 0xF1, 0xCF, 0x1D, 0x4C, 0x34, 0x96,
							0xBF, 0x1A, 0xAB, 0xF6, 0xDE, 0x93, 0x50, 0x02,
							0x07, 0x2E, 0x11, 0xD0, 0x7D, 0x07, 0x8C, 0xA0,
							0x08, 0xC1, 0xFF, 0x26, 0x61, 0x03, 0xF0, 0xD4,
							0x16, 0x1A, 0x84, 0x32, 0x09, 0xC3, 0x87, 0xF5,
							0x19, 0x6A, 0x04, 0x41, 0xE5, 0x00, 0xDA, 0x30,
							0xAA, 0xBF, 0xFF, 0xFB, 0x00, 0x0E, 0x0F, 0x00,
							0xF3, 0xC6, 0x20, 0xC8, 0x0A, 0x0A, 0x00, 0x3E,
							0x04, 0x7F, 0x81, 0xF5, 0xDC, 0x5D, 0xC5, 0xB4,
							0x03, 0xBA, 0x00, 0x00, 0x02, 0x00, 0x0B, 0xFB,
							0xF3, 0x20, 0x5E, 0xF5, 0x7E, 0x43, 0x48, 0x7F,
							0xFF, 0x7E, 0x64, 0x0B, 0xDE, 0xAF, 0xC8, 0x69,
							0x16, 0xE1, 0xEA, 0x40, 0xD2, 0xF1, 0x60, 0x01,
							0xC4, 0x19, 0x52, 0xC0, 0x71, 0xF4, 0x0A, 0x14,
							0x01, 0x2C, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x05,
							0x0A, 0x15, 0x90, 0x00, 0x00, 0x47, 0xEA, 0x45,
							0xE9, 0xD3, 0x4E, 0x6D, 0xFC, 0xD6, 0xC3, 0x93,
							0x52, 0x9A, 0xD9, 0xF0, 0xAA, 0xAD, 0x60, 0xC1,
							0x83, 0x25, 0xA2, 0xBA, 0x3A, 0x34, 0xB3, 0x37,
							0xB9, 0x96, 0xAB, 0x24, 0xA7, 0x19, 0x46, 0x2F,
							0x4E, 0x9A, 0x73, 0x6F, 0xE6, 0xB7, 0x7F, 0x7D,
							0xD9, 0x79, 0x43, 0xDF, 0x93, 0x2B, 0x28, 0x00,
							0x00, 0x02, 0x69, 0x00, 0x1E, 0x80, 0x00, 0x88,
							0x00, 0x10, 0x30, 0x06, 0xFF, 0xFE, 0xFC, 0xC8,
							0x17, 0xBD, 0x5F, 0x90, 0xD2, 0x2D, 0xC3, 0xD4,
							0x81, 0xA5, 0xE2, 0xC0, 0x03, 0x88, 0x32, 0xA5,
							0x80, 0xE3, 0xE8, 0x14, 0x28, 0x02, 0x58, 0x00,
							0x00, 0x01, 0xF0, 0xE5, 0x54, 0x9B, 0xFF, 0xFB,
							0xF2, 0x0F, 0x1C, 0xF1, 0xD4, 0xC3, 0x49, 0x6B,
							0xF1, 0xAA, 0xBF, 0x6D, 0xE9, 0x35, 0x00, 0x20,
							0x72, 0xE1, 0x1D, 0x07, 0xD0, 0x78, 0xCA, 0x00,
							0x8C, 0x1F, 0xF2, 0x66, 0x10, 0x3F, 0x0D, 0x41,
							0x61, 0xA8, 0x43, 0x20, 0x9C, 0x38, 0x7F, 0x51,
							0x96, 0xA0, 0x44, 0x1E, 0x50, 0x0D, 0xA3, 0x0A,
							0xAB, 0xFF, 0xFF, 0xB0, 0x00, 0xE0, 0xF0, 0x0F,
							0x3C, 0x62, 0x0C, 0x80, 0xA0, 0xA0, 0x03, 0xE0,
							0x47, 0xF8, 0x1F, 0x5D, 0xC5, 0xDC, 0x5B, 0x40,
							0x3B, 0xA0, 0x00, 0x00, 0x20, 0x00, 0xBF, 0xBF,
							0x32, 0x05, 0xEF, 0x57, 0xE4, 0x34, 0x87, 0xFF,
							0xF7, 0xE6, 0x40, 0xBD, 0xEA, 0xFC, 0x86, 0x91,
							0x6E, 0x1E, 0xA4, 0x0D, 0x2F, 0x16, 0x00, 0x1C,
							0x41, 0x95, 0x2C, 0x07, 0x1F, 0x40, 0xA1, 0x40,
							0x12, 0xC0, 0x00, 0x00, 0x0C, 0x80, 0x00, 0x50,
							0xA1, 0x59, 0x00, 0x00, 0x04, 0x7E, 0xA4, 0x5E,
							0x9D, 0x34, 0xE6, 0xDF, 0xCD, 0x6C, 0x39, 0x35,
							0x29, 0xAD, 0x9F, 0x0A, 0xAA, 0xD6, 0x0C, 0x18,
							0x32, 0x5A, 0x2B, 0xA3, 0xA3, 0x4B, 0x33, 0x7B,
							0x99, 0x6A, 0xB2, 0x4A, 0x71, 0x94, 0x62, 0xF4,
							0xE9, 0xA7, 0x36, 0xFE, 0x6B, 0x77, 0xF7, 0xDD,
							0x97, 0x94, 0x3D, 0xF9, 0x32, 0xB2, 0x80, 0x00,
							0x00, 0x26, 0x90, 0x01, 0xE8, 0x00, 0x08, 0x80,
							0x01, 0x03, 0x00};

		Test_Msg_Print("Enter count : \n");
		re = scanf("%d", &cnt);
		if (cnt > 1)
		{
			Test_Msg_Print("Enter Period(ms) : \n");
			re = scanf("%d", &period);
		}
		else
			cnt = 1;

		psid = 82051;
		size = sizeof(test_pvd);
		memcpy(tx_msg->data, test_pvd, size);

		memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
		hdr->len = htons(size + 6 + 4);		// header 6byte, psid 4byte
		hdr->seq = 0;
		hdr->payload_id = htons(ePayloadId_TxMsg);
		tx_msg->psid = htonl(psid);
		send_len = SIZE_V2X_APP_EXT_HEADER + 4 + size + SIZE_CRC16_OF_HEADER;
		crc16 = (uint16_t*)&buf[send_len-2];
		calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, size + 10);
		*crc16 = htons(calc_crc16);

		Debug_Msg_Print(DEBUG_MSG_LV_LOW," >> Send Test PVD DATA");
		break;
	  }

	case CMD_SEND_TEST_EXTENSIBLE_V2V:
	{
		int size = -1, psid = -1, i;
		uint16_t *crc16;
		uint16_t calc_crc16;
		V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;
		TLVC_Overall *p_overall;
		V2x_App_Ext_TLVC *p_dummy;
		TLVC_STATUS_CommUnit *p_status;
		struct timeval now;
		struct tm *tm;
		uint64_t keti_time;
		uint16_t package_len;

		Debug_Msg_Print(DEBUG_MSG_LV_LOW," >> Send Extensible(V2V) DATA");
		Test_Msg_Print("Enter added Data Size(except header and crc) : \n");
		re = scanf("%d", &size);
		psid = EM_V2V_MSG;
		if (size < 0 || size > (MAX_DATA_SIZE-1000) || psid < 0)
		{
			perror("Size Error\n");
			return true;
		}

		p_overall = (TLVC_Overall*)tx_msg->data;
		p_overall->type = htonl(EM_PT_OVERALL);
		p_overall->len = htons(sizeof(TLVC_Overall) - 6);
		p_overall->magic[0] = 'E';
		p_overall->magic[1] = 'M';
		p_overall->magic[2] = 'O';
		p_overall->magic[3] = 'P';
		p_overall->version = 1;
		p_overall->num_package = 0;
		p_overall->len_package = 0;

		package_len = ntohs(p_overall->len_package);

		if (size > 0)
		{
			p_dummy  = (V2x_App_Ext_TLVC *)((uint8_t*)p_overall + sizeof(TLVC_Overall) + package_len);

			p_overall->num_package++;
			package_len = package_len + 8 + size;
			p_overall->len_package = htons(package_len);
			p_overall->crc = htons(CalcCRC16((uint8_t*)p_overall, sizeof(TLVC_Overall) - 2));	// TLVC 중 CRC만 제외

			p_dummy->type = htonl(EM_PT_RAW_DATA);
			p_dummy->len = htons(size + 2);		// V=size, C=2
			for(i=0; i < size; i++)
			{
				p_dummy->data[i] = i % 255;
			}

			crc16 = (uint16_t*)((uint8_t*)p_dummy + 6 + size);
			*crc16 = htons(CalcCRC16((uint8_t*)p_dummy, size+6));	// TLVC 중 CRC만 제외
		}

		AddExtStatusData(p_overall, eStatusTxRx_Tx);
		
		memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
		send_len = 16 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package);	// 16 : header(10) + psid(4) + crc(2)
		hdr->len = htons(10 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package));		// seq(2) + payload id(2) + crc(2) + psid(4)
		hdr->seq = 0;
		hdr->payload_id = htons(ePayloadId_TxMsg);
		tx_msg->psid = htonl(EM_V2V_MSG);

		send_len = ntohs(hdr->len) + 6;		// magic 4byte, lenth 2byte

		crc16 = (uint16_t*)&buf[send_len-2];
		calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, send_len - 6);		// magic(4), crc(2)
		*crc16 = htons(calc_crc16);

		break;
	}

	case CMD_SEND_TEST_EXTENSIBLE_V2I:
	{
		int size = -1, psid = -1, i;
		uint16_t *crc16;
		uint16_t calc_crc16;
		V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;
		TLVC_Overall *p_overall;
		V2x_App_Ext_TLVC *p_dummy;
		TLVC_STATUS_CommUnit *p_status;
		struct timeval now;
		struct tm *tm;
		uint64_t keti_time;
		uint16_t package_len;

		Debug_Msg_Print(DEBUG_MSG_LV_LOW," >> Send Extensible(V2I) DATA");
		Test_Msg_Print("Enter added Data Size(except header and crc) : \n");
		re = scanf("%d", &size);
		psid = EM_V2I_MSG;
		if (size < 0 || size > (MAX_DATA_SIZE-1000) || psid < 0)
		{
			perror("Size Error\n");
			return true;
		}

		p_overall = (TLVC_Overall*)tx_msg->data;
		p_overall->type = htonl(EM_PT_OVERALL);
		p_overall->len = htons(sizeof(TLVC_Overall) - 6);
		p_overall->magic[0] = 'E';
		p_overall->magic[1] = 'M';
		p_overall->magic[2] = 'O';
		p_overall->magic[3] = 'P';
		p_overall->version = 1;
		p_overall->num_package = 0;
		p_overall->len_package = 0;

		package_len = ntohs(p_overall->len_package);

		if (size > 0)
		{
			p_dummy  = (V2x_App_Ext_TLVC *)((uint8_t*)p_overall + sizeof(TLVC_Overall) + package_len);

			p_overall->num_package++;
			package_len = package_len + 8 + size;
			p_overall->len_package = htons(package_len);
			p_overall->crc = htons(CalcCRC16((uint8_t*)p_overall, sizeof(TLVC_Overall) - 2));	// TLVC 중 CRC만 제외

			p_dummy->type = htonl(EM_PT_RAW_DATA);
			p_dummy->len = htons(size + 2);		// V=size, C=2
			for(i=0; i < size; i++)
			{
				p_dummy->data[i] = i % 255;
			}

			crc16 = (uint16_t*)((uint8_t*)p_dummy + 6 + size);
			*crc16 = htons(CalcCRC16((uint8_t*)p_dummy, size+6));	// TLVC 중 CRC만 제외
		}

		AddExtStatusData(p_overall, eStatusTxRx_Tx);
		
		memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
		send_len = 16 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package);	// 16 : header(10) + psid(4) + crc(2)
		hdr->len = htons(10 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package));		// seq(2) + payload id(2) + crc(2) + psid(4)
		hdr->seq = 0;
		hdr->payload_id = htons(ePayloadId_TxMsg);
		tx_msg->psid = htonl(EM_V2I_MSG);

		send_len = ntohs(hdr->len) + 6;		// magic 4byte, lenth 2byte

		crc16 = (uint16_t*)&buf[send_len-2];
		calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, send_len - 6);		// magic(4), crc(2)
		*crc16 = htons(calc_crc16);

		break;
	}

	  case CMD_SEND_TEST_EXTENSIBLE_V2V_ADD_SEQUENCE:
	  {
		int size = -1, psid = -1, i;
		uint32_t *p_seq;
		uint16_t *crc16;
		uint16_t calc_crc16;
		V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;
		TLVC_Overall *p_overall;
		V2x_App_Ext_TLVC *p_dummy;
		TLVC_STATUS_CommUnit *p_status;
		struct timeval now;
		struct tm *tm;
		uint64_t keti_time;
		uint16_t package_len;

		Debug_Msg_Print(DEBUG_MSG_LV_LOW," >> Send Extensible(V2V) DATA");
		Test_Msg_Print("Enter count : \n");
		re = scanf("%d", &cnt);
		if (cnt > 1)
		{
			Test_Msg_Print("Enter Period(ms) : \n");
			re = scanf("%d", &period);
		}
		else
			cnt = 1;

		for (i=0; i<cnt; i++)
		{
			psid = EM_V2V_MSG;
			p_overall = (TLVC_Overall*)tx_msg->data;
			p_overall->type = htonl(EM_PT_OVERALL);
			p_overall->len = htons(sizeof(TLVC_Overall) - 6);
			p_overall->magic[0] = 'E';
			p_overall->magic[1] = 'M';
			p_overall->magic[2] = 'O';
			p_overall->magic[3] = 'P';
			p_overall->version = 1;
			p_overall->num_package = 0;
			p_overall->len_package = 0;

			package_len = ntohs(p_overall->len_package);

			// add raw package for sequence number
			size = 4;	// raw package size
			p_dummy  = (V2x_App_Ext_TLVC *)((uint8_t*)p_overall + sizeof(TLVC_Overall) + package_len);
			p_overall->num_package++;
			package_len = package_len + 8 + size;
			p_overall->len_package = htons(package_len);
			p_overall->crc = htons(CalcCRC16((uint8_t*)p_overall, sizeof(TLVC_Overall) - 2));	// TLVC 중 CRC만 제외
			p_dummy->type = htonl(EM_PT_RAW_DATA);
			p_dummy->len = htons(size + 2);		// V=size, C=2
			p_seq = (uint32_t*)p_dummy->data;
			*p_seq = htonl(i+1);
			crc16 = (uint16_t*)((uint8_t*)p_dummy + 6 + size);
			*crc16 = htons(CalcCRC16((uint8_t*)p_dummy, size+6));	// TLVC 중 CRC만 제외

			// add status package
			AddExtStatusData(p_overall, eStatusTxRx_Tx);
			
			memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
			send_len = 16 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package);	// 16 : header(10) + psid(4) + crc(2)
			hdr->len = htons(10 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package));		// seq(2) + payload id(2) + crc(2) + psid(4)
			hdr->seq = 0;
			hdr->payload_id = htons(ePayloadId_TxMsg);
			tx_msg->psid = htonl(EM_V2V_MSG);

			send_len = ntohs(hdr->len) + 6;		// magic 4byte, lenth 2byte

			crc16 = (uint16_t*)&buf[send_len-2];
			calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, send_len - 6);		// magic(4), crc(2)
			*crc16 = htons(calc_crc16);

			Debug_Msg_Print(DEBUG_MSG_LV_MID, "\tCount = %d / %d, Period = %d ms", i+1, cnt, period);
			Debug_Msg_Print_Data(DEBUG_MSG_LV_MID, buf, send_len);
			n = send(fd, buf, send_len, 0);
			if (n < 0)
			{
				perror("send() failed");
			}
			else if (n != send_len)
			{
				fprintf(stderr, "send() sent a different number of bytes than expected\n");
			}
			else if (n == 0)
			{
				perror("closed socket");
				close(fd);
				return false;
			}
			usleep(period * 1000);
		}

		flag_send_fin = true;

		break;
	  }

	  case CMD_SEND_TEST_EXTENSIBLE_V2I_ADD_SEQUENCE:
	  {
		int size = -1, psid = -1, i;
		uint32_t *p_seq;
		uint16_t *crc16;
		uint16_t calc_crc16;
		V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;
		TLVC_Overall *p_overall;
		V2x_App_Ext_TLVC *p_dummy;
		TLVC_STATUS_CommUnit *p_status;
		struct timeval now;
		struct tm *tm;
		uint64_t keti_time;
		uint16_t package_len;

		Debug_Msg_Print(DEBUG_MSG_LV_LOW," >> Send Extensible(V2I) DATA");
		Test_Msg_Print("Enter count : \n");
		re = scanf("%d", &cnt);
		if (cnt > 1)
		{
			Test_Msg_Print("Enter Period(ms) : \n");
			re = scanf("%d", &period);
		}
		else
			cnt = 1;

		for (i=0; i<cnt; i++)
		{
			psid = EM_V2I_MSG;
			p_overall = (TLVC_Overall*)tx_msg->data;
			p_overall->type = htonl(EM_PT_OVERALL);
			p_overall->len = htons(sizeof(TLVC_Overall) - 6);
			p_overall->magic[0] = 'E';
			p_overall->magic[1] = 'M';
			p_overall->magic[2] = 'O';
			p_overall->magic[3] = 'P';
			p_overall->version = 1;
			p_overall->num_package = 0;
			p_overall->len_package = 0;

			package_len = ntohs(p_overall->len_package);

			// add raw package for sequence number
			size = 4;	// raw package size
			p_dummy  = (V2x_App_Ext_TLVC *)((uint8_t*)p_overall + sizeof(TLVC_Overall) + package_len);
			p_overall->num_package++;
			package_len = package_len + 8 + size;
			p_overall->len_package = htons(package_len);
			p_overall->crc = htons(CalcCRC16((uint8_t*)p_overall, sizeof(TLVC_Overall) - 2));	// TLVC 중 CRC만 제외
			p_dummy->type = htonl(EM_PT_RAW_DATA);
			p_dummy->len = htons(size + 2);		// V=size, C=2
			p_seq = (uint32_t*)p_dummy->data;
			*p_seq = htonl(i+1);
			crc16 = (uint16_t*)((uint8_t*)p_dummy + 6 + size);
			*crc16 = htons(CalcCRC16((uint8_t*)p_dummy, size+6));	// TLVC 중 CRC만 제외

			// add status package
			AddExtStatusData(p_overall, eStatusTxRx_Tx);
			
			memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
			send_len = 16 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package);	// 16 : header(10) + psid(4) + crc(2)
			hdr->len = htons(10 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package));		// seq(2) + payload id(2) + crc(2) + psid(4)
			hdr->seq = 0;
			hdr->payload_id = htons(ePayloadId_TxMsg);
			tx_msg->psid = htonl(EM_V2I_MSG);

			send_len = ntohs(hdr->len) + 6;		// magic 4byte, lenth 2byte

			crc16 = (uint16_t*)&buf[send_len-2];
			calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, send_len - 6);		// magic(4), crc(2)
			*crc16 = htons(calc_crc16);

			Debug_Msg_Print(DEBUG_MSG_LV_MID, "\tCount = %d / %d, Period = %d ms", i+1, cnt, period);
			Debug_Msg_Print_Data(DEBUG_MSG_LV_MID, buf, send_len);
			n = send(fd, buf, send_len, 0);
			if (n < 0)
			{
				perror("send() failed");
			}
			else if (n != send_len)
			{
				fprintf(stderr, "send() sent a different number of bytes than expected\n");
			}
			else if (n == 0)
			{
				perror("closed socket");
				close(fd);
				return false;
			}
			usleep(period * 1000);
		}

		flag_send_fin = true;

		break;
	  }

	  default:
		Debug_Msg_Print(DEBUG_MSG_LV_LOW, "\tIt was the wrong choice !");
		break;
	} /*End of switch*/
	
	Debug_Msg_Print(DEBUG_MSG_LV_MID, "\tCount = %d, Period = %d ms", cnt, period);
	for (i=0; i<cnt; i++)
	{
		if (flag_send_fin == true)
		{
			break;
		}
		else
		{
		Debug_Msg_Print(DEBUG_MSG_LV_MID, "\tCount = %d / %d, Period = %d ms", i+1, cnt, period);
		Debug_Msg_Print_Data(DEBUG_MSG_LV_MID, buf, send_len);
		n = send(fd, buf, send_len, 0);
		if (n < 0)
		{
			perror("send() failed");
		}
		else if (n != send_len)
		{
			fprintf(stderr, "send() sent a different number of bytes than expected\n");
		}
		else if (n == 0)
		{
			perror("closed socket");
			close(fd);
			return false;
		}
		usleep(period * 1000);
	}
	}
	cnt = 1;
	period = 100;
	flag_send_fin = false;

	return rtnVal;
}

void* Cmd_thread_func(void *data)
{
	int fd = *(int*)data;
  
	while (state)
	{
		/* Test Application call*/
		if (Test_App_Main(fd) == false)
		{
			state = false;
			fprintf(stderr, "CMD thread Exit\n");
			return 0;
		}

		usleep(5000);
	}
}

/**
  * @name   AnalyzeExtMsg
  * @brief  Extensible Message 로그 출력, 기본적으로 tab으로 띈 후에 출력
  * @param  void *p : TLVC 포인터
  * @return int : 성공 - 0, 실패 - 음수
 **/
static void PrintExtStatusMsg(void *p)
{
	TLVC_STATUS_Tx_ModemUnit *p_tx_modem;
	TLVC_STATUS_Rx_ModemUnit *p_rx_modem;
	TLVC_STATUS_CommUnit *p_comm;
	TLVC_STATUS_ControlUnit *p_control;
	uint8_t *dev_type = (uint8_t*)p + 6;	// T, L 뒤에 dev_type 존재
	char buf[32];
	uint16_t *crc, cal_crc;
	
	switch(*dev_type)
	{
	  case eStatusDevType_ObuModem:
	  {
	  	p_tx_modem = (TLVC_STATUS_Tx_ModemUnit*)p;
		p_rx_modem = (TLVC_STATUS_Rx_ModemUnit*)p;
		int tx_rx = p_tx_modem->tx_rx;

		if (tx_rx == eStatusTxRx_Tx)
		{
			printf("\tObu Modem : Tx\n");
			printf("\tDevice ID : %u\n", htonl(p_tx_modem->dev_id));
			printf("\tVersion - HW : %d / SW : %d\n", htons(p_tx_modem->hw_ver), htons(p_tx_modem->sw_ver));
			printf("\tTx Power - %d, Freq - %d, Bandwidth - %d, Mcs - %d, Scs - %d\n",
					p_tx_modem->tx_power, htons(p_tx_modem->freq), p_tx_modem->bandwidth, p_tx_modem->mcs, p_tx_modem->scs);
			printf("\tLatitude - %d, Longitude - %d\n", htonl(p_tx_modem->latitude), htonl(p_tx_modem->longitude));
			sprintf(buf, "%lu", be64toh(p_tx_modem->timestamp));
			printf("\tTimestamp - %s\n", buf);
			cal_crc = CalcCRC16((uint8_t*)p_tx_modem, htons(p_tx_modem->len) + 4);	// T, L, V 길이
			if(cal_crc != ntohs(p_tx_modem->crc))
				printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_tx_modem->crc), cal_crc);
			}
		else if (tx_rx == eStatusTxRx_Rx)
		{
			printf("\tObu Modem : Rx\n");
			printf("\tDevice ID : %u\n", htonl(p_rx_modem->dev_id));
			printf("\tVersion - HW : %d / SW : %d\n", htons(p_rx_modem->hw_ver), htons(p_rx_modem->sw_ver));
			printf("\tRSSI - %d, RCPI - %d\n", p_rx_modem->rssi, p_rx_modem->rcpi);
			printf("\tLatitude - %d, Longitude - %d\n", htonl(p_rx_modem->latitude), htonl(p_rx_modem->longitude));
			sprintf(buf, "%lu", be64toh(p_rx_modem->timestamp));
			printf("\tTimestamp - %s\n", buf);
			cal_crc = CalcCRC16((uint8_t*)p_rx_modem, htons(p_rx_modem->len) + 4);	// T, L, V 길이
			if(cal_crc != ntohs(p_rx_modem->crc))
				printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_rx_modem->crc), cal_crc);
			}
		else
		{
			printf("Tx_Rx Type Error - %d\n", tx_rx);
			return;
		}
		
	  	break;
	  }

	  case eStatusDevType_Obu:
	  {
	  	p_comm = (TLVC_STATUS_CommUnit*)p;

		printf("\tOBU : %s\n", (p_comm->tx_rx==eStatusTxRx_Tx)?"Tx":"Rx");
		printf("\tDevice ID : %u\n", htonl(p_comm->dev_id));
		printf("\tVersion - HW : %d / SW : %d\n", htons(p_comm->hw_ver), htons(p_comm->sw_ver));
		sprintf(buf, "%lu", be64toh(p_comm->timestamp));
		printf("\tTimestamp - %s\n", buf);
		cal_crc = CalcCRC16((uint8_t*)p_comm, htons(p_comm->len) + 4);	// T, L, V 길이
		if(cal_crc != ntohs(p_comm->crc))
			printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_comm->crc), cal_crc);
	  	break;
	  }

	  case eStatusDevType_Rsu:
	  {
	  	p_comm = (TLVC_STATUS_CommUnit*)p;

		printf("\tRSU : %s\n", (p_comm->tx_rx==eStatusTxRx_Tx)?"Tx":"Rx");
		printf("\tDevice ID : %u\n", htonl(p_comm->dev_id));
		printf("\tVersion - HW : %d / SW : %d\n", htons(p_comm->hw_ver), htons(p_comm->sw_ver));
		sprintf(buf, "%lu", be64toh(p_comm->timestamp));
		printf("\tTimestamp - %s\n", buf);
		cal_crc = CalcCRC16((uint8_t*)p_comm, htons(p_comm->len) + 4);	// T, L, V 길이
		if(cal_crc != ntohs(p_comm->crc))
			printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_comm->crc), cal_crc);
	  	break;
	  }

	  case eStatusDevType_RsuModem:
	  {
	  	p_tx_modem = (TLVC_STATUS_Tx_ModemUnit*)p;
		p_rx_modem = (TLVC_STATUS_Rx_ModemUnit*)p;
		int tx_rx = p_tx_modem->tx_rx;

		if (tx_rx == eStatusTxRx_Tx)
		{
			printf("\tRsu Modem : Tx\n");
			printf("\tDevice ID : %u\n", htonl(p_tx_modem->dev_id));
			printf("\tVersion - HW : %d / SW : %d\n", htons(p_tx_modem->hw_ver), htons(p_tx_modem->sw_ver));
			printf("\tTx Power - %d, Freq - %d, Bandwidth - %d, Mcs - %d, Scs - %d\n",
					p_tx_modem->tx_power, htons(p_tx_modem->freq), p_tx_modem->bandwidth, p_tx_modem->mcs, p_tx_modem->scs);
			printf("\tLatitude - %d, Longitude - %d\n", htonl(p_tx_modem->latitude), htonl(p_tx_modem->longitude));
			sprintf(buf, "%lu", be64toh(p_tx_modem->timestamp));
			printf("\tTimestamp - %s\n", buf);
			cal_crc = CalcCRC16((uint8_t*)p_tx_modem, htons(p_tx_modem->len) + 4);	// T, L, V 길이
			if(cal_crc != ntohs(p_tx_modem->crc))
				printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_tx_modem->crc), cal_crc);
			}
		else if (tx_rx == eStatusTxRx_Rx)
		{
			printf("\tRsu Modem : Rx\n");
			printf("\tDevice ID : %u\n", htonl(p_rx_modem->dev_id));
			printf("\tVersion - HW : %d / SW : %d\n", htons(p_rx_modem->hw_ver), htons(p_rx_modem->sw_ver));
			printf("\tRSSI - %d, RCPI - %d\n", p_rx_modem->rssi, p_rx_modem->rcpi);
			printf("\tLatitude - %d, Longitude - %d\n", htonl(p_rx_modem->latitude), htonl(p_rx_modem->longitude));
			sprintf(buf, "%lu", be64toh(p_rx_modem->timestamp));
			printf("\tTimestamp - %s\n", buf);
			cal_crc = CalcCRC16((uint8_t*)p_rx_modem, htons(p_rx_modem->len) + 4);	// T, L, V 길이
			if(cal_crc != ntohs(p_rx_modem->crc))
				printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_rx_modem->crc), cal_crc);
			}
		else
		{
			printf("Tx_Rx Type Error - %d\n", tx_rx);
			return;
		}

		break;
	  }

	  case eStatusDevType_RsuControl:
	  {
	  	p_control = (TLVC_STATUS_ControlUnit*)p;

		printf("\tRsu Control : %s\n", (p_control->tx_rx==eStatusTxRx_Tx)?"Tx":"Rx");
		printf("\tDevice ID : %u\n", htonl(p_control->dev_id));
		printf("\tVersion - HW : %d / SW : %d\n", htons(p_control->hw_ver), htons(p_control->sw_ver));
		sprintf(buf, "%lu", be64toh(p_control->timestamp));
		printf("\tTimestamp - %s\n", buf);
		cal_crc = CalcCRC16((uint8_t*)p_control, htons(p_control->len) + 4);	// T, L, V 길이
		if(cal_crc != ntohs(p_control->crc))
			printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_control->crc), cal_crc);
	  	break;
	  }

	  default:
	  {
	  	printf("[Error] Unknown Dev Type - %d\n", *dev_type);
	  	break;
	  }
	}
}

/**
  * @name   AnalyzeMsg
  * @brief  Extensible Message 로그 출력
  * @param  uint8_t *msg : 수신한 데이터
  * 		int len : 수신한 데이터 길이
  * @return int : 성공 - 0, 실패 - 음수
 **/
static int AnalyzeMsg(uint8_t *msg, int len)
{
	int i, overall_len, package_len, package_remain_len;
	uint16_t *crc, cal_crc;
	void *p;
	TLVC_Overall *p_overall = NULL;
	V2x_App_Hdr *hdr = (V2x_App_Hdr *)msg;
	V2x_App_RxMsg *rx_msg = (V2x_App_RxMsg *)hdr->data;
	int psid = ntohl(rx_msg->psid);
	int flag_extensible_msg = 0;

	if (len > 0)
	{
		p_overall = (TLVC_Overall *)rx_msg->data;
	}
	else
	{
		printf("[Error] Extensible Message Lenth : %d\n", len);
		return -1;
	}
	
	if(psid == EM_V2V_MSG)
	{
		printf("Get Extensible Message - V2V\n");
		flag_extensible_msg = 1;
	}
	else if (psid == EM_V2I_MSG)
	{
		printf("Get Extensible Message - V2I\n");
		flag_extensible_msg = 1;
	}
	else if (psid == EM_I2V_MSG)
	{
		printf("Get Extensible Message - I2V\n");
		flag_extensible_msg = 1;
	}
	else
	{
		printf("Get Normal Message - PSID(%u)\n", psid);
	}

	if (flag_extensible_msg)
	{
		if (ntohl(p_overall->type) != EM_PT_OVERALL)
		{
			printf("[Error] Overall Type - %u, need - %u\n", ntohl(p_overall->type), EM_PT_OVERALL);
			return -1;
		}

		overall_len = ntohs(p_overall->len);		// V, C 길이
		package_remain_len = package_len = p_overall->len_package;
		printf("Overall Package - Version : %d / Length : %d\n", p_overall->version, overall_len);
		printf("Number of Packages = %d / All Length of Package= %d\n", p_overall->num_package,
																ntohs(package_len));
		cal_crc = CalcCRC16((uint8_t*)p_overall, overall_len + 4);	// T, L, V 길이
		if(cal_crc != ntohs(p_overall->crc))
			printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_overall->crc), cal_crc);


		//AddExtStatusData(p_overall, eStatusTxRx_Rx);
		p = (uint8_t*)p_overall + sizeof(TLVC_Overall);	// next TLVC
		for (i=0; i<p_overall->num_package; i++)
		{
			V2x_App_Ext_TLVC *tlvc = (V2x_App_Ext_TLVC *)p;
			int tlvc_len = ntohs(tlvc->len);
			uint32_t tlvc_type = ntohl(tlvc->type);

			if (package_remain_len < tlvc_len)
			{
				printf("[ERROR] Remain Length - %d\n", tlvc_len);
				break;
			}
				
			if (tlvc_type == EM_PT_STATUS)
			{
				printf("Package : %d (Status Package)\n", i+1);
				PrintExtStatusMsg(p);
			}
			else
			{	
				printf("Package : %d\n\tPSID : %d, TLV lenth : %d\n", i+1, tlvc_type, tlvc_len + 6);
				Debug_Msg_Print_Data(DEBUG_MSG_LV_MID, (uint8_t*)p, tlvc_len + 6);   // 6: T, L 크기 추가
			}

			p = p + tlvc_len + 6; // 6: T, L 크기
			package_remain_len = package_remain_len - tlvc_len - 6; // 6: T, L 크기
		}
	}
	else
	{
		Debug_Msg_Print_Data(DEBUG_MSG_LV_MID, msg, len);
	}


	

	return 0;
}

int main(int argc, char *argv[])
{
	char msg[MAX_RX_PACKET_BY_OBU];
	char ip_addr[256];
	int fd, fd_cnt=0;
	struct sockaddr_in servaddr;
	struct pollfd polls[5];

	if (argc < 2)
	{
		printf("Wrong argument\n");
		fprintf(stderr, "<CMD> <IP>\n");
		return -1;
	}
	else
	{
		printf("IP : %s, Port : %d\n", argv[1], DEST_PORT);
		strcpy(ip_addr, argv[1]);
	}

	fd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
	if(fd < 0)
	{
		perror("cannot open socket");
		return -1;
	}else 
		Debug_Msg_Print(DEBUG_MSG_LV_MID, "open socket");

	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(ip_addr);
	servaddr.sin_port = htons(DEST_PORT);

	if (connect(fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		perror("connect() failed");
		return -1;
	}

	polls[0].fd = fd;
	polls[0].events = POLLIN;
	fd_cnt++;

#if 0
	// Change to NON-BLOCK socket
	int flags = fcntl(fd, F_GETFL, 0);
	if (flags == -1) {
		perror("fcntl F_GETFL failed");
		return -1;
	}

	flags |= O_NONBLOCK;
	if (fcntl(fd, F_SETFL, flags) == -1) {
		perror("fcntl F_SETFL failed");
		return -1;
	}
#endif

	int len, n;
	struct timeval t_val={1, 0}; //sec, msec
	int status = 0;
	int thr_rc = 0;
	int cnt = 0;

	srand(time(NULL));

	thr_rc = pthread_create(&pCmdThread, NULL, Cmd_thread_func, (void *)&fd);
	if (thr_rc < 0)
	{
		perror("cmd thread create error : ");
		return -1;
	}
	pthread_detach(pCmdThread);

	while(state)
    {
		int poll_ret, i;
		poll_ret = poll(polls, 1, 500);
		if (!poll_ret) // timeout
			continue;

		for (i=0; i<fd_cnt; i++)
		{
			if (polls[i].revents & POLLIN)
			{
				memset(msg, 0, sizeof(msg));
				//n = recv(polls[i].fd, msg, sizeof(msg), 0);
				n = read(polls[i].fd, msg, sizeof(msg));
				if(n > 0)
				{
					//printf("### CNT = %d ###\n", cnt++);
					Debug_Msg_Print(DEBUG_MSG_LV_LOW,"\n\n");
					Debug_Msg_Print(DEBUG_MSG_LV_LOW, "read() TCP read n = %d", n);
					//Debug_Msg_Print_Data(DEBUG_MSG_LV_MID, msg, n);
					AnalyzeMsg(msg, n);
				}
				else if (n == 0)
				{
					fprintf(stderr, "recv() connection closed by peer\n");
					state = false;
					break;
				}
				else
				{
					if( errno != EAGAIN && errno != EWOULDBLOCK )
					{
						perror("recv() failed");
						state = false;
						break;
					}
				}
			}
		}
		//n = read(fd, msg, BUF_SIZE);
    }
    //pthread_join(pCmdThread, (void **)&status);
	pthread_cancel(pCmdThread);
    Debug_Msg_Print(DEBUG_MSG_LV_MID, "ByeBye");
	close(fd);
    return 0;
}
