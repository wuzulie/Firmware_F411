#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "config.h"
#include "syslink.h"
#include "radiolink.h"
#include "uart_syslink.h"
#include "config_param.h"
#include "pm.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * stm32,nrf51822ͨ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


static bool isInit = false;
static u8 sendBuffer[64];

static void syslinkRouteIncommingPacket(SyslinkPacket *slp);

static xSemaphoreHandle syslinkAccess;

/* Syslink task, handles communication between nrf and stm and dispatch messages
 */
void syslinkTask(void *param)
{
	SyslinkRxState rxState = waitForFirstStart;
	SyslinkPacket slp;
	u8 c;
	u8 dataIndex = 0;
	u8 cksum[2] = {0};

	radiolinkSetParam();	/*�������߲���*/
	
	while(1)
	{
		if (uartslkGetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForFirstStart:
					rxState = (c == SYSLINK_START_BYTE1) ? waitForSecondStart : waitForFirstStart;
					break;
				case waitForSecondStart:
					rxState = (c == SYSLINK_START_BYTE2) ? waitForType : waitForFirstStart;
					break;
				case waitForType:
					cksum[0] = c;
					cksum[1] = c;
					slp.type = c;
					rxState = waitForLengt;
					break;
				case waitForLengt:
					if (c <= SYSLINK_MTU)
					{
						slp.length = c;
						cksum[0] += c;
						cksum[1] += cksum[0];
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,���ݳ���Ϊ0��У��1*/
					} else 
					{
						rxState = waitForFirstStart;
					}
					break;
				case waitForData:
					slp.data[dataIndex] = c;
					cksum[0] += c;
					cksum[1] += cksum[0];
					dataIndex++;
					if (dataIndex == slp.length)	/*���ݽ�����ɣ�У��1*/
					{
						rxState = waitForChksum1;
					}
					break;
				case waitForChksum1:
					if (cksum[0] == c)	/*У��1��ȷ��׼��У��2*/
					{
						rxState = waitForChksum2;
					} else
					{
						rxState = waitForFirstStart;	/*У�����*/
						IF_DEBUG_ASSERT(1);
					}
					break;
				case waitForChksum2:
					if (cksum[1] == c)	/*����У����ȷ*/
					{
						syslinkRouteIncommingPacket(&slp);	/*�Խ��յ������ݰ����ദ��*/
					} else
					{
						rxState = waitForFirstStart;	/*У�����*/
						IF_DEBUG_ASSERT(1);
					}
					rxState = waitForFirstStart;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*��ʱ����*/
		{
			rxState = waitForFirstStart;
		}
	}
}
/*���ݰ����ദ��*/
static void syslinkRouteIncommingPacket(SyslinkPacket *slp)
{
	uint8_t groupType;

	groupType = slp->type & SYSLINK_GROUP_MASK;	/*type ��������*/
	switch (groupType)
	{
		case SYSLINK_RADIO_GROUP:	/*type[7:4]=0x00*/
			radiolinkSyslinkDispatch(slp);
			break;
		case SYSLINK_PM_GROUP:		/*type[7:4]=0x10*/
			pmSyslinkUpdate(slp);
			break;
		default:
			printf("Unknown packet:%X.\n", slp->type);
			break;
	}
}

/*
 * Public functions
 */

void syslinkInit()
{
	if(isInit) return;

	vSemaphoreCreateBinary(syslinkAccess);

	isInit = true;	
}

bool syslinkTest()
{
	return isInit;
}

int syslinkSendPacket(SyslinkPacket *slp)	/*������� ����DMA����*/
{
	int i = 0;
	int dataSize;
	uint8_t cksum[2] = {0};

	xSemaphoreTake(syslinkAccess, portMAX_DELAY);

	ASSERT(slp->length <= SYSLINK_MTU);

	sendBuffer[0] = SYSLINK_START_BYTE1;
	sendBuffer[1] = SYSLINK_START_BYTE2;
	sendBuffer[2] = slp->type;
	sendBuffer[3] = slp->length;

	memcpy(&sendBuffer[4], slp->data, slp->length);
	dataSize = slp->length + 6;
	// Calculate checksum delux
	for (i = 2; i < dataSize - 2; i++)
	{
		cksum[0] += sendBuffer[i];
		cksum[1] += cksum[0];
	}
	sendBuffer[dataSize-2] = cksum[0];
	sendBuffer[dataSize-1] = cksum[1];

	uartslkSendDataDmaBlocking(dataSize, sendBuffer);	/*����DMA����*/

	xSemaphoreGive(syslinkAccess);

	return 0;
}
