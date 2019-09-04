#include <stdbool.h>
#include "console.h"
#include "atkp.h"
#include "radiolink.h"
#include "usblink.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "semphr.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���ݴ�ӡ��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


atkp_t messageToPrint;
xSemaphoreHandle synch = NULL;

static const char fullMsg[] = "<F>\n";
static bool isInit;


void consoleInit()
{
	if (isInit) return;

	messageToPrint.msgID = UP_PRINTF;
	messageToPrint.dataLen = 0;
	vSemaphoreCreateBinary(synch);

	isInit = true;
}

bool consoleTest(void)
{
	return isInit;
}
/* ����һ���ַ���console������*/
int consolePutchar(int ch)
{
	int i;
	bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

	if (!isInit) 
	{
		return 0;
	}
	if (isInInterrupt) 
	{
		return consolePutcharFromISR(ch);
	}
	if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
	{
		if (messageToPrint.dataLen < ATKP_MAX_DATA_SIZE)	/*#define ATKP_MAX_DATA_SIZE 30*/
		{
			messageToPrint.data[messageToPrint.dataLen] = (u8)ch;
			messageToPrint.dataLen++;
		}
		if (ch == '\n' || messageToPrint.dataLen >= ATKP_MAX_DATA_SIZE)	/*#define ATKP_MAX_DATA_SIZE 30*/
		{
			if (radiolinkGetFreeTxQueuePackets() == 1)
			{
				for (i = 0; i < sizeof(fullMsg) && (messageToPrint.dataLen - i) > 0; i++)
				{
					messageToPrint.data[messageToPrint.dataLen - i] = (u8)fullMsg[sizeof(fullMsg) - i - 1];
				}
			}
			radiolinkSendPacket(&messageToPrint);
			usblinkSendPacket(&messageToPrint);
			messageToPrint.dataLen = 0;
		}
		xSemaphoreGive(synch);
	}
	return (u8)ch;
}
/* �жϷ�ʽ����һ���ַ���console������*/
int consolePutcharFromISR(int ch) 
{
	BaseType_t higherPriorityTaskWoken;

	if (xSemaphoreTakeFromISR(synch, &higherPriorityTaskWoken) == pdTRUE) 
	{
		if (messageToPrint.dataLen < ATKP_MAX_DATA_SIZE)
		{
			messageToPrint.data[messageToPrint.dataLen] = (u8)ch;
			messageToPrint.dataLen++;
		}
		xSemaphoreGiveFromISR(synch, &higherPriorityTaskWoken);
	}

	return ch;
}
/* ����һ���ַ�����console������*/
int consolePuts(char *str)
{
	int ret = 0;

	while(*str)
		ret |= consolePutchar(*str++);

	return ret;
}

