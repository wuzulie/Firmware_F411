#include <stdbool.h>
#include <string.h>
#include "config.h"
#include "usblink.h"
#include "atkp.h"
#include "config_param.h"
#include "ledseq.h"
#include "pm.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * USB通信驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define USBLINK_TX_QUEUE_SIZE 	30 /*接收队列个数*/

static enum
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
}rxState;

static bool isInit;
static atkp_t rxPacket;
static xQueueHandle  txQueue;


/*usb连接初始化*/
void usblinkInit()
{
	if(isInit) return;
	
	usbd_cdc_vcp_Init();
	/*创建发送队列，USBLINK_TX_QUEUE_SIZE个消息*/
	txQueue = xQueueCreate(USBLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(txQueue);
	isInit = true;
}

/*usb连接发送atkpPacket*/
bool usblinkSendPacket(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(txQueue, p, 0);	
}

//获取剩余可用txQueue个数
int usblinkGetFreeTxQueuePackets(void)	
{
	return (USBLINK_TX_QUEUE_SIZE - uxQueueMessagesWaiting(txQueue));
}

//USB发送ATKPPacket任务
void usblinkTxTask(void *param)
{
	atkp_t p;
	u8 sendBuffer[64];
	u8 cksum;
	u8 dataLen;
	while(1)
	{
		xQueueReceive(txQueue, &p, portMAX_DELAY);
		
		sendBuffer[0] = UP_BYTE1;
		sendBuffer[1] = UP_BYTE2;
		sendBuffer[2] = p.msgID;
		sendBuffer[3] = p.dataLen;
		memcpy(&sendBuffer[4], p.data, p.dataLen);
		cksum = 0;
		for (int i = 0; i < p.dataLen+4; i++)
		{
			cksum += sendBuffer[i];
		}
		dataLen = p.dataLen + 5;
		sendBuffer[dataLen - 1] = cksum;
		usbsendData(sendBuffer, dataLen);
		ledseqRun(DATA_TX_LED, seq_linkup);
	}
}

//USB虚拟串口接收ATKPPacket任务
void usblinkRxTask(void *param)
{
	u8 c;
	u8 dataIndex = 0;
	u8 cksum = 0;
	rxState = waitForStartByte1;
	while(1)
	{
		if (usbGetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForStartByte1:
					rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
					cksum = c;
					break;
				case waitForStartByte2:
					rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
					cksum += c;
					break;
				case waitForMsgID:
					rxPacket.msgID = c;
					rxState = waitForDataLength;
					cksum += c;
					break;
				case waitForDataLength:
					if (c <= ATKP_MAX_DATA_SIZE)
					{
						rxPacket.dataLen = c;
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
						cksum += c;
					} else 
					{
						rxState = waitForStartByte1;
					}
					break;
				case waitForData:
					rxPacket.data[dataIndex] = c;
					dataIndex++;
					cksum += c;
					if (dataIndex == rxPacket.dataLen)
					{
						rxState = waitForChksum1;
					}
					break;
				case waitForChksum1:
					if (cksum == c)	/*所有校验正确*/
					{
						ledseqRun(DATA_RX_LED, seq_linkup);
						atkpReceivePacketBlocking(&rxPacket);
					} 
					else	/*校验错误*/
					{
						rxState = waitForStartByte1;	
						IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStartByte1;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*超时处理*/
		{
			rxState = waitForStartByte1;
		}
	}
}

