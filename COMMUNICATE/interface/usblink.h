#ifndef __USBLINK_H
#define __USBLINK_H
#include <stdbool.h>
#include "atkp.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * USBͨ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

void usblinkInit(void);
bool usblinkSendPacket(const atkp_t *p);
int usblinkGetFreeTxQueuePackets(void);
void usblinkRxTask(void *param);
void usblinkTxTask(void *param);


#endif /*usblink.h*/

