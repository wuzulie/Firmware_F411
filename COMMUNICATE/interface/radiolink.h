#ifndef __RADIO_H
#define __RADIO_H
#include <stdint.h>
#include <stdbool.h>
#include "atkp.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ����ͨ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

void radiolinkInit(void);
void radiolinkTask(void *param);
bool radiolinkSendPacket(const atkp_t *p);
bool radiolinkSendPacketBlocking(const atkp_t *p);
int radiolinkGetFreeTxQueuePackets(void);

#endif /*__RADIO_H */

