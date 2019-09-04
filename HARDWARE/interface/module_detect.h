#ifndef __EXP_MODULE_DRIVER_H
#define __EXP_MODULE_DRIVER_H
#include "sys.h"
#include <stdbool.h>
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��չģ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

enum expModuleID
{
	NO_MODULE,
	LED_RING,
	WIFI_CAMERA,
	OPTICAL_FLOW,
	MODULE1,
};


void expModuleDriverInit(void);
void expModuleDriverDmaIsr(void);
enum expModuleID getModuleDriverID(void);

#endif /* __EXP_MODULE_DRIVER_H */

