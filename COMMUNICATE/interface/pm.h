#ifndef __PM_H
#define __PM_H
#include <stdbool.h>
#include "atkp.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��Դ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define PM_BAT_LOW_VOLTAGE_FLY   			3.00f
#define PM_BAT_LOW_VOLTAGE_STATIC   		3.60f

#define PM_BAT_LOW_TIMEOUT   			(1000 * 5) 	/* 5s */

typedef enum
{
	battery,
	charging,
	charged,
	lowPower,
	shutDown,
	
} PMStates;


void pmInit(void);
bool pmTest(void);
void pmTask(void *param);
void pmSyslinkUpdate(atkp_t *slp);
float pmGetBatteryVoltage(void);
bool getIsLowpower(void);


#endif /* __PM_H */
