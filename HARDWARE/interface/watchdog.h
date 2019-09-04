#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���Ź���������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define WATCHDOG_RESET_MS 	150	/*���Ź���λʱ��*/
#define watchdogReset() 	(IWDG_ReloadCounter())


void watchdogInit(u16 xms);
bool watchdogTest(void);


#endif 

