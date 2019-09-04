#ifndef __VL53L1X_H
#define __VL53L1X_H

#include "vl53l1_platform.h"

#include "stabilizer_types.h"
#include "module_mgt.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly	
 * vl53l1x�ײ���������  ��ֲ��ST�ٷ�������
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/10/25
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define VL53L1X_MAX_RANGE			410		//410cm

#define VL53L1X_ADDR 				0x52
#define VL53L1X_DEFAULT_ADDRESS 	0x29	//0b0101001

#define VL53L1X_ID			0xEACC

extern VL53L1_Dev_t	dev;	/*vl53l1x �豸*/
int vl53l1xSetParam(void);	/*����vl53l1x ����*/

#endif /* __VL53L1X_H */

