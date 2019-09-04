#ifndef __PM_H
#define __PM_H
#include <stdbool.h>
#include "atkp.h"
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 电源管理驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define PM_BAT_LOW_VOLTAGE   			3.35f
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
