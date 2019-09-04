#include "config.h"
#include "watchdog.h"
#include "debug_assert.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 看门狗驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


bool watchdogTest(void)
{
	bool wasNormalStart = true;

	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST)) 
	{
		RCC_ClearFlag();
		wasNormalStart = false;
		printf("The system resumed after watchdog timeout [WARNING]\n");
		printAssertSnapshotData();
	}
	return wasNormalStart;
}


void watchdogInit(u16 xms)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* 47000/32Hz => 1.47  1ms*/
	IWDG_SetReload((u16)(1.47*xms));

	watchdogReset();
	IWDG_Enable();
}
