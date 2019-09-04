#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include "sys.h"
#include <stdbool.h>

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

#define WATCHDOG_RESET_MS 	150	/*看门狗复位时间*/
#define watchdogReset() 	(IWDG_ReloadCounter())


void watchdogInit(u16 xms);
bool watchdogTest(void);


#endif 

