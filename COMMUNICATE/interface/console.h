#ifndef __CONSOLE_H
#define __CONSOLE_H
#include <stdbool.h>
#include "sys.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 数据打印驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

void consoleInit(void);
bool consoleTest(void);
int consolePutchar(int ch);	/* 输入一个字符到console缓冲区*/
int consolePutcharFromISR(int ch);	/* 中断方式输入一个字符到console缓冲区*/
int consolePuts(char *str);	/* 输入一个字符串到console缓冲区*/

#endif /*__CONSOLE_H*/
