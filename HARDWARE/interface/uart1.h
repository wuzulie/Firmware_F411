#ifndef __UART1_H
#define __UART1_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * UART1驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


void uart1Init(u32 baudrate);	/*串口1初始化*/
bool uart1Test(void);			/*串口1测试*/
bool uart1GetDataWithTimout(u8 *c);	/*阻塞式接收一个字符*/		
void uart1SendData(uint32_t size, u8* data);	/*发送原始数据*/
int uart1Putchar(int ch);
void uart1Getchar(char * ch);

#endif /* __UART1_H */
