#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * LED驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define LED_NUM 5

typedef enum 
{
	LED_BLUE_L = 0, 
	LED_GREEN_L, 
	LED_RED_L, 
	LED_GREEN_R, 
	LED_RED_R,
} led_e;

#define DATA_RX_LED		LED_GREEN_L	/*无线数据接收指示灯*/
#define DATA_TX_LED		LED_RED_L	/*无线数据发送指示灯*/
#define CHG_LED 		LED_BLUE_L	/*充电指示灯*/
#define LOWBAT_LED		LED_RED_R	/*电池低电量指示灯*/
#define SYS_LED   		LED_GREEN_R	/*系统心跳指示灯*/
#define ERR_LED1       	LED_RED_L	/*出错指示灯1*/
#define ERR_LED2       	LED_RED_R	/*出错指示灯2*/


void ledInit(void);		/* LED初始化 */
bool ledTest(void);		/* LED测试 */
void ledSetAll(void);	/* 开启所有LED */
void ledClearAll(void);	/* 关闭所有LED */
void ledSet(led_e led, bool value);	/* 设置某个LED的状态 */
void ledFlashOne(led_e led, u32 onTime, u32 offTime);	/*LED闪烁1次*/


#endif
