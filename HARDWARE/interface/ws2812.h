#ifndef __WS2812_H__
#define __WS2812_H__
#include <stdbool.h>
#include "sys.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * ws2812 RGB_LED驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


void ws2812Init(void);
void setHeadlightsOn(bool state);
void ws2812PowerControl(bool state);
void ws2812Send(uint8_t (*color)[3], uint16_t len);
void ws2812DmaIsr(void);

#endif /*__WS2812_H__*/
