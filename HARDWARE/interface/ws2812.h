#ifndef __WS2812_H__
#define __WS2812_H__
#include <stdbool.h>
#include "sys.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ws2812 RGB_LED��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


void ws2812Init(void);
void setHeadlightsOn(bool state);
void ws2812PowerControl(bool state);
void ws2812Send(uint8_t (*color)[3], uint16_t len);
void ws2812DmaIsr(void);

#endif /*__WS2812_H__*/
