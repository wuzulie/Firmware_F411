#ifndef __FILTER_H
#define __FILTER_H
#include <stdint.h>

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 滤波功能函数	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define IIR_SHIFT         8

int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);

typedef struct 
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;
	float delay_element_2;
} lpf2pData;

void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
float lpf2pReset(lpf2pData* lpfData, float sample);


#endif //__FILTER_H
