#ifndef __LEDSEQ_H
#define __LEDSEQ_H

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * LED流水灯驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <led.h>

/********************************
*led序列动作: 
*延时、停止、周期循环
*********************************/
#define LEDSEQ_WAITMS(X) (X)
#define LEDSEQ_STOP      -1
#define LEDSEQ_LOOP      -2

typedef struct 
{
	bool value;
	int action;
} ledseq_t;

extern const ledseq_t seq_calibrated[];
extern const ledseq_t seq_lowbat[];
extern const ledseq_t seq_alive[];
extern const ledseq_t seq_linkup[];
extern const ledseq_t seq_charged[];
extern const ledseq_t seq_charging[];


void ledseqInit(void);
bool ledseqTest(void);
void ledseqEnable(bool enable);
void ledseqRun(led_e led, const ledseq_t * sequence);
void ledseqStop(led_e led, const ledseq_t * sequence);
void ledseqSetTimes(ledseq_t *sequence, s32 onTime, s32 offTime);


#endif
