#ifndef __LEDSEQ_H
#define __LEDSEQ_H

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * LED��ˮ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <led.h>

/********************************
*led���ж���: 
*��ʱ��ֹͣ������ѭ��
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
