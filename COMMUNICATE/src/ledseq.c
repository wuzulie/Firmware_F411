#include <stdbool.h>
#include "led.h"
#include "ledseq.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

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

/* LED序列优先级 */
static ledseq_t const * sequences[] = 
{
	seq_lowbat,
	seq_charged,
	seq_charging,
	seq_calibrated,
	seq_alive,
	seq_linkup,
};

/*Led 序列*/
ledseq_t const seq_lowbat[] = 	/*电池低电压序列*/
{
	{ true, LEDSEQ_WAITMS(1000)},
	{    0, LEDSEQ_LOOP},
};
const ledseq_t seq_calibrated[] = /*传感器校准完成序列*/
{
	{ true, LEDSEQ_WAITMS(50)},
	{false, LEDSEQ_WAITMS(450)},
	{    0, LEDSEQ_LOOP},
};
const ledseq_t seq_alive[] = 	/*开机序列*/
{
	{ true, LEDSEQ_WAITMS(50)},
	{false, LEDSEQ_WAITMS(1950)},
	{    0, LEDSEQ_LOOP},
};
const ledseq_t seq_linkup[] = 	/*通信连接序列*/
{
	{ true, LEDSEQ_WAITMS(1)},
	{false, LEDSEQ_WAITMS(0)},
	{    0, LEDSEQ_STOP},
};
const ledseq_t seq_charged[] = 	/*电池充电完成序列*/
{
	{ true, LEDSEQ_WAITMS(1000)},
	{    0, LEDSEQ_LOOP},
};
ledseq_t const seq_charging[] = /*电池充电进行中序列*/
{
	{ true, LEDSEQ_WAITMS(200)},
	{false, LEDSEQ_WAITMS(800)},
	{    0, LEDSEQ_LOOP},
};

#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))
	
static void updateActive(led_e led);		/*更新led的最高优先级序列*/
static int getPrio(const ledseq_t *seq);	/*获取led优先级*/
static void runLedseq(xTimerHandle xTimer);

static bool isInit = false;
static bool ledseqEnabled = true;
static int activeSeq[LED_NUM];		/*每个LED对应的活动优先级序列*/
static int state[LED_NUM][SEQ_NUM];	/*每个LED对应的序列的当前位置*/

static xTimerHandle timer[LED_NUM];	/*定时器句柄*/
static xSemaphoreHandle ledseqSem;	/*信号量*/


void ledseqInit()
{
	int i,j;
	if(isInit) return;

	ledInit();
	
	/*初始化各个序列状态*/
	for(i=0; i<LED_NUM; i++) 
	{
		activeSeq[i] = LEDSEQ_STOP;
		for(j=0; j<SEQ_NUM; j++)
			state[i][j] = LEDSEQ_STOP;
	}
	
	/*创建软件定时器*/
	for(i=0; i<LED_NUM; i++)
		timer[i] = xTimerCreate("ledseqTimer", 1000, pdFALSE, (void*)i, runLedseq);

	vSemaphoreCreateBinary(ledseqSem);	/*创建一个2值信号量*/

	isInit = true;
}

bool ledseqTest(void)
{
	bool status;

	status = isInit & ledTest();
	ledseqEnable(true);
	return status;
}

void ledseqEnable(bool enable)
{
	ledseqEnabled = enable;
}

void ledseqSetTimes(ledseq_t *sequence, int onTime, int offTime)
{
	sequence[0].action = onTime;
	sequence[1].action = offTime;
}

/*运行led的sequence序列*/
void ledseqRun(led_e led, const ledseq_t *sequence)
{
	int prio = getPrio(sequence);	/*获取led优先级序列*/

	if(prio<0) return;

	xSemaphoreTake(ledseqSem, portMAX_DELAY);
	state[led][prio] = 0; 
	updateActive(led);
	xSemaphoreGive(ledseqSem);

	if(activeSeq[led] == prio)	/*当前序列优先级等于活动序列优先级*/
		runLedseq(timer[led]);
}

/*停止led的sequence序列*/
void ledseqStop(led_e led, const ledseq_t *sequence)
{
	int prio = getPrio(sequence);

	if(prio<0) return;

	xSemaphoreTake(ledseqSem, portMAX_DELAY);
	state[led][prio] = LEDSEQ_STOP;  
	updateActive(led);
	xSemaphoreGive(ledseqSem);

	runLedseq(timer[led]);
}

/*FreeRTOS 定时器回调函数*/
static void runLedseq( xTimerHandle xTimer )
{
	bool leave = false;
	const ledseq_t *step;
	led_e led = (led_e)pvTimerGetTimerID(xTimer);

	if (!ledseqEnabled) return;

	while(!leave) 
	{
		int prio = activeSeq[led];

		if (prio == LEDSEQ_STOP)
			return;

		step = &sequences[prio][state[led][prio]];

		state[led][prio]++;

		xSemaphoreTake(ledseqSem, portMAX_DELAY);
		switch(step->action)
		{
			case LEDSEQ_LOOP:
				state[led][prio] = 0;
				break;
			case LEDSEQ_STOP:
				state[led][prio] = LEDSEQ_STOP;
				updateActive(led);
				break;
			default:  /*LED定时*/
				ledSet(led, step->value);	/*定时step->value*/
				if (step->action == 0)
					break;
				xTimerChangePeriod(xTimer, step->action, 0);
				xTimerStart(xTimer, 0);
				leave=true;
				break;
		}
		xSemaphoreGive(ledseqSem);
	}
}

/*获取led序列优先级*/
static int getPrio(const ledseq_t *seq)
{
	int prio;

	for(prio=0; prio<SEQ_NUM; prio++)
		if(sequences[prio]==seq) return prio;

	return -1; /*无效序列*/
}

/*更新led的最高优先级序列*/
static void updateActive(led_e led)
{
	int prio;

	ledSet(led, false);
	activeSeq[led]=LEDSEQ_STOP;
	
	for(prio=0;prio<SEQ_NUM;prio++)
	{
		if (state[led][prio] != LEDSEQ_STOP)
		{
			activeSeq[led]=prio;
			break;
		}
	}
}
