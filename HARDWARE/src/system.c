#include "system.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 系统初始化函数	
 * 包括系统和底层硬件初始化
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static bool systemTest(void);


/*底层硬件初始化*/
void systemInit(void)
{
	u8 cnt = 0;
	
	nvicInit();			/*中断配置初始化*/
	extiInit();			/*外部中断初始化*/	
	delay_init(96);		/*delay初始化*/
	ledInit();			/*led初始化*/
	ledseqInit();		/*led灯序列初始化*/
	
	commInit();			/*通信初始化  STM32 & NRF51822 */
	atkpInit();			/*传输协议初始化*/
	consoleInit();		/*打印初始化*/
	printf("<--------------------->\n");
	
	configParamInit();	/*初始化配置参数*/
	pmInit();			/*电源管理初始化*/
	stabilizerInit();	/*电机 传感器 PID初始化*/
	expModuleDriverInit();/*扩展模块驱动初始化*/
	
	if(systemTest() == true)
	{	
		while(cnt++ < 5)	/*初始化通过 左上绿灯快闪5次*/
		{
			ledFlashOne(LED_GREEN_L, 50, 50);
		}			
	}else
	{		
		while(1)		/*初始化错误 右上红灯间隔1s快闪5次*/
		{
			if(cnt++ > 4)
			{
				cnt=0;
				delay_xms(1000);
			}
			ledFlashOne(LED_RED_R, 50, 50);		
		}
	}

	watchdogInit(WATCHDOG_RESET_MS);	/*看门狗初始化*/
	
}
static bool systemTest(void)
{
	bool pass = true;
	
	pass &= ledseqTest();
	pass &= pmTest();
	pass &= configParamTest();
	pass &= commTest();
	pass &= stabilizerTest();	
	pass &= watchdogTest();
	
	return pass;
}

