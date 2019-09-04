#include "exti.h"
#include "led.h"
#include "motors.h"
#include "debug_assert.h"
#include "module_mgt.h"
#include "ws2812.h"
#include "spi.h"


/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"		 
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 中断配置驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


static u32 sysTickCnt=0;

void nvicInit(void)
{
	NVIC_SetVectorTable(FIRMWARE_START_ADDR,0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

extern void xPortSysTickHandler(void);

/********************************************************
 *SysTick_Handler()
 *滴答定时器中断服务函数
*********************************************************/
void  SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
    {
        xPortSysTickHandler();	
    }else
	{
		sysTickCnt++;	/*调度开启之前计数*/
	}
}
/********************************************************
*getSysTickCnt()
*调度开启之前 返回 sysTickCnt
*调度开启之前 返回 xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}


/**
  * @brief  This function handles SVCall exception.
  */
//void  SVC_Handler(void)
//{
//}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
//void  PendSV_Handler(void)
//{
//}


/**
  * @brief  This function handles NMI exception.
  */
void  NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
__asm void HardFault_Handler(void)
{
	//http://www.st.com/mcu/forums-cat-6778-23.html
	//****************************************************
	//To test this application, you can use this snippet anywhere:
	// //Let's crash the MCU!
	// asm (" MOVS r0, #1 \n"
	// " LDM r0,{r1-r2} \n"
	// " BX LR; \n");
	PRESERVE8
    IMPORT printHardFault
    TST r14, #4
    ITE EQ
    MRSEQ R0, MSP
    MRSNE R0, PSP
    B printHardFault
}

void  printHardFault(u32* hardfaultArgs)
{
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;

	stacked_r0 = ((unsigned long) hardfaultArgs[0]);
	stacked_r1 = ((unsigned long) hardfaultArgs[1]);
	stacked_r2 = ((unsigned long) hardfaultArgs[2]);
	stacked_r3 = ((unsigned long) hardfaultArgs[3]);

	stacked_r12 = ((unsigned long) hardfaultArgs[4]);
	stacked_lr = ((unsigned long) hardfaultArgs[5]);
	stacked_pc = ((unsigned long) hardfaultArgs[6]);
	stacked_psr = ((unsigned long) hardfaultArgs[7]);


	printf("[Hard fault handler]\n");
	printf("R0 = %x\n", stacked_r0);
	printf("R1 = %x\n", stacked_r1);
	printf("R2 = %x\n", stacked_r2);
	printf("R3 = %x\n", stacked_r3);
	printf("R12 = %x\n", stacked_r12);
	printf("LR = %x\n", stacked_lr);
	printf("PC = %x\n", stacked_pc);
	printf("PSR = %x\n", stacked_psr);
	printf("BFAR = %x\n", (*((volatile unsigned int *)(0xE000ED38))));
	printf("CFSR = %x\n", (*((volatile unsigned int *)(0xE000ED28))));
	printf("HFSR = %x\n", (*((volatile unsigned int *)(0xE000ED2C))));
	printf("DFSR = %x\n", (*((volatile unsigned int *)(0xE000ED30))));
	printf("AFSR = %x\n", (*((volatile unsigned int *)(0xE000ED3C))));

	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);	/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}
/**
 * @brief  This function handles Memory Manage exception.
 */
void  MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}

/**
 * @brief  This function handles Bus Fault exception.
 */
void  BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}

/**
 * @brief  This function handles Usage Fault exception.
 */
void  UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);/*错误检测*/
	ledSet(ERR_LED2, 1);

	storeAssertSnapshotData(__FILE__, __LINE__);
	while (1)
	{}
}

/**
 * @brief  This function handles Debug Monitor exception.
 */
void  DebugMon_Handler(void)
{
}

static enum expModuleID lastModuleID = NO_MODULE;

/************************************************************
*光流模块和灯环模块共用DMA1_Stream4, 这里需要分开处理一下
*************************************************************/

void  DMA1_Stream4_IRQHandler(void)
{
	if(getModuleID() == LED_RING)
	{
		lastModuleID = LED_RING;
		ws2812DmaIsr();
	}		
	else if(getModuleID() == OPTICAL_FLOW)
	{
		lastModuleID = OPTICAL_FLOW;
		spiTxDmaIsr();
	}
	else if(getModuleID() == NO_MODULE)
	{
		if(lastModuleID == LED_RING)
		{	
			ws2812DmaIsr();		
			DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, DISABLE);			
			DMA_Cmd(DMA1_Stream4,DISABLE);						
		}		
		else if(lastModuleID == OPTICAL_FLOW)
		{
			spiTxDmaIsr();
			DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, DISABLE);
			DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, DISABLE);
			DMA_Cmd(DMA1_Stream3,DISABLE);
			DMA_Cmd(DMA1_Stream4,DISABLE);			
		}				
	}
}

