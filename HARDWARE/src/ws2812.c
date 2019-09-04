#include <string.h>
#include "ws2812.h"

#include "FreeRTOS.h"
#include "semphr.h"

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

#define TIMING_ONE  80	// 0.8us电平时间
#define TIMING_ZERO 30	// 0.3us电平时间

u16 dmaBuffer0[24];
u16 dmaBuffer1[24];

static bool isInit = false;
static xSemaphoreHandle allLedDone = NULL;

//灯环ws2812初始化
void ws2812Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//使能PORTB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能TIM3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	//使能DMA时钟
	
	//初始化灯环电源控制脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				//PB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//输出模式
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	
	//初始化灯环蓝灯（眼睛灯）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//初始化灯环RGB灯
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				//PB4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);	//配置PB4为定时器3复用

	TIM_TimeBaseStructure.TIM_Period = (120 - 1); //800KHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
			 
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);//使能TIM3 CC2 DMA
	//TIM_Cmd(TIM3, ENABLE);                     
	
	DMA_DeInit(DMA1_Stream4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&TIM3->CCR1;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)dmaBuffer0;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_BufferSize = 24;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	
	DMA_DoubleBufferModeCmd(DMA1_Stream4, ENABLE);//使能双缓冲
	DMA_DoubleBufferModeConfig(DMA1_Stream4, (u32)dmaBuffer1, DMA_Memory_0);
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	if(!isInit)	/*首次接上灯环模块*/
	{
		vSemaphoreCreateBinary(allLedDone);
	}		
	else
	{
		xSemaphoreGive(allLedDone);
	}
		
	ws2812PowerControl(true);
	
	isInit = true;
}

//灯环前向灯开关控制（蓝色眼睛）
void setHeadlightsOn(bool state)
{
  if (state)
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
  else
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/*灯环电源控制*/
void ws2812PowerControl(bool state)
{
  if (state)
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
  else
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

//ws2812颜色填充
static void fillLed(u16 *buffer, u8 *color)
{
    int i;

    for(i=0; i<8; i++) // GREEN
	{
	    buffer[i] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // RED
	{
	    buffer[8+i] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // BLUE
	{
	    buffer[16+i] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
}

static int current_led = 0;
static int total_led = 0;
static u8(*color_led)[3] = NULL;
//ws2812颜色发送至DMA
void ws2812Send(u8 (*color)[3], u16 len)
{
	if(len<1) return;

	xSemaphoreTake(allLedDone, portMAX_DELAY);//等待上一次发送完成

	current_led = 0;
	total_led = len;
	color_led = color;
	
	fillLed(dmaBuffer0, color_led[current_led]);
	current_led++;
	fillLed(dmaBuffer1, color_led[current_led]);
	current_led++;
	
	DMA_Cmd(DMA1_Stream4, ENABLE);	//使能DMA
	TIM_Cmd(TIM3, ENABLE);			//使能定时器
}

//DMA中断处理
void ws2812DmaIsr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	if (total_led == 0)
	{
		TIM_Cmd(TIM3, DISABLE);
		DMA_Cmd(DMA1_Stream4, DISABLE);
	}
	
	if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
	{
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream4) == DMA_Memory_0)//DMA当前使用内存0
		{
			if (current_led<total_led)
				fillLed(dmaBuffer1, color_led[current_led]);
			else
				memset(dmaBuffer1, 0, sizeof(dmaBuffer1));
		}
		else//DMA当前使用内存1
		{
			if (current_led<total_led)
				fillLed(dmaBuffer0, color_led[current_led]);
			else		
				memset(dmaBuffer0, 0, sizeof(dmaBuffer0));
		}
		current_led++;
	}

	if (current_led >= total_led + 2) //多传输2个LED产生60us的低电平
	{
		xSemaphoreGiveFromISR(allLedDone, &xHigherPriorityTaskWoken);
		TIM_Cmd(TIM3, DISABLE); 					
		DMA_Cmd(DMA1_Stream4, DISABLE); 
		total_led = 0;
	}
}


