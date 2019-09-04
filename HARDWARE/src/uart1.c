/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "sys.h"
#include <string.h>
#include "config.h"
#include "uart1.h"
#include "debug_assert.h"
#include "config.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * UART1驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define WIFI_DATA_LEN	8

static bool isInit = false;
static xQueueHandle uart1queue;


/*串口1初始化*/
void uart1Init(u32 baudrate)
{
	if(isInit)	return;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 使能GPIO和USART1时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* 配置USART Rx 为上拉输入 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 配置USART Tx 为复用功能输出 */
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 串口1对应引脚复用映射 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_USART1);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_USART1);
	

	USART_InitStructure.USART_BaudRate            = baudrate;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx ;	/* 接收模式 */
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 接收非空中断配置 */
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag(USART1, USART_FLAG_RXNE);
	USART_Cmd(USART1, ENABLE);

	uart1queue = xQueueCreate(64, sizeof(u8));
	
	isInit = true;
}

/*串口1测试*/
bool uart1Test(void)
{
	return isInit;
}


/*阻塞式接收一个字符*/
bool uart1GetDataWithTimout(u8 *c)
{
	if (xQueueReceive(uart1queue, c, 1000) == pdTRUE)
	{
		return true;
	}

	*c = 0;
	return false;
}

/*发送原始数据*/
void uart1SendData(u32 size, u8* data)
{
	u32 i;

	if (!isInit)
	return;

	for(i = 0; i < size; i++)
	{
		while (!(USART1->SR & USART_FLAG_TXE));
		USART1->DR = (data[i] & 0x00FF);
	}
}

int uart1Putchar(int ch)
{
    uart1SendData(1, (u8 *)&ch);  
    return (u8)ch;
}

void uart1Getchar(char * ch)
{
	xQueueReceive(uart1queue, ch, portMAX_DELAY);
}

/* USART1 中断服务函数 */
void USART1_IRQHandler(void)	
{
	u8 rxData;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetITStatus(USART1, USART_IT_RXNE))
	{
		rxData = USART_ReceiveData(USART1) & 0x00FF;
		xQueueSendFromISR(uart1queue, &rxData, &xHigherPriorityTaskWoken);
	}
}
