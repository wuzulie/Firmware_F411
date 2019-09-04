#ifndef __UART_SYSLINK_H
#define __UART_SYSLINK_H
#include <stdbool.h>
#include "sys.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * usart2 串口通信驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define UARTSLK_TYPE             USART2	
#define UARTSLK_PERIF            RCC_APB1Periph_USART2
#define ENABLE_UARTSLK_RCC       RCC_APB1PeriphClockCmd
#define UARTSLK_IRQ              USART2_IRQn

#define UARTSLK_DMA_IRQ          DMA1_Stream6_IRQn	
#define UARTSLK_DMA_IT_TC        DMA_IT_TC
#define UARTSLK_DMA_STREAM       DMA1_Stream6
#define UARTSLK_DMA_CH           DMA_Channel_4
#define UARTSLK_DMA_IT_TCIF    	 DMA_IT_TCIF6

#define UARTSLK_GPIO_PERIF       RCC_AHB1Periph_GPIOA 
#define UARTSLK_GPIO_PORT        GPIOA
#define UARTSLK_GPIO_TX_PIN      GPIO_Pin_2
#define UARTSLK_GPIO_RX_PIN      GPIO_Pin_3
#define UARTSLK_GPIO_AF_TX_PIN   GPIO_PinSource2
#define UARTSLK_GPIO_AF_RX_PIN   GPIO_PinSource3
#define UARTSLK_GPIO_AF_TX       GPIO_AF_USART2
#define UARTSLK_GPIO_AF_RX       GPIO_AF_USART2

#define UARTSLK_TXEN_PERIF       RCC_AHB1Periph_GPIOA
#define UARTSLK_TXEN_PORT        GPIOA
#define UARTSLK_TXEN_PIN         GPIO_Pin_0
#define UARTSLK_TXEN_EXTI        EXTI_Line0


void uartslkInit(void);		/*串口初始化*/
bool uartslkTest(void);
bool uartslkGetDataWithTimout(u8 *c);	/*从接收队列读取数据(带超时处理)*/
void uartslkSendData(u32 size, u8* data);	/*发送原始数据*/
void uartslkSendDataIsrBlocking(u32 size, u8* data);/*中断方式发送原始数据*/
int uartslkPutchar(int ch);		/*发送一个字符到串口*/
void uartslkSendDataDmaBlocking(u32 size, u8* data);/*通过DMA发送原始数据*/
void uartslkIsr(void);		/*串口中断服务函数*/
void uartslkDmaIsr(void);	/*DMA中断服务函数*/
void uartslkTxenFlowctrlIsr(void);

#endif /* __UART_SYSLINK_H */
