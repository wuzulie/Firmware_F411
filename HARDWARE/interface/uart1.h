#ifndef __UART1_H
#define __UART1_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * UART1��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


void uart1Init(u32 baudrate);	/*����1��ʼ��*/
bool uart1Test(void);			/*����1����*/
bool uart1GetDataWithTimout(u8 *c);	/*����ʽ����һ���ַ�*/		
void uart1SendData(uint32_t size, u8* data);	/*����ԭʼ����*/
int uart1Putchar(int ch);
void uart1Getchar(char * ch);

#endif /* __UART1_H */
