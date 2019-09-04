#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * SPI��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

// Based on 48MHz peripheral clock
#define SPI_BAUDRATE_24MHZ  SPI_BaudRatePrescaler_2		// 24MHz
#define SPI_BAUDRATE_12MHZ  SPI_BaudRatePrescaler_4		// 12MHz
#define SPI_BAUDRATE_6MHZ   SPI_BaudRatePrescaler_8		// 6MHz
#define SPI_BAUDRATE_3MHZ   SPI_BaudRatePrescaler_16	// 3MHz
#define SPI_BAUDRATE_2MHZ   SPI_BaudRatePrescaler_32	// 1.5MHz

/**
 * ��ʼ��SPI.
 */
void spi2Init(void);
void spiBeginTransaction(void);
void spiEndTransaction(void);
void spiTxDmaIsr(void);

/* �շ��������� */
bool spiExchange(size_t length, const uint8_t *data_tx, uint8_t *data_rx);


#endif /* __SPI_H */

