#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * SPI驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

// Based on 48MHz peripheral clock
#define SPI_BAUDRATE_24MHZ  SPI_BaudRatePrescaler_2		// 24MHz
#define SPI_BAUDRATE_12MHZ  SPI_BaudRatePrescaler_4		// 12MHz
#define SPI_BAUDRATE_6MHZ   SPI_BaudRatePrescaler_8		// 6MHz
#define SPI_BAUDRATE_3MHZ   SPI_BaudRatePrescaler_16	// 3MHz
#define SPI_BAUDRATE_2MHZ   SPI_BaudRatePrescaler_32	// 1.5MHz

/**
 * 初始化SPI.
 */
void spi2Init(void);
void spiBeginTransaction(void);
void spiEndTransaction(void);
void spiTxDmaIsr(void);

/* 收发缓冲数据 */
bool spiExchange(size_t length, const uint8_t *data_tx, uint8_t *data_rx);


#endif /* __SPI_H */

