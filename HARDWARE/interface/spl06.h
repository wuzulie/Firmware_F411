#ifndef __SPL06_H
#define __SPL06_H
#include "stm32f4xx.h"
#include "i2cdev.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * SPL06驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define SPL06_I2C_ADDR					(0x76)
#define SPL06_DEFAULT_CHIP_ID			(0x10)

#define SPL06_PRESSURE_MSB_REG			(0x00)  /* Pressure MSB Register */
#define SPL06_PRESSURE_LSB_REG			(0x01)  /* Pressure LSB Register */
#define SPL06_PRESSURE_XLSB_REG			(0x02)  /* Pressure XLSB Register */
#define SPL06_TEMPERATURE_MSB_REG		(0x03)  /* Temperature MSB Reg */
#define SPL06_TEMPERATURE_LSB_REG		(0x04)  /* Temperature LSB Reg */
#define SPL06_TEMPERATURE_XLSB_REG		(0x05)  /* Temperature XLSB Reg */
#define SPL06_PRESSURE_CFG_REG			(0x06)	/* Pressure configuration Reg */
#define SPL06_TEMPERATURE_CFG_REG		(0x07)	/* Temperature configuration Reg */
#define SPL06_MODE_CFG_REG				(0x08)  /* Mode and Status Configuration */
#define SPL06_INT_FIFO_CFG_REG			(0x09)	/* Interrupt and FIFO Configuration */
#define SPL06_INT_STATUS_REG			(0x0A)	/* Interrupt Status Reg */
#define SPL06_FIFO_STATUS_REG			(0x0B)	/* FIFO Status Reg */
#define SPL06_RST_REG					(0x0C)  /* Softreset Register */
#define SPL06_CHIP_ID					(0x0D)  /* Chip ID Register */
#define SPL06_COEFFICIENT_CALIB_REG		(0x10)  /* Coeffcient calibraion Register */

#define SPL06_CALIB_COEFFICIENT_LENGTH	(18)
#define SPL06_DATA_FRAME_SIZE			(6)

#define SPL06_CONTINUOUS_MODE			(0x07)

#define TEMPERATURE_INTERNAL_SENSOR		(0)
#define TEMPERATURE_EXTERNAL_SENSOR		(1)

//测量次数 times / S
#define SPL06_MWASURE_1					(0x00)
#define SPL06_MWASURE_2					(0x01)
#define SPL06_MWASURE_4					(0x02)
#define SPL06_MWASURE_8					(0x03)
#define SPL06_MWASURE_16				(0x04)
#define SPL06_MWASURE_32				(0x05)
#define SPL06_MWASURE_64				(0x06)
#define SPL06_MWASURE_128				(0x07)

//过采样率
#define SPL06_OVERSAMP_1				(0x00)
#define SPL06_OVERSAMP_2				(0x01)
#define SPL06_OVERSAMP_4				(0x02)
#define SPL06_OVERSAMP_8				(0x03)
#define SPL06_OVERSAMP_16				(0x04)
#define SPL06_OVERSAMP_32				(0x05)
#define SPL06_OVERSAMP_64				(0x06)
#define SPL06_OVERSAMP_128				(0x07)

float spl0601_get_temperature(int32_t rawTemperature);
float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature);

bool SPL06Init(I2C_Dev *i2cPort);
void SPL06GetData(float* pressure, float* temperature, float* asl);
void pressureFilter(float* in, float* out);/*限幅平均滤波法*/
float SPL06PressureToAltitude(float pressure/*, float* groundPressure, float* groundTemp*/);

#endif


