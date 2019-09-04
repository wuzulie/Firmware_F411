#ifndef __BMP280_H
#define __BMP280_H
#include "stm32f4xx.h"
#include "i2cdev.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * BMP280驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define BMP280_I2C_ADDR					(0x76)
#define BMP280_DEFAULT_CHIP_ID			(0x58)

#define BMP280_CHIP_ID					(0xD0)  /* Chip ID Register */
#define BMP280_RST_REG					(0xE0)  /* Softreset Register */
#define BMP280_STAT_REG					(0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG			(0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG				(0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG			(0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG			(0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG		(0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG		(0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG		(0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG		(0xFC)  /* Temperature XLSB Reg */

#define BMP280_SLEEP_MODE				(0x00)
#define BMP280_FORCED_MODE				(0x01)
#define BMP280_NORMAL_MODE				(0x03)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE			(6)

#define BMP280_OVERSAMP_SKIPPED			(0x00)
#define BMP280_OVERSAMP_1X				(0x01)
#define BMP280_OVERSAMP_2X				(0x02)
#define BMP280_OVERSAMP_4X				(0x03)
#define BMP280_OVERSAMP_8X				(0x04)
#define BMP280_OVERSAMP_16X				(0x05)


bool bmp280Init(I2C_Dev *i2cPort);
void bmp280GetData(float* pressure, float* temperature, float* asl);

u32 bmp280CompensateT(s32 adcT);
u32 bmp280CompensateP(s32 adcP);
void pressureFilter(float* in, float* out);/*限幅平均滤波法*/
float bmp280PressureToAltitude(float* pressure/*, float* groundPressure, float* groundTemp*/);

#endif


