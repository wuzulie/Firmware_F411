#include <math.h>
#include "stdbool.h"
#include "delay.h"
#include "config.h"
#include "bmp280.h"

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

/*bmp280 气压和温度过采样 工作模式*/
#define BMP280_PRESSURE_OSR			(BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR		(BMP280_OVERSAMP_8X)
#define BMP280_MODE					(BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE)


static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

typedef struct 
{
    u16 dig_T1;	/* calibration T1 data */
    s16 dig_T2; /* calibration T2 data */
    s16 dig_T3; /* calibration T3 data */
    u16 dig_P1;	/* calibration P1 data */
    s16 dig_P2; /* calibration P2 data */
    s16 dig_P3; /* calibration P3 data */
    s16 dig_P4; /* calibration P4 data */
    s16 dig_P5; /* calibration P5 data */
    s16 dig_P6; /* calibration P6 data */
    s16 dig_P7; /* calibration P7 data */
    s16 dig_P8; /* calibration P8 data */
    s16 dig_P9; /* calibration P9 data */
    s32 t_fine; /* calibration t_fine data */
} bmp280Calib;

bmp280Calib  bmp280Cal;

static u8 bmp280ID = 0;
static bool isInit = false;
static s32 bmp280RawPressure = 0;
static s32 bmp280RawTemperature = 0;

static void bmp280GetPressure(void);

bool bmp280Init(I2C_Dev *i2cPort)
{	
    if (isInit)
        return true;

	I2Cx = i2cPort;
	devAddr = BMP280_I2C_ADDR;

    delay_xms(50);
	
	i2cdevReadByte(I2Cx, devAddr, BMP280_CHIP_ID, &bmp280ID);	/* 读取bmp280 ID*/
	
	if(bmp280ID == BMP280_DEFAULT_CHIP_ID)
		printf("BMP280 ID IS: 0x%X\n",bmp280ID);
    else 
        return false;

    /* 读取校准数据 */
    i2cdevRead(I2Cx, devAddr, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (u8 *)&bmp280Cal);	
	i2cdevWriteByte(I2Cx, devAddr, BMP280_CTRL_MEAS_REG, BMP280_MODE);
	i2cdevWriteByte(I2Cx, devAddr, BMP280_CONFIG_REG, 5<<2);		/*配置IIR滤波*/
	
//	printf("BMP280 Calibrate Registor Are: \r\n");
//	for(i=0;i<24;i++)
//		printf("Registor %2d: 0x%X\n",i,p[i]);
	
    isInit = true;

    return true;
}

static void bmp280GetPressure(void)
{
    u8 data[BMP280_DATA_FRAME_SIZE];

    // read data from sensor
    i2cdevRead(I2Cx, devAddr, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
    bmp280RawPressure = (s32)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    bmp280RawTemperature = (s32)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
u32 bmp280CompensateT(s32 adcT)
{
    s32 var1, var2, T;

    var1 = ((((adcT >> 3) - ((s32)bmp280Cal.dig_T1 << 1))) * ((s32)bmp280Cal.dig_T2)) >> 11;
    var2  = (((((adcT >> 4) - ((s32)bmp280Cal.dig_T1)) * ((adcT >> 4) - ((s32)bmp280Cal.dig_T1))) >> 12) * ((s32)bmp280Cal.dig_T3)) >> 14;
    bmp280Cal.t_fine = var1 + var2;
	
    T = (bmp280Cal.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
u32 bmp280CompensateP(s32 adcP)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280Cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280Cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280Cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280Cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280Cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280Cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280Cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280Cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280Cal.dig_P7) << 4);
    return (uint32_t)p;
}

#define FILTER_NUM	5
#define FILTER_A	0.1f

/*限幅平均滤波法*/
void pressureFilter(float* in, float* out)
{	
	static u8 i=0;
	static float filter_buf[FILTER_NUM]={0.0};
	double filter_sum=0.0;
	u8 cnt=0;	
	float deta;		
	
	if(filter_buf[i] == 0.0f)
	{
		filter_buf[i]=*in;
		*out=*in;
		if(++i>=FILTER_NUM)	i=0;
	} else 
	{
		if(i) deta=*in-filter_buf[i-1];
		else deta=*in-filter_buf[FILTER_NUM-1];
		
		if(fabs(deta)<FILTER_A)
		{
			filter_buf[i]=*in;
			if(++i>=FILTER_NUM)	i=0;
		}
		for(cnt=0;cnt<FILTER_NUM;cnt++)
		{
			filter_sum+=filter_buf[cnt];
		}
		*out=filter_sum /FILTER_NUM;
	}
}

void bmp280GetData(float* pressure, float* temperature, float* asl)
{
    static float t;
    static float p;
	
	bmp280GetPressure();

	t = bmp280CompensateT(bmp280RawTemperature)/100.0;		
	p = bmp280CompensateP(bmp280RawPressure)/25600.0;		

	pressureFilter(&p,pressure);
	*temperature = (float)t;/*单位度*/
//	*pressure = (float)p ;	/*单位hPa*/	
	
	*asl=bmp280PressureToAltitude(pressure);	/*转换成海拔*/	
}

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float bmp280PressureToAltitude(float* pressure/*, float* groundPressure, float* groundTemp*/)
{	
    if(*pressure > 0)
    {
		return 44330.f * (powf((1015.7f / *pressure), 0.190295f) - 1.0f);
    }
    else
    {
        return 0;
    }
}
