#include <math.h>
#include "config.h"
#include "config_param.h"
#include "debug_assert.h"
#include "iic1.h"
#include "i2cdev.h"
#include "mpu6500.h"
#include "delay.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * MPU6500驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static u8 buffer[14];
static bool mpu6500EvaluateSelfTest(float low, float high, float value, char* string);

static const unsigned short MPU6500_StTb[256] = 
{
	2620,2646,2672,2699,2726,2753,2781,2808, //7
	2837,2865,2894,2923,2952,2981,3011,3041, //15
	3072,3102,3133,3165,3196,3228,3261,3293, //23
	3326,3359,3393,3427,3461,3496,3531,3566, //31
	3602,3638,3674,3711,3748,3786,3823,3862, //39
	3900,3939,3979,4019,4059,4099,4140,4182, //47
	4224,4266,4308,4352,4395,4439,4483,4528, //55
	4574,4619,4665,4712,4759,4807,4855,4903, //63
	4953,5002,5052,5103,5154,5205,5257,5310, //71
	5363,5417,5471,5525,5581,5636,5693,5750, //79
	5807,5865,5924,5983,6043,6104,6165,6226, //87
	6289,6351,6415,6479,6544,6609,6675,6742, //95
	6810,6878,6946,7016,7086,7157,7229,7301, //103
	7374,7448,7522,7597,7673,7750,7828,7906, //111
	7985,8065,8145,8227,8309,8392,8476,8561, //119
	8647,8733,8820,8909,8998,9088,9178,9270,
	9363,9457,9551,9647,9743,9841,9939,10038,
	10139,10240,10343,10446,10550,10656,10763,10870,
	10979,11089,11200,11312,11425,11539,11654,11771,
	11889,12008,12128,12249,12371,12495,12620,12746,
	12874,13002,13132,13264,13396,13530,13666,13802,
	13940,14080,14221,14363,14506,14652,14798,14946,
	15096,15247,15399,15553,15709,15866,16024,16184,
	16346,16510,16675,16842,17010,17180,17352,17526,
	17701,17878,18057,18237,18420,18604,18790,18978,
	19167,19359,19553,19748,19946,20145,20347,20550,
	20756,20963,21173,21385,21598,21814,22033,22253,
	22475,22700,22927,23156,23388,23622,23858,24097,
	24338,24581,24827,25075,25326,25579,25835,26093,
	26354,26618,26884,27153,27424,27699,27976,28255,
	28538,28823,29112,29403,29697,29994,30294,30597,
	30903,31212,31524,31839,32157,32479,32804,33132
};
/*MPU6500初始化*/
bool mpu6500Init(I2C_Dev *i2cPort)
{
	u8 temp;
	bool state = false;
	
	I2Cx = i2cPort;
	devAddr = MPU6500_ADDR;
	
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1,0x80);	/*复位MPU*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1,0x80);	/*复位MPU*/
	
	delay_xms(50);
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_WHO_AM_I, &temp);	/*读取ID*/
	if (temp == 0x71 || temp == 0x73)	
	{
		printf("MPU9250 I2C connection [OK].\n");
		printf("MPU9250 ID IS: 0x%X\n",temp);
		state = true;
	} else
	{
		printf("MPU9250 I2C connection [FAIL].\n");
	}				
	
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_PWR_MGMT_1,0x03);		/*SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 1 (PLL with Z Gyro reference)*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_INT_ENABLE,0x0);		/*关闭中断*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_INT_PIN_CFG,0x02);	/*使能I2C BYPASS*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG,MPU6500_GYRO_FS_2000<<3);	/*陀螺仪量程±2000*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG,MPU6500_ACCEL_FS_8<<3);	/*加速计量程±8G*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_SMPLRT_DIV,1);				/*采样速率(1000/(1+1)=500Hz)*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_CONFIG,MPU6500_DLPF_BW_98);	/*数字低通滤波器带宽*/
	
	return state;
}
/** Evaluate the values from a MPU6500 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
static bool mpu6500EvaluateSelfTest(float low, float high, float value, char* string)
{
	if (value < low || value > high)
	{
		printf("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",string, low, high, value);
		return false;
	}
	return true;
}
/** Do a MPU6500 self test.
 * @return True if self test passed, false otherwise
 */
bool mpu6500SelfTest()
{
	u8 rawData[6] = {0, 0, 0, 0, 0, 0};
	u8 saveReg[5];
	u8 selfTest[6];
	int gAvg[3]={0}, aAvg[3]={0}, aSTAvg[3]={0}, gSTAvg[3]={0};
	int factoryTrim[6];
	float aDiff[3], gDiff[3];
	u8 FS = 0;
	int i;
	/*保存之前的配置*/
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_SMPLRT_DIV, &saveReg[0]);
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_CONFIG, &saveReg[1]);
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG, &saveReg[2]);
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG_2, &saveReg[3]);
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG, &saveReg[4]);
	/*写入新的的配置*/
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

	for(i = 0; i < 200; i++)
	{
		 // get average current values of gyro and acclerometer
		i2cdevRead(I2Cx, devAddr, MPU6500_RA_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		i2cdevRead(I2Cx, devAddr, MPU6500_RA_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)((int16_t)rawData[2] << 8) | rawData[3];
		gAvg[2] += (int16_t)((int16_t)rawData[4] << 8) | rawData[5];
	}

	for (i = 0; i < 3; i++)
	{ // Get average of 200 values and store as average current readings
		aAvg[i] /= 200;
		gAvg[i] /= 200;
	}

	// Configure the accelerometer for self-test
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay_xms(25); // Delay a while to let the device stabilize

	for(i = 0; i < 200; i++)
	{
		// get average self-test values of gyro and acclerometer
		i2cdevRead(I2Cx, devAddr, MPU6500_RA_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		i2cdevRead(I2Cx, devAddr, MPU6500_RA_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (i =0; i < 3; i++)
	{ // Get average of 200 values and store as average self-test readings
		aSTAvg[i] /= 200;
		gSTAvg[i] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG, 0x00);
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG, 0x00);
	delay_xms(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ST_X_ACCEL, &selfTest[0]); // X-axis accel self-test results
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ST_Y_ACCEL, &selfTest[1]); // Y-axis accel self-test results
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ST_Z_ACCEL, &selfTest[2]); // Z-axis accel self-test results
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ST_X_GYRO, &selfTest[3]); // X-axis gyro self-test results
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ST_Y_GYRO, &selfTest[4]); // Y-axis gyro self-test results
	i2cdevReadByte(I2Cx, devAddr, MPU6500_RA_ST_Z_GYRO, &selfTest[5]); // Z-axis gyro self-test results

	for (i = 0; i < 6; i++)
	{
		if (selfTest[i] != 0)
		{
			factoryTrim[i] = MPU6500_StTb[selfTest[i] - 1];
		}else
		{
			factoryTrim[i] = 0;
		}
	}

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (i = 0; i < 3; i++)
	{
		aDiff[i] = 100.0f*((float)((aSTAvg[i] - aAvg[i]) - factoryTrim[i]))/factoryTrim[i]; // Report percent differences
		gDiff[i] = 100.0f*((float)((gSTAvg[i] - gAvg[i]) - factoryTrim[i+3]))/factoryTrim[i+3]; // Report percent differences
	}

	// Restore old configuration
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_SMPLRT_DIV, saveReg[0]);
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_CONFIG, saveReg[1]);
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_GYRO_CONFIG, saveReg[2]);
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG_2, saveReg[3]);
	i2cdevWriteByte(I2Cx, devAddr, MPU6500_RA_ACCEL_CONFIG, saveReg[4]);

	// Check result
	if (mpu6500EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[0], "gyro X") &&
		mpu6500EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[1], "gyro Y") &&
		mpu6500EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[2], "gyro Z") &&
		mpu6500EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[0], "acc X") &&
		mpu6500EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[1], "acc Y") &&
		mpu6500EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[2], "acc Z"))
	{
		return true;
	}else
	{
		return false;
	}
}
/*获取6轴原始数据*/
void mpu6500GetRawData(s16* ax, s16* ay, s16* az, s16* gx, s16* gy, s16* gz)
{
	i2cdevRead(I2Cx, devAddr, MPU6500_RA_ACCEL_XOUT_H, 14, buffer);
	*ax = (((s16) buffer[0]) << 8) | buffer[1];
	*ay = (((s16) buffer[2]) << 8) | buffer[3];
	*az = (((s16) buffer[4]) << 8) | buffer[5];
	*gx = (((s16) buffer[8]) << 8) | buffer[9];
	*gy = (((s16) buffer[10]) << 8) | buffer[11];
	*gz = (((s16) buffer[12]) << 8) | buffer[13];
}




