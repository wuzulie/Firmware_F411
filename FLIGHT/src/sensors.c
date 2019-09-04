#include <math.h>
#include "stdio.h"
#include "delay.h"
#include "config.h"
#include "config_param.h"
#include "ledseq.h"
#include "mpu6500.h"
#include "sensors.h"
#include "ak8963.h"
#include "bmp280.h"
#include "filter.h"
#include "axis.h"
#include "spl06.h"


/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 传感器控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速计采样个数 */

// MPU9250主机模式读取数据 缓冲区长度
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN       	8
#define SENSORS_BARO_STATUS_LEN		1
#define SENSORS_BARO_DATA_LEN		6
#define SENSORS_BARO_BUFF_LEN       (SENSORS_BARO_STATUS_LEN + SENSORS_BARO_DATA_LEN)

typedef struct
{
	Axis3f     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

BiasObj	gyroBiasRunning;
static Axis3f  gyroBias;

static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

static bool isInit = false;
static sensorData_t sensors;
static Axis3i16	gyroRaw;
static Axis3i16	accRaw;
static Axis3i16 magRaw;

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static bool isMPUPresent=false;
static bool isMagPresent=false;
static bool isBaroPresent=false;

enum {IDLE, BMP280, SPL06}baroType = IDLE;

static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);


/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
/*从队列读取磁力计数据*/
bool sensorsReadMag(Axis3f *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
/*从队列读取气压数据*/
bool sensorsReadBaro(baro_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}
/*传感器中断初始化*/
static void sensorsInterruptInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/*使能MPU6500中断*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	portDISABLE_INTERRUPTS();
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);
	portENABLE_INTERRUPTS();
}

/* 传感器器件初始化 */
void sensorsDeviceInit(void)
{
	i2cdevInit(I2C1_DEV);
	mpu6500Init(I2C1_DEV);	
	
	vTaskDelay(10);
	mpu6500Reset();	// 复位MPU6500
	vTaskDelay(20);	// 延时等待寄存器复位
	
	u8 temp = mpu6500GetDeviceID();
	if (temp == 0x38 || temp == 0x39)
	{
		isMPUPresent=true;
		printf("MPU9250 I2C connection [OK].\n");
	}
	else
	{
		printf("MPU9250 I2C connection [FAIL].\n");
	}
	
	mpu6500SetSleepEnabled(false);	// 唤醒MPU6500
	vTaskDelay(10);		
	mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);	// 设置X轴陀螺作为时钟	
	vTaskDelay(10);		// 延时等待时钟稳定	
	mpu6500SetTempSensorEnabled(true);	// 使能温度传感器	
	mpu6500SetIntEnabled(false);		// 关闭中断	
	mpu6500SetI2CBypassEnabled(true);	// 旁路模式，磁力计和气压连接到主IIC	
	mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);	// 设置陀螺量程	
	mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);// 设置加速计量程	
	mpu6500SetAccelDLPF(MPU6500_ACCEL_DLPF_BW_41);		// 设置加速计数字低通滤波

	mpu6500SetRate(0);// 设置采样速率: 1000 / (1 + 0) = 1000Hz
	mpu6500SetDLPFMode(MPU6500_DLPF_BW_98);// 设置陀螺数字低通滤波
	
	for (u8 i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}


#ifdef SENSORS_ENABLE_MAG_AK8963
	ak8963Init(I2C1_DEV);	//ak8963磁力计初始化
	if (ak8963TestConnection() == true)
	{
		isMagPresent = true;
		ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
		printf("AK8963 I2C connection [OK].\n");
	}
	else
	{
		printf("AK8963 I2C connection [FAIL].\n");
	}
#endif

	if (bmp280Init(I2C1_DEV) == true)//BMP280初始化
	{
		isBaroPresent = true;
		baroType = BMP280;
		vTaskDelay(100);
	}
	else if (SPL06Init(I2C1_DEV) == true)//SPL06初始化
	{
		isBaroPresent = true;
		baroType = SPL06;
		vTaskDelay(100);
	}
	else
	{
		isBaroPresent = false;
	}

	/*创建传感器数据队列*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
}
/*传感器偏置初始化*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}
/*传感器测试*/
bool sensorsTest(void)
{
	bool testStatus = true;

	if (!isInit)
	{
		printf("Uninitialized\n");
		testStatus = false;
	}

	testStatus&=isBaroPresent;
	
	return testStatus;
}

/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	u32 i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}

/* 传感器初始化 */
void sensorsInit(void)
{
	if(isInit) return;

	sensorsDataReady = xSemaphoreCreateBinary();/*创建传感器数据就绪二值信号量*/
	sensorsBiasObjInit(&gyroBiasRunning);
	sensorsDeviceInit();	/*传感器器件初始化*/
	sensorsInterruptInit();	/*传感器中断初始化*/
	
	isInit = true;
}
/*设置传感器从模式读取*/
static void sensorsSetupSlaveRead(void)
{
	mpu6500SetSlave4MasterDelay(19); 	// 从机读取速率: 100Hz = (1000Hz / (1 + 9))

	mpu6500SetI2CBypassEnabled(false);	//主机模式
	mpu6500SetWaitForExternalSensorEnabled(true); 	
	mpu6500SetInterruptMode(0); 		// 中断高电平有效
	mpu6500SetInterruptDrive(0); 		// 推挽输出
	mpu6500SetInterruptLatch(0); 		// 中断锁存模式(0=50us-pulse, 1=latch-until-int-cleared)
	mpu6500SetInterruptLatchClear(1); 	// 中断清除模式(0=status-read-only, 1=any-register-read)
	mpu6500SetSlaveReadWriteTransitionEnabled(false); // 关闭从机读写传输
	mpu6500SetMasterClockSpeed(13); 	// 设置i2c速度400kHz

#ifdef SENSORS_ENABLE_MAG_AK8963
	if (isMagPresent)
	{
		// 设置MPU6500主机要读取的寄存器
		mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); 	// 设置磁力计为0号从机
		mpu6500SetSlaveRegister(0, AK8963_RA_ST1); 				// 从机0需要读取的寄存器
		mpu6500SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); 	// 读取8个字节(ST1, x, y, z heading, ST2 (overflow check))
		mpu6500SetSlaveDelayEnabled(0, true);
		mpu6500SetSlaveEnabled(0, true);
	}
#endif


	if (isBaroPresent && baroType == BMP280)
	{
		// 设置MPU6500主机要读取BMP280的寄存器
		mpu6500SetSlaveAddress(1, 0x80 | BMP280_I2C_ADDR);		// 设置气压计状态寄存器为1号从机
		mpu6500SetSlaveRegister(1, BMP280_STAT_REG);			// 从机1需要读取的寄存器
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_STATUS_LEN);	// 读取1个字节
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);

		mpu6500SetSlaveAddress(2, 0x80 | BMP280_I2C_ADDR);		// 设置气压计数据寄存器为2号从机
		mpu6500SetSlaveRegister(2, BMP280_PRESSURE_MSB_REG);	// 从机2需要读取的寄存器
		mpu6500SetSlaveDataLength(2, SENSORS_BARO_DATA_LEN);	// 读取6个字节
		mpu6500SetSlaveDelayEnabled(2, true);
		mpu6500SetSlaveEnabled(2, true);
	}
	if (isBaroPresent && baroType == SPL06)
	{
		// 设置MPU6500主机要读取SPL06的寄存器
		mpu6500SetSlaveAddress(1, 0x80 | SPL06_I2C_ADDR);		// 设置气压计状态寄存器为1号从机
		mpu6500SetSlaveRegister(1, SPL06_MODE_CFG_REG);			// 从机1需要读取的寄存器
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_STATUS_LEN);	// 读取1个字节
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);

		mpu6500SetSlaveAddress(2, 0x80 | SPL06_I2C_ADDR);		// 设置气压计数据寄存器为2号从机
		mpu6500SetSlaveRegister(2, SPL06_PRESSURE_MSB_REG);		// 从机2需要读取的寄存器
		mpu6500SetSlaveDataLength(2, SENSORS_BARO_DATA_LEN);	// 读取6个字节
		mpu6500SetSlaveDelayEnabled(2, true);
		mpu6500SetSlaveEnabled(2, true);
	}

	mpu6500SetI2CMasterModeEnabled(true);	//使能mpu6500主机模式
	mpu6500SetIntDataReadyEnabled(true);	//数据就绪中断使能
}

/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}

/**
 * 根据样本计算重力加速度缩放因子
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}

/**
 * 计算陀螺方差
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/*处理气压计数据*/
void processBarometerMeasurements(const u8 *buffer)
{
	static float temp;
    static float pressure;
	
	if (baroType == BMP280)
	{
		// Check if there is a new data update
		if ((buffer[0] & 0x08)) /*bit3=1 转换完成*/
		{
			s32 rawPressure = (s32)((((u32)(buffer[1])) << 12) | (((u32)(buffer[2])) << 4) | ((u32)buffer[3] >> 4));
			s32 rawTemp = (s32)((((u32)(buffer[4])) << 12) | (((u32)(buffer[5])) << 4) | ((u32)buffer[6] >> 4));
			temp = bmp280CompensateT(rawTemp)/100.0f;		
			pressure = bmp280CompensateP(rawPressure)/25600.0f;			

//			pressureFilter(&pressure, &sensors.baro.pressure);	
			sensors.baro.pressure = pressure;
			sensors.baro.temperature = (float)temp;	/*单位度*/
			sensors.baro.asl = bmp280PressureToAltitude(&pressure) * 100.f;	/*转换成海拔*/
		}
	}
	else if (baroType == SPL06)
	{
		s32 rawPressure = (int32_t)buffer[1]<<16 | (int32_t)buffer[2]<<8 | (int32_t)buffer[3];
		rawPressure = (rawPressure & 0x800000) ? (0xFF000000 | rawPressure) : rawPressure;
		
		s32 rawTemp = (int32_t)buffer[4]<<16 | (int32_t)buffer[5]<<8 | (int32_t)buffer[6];
		rawTemp = (rawTemp & 0x800000) ? (0xFF000000 | rawTemp) : rawTemp;
		
		temp = spl0601_get_temperature(rawTemp);
		pressure = spl0601_get_pressure(rawPressure, rawTemp);
		sensors.baro.pressure = pressure / 100.0f;
		sensors.baro.temperature = (float)temp; /*单位度*/
		sensors.baro.asl = SPL06PressureToAltitude(sensors.baro.pressure) * 100.f; //cm
	}

}
/*处理磁力计数据*/
void processMagnetometerMeasurements(const uint8_t *buffer)
{
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) 
	{
		int16_t headingx = (((int16_t) buffer[2]) << 8) | buffer[1];
		int16_t headingy = (((int16_t) buffer[4]) << 8) | buffer[3];
		int16_t headingz = (((int16_t) buffer[6]) << 8) | buffer[5];

		sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
		sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
		sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;		
		magRaw.x = headingx;/*用于上传到上位机*/
		
		magRaw.y = headingy;
		magRaw.z = headingz;
	}
}
/*处理加速计和陀螺仪数据*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*注意传感器读取方向(旋转270°x和y交换)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	accRaw.x = ax;/*用于上传到上位机*/
	accRaw.y = ay;
	accRaw.z = az;
	gyroRaw.x = gx - gyroBias.x;
	gyroRaw.y = gy - gyroBias.y;
	gyroRaw.z = gz - gyroBias.z;

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*计算accScale*/
	}
	
	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf(gyroLpf, &sensors.gyro);	

	sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;	/*单位 g(9.8m/s^2)*/
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;

	applyAxis3fLpf(accLpf, &sensors.acc);
}
/*传感器任务*/
void sensorsTask(void *param)
{
	sensorsInit();	/*传感器初始化*/
	vTaskDelay(150);
	sensorsSetupSlaveRead();/*设置传感器从模式读取*/

	while (1)
	{
		if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
		{
			/*确定数据长度*/
			u8 dataLen = (u8) (SENSORS_MPU6500_BUFF_LEN +
				(isMagPresent ? SENSORS_MAG_BUFF_LEN : 0) +
				(isBaroPresent ? SENSORS_BARO_BUFF_LEN : 0));

			i2cdevRead(I2C1_DEV, MPU6500_ADDRESS_AD0_HIGH, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);
			
			/*处理原始数据，并放入数据队列中*/
			processAccGyroMeasurements(&(buffer[0]));

			if (isMagPresent)
			{
				processMagnetometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
			}
			if (isBaroPresent)
			{
				processBarometerMeasurements(&(buffer[isMagPresent ?
					SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6500_BUFF_LEN]));
			}
			
			vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
			xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
			xQueueOverwrite(gyroDataQueue, &sensors.gyro);
			if (isMagPresent)
			{
				xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
			}
			if (isBaroPresent)
			{
				xQueueOverwrite(barometerDataQueue, &sensors.baro);
			}
			xTaskResumeAll();
		}
	}	
}
/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)	
{	
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
	sensorsReadMag(&sensors->mag);
	sensorsReadBaro(&sensors->baro);
}

void __attribute__((used)) EXTI4_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}
/*二阶低通滤波*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (u8 i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
/*传感器数据校准*/
bool sensorsAreCalibrated()	
{
	return gyroBiasFound;
}
/*上位机获取读取原始数据*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag)
{
	*acc = accRaw;
	*gyro = gyroRaw;
	*mag = magRaw;
}

bool getIsMPU9250Present(void)
{
	bool value = isMPUPresent;
#ifdef SENSORS_ENABLE_MAG_AK8963
	value &= isMagPresent;
#endif
	return value;
}


bool getIsBaroPresent(void)
{
	return isBaroPresent;
}



