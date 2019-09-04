#include "string.h"
#include "stdbool.h"
#include "optical_flow.h"
#include "config_param.h"
#include "commander.h"
#include "delay.h"
#include "maths.h"
#include "state_estimator.h"

#include "filter.h"
#include "arm_math.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 光流模块驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 增加光流数据结构体opFlow_t，用于存放光流各项数据，方便用户调试。
********************************************************************************/

#define NCS_PIN					PAout(8)
#define OPTICAL_POWER_ENABLE	PBout(0)

#define RESOLUTION			(0.2131946f)/*1m高度下 1个像素对应的位移，单位cm*/
#define OULIER_LIMIT 		(100)		/*光流像素输出限幅*/
#define VEL_LIMIT			(150.f)		/*光流速度限幅*/

#define VEL_LPF_FILTER			/*低通滤波*/
//#define AVERAGE_FILTER		/*均值滤波*/

static bool isInit = false;
static u8 outlierCount = 0;			/*数据不可用计数*/

opFlow_t opFlow;	/*光流*/


TaskHandle_t opFlowTaskHandle = NULL;

#if defined(__CC_ARM) 
	#pragma anon_unions	/*用于支持结构体联合体*/
#endif

typedef __packed struct motionBurst_s 
{
	__packed union 
	{
		uint8_t motion;
		__packed struct 
		{
			uint8_t frameFrom0    : 1;
			uint8_t runMode       : 2;
			uint8_t reserved1     : 1;
			uint8_t rawFrom0      : 1;
			uint8_t reserved2     : 2;
			uint8_t motionOccured : 1;
		};
	};

	uint8_t observation;
	int16_t deltaX;
	int16_t deltaY;

	uint8_t squal;

	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
} motionBurst_t;

motionBurst_t currentMotion;

static void InitRegisters(void);

//光流电源控制
void opticalFlowPowerControl(bool state)
{
	if(state == true)
		OPTICAL_POWER_ENABLE = true;
	else
		OPTICAL_POWER_ENABLE = false;
}

static void registerWrite(uint8_t reg, uint8_t value)
{
	// 最高位为1 写寄存器
	reg |= 0x80u;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);
	spiExchange(1, &reg, &reg);
	delay_us(50);
	spiExchange(1, &value, &value);
	delay_us(50);

	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(200);
}

static uint8_t registerRead(uint8_t reg)
{
	uint8_t data = 0;

	// 最高位为0 读寄存器
	reg &= ~0x80u;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);	
	spiExchange(1, &reg, &reg);
	delay_us(500);
	spiExchange(1, &data, &data);	
	delay_us(50);
	
	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(200);

	return data;
}

static void readMotion(motionBurst_t * motion)
{
	uint8_t address = 0x16;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);	
	spiExchange(1, &address, &address);
	delay_us(50);
	spiExchange(sizeof(motionBurst_t), (uint8_t*)motion, (uint8_t*)motion);	
	delay_us(50);
	
	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(50);

	uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
	realShutter |= (motion->shutter & 0x0ff) << 8;
	motion->shutter = realShutter;
}

static void InitRegisters(void)
{	
	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5B, 0x32);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x65, 0x20);
	registerWrite(0x6A, 0x18);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x5F, 0x40);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x1F);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xFF);
	registerWrite(0x65, 0x1F);
	registerWrite(0x7F, 0x14);
	registerWrite(0x65, 0x67);
	registerWrite(0x66, 0x08);
	registerWrite(0x63, 0x70);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x02);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);
	registerWrite(0x70, 0x00);

	vTaskDelay(10); // delay 10ms

	registerWrite(0x32, 0x44);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x62, 0xF0);
	registerWrite(0x63, 0x00);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	
//	/*初始化LED_N*/
//	registerWrite(0x7F, 0x0E);
//	registerWrite(0x72, 0x0F);
//	registerWrite(0x7F, 0x00);
}
/*复位光流数据*/
static void resetOpFlowData(void)
{
	for(u8 i=0; i<2; i++)
	{
		opFlow.pixSum[i] = 0;
		opFlow.pixComp[i] = 0;
		opFlow.pixValid[i] = 0;
		opFlow.pixValidLast[i] = 0;
	}
}

/*光流任务函数*/
void opticalFlowTask(void *param)
{	
	static u16 count = 0;	
	u32 lastWakeTime = getSysTickCnt();
		
	opFlow.isOpFlowOk = true;
	
	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, 10);		/*100Hz 10ms周期延时*/
		
		readMotion(&currentMotion);

		if(currentMotion.minRawData == 0 && currentMotion.maxRawData == 0)
		{
			if(count++ > 100 && opFlow.isOpFlowOk == true)
			{
				count = 0;
				opFlow.isOpFlowOk = false;		/*光流出错*/
				vTaskSuspend(opFlowTaskHandle);	/*挂起光流任务*/
			}		
		}else
		{
			count = 0;
		}
		/*连续2帧之间的像素变化，根据实际安装方向调整 (pitch:x)  (roll:y)*/
		int16_t pixelDx = currentMotion.deltaY;
		int16_t pixelDy = -currentMotion.deltaX;

		if (ABS(pixelDx) < OULIER_LIMIT && ABS(pixelDy) < OULIER_LIMIT) 
		{
			opFlow.pixSum[X] += pixelDx;
			opFlow.pixSum[Y] += pixelDy;
		}else 
		{
			outlierCount++;
		}
	}
}

#ifdef AVERAGE_FILTER

#define GROUP		2
#define FILTER_NUM	3

/*限幅均值滤波法*/
void velFilter(float* in, float* out)
{	
	static u8 i=0;
	static float filter_buf[GROUP][FILTER_NUM] = {0.0};
	double filter_sum[GROUP] = {0.0};		
	
	for(u8 j=0;j<GROUP;j++)
	{
		if(filter_buf[j][i] == 0.0f)
		{
			filter_buf[j][i] = in[j];
			out[j] = in[j];			
		} else 
		{			
			if(fabs(in[j]) < VEL_LIMIT)
			{
				filter_buf[j][i] = in[j];
			}
			for(u8 cnt=0;cnt<FILTER_NUM;cnt++)
			{
				filter_sum[j] += filter_buf[j][cnt];
			}
			out[j]=filter_sum[j] /FILTER_NUM;
		}
	}
	if(++i >= FILTER_NUM)	i = 0;
}
#endif

bool getOpFlowData(state_t *state, float dt)
{
	static u8 cnt = 0;
	float height = 0.01f * getFusedHeight();/*读取高度信息 单位m*/
	
	if(opFlow.isOpFlowOk && height<4.0f)	/*4m范围内，光流可用*/
	{
		cnt= 0;
		opFlow.isDataValid = true;
		
		float coeff = RESOLUTION * height;
		float tanRoll = tanf(state->attitude.roll * DEG2RAD);
		float tanPitch = tanf(state->attitude.pitch * DEG2RAD);
		
		opFlow.pixComp[X] = 480.f * tanPitch;	/*像素补偿，负方向*/
		opFlow.pixComp[Y] = 480.f * tanRoll;
		opFlow.pixValid[X] = (opFlow.pixSum[X] + opFlow.pixComp[X]);	/*实际输出像素*/
		opFlow.pixValid[Y] = (opFlow.pixSum[Y] + opFlow.pixComp[Y]);		
		
		if(height < 0.05f)	/*光流测量范围大于5cm*/
		{
			coeff = 0.0f;
		}
		opFlow.deltaPos[X] = coeff * (opFlow.pixValid[X] - opFlow.pixValidLast[X]);	/*2帧之间位移变化量，单位cm*/
		opFlow.deltaPos[Y] = coeff * (opFlow.pixValid[Y] - opFlow.pixValidLast[Y]);	
		opFlow.pixValidLast[X] = opFlow.pixValid[X];	/*上一次实际输出像素*/
		opFlow.pixValidLast[Y] = opFlow.pixValid[Y];
		opFlow.deltaVel[X] = opFlow.deltaPos[X] / dt;	/*速度 cm/s*/
		opFlow.deltaVel[Y] = opFlow.deltaPos[Y] / dt;
		
#ifdef AVERAGE_FILTER
		velFilter(opFlow.deltaVel, opFlow.velLpf);		/*限幅均值滤波法*/
#else
		opFlow.velLpf[X] += (opFlow.deltaVel[X] - opFlow.velLpf[X]) * 0.15f;	/*速度低通 cm/s*/
		opFlow.velLpf[Y] += (opFlow.deltaVel[Y] - opFlow.velLpf[Y]) * 0.15f;	/*速度低通 cm/s*/
#endif			
		opFlow.velLpf[X] = constrainf(opFlow.velLpf[X], -VEL_LIMIT, VEL_LIMIT);	/*速度限幅 cm/s*/
		opFlow.velLpf[Y] = constrainf(opFlow.velLpf[Y], -VEL_LIMIT, VEL_LIMIT);	/*速度限幅 cm/s*/
	
		opFlow.posSum[X] += opFlow.deltaPos[X];	/*累积位移 cm*/
		opFlow.posSum[Y] += opFlow.deltaPos[Y];	/*累积位移 cm*/
	}
	else if(opFlow.isDataValid == true)
	{
		if(cnt++ > 100)	/*超过定点高度，切换为定高模式*/
		{
			cnt = 0;
			opFlow.isDataValid = false;
		}	
		resetOpFlowData();	
	}
	
	return opFlow.isOpFlowOk;	/*返回光流状态*/
}
/*初始化光流模块*/
void opticalFlowInit(void)
{
	if (!isInit) /*第一次初始化通用IO*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		//初始化CS引脚	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能时钟
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);		

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOB, &GPIO_InitStructure);		
	}
	else 
	{
		resetOpFlowData();
		opFlow.isOpFlowOk = true;				
	}
	
	opticalFlowPowerControl(true);	/*打开电源*/
	vTaskDelay(50);
	
	NCS_PIN = 1;
	spi2Init();
	vTaskDelay(40);

	uint8_t chipId = registerRead(0);
	uint8_t invChipId = registerRead(0x5f);
//	printf("Motion chip is: 0x%x\n", chipId);
//	printf("si pihc noitoM: 0x%x\n", invChipId);

	// 上电复位
	registerWrite(0x3a, 0x5a);
	vTaskDelay(5);

	InitRegisters();
	vTaskDelay(5);
	
	if (isInit) 
	{
		vTaskResume(opFlowTaskHandle);	/*恢复光流任务*/
	}
	else if(opFlowTaskHandle == NULL)
	{
		xTaskCreate(opticalFlowTask, "OPTICAL_FLOW", 300, NULL, 4, &opFlowTaskHandle);	/*创建光流模块任务*/
	}		

	vl53lxxInit();	/*初始化vl53lxx*/
	
	isInit = true;
}

/*获取光流数据状态*/
bool getOpDataState(void)
{
	return opFlow.isDataValid;
}
	
	

