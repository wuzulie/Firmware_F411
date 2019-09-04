#include "module_detect.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 扩展模块检测驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define  ADC_SAMPLE_NUM		10	 //采样次数

#define  ADC_LED_RING		2048 //RGB灯坏模块的R1:R2 = 10K :10K
#define  ADC_WIFI_CAMERA	4095 //摄像头模块的R1 = 10K 
#define  ADC_OPTICAL_FLOW	2815 //光流模块2的R1:R2 = 10K :22K
#define  ADC_MODULE1		1280 //MODULE1模块1的R1:R2 = 22K :10K

#define  ADC_MODULE_RANGE	50	 //允许模块电压变化范围值

static GPIO_InitTypeDef  GPIO_InitStructure;
static enum expModuleID moduleID = NO_MODULE;
static u16 adcValue[ADC_SAMPLE_NUM];

static u32 my_abs(int value)
{
	return (value >=0 ? value : -value);
}
	
void expModuleDriverInit(void)	/*扩展模块驱动初始化*/
{
	ADC_InitTypeDef  		ADC_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	DMA_InitTypeDef       	DMA_InitStructure;
	//GPIO_InitTypeDef      	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//使能PORTB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  	//使能ADC1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	//使能DMA时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;					//PB1
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;				//模拟输入
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;        	//不上拉、下拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              		//初始化PB1
	
	/*DMA2_Stream0 channel0 配置*/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&adcValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);//使能DMA2_Stream0
	
	/*ADC1 配置*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //多个ADC模式DMA失能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=100/4=25Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,1,ADC_SampleTime_480Cycles);//配置PB1规则通道9
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);//使能DMA
	ADC_Cmd(ADC1, ENABLE);//使能AD转换器
	ADC_SoftwareStartConv(ADC1);//开启转换
}

enum expModuleID getModuleDriverID(void)
{	
	ADC_Cmd(ADC1, DISABLE);//关闭AD转换器
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;					//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//输入模式
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              		//初始化PB1
	u8 state1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);	//读取状态1
	
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;        	//下拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              		//初始化PB1
	u8 state2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);	//读取状态2
	
	if(state1==SET && state2==RESET)//没有检测到模块插上
	{
		moduleID = NO_MODULE;
	}
	else //检测到模块插上
	{
		u32 sum=0;
		for(int i=0; i<ADC_SAMPLE_NUM; i++)
		{
			sum += adcValue[i];
		}
		sum = sum/ADC_SAMPLE_NUM;
		
		if(my_abs(sum-ADC_LED_RING) <= ADC_MODULE_RANGE)	
			moduleID = LED_RING;
		else if(my_abs(sum-ADC_WIFI_CAMERA) <= ADC_MODULE_RANGE)		
			moduleID = WIFI_CAMERA;			
		else if(my_abs(sum-ADC_OPTICAL_FLOW) <= ADC_MODULE_RANGE)	
			moduleID = OPTICAL_FLOW;
		else if(my_abs(sum-ADC_MODULE1) <= ADC_MODULE_RANGE)		
			moduleID = MODULE1;
		else
			moduleID = NO_MODULE;
			
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;			//PB1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	//模拟输入模式
		GPIO_Init(GPIOB,&GPIO_InitStructure);           //初始化PB1

		ADC_Cmd(ADC1, ENABLE);		//使能AD转换器
		ADC_SoftwareStartConv(ADC1);//开启AD转换器
	}

	return moduleID;

}




