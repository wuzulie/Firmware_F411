#include "sys.h"
#include "delay.h"
#include "motors.h"
#include "pm.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 电机驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0};
static const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };



static u16 ratioToCCRx(u16 val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

void motorsInit(void)	/*电机初始化*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);	//使能PORTA PORTB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4,ENABLE);  	//TIM2和TIM4时钟使能    
	
	TIM_DeInit(TIM4);	//重新初始化TIM4为默认状态
	TIM_DeInit(TIM2);	//重新初始化TIM2为默认状态
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); 	//PB7 复用为TIM4 CH2	MOTOR1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); 	//PB6 复用为TIM4 CH1	MOTOR2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2); 	//PB10复用为TIM2 CH3	MOTOR3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_TIM2); 	//PA5 复用为TIM2 CH1	MOTOR4
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10;	//PB6 7 10
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;        				//复用功能
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//速度100MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      				//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;        				//上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              				//初始化PB6 7 10
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;							//PA5
	GPIO_Init(GPIOA,&GPIO_InitStructure);              				//初始化PA5		
	
	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			//自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//时钟分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//重复计数次数
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);				//初始化TIM4
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);				//初始化TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWM模式1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//使能输出
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//高电平有效
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//空闲高电平	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  	//初始化TIM4 CH2输出比较
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  	//初始化TIM4 CH1输出比较
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  	//初始化TIM2 CH3输出比较
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  	//初始化TIM2 CH1输出比较
	
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR1上的预装载寄存器
 
	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPE使能 
	TIM_ARRPreloadConfig(TIM2,ENABLE);	//TIM2	ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2	

	isInit = true;
}

/*电机测试*/
bool motorsTest(void)
{
	int i;
	
	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{	
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		delay_xms(MOTORS_TEST_ON_TIME_MS);
		motorsSetRatio(MOTORS[i], 0);
		delay_xms(MOTORS_TEST_DELAY_TIME_MS);
	}

	return isInit;
}

extern bool isExitFlip;

/*设置电机PWM占空比*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		u16 ratio=ithrust;

	#ifdef ENABLE_THRUST_BAT_COMPENSATED		
		if(isExitFlip == true)		/*500Hz*/
		{
			float thrust = ((float)ithrust / 65536.0f) * 60;
			float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
			float supply_voltage = pmGetBatteryVoltage();
			float percentage = volts / supply_voltage;
			percentage = percentage > 1.0f ? 1.0f : percentage;
			ratio = percentage * UINT16_MAX;
			motor_ratios[id] = ratio;
		}		
	#endif
		
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM_SetCompare2(TIM4,ratioToCCRx(ratio));
				break;
			case 1:		/*MOTOR_M2*/
				TIM_SetCompare1(TIM4,ratioToCCRx(ratio));
				break;
			case 2:		/*MOTOR_M3*/
				TIM_SetCompare3(TIM2,ratioToCCRx(ratio));
				break;
			case 3:		/*MOTOR_M4*/	
				TIM_SetCompare1(TIM2,ratioToCCRx(ratio));
				break;
			default: break;
		}	
	}
}

