#ifndef __EXTI_H__
#define __EXTI_H__
#include <stdbool.h>

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 外部中断驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

void extiInit(void);
bool extiTest(void);

void EXTI0_Callback(void);
void EXTI1_Callback(void);
void EXTI2_Callback(void);
void EXTI3_Callback(void);
void EXTI4_Callback(void);
void EXTI5_Callback(void);
void EXTI6_Callback(void);
void EXTI7_Callback(void);
void EXTI8_Callback(void);
void EXTI9_Callback(void);
void EXTI10_Callback(void);
void EXTI11_Callback(void);
void EXTI12_Callback(void);
void EXTI13_Callback(void);
void EXTI14_Callback(void);
void EXTI15_Callback(void);

#endif /* __EXTI_H__ */

