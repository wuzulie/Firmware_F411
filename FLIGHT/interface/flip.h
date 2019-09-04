#ifndef __FLIP_H
#define __FLIP_H 
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴空翻控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


void flyerFlipCheck(setpoint_t* setpoint,control_t* control,state_t* state);	/* Flyer 翻滚检测*/

void setFlipDir(u8 dir);

#endif 

