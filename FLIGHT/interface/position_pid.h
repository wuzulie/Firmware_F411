#ifndef __POSITION_PID_H
#define __POSITION_PID_H
#include "stabilizer_types.h"
#include "pid.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 位置PID控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

extern PidObject pidVX;
extern PidObject pidVY;
extern PidObject pidVZ;

extern PidObject pidX;
extern PidObject pidY;
extern PidObject pidZ;

void positionControlInit(float ratePidDt, float posPidDt);
void positionResetAllPID(void);
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt);
void positionPIDwriteToConfigParam(void);
float getAltholdThrust(void);
	
#endif /* __POSITION_PID_H */
