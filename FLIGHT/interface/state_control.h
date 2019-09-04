#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴姿态控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

//#define ENABLE_PID_TUNING	/* 使能PID调节 yaw值不更新 */

void stateControlInit(void);
bool stateControlTest(void);
void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick);

#endif /*__STATE_CONTROL_H */

