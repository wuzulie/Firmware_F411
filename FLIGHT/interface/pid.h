#ifndef __PID_H
#define __PID_H
#include <stdbool.h>
#include "config_param.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * PID驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 增加PID结构体的输出一项(out)。
********************************************************************************/
#define DEFAULT_PID_INTEGRATION_LIMIT 		500.0 //默认pid的积分限幅
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //默认pid输出限幅，0为不限幅

typedef struct
{
	float desired;		//< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	float dt;           //< delta-time dt
	float out;			//< out
} PidObject;

/*pid结构体初始化*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);/*pid积分限幅设置*/
void pidSetOutputLimit(PidObject* pid, const float limit);
void pidSetDesired(PidObject* pid, const float desired);	/*pid设置期望值*/
float pidUpdate(PidObject* pid, const float error);			/*pid更新*/
float pidGetDesired(PidObject* pid);	/*pid获取期望值*/
bool pidIsActive(PidObject* pid);		/*pid状态*/
void pidReset(PidObject* pid);			/*pid结构体复位*/
void pidSetError(PidObject* pid, const float error);/*pid偏差设置*/
void pidSetKp(PidObject* pid, const float kp);		/*pid Kp设置*/
void pidSetKi(PidObject* pid, const float ki);		/*pid Ki设置*/
void pidSetKd(PidObject* pid, const float kd);		/*pid Kd设置*/
void pidSetDt(PidObject* pid, const float dt);		/*pid dt设置*/

#endif /* __PID_H */
