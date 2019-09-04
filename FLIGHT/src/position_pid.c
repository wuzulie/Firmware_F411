#include <math.h>
#include "pid.h"
#include "commander.h"
#include "config_param.h"
#include "position_pid.h"
#include "remoter_ctrl.h"
#include "maths.h"

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
 *
 * 修改说明:
 * 版本V1.3 水平定点PID输出较大，所以在位置环输出设置0.1的系数，
	速率环输出设置0.15系数，从而增加PID的可调性。
********************************************************************************/

#define THRUST_BASE  		(20000)	/*基础油门值*/

#define PIDVX_OUTPUT_LIMIT	120.0f	//ROLL限幅	(单位°带0.15的系数)
#define PIDVY_OUTPUT_LIMIT	120.0f 	//PITCH限幅	(单位°带0.15的系数)
#define PIDVZ_OUTPUT_LIMIT	(40000)	/*PID VZ限幅*/

#define PIDX_OUTPUT_LIMIT	1200.0f	//X轴速度限幅(单位cm/s 带0.1的系数)
#define PIDY_OUTPUT_LIMIT	1200.0f	//Y轴速度限幅(单位cm/s 带0.1的系数)
#define PIDZ_OUTPUT_LIMIT	120.0f	//Z轴速度限幅(单位cm/s)


static float thrustLpf = THRUST_BASE;	/*油门低通*/

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

void positionControlInit(float velocityPidDt, float posPidDt)
{
	pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);	/*vx PID初始化*/
	pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);	/*vy PID初始化*/
	pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);	/*vz PID初始化*/
	pidSetOutputLimit(&pidVX, PIDVX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVY, PIDVY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidVZ, PIDVZ_OUTPUT_LIMIT);		/* 输出限幅 */
	
	pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);			/*x PID初始化*/
	pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);			/*y PID初始化*/
	pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);			/*z PID初始化*/
	pidSetOutputLimit(&pidX, PIDX_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidY, PIDY_OUTPUT_LIMIT);		/* 输出限幅 */
	pidSetOutputLimit(&pidZ, PIDZ_OUTPUT_LIMIT);		/* 输出限幅 */
}

static void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)                                                         
{	
	static u16 altholdCount = 0;
	
	// Roll and Pitch
	attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
	attitude->roll = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);
	
	// Thrust
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);
	
	*thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 60000);	/*油门限幅*/
	
	thrustLpf += (*thrust - thrustLpf) * 0.003f;
	
	if(getCommanderKeyFlight())	/*定高飞行状态*/
	{
		if(fabs(state->acc.z) < 35.f)
		{
			altholdCount++;
			if(altholdCount > 1000)
			{
				altholdCount = 0;
				if(fabs(configParam.thrustBase - thrustLpf) > 1000.f)	/*更新基础油门值*/
					configParam.thrustBase = thrustLpf;
			}
		}else
		{
			altholdCount = 0;
		}
	}else if(getCommanderKeyland() == false)	/*降落完成，油门清零*/
	{
		*thrust = 0;
	}
}

void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt)                                                
{	
	if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs)
	{
		setpoint->velocity.x = 0.1f * pidUpdate(&pidX, setpoint->position.x - state->position.x);
		setpoint->velocity.y = 0.1f * pidUpdate(&pidY, setpoint->position.y - state->position.y);
	}
	
	if (setpoint->mode.z == modeAbs)
	{
		setpoint->velocity.z = pidUpdate(&pidZ, setpoint->position.z - state->position.z);
	}
	
	velocityController(thrust, attitude, setpoint, state);
}

/*获取定高油门值*/
float getAltholdThrust(void)
{
	return thrustLpf;
}

void positionResetAllPID(void)
{
	pidReset(&pidVX);
	pidReset(&pidVY);
	pidReset(&pidVZ);

	pidReset(&pidX);
	pidReset(&pidY);
	pidReset(&pidZ);
}

void positionPIDwriteToConfigParam(void)
{
	configParam.pidPos.vx.kp  = pidVX.kp;
	configParam.pidPos.vx.ki  = pidVX.ki;
	configParam.pidPos.vx.kd  = pidVX.kd;
	
	configParam.pidPos.vy.kp  = pidVY.kp;
	configParam.pidPos.vy.ki  = pidVY.ki;
	configParam.pidPos.vy.kd  = pidVY.kd;
	
	configParam.pidPos.vz.kp  = pidVZ.kp;
	configParam.pidPos.vz.ki  = pidVZ.ki;
	configParam.pidPos.vz.kd  = pidVZ.kd;
	
	configParam.pidPos.x.kp  = pidX.kp;
	configParam.pidPos.x.ki  = pidX.ki;
	configParam.pidPos.x.kd  = pidX.kd;
	
	configParam.pidPos.y.kp  = pidY.kp;
	configParam.pidPos.y.ki  = pidY.ki;
	configParam.pidPos.y.kd  = pidY.kd;
	
	configParam.pidPos.z.kp  = pidZ.kp;
	configParam.pidPos.z.ki  = pidZ.ki;
	configParam.pidPos.z.kd  = pidZ.kd;
}




