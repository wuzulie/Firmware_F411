#include <math.h>
#include "pid.h"
#include "commander.h"
#include "config_param.h"
#include "position_pid.h"
#include "remoter_ctrl.h"
#include "maths.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * λ��PID���ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 ˮƽ����PID����ϴ�������λ�û��������0.1��ϵ����
	���ʻ��������0.15ϵ�����Ӷ�����PID�Ŀɵ��ԡ�
********************************************************************************/

#define THRUST_BASE  		(20000)	/*��������ֵ*/

#define PIDVX_OUTPUT_LIMIT	120.0f	//ROLL�޷�	(��λ���0.15��ϵ��)
#define PIDVY_OUTPUT_LIMIT	120.0f 	//PITCH�޷�	(��λ���0.15��ϵ��)
#define PIDVZ_OUTPUT_LIMIT	(40000)	/*PID VZ�޷�*/

#define PIDX_OUTPUT_LIMIT	1200.0f	//X���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDY_OUTPUT_LIMIT	1200.0f	//Y���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDZ_OUTPUT_LIMIT	120.0f	//Z���ٶ��޷�(��λcm/s)


static float thrustLpf = THRUST_BASE;	/*���ŵ�ͨ*/

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

void positionControlInit(float velocityPidDt, float posPidDt)
{
	pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);	/*vx PID��ʼ��*/
	pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);	/*vy PID��ʼ��*/
	pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);	/*vz PID��ʼ��*/
	pidSetOutputLimit(&pidVX, PIDVX_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidVY, PIDVY_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidVZ, PIDVZ_OUTPUT_LIMIT);		/* ����޷� */
	
	pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);			/*x PID��ʼ��*/
	pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);			/*y PID��ʼ��*/
	pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);			/*z PID��ʼ��*/
	pidSetOutputLimit(&pidX, PIDX_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidY, PIDY_OUTPUT_LIMIT);		/* ����޷� */
	pidSetOutputLimit(&pidZ, PIDZ_OUTPUT_LIMIT);		/* ����޷� */
}

static void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)                                                         
{	
	static u16 altholdCount = 0;
	
	// Roll and Pitch
	attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
	attitude->roll = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);
	
	// Thrust
	float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);
	
	*thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 60000);	/*�����޷�*/
	
	thrustLpf += (*thrust - thrustLpf) * 0.003f;
	
	if(getCommanderKeyFlight())	/*���߷���״̬*/
	{
		if(fabs(state->acc.z) < 35.f)
		{
			altholdCount++;
			if(altholdCount > 1000)
			{
				altholdCount = 0;
				if(fabs(configParam.thrustBase - thrustLpf) > 1000.f)	/*���»�������ֵ*/
					configParam.thrustBase = thrustLpf;
			}
		}else
		{
			altholdCount = 0;
		}
	}else if(getCommanderKeyland() == false)	/*������ɣ���������*/
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

/*��ȡ��������ֵ*/
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




