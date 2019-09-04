#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "atkp.h"
#include "radiolink.h"
#include "usblink.h"
#include "usbd_usr.h"
#include "stabilizer.h"
#include "motors.h"
#include "commander.h"
#include "flip.h"
#include "pm.h"
#include "pid.h"
#include "attitude_pid.h"
#include "sensors.h"
#include "position_pid.h"
#include "config_param.h"
#include "power_control.h"
#include "remoter_ctrl.h"
#include "optical_flow.h"
#include "state_estimator.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 无线通信驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 * 说明：此文件程序基于于匿名科创地面站V4.34通信协议下位机,示例代码修改。
 *
 * 修改说明:
 * 版本V1.3 增加用户数据(USERDATA)上传功能，方便用户上传需要调试的数据到上位机，
 * 上位机实时打印波形，方便用户调试。
********************************************************************************/

//数据拆分宏定义
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )

//数据返回周期时间（单位ms）
#define  PERIOD_STATUS		30
#define  PERIOD_SENSOR 		10
#define  PERIOD_RCDATA 		40
#define  PERIOD_POWER 		100
#define  PERIOD_MOTOR		40
#define  PERIOD_SENSOR2 	40
#define  PERIOD_SPEED   	50
#define  PERIOD_USERDATA   	20

#define ATKP_RX_QUEUE_SIZE 	10 /*ATKP包接收队列消息个数*/

typedef struct  
{
	u16 roll;
	u16 pitch;
	u16 yaw;
	u16 thrust;
}joystickFlyui16_t;


bool isConnect = false;
bool isInit = false;
bool flyable = false;
static joystickFlyui16_t rcdata;
static xQueueHandle rxQueue;

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;
extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;


static void atkpSendPacket(atkp_t *p)
{
	radiolinkSendPacket(p);
	
	if(getusbConnectState())
	{
		usblinkSendPacket(p);
	}	
}
/***************************发送至匿名上位机指令******************************/
static void sendStatus(float roll, float pitch, float yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	p.msgID = UP_STATUS;
	
	_temp = (int)(roll*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(pitch*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(yaw*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.data[_cnt++]=BYTE3(_temp2);
	p.data[_cnt++]=BYTE2(_temp2);
	p.data[_cnt++]=BYTE1(_temp2);
	p.data[_cnt++]=BYTE0(_temp2);
	
	p.data[_cnt++] = fly_model;
	p.data[_cnt++] = armed;
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendSenser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_SENSER;

	_temp = a_x;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}
static void sendRCData(u16 thrust,u16 yaw,u16 roll,u16 pitch,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_RCDATA;
	p.data[_cnt++]=BYTE1(thrust);
	p.data[_cnt++]=BYTE0(thrust);
	p.data[_cnt++]=BYTE1(yaw);
	p.data[_cnt++]=BYTE0(yaw);
	p.data[_cnt++]=BYTE1(roll);
	p.data[_cnt++]=BYTE0(roll);
	p.data[_cnt++]=BYTE1(pitch);
	p.data[_cnt++]=BYTE0(pitch);
	p.data[_cnt++]=BYTE1(aux1);
	p.data[_cnt++]=BYTE0(aux1);
	p.data[_cnt++]=BYTE1(aux2);
	p.data[_cnt++]=BYTE0(aux2);
	p.data[_cnt++]=BYTE1(aux3);
	p.data[_cnt++]=BYTE0(aux3);
	p.data[_cnt++]=BYTE1(aux4);
	p.data[_cnt++]=BYTE0(aux4);
	p.data[_cnt++]=BYTE1(aux5);
	p.data[_cnt++]=BYTE0(aux5);
	p.data[_cnt++]=BYTE1(aux6);
	p.data[_cnt++]=BYTE0(aux6);

	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendPower(u16 votage, u16 current)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_POWER;
	
	p.data[_cnt++]=BYTE1(votage);
	p.data[_cnt++]=BYTE0(votage);
	p.data[_cnt++]=BYTE1(current);
	p.data[_cnt++]=BYTE0(current);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendMotorPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_MOTOR;
	
	p.data[_cnt++]=BYTE1(m_1);
	p.data[_cnt++]=BYTE0(m_1);
	p.data[_cnt++]=BYTE1(m_2);
	p.data[_cnt++]=BYTE0(m_2);
	p.data[_cnt++]=BYTE1(m_3);
	p.data[_cnt++]=BYTE0(m_3);
	p.data[_cnt++]=BYTE1(m_4);
	p.data[_cnt++]=BYTE0(m_4);
	p.data[_cnt++]=BYTE1(m_5);
	p.data[_cnt++]=BYTE0(m_5);
	p.data[_cnt++]=BYTE1(m_6);
	p.data[_cnt++]=BYTE0(m_6);
	p.data[_cnt++]=BYTE1(m_7);
	p.data[_cnt++]=BYTE0(m_7);
	p.data[_cnt++]=BYTE1(m_8);
	p.data[_cnt++]=BYTE0(m_8);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendSenser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_SENSER2;
	
	p.data[_cnt++]=BYTE3(bar_alt);
	p.data[_cnt++]=BYTE2(bar_alt);
	p.data[_cnt++]=BYTE1(bar_alt);
	p.data[_cnt++]=BYTE0(bar_alt);
	
	p.data[_cnt++]=BYTE1(csb_alt);
	p.data[_cnt++]=BYTE0(csb_alt);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendPid(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = 0x10+group-1;

	_temp = p1_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_p * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_i * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 10 ;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendCheck(u8 head, u8 check_sum)
{
	atkp_t p;
	
	p.msgID = UP_CHECK;
	p.dataLen = 2;
	p.data[0] = head;
	p.data[1] = check_sum;
	atkpSendPacket(&p);
}
static void sendUserData(u8 group,s16 a_x,s16 a_y,s16 a_z,s16 v_x,s16 v_y,s16 v_z,s16 p_x,s16 p_y,s16 p_z)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_USER_DATA1+group-1;

	_temp = a_x;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = v_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = v_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = v_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = p_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}
/****************************************************************************/
/*数据周期性发送给上位机，每1ms调用一次*/
static void atkpSendPeriod(void)
{
	static u16 count_ms = 1;
	
	if(!(count_ms % PERIOD_STATUS))
	{
		attitude_t attitude;
		getAttitudeData(&attitude);
		int baroData = getBaroData();
		sendStatus(attitude.roll, attitude.pitch, attitude.yaw, baroData, 0, flyable);			
	}
	if(!(count_ms % PERIOD_SENSOR))
	{
		Axis3i16 acc;
		Axis3i16 gyro;
		Axis3i16 mag;
		getSensorRawData(&acc, &gyro, &mag);
		sendSenser(acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z);
	}
	if(!(count_ms % PERIOD_USERDATA))	/*用户数据*/
	{
		Axis3f acc,vel,pos;
		float thrustBase = 0.1f * configParam.thrustBase;
		
		getStateData(&acc, &vel, &pos);
		sendUserData(1, acc.x, acc.y, acc.z, vel.x, vel.y, vel.z, pos.x, pos.y, pos.z);
		sendUserData(2, opFlow.velLpf[X],opFlow.velLpf[Y],opFlow.posSum[X],opFlow.posSum[Y],
						0,getFusedHeight(),vl53lxx.distance,100.f*vl53lxx.quality,thrustBase);
	}
	if(!(count_ms % PERIOD_RCDATA))
	{
		sendRCData(rcdata.thrust, rcdata.yaw, rcdata.roll,
					rcdata.pitch, 0, 0, 0, 0, 0, 0);
	}
	if(!(count_ms % PERIOD_POWER))
	{
		float bat = pmGetBatteryVoltage();
		sendPower(bat*100,500);
	}
	if(!(count_ms % PERIOD_MOTOR))
	{
		u16 m1,m2,m3,m4;
		motorPWM_t motorPWM;
		getMotorPWM(&motorPWM);
		m1 = (float)motorPWM.m1/65535*1000;
		m2 = (float)motorPWM.m2/65535*1000;
		m3 = (float)motorPWM.m3/65535*1000;
		m4 = (float)motorPWM.m4/65535*1000;
		sendMotorPWM(m1,m2,m3,m4,0,0,0,0);
	}
	if(!(count_ms % PERIOD_SENSOR2))
	{
		int baro = getBaroData() * 100.f;
		sendSenser2(baro, 0);
	}
	if(++count_ms>=65535) 
		count_ms = 1;	
}

static u8 atkpCheckSum(atkp_t *packet)
{
	u8 sum;
	sum = DOWN_BYTE1;
	sum += DOWN_BYTE2;
	sum += packet->msgID;
	sum += packet->dataLen;
	for(int i=0; i<packet->dataLen; i++)
	{
		sum += packet->data[i];
	}
	return sum;
}

static void atkpReceiveAnl(atkp_t *anlPacket)
{
	if(anlPacket->msgID	== DOWN_COMMAND)
	{
		switch(anlPacket->data[0])
		{
			case D_COMMAND_ACC_CALIB:
				break;
			
			case D_COMMAND_GYRO_CALIB:
				break;
			
			case D_COMMAND_MAG_CALIB:				
				break;
			
			case D_COMMAND_BARO_CALIB:
				break;
			
			case D_COMMAND_ACC_CALIB_STEP1:
				break;
			case D_COMMAND_ACC_CALIB_STEP2:
				break;
			case D_COMMAND_ACC_CALIB_STEP3:
				break;
			case D_COMMAND_ACC_CALIB_STEP4:
				break;
			case D_COMMAND_ACC_CALIB_STEP5:
				break;
			case D_COMMAND_ACC_CALIB_STEP6:
				break;
			
			case D_COMMAND_FLIGHT_LOCK:
				flyable = false;
				break;
			
			case D_COMMAND_FLIGHT_ULOCK:
				flyable = true;
		}
	}			
	else if(anlPacket->msgID == DOWN_ACK)
	{
		if(anlPacket->data[0] == D_ACK_READ_PID)/*读取PID参数*/
		{
			sendPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd,
					   pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
					   pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd 
				   );
			sendPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd,
					   pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
					   pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd 
				   );
			sendPid(3, pidVZ.kp, pidVZ.ki, pidVZ.kd,
					   pidZ.kp, pidZ.ki, pidZ.kd,
					   pidVX.kp, pidVX.ki, pidVX.kd
				   );
			sendPid(4, pidX.kp, pidX.ki, pidX.kd,
					   0, 0, 0,
					   0, 0, 0
				   );
		}
		if(anlPacket->data[0] == D_ACK_RESET_PARAM)/*恢复默认参数*/
		{
			resetConfigParamPID();
			
			attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT); /*初始化姿态PID*/	
			positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/
			
			sendPid(1, pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd,
					   pidRatePitch.kp, pidRatePitch.ki, pidRatePitch.kd,
					   pidRateYaw.kp, pidRateYaw.ki, pidRateYaw.kd 
				   );
			sendPid(2, pidAngleRoll.kp, pidAngleRoll.ki, pidAngleRoll.kd,
					   pidAnglePitch.kp, pidAnglePitch.ki, pidAnglePitch.kd,
					   pidAngleYaw.kp, pidAngleYaw.ki, pidAngleYaw.kd 
				   );
			sendPid(3, pidVZ.kp, pidVZ.ki, pidVZ.kd,
					   pidZ.kp, pidZ.ki, pidZ.kd,
					   pidVX.kp, pidVX.ki, pidVX.kd
				   );
			sendPid(4, pidX.kp, pidX.ki, pidX.kd,
					   0, 0, 0,
					   0, 0, 0
				   );
		}
	}
	else if(anlPacket->msgID == DOWN_RCDATA)
	{
		rcdata = *((joystickFlyui16_t*)anlPacket->data);
	}
	else if(anlPacket->msgID == DOWN_POWER)/*nrf51822*/
	{
		pmSyslinkUpdate(anlPacket);
	}
	else if(anlPacket->msgID == DOWN_REMOTER)/*遥控器*/	
	{
		remoterCtrlProcess(anlPacket);
	}
	else if(anlPacket->msgID == DOWN_PID1)
	{
		pidRateRoll.kp  = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidRateRoll.ki  = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidRateRoll.kd  = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidRatePitch.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidRatePitch.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidRatePitch.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidRateYaw.kp   = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidRateYaw.ki   = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidRateYaw.kd   = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID2)
	{
		pidAngleRoll.kp  = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidAngleRoll.ki  = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidAngleRoll.kd  = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		pidAnglePitch.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidAnglePitch.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidAnglePitch.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		pidAngleYaw.kp   = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidAngleYaw.ki   = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidAngleYaw.kd   = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}		
	else if(anlPacket->msgID == DOWN_PID3)
	{
		pidVZ.kp = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidVZ.ki = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidVZ.kd = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		pidZ.kp = 0.1*((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		pidZ.ki = 0.1*((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		pidZ.kd = 0.1*((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		pidVX.kp = 0.1*((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		pidVX.ki = 0.1*((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		pidVX.kd = 0.1*((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		
		pidVY = pidVX;	//位置速率PID，X\Y方向是一样的
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID4)
	{
		pidX.kp = 0.1*((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		pidX.ki = 0.1*((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		pidX.kd = 0.1*((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		pidY = pidX;	//位置保持PID，X\Y方向是一样的
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID5)
	{
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID6)
	{
//		s16 temp1  = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
//		s16 temp2  = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
//		s16 temp3  = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		s16 enable = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		s16 m1_set = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		s16 m2_set = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		s16 m3_set = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		s16 m4_set = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		setMotorPWM(enable,m1_set,m2_set,m3_set,m4_set);
		attitudePIDwriteToConfigParam();
		positionPIDwriteToConfigParam();
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
} 

void atkpTxTask(void *param)
{
	sendMsgACK();
	while(1)
	{
		atkpSendPeriod();
		vTaskDelay(1);
	}
}

void atkpRxAnlTask(void *param)
{
	atkp_t p;
	while(1)
	{
		xQueueReceive(rxQueue, &p, portMAX_DELAY);
		atkpReceiveAnl(&p);
	}
}

void atkpInit(void)
{
	if(isInit) return;
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(rxQueue);
	isInit = true;
}

bool atkpReceivePacketBlocking(atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}
