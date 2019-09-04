#include "string.h"
#include <math.h>
#include "sensfusion6.h"
#include "remoter_ctrl.h"
#include "config_param.h"
#include "commander.h"
#include "flip.h"
#include "radiolink.h"
#include "sensors.h"
#include "pm.h"
#include "stabilizer.h"
#include "module_mgt.h"
#include "ledring12.h"
#include "vl53lxx.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 遥控器控制驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 增加上电校准通过后上传微调信息。
********************************************************************************/

static ctrlVal_t remoterCtrl;/*发送到commander姿态控制数据*/
static MiniFlyMsg_t msg;
static u8 reSendTimes = 3;	/*微调重发次数*/

/*返回四轴信息*/
void sendMsgACK(void)
{
	msg.version = configParam.version;
	msg.mpu_selfTest = getIsMPU9250Present();
	msg.baro_slfTest = getIsBaroPresent();
	msg.isCanFly = getIsCalibrated();
	if(msg.isCanFly == true)	/*校准通过之后发送微调值*/
	{
		if(reSendTimes > 0) /*微调重发次数*/
		{
			reSendTimes--;
			msg.trimPitch = configParam.trimP;
			msg.trimRoll = configParam.trimR;
		}
	}
	msg.isLowpower = getIsLowpower();
	msg.moduleID = getModuleID();
	
	atkp_t p;
	p.msgID = UP_REMOTER;
	p.dataLen = sizeof(msg)+1;
	p.data[0] = ACK_MSG;
	memcpy(p.data+1, &msg, sizeof(msg));
	radiolinkSendPacketBlocking(&p);	
}

/*遥控数据接收处理*/
void remoterCtrlProcess(atkp_t* pk)
{	
	if(pk->data[0] == REMOTER_CMD)
	{
		switch(pk->data[1])
		{
			case CMD_FLIGHT_LAND:
				if(getCommanderKeyFlight() != true)
				{
					setCommanderKeyFlight(true);
					setCommanderKeyland(false);
				}
				else
				{
					setCommanderKeyFlight(false);
					setCommanderKeyland(true);
				}
				break;

			case CMD_EMER_STOP:
				setCommanderKeyFlight(false);
				setCommanderKeyland(false);
				break;
			
			case CMD_FLIP:
				setFlipDir(pk->data[2]);
				break;
			
			case CMD_GET_MSG:
				sendMsgACK();
				break;
			
			case CMD_POWER_MODULE:
				expModulePower(pk->data[2]);
				break;
			
			case CMD_LEDRING_EFFECT:
//				expModulePower(true);
				setLedringEffect(pk->data[2]);
				break;
			
			case CMD_POWER_VL53LXX:
				setVl53lxxState(pk->data[2]);
				break;
		}
	}
	else if(pk->data[0] == REMOTER_DATA)
	{
		remoterData_t remoterData = *(remoterData_t*)(pk->data+1);
		
		remoterCtrl.roll = remoterData.roll;
		remoterCtrl.pitch = remoterData.pitch;
		remoterCtrl.yaw = remoterData.yaw;
		remoterCtrl.thrust = remoterData.thrust * 655.35f;
		remoterCtrl.trimPitch = remoterData.trimPitch;
		remoterCtrl.trimRoll = remoterData.trimRoll;
		
		setCommanderCtrlMode(remoterData.ctrlMode);
		setCommanderFlightmode(remoterData.flightMode);
		flightCtrldataCache(ATK_REMOTER, remoterCtrl);
	}
}

