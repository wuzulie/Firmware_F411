#include <math.h>
#include "maths.h"
#include "commander.h"
#include "anomal_detec.h"
#include "remoter_ctrl.h"
#include "stabilizer.h"
#include "module_mgt.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 异常检测驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#if defined(DETEC_ENABLED)

static u16 outFlipCnt = 0;		


static bool detecFreeFall(float accZ, float accMAG)	/*自由落体检测*/
{
	static u16 cnt;

	/*自由落体*/
	if(fabs(accMAG) < DETEC_FF_THRESHOLD && fabs(accZ + 1.f) < DETEC_FF_THRESHOLD)	
	{	
		if(++cnt >= (DETEC_FF_COUNT))
		{
			return true;
		}		
	}
	else
	{
		cnt=0;
	}
	
	return false;
}

static bool detecTumbled(const state_t *state)	/*碰撞检测*/
{
	static u16 cnt;
	
	float fAbsRoll  = fabs(state->attitude.roll);
	float fAbsPitch = fabs(state->attitude.pitch);
	float fMax = (fAbsRoll >= fAbsPitch) ? fAbsRoll : fAbsPitch;
	
	if(fMax > DETEC_TU_THRESHOLD)
	{
		if(++cnt >= DETEC_TU_COUNT)
		{
			return true;
		}
	}else 
	{
		cnt=0;
	}
	
	return false;
}
#endif

/*异常检测*/
void anomalDetec(const sensorData_t *sensorData, const state_t *state, const control_t *control)
{
#if defined(DETEC_ENABLED)
	
	if(control->flipDir != CENTER) 
	{
		outFlipCnt = 1000;
		return;
	}	

	if(state->isRCLocked == false && 		//遥控器解锁状态
	getCommanderKeyFlight() == false &&		//未飞行状态
	(getCommanderCtrlMode() & 0x01) == 0x01)//定高模式
	{
		float accMAG = (sensorData->acc.x*sensorData->acc.x) +
						(sensorData->acc.y*sensorData->acc.y) +
						(sensorData->acc.z*sensorData->acc.z);

		if(detecFreeFall(state->acc.z/980.f, accMAG) == true)/*自由落体检测*/
		{				
			setCommanderKeyFlight(true);
			setFastAdjustPosParam(35, 10, 0.f);	/*设置快速调整位置参数*/
		}
	}
	
	if(outFlipCnt > 0)	
	{
		outFlipCnt--;
	}
	if(outFlipCnt == 0 && detecTumbled(state)==true)/*碰撞检测*/
	{
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}			

#endif
}


