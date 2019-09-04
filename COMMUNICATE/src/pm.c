#include "sys.h"
#include <string.h>
#include <stdbool.h>
#include "config.h"
#include "system.h"
#include "pm.h"
#include "led.h"
#include "ledseq.h"
#include "commander.h"
#include "radiolink.h"
#include "remoter_ctrl.h"
#include "stabilizer.h"
#include "sensfusion6.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��Դ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef __packed struct _PmSyslinkInfo
{
	__packed union
	{
		u8 flags;
		__packed struct
		{
			u8 pgood  : 1;
			u8 chg    : 1;
			u8 unused : 6;
		};
	};
	float vBat;
	
} PmSyslinkInfo;

static float    batteryVoltage;
static float    batteryVoltageMin = 6.0;
static float    batteryVoltageMax = 0.0;

static bool isInit;
static bool isLowpower;
static PMStates pmState;
static PmSyslinkInfo pmSyslinkInfo;
static u32 batteryLowTimeStamp;

static void pmSetBatteryVoltage(float voltage);


void pmInit(void)
{
	if(isInit) return;

	pmSyslinkInfo.vBat = 3.7f;
	pmSetBatteryVoltage(pmSyslinkInfo.vBat); 
	
	isInit = true;
}

bool pmTest(void)
{
	return isInit;
}

static void pmSetBatteryVoltage(float voltage)	/*���õ�ص�ѹ�����Сֵ*/
{	
	batteryVoltage = voltage;
	
	if (batteryVoltageMax < voltage)
	{
		batteryVoltageMax = voltage;
	}
	if (batteryVoltageMin > voltage)
	{
		batteryVoltageMin = voltage;
	}
}


float pmGetBatteryVoltage(void)
{
	return batteryVoltage;
}

void pmSyslinkUpdate(atkp_t *slp)
{
	memcpy(&pmSyslinkInfo, &slp->data[0], sizeof(pmSyslinkInfo));
	pmSetBatteryVoltage(pmSyslinkInfo.vBat);
}


PMStates pmUpdateState()	/* ���µ�Դ״̬ */
{
	PMStates state;
	bool isCharging = pmSyslinkInfo.chg;
	bool isPgood = pmSyslinkInfo.pgood;
	u32 batteryLowTime;

	batteryLowTime = getSysTickCnt() - batteryLowTimeStamp;

	if (isPgood && !isCharging)
	{
		state = charged;
	}else if (isPgood && isCharging)
	{
		state = charging;
	}else if (!isPgood && !isCharging && (batteryLowTime > PM_BAT_LOW_TIMEOUT))
	{
		state = lowPower;
	}else
	{
		state = battery;
	}

	return state;
}

void pmTask(void *param)	/* ��Դ�������� */
{
	PMStates pmStateOld = battery;
	u32 tickCount;

	tickCount = getSysTickCnt();
	batteryLowTimeStamp = tickCount;

	vTaskDelay(500);

	while(1)
	{
		vTaskDelay(100);
		tickCount = getSysTickCnt();

		if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE)
		{
			batteryLowTimeStamp = tickCount;
		}

		pmState = pmUpdateState();

		if (pmState != pmStateOld)
		{
			switch (pmState)	/*��Դ״̬�л�*/
			{
				case charged:				
					ledseqStop(CHG_LED, seq_charging);
					ledseqRun(CHG_LED, seq_charged);
					break;
				case charging:
					isLowpower = false;
					ledseqStop(LOWBAT_LED, seq_lowbat);
					if(getIsCalibrated())
						ledseqRun(SYS_LED, seq_calibrated);
					else
						ledseqRun(SYS_LED, seq_alive);				
					ledseqStop(CHG_LED, seq_charged);
					ledseqRun(CHG_LED, seq_charging);
					break;
				case lowPower:
					isLowpower = true;
//					ledseqStop(CHG_LED, seq_charging);
					ledseqStop(SYS_LED, seq_alive);
					ledseqStop(SYS_LED, seq_calibrated);
					ledseqRun(LOWBAT_LED, seq_lowbat);
					break;
				case battery:
					ledseqStop(CHG_LED, seq_charging);
					ledseqRun(CHG_LED, seq_charged);
					break;
				default:
					break;
			}
			pmStateOld = pmState;
		}
	}
}


bool getIsLowpower(void)
{
	return isLowpower;
}
