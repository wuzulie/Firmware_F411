#include "module_mgt.h"
#include "module_detect.h"
#include "ledring12.h"
#include "wifi_ctrl.h"
#include "optical_flow.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��չģ�������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static xTimerHandle timer;
static enum expModuleID moduleID = NO_MODULE;

//��ȡ��չģ��ID
enum expModuleID getModuleID(void)
{
	return moduleID;
}

//��չģ���Դ����
void expModulePower(bool state)
{
	wifiPowerControl(false);
	ledringPowerControl(false);
	opticalFlowPowerControl(false);
	
	if(state == true)
	{
		if(moduleID == LED_RING)
		{
			ledringPowerControl(true);
		}
		else if(moduleID == WIFI_CAMERA)
		{
			wifiPowerControl(true);
		}
		else if(moduleID == OPTICAL_FLOW)
		{
			opticalFlowPowerControl(true);	
		}
		else if(moduleID == MODULE1)
		{
		
		}
	}	
}

//�����ʱ���жϼ����չģ��
static void expModuleDetect(xTimerHandle xTimer)
{
	static u8 cnt = 0;
	enum expModuleID id = getModuleDriverID();

	if(id != moduleID && ++cnt > 3)
	{
		cnt = 0;
		moduleID = id;	/*����ģ��ID*/
		
		wifiPowerControl(false);
		ledringPowerControl(false);
		opticalFlowPowerControl(false);
		
		if(id == LED_RING)
		{
			ledring12Init();
		}
		else if(id == WIFI_CAMERA)
		{
			wifiModuleInit();
		}		
		else if(id == OPTICAL_FLOW)
		{
			opticalFlowInit();
		}
		else if(id == MODULE1)
		{
		}	
	}
}

//��չģ���������
void expModuleMgtTask(void* param)
{
	timer = xTimerCreate( "expModuleTimer", 500, pdTRUE, NULL, expModuleDetect);
	xTimerStart(timer, portMAX_DELAY);
	while(1)
	{	
		vTaskDelay(1000);
	}
}

