#include "module_mgt.h"
#include "module_detect.h"
#include "ledring12.h"
#include "wifi_ctrl.h"
#include "optical_flow.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 扩展模块管理驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static xTimerHandle timer;
static enum expModuleID moduleID = NO_MODULE;

//获取扩展模块ID
enum expModuleID getModuleID(void)
{
	return moduleID;
}

//扩展模块电源控制
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

//软件定时器中断检测扩展模块
static void expModuleDetect(xTimerHandle xTimer)
{
	static u8 cnt = 0;
	enum expModuleID id = getModuleDriverID();

	if(id != moduleID && ++cnt > 3)
	{
		cnt = 0;
		moduleID = id;	/*更新模块ID*/
		
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

//扩展模块管理任务
void expModuleMgtTask(void* param)
{
	timer = xTimerCreate( "expModuleTimer", 500, pdTRUE, NULL, expModuleDetect);
	xTimerStart(timer, portMAX_DELAY);
	while(1)
	{	
		vTaskDelay(1000);
	}
}

