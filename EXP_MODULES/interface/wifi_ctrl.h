#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include "config.h"
#include "stabilizer_types.h"
#include "module_mgt.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 手机wifi控制驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/6/22
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define WIFI_POWER_ENABLE	PBout(0)

typedef struct
{
	u8 keyFlight 	: 1;	/*bit0 一键起飞*/
	u8 keyLand 		: 1;	/*bit1 一键降落*/
	u8 emerStop 	: 1;	/*bit2 紧急停机*/
	u8 flipOne 		: 1;	/*bit3 固定方向翻滚*/
	u8 flightMode 	: 1;	/*bit4 飞行模式 1=无头 0=有头*/
	u8 flipFour 	: 1;	/*bit5 4D翻滚*/
	u8 ledControl 	: 1;	/*bit6 灯光控制*/
	u8 gyroCalib 	: 1;	/*bit7 陀螺校准*/	
}wifiCmd_t;


void wifiModuleInit(void);			/*wifi模块初始化*/
void wifiPowerControl(bool state);	/*wifi电源控制*/
void wifiLinkTask(void *param);


#endif /* WIFI_CONTROL_H */

