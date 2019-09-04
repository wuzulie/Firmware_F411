#ifndef REMOTER_CTRL_H
#define REMOTER_CTRL_H
#include "atkp.h"
#include "sys.h"
#include "module_detect.h"

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
********************************************************************************/

/*遥控数据类别*/
typedef enum 
{
	REMOTER_CMD,
	REMOTER_DATA,
}remoterType_e;

/*下行命令*/
#define  CMD_GET_MSG		0x01	/*获取四轴信息（自检）*/
#define  CMD_GET_CANFLY		0x02	/*获取四轴是否能飞*/
#define  CMD_FLIGHT_LAND	0x03	/*起飞、降落*/
#define  CMD_EMER_STOP		0x04	/*紧急停机*/
#define  CMD_FLIP			0x05	/*4D翻滚*/
#define  CMD_POWER_MODULE	0x06	/*打开关闭扩展模块电源*/
#define  CMD_LEDRING_EFFECT	0x07	/*设置RGB灯环效果*/
#define  CMD_POWER_VL53LXX	0x08	/*打开关闭激光*/

/*上行报告*/
#define  ACK_MSG			0x01

/*遥控数据结构*/
typedef __packed struct
{
	float roll;      
	float pitch;  
	float yaw;      
	float thrust;
	float trimPitch;
	float trimRoll;
	u8	ctrlMode;
	bool flightMode;
	bool RCLock;
} remoterData_t;

typedef __packed struct
{
	u8 version;
	bool mpu_selfTest;
	bool baro_slfTest;
	bool isCanFly;
	bool isLowpower;
	enum expModuleID moduleID;
	
	float trimRoll;		/*roll微调*/
	float trimPitch;	/*pitch微调*/
} MiniFlyMsg_t;


void remoterCtrlProcess(atkp_t* pk);
void sendMsgACK(void);


#endif /* WIFI_CONTROL_H */

