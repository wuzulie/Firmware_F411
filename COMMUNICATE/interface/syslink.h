#ifndef __SYSLINK_H
#define __SYSLINK_H
#include <stdbool.h>
#include "stdint.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * stm32,nrf51822串口驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define SYSLINK_MTU 32

#define CRTP_START_BYTE  0xAA
#define SYSLINK_START_BYTE1 0xBC
#define SYSLINK_START_BYTE2 0xCF

// Defined packet types
#define SYSLINK_GROUP_MASK    0xF0

#define SYSLINK_RADIO_GROUP    0x00
#define SYSLINK_RADIO_RAW      0x00
#define SYSLINK_RADIO_CHANNEL  0x01
#define SYSLINK_RADIO_DATARATE 0x02
#define SYSLINK_RADIO_CONTWAVE 0x03
#define SYSLINK_RADIO_RSSI     0x04
#define SYSLINK_RADIO_ADDRESS  0x05

#define SYSLINK_PM_GROUP              0x10
#define SYSLINK_PM_SOURCE             0x10
#define SYSLINK_PM_ONOFF_SWITCHOFF    0x11
#define SYSLINK_PM_BATTERY_VOLTAGE    0x12
#define SYSLINK_PM_BATTERY_STATE      0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14

#define SYSLINK_OW_GROUP    0x20
#define SYSLINK_OW_SCAN     0x20
#define SYSLINK_OW_GETINFO  0x21
#define SYSLINK_OW_READ     0x22
#define SYSLINK_OW_WRITE    0x23

typedef struct _SyslinkPacket
{
	uint8_t type;
	uint8_t length;
	char data[SYSLINK_MTU];
} __attribute__((packed)) SyslinkPacket;

typedef enum
{
	waitForFirstStart,
	waitForSecondStart,
	waitForType,
	waitForLengt,
	waitForData,
	waitForChksum1,
	waitForChksum2
} SyslinkRxState;


void syslinkInit(void);
bool syslinkTest(void);
int syslinkSendPacket(SyslinkPacket *slp);
void syslinkTask(void *param);

#endif
