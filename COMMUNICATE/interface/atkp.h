#ifndef __ATKP_H
#define __ATKP_H
#include <stdint.h>
#include <stdbool.h>
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 飞控通讯协议格式代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 * 说明：本格式部分基于匿名上位机通讯协议编写
 *
 * 修改说明:
 * 版本V1.3 增加用户数据(USERDATA)上传功能，方便用户上传需要调试的数据到上位机，
 * 上位机实时打印波形，方便用户调试。
********************************************************************************/

/*上行帧头*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*下行帧头*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define ATKP_MAX_DATA_SIZE 30

/*通讯数据结构*/
typedef struct
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[ATKP_MAX_DATA_SIZE];
}atkp_t;

/*上行指令ID*/
typedef enum 
{
	UP_VERSION		= 0x00,
	UP_STATUS		= 0x01,
	UP_SENSER		= 0x02,
	UP_RCDATA		= 0x03,
	UP_GPSDATA		= 0x04,
	UP_POWER		= 0x05,
	UP_MOTOR		= 0x06,
	UP_SENSER2		= 0x07,
	UP_FLYMODE		= 0x0A,
	UP_SPEED 		= 0x0B,
	UP_PID1			= 0x10,
	UP_PID2			= 0x11,
	UP_PID3			= 0x12,
	UP_PID4			= 0x13,
	UP_PID5			= 0x14,
	UP_PID6			= 0x15,
	UP_RADIO		= 0x40,
	UP_MSG			= 0xEE,
	UP_CHECK		= 0xEF,
	
	UP_REMOTER		= 0x50,
	UP_PRINTF		= 0x51,
	
	UP_USER_DATA1	= 0xF1,
	UP_USER_DATA2	= 0xF2,
	UP_USER_DATA3	= 0xF3,
	UP_USER_DATA4	= 0xF4,
	UP_USER_DATA5	= 0xF5,
	UP_USER_DATA6	= 0xF6,
	UP_USER_DATA7	= 0xF7,
	UP_USER_DATA8	= 0xF8,
	UP_USER_DATA9	= 0xF9,
	UP_USER_DATA10	= 0xFA,
}upmsgID_e;


/*下行指令*/
#define  D_COMMAND_ACC_CALIB		0x01
#define  D_COMMAND_GYRO_CALIB		0x02
#define  D_COMMAND_MAG_CALIB		0x04
#define  D_COMMAND_BARO_CALIB		0x05
#define  D_COMMAND_ACC_CALIB_EXIT	0x20
#define  D_COMMAND_ACC_CALIB_STEP1	0x21
#define  D_COMMAND_ACC_CALIB_STEP2	0x22
#define  D_COMMAND_ACC_CALIB_STEP3	0x23
#define  D_COMMAND_ACC_CALIB_STEP4	0x24
#define  D_COMMAND_ACC_CALIB_STEP5	0x25
#define  D_COMMAND_ACC_CALIB_STEP6	0x26
#define  D_COMMAND_FLIGHT_LOCK		0xA0
#define  D_COMMAND_FLIGHT_ULOCK		0xA1

#define  D_ACK_READ_PID				0x01
#define  D_ACK_READ_VERSION			0xA0
#define  D_ACK_RESET_PARAM			0xA1
/*下行指令ID*/
typedef enum 
{
	DOWN_COMMAND	= 0x01,
	DOWN_ACK		= 0x02,
	DOWN_RCDATA		= 0x03,
	DOWN_POWER		= 0x05,
	DOWN_FLYMODE	= 0x0A,
	DOWN_PID1		= 0x10,
	DOWN_PID2		= 0x11,
	DOWN_PID3		= 0x12,
	DOWN_PID4		= 0x13,
	DOWN_PID5		= 0x14,
	DOWN_PID6		= 0x15,
	DOWN_RADIO		= 0x40,
	
	DOWN_REMOTER	= 0x50,
}downmsgID_e;


void atkpTxTask(void *param);
void atkpRxAnlTask(void *param);
void atkpInit(void);
bool atkpReceivePacketBlocking(atkp_t *p);

#endif /*ATKP_H*/

