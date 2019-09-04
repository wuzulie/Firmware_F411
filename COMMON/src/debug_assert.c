/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"

#include "debug_assert.h"
#include "led.h"
#include "motors.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


#define MAGIC_ASSERT_INDICATOR 0x2f8a001f

typedef struct SNAPSHOT_DATA 
{
	u32 magicNumber;
	char* fileName;
	int line;
} SNAPSHOT_DATA;

// The .nzds section is not cleared at startup, data here will survive a
// reset (by the watch dog for instance)
SNAPSHOT_DATA snapshot __attribute__((section(".nzds"))) = 
{
	.magicNumber = 0,
	.fileName = "",
	.line = 0
};


void assertFail(char *exp, char *file, int line)
{
	portDISABLE_INTERRUPTS();
	storeAssertSnapshotData(file, line);
	printf("Assert failed %s:%d\n", file, line);

	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);

	ledClearAll();
	ledSet(ERR_LED1, 1);/*������*/
	ledSet(ERR_LED2, 1);

	while (1);
}

void storeAssertSnapshotData(char *file, int line)
{
	snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
	snapshot.fileName = file;
	snapshot.line = line;
}

void printAssertSnapshotData()
{
	if (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) 
	{
		printf("Assert failed at %s:%d\n", snapshot.fileName, snapshot.line);
	} else 
	{
		printf("No assert information found\n");
	}
}



