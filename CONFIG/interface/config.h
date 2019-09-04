#ifndef __CONFIG_H
#define __CONFIG_H
#include "nvic.h"
#include "stdio.h"	/*printf ����*/

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �����ļ�����	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define BOOTLOADER_SIZE		(16*1024)	
#define CONFIG_PARAM_SIZE	(16*1024)

#define CONFIG_PARAM_ADDR 	(FLASH_BASE + BOOTLOADER_SIZE)	/*16K bootloader*/
#define FIRMWARE_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE)	/*16K bootloader+ 16 ģ��eeprom*/


#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

#define P_NAME "MiniFly"
#define MCU_ID_ADDRESS          0x1FFF7A10
#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22


#endif /* __CONFIG_H */
