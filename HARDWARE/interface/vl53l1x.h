#ifndef __VL53L1X_H
#define __VL53L1X_H

#include "vl53l1_platform.h"

#include "stabilizer_types.h"
#include "module_mgt.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly	
 * vl53l1x底层驱动代码  移植于ST官方驱动库
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/10/25
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define VL53L1X_MAX_RANGE			410		//410cm

#define VL53L1X_ADDR 				0x52
#define VL53L1X_DEFAULT_ADDRESS 	0x29	//0b0101001

#define VL53L1X_ID			0xEACC

extern VL53L1_Dev_t	dev;	/*vl53l1x 设备*/
int vl53l1xSetParam(void);	/*设置vl53l1x 参数*/

#endif /* __VL53L1X_H */

