#ifndef __VL53LXX_H
#define __VL53LXX_H

#include "vl53l0x.h"
#include "vl53l1x.h"

#include "stabilizer_types.h"
#include "module_mgt.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly	
 * vl53lxx应用代码, 包括vl53l0x和vl53l1x
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


extern u16 vl53lxxId;	/*vl53芯片ID*/
extern bool isEnableVl53lxx;
extern zRange_t vl53lxx;


void vl53lxxInit(void);
bool vl53lxxReadRange(zRange_t* zrange);
void setVl53lxxState(u8 enable);

#endif /* __VL53LXX_H */

