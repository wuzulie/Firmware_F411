#ifndef __SENSFUSION6_H
#define __SENSFUSION6_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 6轴数据融合代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);	/*数据融合 互补滤波*/
bool getIsCalibrated(void);
void imuTransformVectorBodyToEarth(Axis3f * v);	/*机体到地球*/
void imuTransformVectorEarthToBody(Axis3f * v);	/*地球到机体*/

#endif

