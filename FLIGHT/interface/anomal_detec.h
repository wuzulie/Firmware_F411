#ifndef __ANOMAL_DETEC_H
#define __ANOMAL_DETEC_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 异常检测驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define DETEC_ENABLED

#define DETEC_FF_THRESHOLD 	0.05f	/* accZ接近-1.0程度 表示Free Fall */
#define DETEC_FF_COUNT 		50  	/* 自由落体检测计数 1000Hz测试条件 */

#define DETEC_TU_THRESHOLD 	60		/* 碰撞检测阈值60°*/
#define DETEC_TU_COUNT 		100  	/* 碰撞检测计数 1000Hz测试条件 */

/*异常检测*/
void anomalDetec(const sensorData_t *sensorData, const state_t *state, const control_t *control);

#endif	/*__ANOMAL_DETEC_H*/

