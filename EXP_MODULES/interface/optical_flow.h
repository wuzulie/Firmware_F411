#ifndef __OPTICAL_FLOW_H
#define __OPTICAL_FLOW_H
#include "sys.h"
#include <stdbool.h>
#include "spi.h"
#include "stabilizer_types.h"
#include "module_mgt.h"
#include "vl53lxx.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 光流模块驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 增加光流数据结构体opFlow_t，用于存放光流各项数据，方便用户调试。
********************************************************************************/

typedef struct opFlow_s 
{
	float pixSum[2];		/*累积像素*/
	float pixComp[2];		/*像素补偿*/
	float pixValid[2];		/*有效像素*/
	float pixValidLast[2];	/*上一次有效像素*/
	
	float deltaPos[2];		/*2帧之间的位移 单位cm*/
	float deltaVel[2];		/*速度 单位cm/s*/
	float posSum[2];		/*累积位移 单位cm*/
	float velLpf[2];		/*速度低通 单位cm/s*/
	
	bool isOpFlowOk;		/*光流状态*/
	bool isDataValid;		/*数据有效*/

} opFlow_t;

extern opFlow_t opFlow;

void opticalFlowPowerControl(bool state);	//光流电源控制
bool getOpFlowData(state_t *state, float dt);	//读取光流数据
void opticalFlowInit(void);		/*初始化光流模块*/
bool getOpDataState(void);		/*光流数据状态*/

#endif
