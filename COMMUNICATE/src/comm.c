#include "comm.h"
#include "config.h"
#include "console.h"
#include "radiolink.h"
#include "usblink.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 通信驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit;

void commInit(void)
{
	if (isInit) return;
	radiolinkInit();	/*无线通信初始化*/
	usblinkInit();		/*USB通信初始化*/
	isInit = true;
}

bool commTest(void)
{
  bool pass=isInit;
  
  pass &= consoleTest();
  
  return pass;
}

