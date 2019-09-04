#ifndef __DEBUG_ASSERT_H
#define __DEBUG_ASSERT_H
#include "console.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 断言驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )

#ifdef DEBUG
	#define IF_DEBUG_ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )
#else
	#define IF_DEBUG_ASSERT(e)
#endif

#define ASSERT_FAILED() assertFail( "", __FILE__, __LINE__ )

/**
 * Assert handler function
 */
void assertFail(char *exp, char *file, int line);
/**
 * Print assert snapshot data
 */
void printAssertSnapshotData(void);
/**
 * Store assert snapshot data to be read at startup if a reset is triggered (watchdog)
 */
void storeAssertSnapshotData(char *file, int line);

#endif //__DEBUG_ASSERT_H
