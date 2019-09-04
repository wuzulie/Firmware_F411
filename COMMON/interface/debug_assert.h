#ifndef __DEBUG_ASSERT_H
#define __DEBUG_ASSERT_H
#include "console.h"

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
