#ifndef __EXP_MODULE_H
#define __EXP_MODULE_H
#include "sys.h"
#include "module_detect.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��չģ�������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

enum expModuleID getModuleID(void);
void expModulePower(bool state);
void expModuleMgtTask(void* param);


#endif /* __EXP_MODULE_H */

