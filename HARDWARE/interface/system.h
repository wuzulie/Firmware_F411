#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos �����ļ� */
#include "FreeRTOSConfig.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Project includes */
#include "config.h"
#include "nvic.h"
#include "exti.h"

/*�ײ�Ӳ������*/
#include "sys.h"
#include "delay.h"
#include "uart1.h"
#include "led.h"
#include "ledseq.h"
#include "radiolink.h"
#include "usblink.h"
#include "config_param.h"
#include "comm.h"
#include "commander.h"
#include "sensors.h"
#include "stabilizer.h"
#include "watchdog.h"
#include "pm.h"

/*��չģ������*/
#include "module_detect.h"
#include "module_mgt.h"

void systemInit(void);

#endif /* __SYSTEM_H */














