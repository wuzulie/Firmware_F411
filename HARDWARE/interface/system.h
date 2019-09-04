#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos 配置文件 */
#include "FreeRTOSConfig.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Project includes */
#include "config.h"
#include "nvic.h"
#include "exti.h"

/*底层硬件驱动*/
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

/*扩展模块驱动*/
#include "module_detect.h"
#include "module_mgt.h"

void systemInit(void);

#endif /* __SYSTEM_H */














