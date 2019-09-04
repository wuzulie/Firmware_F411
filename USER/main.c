#include "system.h"	/*ͷ�ļ�����*/

/********************************************************************************
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * main.c
 * ����ϵͳ��ʼ���ʹ�������
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

TaskHandle_t startTaskHandle;
static void startTask(void *arg);

int main()
{
    systemInit();			/*�ײ�Ӳ����ʼ��*/

    xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*������ʼ����*/

    vTaskStartScheduler();	/*�����������*/

    while(1) {};
}
/*��������*/
void startTask(void *arg)
{
    taskENTER_CRITICAL();	/*�����ٽ���*/

    xTaskCreate(radiolinkTask, "RADIOLINK", 150, NULL, 5, NULL);		/*����������������*/

    xTaskCreate(usblinkRxTask, "USBLINK_RX", 150, NULL, 4, NULL);		/*����usb��������*/
    xTaskCreate(usblinkTxTask, "USBLINK_TX", 150, NULL, 3, NULL);		/*����usb��������*/

    xTaskCreate(atkpTxTask, "ATKP_TX", 150, NULL, 3, NULL);				/*����atkp������������*/
    xTaskCreate(atkpRxAnlTask, "ATKP_RX_ANL", 300, NULL, 6, NULL);		/*����atkp��������*/

    xTaskCreate(configParamTask, "CONFIG_TASK", 150, NULL, 1, NULL);	/*����������������*/

    xTaskCreate(pmTask, "PWRMGNT", 150, NULL, 2, NULL);					/*������Դ��������*/

    xTaskCreate(sensorsTask, "SENSORS", 450, NULL, 4, NULL);			/*������������������*/

    xTaskCreate(stabilizerTask, "STABILIZER", 450, NULL, 5, NULL);		/*������̬����*/

    xTaskCreate(expModuleMgtTask, "EXP_MODULE", 150, NULL, 1, NULL);	/*������չģ���������*/

    printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());			/*��ӡʣ���ջ��С*/

    vTaskDelete(startTaskHandle);										/*ɾ����ʼ����*/

    taskEXIT_CRITICAL();	/*�˳��ٽ���*/
}

void vApplicationIdleHook( void )
{
    static u32 tickWatchdogReset = 0;

    portTickType tickCount = getSysTickCnt();

    if (tickCount - tickWatchdogReset > WATCHDOG_RESET_MS)
    {
        tickWatchdogReset = tickCount;
        watchdogReset();
    }

    __WFI();	/*����͹���ģʽ*/
}
















