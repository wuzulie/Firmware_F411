#ifndef __STATE_ESTIMATOR_H
#define __STATE_ESTIMATOR_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ��̬�������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

typedef struct
{
	float vAccDeadband; /* ���ٶ����� */
	float accBias[3];	/* ���ٶ� ƫ��(cm/s/s)*/
	float acc[3];		/* ������ٶ� ��λ(cm/s/s)*/
	float vel[3];		/* �����ٶ� ��λ(cm/s)*/
	float pos[3]; 		/* ����λ�� ��λ(cm)*/
} estimator_t;

void positionEstimate(sensorData_t* sensorData, state_t* state, float dt);	
float getFusedHeight(void);	/*��ȡ�ںϸ߶�*/
void estRstHeight(void);	/*��λ����߶�*/
void estRstAll(void);		/*��λ���й���*/

#endif /* __STATE_ESTIMATOR_H */


