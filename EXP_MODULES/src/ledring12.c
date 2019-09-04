#include <stdint.h>
#include <math.h>
#include <string.h>
#include "ws2812.h"
#include "ledring12.h"
#include "stabilizer.h"
#include "module_mgt.h"

#include "FreeRTOS.h"
#include "timers.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * RGB�ƻ�Ч����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define NBR_LEDS  12	//ws2812 RGB�Ƹ���

#define COPY_COLOR(dest, orig)  dest[0]=orig[0]; dest[1]=orig[1]; dest[2]=orig[2]
#define ADD_COLOR(dest, o1, o2)  dest[0]=(o1[0]>>1)+(o2[0]>>1);dest[1]=(o1[1]>>1)+(o2[1]>>1);dest[2]=(o1[2]>>1)+(o2[2]>>1);
#define LIMIT(a) ((a>255)?255:(a<0)?0:a)
#define SIGN(a) ((a>=0)?1:-1)
#define DEADBAND(a, b) ((a<b) ? 0:a)

typedef void (*Ledring12Effect)(uint8_t buffer[][3], bool reset);

static xTimerHandle timer;

static const uint8_t black[3] 	= {0x00, 0x00, 0x00};//��ɫ��������
//static const uint8_t white[3] 	= {0xff, 0xff, 0xff};//��ɫ
//static const uint8_t red[3]		= {0xff, 0x00, 0x00};//��ɫ
//static const uint8_t green[3] 	= {0x00, 0xff, 0x00};//��ɫ
static const uint8_t blue[3]	= {0x00, 0x00, 0xff};//��ɫ

uint32_t effect = 1;
static bool isInit = false;
static bool headlightEnable = true;

/**************** blackEffect(LED����) ***************/
static void blackEffect(uint8_t buffer[][3], bool reset)
{
	int i;
	if(reset)
	{
		for (i=0; i<NBR_LEDS; i++) 
		{
		  buffer[i][0] = 0;
		  buffer[i][1] = 0;
		  buffer[i][2] = 0;
		}
	}
	headlightEnable = false;
}

/*************** colorTest(10����ɫ����) **************/
#define TEST_COLOR_NUM  9
#define TEST_DELAY 		10
static uint8_t test_pat[TEST_COLOR_NUM][3] = {
{0,  0,  0 },//��ɫ��������
{32, 32, 32},//��ɫ
{64, 0,  0 },//��ɫ
{0,  64, 0 },//��ɫ
{0,  0,  64},//��ɫ
{64, 64, 0 },//��ɫ
{64, 32, 0 },//��ɫ
{64, 0,  64},//�Ϻ�ɫ
{0,  64, 64},//��ɫ
};
static uint8_t test_eff_nbr = 0;
static uint8_t test_delay_counter = 0;
static uint8_t headlight_test_counter =0;
static bool test_front = false;
static void colorTest(uint8_t buffer[][3], bool reset)
{
	for(int i=0; i<NBR_LEDS; i++)
	{
		buffer[i][0] = test_pat[test_eff_nbr][0];
		buffer[i][1] = test_pat[test_eff_nbr][1];
		buffer[i][2] = test_pat[test_eff_nbr][2];
	}

	test_delay_counter++;
	headlight_test_counter++;

	if (test_delay_counter > TEST_DELAY) 
	{
		test_delay_counter = 0;
		test_eff_nbr = (test_eff_nbr + 1)%TEST_COLOR_NUM;
	}

	if (headlight_test_counter > (TEST_DELAY*3)) 
	{
		headlight_test_counter = 0;
		test_front = !test_front;
		headlightEnable = test_front;
	}
}

/************ attitudeInduceEffect(��̬��Ӧ) *************/
static void attitudeInduceEffect(uint8_t buffer[][3], bool reset)
{
	attitude_t get;
	const int led_middle = 10;
	
	if (reset)
	{
		int i;
		for (i=0; i<NBR_LEDS; i++) 
		{
			buffer[i][0] = 0;
			buffer[i][1] = 0;
			buffer[i][2] = 0;
		}
	}

	getAttitudeData(&get);
	float pitch = -get.pitch;
	float roll  = get.roll;

	pitch = (pitch>20)?20:(pitch<-20)?-20:pitch;
	roll = (roll>20)?20:(roll<-20)?-20:roll;

	pitch=SIGN(pitch)*pitch*pitch;
	roll*=SIGN(roll)*roll;

	buffer[8][0] = LIMIT(led_middle + pitch);
	buffer[9][0] = LIMIT(led_middle + pitch);
	buffer[10][0] = LIMIT(led_middle + pitch);

	buffer[5][2] = LIMIT(led_middle - roll);
	buffer[6][2] = LIMIT(led_middle - roll);
	buffer[7][2] = LIMIT(led_middle - roll);

	buffer[2][0] = LIMIT(led_middle - pitch);
	buffer[3][0] = LIMIT(led_middle - pitch);
	buffer[4][0] = LIMIT(led_middle - pitch);

	buffer[11][2] = LIMIT(led_middle + roll);
	buffer[0][2] = LIMIT(led_middle + roll);
	buffer[1][2] = LIMIT(led_middle + roll);
}

/************** gyroInduceEffect(���ݸ�Ӧ) ****************/
#define MAX_RATE 512
static void gyroInduceEffect(uint8_t buffer[][3], bool reset)
{
	static sensorData_t sensorData;
	static uint8_t brightness = 0;
	
	getSensorData(&sensorData);
	int i;
	int gyroX = sensorData.gyro.x;
	int gyroY = sensorData.gyro.y;
	int gyroZ = sensorData.gyro.z;

	gyroX = (gyroX>MAX_RATE) ? MAX_RATE:(gyroX<-MAX_RATE) ? -MAX_RATE:gyroX;
	gyroY = (gyroY>MAX_RATE) ? MAX_RATE:(gyroY<-MAX_RATE) ? -MAX_RATE:gyroY;
	gyroZ = (gyroZ>MAX_RATE) ? MAX_RATE:(gyroZ<-MAX_RATE) ? -MAX_RATE:gyroZ;

	gyroX = SIGN(gyroX) * gyroX / 2;
	gyroY = SIGN(gyroY) * gyroY / 2;
	gyroZ = SIGN(gyroZ) * gyroZ / 2;

	gyroX = DEADBAND(gyroX, 5);
	gyroY = DEADBAND(gyroY, 5);
	gyroZ = DEADBAND(gyroZ, 5);

	for (i=0; i < NBR_LEDS; i++)
	{
	  buffer[i][0] = (uint8_t)(LIMIT(gyroZ));
	  buffer[i][1] = (uint8_t)(LIMIT(gyroY));
	  buffer[i][2] = (uint8_t)(LIMIT(gyroX));
	}
	brightness++;
}

/************* binkLED(��ɫLED��˸����) **************/
static void binkLED(uint8_t buffer[][3], bool reset)
{
	int i;
	static int tic = 0;

	if(reset)
	{
		for (i=0; i<NBR_LEDS; i++) 
		{
			COPY_COLOR(buffer[i], black);
		}
	}

	if((tic < 10) && (tic & 1))
	{
		for (i=0; i<NBR_LEDS; i++) 
		{
			COPY_COLOR(buffer[i], blue);
		}
	}
	else
	{
		for(i=0; i<NBR_LEDS; i++) 
		{
			COPY_COLOR(buffer[i], black);
		}
	}
	if(++tic >= 20) tic = 0;
}

/************* flashlight(��ɫ�ֵ�Ͳ) ****************/
static void flashlight(uint8_t buffer[][3], bool reset)
{
	for(int i=0; i<NBR_LEDS; i++) 
	{
		buffer[i][0] = 0XFF;
		buffer[i][1] = 0XFF;
		buffer[i][2] = 0XFF;
	}
}

/**************** breathingLED(������) ***************/
static float glowstep = 3.0f;
static void breathingLED(uint8_t buffer[][3], bool reset)
{
	int i;
	static bool dir;
	static uint8_t kind = 1;
	static float brightness=0;

	if(reset)
	{		
		brightness = 0;
		dir = true;
	}
	
	if(dir==true)
	{
		brightness ++;
		if(brightness > 30)
			dir = false;
	}
	else
	{
		brightness --;

		if(brightness <= 0)
		{
			dir = true;
			
			if(++kind == 8)
			{
				kind = 1;
			}
		}			
	}
	
	for (i=0; i<NBR_LEDS; i++)
	{
		if(kind & 0x1)
			buffer[i][0] = brightness * glowstep;
		else
			buffer[i][0] = 0;
		
		if(kind & 0x2)
			buffer[i][1] = brightness * glowstep;
		else
			buffer[i][1] = 0;
			
		if(kind & 0x4)
			buffer[i][2] = brightness * glowstep;
		else
			buffer[i][2] = 0;
	}
}

/******************** Red spin(��ɫ��ˮ��) *****************/
static const uint8_t blueRing[][3] = {{64, 64, 255}, {32,32,64}, {8,8,16},
									  {0,0,0}, {0,0,0},{0,0,0},
									  {0,0,0}, {0,0,0},{0,0,0},
									  {0,0,0}, {0,0,0},{0,0,0},
                                      };
static const uint8_t redRing[][3] = { {128, 0, 0}, {64,0,0}, {32,0,0},
                                      {4,0,0}, {2,0,0}, {1,0,0},
									  {0,0,0}, {0,0,0},{0,0,0},
									  {0,0,0}, {0,0,0},{0,0,0},
                                      };

static void redSpinEffect(uint8_t buffer[][3], bool reset)
{
	int i;
	uint8_t temp[3];

	if (reset)
	{
		for (i=0; i<NBR_LEDS; i++) 
		{
			COPY_COLOR(buffer[i], redRing[i]);
		}
	}
	COPY_COLOR(temp, buffer[0]);
	for (i=0; i<(NBR_LEDS-1); i++)
	{
		COPY_COLOR(buffer[i], buffer[i+1]);
	}
	COPY_COLOR(buffer[(NBR_LEDS-1)], temp);
}

/****************** Color spin(�ס��졢����ɫ��ˮ��) *******************/
static const uint8_t colorRing[][3] = {{0,0,32},{0,0,16}, 	{0,0,8},
                                       {0,0,4}, {16,16,16}, {8,8,8},
                                       {4,4,4},	{32,0,0},	{16,0,0},
                                       {8,0,0}, {4,0,0}, 	{2,0,0},
                                      };

static void colorSpinEffect(uint8_t buffer[][3], bool reset)
{
	int i;
	uint8_t temp[3];

	if (reset)
	{
		for (i=0; i<NBR_LEDS; i++) 
		{
			COPY_COLOR(buffer[i], colorRing[i]);
		}
	}

	COPY_COLOR(temp, buffer[0]);
	for (i=0; i<(NBR_LEDS-1); i++) 
	{
		COPY_COLOR(buffer[i], buffer[i+1]);
	}
	COPY_COLOR(buffer[(NBR_LEDS-1)], temp);
}

/**************** spinEffect2(��ʱ����ˮ��) ***************/
static void spinEffect2(uint8_t buffer[][3], bool reset)
{
	int i;
	uint8_t temp[3];

	if(reset)
	{
		for (i=0; i<NBR_LEDS; i++) 
		{
			COPY_COLOR(buffer[(NBR_LEDS-i)%NBR_LEDS], blueRing[i]);
		}
	}

	COPY_COLOR(temp, buffer[(NBR_LEDS-1)]);
	for(i=(NBR_LEDS-1); i>=0; i--) 
	{
		COPY_COLOR(buffer[i], buffer[i-1]);
	}
	COPY_COLOR(buffer[0], temp);
}

/************ doubleSpinEffect(˳��ʱ����) **************/
static void doubleSpinEffect(uint8_t buffer[][3], bool reset) 
{
	static uint8_t sub1[NBR_LEDS][3];
	static uint8_t sub2[NBR_LEDS][3];
	int i;
	static int step;
	if (reset) step = 0;
	redSpinEffect(sub1, reset);
	spinEffect2(sub2, reset);
	for (i=0; i<NBR_LEDS; i++)
	{
		ADD_COLOR(buffer[i], sub1[i], sub2[i]);/*��ɫ�ϲ�*/
	}
	step ++;
}

/**************** Effect list ***************/
Ledring12Effect effectsFct[] =
{
	blackEffect,			//ȫ�ڣ��ر�״̬
	colorTest,				//10����ɫ����
	attitudeInduceEffect,	//��ָ̬ʾЧ��
	gyroInduceEffect,		//������ǿ��ָʾЧ��
	binkLED,				//��ɫ��˸Ч��
	flashlight,				//��ɫ�����ֵ�ͲЧ��
	breathingLED,			//��ɫ������Ч��	
	redSpinEffect,			//��ɫ��ˮ��Ч��
	colorSpinEffect,		//��ɫ��ˮ��Ч��
	doubleSpinEffect,		//˫ɫ��ˮ��Ч��
	spinEffect2,
};

//�ƻ������ʱ���ж�
static void ledring12Timer(xTimerHandle timer)
{
	bool reset = true;
	static int current_effect = 0;
	static uint8_t buffer[NBR_LEDS][3];
	
	if(getModuleID() != LED_RING)	/*ȡ�µƻ�*/
	{				
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTimerStopFromISR(timer, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken != pdFALSE)
		{
			portYIELD();
		}
	}
	
	if(current_effect != effect)
		reset = true;
	else 
		reset = false;
	current_effect = effect;
	
	effectsFct[current_effect](buffer, reset);
	ws2812Send(buffer, NBR_LEDS);
	setHeadlightsOn(headlightEnable);
}

//�ƻ���ʼ��
void ledring12Init(void)
{
	ws2812Init();
	
	if(!isInit)	/*��һ�ν��ϵƻ�*/
	{	
		timer = xTimerCreate( "ringTimer", 50, pdTRUE, NULL, ledring12Timer);
	}	
	xTimerStart(timer, 100);

	isInit = true;
}

//�ƻ���Դ����
void ledringPowerControl(bool state)
{
	ws2812PowerControl(state);
}

//�ƻ�Ч������
void setLedringEffect(u8 set)
{
	effect = set;
}

