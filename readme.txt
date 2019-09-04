
最新资料下载地址:
	http://www.openedv.com/thread-105197-1-1.html

MiniFly外形:

         HEAD
	  M4  ↑  M1
	   \     /
		\   /
		 \ /
		 / \
		/   \
	   /     \
	  M3     M2
	
硬件资源:
	1,MCU:STM32F411CEU6 (FLAH:512K, RAM:128K, 系统运行时钟频率:96MHz)
	2,9轴MPU9250连接在IIC1上(IMU_SCL:PB8, IMU_SDA:PB9, 通信方式:模拟IIC) 
	3,气压计BMP280连接在MPU9250的辅助IIC上(AUX_DA,AUX_CL)
	4,无线通信NFR51822连接在UART2上(NRF_RX:PA2, NRF_TX:PA3, NRF_FLOW_CTRL:PA0) 
	5,MOTOR1连接在TIM4_CH2上(PB7)
	6,MOTOR2连接在TIM4_CH1上(PB6)
	7,MOTOR3连接在TIM2_CH3上(PB10)
	8,MOTOR4连接在TIM2_CH1上(PA5)
	9,LED_BLUE_L连接在PB12上	(MORTOR3对应的蓝色LED, 高电平有效)
	10,LED_GREEN_L连接在PA6上	(MORTOR4对应的绿色LED, 低电平有效)
	11,LED_RED_L连接在PA7上		(MORTOR4对应的红色LED, 低电平有效)
	12,LED_GREEN_R连接在PC13上	(MORTOR1对应的绿色LED, 低电平有效)
	13,LED_RED_R连接在PC14上	(MORTOR1对应的红色LED, 低电平有效)
	14,扩展IIC接口(SDA:PB4, SCL:PB5) 
	15,扩展SPI2接口(SCK:PB13, MISO:PB14, MOSI:PB15)  
	16,扩展UART1接口(RX1:PB3, TX1:PA15, 外挂摄像头模块需用此接口)  
	17,扩展GPIO(CS0:PC15, CS1:PB0, CS2:PB1, CS3:PA8). 	
	18,USB_SLAVE接口(USB_ID:PA10, USB_DM:PA11, USB_DP:PA12)

实验现象:
	MiniFly开机后，MOTOR1~4电机以20%的占空比依次转动50ms,然后关闭电机; 
	
	灯语:
		LED_BLUE_L:充电指示灯，充电时1S周期闪烁,停止充电后常亮;
		LED_GREEN_L:无线通信数据接收指示灯，接收一个包闪一次;
		LED_RED_L:无线通信数据发送指示灯，发送一个包闪一次;
		LED_GREEN_R:2S周期慢闪指示传感器未校准，校准完成后0.5S周期快闪;
		LED_RED_R:常亮表示电池处于低电量状态，请停止飞行，然后给电池充电;
		LED_RED_L和LED_RED_R同时常量，表示MiniFly进入错误状态; 

注意事项:
	代码下载和调试前，请将下载器开关拨到STM32档。
	Bootloader起始地址(BOOTLOADER_ADDR) 0x08000000;
	固件起始地址(FIRMWARE_START_ADDR) 	0x08008000;


固件更新记录:
	Firmware V1.0 Release(硬件版本:V1.32, DATE:2017-06-30)
	
	
	Firmware V1.1 Release(硬件版本:V1.32和V1.4, DATE:2017-10-20)
		1.MCU和传感器通信采用硬件IIC,读取传感器采用中断方式，数据读取速率更快;
		2.创建传感器任务sensorsTask，用于读取和处理传感器数据，处理的传感器数据放入队列，等待被使用，降低stabilizerTask任务负担;
		3.陀螺仪和加速计采用2阶低通滤波，滤波效果更好，降低振动，飞行更稳定;
		4.创建扩展模块任务expModuleMgtTask，以及文件分组EXP_MODULES,用于管理扩展模块;
		5.扩展接口增加wifi航拍模块ATK-WIFI-MODULE，支持手机（平板）wifi控制MiniFly(飞行，拍照，录像);
		6.扩展接口增加酷炫RGB灯环模块ATK-LED-RING,并提供10种灯环效果用作测试，用户可以开发自己喜欢的灯环效果;
		7.每个扩展模块都有自己的ID，我们在HARDWARE分组下增加module_detect.c,用于检测MiniFly挂载的扩展模块;
		8.扩展模块的使用说明请参考模块用户手册;


	Firmware V1.2 Release(硬件版本:V1.4, DATE:2018-6-22)
		1.修改姿态估测功能，根据加速度估测速度以及位置，同时使用气压计或者激光数据(需搭配光流模块)校正速度和位置信息;
		2.修改定高方式，采用串级PID(位置环+速度环)控制高度，增加稳定性的同时提高了响应速度，定高也不会受到重量影响，默认一键起飞高度80cm;
		3.修改一键降落功能，四轴自动降落触地停机或者着陆之后减速停机，在定高和定点模式，拉油门过大，四轴触地停机;
		4.增加光流定点，激光(2m以内)定高功能，MiniFly搭配我们的光流模块即可实现稳定悬停(遥控器设置为定点模式),
			光流模块使用扩展SPI2接口和扩展IIC接口，并创建光流任务和激光测距任务.定点模式关闭空翻功能;
		5.增加SPL06气压计功能，代码自动检测并使用BMP280或者SPL06;
		6.COMMON分组添加filter2.c和maths.c源文件，用于对某些函数的支持;
		7.修改抛飞功能，水平抛飞，抛出之后稳定在抛飞高度;
		8.修改扩展模块管理方式，MiniFly上电之后，实时监测扩展模块状态，实现扩展模块即插即用(同时只支持一个扩展模块)，
			扩展模块相关应用源码文件移至分组EXP_MODULES;
		9.修改遥控控制频率和光流数据读取100Hz,姿态融合频率和角度环PID频率为250Hz,空翻检测和控制输出500Hz;
		10.修改遥控在未开机状态下四轴也进入抛飞状态的bug,修改之后，四轴需在遥控器设置为定高或者定点模式并且解锁状态，才能实现抛飞功能;
		11.修改写入PID参数后不能保存的bug,修改之后，写入的PID参数，在四轴停机1.5s之后自动保存;
		12.扩展模块的使用说明请参考对应模块用户手册;


	Firmware V1.2.1 Release(硬件版本:V1.4, DATE:2018-8-16)
		1.关闭电机油门补偿(motors.h : ENABLE_THRUST_BAT_COMPENSATED)，增加空翻稳定性。

	
	Firmware V1.3 Release(硬件版本:V1.4, DATE:2018-11-16)
		1.移植iNAV_Flight定点算法，光流融合加速计，定点更稳，抗干扰更强;
		2.支持4m版本光流模块,且飞控自动识别2m版本光流和4m版本光流模块，4m版本光流抗干扰能力更强;
		3.挂载光流模块后，用户可在遥控端选择是否使用激光传感器定高，关闭后使用气压计定高，方便用户室外飞行;
		4.支持微调记录功能，四轴上电并于遥控器通信正常后，读取四轴微调数据，这样可以保证同一遥控器控制其它四轴时，不用重新微调;
		5.增加用户数据(USERDATA)上传功能，方便用户上传需要调试的数据到上位机，上位机实时打印波形，方便用户调试;
		6.纠正姿态PID初始化参数错误的bug;
		7.纠正一键降落经常不能稳定着陆的bug;
		

					正点原子@ALIENTEK
					2017-06-30
					广州市星翼电子科技有限公司
					电话：020-38271790
					传真：020-36773971
					购买：http://shop62103354.taobao.com
					http://shop62057469.taobao.com
					公司网站：www.alientek.com
					技术论坛：www.openedv.com
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
















