#ifndef __VL53LXX_I2C_H
#define __VL53LXX_I2C_H
#include "sys.h" 
#include "stdbool.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * VL53 IIC驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 增加对vl53l1x的IIC驱动。
********************************************************************************/

/*IO方向设置*/
#define SDA_IN()  {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=0<<4*2;}	//PB4输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=1<<4*2;} //PB4输出模式
/*IO操作函数*/	 
#define VL53_SCL    PBout(5) 	//SCL
#define VL53_SDA    PBout(4) 	//SDA	 
#define READ_SDA	PBin(4)  	//输入SDA 


//VL53所有操作函数
void vl53IICInit(void);			/*初始化VL53的IO口*/				 
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data);		/*读一字节*/
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data);		/*写一字节*/
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);	/*连续读取多个字节*/
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);	/*连续写入多个字节*/
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic 写入某个位*/

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);	/*连续读取多个字节*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*连续写入多个字节*/
	
#endif 


