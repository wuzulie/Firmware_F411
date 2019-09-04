#ifndef __IIC1_H
#define __IIC1_H
#include "sys.h" 
#include "stdbool.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * IIC1驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

/*IO方向设置*/
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
/*IO操作函数*/	 
#define IIC1_SCL    PBout(8) 	//SCL
#define IIC1_SDA    PBout(9) 	//SDA	 
#define READ_SDA	PBin(9)  	//输入SDA 


//IIC1所有操作函数
void iicDevInit(void);					/*初始化IIC1的IO口*/				 
//u8 iicDevReadByte(u8 devaddr,u8 addr);	/*读一字节*/
u8 iicDevReadByte(u8 devaddr,u8 addr, u8* data);	/*读一字节*/
void iicDevWriteByte(u8 devaddr,u8 addr,u8 data);	/*写一字节*/
void iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);/*连续读取多个字节*/
void iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);/*连续写入多个字节*/
bool iicDevWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic 写入某个位*/

#endif
















