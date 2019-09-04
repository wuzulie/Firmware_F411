#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 模拟IIC驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.1
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=0<<4*2;}	//PB4输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=1<<4*2;} //PB4输出模式
//IO操作函数	 
#define IIC_SCL    PBout(5) //SCL
#define IIC_SDA    PBout(4) //SDA	 
#define READ_SDA   PBin(4)  //输入SDA 


//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号 
uint8_t IIC_ReadByte(uint8_t devaddr,uint8_t addr);	/*读一字节*/
void IIC_WriteByte(uint8_t devaddr,uint8_t addr,uint8_t data);	/*写一字节*/
void IIC_Read(uint8_t devaddr,uint8_t addr,uint8_t len,uint8_t *rbuf);	/*连续读取多个字节*/
void IIC_Write(uint8_t devaddr,uint8_t addr,uint8_t len,uint8_t *wbuf);/*连续写入多个字节*/

#endif
















