#include "iic1.h"
#include "delay.h"
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


static bool isInit;

static void iicDevStart(void);			//发送iic开始信号
static void iicDevStop(void);	  		//发送iic停止信号
static void iicDevAck(void);			//iic发送ACK信号
static void iicDevNAck(void);			//iic不发送ACK信号 
static u8 iicDevWaitAck(void);			//iic等待ACK信号
static void iicDevSendByte(u8 txd);		//iic发送一个字节
static u8 iicDevReceiveByte(u8 ack);	//iic读取一个字节

//初始化iic
void iicDevInit(void)
{	
	if(isInit)	return;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*使能IIC1时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/*SCL PB6   SDA PB7*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);	

	IIC1_SCL=1;
	IIC1_SDA=1;

	isInit=true;
}
//产生IIC1起始信号
static void iicDevStart(void)
{
	SDA_OUT();     //sda线输出
	IIC1_SDA=1;	  	  
	IIC1_SCL=1;
	delay_us(4);
 	IIC1_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC1_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC1停止信号
static void iicDevStop(void)
{
	SDA_OUT();//sda线输出
	IIC1_SCL=0;
	IIC1_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC1_SCL=1; 
	IIC1_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 iicDevWaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC1_SDA=1;delay_us(1);	   
	IIC1_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			iicDevStop();
			return 1;
		}
	}
	IIC1_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
static void iicDevAck(void)
{
	IIC1_SCL=0;
	SDA_OUT();
	IIC1_SDA=0;
	delay_us(1);
	IIC1_SCL=1;
	delay_us(1);
	IIC1_SCL=0;
}
//不产生ACK应答		    
static void iicDevNAck(void)
{
	IIC1_SCL=0;
	SDA_OUT();
	IIC1_SDA=1;
	delay_us(1);
	IIC1_SCL=1;
	delay_us(1);
	IIC1_SCL=0;
}					 				     
//IIC1发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void iicDevSendByte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC1_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC1_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   
		IIC1_SCL=1;
		delay_us(1); 
		IIC1_SCL=0;	
		delay_us(1);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static u8 iicDevReceiveByte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC1_SCL=0; 
        delay_us(1);
		IIC1_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        iicDevNAck();//发送nACK
    else
        iicDevAck(); //发送ACK   
    return receive;
}

//从指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 iicDevReadByte(u8 devaddr,u8 addr, u8* data)
{				  
	u8 temp=0;		  	    																 
	iicDevStart();  
	iicDevSendByte(devaddr);//发送器件写命令 	   
	iicDevWaitAck(); 
	iicDevSendByte(addr);   //发送低地址
	iicDevWaitAck();	

	iicDevStart();  	 	   
	iicDevSendByte(devaddr|1);//发送器件读命令			   
	iicDevWaitAck();	 
	temp=iicDevReceiveByte(0);			   
	iicDevStop();//产生一个停止条件	 
	*data = temp;
	return temp;
}
//连续读多个字节
//addr:起始地址
//rbuf:读数据缓存
//len:数据长度
void iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	iicDevStart();  
	iicDevSendByte(devaddr);  
	iicDevWaitAck();	
	iicDevSendByte(addr);//地址自增  
	iicDevWaitAck();	

	iicDevStart();  	
	iicDevSendByte(devaddr|1);  	
	iicDevWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = iicDevReceiveByte(0);//最后一个字节不应答
		}
		else
			rbuf[i] = iicDevReceiveByte(1);
	}
	iicDevStop( );	
}
//从指定地址写入一个数据
//WriteAddr :写入数据的目的地址    
//DataToWrite:要写入的数据
void iicDevWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	iicDevStart();  
	iicDevSendByte(devaddr); //发送器件写命令 	 
	iicDevWaitAck();	   
	iicDevSendByte(addr);   //发送低地址
	iicDevWaitAck(); 	 										  		   
	iicDevSendByte(data); //发送字节							   
	iicDevWaitAck();  		    	   
	iicDevStop();		//产生一个停止条件 	 
}

//连续写多个字节
//addr:起始地址
//wbuf:写数据缓存
//len:数据的长度
void iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	iicDevStart();  
	iicDevSendByte(devaddr);  	
	iicDevWaitAck();	
	iicDevSendByte(addr);  //地址自增
	iicDevWaitAck();	
	for(i=0; i<len; i++)
	{
		iicDevSendByte(wbuf[i]);  
		iicDevWaitAck();		
	}
	iicDevStop( );	
}
//iic 写入某个位
bool iicDevWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data)
{
    u8 byte;
    iicDevReadByte(devaddr, addr, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    iicDevWriteByte(devaddr, addr, byte);
	return true;
}























