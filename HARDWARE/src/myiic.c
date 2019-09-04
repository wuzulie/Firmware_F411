#include "myiic.h"
#include "delay.h"

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

//初始化IIC
void IIC_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*使能IIC时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/*SDA PB4	SCL PB5*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);	

	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(1);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

//从指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t IIC_ReadByte(uint8_t devaddr,uint8_t addr)
{				  
	uint8_t temp=0;		  	    																 
	IIC_Start();  
	IIC_Send_Byte(devaddr);//发送器件写命令 	   
	IIC_Wait_Ack(); 
	IIC_Send_Byte(addr);   //发送低地址
	IIC_Wait_Ack();	

	IIC_Start();  	 	   
	IIC_Send_Byte(devaddr|1);//发送器件读命令			   
	IIC_Wait_Ack();	 
	temp=IIC_Read_Byte(0);			   
	IIC_Stop();//产生一个停止条件	    
	return temp;
}
//连续读多个字节
//addr:起始地址
//rbuf:读数据缓存
//len:数据长度
void IIC_Read(uint8_t devaddr,uint8_t addr,uint8_t len,uint8_t *rbuf)
{
	int i=0;
	IIC_Start();  
	IIC_Send_Byte(devaddr);  
	IIC_Wait_Ack();	
	IIC_Send_Byte(addr);//地址自增  
	IIC_Wait_Ack();	

	IIC_Start();  	
	IIC_Send_Byte(devaddr|1);  	
	IIC_Wait_Ack();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = IIC_Read_Byte(0);//最后一个字节不应答
		}
		else
			rbuf[i] = IIC_Read_Byte(1);
	}
	IIC_Stop( );	
}
//从指定地址写入一个数据
//WriteAddr :写入数据的目的地址    
//DataToWrite:要写入的数据
void IIC_WriteByte(uint8_t devaddr,uint8_t addr,uint8_t data)
{				   	  	    																 
	IIC_Start();  
	IIC_Send_Byte(devaddr); //发送器件写命令 	 
	IIC_Wait_Ack();	   
	IIC_Send_Byte(addr);   //发送低地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(data); //发送字节							   
	IIC_Wait_Ack();  		    	   
	IIC_Stop();		//产生一个停止条件 	 
}

//连续写多个字节
//addr:起始地址
//wbuf:写数据缓存
//len:数据的长度
void IIC_Write(uint8_t devaddr,uint8_t addr,uint8_t len,uint8_t *wbuf)
{
	int i=0;
	IIC_Start();  
	IIC_Send_Byte(devaddr);  	
	IIC_Wait_Ack();	
	IIC_Send_Byte(addr);  //地址自增
	IIC_Wait_Ack();	
	for(i=0; i<len; i++)
	{
		IIC_Send_Byte(wbuf[i]);  
		IIC_Wait_Ack();		
	}
	IIC_Stop();	
}

































