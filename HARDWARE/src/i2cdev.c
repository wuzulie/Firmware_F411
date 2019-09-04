#include <stdint.h>
#include <stdbool.h>
#include "i2cdev.h"
#include "i2c_drv.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * IIC器件读写控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


/**
 * IIC器件初始化
 */
int i2cdevInit(I2C_Dev *dev)
{
	i2cdrvInit(dev);

	return true;
}

/**
 * IIC读取外设一个字节
 */
bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t *data)
{
	return i2cdevRead(dev, devAddress, memAddress, 1, data);
}

/**
 * IIC读取外设某一个位
 */
bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data)
{
	uint8_t byte;
	bool status;

	status = i2cdevRead(dev, devAddress, memAddress, 1, &byte);
	*data = byte & (1 << bitNum);

	return status;
}

/**
 * IIC读取外设多个位
 */
bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	bool status;
	uint8_t byte;

	if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		byte &= mask;
		byte >>= (bitStart - length + 1);
		*data = byte;
	}
	return status;
}
/**
 * IIC读取外设多个字节
 */
bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data)
{
	I2cMessage message;

//	if (memAddress == I2CDEV_NO_MEM_ADDR)
//	{
//		i2cdrvCreateMessage(&message, devAddress, i2cRead, len, data);
//	}
//	else
	{
		i2cdrvCreateMessageIntAddr(&message, devAddress, false, memAddress, i2cRead, len, data);
	}

	return i2cdrvMessageTransfer(dev, &message);
}
/**
 * IIC读取外设16位寄存器（内存）
 */
bool i2cdevRead16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data)
{
	I2cMessage message;

	i2cdrvCreateMessageIntAddr(&message, devAddress, true, memAddress, i2cRead, len, data);

	return i2cdrvMessageTransfer(dev, &message);
}
/**
 * IIC写入外设一个字节
 */
bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t data)
{
	return i2cdevWrite(dev, devAddress, memAddress, 1, &data);
}
/**
 * IIC写入外设某一个位
 */
bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(dev, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}
/**
 * IIC写入外设多个位
 */
bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data)
{
	bool status;
	uint8_t byte;

	if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		byte &= ~(mask); // zero all important bits in existing byte
		byte |= data; // combine data with existing byte
		status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
	}

	return status;
}
/**
 * IIC写入外设多个字节
 */
bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data)
{
	I2cMessage message;

//	if (memAddress == I2CDEV_NO_MEM_ADDR)
//	{
//		i2cdrvCreateMessage(&message, devAddress, i2cWrite, len, data);
//	}
//	else
	{
		i2cdrvCreateMessageIntAddr(&message, devAddress, false, memAddress, i2cWrite, len, data);
	}

	return i2cdrvMessageTransfer(dev, &message);
}
/**
 * IIC写入外设16位寄存器（内存）
 */
bool i2cdevWrite16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data)
{
	I2cMessage message;

	i2cdrvCreateMessageIntAddr(&message, devAddress, true, memAddress, i2cWrite, len, data);

	return i2cdrvMessageTransfer(dev, &message);
}
