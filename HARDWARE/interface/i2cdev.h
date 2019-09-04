#ifndef __I2CDEV_H
#define __I2CDEV_H
#include <stdint.h>
#include <stdbool.h>
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


#define I2CDEV_NO_MEM_ADDR  0xFF

typedef I2cDrv    I2C_Dev;
#define I2C1_DEV  &sensorsBus
#define I2C3_DEV  &deckBus

/**
 * Read bytes from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);

/**
 * Read bytes from an I2C peripheral with a 16bit internal reg/mem address
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevRead16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data);

/**
 * I2C device init function.
 * @param I2Cx  Pointer to I2C peripheral to initialize.
 *
 * @return TRUE if initialization went OK otherwise FALSE.
 */
int i2cdevInit(I2C_Dev *dev);

/**
 * Read a byte from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t *data);

/**
 * Read a bit from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number 0 - 7 to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data);
/**
 * Read up to 8 bits from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to read, 1 - 8.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t *data);

/**
 * Write bytes to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data from that will be written.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);

/**
 * Write bytes to an I2C peripheral with 16bit internal reg/mem address.
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data from that will be written.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWrite16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data);

/**
 * Write a byte to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  The byte to write.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t data);

/**
 * Write a bit to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number, 0 - 7, to write.
 * @param data  The bit to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data);

/**
 * Write up to 8 bits to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to write, 1 - 8.
 * @param data  The byte containing the bits to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

#endif //__I2CDEV_H
