#include "config.h"
#include "stdbool.h"
#include "delay.h"
#include "ak8963.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * AK8963驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static uint8_t devAddr;
static u8 buffer[6];
static I2C_Dev *I2Cx;
static bool isInit;

static bool ak8963EvaluateSelfTest(s16 min, s16 max, s16 value, char* string);

void ak8963Init(I2C_Dev *i2cPort)
{
	if (isInit)
		return;

	I2Cx = i2cPort;
	devAddr = AK8963_ADDRESS_00;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ak8963TestConnection()
{
	if (i2cdevReadByte(I2Cx, devAddr, AK8963_RA_WIA, buffer) == 1)
	{
		return (buffer[0] == 0x48);
	}
	return false;
}

/*AK8963自检*/
bool ak8963SelfTest(void)
{
	bool testStatus = true;
	s16 mx, my, mz;  // positive magnetometer measurements
	u8 confSave;
	u8 timeout = 20;
	
	if (i2cdevReadByte(I2Cx, devAddr, AK8963_RA_CNTL, &confSave) == false)
	{
		// TODO: error handling
		return false;
	}

	ak8963SetMode(AK8963_MODE_POWERDOWN);
	ak8963SetSelfTest(true);
	ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_SELFTEST);
	ak8963GetOverflowStatus();// Clear ST1 by reading ST2
	
	while (!ak8963GetDataReady() && timeout--)
	{
		delay_xms(1);
	}
	ak8963GetHeading(&mx, &my, &mz);
	ak8963SetMode(AK8963_MODE_POWERDOWN);

	if (ak8963EvaluateSelfTest(AK8963_ST_X_MIN, AK8963_ST_X_MAX, mx, "X") &&
		ak8963EvaluateSelfTest(AK8963_ST_Y_MIN, AK8963_ST_Y_MAX, my, "Y") &&
		ak8963EvaluateSelfTest(AK8963_ST_Z_MIN, AK8963_ST_Z_MAX, mz, "Z"))
	{
		printf("AK8963 Self test [OK].\n");
	}else
	{
		printf("AK8963 Self test [FAIL].\n");
		testStatus = false;
	}
	
	ak8963SetMode(confSave);// Power up with saved config

	return testStatus;
}


static bool ak8963EvaluateSelfTest(s16 min, s16 max, s16 value, char* string)
{
	if (value < min || value > max)
	{
		printf("Self test %s [FAIL]. low: %d, high: %d, measured: %d\n", string, min, max, value);
		return false;
	}
	return true;
}

u8 ak8963GetDeviceID()
{
	i2cdevReadByte(I2Cx, devAddr, AK8963_RA_WIA, buffer);
	return buffer[0];
}

// INFO 寄存器
u8 ak8963GetInfo()
{
	i2cdevReadByte(I2Cx, devAddr, AK8963_RA_INFO, buffer);
	return buffer[0];
}

// ST1 寄存器
u8 ak8963GetDataReady()
{
	i2cdevReadBit(I2Cx, devAddr, AK8963_RA_ST1, AK8963_ST1_DRDY_BIT, buffer);
	return buffer[0];
}

// H* 寄存器
void ak8963GetHeading(s16 *x, s16 *y, s16 *z)
{
	i2cdevRead(I2Cx, devAddr, AK8963_RA_HXL, 6, buffer);
	*x = (((s16) buffer[1]) << 8) | buffer[0];
	*y = (((s16) buffer[3]) << 8) | buffer[2];
	*z = (((s16) buffer[5]) << 8) | buffer[4];
}
s16 ak8963GetHeadingX()
{
	i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_SINGLE);
	i2cdevRead(I2Cx, devAddr, AK8963_RA_HXL, 2, buffer);
	return (((s16) buffer[1]) << 8) | buffer[0];
}
s16 ak8963GetHeadingY()
{
	i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_SINGLE);
	i2cdevRead(I2Cx, devAddr, AK8963_RA_HYL, 2, buffer);
	return (((s16) buffer[1]) << 8) | buffer[0];
}
s16 ak8963GetHeadingZ()
{
	i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_SINGLE);
	i2cdevRead(I2Cx, devAddr, AK8963_RA_HZL, 2, buffer);
	return (((s16) buffer[1]) << 8) | buffer[0];
}

// ST2 寄存器
bool ak8963GetOverflowStatus()
{
	i2cdevReadBit(I2Cx, devAddr, AK8963_RA_ST2, AK8963_ST2_HOFL_BIT, buffer);
	return buffer[0];
}
bool ak8963GetDataError()
{
	i2cdevReadBit(I2Cx, devAddr, AK8963_RA_ST2, AK8963_ST2_DERR_BIT, buffer);
	return buffer[0];
}

// CNTL 寄存器
u8 ak8963GetMode()
{
	i2cdevReadByte(I2Cx, devAddr, AK8963_RA_CNTL, buffer);
	return buffer[0];
}
void ak8963SetMode(u8 mode)
{
	i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, mode);
}
void ak8963Reset()
{
	i2cdevWriteBits(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_CNTL_MODE_BIT,
					AK8963_CNTL_MODE_LENGTH, AK8963_MODE_POWERDOWN);
}

// ASTC 寄存器
void ak8963SetSelfTest(bool enabled)
{
	i2cdevWriteBit(I2Cx, devAddr, AK8963_RA_ASTC, AK8963_ASTC_SELF_BIT, enabled);
}

// I2CDIS
void ak8963DisableI2C()
{
	i2cdevWriteBit(I2Cx, devAddr, AK8963_RA_I2CDIS, AK8963_I2CDIS_BIT, true);
}

// ASA* 寄存器
void ak8963GetAdjustment(s8 *x, s8 *y, s8 *z)
{
	i2cdevRead(I2Cx, devAddr, AK8963_RA_ASAX, 3, buffer);
	*x = buffer[0];
	*y = buffer[1];
	*z = buffer[2];
}
void ak8963SetAdjustment(s8 x, s8 y, s8 z)
{
	buffer[0] = x;
	buffer[1] = y;
	buffer[2] = z;
	i2cdevWrite(I2Cx, devAddr, AK8963_RA_ASAX, 3, buffer);
}
u8 ak8963GetAdjustmentX()
{
	i2cdevReadByte(I2Cx, devAddr, AK8963_RA_ASAX, buffer);
	return buffer[0];
}
void ak8963SetAdjustmentX(u8 x)
{
	i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_ASAX, x);
}
u8 ak8963GetAdjustmentY()
{
	i2cdevReadByte(I2Cx, devAddr, AK8963_RA_ASAY, buffer);
	return buffer[0];
}
void ak8963SetAdjustmentY(u8 y)
{
	i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_ASAY, y);
}
u8 ak8963GetAdjustmentZ()
{
	i2cdevReadByte(I2Cx, devAddr, AK8963_RA_ASAZ, buffer);
	return buffer[0];
}
void ak8963SetAdjustmentZ(u8 z)
{
	i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_ASAZ, z);
}

