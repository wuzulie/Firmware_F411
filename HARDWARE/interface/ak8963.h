#ifndef _AK8963_H_
#define _AK8963_H_
#include <sys.h>
#include "i2cdev.h"
#include <stdint.h>

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


#define AK8963_ADDRESS_00         0x0C
#define AK8963_ADDRESS_01         0x0D
#define AK8963_ADDRESS_10         0x0E // default for InvenSense MPU-6050 evaluation board
#define AK8963_ADDRESS_11         0x0F
#define AK8963_DEFAULT_ADDRESS    AK8963_ADDRESS_00

#define AK8963_RA_WIA             0x00
#define AK8963_RA_INFO            0x01
#define AK8963_RA_ST1             0x02
#define AK8963_RA_HXL             0x03
#define AK8963_RA_HXH             0x04
#define AK8963_RA_HYL             0x05
#define AK8963_RA_HYH             0x06
#define AK8963_RA_HZL             0x07
#define AK8963_RA_HZH             0x08
#define AK8963_RA_ST2             0x09
#define AK8963_RA_CNTL            0x0A
#define AK8963_RA_RSV             0x0B // RESERVED, DO NOT USE
#define AK8963_RA_ASTC            0x0C
#define AK8963_RA_TS1             0x0D // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_TS2             0x0E // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_I2CDIS          0x0F
#define AK8963_RA_ASAX            0x10
#define AK8963_RA_ASAY            0x11
#define AK8963_RA_ASAZ            0x12

#define AK8963_ST1_DRDY_BIT       0

#define AK8963_ST2_HOFL_BIT       3
#define AK8963_ST2_DERR_BIT       2

#define AK8963_CNTL_MODE_BIT      3
#define AK8963_CNTL_MODE_LENGTH   4

#define AK8963_MODE_POWERDOWN     0x00
#define AK8963_MODE_SINGLE        0x01
#define AK8963_MODE_CONT1         0x02
#define AK8963_MODE_CONT2         0x06
#define AK8963_MODE_EXTTRIG       0x04
#define AK8963_MODE_SELFTEST      0x08
#define AK8963_MODE_FUSEROM       0x0F
#define AK8963_MODE_14BIT         0x00
#define AK8963_MODE_16BIT         0x10

#define AK8963_ASTC_SELF_BIT      6

#define AK8963_I2CDIS			0x1B
#define AK8963_I2CDIS_BIT         0

#define AK8963_ST_X_MIN           (s16)(-200)
#define AK8963_ST_X_MAX           (s16)(200)
#define AK8963_ST_Y_MIN           (s16)(-200)
#define AK8963_ST_Y_MAX           (s16)(200)
#define AK8963_ST_Z_MIN           (s16)(-3200)
#define AK8963_ST_Z_MAX           (s16)(-800)

#define MAG_GAUSS_PER_LSB		(float)(666.7f)

void ak8963Init(I2C_Dev *i2cPort);
bool ak8963TestConnection(void);
bool ak8963SelfTest(void);	/*AK8963自检*/

uint8_t ak8963GetDeviceID(void);	// WIA 寄存器
uint8_t ak8963GetInfo(void);		// INFO 寄存器
uint8_t ak8963GetDataReady(void);	// ST1 寄存器

// H* registers
void ak8963GetHeading(s16 *x, s16 *y, s16 *z);
s16 ak8963GetHeadingX(void);
s16 ak8963GetHeadingY(void);
s16 ak8963GetHeadingZ(void);

bool ak8963GetOverflowStatus(void);// ST2 寄存器
bool ak8963GetDataError(void);

uint8_t ak8963GetMode(void);// CNTL 寄存器
void ak8963SetMode(u8 mode);
void ak8963Reset(void);

void ak8963SetSelfTest(bool enabled);// ASTC 寄存器

void ak8963DisableI2C(void); // I2CDIS

// ASA* registers
void ak8963GetAdjustment(s8 *x, s8 *y, s8 *z);
void ak8963SetAdjustment(s8 x, s8 y, s8 z);
uint8_t ak8963GetAdjustmentX(void);
void ak8963SetAdjustmentX(u8 x);
uint8_t ak8963GetAdjustmentY(void);
void ak8963SetAdjustmentY(u8 y);
uint8_t ak8963GetAdjustmentZ(void);
void ak8963SetAdjustmentZ(u8 z);

#endif /* _AK8963_H_ */
