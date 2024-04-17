#ifndef _SC7A20_H_
#define _SC7A20_H_
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#define u8 uint8_t
#define u16 uint16_t
#define s8 int8_t
#define s16 int16_t

#define SC7A20_I2C_WADDR 0x18
#define SC7A20_I2C_RADDR 0x18

#define SC7A20_REG_WHO_AM_I     0x0F
#define SC7A20_REG_CTRL_1		0x20
#define SC7A20_REG_CTRL_2		0x21
#define SC7A20_REG_CTRL_3		0x22
#define SC7A20_REG_CTRL_4		0x23
#define SC7A20_REG_X_L          0x28
#define SC7A20_REG_X_H          0x29
#define SC7A20_REG_Y_L          0x2A
#define SC7A20_REG_Y_H          0x2B
#define SC7A20_REG_Z_L          0x2C
#define SC7A20_REG_Z_H          0x2D
#define SC7A20_REG_STATUS		0x27

#define SC7A20_CHIP_ID          0x11

int SC7A20_check(const struct device *dev_i2c);

bool SC7A20_Init(const struct device *dev_i2c);

s16 SC7A20_12bitComplement(uint8_t msb, uint8_t lsb);

bool SC7A20_ReadAcceleration(const struct device *dev_i2c, s16* pXa, s16* pYa, s16* pZa);

bool SC7A20_GetZAxisAngle(const struct device *dev_i2c, s16 AcceBuff[3], float* pAngleZ);

#endif /*_SC7A20_H_*/