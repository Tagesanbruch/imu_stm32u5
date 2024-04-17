#include "sc7a20.h"
#include "math.h"
#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define PI 3.1415926535898
int SC7A20_check(const struct device *dev_i2c)
{
    uint8_t partid[1] = {0x0};
    int ret = i2c_burst_read(dev_i2c, 0x18, SC7A20_REG_WHO_AM_I, partid, 1);    // Read part ID 
	
    if(ret) printk("SC7A20 not present!\n");
    else
      {
      if (partid[0] == SC7A20_CHIP_ID) printk("SC7A20: present! Part ID: %dd\n", partid[0]);
      else
        { 
        printk("SC7A20: Error! Part ID: %dd\n", partid[0]);
        ret = 0;
        }
      }
    return ret;
}
 
bool SC7A20_Init(const struct device *dev_i2c)
{ 
	if(SC7A20_check(dev_i2c) != 0){
		return false;
	}
	u8 tempreg = 0;
	tempreg = 0x47;
	i2c_reg_write_byte(dev_i2c, SC7A20_I2C_WADDR, SC7A20_REG_CTRL_1, tempreg);//50Hz+正常模式xyz使能
	tempreg = 0x00;
	i2c_reg_write_byte(dev_i2c, SC7A20_I2C_WADDR, SC7A20_REG_CTRL_2, tempreg);//关闭滤波器，手册上面没有滤波器截止频率设置说明，开启后无法测量静止状态下的重力加速度
	tempreg = 0x00;
	i2c_reg_write_byte(dev_i2c, SC7A20_I2C_WADDR, SC7A20_REG_CTRL_3, tempreg); //关闭中断
	tempreg = 0x88;
	i2c_reg_write_byte(dev_i2c, SC7A20_I2C_WADDR, SC7A20_REG_CTRL_4, tempreg); //读取完成再更新，小端模式，、2g+正常模式，高精度模式
	return true;
}
 
s16 SC7A20_12bitComplement(uint8_t msb, uint8_t lsb)
{
	s16 temp;
 
	temp = msb << 8 | lsb;
	temp = temp >> 4;   //只有高12位有效
	temp = temp & 0x0fff;
	if (temp & 0x0800) //负数 补码==>原码
	{
		temp = temp & 0x07ff; //屏弊最高位      
		temp = ~temp;
		temp = temp + 1;
		temp = temp & 0x07ff;
		temp = -temp;       //还原最高位
	}
	return temp;
}

bool SC7A20_ReadAcceleration(const struct device *dev_i2c, s16* pXa, s16* pYa, s16* pZa)
{
	u8 buff[6];
	u8 i;
	s16 temp;
 
	memset(buff, 0, 6);
	int ret = i2c_burst_read(dev_i2c, 0x18, SC7A20_REG_X_L, &buff[0], 1);
	ret = i2c_burst_read(dev_i2c, 0x18, SC7A20_REG_X_H, &buff[1], 1);
	ret = i2c_burst_read(dev_i2c, 0x18, SC7A20_REG_Y_L, &buff[2], 1);
	ret = i2c_burst_read(dev_i2c, 0x18, SC7A20_REG_Y_H, &buff[3], 1);
	ret = i2c_burst_read(dev_i2c, 0x18, SC7A20_REG_Z_L, &buff[4], 1);
	ret = i2c_burst_read(dev_i2c, 0x18, SC7A20_REG_Z_H, &buff[5], 1);
	*pXa = SC7A20_12bitComplement(buff[1], buff[0]);
	*pYa = SC7A20_12bitComplement(buff[3], buff[4]);
	*pZa = SC7A20_12bitComplement(buff[5], buff[2]);
	// printk("b0=%x, b1=%x, b2=%x\n", buff[0], buff[1], buff[2]);
	// printk("b3=%x, b4=%x, b5=%x\n", buff[3], buff[4], buff[5]);
	return true;
}

bool SC7A20_GetZAxisAngle(const struct device *dev_i2c, s16 AcceBuff[3], float* pAngleZ)
{
	double fx, fy, fz;
	double A;
	s16 Xa, Ya, Za;
 
	if (SC7A20_ReadAcceleration(dev_i2c, &Xa, &Ya, &Za) == false) return false;
	AcceBuff[0] = Xa;	
	AcceBuff[1] = Ya;	
	AcceBuff[2] = Za;	
 
	fx = Xa;
	fx *= 2.0 / 4096;
	fy = Ya;
	fy *= 2.0 / 4096;
	fz = Za;
	fz *= 2.0 / 4096;
 
	// AngleZ Calculation
	A = fx * fx + fy * fy;
	A = sqrt(A);
	A = (double)A / fz;
	A = atan(A);
	A = A * 180 / PI;
 
	*pAngleZ = A; 
	return true;
}

