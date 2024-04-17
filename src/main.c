/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#include <stdio.h>
#include <math.h>
#include "sc7a20.h"
#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *dev_i2c = DEVICE_DT_GET(I2C_NODE);


void SC7A20_Test(void)
{
	u8 buff[7];
	s16 acc[3];
	float zAngle;
	
	while (1)
	{
		if (false == SC7A20_GetZAxisAngle(dev_i2c, acc, &zAngle))
		{
			printk("Failed to read acc data.\r\n");
		}
		else
		{
			printk("%f\r\n", zAngle);
		}
 
		k_sleep(K_MSEC(10));
	}
 
}

int main(void)
{
	  
	if (!device_is_ready(dev_i2c)) {
		printk("Error getting I2C device\n");
		return;
	}
	SC7A20_Init(dev_i2c);
	printk("SC7A20H example started.\n");
	s16 acc[3];
	float zangle;
	while (1) {
		SC7A20_Test();
		k_sleep(K_MSEC(200));
		bool ret;
		ret = SC7A20_GetZAxisAngle(dev_i2c, acc, &zangle);
	}
	return 0;
}
