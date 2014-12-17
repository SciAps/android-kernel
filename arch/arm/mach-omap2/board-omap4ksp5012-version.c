/*
 * FPGA Version Check initialization for Phytec KSP-5012 Board.
 *
 * Copyright (C) 2014 PHYTEC America, LLC.
 *
 * Author: Russell Robinson <rrobinson@phytec.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#include <plat/board.h>
#include <plat/common.h>

#include "common.h"

#ifdef CONFIG_BATTERY_ANDROID
#include <linux/platform_data/android_battery.h>
#if (defined(CONFIG_MFD_LIBS_PWR) || defined(CONFIG_MFD_LIBS_PWR_MODULE)) && \
	!defined(CONFIG_BATTERY_LIBS)
#include <linux/mfd/libs_pwr.h>
#endif
#ifdef CONFIG_BATTERY_LIBS
#include <linux/power/libs_battery.h>
#endif
#endif

#define KSP5012_RTC_IRQ                 117

int bus_id = 1;
module_param(bus_id, int, 0);

#ifdef CONFIG_BATTERY_ANDROID
/* TODO: ADC implementation for KSP-5012 battery functions.
 *       Use charger/smart battery implementation when available */
static void ksp5012_bat_charge_enable(int enable)
{
	return;
}

static struct android_bat_platform_data ksp5012_bat_pdata = {
	.set_charging_enable = ksp5012_bat_charge_enable,
	.poll_charge_source = libs_bat_poll_charge_source,
	.get_capacity = libs_bat_get_capacity,
	.get_voltage_now = libs_bat_get_voltage_now,
	.temp_high_threshold = 50, // dummy value
	.temp_high_recovery = 75, // dummy value
	.temp_low_recovery = 10, // dummy value
	.temp_low_threshold = 20, // dummy value
	.full_charging_time = 600, // dummy value
	.recharging_time = 300, // dummy value
	.recharging_voltage = 24000, // dummy value
};

static struct platform_device ksp5012_android_bat_device = {
	.name = "android-battery",
	.id = -1,
	.dev = {
		.platform_data = &ksp5012_bat_pdata,
	},
};

#ifdef CONFIG_BATTERY_LIBS
static struct libs_bat_platform_data ksp5012_libs_bat_pdata = {
	.voltage = 16000,
	.capacity = 100,
};

static struct platform_device ksp5012_libs_bat_device = {
	.name = "libs-battery",
	.id = -1,
	.dev = {
		.platform_data = &ksp5012_libs_bat_pdata,
	},
};
#endif
#endif


static struct platform_device *battery_devices[] __initdata = {
#ifdef CONFIG_BATTERY_ANDROID
#ifdef CONFIG_BATTERY_LIBS
	&ksp5012_libs_bat_device,
#endif
	&ksp5012_android_bat_device,
#endif
};

struct i2c_client *rtc_client;
struct i2c_client *libs_pwr_client;

static struct i2c_board_info __initdata ksp5012_libs_pwr_i2c_info = {
		I2C_BOARD_INFO("libs_pwr", 0x59),
};

static struct i2c_board_info __initdata ksp5012_rtc_i2c_info = {
		I2C_BOARD_INFO("rtc8564", 0x51),
		.irq = OMAP_GPIO_IRQ(KSP5012_RTC_IRQ),
};

static int __init ksp5012_version_init(void)
{
	struct i2c_adapter *i2c_adap;

	i2c_adap = i2c_get_adapter(bus_id);
	rtc_client = i2c_new_device(i2c_adap, &ksp5012_rtc_i2c_info);
#if defined(CONFIG_MFD_LIBS_PWR) || defined(CONFIG_MFD_LIBS_PWR_MODULE)
	libs_pwr_client = i2c_new_device(i2c_adap, &ksp5012_libs_pwr_i2c_info);
#endif

#ifdef CONFIG_BATTERY_ANDROID
	platform_add_devices(battery_devices, ARRAY_SIZE(battery_devices));
#endif

	return 0;
}

static void __init ksp5012_version_exit(void)
{
	printk("%s: Goodbye!\n", __func__);
}


module_init(ksp5012_version_init);
module_exit(ksp5012_version_exit);

MODULE_AUTHOR("Russell Robinson <rrobinson@phytec.com>");
MODULE_DESCRIPTION("KSP5012 Version-specific Initialization");
MODULE_LICENSE("GPL");

