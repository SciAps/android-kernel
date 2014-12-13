/*
 * LTC1760 (Charger)
 *
 * Based on bq27x00_battery.c:
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * This driver reads battery information from the ND2054 batteries through
 * the LTC1760 charger.  Although the LTC1760 maintains information about
 * the batteries which can be read over the I2C bus, its information is
 * insufficient for reporting purposes.
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <plat/mux.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION			"1.0.0"

#define LTC1760_REG_STATE		0x01 /* BatterySystemState */

/*
 * For simplicity, a single ltc1760 device is supported.  If support for
 * multiple LTC1760 devices was required, this would have to be an array of
 * devices or a structure allocated in the ltc1760_charger_probe() routine.
 *
 * Note that there is an association between LTC1760 charger devices and
 * devices corresponding to the batteries.  In order for the battery driver to
 * send an I2C message to the battery, it must call ltc1760_select_battery()
 * to connect the host I2C bus to the appropriate battery.  If support for
 * multiple LTC1760 devices is added, a way of mapping batteries to the
 * charger that they are connected to will be needed.
 */
struct ltc1760_device_info {
	struct device *dev;
	u16 state;
} ltc_device_info;

static DEFINE_MUTEX(battery_mutex);

#define SELECT_BATTERY1	0x1000
#define SELECT_BATTERY2 0x2000

/*
 * Read a 16 bit value from an I2C register in the LTC1760.
 */
static int ltc1760_read(struct ltc1760_device_info *di, u8 reg)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 2;		/* all registers are 16 bits wide */

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	ret = get_unaligned_le16(data);

	return ret;
}

/*
 * Write a 16 bit value into an I2C register in the LTC1760.
 */
static int ltc1760_write(struct ltc1760_device_info *di, u8 reg, u16 val)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[1];
	unsigned char data[3];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	data[0] = reg;
	data[1] = val & 0x00FF;
	data[2] = (val & 0xFF00) >> 8;
	msg[0].len = 3; /* 1 reg addr + 2 cmd bytes */
	msg[0].buf = data;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	return ret;
}

/*
 * Select one of the two batteries connected to the LTC1760.  This connects
 * the host I2C bus to either I2C B1 (battery 1 I2C bus) or I2C B2
 * (battery 2 I2C bus).  The host can then access the battery at the I2C
 * address defined in the Smart Battery specification.
 */
int ltc1760_select_battery(u8 battery_num)
{
	struct i2c_client *client = to_i2c_client(ltc_device_info.dev);
	u16 val;

	if (ltc_device_info.dev == NULL)	// not yet probed
		return -EINVAL;

	if (battery_num > 1)
		return -EINVAL;

	if (!client->adapter)
		return -ENODEV;

	val = battery_num ? SELECT_BATTERY2 : SELECT_BATTERY1;
	return ltc1760_write(&ltc_device_info, LTC1760_REG_STATE, val);
}
EXPORT_SYMBOL_GPL(ltc1760_select_battery);

static int ltc1760_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	if (ltc_device_info.dev != NULL)
		return -EBUSY;

	ltc_device_info.dev = &client->dev;

	/* See what batteries are present */
	ltc_device_info.state =
			ltc1760_read(&ltc_device_info, LTC1760_REG_STATE);

	return 0;
}

static int ltc1760_charger_remove(struct i2c_client *client)
{
	ltc_device_info.dev = NULL;
	ltc_device_info.state = 0;	// indicates no batteries installed

	return 0;
}

static const struct i2c_device_id ltc1760_id[] = {
	{ "ltc1760", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ltc1760_id);

static struct i2c_driver ltc1760_charger_driver = {
	.driver = {
		.name = "ltc1760-charger",
	},
	.probe = ltc1760_charger_probe,
	.remove = ltc1760_charger_remove,
	.id_table = ltc1760_id,
};

static inline int ltc1760_charger_i2c_init(void)
{
	int ret = i2c_add_driver(&ltc1760_charger_driver);
	if (ret)
		printk(KERN_ERR "Unable to register LTC1760 i2c driver\n");

	return ret;
}

static inline void ltc1760_charger_i2c_exit(void)
{
	i2c_del_driver(&ltc1760_charger_driver);
}

/*
 * Module stuff
 */

static int __init ltc1760_charger_init(void)
{
	int ret;

	ret = ltc1760_charger_i2c_init();

	return ret;
}
module_init(ltc1760_charger_init);

static void __exit ltc1760_charger_exit(void)
{
	ltc1760_charger_i2c_exit();
}
module_exit(ltc1760_charger_exit);

MODULE_AUTHOR("Steve Schfter <steve@scheftech.com>");
MODULE_DESCRIPTION("LTC1760 charger driver");
MODULE_LICENSE("GPL");
