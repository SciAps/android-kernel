/*
 * ND2054 (Battery) driver
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
 *
 * This driver's involvement with the LTC1760 is limited to the use of
 * ltc1760_select_battery() to select which ND2054 battery is connected
 * to the host's I2C bus.
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
#include <linux/power/ltc1760_charger.h>

#define DRIVER_VERSION			"1.0.0"

#define ND2054_REG_TTF			0x12 /* Average time to full */
#define ND2054_REG_TTE			0x13 /* Average time to empty */
#define ND2054_REG_TEMP			0x08 /* Temperature */
#define ND2054_REG_VOLT			0x09 /* Voltage */
#define ND2054_REG_AI			0x0b /* Average current */
#define ND2054_REG_RSOC			0x0D /* Relative State-of-Charge */
#define ND2054_REG_REMAIN_CHARGE	0x0F /* reamaining charge capacity */
#define ND2054_REG_FULL_CHARGE		0x10 /* full charge capacity  */
#define ND2054_REG_STATUS		0x16 /* Battery Status */
#define ND2054_REG_CYCT			0x17 /* Cycle count total */
#define ND2054_REG_DCAP			0x18 /* Design capacity */

/* bits in ND2054_REG_STATUS */
#define ND2054_STATUS_INIT		0x08
#define ND2054_STATUS_DISCHARGING	0x04
#define ND2054_STATUS_FULL_CHRG		0x02
#define ND2054_STATUS_FULL_DISCHRG	0x01

struct nd2054_device_info;

struct nd2054_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_full;
	int charge_full;
	int charge_design_full;
	int cycle_count;
	int capacity;
	int status;
};

struct nd2054_device_info {
	struct device 		*dev;

	unsigned long last_update;
	struct delayed_work work;

	bool battery_present[2];
	struct power_supply	bat[2];
	struct nd2054_reg_cache cache[2];

	struct	mutex lock;
	/* reference to the cooling device */
	struct	thermal_dev *tdev;
};

static enum power_supply_property nd2054_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static unsigned int poll_interval = 1;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

static DEFINE_MUTEX(battery_mutex);

/*
 * nd2054_read().  Read the indicated register from the battery over the
 * I2C bus.  It is assumed that the appropriate battery as been selected
 * in the LTC charger configuration.
 */
static int nd2054_read(struct nd2054_device_info *di, u8 reg)
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

#if 0
The host does not currently write to any battery registers.
/*
 * nd2054_write().  Write to the indicated register from the battery over the
 * I2C bus.  It is assumed that the appropriate battery as been selected in
 * the LTC charger configuration.
 */
static int nd2054_write(struct nd2054_device_info *di, u8 reg, u16 val)
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
#endif

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int nd2054_battery_read_rsoc(struct nd2054_device_info *di)
{
	int rsoc;

	rsoc = nd2054_read(di, ND2054_REG_RSOC);

	if (rsoc < 0)
		dev_dbg(di->dev, "error reading relative State-of-Charge\n");

	return rsoc;
}

/*
 * Return a battery charge value in uAh
 * Or < 0 if something fails.
 */
static int nd2054_battery_read_charge(struct nd2054_device_info *di, u8 reg)
{
	int charge;

	charge = nd2054_read(di, reg);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	charge *= 1000;

	return charge;
}

/*
 * Return the battery remaining capaciy in uAh
 * Or < 0 if something fails.
 */
static inline int nd2054_battery_remaining_charge(struct nd2054_device_info *di)
{
	return nd2054_battery_read_charge(di, ND2054_REG_REMAIN_CHARGE);
}

/*
 * Return the battery full charge capacity in uAh
 * Or < 0 if something fails.
 */
static inline int nd2054_battery_full_charge(struct nd2054_device_info *di)
{
	return nd2054_battery_read_charge(di, ND2054_REG_FULL_CHARGE);
}

/*
 * Return the battery design capacity in uAh
 * Or < 0 if something fails.
 */
static int nd2054_battery_design_cap(struct nd2054_device_info *di)
{
	int dcap;

	dcap = nd2054_read(di, ND2054_REG_DCAP);

	if (dcap < 0) {
		dev_dbg(di->dev, "error reading design capacity\n");
		return dcap;
	}

	dcap *= 1000;
	return dcap;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int nd2054_battery_read_temperature(struct nd2054_device_info *di)
{
	int temp;

	temp = nd2054_read(di, ND2054_REG_TEMP);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	temp -= 2731;	// convert from .1 degrees K

	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int nd2054_battery_read_cyct(struct nd2054_device_info *di)
{
	int cyct;

	cyct = nd2054_read(di, ND2054_REG_CYCT);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int nd2054_battery_read_time(struct nd2054_device_info *di, u8 reg)
{
	int tval;

	tval = nd2054_read(di, reg);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Update the battery information held by the driver.
 */
static void nd2054_update(struct nd2054_device_info *di, int battery_num)
{
	struct nd2054_reg_cache cache = {0, };
	int err;

	mutex_lock(&di->lock);
	if (ltc1760_select_battery(battery_num) < 0) {
		mutex_unlock(&di->lock);
		return;
	}

	cache.status = nd2054_read(di, ND2054_REG_STATUS);
	if (cache.status >= 0) {
		/* If the battery just became present, register it. */
		if (di->battery_present[battery_num] == 0) {
			if ((err = power_supply_register(di->dev,
						&(di->bat[battery_num]))))
			dev_err(di->dev, "fail to register battery %d: %d\n",
						err, battery_num);
		}

		di->battery_present[battery_num] = 1;
		if (!(cache.status & ND2054_STATUS_INIT)) {
			dev_info(di->dev, "battery is not initilized! "
					"Ignoring capacity values\n");
			cache.capacity = -ENODATA;
			cache.time_to_empty = -ENODATA;
			cache.time_to_full = -ENODATA;
			cache.charge_full = -ENODATA;
		} else {
			cache.capacity = nd2054_battery_read_rsoc(di);
			cache.time_to_empty = nd2054_battery_read_time(di,
							ND2054_REG_TTE);
			cache.time_to_full = nd2054_battery_read_time(di,
							ND2054_REG_TTF);
			cache.charge_full = nd2054_battery_full_charge(di);
		}
		cache.temperature = nd2054_battery_read_temperature(di);
		cache.cycle_count = nd2054_battery_read_cyct(di);

		/* We only have to read charge design full once */
		if (di->cache[battery_num].charge_design_full <= 0) {
			di->cache[battery_num].charge_design_full =
						nd2054_battery_design_cap(di);
		}
	} else {
		if (di->battery_present[battery_num] == 1) {
			power_supply_unregister(&di->bat[battery_num]);
			di->battery_present[battery_num] = 0;
		}
	}
	mutex_unlock(&di->lock);

	if (di->battery_present[battery_num]) {
		if (memcmp(&(di->cache[battery_num]),
				&cache, sizeof(cache)) != 0) {
			di->cache[battery_num] = cache;
			power_supply_changed(&di->bat[battery_num]);
		}
	}

	di->last_update = jiffies;
}

static void nd2054_battery_poll(struct work_struct *work)
{
	struct nd2054_device_info *di =
		container_of(work, struct nd2054_device_info, work.work);

	nd2054_update(di, 0);	/* first battery */
	nd2054_update(di, 1);	/* second battery */

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}

/*
 * Return the battery average current in uA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int nd2054_battery_current(struct nd2054_device_info *di,
	union power_supply_propval *val)
{
	int curr;

	curr = nd2054_read(di, ND2054_REG_AI);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	/* the battery returns a signed value */
	val->intval = (int)((s16)curr) * 1000;

	return 0;
}

static int nd2054_battery_status(struct nd2054_device_info *di,
	int battery_num, union power_supply_propval *val)
{
	int status;

	if (di->cache[battery_num].status & ND2054_STATUS_FULL_CHRG)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (di->cache[battery_num].status & ND2054_STATUS_DISCHARGING)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;

	val->intval = status;

	return 0;
}

static int nd2054_battery_capacity_level(struct nd2054_device_info *di,
	int battery_num, union power_supply_propval *val)
{
	int level;

	if (di->cache[battery_num].status & ND2054_STATUS_FULL_CHRG)
		level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (di->cache[battery_num].status & ND2054_STATUS_FULL_CHRG)
		level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	val->intval = level;

	return 0;
}

/*
 * Return the battery Voltage in uVolts
 * Or < 0 if something fails.
 */
static int nd2054_battery_voltage(struct nd2054_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = nd2054_read(di, ND2054_REG_VOLT);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return volt;
	}

	val->intval = volt * 1000;

	return 0;
}

static int nd2054_simple_value(int value, union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

static int nd2054_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct nd2054_device_info *di = dev_get_drvdata(psy->dev->parent);
	int ret = 0;
	int bat_num;

	if (psy == &(di->bat[0]))
		bat_num = 0;
	else if (psy == &(di->bat[1]))
		bat_num = 1;
	else
		return -EINVAL;

	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		nd2054_battery_poll(&di->work.work);
	}

	if ((psp != POWER_SUPPLY_PROP_PRESENT) &&
	    (di->cache[bat_num].status < 0))
		return -ENODEV;

	mutex_lock(&di->lock);
	if (ltc1760_select_battery(bat_num) < 0) {
		mutex_unlock(&di->lock);
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = nd2054_battery_status(di, bat_num, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nd2054_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->battery_present[bat_num];
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = nd2054_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = nd2054_simple_value(di->cache[bat_num].capacity, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = nd2054_battery_capacity_level(di, bat_num, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = nd2054_simple_value(di->cache[bat_num].temperature, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = nd2054_simple_value(di->cache[bat_num].time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = nd2054_simple_value(di->cache[bat_num].time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = nd2054_simple_value(nd2054_battery_remaining_charge(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = nd2054_simple_value(di->cache[bat_num].charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = nd2054_simple_value(di->cache[bat_num].charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = nd2054_simple_value(di->cache[bat_num].cycle_count, val);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&di->lock);

	return ret;
}

static void nd2054_external_power_changed(struct power_supply *psy)
{
	struct nd2054_device_info *di = dev_get_drvdata(psy->dev->parent);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static int nd2054_add_battery(struct power_supply *battery,
				struct i2c_client *client, int battery_num)
{
	char *name;

	name = kasprintf(GFP_KERNEL, "ND2054-%d", battery_num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate battery name\n");
		return -ENOMEM;
	}

	battery->name = name;
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = nd2054_battery_props;
	battery->num_properties = ARRAY_SIZE(nd2054_battery_props);
	battery->get_property = nd2054_battery_get_property;
	battery->external_power_changed = nd2054_external_power_changed;
	return 0;
}

static void nd2054_remove_battery(struct power_supply *battery)
{
        kfree(battery->name);
        return;
}

//-------------------------------------------------------------------------

/*
 * Since this driver's access to a battery involves selecting the battery in
 * a LTC1760 register, we can not yet access the batteries here -- the
 * LTC1760 might not yet be probed.  Rather, we wait until the periodic poll
 * or an application reading a property.
 */
static int nd2054_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct nd2054_device_info *di;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&di->work, nd2054_battery_poll);
	schedule_delayed_work(&di->work, HZ);

	mutex_init(&di->lock);
	di->dev = &client->dev;
	i2c_set_clientdata(client, di);

	nd2054_add_battery(&(di->bat[0]), client, 0);
	nd2054_add_battery(&(di->bat[1]), client, 1);

	return 0;
}

static int nd2054_battery_remove(struct i2c_client *client)
{
	struct nd2054_device_info *di = i2c_get_clientdata(client);

	/*
	 * power_supply_unregister calls nd2054_battery_get_property which
	 * calls nd2054_battery_poll.
	 * Make sure that nd2054_battery_poll will not call
	 * schedule_delayed_work again after unregister (which causes OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);

	if (di->battery_present[0] == 1) {
		power_supply_unregister(&di->bat[0]);
		di->battery_present[0] = 0;
	}
	nd2054_remove_battery(&di->bat[0]);

	if (di->battery_present[1] == 1) {
		power_supply_unregister(&di->bat[1]);
		di->battery_present[1] = 0;
	}
	nd2054_remove_battery(&di->bat[1]);

	mutex_destroy(&di->lock);
	kfree(di);

	return 0;
}

static const struct i2c_device_id nd2054_id[] = {
	{ "nd2054", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, nd2054_id);

static struct i2c_driver nd2054_battery_driver = {
	.driver = {
		.name = "nd2054-battery",
	},
	.probe = nd2054_battery_probe,
	.remove = nd2054_battery_remove,
	.id_table = nd2054_id,
};

static inline int nd2054_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&nd2054_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register ND2054 i2c driver\n");

	return ret;
}

static inline void nd2054_battery_i2c_exit(void)
{
	i2c_del_driver(&nd2054_battery_driver);
}

static int __init nd2054_battery_init(void)
{
	int ret;

	ret = nd2054_battery_i2c_init();

	return ret;
}
module_init(nd2054_battery_init);

static void __exit nd2054_battery_exit(void)
{
	nd2054_battery_i2c_exit();
}
module_exit(nd2054_battery_exit);

MODULE_AUTHOR("Steve Schfter <steve@scheftech.com>");
MODULE_DESCRIPTION("ND2054 battery driver");
MODULE_LICENSE("GPL");
