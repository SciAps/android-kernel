/*
 * An I2C driver for the Philips PCF8563 RTC
 * Copyright 2005-06 Tower Technologies
 *
 * Author: Alessandro Zummo <a.zummo@towertech.it>
 * Maintainers: http://www.nslu2-linux.org/
 *
 * based on the other drivers in this same directory.
 *
 * http://www.semiconductors.philips.com/acrobat/datasheets/PCF8563-04.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>

#define DRV_VERSION "0.4.3"

#define PCF8563_REG_ST1		0x00 /* status */
#define PCF8563_REG_ST2		0x01

#define PCF8563_REG_SC		0x02 /* datetime */
#define PCF8563_REG_MN		0x03
#define PCF8563_REG_HR		0x04
#define PCF8563_REG_DM		0x05
#define PCF8563_REG_DW		0x06
#define PCF8563_REG_MO		0x07
#define PCF8563_REG_YR		0x08

#define PCF8563_REG_AMN		0x09 /* alarm */
#define PCF8563_REG_AHR		0x0A
#define PCF8563_REG_ADM		0x0B
#define PCF8563_REG_ADW		0x0C

#define PCF8563_REG_CLKO	0x0D /* clock out */
#define PCF8563_REG_TMRC	0x0E /* timer control */
#define PCF8563_REG_TMR		0x0F /* timer */

#define PCF8563_SC_LV		0x80 /* low voltage */
#define PCF8563_MO_C		0x80 /* century */

#define PCF8563_TIE_BIT		0x01 /* timer interrupt enable bit */
#define PCF8563_AIE_BIT		0x02 /* alarm interrupt enable bit */
#define PCF8563_TF_BIT		0x04 /* timer flag bit */
#define PCF8563_AF_BIT		0x08 /* alarm flag bit */

static struct i2c_driver pcf8563_driver;

struct pcf8563 {
	struct rtc_device *rtc;
	/*
	 * The meaning of MO_C bit varies by the chip type.
	 * From PCF8563 datasheet: this bit is toggled when the years
	 * register overflows from 99 to 00
	 *   0 indicates the century is 20xx
	 *   1 indicates the century is 19xx
	 * From RTC8564 datasheet: this bit indicates change of
	 * century. When the year digit data overflows from 99 to 00,
	 * this bit is set. By presetting it to 0 while still in the
	 * 20th century, it will be set in year 2000, ...
	 * There seems no reliable way to know how the system use this
	 * bit.  So let's do it heuristically, assuming we are live in
	 * 1970...2069.
	 */
	int c_polarity;	/* 0: MO_C=1 means 19xx, otherwise MO_C=1 means 20xx */
	int voltage_low; /* incicates if a low_voltage was detected */
};

/*
 * In the routines that deal directly with the pcf8563 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int pcf8563_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);
	unsigned char buf[13] = { PCF8563_REG_ST1 };

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, buf },	/* setup read ptr */
		{ client->addr, I2C_M_RD, 13, buf },	/* read status + date */
	};

	/* read registers */
	if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	if (buf[PCF8563_REG_SC] & PCF8563_SC_LV) {
		pcf8563->voltage_low = 1;
		dev_info(&client->dev,
			"low voltage detected, date/time is not reliable.\n");
	}

	dev_dbg(&client->dev,
		"%s: raw data is st1=%02x, st2=%02x, sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x\n",
		__func__,
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8]);


	tm->tm_sec = bcd2bin(buf[PCF8563_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(buf[PCF8563_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF8563_REG_HR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF8563_REG_DM] & 0x3F);
	tm->tm_wday = buf[PCF8563_REG_DW] & 0x07;
	tm->tm_mon = bcd2bin(buf[PCF8563_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[PCF8563_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */
	/* detect the polarity heuristically. see note above. */
	pcf8563->c_polarity = (buf[PCF8563_REG_MO] & PCF8563_MO_C) ?
		(tm->tm_year >= 100) : (tm->tm_year < 100);

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* the clock can give out invalid datetime, but we cannot return
	 * -EINVAL otherwise hwclock will refuse to set the time on bootup.
	 */
	if (rtc_valid_tm(tm) < 0)
		dev_err(&client->dev, "retrieved date/time is not valid.\n");

	return 0;
}

static int pcf8563_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);
	int i, err;
	unsigned char buf[9];

	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* hours, minutes and seconds */
	buf[PCF8563_REG_SC] = bin2bcd(tm->tm_sec);
	buf[PCF8563_REG_MN] = bin2bcd(tm->tm_min);
	buf[PCF8563_REG_HR] = bin2bcd(tm->tm_hour);

	buf[PCF8563_REG_DM] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[PCF8563_REG_MO] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	buf[PCF8563_REG_YR] = bin2bcd(tm->tm_year % 100);
	if (pcf8563->c_polarity ? (tm->tm_year >= 100) : (tm->tm_year < 100))
		buf[PCF8563_REG_MO] |= PCF8563_MO_C;

	buf[PCF8563_REG_DW] = tm->tm_wday & 0x07;

	/* write register's data */
	for (i = 0; i < 7; i++) {
		unsigned char data[2] = { PCF8563_REG_SC + i,
						buf[PCF8563_REG_SC + i] };

		err = i2c_master_send(client, data, sizeof(data));
		if (err != sizeof(data)) {
			dev_err(&client->dev,
				"%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, data[0], data[1]);
			return -EIO;
		}
	};

	return 0;
}

#ifdef CONFIG_RTC_INTF_DEV
static int pcf8563_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(to_i2c_client(dev));
	struct rtc_time tm;

	switch (cmd) {
	case RTC_VL_READ:
		if (pcf8563->voltage_low)
			dev_info(dev, "low voltage detected, date/time is not reliable.\n");

		if (copy_to_user((void __user *)arg, &pcf8563->voltage_low,
					sizeof(int)))
			return -EFAULT;
		return 0;
	case RTC_VL_CLR:
		/*
		 * Clear the VL bit in the seconds register in case
		 * the time has not been set already (which would
		 * have cleared it). This does not really matter
		 * because of the cached voltage_low value but do it
		 * anyway for consistency.
		 */
		if (pcf8563_get_datetime(to_i2c_client(dev), &tm))
			pcf8563_set_datetime(to_i2c_client(dev), &tm);

		/* Clear the cached value. */
		pcf8563->voltage_low = 0;

		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}
#else
#define pcf8563_rtc_ioctl NULL
#endif

static int pcf8563_alarm_irq_enable(struct i2c_client *client, unsigned int enabled)
{
	unsigned char buf = PCF8563_REG_ST2;
	unsigned char data[2];
	int val, err;

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, &buf },
		{ client->addr, I2C_M_RD, 1, &buf },
	};

	if ((i2c_transfer(client->adapter, msgs, 2)) !=2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	if (enabled)
		val = buf | PCF8563_AIE_BIT;
	else
		val = buf & ~(PCF8563_AIE_BIT);

	data[0] = PCF8563_REG_ST2;
	data[1] = val;

	err = i2c_master_send(client, data, sizeof(data));
	if (err != sizeof(data)) {
		dev_err(&client->dev,
			"%s: err=%d addr=%02x, data=%02x\n",
			__func__, err, data[0], data[1]);
		return -EIO;
	}

	return 0;
}

static int pcf8563_read_alarm(struct i2c_client *client, struct rtc_wkalrm *alm)
{
	unsigned char buf[13] = { PCF8563_REG_ST1 };

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, buf },	/* setup read ptr */
		{ client->addr, I2C_M_RD, 13, buf },	/* read status + date */
	};


	/* read registers 0x09->0x0C only */
	if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	dev_dbg(&client->dev,
		"%s: raw data is amin=%02x, ahr=%02x, aday=%02x, awkday=%02x\n",
		__func__, buf[9], buf[10], buf[11], buf[12]);

	// read all alarm addresses
	alm->time.tm_min = bcd2bin(buf[PCF8563_REG_AMN] & 0x7F);
	alm->time.tm_hour = bcd2bin(buf[PCF8563_REG_AHR] & 0x3F); /* rtc hr 0-23 */
	alm->time.tm_mday = bcd2bin(buf[PCF8563_REG_ADM] & 0x3F);
	alm->time.tm_wday = buf[PCF8563_REG_ADW] & 0x07;

	if (buf[PCF8563_REG_ST2] & (PCF8563_AIE_BIT | PCF8563_TIE_BIT))
		alm->enabled = 1;

	return 0;
}

static int pcf8563_set_alarm(struct i2c_client *client, struct rtc_wkalrm *alm)
{
	int i, err;
	int ret = 0;
	unsigned char buf[13];

	dev_dbg(&client->dev, "%s: amin=%d, ahr=%d, aday=%d, awkday=%d\n",
		__func__, alm->time.tm_min, alm->time.tm_hour,
		alm->time.tm_mday, alm->time.tm_wday);

	ret = pcf8563_alarm_irq_enable(client, 0);
	if (ret)
		goto out;

	/* hours, minutes and seconds */
	buf[PCF8563_REG_AMN] = bin2bcd(alm->time.tm_min);
	buf[PCF8563_REG_AHR] = bin2bcd(alm->time.tm_hour);
	buf[PCF8563_REG_ADM] = bin2bcd(alm->time.tm_mday);
	buf[PCF8563_REG_ADW] = alm->time.tm_wday & 0x07;

	/* write register's data */
	for (i = 7; i < 11; i++) {
		unsigned char data[2] = { PCF8563_REG_SC + i,
						buf[PCF8563_REG_SC + i] };

		err = i2c_master_send(client, data, sizeof(data));
		if (err != sizeof(data)) {
			dev_err(&client->dev,
				"%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, data[0], data[1]);
			return -EIO;
		}
	};

	if (alm->enabled)
		ret = pcf8563_alarm_irq_enable(client, 1);

out:
	return ret;
}

static int pcf8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return pcf8563_get_datetime(to_i2c_client(dev), tm);
}

static int pcf8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return pcf8563_set_datetime(to_i2c_client(dev), tm);
}

static int pcf8563_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	return pcf8563_read_alarm(to_i2c_client(dev), alm);
}

static int pcf8563_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	return pcf8563_set_alarm(to_i2c_client(dev), alm);
}

static int pcf8563_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	return pcf8563_alarm_irq_enable(to_i2c_client(dev), enabled);
}

static const struct rtc_class_ops pcf8563_rtc_ops = {
	.ioctl		= pcf8563_rtc_ioctl,
	.read_time	= pcf8563_rtc_read_time,
	.set_time	= pcf8563_rtc_set_time,
	.read_alarm	= pcf8563_rtc_read_alarm,
	.set_alarm	= pcf8563_rtc_set_alarm,
	.alarm_irq_enable = pcf8563_rtc_alarm_irq_enable,
};

static irqreturn_t pcf8563_rtc_interrupt(int irq, void *pcf8563_client)
{
	struct i2c_client *client = pcf8563_client;
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);

	unsigned long events = 0;
	int ret = IRQ_NONE;

	unsigned char buf = PCF8563_REG_ST2;
	unsigned char data[2];
	int err;

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, &buf },
		{ client->addr, I2C_M_RD, 1, &buf },
	};

	if ((i2c_transfer(client->adapter, msgs, 2)) !=2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		goto out;
	}

	if (buf & PCF8563_AF_BIT)
	{
		events = RTC_IRQF | RTC_AF;
	}
	else
	{
		events = RTC_IRQF | RTC_PF;
	}

	data[0] = PCF8563_REG_ST2;
	data[1] = buf & ~(PCF8563_AF_BIT | PCF8563_TF_BIT);

	err = i2c_master_send(client, data, sizeof(data));
	if (err != sizeof(data)) {
		dev_err(&client->dev,
			"%s: err=%d addr=%02x, data=%02x\n",
			__func__, err, data[0], data[1]);
		goto out;
        }

	rtc_update_irq(pcf8563->rtc, 1, events);

	ret = IRQ_HANDLED;

out:
	return ret;	
}

static int pcf8563_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pcf8563 *pcf8563;
	int err = 0;
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	pcf8563 = kzalloc(sizeof(struct pcf8563), GFP_KERNEL);
	if (!pcf8563)
		return -ENOMEM;

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	i2c_set_clientdata(client, pcf8563);

	/* Set device as wakeup capable */
	device_init_wakeup(&client->dev, 1);

	pcf8563->rtc = rtc_device_register(pcf8563_driver.driver.name,
				&client->dev, &pcf8563_rtc_ops, THIS_MODULE);

	if (IS_ERR(pcf8563->rtc)) {
		err = PTR_ERR(pcf8563->rtc);
		goto exit_kfree;
	}

        ret = request_threaded_irq(client->irq, NULL, pcf8563_rtc_interrupt,
                                   IRQF_TRIGGER_RISING,
                                   pcf8563_driver.driver.name, client);

        if (ret < 0) {
                dev_err(&client->dev, "IRQ is not free.\n");
                goto exit_unreg;
        }

	return 0;

exit_unreg:
	rtc_device_unregister(pcf8563->rtc);
exit_kfree:
	kfree(pcf8563);

	return err;
}

static int pcf8563_remove(struct i2c_client *client)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);

	if (pcf8563->rtc)
		rtc_device_unregister(pcf8563->rtc);

	kfree(pcf8563);

	return 0;
}

static const struct i2c_device_id pcf8563_id[] = {
	{ "pcf8563", 0 },
	{ "rtc8564", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcf8563_id);

static struct i2c_driver pcf8563_driver = {
	.driver		= {
		.name	= "rtc-pcf8563",
	},
	.probe		= pcf8563_probe,
	.remove		= pcf8563_remove,
	.id_table	= pcf8563_id,
};

module_i2c_driver(pcf8563_driver);

MODULE_AUTHOR("Alessandro Zummo <a.zummo@towertech.it>");
MODULE_DESCRIPTION("Philips PCF8563/Epson RTC8564 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
