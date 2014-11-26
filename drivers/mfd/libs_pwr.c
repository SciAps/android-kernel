#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/mfd/libs_pwr.h>
#include <linux/platform_data/android_battery.h>

static struct i2c_client *libs_client;

struct libs_pwr_data {
	struct	i2c_client	*client;

	/* battery profile values */
	unsigned int		batt_max_voltage;
	unsigned int		batt_low_voltage;
	unsigned int		batt_min_voltage;
	unsigned int		batt_prev_voltage;

	bool			nvram_unlocked;
};

typedef struct
{
	const char	*command;
	u8		buf[3];
} libs_cmds;

typedef struct
{
	const char	*name;
	u8		reg;
	const bool	writable;
	const bool	nvram;
} libs_regs;

static libs_cmds libs_known_cmds[] = {
	{ "reset_micro",	{0xA0, 0x7A, 0xA7} },
	{ "enter_bl",		{0xA1, 0x3C, 0xC3} },
	{ "unlock_nvram",	{0xA2, 0x55, 0xAA} },
};

static libs_regs libs_known_regs[] = {
	/* read-only constants*/
	{ "protocol", 		0x00,	0,	0 },
	{ "fw_code_lw",		0x01,	0,	0 },
	{ "fw_code_hw",		0x02,	0,	0 },
	{ "version_lw",		0x03,	0,	0 },
	{ "version_hw",		0x04,	0,	0 },

	/* read-only */
	{ "status",		0x40,	0,	0 },
	{ "vcap_sense",		0x41,	0,	0 },
	{ "vbat_sense",		0x42,	0,	0 },
	{ "3v3_sense",		0x43,	0,	0 },
	{ "5v0_sense",		0x44,	0,	0 },
	{ "6v0_sense",		0x45,	0,	0 },

	/* read-write */
	{ "x_location",		0x46,	1,	0 },
	{ "y_location",		0x47,	1,	0 },
	{ "z_location",		0x48,	1,	0 },
	{ "control",		0x80,	1,	0 },
	{ "move_steps",		0x81,	1,	0 },
	{ "current",		0x82,	1,	0 },
	{ "speed",		0x83,	1,	0 },

	/* read-write if nvram_unlocked, else read-only */
	{ "hw_rev",		0x84, 	1,	1 },
	{ "temp_cal",		0x85,	1,	1 },
	{ "laser_type",		0x86,	1,	1 },
	{ "serial_num_lw",	0x87,	1,	1 },
	{ "serial_num_hw",	0x88,	1,	1 },
	{ "stepper_range",	0x89,	1,	1 },
	{ "stepper_offset",	0x8A,	1,	1 },
};

static const int libs_cmd_num = sizeof(libs_known_cmds)/sizeof(libs_cmds);
static const int libs_reg_num = sizeof(libs_known_regs)/sizeof(libs_regs);

static int libs_lookup(bool cmd, char *buf)
{
	int i;

	if (cmd) {
		for (i = 0; i < libs_cmd_num; i++) {
			if (!strcmp(libs_known_cmds[i].command, buf)) {
				return i;
			}
		}
	} else {
		for (i = 0; i < libs_reg_num; i++) {
			if (!strcmp(libs_known_regs[i].name, buf)) {
				return i;
			}
		}
	}

	/* didn't find an index that matched */
	return -EINVAL;
}

static int libs_pwr_i2c_write(struct i2c_client *client,
			u8 *buf, int count)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= count,
			.buf	= buf,
		},
	};
	
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s write error %d\n", __func__, ret);
	return ret;
}

static int libs_pwr_i2c_read(struct i2c_client *client,
			u8 *reg, int count,
			u8 *buf)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= count,
			.buf	= buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&client->dev, "%s read error %d\n", __func__, ret);
	return ret;
}

static ssize_t libs_show_all_reg(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"Dumping all registers is currently unsupported.\n"
			"Write an individual register name to this sysfs\n"
			"node to output its contents\n");
}


static ssize_t libs_print_reg(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	char *blank;
	char *reg_name = NULL;
	u8 data[2];
	int index;
	int buf_size, err;

	blank = strchr(buf, ' ');
	if (NULL != blank) {
		dev_err(dev, "Too many parameters. Input register name only.");
		return -EINVAL;
	}

	buf_size = sizeof(char)*strlen(buf);
	reg_name = kzalloc(buf_size, GFP_KERNEL);
	if (NULL == reg_name) {
		dev_err(dev, "Could not allocate memory for reg_name\n");
		return -ENOMEM;
	}
	strncpy(reg_name, buf, buf_size - 1);

	index = libs_lookup(0, reg_name);
	
	if (index < 0) {
		dev_err(dev, "%s is an invalid register name\n", reg_name);
		return -EINVAL;
	} else {
		err = libs_pwr_i2c_read(libs_client,
				&(libs_known_regs[index].reg), 2,
				data);
		if (err < 0)
			return err;


		dev_info(dev, "%s value: 0x%.2x%.2x\n",
				reg_name, data[1], data[0]);
	}
	return count;
}

static ssize_t libs_store_reg(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);
	char *blank, end;
	char *reg_name = NULL, *temp1 = NULL, *temp2 = NULL;
	u8 data[3];
	int size, index, res, ret;

	/* extract the register name from the input */
	blank = strchr(buf, ' ');
	size = blank - buf;
	reg_name = kzalloc(sizeof(char)*(size + 1), GFP_KERNEL);
	if (NULL == reg_name) {
		dev_err(dev, "Could not allocate memory for reg_name\n");
		return -ENOMEM;
	}
	strncpy(reg_name, buf, size);

	/* store the data_low and data_high strings */
	temp1 = kzalloc(sizeof(char)*5, GFP_KERNEL);
	temp2 = kzalloc(sizeof(char)*5, GFP_KERNEL);
	if (NULL == temp1 || NULL == temp2) {
		dev_err(dev, "Could not allocate memory for data\n");
		return -ENOMEM;
	}
	res = sscanf(++blank, "%s %s%c", temp1, temp2, &end);
	if (res != 3) {
		dev_err(dev, "Invalid inputs\n"
			"Expected [reg_name] [data_low] [data_high]\n");
		return -EINVAL;
	}

	/* convert the data strings into u8 values */
	ret = kstrtou8(temp1, 0, &data[1]);
	if (ret < 0) {
		dev_err(dev, "[data_low] is not in the proper 0x## format\n");
		return -EINVAL;
	}

	ret = kstrtou8(temp2, 0, &data[2]);
	if (ret < 0) {
		dev_err(dev, "[data_low] is not in the proper 0x## format\n");
		return -EINVAL;
	}

	/* find register name in the lookup table */
	index = libs_lookup(0, reg_name);
	
	/* write selectively based on writable and nvram flags */
	if (index < 0) {
		dev_err(dev, "%s is an invalid register name\n", reg_name);
		return -EINVAL;
	} else if (!libs_known_regs[index].writable) {
		dev_err(dev, "%s is read-only\n", reg_name);
		return -EINVAL;
	} else if (libs_known_regs[index].nvram &&
			!libs_pwr->nvram_unlocked) {
		dev_err(dev, "%s requires the NVRAM to be unlocked "
				"before writing is possible\n", reg_name);
		return -EINVAL;
	} else {
		data[0] = libs_known_regs[index].reg;
		libs_pwr_i2c_write(libs_client, data, 3);
	
		return count;
	}
}

static ssize_t libs_trig_cmd(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);
	char *blank;
	char *cmd = NULL;
	int index;
	int buf_size, err;

	blank = strchr(buf, ' ');
	if (NULL != blank) {
		dev_err(dev, "Too many parameters. Input register name only.");
		return -EINVAL;
	}

	buf_size = sizeof(char)*strlen(buf);
	cmd = kzalloc(buf_size, GFP_KERNEL);
	if (NULL == cmd) {
		dev_err(dev, "Could not allocate memory for cmd\n");
		return -ENOMEM;
	}
	strncpy(cmd, buf, buf_size - 1);

	index = libs_lookup(1, cmd);

	if (index < 0) {
		dev_err(dev, "%s is an invalid command\n", cmd);
		return -EINVAL;
	} else {
		err = libs_pwr_i2c_write(libs_client,
				libs_known_cmds[index].buf, 3);
		if (err < 0)
			return err;

		if (!strcmp("unlock_nvram", cmd)) {
			libs_pwr->nvram_unlocked = 1;
			i2c_set_clientdata(libs_client, libs_pwr);
		}
	}
	return count;
}

static ssize_t libs_show_bvars(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);

	return scnprintf(buf, PAGE_SIZE,
		"BATT_MAX_VOLTAGE = 0x%.4x [%d]\n"
		"BATT_LOW_VOLTAGE = 0x%.4x [%d]\n"
		"BATT_MIN_VOLTAGE = 0x%.4x [%d]\n",
		libs_pwr->batt_max_voltage, libs_pwr->batt_max_voltage,
		libs_pwr->batt_low_voltage, libs_pwr->batt_low_voltage,
		libs_pwr->batt_min_voltage, libs_pwr->batt_min_voltage);
}

static ssize_t libs_show_bvar_max(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);

	return scnprintf(buf, PAGE_SIZE,
		"BATT_MAX_VOLTAGE = 0x%.4x [%d]\n",
		libs_pwr->batt_max_voltage, libs_pwr->batt_max_voltage);
}

static ssize_t libs_show_bvar_low(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);

	return scnprintf(buf, PAGE_SIZE,
		"BATT_LOW_VOLTAGE = 0x%.4x [%d]\n",
		libs_pwr->batt_low_voltage, libs_pwr->batt_low_voltage);
}

static ssize_t libs_show_bvar_min(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);

	return scnprintf(buf, PAGE_SIZE,
		"BATT_MIN_VOLTAGE = 0x%.4x [%d]\n",
		libs_pwr->batt_min_voltage, libs_pwr->batt_min_voltage);
}

static ssize_t libs_store_bvar_max(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);
	char * bvar_temp_val = NULL;
	char end;
	int res, ret, size;

	/* handle integer input */
	size = strlen(buf);
	bvar_temp_val = kzalloc(sizeof(char) * size, GFP_KERNEL);
	if (NULL == bvar_temp_val) {
		dev_err(dev, "Could not allocate memory for max variable\n");
		return -ENOMEM;
	}
	res = sscanf(buf, "%s%c", bvar_temp_val, &end);
	if (res != 2) {
			dev_err(dev, "Invalid input. "
			"Expected [max variable integer]\n");
	}

	/* convert the input string into uint */
	ret = kstrtouint(bvar_temp_val, 0, &(libs_pwr->batt_max_voltage));
	if (ret < 0) {
		dev_err(dev, "Input is not a number\n");
		return -EINVAL;
	}

	i2c_set_clientdata(libs_client, libs_pwr);

	return count;

}
static ssize_t libs_store_bvar_low(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);
	char * bvar_temp_val = NULL;
	char end;
	int res, ret, size;

	/* handle integer input */
	size = strlen(buf);
	bvar_temp_val = kzalloc(sizeof(char) * size, GFP_KERNEL);
	if (NULL == bvar_temp_val) {
		dev_err(dev, "Could not allocate memory for low variable\n");
		return -ENOMEM;
	}
	res = sscanf(buf, "%s%c", bvar_temp_val, &end);
	if (res != 2) {
			dev_err(dev, "Invalid input. "
			"Expected [low variable integer]\n");
	}

	/* convert the input string into uint */
	ret = kstrtouint(bvar_temp_val, 0, &(libs_pwr->batt_low_voltage));
	if (ret < 0) {
		dev_err(dev, "Input is not a number\n");
		return -EINVAL;
	}

	i2c_set_clientdata(libs_client, libs_pwr);

	return count;

}
static ssize_t libs_store_bvar_min(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);
	char * bvar_temp_val = NULL;
	char end;
	int res, ret, size;

	/* handle integer input */
	size = strlen(buf);
	bvar_temp_val = kzalloc(sizeof(char) * size, GFP_KERNEL);
	if (NULL == bvar_temp_val) {
		dev_err(dev, "Could not allocate memory for min variable\n");
		return -ENOMEM;
	}
	res = sscanf(buf, "%s%c", bvar_temp_val, &end);
	if (res != 2) {
			dev_err(dev, "Invalid input. "
			"Expected [min variable integer]\n");
	}

	/* convert the input string into uint */
	ret = kstrtouint(bvar_temp_val, 0, &(libs_pwr->batt_min_voltage));
	if (ret < 0) {
		dev_err(dev, "Input is not a number\n");
		return -EINVAL;
	}

	i2c_set_clientdata(libs_client, libs_pwr);

	return count;
}

/* begin external battery functions */
int libs_bat_get_voltage_now(void)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);
        u8 reg;
        u8 data[2];
	unsigned int batt_voltage_now;

        reg = libs_known_regs[libs_lookup(0, "vbat_sense")].reg;
        libs_pwr_i2c_read(libs_client, &reg, 2, data);
	batt_voltage_now = (data[1] << 8) + data[0];


	libs_pwr->batt_prev_voltage = batt_voltage_now;
	i2c_set_clientdata(libs_client, libs_pwr);

	return batt_voltage_now;
}
EXPORT_SYMBOL(libs_bat_get_voltage_now);

/* capacity constants */
#define BATT_MAX_VOLTAGE	0x3BC4 // 15300
#define BATT_LOW_VOLTAGE	0x32C8 // 13000
#define BATT_MIN_VOLTAGE	0x280A // 10250

#define PIECEWISE1_M		41 // 0.04*1024
#define PIECEWISE1_BSHIFT	10 // 2^10 = 1024
#define PIECEWISE1_SUB		512
#define PIECEWISE2_M		13 // 4096/312
#define PIECEWISE2_BSHIFT	12 // 2^12 = 4096
#define PIECEWISE2_SUB		33
int libs_bat_get_capacity(void)
{
	struct libs_pwr_data *libs_pwr = i2c_get_clientdata(libs_client);
	int batt_capacity = 0;

	if (libs_pwr->batt_prev_voltage > libs_pwr->batt_max_voltage) {
		batt_capacity = 100;
	} else if (libs_pwr->batt_prev_voltage > libs_pwr->batt_low_voltage) {
		batt_capacity = ((libs_pwr->batt_prev_voltage * PIECEWISE1_M)
			>> PIECEWISE1_BSHIFT) - PIECEWISE1_SUB;
	} else if (libs_pwr->batt_prev_voltage >= libs_pwr->batt_min_voltage) {
		batt_capacity = ((libs_pwr->batt_prev_voltage * PIECEWISE2_M)
			>> PIECEWISE2_BSHIFT) - PIECEWISE2_SUB;
	}

	return (int) batt_capacity;
}
EXPORT_SYMBOL(libs_bat_get_capacity);

int libs_bat_poll_charge_source(void)
{
	if (libs_bat_get_voltage_now() >= 0x4E20) {
		return CHARGE_SOURCE_AC;
	} else {
		return CHARGE_SOURCE_NONE;
	}
}
EXPORT_SYMBOL(libs_bat_poll_charge_source);
/* end external battery functions */

static DEVICE_ATTR(read_reg, 0666, libs_show_all_reg, libs_print_reg);
static DEVICE_ATTR(write_reg, 0222, NULL, libs_store_reg);
static DEVICE_ATTR(cmd, 0222, NULL, libs_trig_cmd);
//static DEVICE_ATTR(print_batt_vars, 0444, libs_show_bvars, NULL);
static DEVICE_ATTR(max_voltage, 0666, libs_show_bvar_max, libs_store_bvar_max);
static DEVICE_ATTR(low_voltage, 0666, libs_show_bvar_low, libs_store_bvar_low);
static DEVICE_ATTR(min_voltage, 0666, libs_show_bvar_min, libs_store_bvar_min);

static int libs_setup_sysfs(struct i2c_client *client)
{
	device_create_file(&client->dev, &dev_attr_read_reg);
	device_create_file(&client->dev, &dev_attr_write_reg);
	device_create_file(&client->dev, &dev_attr_cmd);
	device_create_file(&client->dev, &dev_attr_max_voltage);
	device_create_file(&client->dev, &dev_attr_low_voltage);
	device_create_file(&client->dev, &dev_attr_min_voltage);

	return 0;
}

static int libs_delete_sysfs(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_read_reg);
	device_remove_file(&client->dev, &dev_attr_write_reg);
	device_remove_file(&client->dev, &dev_attr_cmd);
	device_remove_file(&client->dev, &dev_attr_max_voltage);
	device_remove_file(&client->dev, &dev_attr_low_voltage);
	device_remove_file(&client->dev, &dev_attr_min_voltage);

	return 0;
}

static int libs_pwr_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct libs_pwr_data *libs_pwr;

	libs_pwr = kzalloc(sizeof(struct libs_pwr_data), GFP_KERNEL);
	if (!libs_pwr) {
		dev_err(&client->dev, "cannot allocate memory\n");
		return -ENOMEM;
	}

	libs_pwr->batt_max_voltage	= BATT_MAX_VOLTAGE;
	libs_pwr->batt_low_voltage	= BATT_LOW_VOLTAGE;
	libs_pwr->batt_min_voltage	= BATT_MIN_VOLTAGE;
	libs_pwr->batt_prev_voltage	= BATT_LOW_VOLTAGE;
	libs_pwr->nvram_unlocked = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c not supported\n");
		return -EPFNOSUPPORT;
	}

	i2c_set_clientdata(client, libs_pwr);
	libs_setup_sysfs(client);

	libs_client = client;
	dev_info(&client->dev, "device probed\n");
	
	return 0;
}
static int libs_pwr_remove(struct i2c_client *client)
{
	struct libs_pwr_data *libs_pwr;

	libs_pwr = i2c_get_clientdata(client);

	libs_delete_sysfs(client);
	i2c_unregister_device(client);

	kfree(libs_pwr);

	return 0;
}

static void libs_pwr_shutdown(struct i2c_client *client)
{
	int err;

	err = libs_pwr_i2c_write(client,
                                libs_known_cmds[0].buf, 3);
        if (err < 0)
		dev_err(&client->dev, "%s: could not shutdown PIC\n", __func__);
}

static struct i2c_device_id libs_pwr_idtable[] = {
	{ "libs_pwr", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, libs_pwr_idtable);

static struct i2c_driver libs_pwr_driver = {
	.driver = {
		.name   = "libs_pwr",
	},

	.id_table   = libs_pwr_idtable,
	.probe      = libs_pwr_probe,
	.remove     = libs_pwr_remove,
	.shutdown   = libs_pwr_shutdown,
#if 0
	.suspend    = libs_pwr_suspend,
	.resume     = libs_pwr_resume,
#endif
};
module_i2c_driver(libs_pwr_driver);

MODULE_AUTHOR("Russell Robinson <rrobinson@phytec.com>");
MODULE_DESCRIPTION("Sciaps LIBs Power Board Micro I2C client driver");
MODULE_LICENSE("GPL");
