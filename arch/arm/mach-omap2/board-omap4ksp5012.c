/*
 * Board support file for Phytec KSP-5012 Board.
 *
 * Copyright (C) 2013 Phytec Messtechnik GmbH
 *
 * Author: Russell Robinson <rrobinson@phytec.com>
 *
 * Based on mach-omap2/board-omap4pcm049.c
 *
 * Author: Jan Weitzel <armlinux@phytec.de>
 *         Steve Schefter <steve@scheftech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/omapfb.h>
#include <linux/usb/otg.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/tmp102.h>
#include <linux/mmc/card.h>
#ifdef CONFIG_TOUCHSCREEN_FT5X06
#include <linux/input/ft5x06_ts.h>
#endif
#include <linux/reboot.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
#include <linux/mtd/nand.h>
#include <linux/smsc911x.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/leds-pca9532.h>
#include <linux/platform_data/omap-abe-wm8974.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/omap_die_governor.h>

#include <linux/pwm_backlight.h>
#include <linux/input/adxl34x.h>

#include <mach/hardware.h>
#include <mach/omap-secure.h>

#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <video/omapdss.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/gpmc.h>
#include <plat/gpmc-smsc911x.h>
#include <plat/nand.h>
#include <plat/mmc.h>
#include <plat/pwm.h>
#include <plat/remoteproc.h>
#include <plat/vram.h>
#include <plat/clock.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/drm.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif
#include <plat/android-display.h>
#ifdef CONFIG_PANEL_GENERIC_DPI
#include <video/omap-panel-generic-dpi.h>
#endif

/* for TI Shared Transport devices */
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wakelock.h>

#include "common.h"
#include "omap4_ion.h"
#include "omap_ram_console.h"
#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "pm.h"
#include "common-board-devices.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"

#define KSP5012_ETH_GPIO_IRQ		121
#define KSP5012_ETH_CS			5
#define KSP5012_FT5x06_GPIO_IRQ		107
#define KSP5012_FT5x06_GPIO_WAKE	108
#define KSP5012_LCD_ENABLE		171
#define KSP5012_CAM_PWDN		111
#define KSP5012_RTC_IRQ			117
#define KSP5012_ADXL34X_IRQ1		175
#define KSP5012_ADXL34X_IRQ2		176
#define KSP5012_ADXL34X_IRQ		KSP5012_ADXL34X_IRQ1
#define KSP5012_USBB1_PWR		101
#define TPS62361_GPIO			182	/* VCORE1 power control */
#define GPIO_WL_EN			106
#define GPIO_BT_EN			173
#define GPIO_POWER_BUTTON		139
#define GPIO_FPGA_RESET		140

#define EMIF_SDRAM_CONFIG		0x0008
#define EBANK_SHIFT			3
#define EBANK_MASK			(1 << 3)

static struct gpio_keys_button ksp5012_gpio_keys[] = {
	{
		.desc			= "power_button",
		.type 			= EV_KEY,
		.code			= KEY_POWER,
		.gpio			= GPIO_POWER_BUTTON,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5,
	},
};

static struct gpio_keys_platform_data ksp5012_gpio_keys_data = {
	.buttons	= ksp5012_gpio_keys,
	.nbuttons	= ARRAY_SIZE(ksp5012_gpio_keys),
};

static struct platform_device ksp5012_gpio_keys_device = {
	.name	= "gpio-keys",
	.id = -1,
	.dev	= {
		.platform_data	= &ksp5012_gpio_keys_data,
	},
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "modul:red:status1",
		.default_trigger	= "heartbeat",
		.gpio			= 152,
	},
	{
		.name			= "modul:green:status2",
		.default_trigger	= "mmc0",
		.gpio			= 153,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct omap2_pwm_platform_config pwm_config = {
	.timer_id	= 9, // GPT9_PWM_EVT
	.polarity	= 1, // Active-high
};

static struct platform_device pwm_device = {
	.name	= "omap-pwm",
	.id	= 0,
	.dev	= {
		.platform_data = &pwm_config
	},
};

static struct platform_pwm_backlight_data ksp5012_backlight_data = {
	.pwm_id         = 0,
	.max_brightness = 255,
	.dft_brightness = 127,
	.pwm_period_ns  = 1000,
};

static struct platform_device ksp5012_backlight_device = {
	.name   = "pwm-backlight",
	.id	= -1,
	.dev    = {
		.parent = &pwm_device.dev,
		.platform_data  = &ksp5012_backlight_data,
	},
};

#ifdef CONFIG_OMAP4_DUTY_CYCLE_GOVERNOR
static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= DUTY_GOVERNOR_DEFAULT_TEMP,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 24,
		},
	},
};

static void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
				   ARRAY_SIZE
				   (omap4_duty_governor_pcb_sections));
}
#else
static void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/

/* Initial set of thresholds for different thermal zones */
static struct omap_thermal_zone thermal_zones[] = {
	OMAP_THERMAL_ZONE("safe", 0, 25000, 65000, 250, 1000, 400),
	OMAP_THERMAL_ZONE("monitor", 0, 60000, 80000, 250, 250, 250),
	OMAP_THERMAL_ZONE("alert", 0, 75000, 90000, 250, 250, 150),
	OMAP_THERMAL_ZONE("critical", 1, 85000, 115000, 250, 250, 50),
};

static struct omap_die_governor_pdata omap_gov_pdata = {
	.zones = thermal_zones,
	.zones_num = ARRAY_SIZE(thermal_zones),
};


static void __init pcm049_audio_mux_init(void)
{
	/* abe_mcbsp3_fsx */
	omap_mux_init_signal("abe_pdm_lb_clk", OMAP_MUX_MODE1 |
			OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	/* abe_mcbsp3_clkx */
	omap_mux_init_signal("abe_pdm_frame", OMAP_MUX_MODE1 |
			OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	/* abe_mcbsp3_dx */
	omap_mux_init_signal("abe_pdm_dl_data", OMAP_MUX_MODE1 |
			OMAP_OFF_EN);
	/* abe_mcbsp3_dr */
	omap_mux_init_signal("abe_pdm_ul_data", OMAP_MUX_MODE1 |
			OMAP_PIN_INPUT | OMAP_OFF_EN);
}

static struct regulator_consumer_supply pcm049_vcc_3v3_consumer_supply[] = {
	REGULATOR_SUPPLY("vdd33a", "smsc911x.0"),
	/* max1027 */
	REGULATOR_SUPPLY("vcc", "4-0064"),
};

struct regulator_init_data pcm049_vcc_3v3_initdata = {
	.consumer_supplies = pcm049_vcc_3v3_consumer_supply,
	.num_consumer_supplies = ARRAY_SIZE(pcm049_vcc_3v3_consumer_supply),
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config pcm049_vcc_3v3_config = {
	.supply_name		= "pcm049_vcc_3v3",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &pcm049_vcc_3v3_initdata,
};

static struct platform_device pcm049_vcc_3v3_device = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &pcm049_vcc_3v3_config,
	},
};

static struct regulator_consumer_supply pcm049_vcc_1v8_consumer_supply[] = {
	REGULATOR_SUPPLY("vddvario", "smsc911x.0"),
};

struct regulator_init_data pcm049_vcc_1v8_initdata = {
	.consumer_supplies = pcm049_vcc_1v8_consumer_supply,
	.num_consumer_supplies = ARRAY_SIZE(pcm049_vcc_1v8_consumer_supply),
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config pcm049_vcc_1v8_config = {
	.supply_name		= "pcm049_vcc_1v8",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &pcm049_vcc_1v8_initdata,
};

static struct platform_device pcm049_vcc_1v8_device = {
	.name	= "reg-fixed-voltage",
	.id	= ARRAY_SIZE(pcm049_vcc_3v3_consumer_supply), //next id after
	.dev	= {
		.platform_data = &pcm049_vcc_1v8_config,
	},
};

static struct omap_abe_wm8974_data ksp5012_abe_audio_data = {
	.card_name = "KSP5012",
	.mclk_freq = 12288000,
};


static struct platform_device ksp5012_abe_audio_device = {
	.name		= "omap-abe-wm8974",
	.id		= -1,
	.dev = {
		.platform_data = &ksp5012_abe_audio_data,
        },
};

#ifdef CONFIG_WL12XX_PLATFORM_DATA
static struct regulator_consumer_supply pcm049_vmmc5_supply = {
        .supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data pcm049_vmmc5 = {
        .constraints = {
                .valid_ops_mask = REGULATOR_CHANGE_STATUS,
        },
        .num_consumer_supplies = 1,
        .consumer_supplies = &pcm049_vmmc5_supply,
};
static struct fixed_voltage_config pcm049_vwlan = {
	.supply_name			= "vwl1271",
	.microvolts			= 1800000, /* 1.8V */
	.startup_delay			= 70000, /* 70msec */
	.gpio				= GPIO_WL_EN,
	.enable_high			= 1,
	.enabled_at_boot		= 0,
	.init_data = &pcm049_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= ARRAY_SIZE(pcm049_vcc_3v3_consumer_supply)
				+ ARRAY_SIZE(pcm049_vcc_1v8_consumer_supply),
	.dev = {
		.platform_data = &pcm049_vwlan,
	},
};

#endif	// ifdef CONFIG_WL12XX_PLATFORM_DATA

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
static void pcm049_wl1271_init_card(struct mmc_card *card)
{
	/*
	 * Although the TiWi datasheet claims a max SDIO clock of 25MHz,
	 * emperical evidence indicates otherwise.
	 */
	card->cis.max_dtr = 14000000;
}

#endif	// if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
//		.ocr_mask 	= MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
//		.ocr_mask	= MMC_VDD_165_195,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	{
		.mmc		= 5,
		.name		= "wl1271",
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
//		.ocr_mask	= MMC_VDD_165_195, /* 1V8 */
//		.ocr_mask 	= MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
		.built_in	= true,
		.nonremovable	= true,
		.init_card	= pcm049_wl1271_init_card,
		.gpio_wp	= -EINVAL,
	},
#endif
	{}	/* Terminator */
};

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct omap_smsc911x_platform_data __initdata board_smsc911x_data = {
	.cs		= KSP5012_ETH_CS,
	.gpio_irq	= KSP5012_ETH_GPIO_IRQ,
	.gpio_reset	= -EINVAL,
	.flags		= SMSC911X_USE_16BIT,
};

static inline void __init pcm049_init_smsc911x(void)
{
	omap_mux_init_gpio(KSP5012_ETH_GPIO_IRQ, OMAP_PIN_INPUT);
	gpmc_smsc911x_init(&board_smsc911x_data);
}
#else
static inline void __init pcm049_init_smsc911x(void) { return; }
#endif

#if defined(CONFIG_MTD_NAND_OMAP2) || defined(CONFIG_MTD_NAND_OMAP2_MODULE)
static struct omap_nand_platform_data pcm049_nand_data = {
	.cs			= 0,
	.ecc_opt		= OMAP_ECC_BCH8_CODE_HW,
	.dev_ready		= 1,
};

static void __init pcm049_init_nand(void)
{
	if (gpmc_nand_init(&pcm049_nand_data) < 0)
		pr_err("Unable to register NAND device\n");
}
#else
static inline void pcm049_init_nand(void) { return; }
#endif

static int pcm049_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "%s: NULL platform data\n", __func__);
		return -EINVAL;
	}

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret < 0) {
			pr_err("Failed configuring MMC1 card detect\n");
			return ret;
		}
		else {
			pdata->slots[0].card_detect_irq = ret;
			pdata->slots[0].card_detect = twl6030_mmc_card_detect;
		}
	}
	return 0;
}

static __init void hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed hsmmc_set_late_init\n");
		return;
	}
	pdata = dev->platform_data;

	pdata->init = pcm049_hsmmc_late_init;
}

static int __init pcm049_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		hsmmc_set_late_init(&c->pdev->dev);

	return 0;
}

/*
 * Setup CFG_TRANS mode as follows:
 * 0x00 (OFF) when in OFF state(bit offset 4)
 * - these bits a read only, so don't touch them
 * 0x00 (OFF) when in sleep (bit offset 2)
 * 0x01 (PWM/PFM Auto) when in ACTive state (bit offset 0)
 */
#define TWL6030_REG_VCOREx_CFG_TRANS_MODE		(0x00 << 4 | \
		TWL6030_RES_OFF_CMD << TWL6030_REG_CFG_TRANS_SLEEP_CMD_OFFSET |\
		TWL6030_RES_AUTO_CMD << TWL6030_REG_CFG_TRANS_ACT_CMD_OFFSET)

#define TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC "OFF=OFF SLEEP=OFF ACT=AUTO"

/* OMAP4430 - All vcores: 1, 2 and 3 should go down with PREQ */
static struct twl_reg_setup_array omap4430_twl6030_setup[] = {
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE1_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE1 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE2_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE2 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE3_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE3 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{ .desc = NULL} /* TERMINATOR */
};

/* OMAP4460 - VCORE3 is unused, 1 and 2 should go down with PREQ */
static struct twl_reg_setup_array omap4460_twl6030_setup[] = {
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE1_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE 1" TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE2_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE2 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_CFG_SMPS_PD,
		.val = 0x77,
		.desc = "VCORE1 disable PD on shutdown",
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE3_CFG_GRP,
		.val = 0x00,
		.desc = "VCORE3 - remove binding to groups",
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VMEM_CFG_GRP,
		.val = 0x00,
		.desc = "VMEM - remove binding to groups",
	},
	{ .desc = NULL} /* TERMINATOR */
};

static struct twl4030_platform_data pcm049_twldata;

static struct platform_device *pcm049_devices[] __initdata = {
	&pcm049_vcc_3v3_device,
	&pcm049_vcc_1v8_device,
//	&omap_vwlan_device,
//	&omap_vedt_device,
	&ksp5012_abe_audio_device,
	&leds_gpio,
	&ksp5012_gpio_keys_device,
	&pwm_device,
	&ksp5012_backlight_device,
};

static struct at24_platform_data board_eeprom = {
	.byte_len = 4096,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16,
};

/* TMP102 PCB Temperature sensor close to OMAP
 * values for .slope and .offset are taken from OMAP5 structure,
 * as TMP102 sensor was not used by domains other then CPU
 */
static struct tmp102_platform_data tmp102_omap_info = {
	.slope = 470,
	.slope_cpu = 1063,
	.offset = -1272,
	.offset_cpu = -477,
	.domain = "pcb", /* for hotspot extrapolation */
};

#ifdef CONFIG_TOUCHSCREEN_FT5X06
static struct ft5x0x_ts_platform_data pba_ft5x06_pdata = {
	.irq_pin	= KSP5012_FT5x06_GPIO_IRQ,
	.wake_pin	= KSP5012_FT5x06_GPIO_WAKE,
};
#endif

#ifdef CONFIG_INPUT_ADXL34X
static struct adxl34x_platform_data adxl34x_info = {
	.x_axis_offset = 0,
	.y_axis_offset = 0,
	.z_axis_offset = 0,
	.tap_threshold = 0x31,
	.tap_duration = 0x10,
	.tap_latency = 0x60,
	.tap_window = 0xF0,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
	.act_axis_control = 0xFF,
	.activity_threshold = 5,
	.inactivity_threshold = 3,
	.inactivity_time = 4,
	.free_fall_threshold = 0x7,
	.free_fall_time = 0x20,
	.data_rate = 0x8,
	.data_range = ADXL_FULL_RES,

	.ev_type = EV_ABS,
	.ev_code_x = ABS_X,		/* EV_REL */
	.ev_code_y = ABS_Y,		/* EV_REL */
	.ev_code_z = ABS_Z,		/* EV_REL */

	.ev_code_tap = {BTN_0, BTN_1, BTN_2}, /* EV_KEY x,y,z */

/*	.ev_code_ff = KEY_F,*/		/* EV_KEY */
/*	.ev_code_act_inactivity = KEY_A,*/	/* EV_KEY */
	.power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
	.fifo_mode = ADXL_FIFO_STREAM,
	.orientation_enable = 0,
	.deadzone_angle = ADXL_DEADZONE_ANGLE_10p8,
	.divisor_length = ADXL_LP_FILTER_DIVISOR_16,
	/* EV_KEY {+Z, +Y, +X, -X, -Y, -Z} */
	.ev_codes_orient_3d = {BTN_Z, BTN_Y, BTN_X, BTN_A, BTN_B, BTN_C},
};
#endif

static struct i2c_board_info __initdata pcm049_i2c_1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &pcm049_twldata,
	},
	{
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &board_eeprom,
	},
	{
		I2C_BOARD_INFO("tmp102_temp_sensor", 0x4B),
		.platform_data = &tmp102_omap_info,
	},
};

/*
 * Caution: if you are going to change the order of these, you will need
 * to change the pcm049_i2c_4_boardinfo irq settings in pcm049_i2c_init().
 */
static struct i2c_board_info __initdata pcm049_i2c_4_boardinfo[] = {
#ifdef CONFIG_TOUCHSCREEN_FT5X06
	{
		/* Touch controller built into LCD panel (capacitive) */
		I2C_BOARD_INFO(FT5X0X_NAME, 0x38),	/* Touch controller */
		.irq = OMAP_GPIO_IRQ(KSP5012_FT5x06_GPIO_IRQ),
		.platform_data = &pba_ft5x06_pdata,
	},
#endif
#ifdef CONFIG_INPUT_ADXL34X_I2C
        {
                I2C_BOARD_INFO("adxl34x", 0x53),
                .irq = OMAP_GPIO_IRQ(KSP5012_ADXL34X_IRQ),
                .platform_data = &adxl34x_info,
        },
#endif
	{
		I2C_BOARD_INFO("wm8974", 0x1a), /* Audio */
	},
};

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
                                struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request(USE_MUTEX_LOCK);
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id,
							USE_MUTEX_LOCK);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata pcm049_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata pcm049_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata pcm049_i2c_4_bus_pdata;

static int __init pcm049_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &pcm049_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &pcm049_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &pcm049_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &pcm049_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(3, &pcm049_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &pcm049_i2c_4_bus_pdata);

	omap4_pmic_get_config(&pcm049_twldata, TWL_COMMON_PDATA_USB,
			TWL_COMMON_PDATA_MADC |
			TWL_COMMON_PDATA_BCI |
			TWL_COMMON_REGULATOR_VDAC |
			TWL_COMMON_REGULATOR_VAUX1 |
			TWL_COMMON_REGULATOR_VAUX2 |
			TWL_COMMON_REGULATOR_VAUX3 |
			TWL_COMMON_REGULATOR_VMMC |
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VUSIM |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_CLK32KG |
			TWL_COMMON_REGULATOR_V1V8 |
			TWL_COMMON_REGULATOR_V2V1 |
			TWL_COMMON_REGULATOR_SYSEN |
			TWL_COMMON_REGULATOR_CLK32KAUDIO |
			TWL_COMMON_REGULATOR_REGEN1);

	/* Add one-time registers configuration */
	if (cpu_is_omap443x())
		pcm049_twldata.reg_setup_script = omap4430_twl6030_setup;
	else if (cpu_is_omap446x())
		pcm049_twldata.reg_setup_script = omap4460_twl6030_setup;

	/* RTC-8564 IRQ mux */
	omap_mux_init_gpio(KSP5012_RTC_IRQ, OMAP_PIN_INPUT);

	//some of these should be at 400 rather than 100
	omap_register_i2c_bus(1, 100, pcm049_i2c_1_boardinfo,
				ARRAY_SIZE(pcm049_i2c_1_boardinfo));
	omap_register_i2c_bus(3, 100, NULL, 0);
	omap_register_i2c_bus(4, 100, pcm049_i2c_4_boardinfo,
				ARRAY_SIZE(pcm049_i2c_4_boardinfo));

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

        /*
         * Drive MSECURE high for TWL6030/6032 write access.
         */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	/* WLAN IRQ - GPIO 109 */
	OMAP4_MUX(SDMMC1_DAT7, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 106 */
	OMAP4_MUX(SDMMC1_DAT4, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06
	/* EDT FT5X06 WAKE - GPIO 108 */
	OMAP4_MUX(SDMMC1_DAT6, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* EDT FT5X06 IRQ - GPIO 107 */
	OMAP4_MUX(SDMMC1_DAT5, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
#endif

	/* CSI21 */
	OMAP4_MUX(CSI21_DX0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DY0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DY1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DY2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* CAM (OV5650) PWDN */
	OMAP4_MUX(ABE_MCBSP2_DR, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* CAM (OV5650) RESETB */
	OMAP4_MUX(CAM_GLOBALRESET, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),

	/* RTC-8564 IRQ */
	OMAP4_MUX(ABE_MCBSP1_FSX, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN), //GPIO_117

	/* ADXL34X IRQ */
	OMAP4_MUX(KPD_ROW3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN), //GPIO_175
	OMAP4_MUX(KPD_ROW4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN), //GPIO_176

	/* FPGA */
	OMAP4_MUX(ABE_MCBSP2_CLKX, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), // GPIO_110
	OMAP4_MUX(ABE_MCBSP2_DX, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), // GPIO_112
	OMAP4_MUX(ABE_MCBSP1_CLKX, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), // GPIO_114
	OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE3 | OMAP_PIN_INPUT), // GPIO_115

	/* Wake signal */
	OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN), //GPIO_139
	OMAP4_MUX(ABE_CLKS, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN), //GPIO_118

	/* GPS ON-OFF */
	OMAP4_MUX(UNIPRO_TY0, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), // GPIO_172
	/* GPS RESET */
	OMAP4_MUX(UNIPRO_TY1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), // GPIO_174
	/* GPS SYSTEM-ON */
	OMAP4_MUX(UNIPRO_RX1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN), //GPI_177

	/* MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#ifdef CONFIG_OMAP2_DSS_DPI
	/* dispc2_data23 */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data22 */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data21 */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data20 */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data19 */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data18 */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data15 */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data14 */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data13 */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data12 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data11 */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data10 */
	OMAP4_MUX(DPM_EMU3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data9 */
	OMAP4_MUX(DPM_EMU4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data16 */
	OMAP4_MUX(DPM_EMU5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data17 */
	OMAP4_MUX(DPM_EMU6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_hsync */
	OMAP4_MUX(DPM_EMU7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_pclk */
	OMAP4_MUX(DPM_EMU8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_vsync */
	OMAP4_MUX(DPM_EMU9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_de */
	OMAP4_MUX(DPM_EMU10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data8 */
	OMAP4_MUX(DPM_EMU11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data7 */
	OMAP4_MUX(DPM_EMU12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data6 */
	OMAP4_MUX(DPM_EMU13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data5 */
	OMAP4_MUX(DPM_EMU14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data4 */
	OMAP4_MUX(DPM_EMU15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data3 */
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data2 */
	OMAP4_MUX(DPM_EMU17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data1 */
	OMAP4_MUX(DPM_EMU18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data0 */
	OMAP4_MUX(DPM_EMU19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* LCD Backlight GPIO for Splash */
	OMAP4_MUX(ABE_DMIC_DIN3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
#endif	/* ifdef CONFIG_OMAP2_DSS_DPI */
	OMAP4_MUX(SYS_NIRQ1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_WAKEUPENABLE),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

/* Display */
static int pcm049_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	return gpio_direction_output(KSP5012_LCD_ENABLE, 1);
}

static void pcm049_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_direction_output(KSP5012_LCD_ENABLE, 0);
}

#if defined(CONFIG_OMAP2_DSS_DPI) && defined(CONFIG_PANEL_GENERIC_DPI)
static struct panel_generic_dpi_data omap4_dpi_panel = {
	.name			= "nhd50800480tf-atxl",
	.platform_enable	= pcm049_panel_enable_lcd,
	.platform_disable	= pcm049_panel_disable_lcd,
};

struct omap_dss_device pcm049_dpi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dpi",
	.driver_name		= "generic_dpi_panel",
	.data			= &omap4_dpi_panel,
	.phy.dpi.data_lines	= 24,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};
#endif

static struct omap_dss_device *pcm049_dss_devices[] = {
#if defined(CONFIG_OMAP2_DSS_DPI) && defined(CONFIG_PANEL_GENERIC_DPI)
	&pcm049_dpi_device,
#endif
};

static struct omap_dss_board_info pcm049_dss_data = {
	.num_devices	= ARRAY_SIZE(pcm049_dss_devices),
	.devices	= pcm049_dss_devices,
#ifdef CONFIG_OMAP2_DSS_DPI
	.default_device	= &pcm049_dpi_device,
#endif
};

#define PCM049_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */

static struct dsscomp_platform_data dsscomp_config_pcm049 = {
	.tiler1d_slotsz = PCM049_FB_RAM_SIZE,
};

static struct sgx_omaplfb_config omaplfb_config_pcm049[] = {
	{
		.tiler2d_buffers = 2,
		.swap_chain_length = 2,
	},
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	},
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_pcm049 = {
	.num_configs = ARRAY_SIZE(omaplfb_config_pcm049),
	.configs = omaplfb_config_pcm049,
};

static struct omapfb_platform_data pcm049_fb_pdata = {
	.mem_desc = {
		.region_cnt = ARRAY_SIZE(omaplfb_config_pcm049),
	},
};

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = 1,
	.reset_gpio_port[0]  = KSP5012_USBB1_PWR,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
};

static void __init pcm049_ehci_ohci_init(void)
{
	struct clk *auxclk;

	auxclk = omap_clk_get_by_name("auxclk3_src_ck");
	if (auxclk)
		clk_enable(auxclk);
	else
		printk("can not get clock for ehci/ohci USB\n");

	omap_mux_init_signal("gpmc_ncs4.gpio_101",
				OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT);
	usbhs_init(&usbhs_bdata);

	return;
}
#else
static void __init pcm049_ehci_ohci_init(void){}
#endif

static void __init pcm049_display_init(void)
{
	omapfb_set_platform_data(&pcm049_fb_pdata);
	omap_vram_set_sdram_vram(PCM049_FB_RAM_SIZE, 0);
	omap_mux_init_gpio(KSP5012_LCD_ENABLE, OMAP_PIN_OUTPUT);

	if ((gpio_request(KSP5012_LCD_ENABLE, "DISP_ENA") == 0) &&
		(gpio_direction_output(KSP5012_LCD_ENABLE, 1) == 0)) {
		gpio_export(KSP5012_LCD_ENABLE, 0);
		gpio_set_value(KSP5012_LCD_ENABLE, 0);
	} else
		printk(KERN_ERR "could not obtain gpio for DISP_ENA");

	omap_display_init(&pcm049_dss_data);
}

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
#if 0
static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	if (on) {
		gpio_direction_output(GPIO_WL_EN, 1);
		mdelay(1);
	} else {
		gpio_direction_output(GPIO_WL_EN, 0);
	}

	return 0;
}
#endif
#ifdef CONFIG_WL12XX_PLATFORM_DATA
#define GPIO_WIFI_IRQ 109
static struct wl12xx_platform_data omap_pcm049_wlan_data  __initdata = {
	.board_ref_clock = 2,
};
#endif

static void __init pcm049_wl12xx_init(void)
{
	omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_pcm049_wlan_data.irq = gpio_to_irq(GPIO_WIFI_IRQ);
	if (wl12xx_set_platform_data(&omap_pcm049_wlan_data))
		pr_err("error setting wl12xx data\n");
	if (platform_device_register(&omap_vwlan_device))
		pr_err("Error registering wl12xx device\n");
#endif

	return;
}
#endif

/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is capable
 * enough to wake-up the OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev,
                pm_message_t state)
{
        return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
        return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(struct kim_data_s *kdata)
{
        int port_id = 0;
        int err = 0;
        if (uart_req) {
                if (!kdata) {
                        pr_err("%s: NULL kim_data pointer\n", __func__);
                        return -EINVAL;
                }
                err = sscanf(kdata->dev_name, "/dev/ttyO%d", &port_id);
                if (!err) {
                        pr_err("%s: Wrong UART name: %s\n", __func__,
                                kdata->dev_name);
                        return -EINVAL;
                }
                err = omap_serial_ext_uart_disable(port_id);
                if (!err)
                        uart_req = false;
        }
        wake_unlock(&st_wk_lock);
        return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(struct kim_data_s *kdata)
{
        int port_id = 0;
        int err = 0;
        if (!uart_req) {
                if (!kdata) {
                        pr_err("%s: NULL kim_data pointer\n", __func__);
                        return -EINVAL;
                }
                err = sscanf(kdata->dev_name, "/dev/ttyO%d", &port_id);
                if (!err) {
                        pr_err("%s: Wrong UART name: %s\n", __func__,
                                kdata->dev_name);
                        return -EINVAL;
                }
                err = omap_serial_ext_uart_enable(port_id);
                if (!err)
                        uart_req = true;
        }
        wake_lock(&st_wk_lock);
        return err;
}

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = GPIO_BT_EN,
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3686400, //115200
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};

static struct platform_device wl12xx_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static inline void __init ksp5012_btwilink_init(void)
{
	pr_info("ksp5012: bt init\n");

	if (wilink_pdata.nshutdown_gpio != -1)
		omap_mux_init_gpio(wilink_pdata.nshutdown_gpio,
				OMAP_PIN_OUTPUT);

	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
}

#if defined(CONFIG_TI_EMIF) || defined(CONFIG_TI_EMIF_MODULE)
static struct __devinitdata emif_custom_configs custom_configs = {
	.mask   = EMIF_CUSTOM_CONFIG_LPMODE,
	.lpmode = EMIF_LP_MODE_SELF_REFRESH,
	.lpmode_timeout_performance = 512,
	.lpmode_timeout_power = 512,
	/* only at OPP100 should we use performance value */
	.lpmode_freq_threshold = 400000000,
};
#endif

static void __init emif_setup_device_details(unsigned long base)
{
	unsigned long cs1_used;
	unsigned char *emif;

	emif = ioremap(base, SZ_256);

	cs1_used = (readl(emif + EMIF_SDRAM_CONFIG) & EBANK_MASK)
		>> EBANK_SHIFT;

	if (memblock_phys_mem_size() > SZ_512M) {
		if (cs1_used) {
			omap_emif_set_device_details(1,
				&lpddr2_nanya_8G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);

			omap_emif_set_device_details(2,
				&lpddr2_nanya_8G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
			printk(KERN_INFO "%s: "
				"registered Nanya LPDDR_S4 1GB RAM with 2 CS\n",
				__func__);
		} else {
			omap_emif_set_device_details(1,
				&lpddr2_nanya_8G_S4_x2_1CS_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);

			omap_emif_set_device_details(2,
				&lpddr2_nanya_8G_S4_x2_1CS_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
			printk(KERN_INFO "%s: "
				"registered Nanya LPDDR_S4 1GB RAM with 1 CS\n",
				__func__);
		}
	} else {
		omap_emif_set_device_details(1, &lpddr2_nanya_4G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);

		omap_emif_set_device_details(2, &lpddr2_nanya_4G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
		printk(KERN_INFO
			"%s: registered Nanya LPDDR_S4 512MB RAM\n", __func__);
	}
}

static void __init pcm049_init(void)
{
	int status;

	//emif_setup_device_details(0x4C000000);

	omap4_mux_init(board_mux, NULL, OMAP_PACKAGE_CBS);

	omap_mux_init_signal("fref_clk4_req", OMAP_MUX_MODE1);
	pcm049_audio_mux_init();
	omap_create_board_props();

	pcm049_i2c_init();
	omap4_board_serial_init();

	pcm049_display_init();

	platform_add_devices(pcm049_devices, ARRAY_SIZE(pcm049_devices));

	pcm049_init_smsc911x();
	pcm049_hsmmc_init(mmc);

	pcm049_ehci_ohci_init();
	usb_musb_init(&musb_board_data);

	init_duty_governor();
	omap_init_dmm_tiler();
	omap4_register_ion();
	omap_die_governor_register_pdata(&omap_gov_pdata);
	pcm049_init_nand();

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	pcm049_wl12xx_init();
	ksp5012_btwilink_init();
#endif

	if (cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}

	/* SOM status LEDs */
	omap_mux_init_gpio(152, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(153, OMAP_PIN_OUTPUT);

#if 0
	if (gpio_request(152, "LED_152") == 0)
		gpio_direction_output(152, 0);
	if (gpio_request(153, "LED_153") == 0)
		gpio_direction_output(153, 0);
#endif

	omap_enable_smartreflex_on_init();
}

static void __init pcm049_reserve(void)
{
#if defined(CONFIG_OMAP_RAM_CONSOLE)
	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);
#endif

	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);
	omap_android_display_setup(&pcm049_dss_data,
			&dsscomp_config_pcm049,
			&omaplfb_plat_data_pcm049,
			&pcm049_fb_pdata);

	omap4_ion_init();
	omap4_secure_workspace_addr_default();
	omap_reserve();
}

static void __init ksp5012_init_early(void)
{
	omap4430_init_early();
	if (cpu_is_omap446x())
		omap_tps6236x_gpio_no_reset_wa(TPS62361_GPIO, -1, 32);
}

MACHINE_START(PCM049, "pcm049")
	/* Maintainer: Jan Weitzel - Phytec Messtechnik GmbH */
	.atag_offset	= 0x100,
	.reserve	= pcm049_reserve,
	.map_io		= omap4_map_io,
	.init_early	= ksp5012_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= pcm049_init,
	.timer		= &omap4_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
