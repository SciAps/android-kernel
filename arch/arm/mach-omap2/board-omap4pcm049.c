/*
 * Board support file for Phytec phyCORE-OMAP4 Board.
 *
 * Copyright (C) 2012 Phytec Messtechnik GmbH
 *
 * Author: Jan Weitzel <armlinux@phytec.de>
 *         Steve Schefter <steve@scheftech.com>
 *
 * Based on mach-omap2/board-omap4panda.c
 *
 * Author: David Anders <x0132446@ti.com>
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
#include <linux/mfd/stmpe.h>
#include <linux/mmc/card.h>
#ifdef CONFIG_TOUCHSCREEN_EDT_FT5X06
#include <linux/input/edt-ft5x06.h>
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
#include <linux/platform_data/omap4-keypad.h>
#include <linux/platform_data/omap-abe-tlv320aic3x.h>
#include <linux/omap4_duty_cycle_governor.h>

#include <sound/tlv320aic3x.h>

#include <mach/hardware.h>
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
#include <plat/omap4-keypad.h>
#include <plat/android-display.h>
#ifdef CONFIG_PANEL_GENERIC_DPI
#include <video/omap-panel-generic-dpi.h>
#endif
#ifdef CONFIG_PANEL_TC358765
#include <video/omap-panel-tc358765.h>
#endif

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

#define OMAP4_PCM049_ETH_GPIO_IRQ	121
#define OMAP4_PCM049_ETH_CS		5
#define OMAP4_PCM049_STMPE811_GPIO_IRQ	117
#define OMAP4_PCM049_FT5x06_GPIO_IRQ	64
#define OMAP4_PCM049_LCD_ENABLE		118
#define OMAP4_PCM049_OTG_VBUS		115
#define OMAP_UART_GPIO_MUX_MODE_143	143
#define TPS62361_GPIO			182	/* VCORE1 power control */
#define	HDMI_GPIO_HPD			63

#if 0
// The keypad is disabled by default in favour of using the on-screeen
// Navigation Bar.
static int pcm049_keymap[] = {
	KEY(0, 0, KEY_1),
	KEY(0, 1, KEY_2),
	KEY(0, 2, KEY_3),

	KEY(1, 0, KEY_4),
	KEY(1, 1, KEY_5),
	KEY(1, 2, KEY_6),

	KEY(2, 0, KEY_7),
	KEY(2, 1, KEY_8),
	KEY(2, 2, KEY_9),

	KEY(3, 0, KEY_BACK),
	KEY(3, 1, KEY_0),
	KEY(3, 2, KEY_HOME),
};

static struct omap_device_pad keypad_pads[] = {
	{	.name   = "kpd_col1.kpd_col1",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE1,
	},
	{	.name   = "kpd_col1.kpd_col1",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE1,
	},
	{	.name   = "kpd_col2.kpd_col2",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE1,
	},
	{	.name   = "kpd_col3.kpd_col3",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE1,
	},
	{	.name   = "kpd_col4.kpd_col4",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE1,
	},
	{	.name   = "kpd_col5.kpd_col5",
		.enable = OMAP_WAKEUP_EN | OMAP_MUX_MODE1,
	},
	{	.name   = "kpd_row0.kpd_row0",
		.enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
			OMAP_MUX_MODE1 | OMAP_INPUT_EN,
	},
	{	.name   = "kpd_row1.kpd_row1",
		.enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
			OMAP_MUX_MODE1 | OMAP_INPUT_EN,
	},
	{	.name   = "kpd_row2.kpd_row2",
		.enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
			OMAP_MUX_MODE1 | OMAP_INPUT_EN,
	},
	{	.name   = "kpd_row3.kpd_row3",
		.enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
			OMAP_MUX_MODE1 | OMAP_INPUT_EN,
	},
	{	.name   = "kpd_row4.kpd_row4",
		.enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
			OMAP_MUX_MODE1 | OMAP_INPUT_EN,
	},
	{	.name   = "kpd_row5.kpd_row5",
		.enable = OMAP_PULL_ENA | OMAP_PULL_UP | OMAP_WAKEUP_EN |
			OMAP_MUX_MODE1 | OMAP_INPUT_EN,
	},
};

static struct matrix_keymap_data pcm049_keymap_data = {
	.keymap                 = pcm049_keymap,
	.keymap_size            = ARRAY_SIZE(pcm049_keymap),
};

static struct omap4_keypad_platform_data pcm049_keypad_data = {
	.keymap_data            = &pcm049_keymap_data,
	.rows                   = 4,
	.cols                   = 3,
};

static struct omap_board_data keypad_data = {
	.id			= 1,
	.pads			= keypad_pads,
	.pads_cnt		= ARRAY_SIZE(keypad_pads),
};

static void __init pcm049_keyboard_mux_init(void)
{
	omap_mux_init_signal("kpd_col0.kpd_col0", OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col1.kpd_col1", OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col2.kpd_col2", OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col3.kpd_col3", OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col4.kpd_col4", OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_col5.kpd_col5", OMAP_MUX_MODE1);
	omap_mux_init_signal("kpd_row0.kpd_row0",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_MUX_MODE1 | OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row1.kpd_row1",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_MUX_MODE1 | OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row2.kpd_row2",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_MUX_MODE1 | OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row3.kpd_row3",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_MUX_MODE1 | OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row4.kpd_row4",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_MUX_MODE1 | OMAP_INPUT_EN);
	omap_mux_init_signal("kpd_row5.kpd_row5",
				OMAP_PULL_ENA | OMAP_PULL_UP |
				OMAP_MUX_MODE1 | OMAP_INPUT_EN);
}
#endif // if 0

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
	/* tlv320aic3x analog supplies */
	REGULATOR_SUPPLY("AVDD", "4-0018"),
	REGULATOR_SUPPLY("DRVDD", "4-0018"),
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
	/* tlv320aic3x digital supplies */
	REGULATOR_SUPPLY("IOVDD", "4-0018"),
	REGULATOR_SUPPLY("DVDD", "4-0018"),
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

static struct omap_abe_tlv320aic3x_data pcm049_abe_audio_data = {
	.card_name = "PCM049",
	.mclk_freq = 19200000,
};

static struct platform_device pcm049_abe_audio_device = {
	.name		= "omap-abe-tlv320aic3x",
	.id		= -1,
	.dev = {
		.platform_data = &pcm049_abe_audio_data,
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
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
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

static int pcm049_set_vbus(struct phy_companion *x, bool enabled)
{
	return gpio_direction_output(OMAP4_PCM049_OTG_VBUS, enabled);
}

#ifdef CONFIG_WL12XX_PLATFORM_DATA
#define GPIO_WIFI_IRQ	109
static struct wl12xx_platform_data omap_pcm049_wlan_data  __initdata = {
	.board_ref_clock = 2,
};
#endif
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
static void pcm049_wl1271_init_card(struct mmc_card *card)
{
	/*
	 * Although the TiWi datasheet claims a max SDIO clock of 25MHz,
	 * emperical evidence indicates otherwise.
	 */
	card->cis.max_dtr = 14000000;
}

static void __init pcm049_wifi_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);

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
	omap_pcm049_wlan_data.irq = gpio_to_irq(GPIO_WIFI_IRQ);
	if (wl12xx_set_platform_data(&omap_pcm049_wlan_data))
		pr_err("error setting wl12xx data\n");
	if (platform_device_register(&omap_vwlan_device))
		pr_err("Error registering wl12xx device\n");
#endif
#define GPIO_WL_EN     106
	if (gpio_request(GPIO_WL_EN, "wl-en") < 0)
		printk(KERN_WARNING "failed to request GPIO#%d\n", GPIO_WL_EN);
	else if (gpio_direction_output(GPIO_WL_EN, 1))
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"output\n", GPIO_WL_EN);
	else
		omap_mux_init_gpio(GPIO_WL_EN, OMAP_PIN_OUTPUT);

#define GPIO_BT_EN     107
	/* Currently no support for the BT portion of the TiWi-R2 */
	omap_mux_init_gpio(GPIO_BT_EN, OMAP_PIN_OUTPUT);
	if (gpio_request(GPIO_BT_EN, "bt-en") < 0)
		printk(KERN_WARNING "failed to request GPIO#%d\n", GPIO_BT_EN);
	else if (gpio_direction_output(GPIO_BT_EN, 0))
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"output\n", GPIO_BT_EN);
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
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.mmc		= 5,
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
		.name		= "wl1271",
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
		.init_card	= pcm049_wl1271_init_card,
#else
		.caps		= MMC_CAP_4_BIT_DATA,
		/* Set by J12 on the carrier board, default 3.3V: */
		.ocr_mask	= MMC_VDD_32_33,
		.gpio_cd	= 30,
#endif
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};


#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct omap_smsc911x_platform_data __initdata board_smsc911x_data = {
	.cs		= OMAP4_PCM049_ETH_CS,
	.gpio_irq	= OMAP4_PCM049_ETH_GPIO_IRQ,
	.gpio_reset	= -EINVAL,
	.flags		= SMSC911X_USE_16BIT,
};

static inline void __init pcm049_init_smsc911x(void)
{
	omap_mux_init_gpio(OMAP4_PCM049_ETH_GPIO_IRQ, OMAP_PIN_INPUT);
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
	&pcm049_abe_audio_device,
	&leds_gpio,
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

static struct stmpe_gpio_platform_data pba_gpio_stm_data = {
	.gpio_base = -1,
	.norequest_mask = STMPE_GPIO_NOREQ_811_TOUCH,
};

static struct stmpe_ts_platform_data pba_ts_stm_pdata = {
	.sample_time = 4,
	.mod_12b = 1,
	.ref_sel = 0,
	.adc_freq = 1,
	.ave_ctrl = 3,
	.touch_det_delay = 3,
	.settling = 3,
	.fraction_z = 7,
	.i_drive = 0,
};

static struct stmpe_platform_data pba_stm_pdata = {
	.blocks = STMPE_BLOCK_GPIO | STMPE_BLOCK_TOUCHSCREEN,
	.irq_base = TWL6030_IRQ_END,
	.irq_trigger = IRQF_TRIGGER_LOW,
	.gpio = &pba_gpio_stm_data,
	.ts = &pba_ts_stm_pdata,
};

#ifdef CONFIG_TOUCHSCREEN_EDT_FT5X06
static struct edt_ft5x06_platform_data pba_ft5x06_pdata = {
	.reset_pin      =  -1,          /* static high */
};
#endif

static struct pca9532_platform_data pba_pca9532 = {
	.leds = {
		{
			.name = "board:red:free_use1",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		}, {
			.name = "board:yellow:free_use2",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		}, {
			.name = "board:yellow:free_use3",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		}, {
			.name = "board:green:free_use4",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
	},
	.psc = { 1, 1 },
	.pwm = { 1, 1 },
};

static struct aic3x_setup_data pcm049_aic33_setup = {
	.gpio_func[0] = AIC3X_GPIO1_FUNC_DISABLED,
	.gpio_func[1] = AIC3X_GPIO2_FUNC_DIGITAL_MIC_INPUT,
};

static struct aic3x_pdata pcm049_aic33_data = {
	.setup = &pcm049_aic33_setup,
	.gpio_reset = -1,
};

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

static struct i2c_board_info __initdata pcm049_i2c_2_boardinfo[] = {
};

static struct i2c_board_info __initdata pcm049_i2c_3_boardinfo[] = {
};

/*
 * Caution: if you are going to change the order of these, you will need
 * to change the pcm049_i2c_4_boardinfo irq settings in pcm049_i2c_init().
 */
static struct i2c_board_info __initdata pcm049_i2c_4_boardinfo[] = {
	{
		/* Touch controller built into baseboard (resistive) */
		I2C_BOARD_INFO("stmpe811", 0x41),	/* Touch controller */
		.platform_data = &pba_stm_pdata,
	},
#ifdef CONFIG_TOUCHSCREEN_EDT_FT5X06
	{
		/* Touch controller built into LCD panel (capacitive) */
		I2C_BOARD_INFO("edt-ft5x06", 0x38),	/* Touch controller */
		.platform_data = &pba_ft5x06_pdata,
	},
#endif
	{
		I2C_BOARD_INFO("max1037", 0x64),	/* A/D converter */
	},
	{
		I2C_BOARD_INFO("pca9533", 0x62),	/* Leds pca9533 */
		.platform_data = &pba_pca9532,
	},
	{
		I2C_BOARD_INFO("tlv320aic3007", 0x18),	/* Audio */
		.platform_data = &pcm049_aic33_data,
	},
#ifdef CONFIG_PANEL_TC358765
	{
		I2C_BOARD_INFO("tc358765_i2c_driver", 0x0f), /* DSI -> LVDS */
	},
#endif
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
static struct omap_i2c_bus_board_data __initdata pcm049_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata pcm049_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata pcm049_i2c_4_bus_pdata;

static int __init pcm049_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &pcm049_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &pcm049_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &pcm049_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &pcm049_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &pcm049_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &pcm049_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &pcm049_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &pcm049_i2c_4_bus_pdata);

	if (omap_mux_init_gpio(OMAP4_PCM049_STMPE811_GPIO_IRQ, OMAP_PIN_INPUT))
		printk(KERN_ERR "Failed to mux GPIO%d for STMPE811 IRQ\n",
			OMAP4_PCM049_STMPE811_GPIO_IRQ);
	else if (gpio_request(OMAP4_PCM049_STMPE811_GPIO_IRQ, "STMPE811 irq"))
		printk(KERN_ERR "Failed to request GPIO%d for STMPE811 IRQ\n",
			OMAP4_PCM049_STMPE811_GPIO_IRQ);
	else
		gpio_direction_input(OMAP4_PCM049_STMPE811_GPIO_IRQ);

#ifdef CONFIG_TOUCHSCREEN_EDT_FT5X06
	if (omap_mux_init_gpio(OMAP4_PCM049_FT5x06_GPIO_IRQ, OMAP_PIN_INPUT))
		printk(KERN_ERR "Failed to mux GPIO%d for FT5x06 IRQ\n",
			OMAP4_PCM049_FT5x06_GPIO_IRQ);
#endif

	omap4_pmic_get_config(&pcm049_twldata, TWL_COMMON_PDATA_USB,
			TWL_COMMON_REGULATOR_VDAC |
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
			TWL_COMMON_REGULATOR_V2V1);
	pcm049_twldata.usb->otg_set_vbus = pcm049_set_vbus;

	/* Add one-time registers configuration */
	if (cpu_is_omap443x())
		pcm049_twldata.reg_setup_script = omap4430_twl6030_setup;
	else if (cpu_is_omap446x())
		pcm049_twldata.reg_setup_script = omap4460_twl6030_setup;

	//some of these should be at 400 rather than 100
	omap_register_i2c_bus(1, 100, pcm049_i2c_1_boardinfo,
				ARRAY_SIZE(pcm049_i2c_1_boardinfo));
	omap_register_i2c_bus(2, 100, pcm049_i2c_2_boardinfo,
				ARRAY_SIZE(pcm049_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 100, pcm049_i2c_3_boardinfo,
				ARRAY_SIZE(pcm049_i2c_3_boardinfo));
	pcm049_i2c_4_boardinfo[0].irq =
			gpio_to_irq(OMAP4_PCM049_STMPE811_GPIO_IRQ),
#ifdef CONFIG_TOUCHSCREEN_EDT_FT5X06
	pcm049_i2c_4_boardinfo[1].irq =
			gpio_to_irq(OMAP4_PCM049_FT5x06_GPIO_IRQ),
#endif
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
	return gpio_direction_output(OMAP4_PCM049_LCD_ENABLE, 1);
}

static void pcm049_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_direction_output(OMAP4_PCM049_LCD_ENABLE, 0);
}

#if defined(CONFIG_OMAP2_DSS_DPI) && defined(CONFIG_PANEL_GENERIC_DPI)
static struct panel_generic_dpi_data omap4_dpi_panel = {
	.name			= "pd050vl1",
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

#if defined(CONFIG_OMAP2_DSS_DSI) && defined(CONFIG_PANEL_TC358765)

static struct tc358765_board_data omap_dsi_panel = {
	.lp_time	= 0x4,
	.clrsipo	= 0x3,
	.lv_is		= 0x1,
	.lv_nd		= 0x6,
	.vtgen		= 0x0,
	.vsdelay	= 0xf02,
	.pclkdiv	= 0x0,
	.pclksel	= 0x0,
	.lvdlink	= 0x0,
	.msf		= 0x0,
	.evtmode	= 0x1,
};

static struct omap_dss_device pcm049_dsi_device = {
	.name                   = "lcd",
	.driver_name            = "tc358765",
	.type                   = OMAP_DISPLAY_TYPE_DSI,
	.data			= &omap_dsi_panel,
	.phy.dsi                = {
		.clk_lane       = 3,
		.clk_pol        = 0,
		.data1_lane     = 1,
		.data1_pol      = 0,
		.data2_lane     = 2,
		.data2_pol      = 0,
		.data3_lane     = 4,
		.data3_pol      = 0,
		.data4_lane     = 5,
		.data4_pol      = 0,
		.module		= 0,
	},

	.clocks = {
		.dispc = {
			 .channel = {
				.lck_div        = 1,
				.pck_div        = 2,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
		},

		.dsi = {
			.regn           = 38,
			.regm           = 441,
			.regm_dispc     = 6,
			.regm_dsi       = 9,
			.lp_clk_div     = 5,
			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
		},
	},

	.panel = {
		.timings = {
			.x_res		= 800,		//= 1280,
			.y_res		= 480,		//= 800,
			.pixel_clock	= 32000,
			.hfp		= 42,
			.hsw		= 128,
			.hbp		= 86,
			.vfp		= 10,
			.vsw		= 2,
			.vbp		= 33,
		},
		.width_in_um = 217000,
		.height_in_um = 135600,
	},

	.ctrl = {
		.pixel_size = 24,
	},

	.channel = OMAP_DSS_CHANNEL_LCD,
	.platform_enable = pcm049_panel_enable_lcd,
	.platform_disable = pcm049_panel_disable_lcd,
};

#endif

#ifdef CONFIG_OMAP4_DSS_HDMI
static struct omap_dss_hdmi_data pcm049_hdmi_data = {
	.hpd_gpio = HDMI_GPIO_HPD,
};

static struct omap_dss_device  pcm049_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
	.data = &pcm049_hdmi_data,
};
#endif	// ifdef CONFIG_OMAP4_DSS_HDMI

static struct omap_dss_device *pcm049_dss_devices[] = {
#if defined(CONFIG_OMAP2_DSS_DPI) && defined(CONFIG_PANEL_GENERIC_DPI)
	&pcm049_dpi_device,
#endif
#if defined(CONFIG_OMAP2_DSS_DSI) && defined(CONFIG_PANEL_TC358765)
	&pcm049_dsi_device,
#endif
#ifdef CONFIG_OMAP4_DSS_HDMI
	&pcm049_hdmi_device,
#endif
};

static struct omap_dss_board_info pcm049_dss_data = {
	.num_devices	= ARRAY_SIZE(pcm049_dss_devices),
	.devices	= pcm049_dss_devices,
#ifdef CONFIG_OMAP2_DSS_DPI
	.default_device	= &pcm049_dpi_device,
#else
	.default_device	= &pcm049_dsi_device,
#endif
};

#define PCM049_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
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

	usbhs_init(&usbhs_bdata);

	return;
}
#else
static void __init pcm049_ehci_ohci_init(void){}
#endif

static void __init pcm049_display_init(void)
{
#ifdef CONFIG_OMAP2_DSS_DSI
	u32 reg;
#endif
	omap_vram_set_sdram_vram(PCM049_FB_RAM_SIZE, 0);
	omap_mux_init_gpio(OMAP4_PCM049_LCD_ENABLE, OMAP_PIN_OUTPUT);
	if ((gpio_request(OMAP4_PCM049_LCD_ENABLE, "DISP_ENA") == 0) &&
		(gpio_direction_output(OMAP4_PCM049_LCD_ENABLE, 1) == 0)) {
		gpio_export(OMAP4_PCM049_LCD_ENABLE, 0);
		gpio_set_value(OMAP4_PCM049_LCD_ENABLE, 0);
	} else
		printk(KERN_ERR "could not obtain gpio for DISP_ENA");

#ifdef CONFIG_OMAP2_DSS_DSI
        /* Enable 5 lanes in DSI1 module, disable pull down */
        reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
        reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
        reg |= 0x1F << OMAP4_DSI1_LANEENABLE_SHIFT;
        reg &= ~OMAP4_DSI1_PIPD_MASK;
        reg |= 0x1F << OMAP4_DSI1_PIPD_SHIFT;
        omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
#endif
	omap_display_init(&pcm049_dss_data);

#ifdef CONFIG_OMAP4_DSS_HDMI
	omap_hdmi_init(OMAP_HDMI_SDA_SCL_EXTERNAL_PULLUP);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT_PULLDOWN);
#endif
}

static void __init pcm049_init(void)
{
	int status;

	omap4_mux_init(board_mux, NULL, OMAP_PACKAGE_CBS);
	omap_mux_init_signal("fref_clk4_req", OMAP_MUX_MODE1);
	pcm049_audio_mux_init();
	omap_create_board_props();
	pcm049_i2c_init();

	platform_add_devices(pcm049_devices, ARRAY_SIZE(pcm049_devices));

	omap4_board_serial_init();
	pcm049_init_smsc911x();
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	pcm049_wifi_init();
#endif
	pcm049_hsmmc_init(mmc);
	pcm049_ehci_ohci_init();
	omap_mux_init_gpio(OMAP4_PCM049_OTG_VBUS, 0);
	if (gpio_request(OMAP4_PCM049_OTG_VBUS, "OTG_VBUS") == 0)
		gpio_direction_output(OMAP4_PCM049_OTG_VBUS, 0);
	usb_musb_init(&musb_board_data);

#if 0
	// The keypad is disabled by default
	pcm049_keyboard_mux_init();
	status = omap4_keyboard_init(&pcm049_keypad_data, &keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);
#endif

	omap_init_dmm_tiler();
	omap4_register_ion();
	pcm049_display_init();
	init_duty_governor();

	pcm049_init_nand();

	if (cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}
	omap_mux_init_gpio(152, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(153, OMAP_PIN_OUTPUT);

#if 0
	if (gpio_request(152, "LED_152") == 0)
		gpio_direction_output(152, 0);
	if (gpio_request(153, "LED_153") == 0)
		gpio_direction_output(153, 0);
#endif

	omap_enable_smartreflex_on_init();
#if 0
	/*
	 * 7X-38.400MBB-T oscillator uses:
	 * Up time = startup time(max 10ms) + enable time (max 100ns: round 1us)
	 * Down time = disable time (max 100ns: round 1us)
	 */
	omap_pm_set_osc_lp_time(11000, 1);
#endif
}

static void __init pcm049_reserve(void)
{
#if defined(CONFIG_OMAP_RAM_CONSOLE)
	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);
#endif
	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);
	omap4_ion_init();
	omap_reserve();
}

MACHINE_START(PCM049, "pcm049")
	/* Maintainer: Jan Weitzel - Phytec Messtechnik GmbH */
	.atag_offset	= 0x100,
	.reserve	= pcm049_reserve,
	.map_io		= omap4_map_io,
	.init_early	= omap4430_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= pcm049_init,
	.timer		= &omap4_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
