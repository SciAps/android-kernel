/*
 * Bluetooth Broadcom LWB rfkill power control via GPIO
 *
 */

#ifndef _LINUX_BCM_LWB_RFKILL_H
#define _LINUX_BCM_LWB_RFKILL_H

#include <linux/rfkill.h>

struct bcm_lwb_rfkill_platform_data {
	int nshutdown_gpio;

	struct rfkill *rfkdev;  /* for driver only */
};

#endif
