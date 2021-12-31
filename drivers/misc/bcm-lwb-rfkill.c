/*
 * Driver file for Sterling LWB rfkill driver
 *
 */


#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/rfkill.h>
#include "linux/bcm-lwb-rfkill.h"


struct bcm_lwb_rfkill_data {
	int nshutdown_gpio;
	struct rfkill			*rfkdev;
};

enum {
	RFKILL_BCM_LWB,
};


static int bcm_lwb_rfkill_set_block(void *data, bool blocked)
{
	struct platform_device *pdev = data;
	struct bcm_lwb_rfkill_data *rfkill = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s: %d\n", __func__, blocked);

	gpio_set_value(rfkill->nshutdown_gpio, (blocked?0:1));

	return 0;
}


static const struct rfkill_ops bcm_lwb_rfkill_ops = {
	.set_block = bcm_lwb_rfkill_set_block,
};


static int bcm_lwb_rfkill_probe(struct platform_device *pdev)
{
	struct bcm_lwb_rfkill_data *rfkill;
	struct bcm_lwb_rfkill_platform_data *pdata = pdev->dev.platform_data;
	struct rfkill *rfkdev;
	int ret = 0;

	dev_info(&pdev->dev, "%s: pdata->nshutdown_gpio: %d \n", __func__, pdata->nshutdown_gpio);

	rfkill = kzalloc(sizeof(*rfkill), GFP_KERNEL);
	if (!rfkill) {
		dev_err(&pdev->dev,
			"%s: no memory to alloc driver data\n", __func__);
		ret = -ENOMEM;
		goto error0;
	}

	rfkill->nshutdown_gpio = pdata->nshutdown_gpio;

	platform_set_drvdata(pdev, rfkill);

	if (gpio_request(rfkill->nshutdown_gpio, "bcm-lwb-bt-shutdown") < 0) {
        printk(KERN_WARNING "%s: failed to request GPIO#%d\n", __func__, rfkill->nshutdown_gpio);
		goto error1;
	}
    else if (gpio_direction_output(rfkill->nshutdown_gpio, 0) < 0) {
        printk(KERN_WARNING "%s: GPIO#%d cannot be configured as "
                "output\n", __func__, rfkill->nshutdown_gpio);
		goto error1;
	}

	/* WWAN rfkill device registration */
	rfkill->rfkdev = rfkill_alloc(pdev->name,
					&pdev->dev,
					RFKILL_TYPE_BLUETOOTH,
					&bcm_lwb_rfkill_ops,
					pdev);

	rfkdev = rfkill->rfkdev;

	if (!rfkdev) {
		dev_err(&pdev->dev,
			"%s: Error allocating modem rfkdev\n", __func__);
		ret = -ENOMEM;
		goto error1;
	}

	/* S/W blocked by default, persistent */
	{
		bool blocked = 0; // AAD: Temporary not blocked!
		rfkill_init_sw_state(rfkdev, blocked);
		gpio_set_value(rfkill->nshutdown_gpio, (blocked?0:1));
	}

	ret = rfkill_register(rfkdev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: Error registering modem rfkdev: %d\n",
			__func__, ret);
		ret = -EINVAL;
		goto error2;
	}

	/* hardware unblocked */
	if (rfkill->rfkdev)
		rfkill_set_hw_state(rfkdev, 0);

	return 0;

error2:
	rfkill_destroy(rfkdev);
error1:
	kfree(rfkill);
error0:
	return ret;
}


static int bcm_lwb_rfkill_remove(struct platform_device *pdev)
{
	struct bcm_lwb_rfkill_data *rfkill = platform_get_drvdata(pdev);

	gpio_free(rfkill->nshutdown_gpio);

	rfkill_unregister(rfkill->rfkdev);
	rfkill_destroy(rfkill->rfkdev);
	kfree(rfkill);

	return 0;
}


/* List of device names supported by this driver */
static struct platform_device_id bcm_lwb_rfkill_id_table[] = {
	{"bcm-lwb-rfkill", RFKILL_BCM_LWB},
	{},
};
MODULE_DEVICE_TABLE(platform, bcm_lwb_rfkill_id_table);

static struct platform_driver bcm_lwb_rfkill_driver = {
	.probe = bcm_lwb_rfkill_probe,
	.remove = __devexit_p(bcm_lwb_rfkill_remove),
	.driver = {
		.name = "bcm-lwb-rfkill",
		.owner = THIS_MODULE,
	},
	.id_table = bcm_lwb_rfkill_id_table,
};


static int __init bcm_lwb_rfkill_init(void)
{
	pr_err("%s\n", __func__);

	return platform_driver_register(&bcm_lwb_rfkill_driver);
}

static void __exit bcm_lwb_rfkill_exit(void)
{
	pr_err("%s\n", __func__);

	platform_driver_unregister(&bcm_lwb_rfkill_driver);
}

module_init(bcm_lwb_rfkill_init);
module_exit(bcm_lwb_rfkill_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sciaps");
MODULE_DESCRIPTION("RFKILL driver for BCM LWB modem");
