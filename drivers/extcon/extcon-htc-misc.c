/**
 * extcon-htc-misc.c - HTC MISC virtual extcon connector
 *
 * Copyright (C) 2017 HTC Corp, Ltd.
 * JJ Lee <jj_lee@htc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/extcon.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

static struct extcon_dev *htc_misc_extcon = NULL;

static const unsigned int htc_misc_supported_cable[] = {
	EXTCON_AUDIO_BEATS,
	EXTCON_AUDIO_DQ,
	EXTCON_AUDIO_FM,
	EXTCON_AUDIO_UNSUPPORTED,
	EXTCON_NONE,
};

static bool htc_misc_extcon_ready(void)
{
	if (!htc_misc_extcon)
		return false;
	return true;
}

int htc_misc_extcon_set_state(unsigned int id, bool state)
{
	int i;

	if (!htc_misc_extcon_ready())
		return -ENOENT;

	/* Validate id */
	for (i = 0 ; i < ARRAY_SIZE(htc_misc_supported_cable) - 1 ; i++)
		if (htc_misc_supported_cable[i] == id) {
			if (htc_misc_supported_cable[i] == EXTCON_AUDIO_BEATS) {
				extcon_set_state(htc_misc_extcon, id, state);
				return extcon_sync(htc_misc_extcon, id);
			} else
				return extcon_set_state_sync(htc_misc_extcon, id, state);
		}

	return -EINVAL;
}
EXPORT_SYMBOL(htc_misc_extcon_set_state);

static int htc_misc_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct extcon_dev *edev;
	int ret;

	pr_info("%s\n", __func__);

        /* Initialize extcon device */
        edev = devm_extcon_dev_allocate(&pdev->dev,
			htc_misc_supported_cable);
        if (IS_ERR(edev)) {
                dev_err(&pdev->dev, "failed to allocate memory for extcon\n");
                return PTR_ERR(edev);
        }

	ret = devm_extcon_dev_register(dev, edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}
	platform_set_drvdata(pdev, edev);

	htc_misc_extcon = edev;

	return 0;
}

static int htc_misc_extcon_remove(struct platform_device *pdev)
{
	htc_misc_extcon = NULL;

	return 0;
}

static const struct of_device_id htc_misc_extcon_dt_match[] = {
	{ .compatible = "htc-misc-extcon", },
	{ }
};
MODULE_DEVICE_TABLE(of, htc_misc_extcon_dt_match);

static struct platform_driver htc_misc_extcon_driver = {
	.probe		= htc_misc_extcon_probe,
	.remove		= htc_misc_extcon_remove,
	.driver		= {
		.name	= "htc-misc-extcon",
		.of_match_table = htc_misc_extcon_dt_match,
	},
};
module_platform_driver(htc_misc_extcon_driver);


MODULE_DESCRIPTION("HTC MISC virtual extcon connector driver");
MODULE_AUTHOR("JJ Lee <jj_lee@htc.com>");
MODULE_LICENSE("GPL v2");
