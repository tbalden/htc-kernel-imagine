/*
 * Driver for the TPS61099 external power source
 *
 * Copyright (C) 2017 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#define TPS61099_ENABLE_GPIO "tps,pmi_gpio_en"

struct tps61099_power {
	struct device		*dev;
	struct regulator_desc	pswitch_rdesc;
	struct regulator_dev	*pswitch_rdev;

	int 		gpio_tps_en;
	bool		power_enabled;
};

static int tps61099_pswitch_regulator_enable(struct regulator_dev *rdev)
{
	struct tps61099_power *tp = rdev_get_drvdata(rdev);

	gpio_direction_output(tp->gpio_tps_en, 1);
	pr_info("TPS61099 external power source enabled\n");
	tp->power_enabled = true;
	return 0;
}

static int tps61099_pswitch_regulator_disable(struct regulator_dev *rdev)
{
	struct tps61099_power *tp = rdev_get_drvdata(rdev);

	gpio_direction_output(tp->gpio_tps_en, 0);
	pr_info("TPS61099 external power source disabled\n");
	tp->power_enabled = false;
	return 0;
}

static int tps61099_pswitch_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct tps61099_power *tp = rdev_get_drvdata(rdev);

	return tp->power_enabled;
}

static struct regulator_ops tps61099_pswitch_regulator_ops = {
	.enable		= tps61099_pswitch_regulator_enable,
	.disable	= tps61099_pswitch_regulator_disable,
	.is_enabled	= tps61099_pswitch_regulator_is_enabled,
};

static int tps61099_regulator_init(struct tps61099_power *tp) {
	struct device *dev = tp->dev;
	struct regulator_config cfg = {};
	struct regulator_init_data *init_data;
	int ret = 0;

	init_data = devm_kzalloc(dev, sizeof(*init_data), GFP_KERNEL);
	if (!init_data)
		return -ENOMEM;

	init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	tp->pswitch_rdesc.owner = THIS_MODULE;
	tp->pswitch_rdesc.type = REGULATOR_VOLTAGE;
	tp->pswitch_rdesc.ops = &tps61099_pswitch_regulator_ops;
	tp->pswitch_rdesc.name = kbasename(dev->of_node->full_name);

	cfg.dev = dev;
	cfg.init_data = init_data;
	cfg.driver_data = tp;
	cfg.of_node = dev->of_node;

	tp->pswitch_rdev = devm_regulator_register(dev, &tp->pswitch_rdesc, &cfg);
	if (IS_ERR(tp->pswitch_rdev)) {
		ret = PTR_ERR(tp->pswitch_rdev);
		pr_err("TPS61099 - %s: regulator register failed:%d\n", __func__, ret);
	}

	return 0;
}

static int htc_tps61099_probe(struct platform_device *pdev)
{
	struct tps61099_power *tp;
	struct device_node *node = pdev->dev.of_node;
	int gpio_en;

	tp = devm_kzalloc(&pdev->dev, sizeof(struct tps61099_power), GFP_KERNEL);
	if (!tp)
		return -ENOMEM;

	tp->dev = &pdev->dev;

	/* Get our device tree node */
	gpio_en = of_get_named_gpio(node, TPS61099_ENABLE_GPIO, 0);
	gpio_direction_output(gpio_en, 0);

	tp->gpio_tps_en = gpio_en;
	tp->power_enabled = false;

	tps61099_regulator_init(tp);

	platform_set_drvdata(pdev, tp);
	pr_info("TPS61099 %s - probe TPS61099 driver\n", __func__);
	return 0;
}

static int htc_tps61099_remove(struct platform_device *pdev)
{
	pr_info("TPS61099 %s - remove TPS61099 driver\n", __func__);
	return 0;
}

static void htc_tps61099_shutdown(struct platform_device *pdev)
{
	struct tps61099_power *tp = platform_get_drvdata(pdev);

	if (!tp) {
		pr_err("%s, no driver data\n", __func__);
		return;
	}
	if (tp->power_enabled) {
		gpio_direction_output(tp->gpio_tps_en, 0);
		tp->power_enabled = false;
	}
	pr_info("TPS61099 %s - clear setting during device shutdown\n", __func__);
	return;
}

static const struct of_device_id tps61099_dt_match_table[] = {
	{.compatible = "htc,tps61099" },
	{},
};
MODULE_DEVICE_TABLE(of, tps61099_dt_match_table);

static struct platform_driver htc_tps61099_driver = {
	.driver = {
		.name		= "htc,tps61099",
		.owner		= THIS_MODULE,
		.of_match_table = tps61099_dt_match_table,
	},
	.probe		= htc_tps61099_probe,
	.remove		= htc_tps61099_remove,
	.shutdown	= htc_tps61099_shutdown,
};

static int __init htc_tps61099_init(void)
{
	return platform_driver_register(&htc_tps61099_driver);
}

static void __exit htc_tps61099_exit(void)
{
	platform_driver_unregister(&htc_tps61099_driver);
}

MODULE_DESCRIPTION("TPS61099 external power source");
MODULE_LICENSE("GPL");

module_init(htc_tps61099_init);
module_exit(htc_tps61099_exit);
