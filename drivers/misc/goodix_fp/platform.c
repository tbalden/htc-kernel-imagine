/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif
#include <linux/device.h>

#ifdef CONFIG_HTC_PCN_FP_ENABLE_HW_RESET_PINCTRL
#define GOODIX_RESET_LOW   "goodix_reset_low"
#define GOODIX_RESET_HIGH  "goodix_reset_high"
struct pinctrl *goodix_pinctrl;
struct pinctrl_state *goodix_pinctrl_state[2];

int htc_hw_reset_init(struct device *device)
{
    int ret = 0;

    // 1. Get pinctrl
    goodix_pinctrl = devm_pinctrl_get(device);
    if(IS_ERR(goodix_pinctrl)) {
        pr_err("%s : fail to get pinctrl!\n",__func__);
        ret = -1;
        goto err;
    }

    // 2. Get pinctrl_state
    goodix_pinctrl_state[0] = pinctrl_lookup_state(goodix_pinctrl,GOODIX_RESET_LOW);
    goodix_pinctrl_state[1] = pinctrl_lookup_state(goodix_pinctrl,GOODIX_RESET_HIGH);
    if (IS_ERR(goodix_pinctrl_state[0]) || IS_ERR(goodix_pinctrl_state[1])) {
        pr_err("%s : fail to get pinctrl_state!\n",__func__);
        ret = -2;
        goto err;        
    } 

    // 3. Per HW ask, Set reset_pin as low right after Enabling 3V3 power
    ret = pinctrl_select_state(goodix_pinctrl, goodix_pinctrl_state[0]);
    if (ret) {
        ret = -3;
        pr_err("%s : fail to set %s\n",__func__,GOODIX_RESET_LOW);
        goto err;
    }

err:
    return ret;
}
#endif //CONFIG_HTC_PCN_FP_ENABLE_HW_RESET_PINCTRL

#ifdef CONFIG_HTC_PCN_FP_ENABLE_3V3_POWER
int enable_goodix_power(struct gf_dev *gf_dev)
{
    int ret = 0;
    struct device *dev = &gf_dev->spi->dev;
    struct device_node *np = dev->of_node;
    gf_dev->pwr_gpio = of_get_named_gpio(np, "fp,3v3_gpio_en", 0);
	if (gf_dev->pwr_gpio < 0) {
		pr_err("falied to get power gpio!\n");
		return -1;
	}

	ret = devm_gpio_request(dev, gf_dev->pwr_gpio, "goodix_3v3_power");
	if (ret) {
		pr_err("failed to request pwr_gpio, rc = %d\n", ret);
		return -2;
	}
	gpio_direction_output(gf_dev->pwr_gpio, 1);

    pr_info("%s : Enable 3V3 power for goodix successfully!\n",__func__);
    return 0;

}
#endif //CONFIG_HTC_PCN_FP_ENABLE_3V3_POWER//


int gf_parse_dts(struct gf_dev* gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}

    // Duration between power & reset must >= 10 ms.
    // Do not do control reset pin here, do hw_reset later.
	// gpio_direction_output(gf_dev->reset_gpio, 0);

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);

err_irq:
	devm_gpio_free(dev, gf_dev->reset_gpio);
err_reset:
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n",__func__);
	if (gpio_is_valid(gf_dev->irq_gpio))
	{
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio))
	{
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev* gf_dev)
{
	int rc = 0;

	msleep(10);
	pr_info("---- power on ok ----\n");

	return rc;
}

int gf_power_off(struct gf_dev* gf_dev)
{
	int rc = 0;

	pr_info("---- power off ----\n");
	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
#ifdef CONFIG_HTC_PCN_FP_ENABLE_HW_RESET_PINCTRL
    int ret = 0;
    ret = pinctrl_select_state(goodix_pinctrl, goodix_pinctrl_state[0]);
    if (ret) {
        pr_err("%s : fail to set %s\n",__func__,GOODIX_RESET_LOW);
        goto err;
    }

    mdelay(3);

    ret = pinctrl_select_state(goodix_pinctrl, goodix_pinctrl_state[1]);
    if (ret) {
        pr_err("%s : fail to set %s\n",__func__,GOODIX_RESET_HIGH);
        goto err;
    }

err:
    return ret;
#else
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
#endif //CONFIG_HTC_PCN_FP_ENABLE_HW_RESET_PINCTRL
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

