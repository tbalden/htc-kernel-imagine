/*****************************************************************************
* File: device.c
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>

#include "device.h"
#include "utils.h"
#include "hardware.h"
#include "config.h"
#include "debug.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
enum EDGE_KEY_ID
{
    Flat_Key   = 0,
    Convex_Key = 1,
    New_Key    = 2,
    New_Convex_Key    = 3,
    Vd_Shift_Key      = 4,
    Weak_Vd_Shift_Key = 5,
};
/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBALS                                                                  */
/*==========================================================================*/
struct snt8100fsr *snt8100fsr_g; // The first and main device and sysFS device
#ifdef CONFIG_NANOHUB_SILEGO
struct snt8100fsr *snt8100fsr_spi_g;
#endif
struct snt8100fsr *snt8100fsr_wake_i2c_g; // The i2c wakeup device

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
#if USE_DEVICE_TREE_CONFIG
static int snt_request_named_gpio(struct snt8100fsr *snt8100fsr,
                                  const char *label,
                                  int *gpio);
#ifdef CONFIG_HTC_FEATURE
int select_pin_ctl(struct snt8100fsr *snt8100fsr,
                          const char *name);
#else
static int select_pin_ctl(struct snt8100fsr *snt8100fsr,
                          const char *name);
#endif
#endif

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
/*
 * Device initialization using device tree node configuration
 */
#if USE_DEVICE_TREE_CONFIG // config.h =======================================
#ifdef CONFIG_HTC_FEATURE
int snt_power_init(struct snt8100fsr *snt8100fsr) {
    int ret;
#if 0
    ret = snt_request_named_gpio(snt8100fsr, "snt,gpio_1v2_en",
            &snt8100fsr->snt1v2_en_gpio);
    if(ret) {
        PRINT_ERR("Request snt1v2_en_gpio failed");
        return 0;
    }
#endif
    ret = snt_request_named_gpio(snt8100fsr, "snt,gpio_2v8_en",
            &snt8100fsr->snt2v8_en_gpio);
    if(ret) {
        PRINT_ERR("Request snt2v8_en_gpio failed");
        return 0;
    }
#if 0
    ret = gpio_direction_output(snt8100fsr->snt1v2_en_gpio, 1 );

    if(ret) {
        PRINT_ERR("Set snt1v2_en_gpio as output failed");
        return 0;
    }
#endif
    ret = gpio_get_value(snt8100fsr->snt2v8_en_gpio);
    PRINT_INFO("GPIO 2v8 = %d", ret);

    ret = gpio_direction_output(snt8100fsr->snt2v8_en_gpio, 1);
    if(ret) {
        PRINT_ERR("Set snt2v8_en_gpio as output failed");
        return 0;
    }
#if 0
    gpio_set_value(snt8100fsr->snt1v2_en_gpio, 0);
    gpio_set_value(snt8100fsr->snt1v2_en_gpio, 0);
    ret = gpio_get_value(snt8100fsr->snt1v2_en_gpio);
    PRINT_INFO("GPIO 1v2 = %d", ret);
    ret = gpio_get_value(snt8100fsr->snt2v8_en_gpio);
    PRINT_INFO("GPIO 2v8 = %d", ret);
#endif
    msleep(20);
#if 0
    gpio_set_value(snt8100fsr->snt1v2_en_gpio, 1);
    gpio_set_value(snt8100fsr->snt1v2_en_gpio, 1);
    ret = gpio_get_value(snt8100fsr->snt1v2_en_gpio);
    PRINT_INFO("GPIO 1v2 = %d", ret);
#endif
    ret = gpio_get_value(snt8100fsr->snt2v8_en_gpio);
    PRINT_INFO("GPIO 2v8 = %d", ret);

    return 1;
}
#endif
#ifdef CONFIG_NANOHUB_SILEGO
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr)
{
    int ret = 0, data;
    struct device *dev = &spi->dev;
    struct device_node *np = dev->of_node;

    // Check device tree
    if (!np) {
        PRINT_ERR("no of node found");
        return -EINVAL;
    }

    ret = of_property_read_u32(np, "snt,spi-freq-khz", &data);
    if(ret < 0) {
        dev_err(dev, "snt,spi-freq-khz not found\n");
    } else {
        snt8100fsr->spi_freq_khz = data;
    }
    dev_info(dev, "spi_freq_khz %d\n", snt8100fsr->spi_freq_khz);

    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    return ret;
}
#else
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr)
{
    int ret = 0, data, i;
    struct device *dev = &spi->dev;
    struct device_node *np = dev->of_node;

    // Check device tree
    if (!np) {
        PRINT_ERR("no of node found");
        return -EINVAL;
    }

    // Request gpio
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_hostirq",
                           &snt8100fsr->hostirq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_wakeirq",
                           &snt8100fsr->wakeirq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_rst",
                           &snt8100fsr->rst_gpio);

    PRINT_INFO("GPIO IRQ=%d, RST=%d",
		    gpio_get_value(snt8100fsr->hostirq_gpio),
		    gpio_get_value(snt8100fsr->rst_gpio));
#ifdef CONFIG_HTC_FEATURE
    if(gpio_is_valid(snt8100fsr->hostirq_gpio)) {
        IRQ_GPIO = snt8100fsr->hostirq_gpio;
        snt8100fsr->hostirq = IRQ_NUMBER = gpio_to_irq(snt8100fsr->hostirq_gpio);
    }
#endif

    ret = of_property_read_u32(np, "snt,spi-freq-khz", &data);
    if(ret < 0) {
        dev_err(dev, "snt,spi-freq-khz not found\n");
    } else {
        snt8100fsr->spi_freq_khz = data;
    }
    dev_info(dev, "spi_freq_khz %d\n", snt8100fsr->spi_freq_khz);

    // Request pinctrl
    snt8100fsr->snt_pinctrl = devm_pinctrl_get(dev);
    if(IS_ERR(snt8100fsr->snt_pinctrl)) {
        if(PTR_ERR(snt8100fsr->snt_pinctrl) == -EPROBE_DEFER)
        {
            dev_info(dev, "pinctrl not ready\n");
            return -EPROBE_DEFER;
        }
        dev_err(dev, "Target does not use pinctrl\n");
        snt8100fsr->snt_pinctrl = NULL;
        return  -EINVAL;
    }

    for(i = 0;i < ARRAY_SIZE(snt8100fsr->pinctrl_state);i++) {
        const char *n = pctl_names[i];
        struct pinctrl_state *state =
            pinctrl_lookup_state(snt8100fsr->snt_pinctrl, n);
        if (IS_ERR(state)) {
            dev_err(dev, "cannot find '%s'\n", n);
            return -EINVAL;
        }
        dev_info(dev, "found pin control %s\n", n);
        snt8100fsr->pinctrl_state[i] = state;
    }

#ifdef CONFIG_HTC_FEATURE
    ret = select_pin_ctl(snt8100fsr, "bus_active");
    if(ret)
        PRINT_ERR("bus_active pin control fail");

    msleep(10);
#endif

    ret = select_pin_ctl(snt8100fsr, "snt_reset_active");
    if(ret)
        return ret;

    ret = select_pin_ctl(snt8100fsr, "snt_hostirq_default");
    if(ret)
        return ret;

    device_init_wakeup(snt8100fsr->dev, 1);
    mutex_init(&snt8100fsr->track_report_sysfs_lock);
#ifdef USE_NANOHUB_BUS
    mutex_init(&snt8100fsr->sb_lock);
#endif

    //ret =  devm_request_threaded_irq(dev, gpio_to_irq(snt8100fsr->hostirq_gpio),
    //    NULL, fpc1020_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
    //    dev_name(dev), snt8100fsr);

    // Request that the interrupt should be wakeable
    //enable_irq_wake(gpio_to_irq(snt8100fsr->hostirq_gpio));

    //ret = sysfs_create_group(&dev->kobj, &attribute_group);
    //if(ret)
    //    return ret;

    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

#ifdef CONFIG_HTC_FEATURE
    snt_power_init(snt8100fsr);
#else    //Only enable hardware when device ready to flash firmware
    //FIXME Remove if no need
    dev_info(dev, "Enabling hardware\n");
    mutex_lock(&snt8100fsr->sb_lock);
    dev_info(dev, "[EDGE] %s(%d): snt8100sfr_spi_setup enable\n", __func__, __LINE__);
    (void)select_pin_ctl(snt8100fsr, "snt_reset_active");
    usleep_range(100, 900);
    (void)select_pin_ctl(snt8100fsr, "snt_reset_normal");
    usleep_range(100, 100);
    mutex_unlock(&snt8100fsr->sb_lock);
#endif    //CONFIG_HTC_FEATURE

    return ret;
}
#endif
#ifdef CONFIG_HTC_FEATURE
int snt_i2c_device_init(struct i2c_client *i2c,
                        struct snt8100fsr *snt8100fsr) {
    int ret = 0, i;
    struct device *dev = &i2c->dev;

    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_hostirq",
                           &snt8100fsr->hostirq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_wakeirq",
                           &snt8100fsr->wakeirq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_rst",
                           &snt8100fsr->rst_gpio);

#ifdef CONFIG_HTC_FEATURE
    if(gpio_is_valid(snt8100fsr->hostirq_gpio)) {
        IRQ_GPIO = snt8100fsr->hostirq_gpio;
        snt8100fsr->hostirq = IRQ_NUMBER = gpio_to_irq(snt8100fsr->hostirq_gpio);
    }
#endif

    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    // Request pinctrl
    snt8100fsr->snt_pinctrl = devm_pinctrl_get(dev);
    if(IS_ERR(snt8100fsr->snt_pinctrl)) {
        if(PTR_ERR(snt8100fsr->snt_pinctrl) == -EPROBE_DEFER)
        {
            dev_info(dev, "pinctrl not ready\n");
            return -EPROBE_DEFER;
        }
        dev_err(dev, "Target does not use pinctrl\n");
        snt8100fsr->snt_pinctrl = NULL;
        return  -EINVAL;
    }

    for(i = 0;i < ARRAY_SIZE(snt8100fsr->pinctrl_state);i++) {
        const char *n = pctl_names[i];
        struct pinctrl_state *state =
            pinctrl_lookup_state(snt8100fsr->snt_pinctrl, n);
        if (IS_ERR(state)) {
            dev_err(dev, "cannot find '%s'\n", n);
            return -EINVAL;
        }
        dev_info(dev, "found pin control %s\n", n);
        snt8100fsr->pinctrl_state[i] = state;
    }
if (0) {
    ret = select_pin_ctl(snt8100fsr, "snt_reset_active");
    if(ret)
        return ret;
}
    ret = select_pin_ctl(snt8100fsr, "snt_hostirq_default");
    if(ret)
        return ret;

    device_init_wakeup(snt8100fsr->dev, 1);

    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);
if (0) {
    PRINT_INFO("Enabling hardware\n");
    mutex_lock(&snt8100fsr->sb_lock);
    PRINT_INFO("snt8100sfr_i2c_setup enable\n");
    (void)select_pin_ctl(snt8100fsr, "snt_reset_active");
    usleep_range(100, 900);
    (void)select_pin_ctl(snt8100fsr, "snt_reset_normal");
    usleep_range(100, 100);
    mutex_unlock(&snt8100fsr->sb_lock);
}
    return 0;
}

int snt_nanohub_device_init(struct platform_device *pdev,
                        struct snt8100fsr *snt8100fsr) {
    int ret = 0, i;
    struct device *dev = &pdev->dev;

    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_hostirq",
                           &snt8100fsr->hostirq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_wakeirq",
                           &snt8100fsr->wakeirq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_rst",
                           &snt8100fsr->rst_gpio);

#ifdef CONFIG_HTC_FEATURE
    if(gpio_is_valid(snt8100fsr->hostirq_gpio)) {
        IRQ_GPIO = snt8100fsr->hostirq_gpio;
        snt8100fsr->hostirq = IRQ_NUMBER = gpio_to_irq(snt8100fsr->hostirq_gpio);
    }
#endif

    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    // Request pinctrl
    snt8100fsr->snt_pinctrl = devm_pinctrl_get(dev);
    if(IS_ERR(snt8100fsr->snt_pinctrl)) {
        if(PTR_ERR(snt8100fsr->snt_pinctrl) == -EPROBE_DEFER)
        {
            dev_info(dev, "pinctrl not ready\n");
            return -EPROBE_DEFER;
        }
        dev_err(dev, "Target does not use pinctrl\n");
        snt8100fsr->snt_pinctrl = NULL;
        return  -EINVAL;
    }

    for(i = 0;i < ARRAY_SIZE(snt8100fsr->pinctrl_state);i++) {
        const char *n = pctl_names[i];
        struct pinctrl_state *state =
            pinctrl_lookup_state(snt8100fsr->snt_pinctrl, n);
        if (IS_ERR(state)) {
            dev_err(dev, "cannot find '%s'\n", n);
            return -EINVAL;
        }
        dev_info(dev, "found pin control %s\n", n);
        snt8100fsr->pinctrl_state[i] = state;
    }

    ret = select_pin_ctl(snt8100fsr, "snt_reset_normal");
    if(ret)
        return ret;

    ret = select_pin_ctl(snt8100fsr, "snt_wakeirq_default");
    if(ret)
        return ret;

    ret = select_pin_ctl(snt8100fsr, "snt_hostirq_default");
    if(ret)
        return ret;

    device_init_wakeup(snt8100fsr->dev, 1);

    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);
    mutex_init(&snt8100fsr->bypass_read_lock);
    mutex_init(&snt8100fsr->bypass_write_lock);
    return 0;
}
#endif

static int snt_request_named_gpio(struct snt8100fsr *snt8100fsr,
                                  const char *label,
                                  int *gpio)
{
    struct device *dev = snt8100fsr->dev;
    struct device_node *np = dev->of_node;
    int rc = of_get_named_gpio(np, label, 0);
    if (rc < 0) {
        PRINT_ERR("failed to get '%s'", label);
        return rc;
    }
    *gpio = rc;
    rc = devm_gpio_request(dev, *gpio, label);
    if (rc) {
        PRINT_ERR("failed to request gpio %d", *gpio);
        return rc;
    }
    PRINT_DEBUG("%s %d", label, *gpio);
    return 0;
}

#ifdef CONFIG_HTC_FEATURE
int select_pin_ctl(struct snt8100fsr *snt8100fsr, const char *name)
#else
static int select_pin_ctl(struct snt8100fsr *snt8100fsr, const char *name)
#endif
{
    size_t i;
    int ret;
    struct device *dev = snt8100fsr->dev;
    for (i = 0; i < ARRAY_SIZE(snt8100fsr->pinctrl_state); i++) {
        const char *n = pctl_names[i];
        if (!strncmp(n, name, strlen(n))) {
            ret = pinctrl_select_state(snt8100fsr->snt_pinctrl,
                    snt8100fsr->pinctrl_state[i]);
            if (ret)
                dev_err(dev, "cannot select '%s'\n", name);
            else
                dev_info(dev, "Selected '%s'\n", name);
            goto exit;
        }
    }
    ret = -EINVAL;
    dev_err(dev, "%s:'%s' not found\n", __func__, name);

exit:
    return ret;
}
#else // ---------------------------------------------------------------------
/*
 * Device initialization using the CONFIG.H file
 */
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr) {
    struct device *dev = &spi->dev;

    snt8100fsr->hostirq_gpio = BEAGLEBONE_GPIO49;
    snt8100fsr->wakeirq_gpio = 0;
    snt8100fsr->rst_gpio = 0;
    snt8100fsr->spi_freq_khz = SPI_MAX_SPEED_HZ;
    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    dev_info(dev, "spi_freq_khz %d\n", snt8100fsr->spi_freq_khz);

    // Request pinctrl
    snt8100fsr->snt_pinctrl = NULL;

    device_init_wakeup(snt8100fsr->dev, 1);

    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);
    return 0;
}

int snt_i2c_device_init(struct i2c_client *i2c,
                        struct snt8100fsr *snt8100fsr) {
    snt8100fsr->hostirq_gpio = BEAGLEBONE_GPIO49;
    snt8100fsr->wakeirq_gpio = 0;
    snt8100fsr->rst_gpio = 0;
    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    // Request pinctrl
    snt8100fsr->snt_pinctrl = NULL;

    device_init_wakeup(snt8100fsr->dev, 1);

    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);
    return 0;
}

#ifdef CONFIG_HTC_FEATURE
int snt_nanohub_device_init(struct platform_device *pdev,
                        struct snt8100fsr *snt8100fsr) {
    snt8100fsr->hostirq_gpio = BEAGLEBONE_GPIO49;
    snt8100fsr->wakeirq_gpio = 0;
    snt8100fsr->rst_gpio = 0;
    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    // Request pinctrl
    snt8100fsr->snt_pinctrl = NULL;

    device_init_wakeup(snt8100fsr->dev, 1);

    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);
    return 0;
}
#endif
#endif // ====================================================================


static ssize_t snt_spi_test_set(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct snt8100fsr *snt8100fsr = dev_get_drvdata(dev);

    dev_info(&snt8100fsr->spi->dev, "%s\n",__func__);

    return ret? ret:count;
}

static DEVICE_ATTR(snt_spi_test, S_IWUSR, NULL, snt_spi_test_set);

static struct attribute *attributes[] = {
    &dev_attr_snt_spi_test.attr,
    NULL
};

static const struct attribute_group attribute_group = {
    .attrs = attributes,
};

int snt_suspend(struct device *dev)
{
    int ret;

    PRINT_FUNC();
    // We don't mutex lock here, due to write_register locking

    PRINT_DEBUG("Setting frame rate to %d",
                snt8100fsr_g->suspended_frame_rate);
    ret = write_register(snt8100fsr_g,
                         REGISTER_FRAME_RATE,
                         &snt8100fsr_g->suspended_frame_rate);
    if (ret) {
        PRINT_CRIT("write_register(REGISTER_FRAME_RATE) failed");
    }

    PRINT_DEBUG("done");
    return 0;
}

int snt_resume(struct device *dev)
{
    int ret;

    PRINT_FUNC();

    // We mutex lock here since we're calling sb_wake_device which never locks
    mutex_lock(&snt8100fsr_g->sb_lock);

    ret = sb_wake_device(snt8100fsr_g);

    if (ret) {
        PRINT_CRIT("sb_wake_device() failed");
        return ret;
    }

    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done");
    return 0;
}

void snt_get_hw_id(struct platform_device *pdev,
                  struct snt8100fsr *snt8100fsr)
{

    int ret = 0, i = 0, ID_Num = 0;
    struct property *prop;
    u8 data[10] = {0}, bom_id = 255;
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    struct device_node *mfgnode = of_find_node_by_path("/chosen/mfg");
    uint8_t eng_id[4] = {0};

    snt8100fsr->flash_flag = Weak_Vd_Shift_Key;
    snt8100fsr->weak_struct_flag = 1;

    if (mfgnode) {
        ret = of_property_read_u8(mfgnode, "skuid.bom_id", &bom_id);
        if ( ret < 0 ) {
            PRINT_ERR("Failed to get property: bom_id");
            return;
        }
        ret = of_property_read_u8_array(mfgnode, "skuid.engineer_id", eng_id, sizeof(uint32_t)/sizeof(uint8_t));
        if ( ret < 0) {
            PRINT_ERR("Failed to get property: engineer_id");
            return;
        }
        printk(KERN_INFO "[EDGE] BOM_ID = %d, ENG_ID = %d",bom_id, eng_id[0]&0x03);
    } else {
        PRINT_ERR("Failed to find device node");
	return;
    }

    // Check device tree
    if (!np) {
        PRINT_ERR("no of node found");
        return;
    } else {
        prop = of_find_property(np, "VD_SHIFT_KEY_ENG_ID", &ID_Num);
        if (prop) {
            memcpy(data, prop->value, ID_Num);
            for (i =0; i < ID_Num; i++) {
                if ((data[i]&0x0F) == (eng_id[0]&0x03)) {
                    if(eng_id[0]&0x03) {
                        snt8100fsr->weak_struct_flag = 0;
                        snt8100fsr->flash_flag = Vd_Shift_Key;
                    } else {
                        snt8100fsr->flash_flag = Weak_Vd_Shift_Key;
                    }
                    return;
                }
            }
        }
        snt8100fsr->weak_struct_flag = 0;
        prop = of_find_property(np, "NEW_CONVEX_KEY_ENG_ID", &ID_Num);
        if (prop) {
            memcpy(data, prop->value, ID_Num);
            for (i =0; i < ID_Num; i++) {
                if ((data[i]&0x0F) == (eng_id[0]&0x03)) {
                    snt8100fsr->flash_flag = New_Convex_Key;
                }
            }
        }
        prop = of_find_property(np, "NEW_KEY_ID", &ID_Num);
        if (prop) {
            memcpy(data, prop->value, ID_Num);
            for (i =0; i < ID_Num; i++) {
                if (data[i] == bom_id) {
                    snt8100fsr->flash_flag = New_Key;
                    return;
                }
            }
        }
        prop = of_find_property(np, "NEW_KEY_ENG_ID", &ID_Num);
        if (prop) {
            memcpy(data, prop->value, ID_Num);
            for (i =0; i < ID_Num; i++) {
                if (((data[i] >> 4) == bom_id)&&(((data[i]&0x0F) == (eng_id[0]&0x01)))) {
                    snt8100fsr->flash_flag = New_Key;
                    return;
                }
            }
        }
        prop = of_find_property(np, "FLAT_KEY_ID", &ID_Num);
        if (prop) {
            memcpy(data, prop->value, ID_Num);
            for (i =0; i < ID_Num; i++) {
                if (data[i] == bom_id) {
                    snt8100fsr->flash_flag = Flat_Key;
                    return;
                }
            }
        }
        prop = of_find_property(np, "CONVEX_KEY_ID", &ID_Num);
        if (prop) {
            memcpy(data, prop->value, ID_Num);
            for (i =0; i < ID_Num; i++) {
                if (data[i] == bom_id) {
                    snt8100fsr->flash_flag = Convex_Key;
                    return;
                }
            }
        }
        prop = of_find_property(np, "CONVEX_KEY_FLAG", NULL);
        if (prop) {
            PRINT_DEBUG("CONVEX_KEY_FLAG exist");
            snt8100fsr->flash_flag = Convex_Key;
            return;
        }
        prop = of_find_property(np, "NEW_KEY_FLAG", NULL);
        if (prop) {
            PRINT_DEBUG("NEW_KEY_FLAG exist");
            snt8100fsr->flash_flag = New_Key;
            return;
        }
        prop = of_find_property(np, "NEW_CONVEX_FLAG", NULL);
        if (prop) {
            PRINT_DEBUG("NEW_CONVEX_FLAG exist");
            snt8100fsr->flash_flag = New_Convex_Key;
            return;
        }
    }
}
