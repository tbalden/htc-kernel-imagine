/*
 * Driver for the TUSB544 USB3.0 re-drive
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/workqueue.h>

#include "tusb544.h"

#define TUSB544_REDRIVER_ID	"tusb544,redriver_id"
#define TUSB544_POWER_GPIO	"tusb544,v_3v3_en"
#define TUSB544_1V8_GPIO	"tusb544,v_1v8_en"
#define TUSB544_VDD_GPIO	"tusb544,v_vdd_en"

#define DELAY_POWER_SEQUENCE 400

//extern bool get_speed_lock(void);//[FIX ME?] Do we need to lock speed
static const char * const cc_state_name[] = {
	[CC_STATE_USB3]		= "USB3",
	[CC_STATE_DP]		= "Display Port",
	[CC_STATE_OPEN]		= "Open",
};

static const char * const cc_orientation_name[] = {
	[CC_ORIENTATION_NONE]	= "None",
	[CC_ORIENTATION_CC1]	= "CC1",
	[CC_ORIENTATION_CC2]	= "CC2",
};

struct tusb544_chip {
	struct device		*device;
	struct i2c_client	*i2c_client;

	int gpio_3v3_power;
	int gpio_1v8;
	int gpio_vdd;

	struct work_struct	update_work;
	struct workqueue_struct	*tusb_wq;
	struct mutex		state_lock;

	bool			usb3_disable;
	bool			probe_done;
	//bool			vdd_state;
	enum cc_state		cc_state;
	enum cc_orientation	cc_orientation;
	//atomic_t		pm_suspended;
};

struct tusb544_chip *tusb_chip = NULL;
static void tusb544_usb3_ui_update_state(void);

void set_redriver_status(void)
{
	struct tusb544_chip *chip = tusb_chip;
//	bool temp = get_speed_lock();
	bool temp = false;//[FIX ME?] Do we need to lock speed


	if (chip->usb3_disable == temp)
		return;

	chip->usb3_disable = temp;

	tusb544_usb3_ui_update_state();
	return;
}

static int tusb544_3v3_power_switch(struct tusb544_chip *chip, bool enabled)
{
	int val = -1;

	if (!chip)
	{
		pr_err("TUSB544 %s - Error: Chip structure is NULL!\n",
					__func__);
		return -ENODEV;
	}

	gpio_direction_output(chip->gpio_3v3_power, enabled ? 1 : 0);
	val = gpio_get_value(chip->gpio_3v3_power);
	dev_err(chip->device, "TUSB544 %s - power %s V_LED_3v3, val:%d\n",
				__func__, enabled ? "on" : "off", val);
	return val;
}


static int tusb544_1v8_switch(struct tusb544_chip *chip, bool enabled)
{
	int val = -1;

	if (!chip)
	{
		pr_err("TUSB544 %s - Error: Chip structure is NULL!\n",
					__func__);
		return -ENODEV;
	}

	gpio_direction_output(chip->gpio_1v8, enabled ? 1 : 0);
	val = gpio_get_value(chip->gpio_1v8);
	dev_err(chip->device, "TUSB544 %s - switch 1v8 to %s state, val:%d\n",
			 __func__, enabled ? "active" : "sleep", val);
	return 0;
}

static int tusb544_vdd_switch(struct tusb544_chip *chip, bool enabled)
{
	int val = -1;

	if (!chip)
	{
		pr_err("TUSB544 %s - Error: Chip structure is NULL!\n",
					__func__);
		return -ENODEV;
	}

	gpio_direction_output(chip->gpio_vdd, enabled ? 1 : 0);
	val = gpio_get_value(chip->gpio_vdd);
	dev_err(chip->device, "TUSB544 %s - switch vdd to %s state, val:%d\n",
			 __func__, enabled ? "active" : "sleep", val);
	return 0;
}

static int tusb544_aux_snoop(struct tusb544_chip *chip, bool enabled, u8 pin)
{
	int ret = 0;
	u16 aux_ctrl_value = (3 != pin) ? DISABLE_AUX_SNOOP_C : DISABLE_AUX_SNOOP_D;

	if (enabled) {
		aux_ctrl_value = ENABLE_AUX_SNOOP;
	} else {
		if (chip->cc_orientation == CC_ORIENTATION_CC1) {
			aux_ctrl_value |= AUX_SBU_CC1;
		} else if (chip->cc_orientation == CC_ORIENTATION_CC2) {
			aux_ctrl_value |= AUX_SBU_CC2;
		}
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_AUX, aux_ctrl_value);
	if (ret < 0) {
		dev_err(chip->device,
			"TUSB544 %s - cannot %s aux snoop, ret=%d\n",
			__func__, enabled ? "enable" : "disable", ret);
		return ret;
	}

	dev_err(chip->device, "TUSB544 %s - %s aux snoop, aux_ctrl_value = 0x%x\n",
		 __func__, enabled ? "enable" : "disable", aux_ctrl_value);
	return ret;
}

static void tusb544_dp_update_state(u8 pin)
{
	struct tusb544_chip *chip = tusb_chip;
	int ret = 0;
	bool switch_1v8_enabled = true;
	bool aux_snoop_enabled = false;
	u8 config_ctrl_value = 0;

	if (chip->cc_orientation == CC_ORIENTATION_CC1)
		config_ctrl_value = (3 != pin) ? DP_ON_CC1_C : DP_ON_CC1_D;
	else if (chip->cc_orientation == CC_ORIENTATION_CC2)
		config_ctrl_value = (3 != pin) ? DP_ON_CC2_C : DP_ON_CC2_D;

	ret = tusb544_1v8_switch(chip, switch_1v8_enabled);
	if (ret < 0)
		return;

	ret = tusb544_aux_snoop(chip, aux_snoop_enabled, pin);
	if (ret < 0)
		return;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_CONFIG_CTRL,
					config_ctrl_value | EQ_OVERRIDE);
	if (ret < 0) {
		dev_err(chip->device, "TUSB544 %s - i2c write data failed\n", __func__);
		return;
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX1, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x21) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX2, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x20) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX1, 0x88);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX1(0x11) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX2, 0x88);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX2(0x10) failed\n", __func__);
	}

	dev_err(chip->device,
		 "TUSB544 %s - state = %s, orientation = %s, switch = %s, aux_snoop = %s, config_ctrl = 0x%x\n",
		 __func__, cc_state_name[chip->cc_state],
		 cc_orientation_name[chip->cc_orientation],
		 switch_1v8_enabled ? "enabled" : "disabled",
		 aux_snoop_enabled ? "enabled" : "disabled",
		 config_ctrl_value);
	return;
}

static void tusb544_usb3_ui_update_state(void)
{
	struct tusb544_chip *chip = tusb_chip;
	int ret = 0;
	bool vdd_enabled;
	bool switch_1v8_enabled;
	bool aux_snoop_enabled = true;
	u8 config_ctrl_value = 0;

	switch (chip->cc_state) {
	case CC_STATE_USB3:
		if (!chip->usb3_disable) {
			vdd_enabled = true;
			switch_1v8_enabled = true;
			if (chip->cc_orientation == CC_ORIENTATION_CC1)
				config_ctrl_value = USB3_ON_CC1;
			else if (chip->cc_orientation == CC_ORIENTATION_CC2)
				config_ctrl_value = USB3_ON_CC2;
		} else {
			vdd_enabled = false;
			switch_1v8_enabled = false;
			config_ctrl_value = DISABLE_TX_RX;
		}
		break;
	case CC_STATE_OPEN:
	default:
		vdd_enabled = false;
		switch_1v8_enabled = false;
		config_ctrl_value = DISABLE_TX_RX;
		break;
	}

	// power on the chip if cable plugging
	if (vdd_enabled) {
		ret = tusb544_vdd_switch(chip, vdd_enabled);
		if (ret < 0) {
			dev_err(chip->device, "TUSB544 %s - power %s chip failed, ret = %d\n",
				 __func__, vdd_enabled ? "on" : "off", ret);
			return;
		}
	}

	ret = tusb544_1v8_switch(chip, switch_1v8_enabled);
	if (ret < 0)
		return;

	if (vdd_enabled && DELAY_POWER_SEQUENCE > 0)
		msleep(DELAY_POWER_SEQUENCE);

	ret = tusb544_aux_snoop(chip, aux_snoop_enabled, 0);
	if (ret < 0)
		return;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_CONFIG_CTRL,
					config_ctrl_value | EQ_OVERRIDE);
	if (ret < 0) {
		dev_err(chip->device, "TUSB544 %s - i2c write data failed\n", __func__);
		return;
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX1, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x21) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX2, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x20) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX1, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX1(0x11) failed\n", __func__);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX2, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX2(0x10) failed\n", __func__);
	}

	// power off the chip if cable unplugging
	if (!vdd_enabled) {
		ret = tusb544_vdd_switch(chip, vdd_enabled);
		if (ret < 0) {
			dev_err(chip->device, "TUSB544 %s - power %s chip failed, ret = %d\n",
				 __func__, vdd_enabled ? "on" : "off", ret);
		}
	}

	dev_err(chip->device,
		 "TUSB544 %s - state = %s, orientation = %s, switch = %s, aux_snoop = %s, config_ctrl = 0x%x\n",
		 __func__, cc_state_name[chip->cc_state],
		 cc_orientation_name[chip->cc_orientation],
		 switch_1v8_enabled ? "enabled" : "disabled",
		 aux_snoop_enabled ? "enabled" : "disabled",
		 config_ctrl_value);
	return;
}

static void tusb544_usb3_update_state(struct work_struct *w)
{
	struct tusb544_chip *chip = container_of(w, struct tusb544_chip, update_work);
	int ret = 0;
	bool vdd_enabled;
	bool switch_1v8_enabled;
	bool aux_snoop_enabled = true;
	u8 config_ctrl_value = 0;

	switch (chip->cc_state) {
	case CC_STATE_USB3:
		if (!chip->usb3_disable) {
			vdd_enabled = true;
			switch_1v8_enabled = true;
			if (chip->cc_orientation == CC_ORIENTATION_CC1)
				config_ctrl_value = USB3_ON_CC1;
			else if (chip->cc_orientation == CC_ORIENTATION_CC2)
				config_ctrl_value = USB3_ON_CC2;
		} else {
			vdd_enabled = false;
			switch_1v8_enabled = false;
			config_ctrl_value = DISABLE_TX_RX;
		}
		break;
	case CC_STATE_OPEN:
	default:
		vdd_enabled = false;
		switch_1v8_enabled = false;
		config_ctrl_value = DISABLE_TX_RX;
		break;
	}

	// power on the chip if cable plugging
	if (vdd_enabled) {
		ret = tusb544_vdd_switch(chip, vdd_enabled);
		if (ret < 0) {
			dev_err(chip->device, "TUSB544 %s - power %s chip failed, ret = %d\n",
				 __func__, vdd_enabled ? "on" : "off", ret);
			return;
		}
	}

	ret = tusb544_1v8_switch(chip, switch_1v8_enabled);
	if (ret < 0)
		return;

	if (vdd_enabled && DELAY_POWER_SEQUENCE > 0)
		msleep(DELAY_POWER_SEQUENCE);

	ret = tusb544_aux_snoop(chip, aux_snoop_enabled, 0);
	if (ret < 0)
		return;

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_CONFIG_CTRL,
					config_ctrl_value | EQ_OVERRIDE);
	if (ret < 0) {
		dev_err(chip->device, "TUSB544 %s - i2c write data failed, ret=%d\n", __func__, ret);
		return;
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX1, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x21) failed, , ret=%d\n", __func__, ret);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_USB3_TX2, 0x7);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write USB3 TX2(0x20) failed, ret=%d\n", __func__, ret);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX1, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX1(0x11) failed, ret=%d\n", __func__, ret);
	}

	ret = i2c_smbus_write_byte_data(chip->i2c_client, REG_DP_TX2, 0x70);
	if (ret < 0) {
		dev_info(chip->device, "%s - i2c write TX2(0x10) failed, ret=%d\n", __func__, ret);
	}

	// power off the chip if cable unplugging
	if (!vdd_enabled) {
		ret = tusb544_vdd_switch(chip, vdd_enabled);
		if (ret < 0) {
			dev_err(chip->device, "TUSB544 %s - power %s chip failed, ret = %d\n",
				 __func__, vdd_enabled ? "on" : "off", ret);
		}
	}

	dev_err(chip->device,
		 "TUSB544 %s - state = %s, orientation = %s, switch = %s, aux_snoop = %s, config_ctrl = 0x%x\n",
		 __func__, cc_state_name[chip->cc_state],
		 cc_orientation_name[chip->cc_orientation],
		 switch_1v8_enabled ? "enabled" : "disabled",
		 aux_snoop_enabled ? "enabled" : "disabled",
		 config_ctrl_value);
	return;
}

void tusb544_update_state(enum cc_state cc_state, enum cc_orientation cc_orientation, u8 pin)
{
	struct tusb544_chip *chip = tusb_chip;

	if (IS_ERR_OR_NULL(chip)) {
		pr_err("%s, no dev\n", __func__);
		return;
	}

	mutex_lock(&chip->state_lock);

	chip->cc_state = cc_state;
	chip->cc_orientation = cc_orientation;

	if (!chip->probe_done) {
		queue_work(chip->tusb_wq, &chip->update_work);
	} else if (cc_state != CC_STATE_DP) {
		tusb544_usb3_ui_update_state();
	} else {
		tusb544_dp_update_state(pin);
	}

	mutex_unlock(&chip->state_lock);
	return;
}

static int tusb544_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	int ret = 0;
	struct tusb544_chip *chip;
	struct device_node *node = client->dev.of_node;
	int val = -1;
	int gpio_redriver_id;
	int gpio_3v3_power;
	int gpio_1v8;
	int gpio_vdd;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
			"TUSB544 %s - i2c_check_functionality error\n",
			__func__);
		return -EIO;
	}

	/* Check if it had redriver component */
	gpio_redriver_id = of_get_named_gpio(node, TUSB544_REDRIVER_ID, 0);
	val = gpio_get_value(gpio_redriver_id);
	if (val == 0) {
		dev_info(&client->dev, "TUSB544 %s - It has redriver component\n",
			__func__);
		/* pull down redriver_id once we confirm the component status */
		gpio_set_value(gpio_redriver_id, 0);
	} else {
		dev_info(&client->dev, "TUSB544 %s - It doesn't have redriver component"
			", so just return the probe\n",
			__func__);
		/* pull down redriver_id once we confirm the component status */
		gpio_set_value(gpio_redriver_id, 0);
		return ret;
	}

	/* Get our device tree node */
	gpio_3v3_power = of_get_named_gpio(node, TUSB544_POWER_GPIO, 0);
	gpio_1v8 = of_get_named_gpio(node, TUSB544_1V8_GPIO, 0);
	gpio_vdd = of_get_named_gpio(node, TUSB544_VDD_GPIO, 0);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->probe_done = false;
	chip->device = &client->dev;
	chip->i2c_client = client;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->state_lock);
	chip->gpio_3v3_power = gpio_3v3_power;
	chip->gpio_1v8 = gpio_1v8;
	chip->gpio_vdd = gpio_vdd;

	INIT_WORK(&chip->update_work, tusb544_usb3_update_state);
	chip->tusb_wq = alloc_ordered_workqueue("tusb_wq", 0);
//	chip->usb3_disable = get_speed_lock();
	chip->usb3_disable = false;//we don't need to lock speed now

	tusb_chip = chip;

	/* Enable V_LED_3V3, it's power source for TUSB544 */
	ret = tusb544_3v3_power_switch(chip, true);
	if (ret < 1) {
		dev_err(&client->dev,
			"TUSB544 %s - Error: V_3V3 can't be powered:%d\n", __func__, ret);
	}

	/* Pull up power enable pin to enable TUSB544 */
	ret = tusb544_vdd_switch(chip, true);
	if (ret < 0) {
		dev_err(&client->dev,
			"TUSB544 %s - Error: TUSB544 can't be powered:%d\n", __func__, ret);
		return ret;
	}

	tusb544_update_state(CC_STATE_OPEN, CC_ORIENTATION_NONE, 0);

	chip->probe_done = true;

	dev_err(&client->dev,
		 "TUSB544 %s - probing TUSB544 i2c driver done\n", __func__);

	return ret;
}

static int tusb544_i2c_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "TUSB544 %s - remove TUSB544 i2c driver\n",
		 __func__);
	return 0;
}

static const struct of_device_id tusb544_i2c_match_table[] = {
	{.compatible = "ti,tusb544-i2c" },
	{},
};

static const struct i2c_device_id tusb544_i2c_id[] = {
	{ "tusb544-i2c", 0 },
	{ }
};

static struct i2c_driver tusb544_i2c_driver = {
	.driver = {
		.name		= "tusb544-i2c",
		.owner		= THIS_MODULE,
		.of_match_table = tusb544_i2c_match_table,
	},
	.probe		= tusb544_i2c_probe,
	.remove		= tusb544_i2c_remove,
	.id_table	= tusb544_i2c_id,
};
module_i2c_driver(tusb544_i2c_driver);

