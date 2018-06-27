/*
 * TI LP855x Backlight Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_data/lp855x.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>

/* HTC ADD */
#include <linux/regmap.h>
#include <linux/htc_flashlight.h>
#include <linux/msm_drm_notify.h>


/* LP8550/1/2/3/6 Registers */
#define LP855X_BRIGHTNESS_CTRL		0x00
#define LP855X_DEVICE_CTRL		0x01
#define LP855X_EEPROM_START		0xA0
#define LP855X_EEPROM_END		0xA7
#define LP8556_EPROM_START		0xA0
#define LP8556_EPROM_END		0xAF

/* LP8555/7 Registers */
#define LP8557_BL_CMD			0x00
#define LP8557_BL_MASK			0x01
#define LP8557_BL_ON			0x01
#define LP8557_BL_OFF			0x00
#define LP8557_BRIGHTNESS_CTRL		0x04
#define LP8557_CONFIG			0x10
#define LP8555_EPROM_START		0x10
#define LP8555_EPROM_END		0x7A
#define LP8557_EPROM_START		0x10
#define LP8557_EPROM_END		0x1E

#define DEFAULT_BL_NAME		"lcd-backlight"
#define MAX_BRIGHTNESS		255

/* HTC ADD */
#define MAX_BRIGHTNESS_12BIT			4095
#define MAX_CURRENT_20MA			0x30
#define MAX_CURRENT_30MA			0x60
#define LP8556_BACKLIGHT_12BIT_LSB_MASK		0xFF
#define LP8556_BACKLIGHT_12BIT_MSB_MASK		0x0F
#define LP8556_BACKLIGHT_12BIT_MSB_SHIFT	8
#define LP8556_MAX_CURRENT_MASK			(BIT(4) | BIT(5) | BIT(6))
#define LP8556_CFG1_REG				0xA1
#define LP8556_FULLBRIGHT_LSB_REG		0x10
#define LP8556_FULLBRIGHT_MSB_REG		0x11
#define LP8556_MAX_REGISTER			0xAF

#define FLASH_MODE_BRIGHTNESS			4095
#define FLASH_MODE_MAX_DURATION_MS		500
#define TORCH_MODE_MAX_DURATION_MS		10000

enum lp855x_brightness_ctrl_mode {
	PWM_BASED = 1,
	REGISTER_BASED,
};

enum lp855x_command_sets {
	POWER_ON_CMD = 0,
	POWER_ON_AOD_CMD,
	ENABLE_AOD_CMD,
	DISABLE_AOD_CMD,
	MAX_BL_CMD,
};

const char* const lp855x_command_keyword[MAX_BL_CMD] = {
	"normal-power-on-cmds",
	"aod-power-on-cmds",
	"aod-enable-cmds",
	"aod-disable-cmds",
};

struct lp855x_regcmd {
	unsigned int address;
	unsigned int parameter;
};

struct lp855x_regcmd_sets{
	int reg_cmds_num;
	struct lp855x_regcmd *reg_cmds;
};

struct lp855x;

/*
 * struct lp855x_device_config
 * @pre_init_device: init device function call before updating the brightness
 * @reg_brightness: register address for brigthenss control
 * @reg_devicectrl: register address for device control
 * @post_init_device: late init device function call
 */
struct lp855x_device_config {
	int (*pre_init_device)(struct lp855x *);
	u8 reg_brightness;
	u8 reg_devicectrl;
	int (*post_init_device)(struct lp855x *);
};

struct lp855x {
	const char *chipname;
	enum lp855x_chip_id chip_id;
	enum lp855x_brightness_ctrl_mode mode;
	struct lp855x_device_config *cfg;
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct lp855x_platform_data *pdata;
	struct pwm_device *pwm;
	struct regulator *supply;	/* regulator for VDD input */
	struct regulator *enable;	/* regulator for EN/VDDIO input */

	/* HTC ADD */
	struct regmap *regmap;
	struct delayed_work flash_work;
	bool flash_enabled;
	struct htc_flashlight_dev flash_dev;
	int torch_brightness;
	int current_blank;
	int next_blank;
	struct notifier_block drm_notifier;
	struct lp855x_regcmd_sets cmd_sets[MAX_BL_CMD];
};

static int lp855x_write_byte(struct lp855x *lp, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(lp->client, reg, data);
}

static int lp855x_update_bit(struct lp855x *lp, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = i2c_smbus_read_byte_data(lp->client, reg);
	if (ret < 0) {
		dev_err(lp->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	}

	tmp = (u8)ret;
	tmp &= ~mask;
	tmp |= data & mask;

	return lp855x_write_byte(lp, reg, tmp);
}

static bool lp855x_is_valid_rom_area(struct lp855x *lp, u8 addr)
{
	u8 start, end;

	switch (lp->chip_id) {
	case LP8550:
	case LP8551:
	case LP8552:
	case LP8553:
		start = LP855X_EEPROM_START;
		end = LP855X_EEPROM_END;
		break;
	case LP8556:
		start = LP8556_EPROM_START;
		end = LP8556_EPROM_END;
		break;
	case LP8555:
		start = LP8555_EPROM_START;
		end = LP8555_EPROM_END;
		break;
	case LP8557:
		start = LP8557_EPROM_START;
		end = LP8557_EPROM_END;
		break;
	default:
		return false;
	}

	return addr >= start && addr <= end;
}

static int lp8557_bl_off(struct lp855x *lp)
{
	/* BL_ON = 0 before updating EPROM settings */
	return lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK,
				LP8557_BL_OFF);
}

static int lp8557_bl_on(struct lp855x *lp)
{
	/* BL_ON = 1 after updating EPROM settings */
	return lp855x_update_bit(lp, LP8557_BL_CMD, LP8557_BL_MASK,
				LP8557_BL_ON);
}

static struct lp855x_device_config lp855x_dev_cfg = {
	.reg_brightness = LP855X_BRIGHTNESS_CTRL,
	.reg_devicectrl = LP855X_DEVICE_CTRL,
};

static struct lp855x_device_config lp8557_dev_cfg = {
	.reg_brightness = LP8557_BRIGHTNESS_CTRL,
	.reg_devicectrl = LP8557_CONFIG,
	.pre_init_device = lp8557_bl_off,
	.post_init_device = lp8557_bl_on,
};

/*
 * Device specific configuration flow
 *
 *    a) pre_init_device(optional)
 *    b) update the brightness register
 *    c) update device control register
 *    d) update ROM area(optional)
 *    e) post_init_device(optional)
 *
 */
static int lp855x_configure(struct lp855x *lp)
{
	u8 val, addr;
	int i, ret;
	struct lp855x_platform_data *pd = lp->pdata;

	switch (lp->chip_id) {
	case LP8550:
	case LP8551:
	case LP8552:
	case LP8553:
	case LP8556:
		lp->cfg = &lp855x_dev_cfg;
		break;
	case LP8555:
	case LP8557:
		lp->cfg = &lp8557_dev_cfg;
		break;
	default:
		return -EINVAL;
	}

	if (lp->cfg->pre_init_device) {
		ret = lp->cfg->pre_init_device(lp);
		if (ret) {
			dev_err(lp->dev, "pre init device err: %d\n", ret);
			goto err;
		}
	}

	val = pd->initial_brightness;
	ret = lp855x_write_byte(lp, lp->cfg->reg_brightness, val);
	if (ret)
		goto err;

	val = pd->device_control;
	ret = lp855x_write_byte(lp, lp->cfg->reg_devicectrl, val);
	if (ret)
		goto err;

	if (pd->size_program > 0) {
		for (i = 0; i < pd->size_program; i++) {
			addr = pd->rom_data[i].addr;
			val = pd->rom_data[i].val;
			if (!lp855x_is_valid_rom_area(lp, addr))
				continue;

			ret = lp855x_write_byte(lp, addr, val);
			if (ret)
				goto err;
		}
	}

	if (lp->cfg->post_init_device) {
		ret = lp->cfg->post_init_device(lp);
		if (ret) {
			dev_err(lp->dev, "post init device err: %d\n", ret);
			goto err;
		}
	}

	return 0;

err:
	return ret;
}

static void lp855x_pwm_ctrl(struct lp855x *lp, int br, int max_br)
{
	unsigned int period = lp->pdata->period_ns;
	unsigned int duty = br * period / max_br;
	struct pwm_device *pwm;

	/* request pwm device with the consumer name */
	if (!lp->pwm) {
		pwm = devm_pwm_get(lp->dev, lp->chipname);
		if (IS_ERR(pwm))
			return;

		lp->pwm = pwm;

		/*
		 * FIXME: pwm_apply_args() should be removed when switching to
		 * the atomic PWM API.
		 */
		pwm_apply_args(pwm);
	}

	pwm_config(lp->pwm, duty, period);
	if (duty)
		pwm_enable(lp->pwm);
	else
		pwm_disable(lp->pwm);
}


/* HTC Implmenetation */
static int lp8556_backlight_update_brightness_register(struct lp855x *lp, int brightness)
{
	int ret;
	struct regmap *regmap = lp->regmap;
	u8 val, max_current;

	if (lp->bl->props.max_brightness == MAX_BRIGHTNESS_12BIT)
	{
		if (brightness == MAX_BRIGHTNESS_12BIT)
			max_current = MAX_CURRENT_30MA; //Set maximum LED current to 30mA
		else
			max_current = MAX_CURRENT_20MA; //Set maximum LED current to 20mA

		ret = regmap_update_bits(regmap, LP8556_CFG1_REG, LP8556_MAX_CURRENT_MASK, max_current);

		if (ret)
			return ret;

		val = brightness & LP8556_BACKLIGHT_12BIT_LSB_MASK;
		ret = regmap_write(regmap, LP8556_FULLBRIGHT_LSB_REG, val);

		if (ret)
			return ret;

		val = (brightness >> LP8556_BACKLIGHT_12BIT_MSB_SHIFT) & LP8556_BACKLIGHT_12BIT_MSB_MASK;
		ret = regmap_write(regmap, LP8556_FULLBRIGHT_MSB_REG, val);

	}
	else
	{
		val = brightness & 0xFF;
		ret = regmap_write(regmap, LP855X_BRIGHTNESS_CTRL, val);
	}

	return ret;
}

static int lp855x_flash_en_locked(struct lp855x *lp, int en, int duration, int level)
{
	int rc = 0;

	if (en)
	{
		if (lp->flash_enabled)
		{
			pr_info("%s: already enabled\n", __func__);
			rc = -EBUSY;
			goto exit;
		}

		if (level <= 0 || level > lp->bl->props.max_brightness)
			level = lp->bl->props.max_brightness;

		if (duration < 10)
			duration = 10;

		schedule_delayed_work(&lp->flash_work, msecs_to_jiffies(duration));
	}
	else
	{
		level = lp->bl->props.brightness;
	}

	pr_info("%s: (%d, %d) [level=%d]\n", __func__, en, duration, level);

	rc = lp8556_backlight_update_brightness_register(lp, level);

	lp->flash_enabled = en;
exit:
	pr_info("%s: (%d, %d) done, rc=%d\n", __func__, en, duration, rc);

	return rc;
}

static int lp855x_flash_en(struct lp855x *lp, int en, int duration, int level)
{
	int rc = 0;

	if (!en) {
		cancel_delayed_work_sync(&lp->flash_work);
	}

	mutex_lock(&lp->bl->update_lock);
	rc = lp855x_flash_en_locked(lp, en, duration, level);
	mutex_unlock(&lp->bl->update_lock);

	return rc;
}

static void lp855x_flash_off_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct lp855x *lp = container_of(dwork, struct lp855x, flash_work);

	mutex_lock(&lp->bl->update_lock);
	lp855x_flash_en_locked(lp, 0, 0, 0);
	mutex_unlock(&lp->bl->update_lock);
}

static int lp855x_flash_mode(struct htc_flashlight_dev *fl_dev, int mode1, int mode2)
{
	struct lp855x *lp = container_of(fl_dev, struct lp855x, flash_dev);

	return lp855x_flash_en(lp, mode1, FLASH_MODE_MAX_DURATION_MS, FLASH_MODE_BRIGHTNESS);
}

static int lp855x_torch_mode(struct htc_flashlight_dev *fl_dev, int mode1, int mode2)
{
	struct lp855x *lp = container_of(fl_dev, struct lp855x, flash_dev);

	return lp855x_flash_en(lp, mode1, TORCH_MODE_MAX_DURATION_MS, lp->torch_brightness);
}

static ssize_t lp855x_get_flash_en(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lp->flash_enabled);
}

static ssize_t lp855x_set_flash_en(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	int data = 0, level = 0, rc = 0;

	if (sscanf(buf, "%d %d", &data, &level) < 1)
		return -EINVAL;

	rc = lp855x_flash_en(lp, !!data, data, level);

	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(flash_en, (S_IRUGO | S_IWUSR | S_IWGRP),
	lp855x_get_flash_en, lp855x_set_flash_en);
static struct attribute *lp855x_flash_attributes[] = {
	&dev_attr_flash_en.attr,
	NULL,
};

static const struct attribute_group lp855x_flash_attr_group = {
	.attrs = lp855x_flash_attributes,
};

static void lp855x_exec_cmds(struct regmap *regmap, struct lp855x_regcmd *cmds, int num)
{
	int ret = 0;

	if (!regmap) {
		pr_err("%s, regmap is invalid\n", __func__);
		return;
	}

	if (!cmds) {
		pr_err("%s, regcmd is invalid\n", __func__);
		return;
	}

	while (num-- > 0) {
		ret = regmap_write(regmap, cmds->address, cmds->parameter);
		if (ret) {
			pr_err("%s: I2C write NG:address: %x, parameter: %x\n", __func__, cmds->address, cmds->parameter);
		} else {
			pr_debug("%s: I2C write OK:address: %x, parameter: %x\n", __func__, cmds->address, cmds->parameter);
		}
		cmds++;
	}
}

static void lp855x_parse_dt_reg_cmd_sets(struct device *dev, const char *cmd_key, struct lp855x_regcmd_sets *cmd_sets)
{
	const char *data;
	struct lp855x_regcmd *m_cmds;
	int blen = 0, sets;

	if (!cmd_key) {
		dev_err(dev, "%s cmd_key is invalid\n", __func__);
		return;
	}

	if (!cmd_sets) {
		dev_err(dev, "%s: cmd_sets is invalid, key= %s\n", __func__, cmd_key);
		return;
	}

	data = of_get_property(dev->of_node, cmd_key, &blen);
	if (!data || !blen) {
		dev_err(dev, "%s: failed, key=%s length=%d\n", __func__, cmd_key, blen);
		return;
	}

	sets = blen/2;
	m_cmds = devm_kzalloc(dev, sizeof(*m_cmds) * sets, GFP_KERNEL);

	if (!m_cmds) {
		dev_err(dev, "%s: out of memory for commands\n", __func__);
		return;
	}

	cmd_sets->reg_cmds = m_cmds;
	cmd_sets->reg_cmds_num = sets;

	while (sets-- > 0) {
		m_cmds->address = (unsigned char)*data++;
		m_cmds->parameter = (unsigned char)*data++;
		pr_info("%s, parse cmd-> address:%x , parameter:%x\n", __func__, m_cmds->address, m_cmds->parameter);
		++m_cmds;
	}

	return;
}

static void check_blank_status(struct lp855x *lp, unsigned long event)
{
	int cmd = -1;

	if (!lp || lp->next_blank == lp->current_blank)
		return ;

	switch (lp->next_blank) {
	case MSM_DRM_BLANK_SUSPEND:
	case MSM_DRM_BLANK_POWERDOWN:
	/* No need to send bl commands for MSM_DRM_BLANK_SUSPEND/MSM_DRM_BLANK_POWERDOWN.
	 * Update current_blank directly*/
		lp->current_blank = lp->next_blank;
		break;
	case MSM_DRM_BLANK_UNBLANK:
		if (lp->current_blank == MSM_DRM_BLANK_STANDBY) {
			cmd = DISABLE_AOD_CMD;
		}else if (lp->current_blank == MSM_DRM_BLANK_SUSPEND || lp->current_blank == MSM_DRM_BLANK_POWERDOWN) {
			cmd = POWER_ON_CMD;
		}
		break;
	case MSM_DRM_BLANK_STANDBY:
		if (lp->current_blank == MSM_DRM_BLANK_UNBLANK) {
			cmd = ENABLE_AOD_CMD;
		} else if (lp->current_blank == MSM_DRM_BLANK_SUSPEND || lp->current_blank == MSM_DRM_BLANK_POWERDOWN) {
			cmd = POWER_ON_AOD_CMD;
		}
		break;
	}

	if (((cmd > -1 && cmd < MAX_BL_CMD) && event == MSM_DRM_EVENT_BLANK) ||
	((cmd == DISABLE_AOD_CMD || cmd == ENABLE_AOD_CMD) && event == MSM_DRM_REQUEST_EVENT_BLANK)) {
		lp855x_exec_cmds(lp->regmap, lp->cmd_sets[cmd].reg_cmds, lp->cmd_sets[cmd].reg_cmds_num);
		pr_info("[DISP] Send backlight command %d\n", cmd);
		lp->current_blank = lp->next_blank;
	}
}

static int drm_notifier_callback(struct notifier_block *self,
					unsigned long event, void *data)
{
	struct msm_drm_notifier *notifier_data = data;
	struct lp855x *lp;

	if (notifier_data->id != MSM_DRM_PRIMARY_DISPLAY)
		return 0;

	lp = container_of(self, struct lp855x, drm_notifier);
	lp->next_blank = *(int *)notifier_data->data;

	pr_info("DRM event: %u\n", (unsigned int)event);

	check_blank_status(lp, event);

	if ((lp->current_blank == MSM_DRM_BLANK_UNBLANK || lp->current_blank == MSM_DRM_BLANK_STANDBY) && event == MSM_DRM_EVENT_BLANK)
		backlight_update_status(lp->bl);

	return 0;
}

/* HTC Implmenetation End */


static int lp855x_bl_update_status(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int ret = 0;

	if(lp->flash_enabled)
		return -EBUSY;

	check_blank_status(lp, MSM_DRM_EVENT_BLANK);

	if (lp->current_blank == MSM_DRM_BLANK_SUSPEND || lp->current_blank == MSM_DRM_BLANK_POWERDOWN) {
		pr_info("Needn't update brightness.");
		return 0;
	}

	if (lp->mode == PWM_BASED)
		lp855x_pwm_ctrl(lp, brightness, bl->props.max_brightness);
	else if (lp->mode == REGISTER_BASED && lp->chip_id == LP8556)
		ret = lp8556_backlight_update_brightness_register(lp, brightness);
	else
		lp855x_write_byte(lp, lp->cfg->reg_brightness, (u8)brightness);
	return ret;
}

static const struct backlight_ops lp855x_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lp855x_bl_update_status,
};

static int lp855x_backlight_register(struct lp855x *lp)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct lp855x_platform_data *pdata = lp->pdata;
	const char *name = pdata->name ? pdata->name : DEFAULT_BL_NAME;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_PLATFORM;

	if (lp->chip_id == LP8556)
		props.max_brightness = MAX_BRIGHTNESS_12BIT;
	else
		props.max_brightness = MAX_BRIGHTNESS;

	if (pdata->initial_brightness > props.max_brightness)
		pdata->initial_brightness = props.max_brightness;

	props.brightness = pdata->initial_brightness;

	bl = devm_backlight_device_register(lp->dev, name, lp->dev, lp,
				       &lp855x_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	lp->bl = bl;

	return 0;
}

static ssize_t lp855x_get_chip_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lp->chipname);
}

static ssize_t lp855x_get_bl_ctl_mode(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	char *strmode = NULL;

	if (lp->mode == PWM_BASED)
		strmode = "pwm based";
	else if (lp->mode == REGISTER_BASED)
		strmode = "register based";

	return scnprintf(buf, PAGE_SIZE, "%s\n", strmode);
}

static DEVICE_ATTR(chip_id, S_IRUGO, lp855x_get_chip_id, NULL);
static DEVICE_ATTR(bl_ctl_mode, S_IRUGO, lp855x_get_bl_ctl_mode, NULL);

static struct attribute *lp855x_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_bl_ctl_mode.attr,
	NULL,
};

static const struct attribute_group lp855x_attr_group = {
	.attrs = lp855x_attributes,
};

#ifdef CONFIG_OF
static int lp855x_parse_dt(struct lp855x *lp)
{
	struct device *dev = lp->dev;
	struct device_node *node = dev->of_node;
	struct lp855x_platform_data *pdata;
	int rom_length, i;

	if (!node) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	of_property_read_string(node, "bl-name", &pdata->name);
	of_property_read_u8(node, "dev-ctrl", &pdata->device_control);
	of_property_read_u8(node, "init-brt", &pdata->initial_brightness);
	of_property_read_u32(node, "pwm-period", &pdata->period_ns);
	/*HTC ADD*/
	of_property_read_u32(node, "torch-brt", &lp->torch_brightness);

	for (i = 0; i < MAX_BL_CMD; ++i) {
		lp855x_parse_dt_reg_cmd_sets(dev, lp855x_command_keyword[i], &lp->cmd_sets[i]);
	}

	/* Fill ROM platform data if defined */
	rom_length = of_get_child_count(node);
	if (rom_length > 0) {
		struct lp855x_rom_data *rom;
		struct device_node *child;
		int i = 0;

		rom = devm_kzalloc(dev, sizeof(*rom) * rom_length, GFP_KERNEL);
		if (!rom)
			return -ENOMEM;

		for_each_child_of_node(node, child) {
			of_property_read_u8(child, "rom-addr", &rom[i].addr);
			of_property_read_u8(child, "rom-val", &rom[i].val);
			i++;
		}

		pdata->size_program = rom_length;
		pdata->rom_data = &rom[0];
	}

	lp->pdata = pdata;

	return 0;
}
#else
static int lp855x_parse_dt(struct lp855x *lp)
{
	return -EINVAL;
}
#endif

static int lp855x_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lp855x *lp;
	struct regmap_config regmap_cfg;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	lp = devm_kzalloc(&cl->dev, sizeof(struct lp855x), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;

	lp->client = cl;
	lp->dev = &cl->dev;
	lp->chipname = id->name;
	lp->chip_id = id->driver_data;
	lp->pdata = dev_get_platdata(&cl->dev);

	if (!lp->pdata) {
		ret = lp855x_parse_dt(lp);
		if (ret < 0)
			return ret;
	}

	if (lp->pdata->period_ns > 0)
		lp->mode = PWM_BASED;
	else
		lp->mode = REGISTER_BASED;

	lp->supply = devm_regulator_get(lp->dev, "power");
	if (IS_ERR(lp->supply)) {
		if (PTR_ERR(lp->supply) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		lp->supply = NULL;
	}

	lp->enable = devm_regulator_get_optional(lp->dev, "enable");
	if (IS_ERR(lp->enable)) {
		ret = PTR_ERR(lp->enable);
		if (ret == -ENODEV) {
			lp->enable = NULL;
		} else {
			if (ret != -EPROBE_DEFER)
				dev_err(lp->dev, "error getting enable regulator: %d\n",
					ret);
			return ret;
		}
	}

	if (lp->supply) {
		ret = regulator_enable(lp->supply);
		if (ret < 0) {
			dev_err(&cl->dev, "failed to enable supply: %d\n", ret);
			return ret;
		}
	}

	if (lp->enable) {
		ret = regulator_enable(lp->enable);
		if (ret < 0) {
			dev_err(lp->dev, "failed to enable vddio: %d\n", ret);
			return ret;
		}

		/*
		 * LP8555 datasheet says t_RESPONSE (time between VDDIO and
		 * I2C) is 1ms.
		 */
		usleep_range(1000, 2000);
	}

	/* Setup regmap */
	memset(&regmap_cfg, 0, sizeof(struct regmap_config));
	regmap_cfg.reg_bits = 8;
	regmap_cfg.val_bits = 8;
	regmap_cfg.name = id->name;
	regmap_cfg.max_register = LP8556_MAX_REGISTER;

	lp->regmap = devm_regmap_init_i2c(cl, &regmap_cfg);

	if (IS_ERR(lp->regmap))
	{
		return PTR_ERR(lp->regmap);
	}

	i2c_set_clientdata(cl, lp);

	ret = lp855x_configure(lp);
	if (ret) {
		dev_err(lp->dev, "device config err: %d", ret);
		return ret;
	}

	ret = lp855x_backlight_register(lp);
	if (ret) {
		dev_err(lp->dev,
			"failed to register backlight. err: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&lp->dev->kobj, &lp855x_attr_group);
	if (ret) {
		dev_err(lp->dev, "failed to register sysfs. err: %d\n", ret);
		return ret;
	}

	//backlight_update_status(lp->bl);

	lp->next_blank = lp->current_blank = MSM_DRM_BLANK_UNBLANK;
	lp->drm_notifier.notifier_call = drm_notifier_callback;
	msm_drm_register_client(&lp->drm_notifier);

	if(lp->torch_brightness != 0)
	{
		INIT_DELAYED_WORK(&lp->flash_work, lp855x_flash_off_work);
		lp->flash_dev.id = 1;
		lp->flash_dev.flash_func = lp855x_flash_mode;
		lp->flash_dev.torch_func = lp855x_torch_mode;

		if (register_htc_flashlight(&lp->flash_dev))
		{
			pr_err("%s: register htc_flashlight failed!\n", __func__);
			lp->flash_dev.id = -1;
		}
		else
		{
			ret = sysfs_create_group(&lp->bl->dev.kobj, &lp855x_flash_attr_group);
			dev_info(lp->dev, "create flash attrs, ret=%d\n", ret);
		}
	}


	return 0;
}

static int lp855x_remove(struct i2c_client *cl)
{
	int i;
	struct lp855x *lp = i2c_get_clientdata(cl);

	lp->bl->props.brightness = 0;
	backlight_update_status(lp->bl);
	if (lp->supply)
		regulator_disable(lp->supply);
	sysfs_remove_group(&lp->dev->kobj, &lp855x_attr_group);

	if(lp->torch_brightness != 0)
	{
		sysfs_remove_group(&lp->bl->dev.kobj, &lp855x_flash_attr_group);
		unregister_htc_flashlight(&lp->flash_dev);
		lp->torch_brightness = 0;
		lp->flash_dev.id = -1;
	}

	for (i = 0; i < MAX_BL_CMD; ++i) {
		if (lp->cmd_sets[i].reg_cmds) {
			devm_kfree(lp->dev, lp->cmd_sets[i].reg_cmds);
			lp->cmd_sets[i].reg_cmds = NULL;
			lp->cmd_sets[i].reg_cmds_num = 0;
		}
	}

	msm_drm_unregister_client(&lp->drm_notifier);
	return 0;
}

static const struct of_device_id lp855x_dt_ids[] = {
	{ .compatible = "ti,lp8550", },
	{ .compatible = "ti,lp8551", },
	{ .compatible = "ti,lp8552", },
	{ .compatible = "ti,lp8553", },
	{ .compatible = "ti,lp8555", },
	{ .compatible = "ti,lp8556", },
	{ .compatible = "ti,lp8557", },
	{ }
};
MODULE_DEVICE_TABLE(of, lp855x_dt_ids);

static const struct i2c_device_id lp855x_ids[] = {
	{"lp8550", LP8550},
	{"lp8551", LP8551},
	{"lp8552", LP8552},
	{"lp8553", LP8553},
	{"lp8555", LP8555},
	{"lp8556", LP8556},
	{"lp8557", LP8557},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp855x_ids);

static struct i2c_driver lp855x_driver = {
	.driver = {
		   .name = "lp855x",
		   .of_match_table = of_match_ptr(lp855x_dt_ids),
		   },
	.probe = lp855x_probe,
	.remove = lp855x_remove,
	.id_table = lp855x_ids,
};

static int __init lp855x_init(void)
{
	int ret = -ENODEV;

	ret = i2c_add_driver(&lp855x_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(lp855x_init);

static void __exit lp855x_exit(void)
{
	i2c_del_driver(&lp855x_driver);
}
module_exit(lp855x_exit);

MODULE_DESCRIPTION("Texas Instruments LP855x Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_LICENSE("GPL");
