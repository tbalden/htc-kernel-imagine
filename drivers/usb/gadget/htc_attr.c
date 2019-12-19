/*
 * Copyright (C) 2017 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/delay.h>


#if defined(CONFIG_USB_CONFIGFS_UEVENT)
int htc_usb_enable_function(char *name, int ebl)
{
	struct gadget_info *gi = dev_get_drvdata(android_device);
	struct usb_composite_dev *cdev = &gi->cdev;
	char state_buf[60];
	char name_buf[60];
	char *function[3];
	if (!strcmp(name, "ffs"))
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "adb");
	else
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", name);
	function[0] = state_buf;
	function[2] = NULL;

	if (ebl) {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_on");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_on.this_device->kobj, KOBJ_CHANGE,
				function);
	} else {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_off");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_off.this_device->kobj, KOBJ_CHANGE,
				function);
	}
	return 0;
}
#endif

/* show current os type for mac or non-mac */
static ssize_t show_os_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct gadget_info *gi = dev_get_drvdata(dev);
	struct usb_composite_dev *cdev = &gi->cdev;

	length = sprintf(buf, "%d\n", cdev->os_type);
	pr_info("[USB] %s: %s\n", __func__, buf);
	return length;
}

static int usb_disable;
static ssize_t show_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", usb_disable);
	return length;
}

extern void htc_dwc3_disable_usb(int disable_usb);
static ssize_t store_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int disable_usb_function;

	sscanf(buf, "%d ", &disable_usb_function);
	pr_info("USB_disable set %d\n", disable_usb_function);
	usb_disable = disable_usb_function;
	htc_dwc3_disable_usb(disable_usb_function);
	return count;
}

ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct gadget_info *gi = dev_get_drvdata(dev);
	struct usb_composite_dev *cdev = &gi->cdev;
	int ret = 0;
	int i;

	if ((connect2pc == 1) && !usb_disable) {
		if (cdev && cdev->gadget) {
			if (cdev->gadget->speed == USB_SPEED_SUPER)
				ret = 2;
			else
				ret = 1;
		}

		if (ret == 0) {
			pr_info("htc_attr waiting cdev->gadget to true\n");
			for (i = 0; i < 10; i++) {
				if (!cdev->gadget) {
					msleep(50);
				} else {
					if (cdev->gadget->speed == USB_SPEED_SUPER)
						ret = 2;
					else
						ret = 1;

					pr_info("htc_attr break  i=%d ret=%d\n", i, ret);
					break;

				}
			}
			if (ret == 0) {
				pr_err("htc_attr usb_cable_connect error\n");
			}
		}

	}
	length = sprintf(buf, "%d", ret);
	return length;
}

static ssize_t show_pd_cap(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int pd_cap = 0;
	struct gadget_info *gi = dev_get_drvdata(dev);
	union power_supply_propval pval = {0};
	bool pd_active, pd_allowed;

	if (!gi->usb_psy) {
		gi->usb_psy = power_supply_get_by_name("usb");
		if (!gi->usb_psy) {
			pr_warn("Could not get usb power_supply\n");
			return -ENODEV;
		}
	}

	power_supply_get_property(gi->usb_psy, POWER_SUPPLY_PROP_PD_ALLOWED, &pval);
	pd_allowed = pval.intval;

	power_supply_get_property(gi->usb_psy, POWER_SUPPLY_PROP_PD_ACTIVE, &pval);
	pd_active = pval.intval;

	if (pd_allowed && pd_active)
		pd_cap = 1;
	else
		pd_cap = 0;

	pr_info("%s : show pd capability = %d\n", __func__, pd_cap);
	return snprintf(buf, PAGE_SIZE, "%d\n", pd_cap);
}

void htc_set_usbmode(bool on)
{
	usbmode = on;
	return;
}

extern int usb_get_dwc_property(int prop_type);
static ssize_t show_usb_ac_cable_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	length = sprintf(buf, "%d\n", usb_get_dwc_property(PROPERTY_CHG_STATUS));
	return length;
}

extern int get_vendor_request(void);
extern void clear_hcmd_cmd(void);
static ssize_t show_hcmd_request(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = get_vendor_request();

	if (ret != 0)
		pr_info("%s:%s(parent:%s): tgid=%d, result=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid, ret);

	clear_hcmd_cmd();
	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static DEVICE_ATTR(usb_ac_cable_status, 0444, show_usb_ac_cable_status, NULL);
static DEVICE_ATTR(usb_disable, 0664, show_usb_disable_setting, store_usb_disable_setting);
//static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL);
static DEVICE_ATTR(pd_cap, 0444, show_pd_cap, NULL);
static DEVICE_ATTR(os_type, 0444, show_os_type, NULL);
static DEVICE_ATTR(hcmd, 0440, show_hcmd_request, NULL);

static __maybe_unused struct attribute *android_htc_usb_attributes[] = {
	&dev_attr_usb_ac_cable_status.attr,
	&dev_attr_usb_disable.attr,
	//&dev_attr_usb_cable_connect.attr,
	&dev_attr_pd_cap.attr,
	&dev_attr_os_type.attr,
	&dev_attr_hcmd.attr,
	NULL
};

static  __maybe_unused const struct attribute_group android_usb_attr_group = {
	.attrs = android_htc_usb_attributes,
};

static void setup_vendor_info(struct device *dev)
{
	if (sysfs_create_group(&dev->kobj, &android_usb_attr_group))
		pr_err("%s: fail to create sysfs\n", __func__);
	/* Link android_usb to /sys/devices/platform */
	if (sysfs_create_link(&platform_bus.kobj, &dev->kobj, "android_usb"))
		pr_err("%s: fail to link android_usb to /sys/devices/platform/\n", __func__);
}

