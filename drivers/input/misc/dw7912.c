/*
 * dw7912.c  --  Vibrator driver for dw7912
 *
 * Copyright (C) 2017 Dongwoon Anatech Co. Ltd. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include <linux/dw7912.h>
#include <linux/leds.h>
#include <linux/memory.h>
#include <linux/hrtimer.h>
#include <linux/atomic.h>
#include <linux/vibtrig.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/async.h>

#ifdef CONFIG_UCI_NOTIFICATIONS
#include <linux/notification/notification.h>
#include <linux/uci/uci.h>
#include <linux/alarmtimer.h>
#endif

struct dw7912_priv {
	struct led_classdev             cdev;
	struct work_struct              haptics_work;
	struct hrtimer                  stop_timer;
	struct mutex                    play_lock;
#ifdef CONFIG_VIB_TRIGGERS
	struct vib_trigger_enabler      enabler;
#endif
	atomic_t                        state;
	atomic_t                        realstate;
	u8 waveNum;
	u32 defaultFreq;
	u32 calFreq;
	u32 vibTime;
	u32 bc_status;
	int en;
	int waveSize1;
	int waveSize2;
	int waveSize3;
#ifdef CONFIG_INPUT_DW7912_VOLTAGE_SWITCH_WA
	int sideKeys_Voltage;
	int notification_Voltage;
#endif
	u8 mem_header_1[8];
	u8 mem_header_2[8];
	u8 mem_header_3[8];
	u8 wave_buf[300];
	struct i2c_client *dwclient;
};
u8 *mem_wave_1;
u8 *mem_wave_2;
u8 *mem_wave_3;

enum f34_version {
	Freq_140 = 140,
	Freq_141,
	Freq_142,
	Freq_143,
	Freq_144,
	Freq_145,
	Freq_146,
	Freq_147,
	Freq_148,
	Freq_149,
	Freq_150,
	Freq_151,
	Freq_152,
	Freq_153,
	Freq_154,
	Freq_155,
	Freq_156,
	Freq_157,
	Freq_158,
	Freq_159,
	Freq_160,
};


static struct dw7912_priv *Gdw7912;
int real_time_playback_mode_test(struct dw7912_priv *p);
int real_time_playback(struct dw7912_priv *p, u8 *data, u32 size);

#define HAP_MAX_PLAY_TIME_MS	15000
#define HEADER_LOCATION		"/data/misc/HD_DW.txt"
#define WAVEFORM_LOCATION	"/data/misc/WF_DW.txt"

#if 1
	#define gprintk(fmt, x... ) printk( "[VIB] " fmt, ## x)
#else
	#define gprintk(x...) do { } while (0)
#endif


#ifdef CONFIG_UCI_NOTIFICATIONS

static bool notification_duration_detected = 0;

static int notification_booster = 0;
static int notification_booster_overdrive_perc = 70;
static int call_alarm_booster_overdrive_perc = 70;
static int vibration_power_set = 0;
static int vibration_power_percentage = 50;

static int suspend_booster = 0;
static int vmax_needs_reset = 1;
//static int alarm_value_counter = 0;
//static int last_value = 0;
//static unsigned long last_alarm_value_jiffies = 0;

int uci_get_notification_booster(void) {
    return uci_get_user_property_int_mm("notification_booster", notification_booster,0,100);
}
int uci_get_notification_booster_overdrive_perc(void) {
    return uci_get_user_property_int_mm("notification_booster_overdrive_perc", notification_booster_overdrive_perc,50,100);
}
int uci_get_call_alarm_booster_overdrive_perc(void) {
    return uci_get_user_property_int_mm("call_alarm_booster_overdrive_perc", call_alarm_booster_overdrive_perc,50,100);
}

int uci_get_vibration_power_percentage(void) {
    return uci_get_user_property_int_mm("vibration_power_percentage", vibration_power_percentage,0,100);
}
int uci_get_vibration_sidekeys_percentage(void) {
    return uci_get_user_property_int_mm("vibration_sidekeys_percentage", 50,20,100);
}
int uci_get_vibration_power_set(void) {
    return uci_get_user_property_int_mm("vibration_power_set", vibration_power_set,0,1);
}

// register user uci listener
static int current_power_set = 0;
static int current_perc_set = 50;
static int current_sk_perc_set = 50;

struct dw7912_priv *g_p;
static int original_notif_voltage = 0;
static int original_sidekeys_voltage = 0;

void haptics_voltage_switch(bool enabled);

#define MAX_VOLTAGE 253

static DEFINE_MUTEX(boost_lock);

static bool boost_reset = true;
static void boost_voltage(bool on,bool call_or_alarm) {
	static bool boosted = false;
	static bool boosted_call_or_alarm = false;
	pr_info("%s [VIB] haptics uci boost... %d \n",__func__,on);
	mutex_lock(&boost_lock);
	if ((on && (!boosted || call_or_alarm!=boosted_call_or_alarm))||boost_reset) {
		boosted = true;
		boosted_call_or_alarm = call_or_alarm;
		if (call_or_alarm) {
			g_p->notification_Voltage = (MAX_VOLTAGE) * uci_get_call_alarm_booster_overdrive_perc() / 100;
		} else {
			g_p->notification_Voltage = (MAX_VOLTAGE) * uci_get_notification_booster_overdrive_perc() / 100;
		}
		haptics_voltage_switch(true);
	} else if ((!on && boosted)||boost_reset) {
		boosted = false;
		g_p->notification_Voltage = current_power_set?(original_notif_voltage * current_perc_set)/25:original_notif_voltage;
		if (g_p->notification_Voltage > MAX_VOLTAGE) g_p->notification_Voltage = MAX_VOLTAGE;
		haptics_voltage_switch(true);
	}
	mutex_unlock(&boost_lock);
}

static bool boost_call_alarm_always = true;
static void uci_user_listener(void) {
    pr_info("%s haptics uci user parse happened...\n",__func__);
    vmax_needs_reset = 1;
    {
		int set = uci_get_vibration_power_set();
		int perc = uci_get_vibration_power_percentage();
		int sk_perc = uci_get_vibration_sidekeys_percentage();
		boost_call_alarm_always = uci_get_user_property_int_mm("boost_call_alarm_always", 1,0,1);
		if (set!=current_power_set || current_perc_set!=perc || current_sk_perc_set!=sk_perc) {
			if (g_p) {
				g_p->notification_Voltage = set?(original_notif_voltage * perc)/25:original_notif_voltage;
				g_p->sideKeys_Voltage = set?(original_sidekeys_voltage * sk_perc)/25:original_sidekeys_voltage*150/50; // boost original in unset mode as well, it's too low if haptics switch is called otherwise!
				if (g_p->notification_Voltage > MAX_VOLTAGE) g_p->notification_Voltage = MAX_VOLTAGE;
				if (g_p->sideKeys_Voltage > MAX_VOLTAGE) g_p->sideKeys_Voltage = MAX_VOLTAGE;
				pr_info("%s [VIB] set notif voltage %d ...\n",__func__, g_p->notification_Voltage);
				pr_info("%s [VIB] set sidekeys voltage %d ...\n",__func__, g_p->sideKeys_Voltage);
				haptics_voltage_switch(true);
				current_power_set = set;
				current_perc_set = perc;
				current_sk_perc_set = sk_perc;
			}
		}
    }
}

static int boost_only_in_pocket = 1;
static bool face_down_hr = false;
static bool proximity = false;
static bool in_pocket = false;

int uci_get_boost_only_in_pocket(void) {
    return uci_get_user_property_int_mm("boost_only_in_pocket", boost_only_in_pocket, 0, 1);
}

// register sys uci listener
static void uci_sys_listener(void) {
    // TODO use ntf listener instead

    pr_info("%s [VIB] uci sys parse happened...\n",__func__);
    proximity = !!uci_get_sys_property_int_mm("proximity", 1,0,1);
    face_down_hr = !!uci_get_sys_property_int_mm("face_down_hr", 0,0,1);
    // check if perfectly horizontal facedown is not true, and in proximity 
    // ...(so it's supposedly not on table, but in pocket) then in_pocket = true
    in_pocket = !face_down_hr && proximity;
}

void set_suspend_booster(int value) {
    suspend_booster = !!value;
}
EXPORT_SYMBOL(set_suspend_booster);

void set_notification_booster(int value) {
    notification_booster = value;
}
EXPORT_SYMBOL(set_notification_booster);
int get_notification_booster(void) {
    return notification_booster;
}
EXPORT_SYMBOL(get_notification_booster);
void set_notification_boost_only_in_pocket(int value) {
    boost_only_in_pocket = value;
}
EXPORT_SYMBOL(set_notification_boost_only_in_pocket);
int get_notification_boost_only_in_pocket(void) {
    return boost_only_in_pocket;
}
EXPORT_SYMBOL(get_notification_boost_only_in_pocket);

// TODO call ntf_haptic ##################
static bool is_call_or_alarm(int duration) {
	return duration == MIN_TD_VALUE_NOTIFICATION_ALARM || duration == MIN_TD_VALUE_NOTIFICATION_CALL;
}
static bool should_boost_call_alarm_always(int duration) {
	if (is_call_or_alarm(duration)) {
		if (boost_call_alarm_always) return true;
	}
	return false;
}

static int should_not_boost(void) {
    int l_boost_only_in_pocket = uci_get_boost_only_in_pocket();
    if (ntf_is_screen_on() && ntf_wake_by_user()) return 1;
    if ((l_boost_only_in_pocket && in_pocket) || !l_boost_only_in_pocket) return 0;
    return 1;
}

static int smart_get_boost_on(void) {
    int level = smart_get_notification_level(NOTIF_VIB_BOOSTER);
    int ret = !suspend_booster && uci_get_notification_booster();
    if (level != NOTIF_DEFAULT) {
	ret = 0; // should suspend boosting if not DEFAULT level
    }
    pr_info("%s smart_notif =========== level: %d  notif vib should boost %d \n",__func__, level, ret);
    return ret;
}

#endif

bool is_bc_running(void) {
	struct dw7912_priv *pDW = Gdw7912;
	return (pDW->bc_status);
}

int memory_mode_play_set(struct dw7912_priv *p, int mode)
{
	struct i2c_client *c = p->dwclient;
	i2c_smbus_write_byte_data(c, 0x03, 0x01);// play type set 1: memory, 0:RTP

	/*** clean wave seq and wave seq loop register ***/
	i2c_smbus_write_byte_data(c, 0x0c, 0x0);
	i2c_smbus_write_byte_data(c, 0x0d, 0x0);
	i2c_smbus_write_byte_data(c, 0x0e, 0x0);
	i2c_smbus_write_byte_data(c, 0x0f, 0x0);
	i2c_smbus_write_byte_data(c, 0x10, 0x0);
	i2c_smbus_write_byte_data(c, 0x11, 0x0);
	i2c_smbus_write_byte_data(c, 0x12, 0x0);
	i2c_smbus_write_byte_data(c, 0x13, 0x0);
	i2c_smbus_write_byte_data(c, 0x14, 0x0);
	i2c_smbus_write_byte_data(c, 0x15, 0x0);
	i2c_smbus_write_byte_data(c, 0x16, 0x0);
	i2c_smbus_write_byte_data(c, 0x17, 0x0);

	switch(mode) {
		case 0:
			i2c_smbus_write_byte_data(c, 0x0c, 0x01);
			i2c_smbus_write_byte_data(c, 0x09, 0x01);
			break;
		case 1:
			i2c_smbus_write_byte_data(c, 0x0c, 0x02);
			i2c_smbus_write_byte_data(c, 0x09, 0x01);
			break;

		case 2:
			i2c_smbus_write_byte_data(c, 0x0c, 0x03);
			i2c_smbus_write_byte_data(c, 0x09, 0x01);
			break;
		case 3:
			i2c_smbus_write_byte_data(c, 0x0c, 0x04);
			i2c_smbus_write_byte_data(c, 0x09, 0x01);
			break;
		case 4:
			i2c_smbus_write_byte_data(c, 0x0c, 0x05);
			i2c_smbus_write_byte_data(c, 0x09, 0x01);
			break;
		case 5:
			i2c_smbus_write_byte_data(c, 0x0c, 0x06);
			i2c_smbus_write_byte_data(c, 0x09, 0x01);
			break;
		default:
			gprintk("undefined play mode\n");
			break;
	}

	gprintk("memory data write done\n");
	return 0;
}

int memory_mode_play(struct dw7912_priv *p, u8 *head_data, u8 *data, int waveSize)
{
	int rc = 0;
	int k, j, loop, tail, start_addr, st_addr, trans_size;
	struct i2c_client *c = p->dwclient;
	u8 *cpy_head_data = kzalloc(sizeof(u8) * sizeof(&head_data), GFP_KERNEL);
	u8 *cpy_data = NULL;
	struct i2c_msg xfer[] = {
		{
			.addr = 0x59,
			.flags = 0,
			.len = 8,
			.buf = cpy_head_data,
		}
	};

	if (!cpy_head_data) {
		gprintk("dynamic allocate cpy head error\n");
		return -1;
	}
	cpy_data = kzalloc(sizeof(u8) * waveSize, GFP_KERNEL);
	if (!cpy_data) {
		gprintk("dynamic allocate cpy data error\n");
		kfree(cpy_head_data);
		return -1;
	}

	memcpy(cpy_head_data, head_data, sizeof(&head_data));
	memcpy(cpy_data, data, waveSize);

	i2c_smbus_write_byte_data(c, 0x03, 0x01); //memory mode set

	rc = i2c_transfer(c->adapter, xfer, 1);
	mdelay(10);
	if (rc < 0) {
		gprintk("M_header write failed\n");
		goto i2c_transfer_fail;
	}

	st_addr = ((int)cpy_head_data[3]<<8) | cpy_head_data[4];
	trans_size = 250;
	loop = waveSize / trans_size;// step 3 : wave data write
	tail = waveSize % trans_size;
	gprintk("data size: %d, loop: %d, tail: %d\n", waveSize, loop, tail);

	for (k=0; k<loop; k++) {
		start_addr = st_addr + k * trans_size;
		p->wave_buf[0] = 0x1b;
		p->wave_buf[1] = start_addr >> 8;
		p->wave_buf[2] = start_addr;
		for (j=0; j<trans_size; j++) {
			p->wave_buf[j + 3] = cpy_data[j + k * trans_size];
			//gprintk(" data: %x  cpy: %x\n", p->wave_buf[j + 3], cpy_data[j + k * trans_size]);
		}

		xfer[0].len   = trans_size + 3;
		xfer[0].buf   = p->wave_buf;
		rc = i2c_transfer(c->adapter, xfer, 1);
		mdelay(10);
		if (rc < 0) {
			gprintk("data write failed\n");
			goto i2c_transfer_fail;
		}
	}

	start_addr = st_addr + loop * trans_size;
	p->wave_buf[0] = 0x1b;
	p->wave_buf[1] = start_addr >> 8;
	p->wave_buf[2] = start_addr;
	for (j=0; j<tail; j++) {
		p->wave_buf[j + 3] = cpy_data[j + loop * trans_size];
		//gprintk(" data: %x cpy: %x\n", p->wave_buf[j + 3], cpy_data[j + loop * trans_size]);
	}
	xfer[0].len   = tail + 3;
	xfer[0].buf   = p->wave_buf;
	rc = i2c_transfer(c->adapter, xfer, 1);
	mdelay(10);
	if (rc < 0) {
		gprintk("data write failed\n");
		goto i2c_transfer_fail;
	}

	gprintk("memory data write done\n");

i2c_transfer_fail:
	kfree(cpy_head_data);
	kfree(cpy_data);

	return rc;
}


#if 1
extern void register_squeeze(unsigned long timestamp, int vibration);

static void dw7912_haptics_off_work(struct work_struct *haptics_off_work)
{
	struct dw7912_priv *pDW = Gdw7912;
	mutex_lock(&pDW->play_lock);
	{
		i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x0);
		gprintk("%s off work - %s\n", __func__, "off");
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0d, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0e, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0f, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x10, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x11, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x12, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x13, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x14, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x15, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x16, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x17, 0x0);
	}
	mutex_unlock(&pDW->play_lock);
}
DECLARE_WORK(haptics_off_work,dw7912_haptics_off_work);
#endif

static void dw7912_haptics_work(struct work_struct *work)
{
	struct dw7912_priv *pDW = Gdw7912;
	bool enable;

	mutex_lock(&pDW->play_lock);

	if(is_bc_running()) {
		gprintk("BC status is running! Skip vibrator access!");
		mutex_unlock(&pDW->play_lock);
		return ;
	}

	enable = atomic_read(&pDW->state);
	pr_info("%s [VIB] atomic read %d...\n",__func__, enable);

	if (enable) {
		atomic_set(&pDW->realstate, 1);
		if (pDW->vibTime == 100000) {
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on with pattern 1");
		} else if (pDW->vibTime == 200000) {
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x05);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on with pattern 2");
		} else if (pDW->vibTime == 300000) {
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x05);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on with pattern 3");
		} else if (pDW->vibTime == 400000) {
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on with pattern 4");
#if 1
			gprintk("%s\n", "squeeze: pattern 4 vibration... edge sense");
			register_squeeze(jiffies, 1);
#endif
		} else if (pDW->vibTime == 410000) {
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on with pattern 4");
#if 1
			gprintk("%s\n", "squeeze: pattern 4 vibration... edge sense");
			register_squeeze(jiffies, 1);
#endif
		} else if (pDW->vibTime == 500000) {
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x05);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on with pattern 5");
		} else if (pDW->vibTime == 600000) {
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on with pattern 5");
		} else {
#if 1
			int ret = 0;
			ret = i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			if (ret<0) {
				pr_info("%s [flashwake] [VIB] i2c error... waking idle cpus and retry... %d\n",__func__,ret);
				wake_up_all_idle_cpus();
				mdelay(30);
				ret = i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			}
#endif
			i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x06);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x14, 0x0F);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
			gprintk("%s\n", "on");
			pr_info("%s [VIB] vibtime %d...\n",__func__, pDW->vibTime);
			hrtimer_start(&pDW->stop_timer, ktime_set(pDW->vibTime / MSEC_PER_SEC, (pDW->vibTime % MSEC_PER_SEC) * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	} else {
		i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x0);
		gprintk("%s\n", "off");
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0d, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0e, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0f, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x10, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x11, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x12, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x13, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x14, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x15, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x16, 0x0);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x17, 0x0);
		atomic_set(&pDW->realstate, 0);
	}
	mutex_unlock(&pDW->play_lock);
}
#ifdef CONFIG_UCI_NOTIFICATIONS
static bool vibrator_trigger_count = 0;

#endif

static enum hrtimer_restart hap_stop_timer(struct hrtimer *timer)
{
	struct dw7912_priv *pDW = Gdw7912;

	if (atomic_read(&pDW->state)) {
		pr_info("%s [VIB] atomic read 1, atomic set 0...\n",__func__);
		atomic_set(&pDW->state, 0);
		schedule_work(&pDW->haptics_work);
	} else {
#if 1
		pr_info("%s [VIB] atomic read 0...\n",__func__);
		if (vibrator_trigger_count-->0) {
			pr_info("%s [VIB] atomic set 0...\n",__func__);
			atomic_set(&pDW->state, 0);
			schedule_work(&haptics_off_work);
		} else
#endif
		gprintk("Vibrator already stop\n");
	}

	return HRTIMER_NORESTART;
}

void waveform_freq_set(struct dw7912_priv *p, u32 freqNum)
{
	if (freqNum > 160 || freqNum < 140) {
		gprintk("Calibration frequency out of range %u\n", freqNum);
	} else {
		if (memory_mode_play(p, mem_cali_header6[freqNum - 140], mem_cali_wave6[freqNum - 140],
				((mem_cali_header6[freqNum - 140][5] & 0x0F) << 8) + mem_cali_header6[freqNum - 140][6]) < 0) {
			gprintk("Calibration frequency setting fail %u\n", freqNum);
		} else {
			gprintk("Calibration frequency setting done %u\n", freqNum);
		}
	}
}
static ssize_t dw_haptics_store_waveform(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	if (buf[0] == '1') {
		pDW->waveNum = 1;
	} else if (buf[0] == '2') {
		pDW->waveNum = 2;
	} else
		gprintk("Parameter Error\n");

	return count;
}
static ssize_t dw_haptics_show_waveform(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw7912_priv *pDW = Gdw7912;
	int i = 0;
	size_t count = 0;

	if (pDW->waveNum == 1) {
		//count += snprintf(buf + count, PAGE_SIZE - count, "Header 1:\n");
		gprintk("Header 1:\n");
		for (i = 0; i < 8; i++)
			gprintk("Mem_header_1[%d]: %x\n", i, pDW->mem_header_1[i]);
		/*for (i = 0; i < sizeof(pDW->mem_header_1); i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%x ", pDW->mem_header_1[i]);
			if((i+1)%5 == 0)
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}*/
		//i = 0;
		//count += scnprintf(buf + count, PAGE_SIZE - count, "\nWaveform 1:\n");
		gprintk("Waveform 1:\n");
		for (i = 0; i < pDW->waveSize1; i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%x ", mem_wave_1[i]);
			if((i+1)%5 == 0)
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
		gprintk("end of waveform 1 show");
	} else if (pDW->waveNum == 2) {
		count += snprintf(buf + count, PAGE_SIZE - count, "Header 2:\n");
		for (i = 0; i < sizeof(pDW->mem_header_2); i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%x ", pDW->mem_header_2[i]);
			if((i+1)%5 == 0)
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
		i = 0;
		count += scnprintf(buf + count, PAGE_SIZE - count, "\nWaveform 2:\n");
		for (i = 0; i < pDW->waveSize2; i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%x ", mem_wave_2[i]);
			if((i+1)%5 == 0)
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
		gprintk("end of waveform 2 show");
	} else if (pDW->waveNum == 3) {
		count += snprintf(buf + count, PAGE_SIZE - count, "Header 3:\n");
		for (i = 0; i < sizeof(pDW->mem_header_3); i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%x ", pDW->mem_header_3[i]);
			if((i+1)%5 == 0)
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
		i = 0;
		count += scnprintf(buf + count, PAGE_SIZE - count, "\nWaveform 3:\n");
		for (i = 0; i < pDW->waveSize3; i++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%x ", mem_wave_3[i]);
			if((i+1)%5 == 0)
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
		gprintk("end of waveform 2 show");
	}

	return count;
}

static ssize_t dw_haptics_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	if(is_bc_running()) {
		gprintk("BC status is running! Skip vibrator access!");
		return 0;
	}

	if (buf[0] == '1') {
/*		i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x00); // play type set 1: memory, 0:RTP
		i2c_smbus_write_byte_data(pDW->dwclient, 0x08, 0x96); //step3 : vd-calmp set 6v
		i2c_smbus_write_byte_data(pDW->dwclient, 0x23, 0x06); //step4 : boost offset set
		real_time_playback(pDW, rtp_wave1, sizeof(rtp_wave1), 2);
		real_time_playback(pDW, rtp_wave2, sizeof(rtp_wave2), 2);
		real_time_playback(pDW, rtp_wave3, sizeof(rtp_wave3), 2);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01); //step5 : set go bit*/

		gprintk(" ECHO 1 \n");
		memory_mode_play_set(pDW, 0);
	} else if (buf[0] == '2') {
		gprintk(" ECHO 2 \n");
		memory_mode_play_set(pDW, 1);
	} else if (buf[0] == '3') {
		gprintk(" ECHO 3 \n");
		memory_mode_play_set(pDW, 2);
	} else if (buf[0] == '4') {
		memory_mode_play_set(pDW, 3);
	} else if (buf[0] == '5') {
		memory_mode_play_set(pDW, 4);
	} else if (buf[0] == '6') {
		memory_mode_play_set(pDW, 5);
	} else if (buf[0] == '7') {
		gprintk(" Set infinite mode with timer, time %u\n", pDW->vibTime);
		if (pDW->vibTime > 0) {
			pr_info("%s [VIB] cancel stop timer #1...\n",__func__);
			hrtimer_cancel(&pDW->stop_timer);
			cancel_work_sync(&pDW->haptics_work);
			atomic_set(&pDW->state, 1);
			schedule_work(&pDW->haptics_work);
		} else {
			pr_info("%s [VIB] cancel stop timer #2...\n",__func__);
			hrtimer_cancel(&pDW->stop_timer);
			cancel_work_sync(&pDW->haptics_work);
		}
	} else if (buf[0] == '8') {
	} else if (buf[0] == '9') {
	} else if (buf[0] == 'a') {
	} else if (buf[0] == 'p' && buf[1] == 'l'  && buf[2] == 'a' && buf[3] == 'y') {
		gprintk(" Echo play !\n");
		i2c_smbus_write_byte_data(pDW->dwclient, 0x03, 0x01);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x04);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0d, 0x89);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0e, 0x04);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x0f, 0x87);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x10, 0x01);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x11, 0xb1);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x12, 0x02);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x13, 0x00);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x14, 0x01);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x15, 0x01);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x16, 0x00);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x17, 0x00);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
	} else {
		i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
	}

	return count;
}

static ssize_t dw_haptics_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw7912_priv *pDW = Gdw7912;
	int ret = 0;
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x01);
	gprintk("chip status : %x\n", ret);

	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x1b);
	gprintk("RAM_ADDRH : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x1c);
	gprintk("RAM_ADDRL : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x08);
	gprintk("VD_CLAMP : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x23); //step4 : boost offset set
	gprintk(" 0x23 status : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x03); //check 0x03
	gprintk(" 0x03 status : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x01);
	gprintk(" chip status : %x\n", ret);
	return snprintf(buf, PAGE_SIZE, "[VIB] status = %x\n", ret);
}

static ssize_t dw_haptics_show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	struct dw7912_priv *pDW = Gdw7912;
	bool enable;

	enable = atomic_read(&pDW->realstate);

	return snprintf(buf, PAGE_SIZE, "%d\n", enable ? 1 : 0);
}

static ssize_t dw_haptics_store_state(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t dw_haptics_show_duration(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw7912_priv *pDW = Gdw7912;
	ktime_t time_rem;
	s64 time_us = 0;

	if (hrtimer_active(&pDW->stop_timer)) {
		time_rem = hrtimer_get_remaining(&pDW->stop_timer);
		time_us = ktime_to_us(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_us / 1000);
}

static ssize_t dw_haptics_store_duration(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	int rc;
	u32 val;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 100000 && val != 200000 && val != 300000 && val != 400000 && val != 410000 && val != 500000 && val > HAP_MAX_PLAY_TIME_MS)
		return -EINVAL;
	else
		pDW->vibTime = val;

	gprintk("time=%u\n", val);
#ifdef CONFIG_UCI_NOTIFICATIONS
        pr_info("%s [CLEANSLATE] playtime duration %d\n",__func__,val);
        if (val >= MIN_TD_VALUE_NOTIFICATION) {
                notification_duration_detected = 1;
                if (should_boost_call_alarm_always(val) || (smart_get_boost_on() && !should_not_boost())) { // raise voltage to boosted value in case of notification durations...
			boost_voltage(true, is_call_or_alarm(val));
                } else {
			boost_voltage(false,false);
		}
        } else {
                notification_duration_detected = 0;
		boost_voltage(false,false);
        }
#endif
	return count;
}

static ssize_t dw_haptics_show_activate(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t dw_haptics_store_activate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;

	int ret = 0;
	u32 val;

	if(is_bc_running()) {
		gprintk("BC status is running! Skip vibrator access!");
		return 0;
	}

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0) {
		gprintk("%s: return value %d\n", __func__, ret);
		return ret;
	}
	gprintk("%s\n", val ?"enable" :"disable");

	pr_info("%s [VIB] cancel stop timer...\n",__func__);
	hrtimer_cancel(&pDW->stop_timer);
	cancel_work_sync(&pDW->haptics_work);

	mutex_lock(&pDW->play_lock);
	if (val && (pDW->vibTime > 0)) {
#ifdef CONFIG_UCI_NOTIFICATIONS
		if (pDW->vibTime <= HAP_MAX_PLAY_TIME_MS) { // skip special clickity button vibs...
			ntf_vibration(pDW->vibTime);
		}
#endif
		pr_info("%s [VIB] atomic set 1...\n",__func__);
		atomic_set(&pDW->state, 1);
	} else {
		pr_info("%s [VIB] atomic set 0...\n",__func__);
		atomic_set(&pDW->state, 0);
		pDW->vibTime = 0;
	}
	schedule_work(&pDW->haptics_work);
	mutex_unlock(&pDW->play_lock);

	return count;
}

static ssize_t dw_haptics_show_register(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	struct dw7912_priv *pDW = Gdw7912;
	int ret = 0, i = 0;
	uint32_t buf_size = 0;

	for (i = 0x0; i <= 0x1F; i++) {
		ret = i2c_smbus_read_byte_data(pDW->dwclient, i);
		snprintf(buf+(i*10), 10, "REG%x=%x\n", i, ret);
		buf_size += 10;
		gprintk("I2C READ 0x%x from DRV2624 REG 0x%2x\n", ret, i);
	}
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x23);
	snprintf(buf+(i*10), 10, "REG23=%x\n", ret);
	buf_size += 10;
	gprintk("I2C READ 0x%x from DRV2624 REG 0x23\n", ret);

	return buf_size;
}

static ssize_t dw_haptics_store_register(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t reg = 0,val = 0;
	struct dw7912_priv *pDW = Gdw7912;
	int ret = 0;

	sscanf(buf, "%x %x", &reg, &val);
	ret = i2c_smbus_write_byte_data(pDW->dwclient, reg, val);
	if (ret < 0)
		gprintk("vib_settings_store REG%x failed\n", reg);
	else
		gprintk("vib_settings_store REG%x = %x\n", reg, i2c_smbus_read_byte_data(pDW->dwclient, reg));

	return count;
}


static ssize_t dw_haptics_show_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw7912_priv *pDW = Gdw7912;

	return snprintf(buf, PAGE_SIZE, "en : %d\n", gpio_get_value(pDW->en));
}

static ssize_t dw_haptics_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	if (buf[0] == '1') {
		gprintk(" reset 1 \n");
		gpio_direction_output(pDW->en, 1);
	} else if (buf[0] == '0') {
		gprintk(" reset 0 \n");
		gpio_direction_output(pDW->en, 0);
	} else if (buf[0] == 'a') {
		memory_mode_play(pDW, mem_header1, mem_wave1, sizeof(mem_wave1));
	} else if (buf[0] == 'b') {
		memory_mode_play(pDW, mem_header2, mem_wave2, sizeof(mem_wave2));
	} else if (buf[0] == 'c') {
		memory_mode_play(pDW, mem_header3, mem_wave3, sizeof(mem_wave3));
	} else if (buf[0] == 'd') {
		memory_mode_play(pDW, mem_header4, mem_wave4, sizeof(mem_wave4));
	} else if (buf[0] == 'e') {
		memory_mode_play(pDW, mem_header5, mem_wave5, sizeof(mem_wave5));
	} else if (buf[0] == 'T') {
		gprintk(" set trig2 raise \n");
		i2c_smbus_write_byte_data(pDW->dwclient, 0x19, 0x01);
	} else if (buf[0] == 'v') {
		gprintk(" set vd calmp to 0xFA\n");
		i2c_smbus_write_byte_data(pDW->dwclient, 0x08, 0xFA);
	} else if (buf[0] == 's' && buf[1] == 'w' && buf[2] == 'r'&& buf[3] == 's'&& buf[4] == 't') {
		gprintk(" set sw reset\n");
		i2c_smbus_write_byte_data(pDW->dwclient, 0x2F, 0x01);
	}

	return count;
}

static ssize_t dw_haptics_show_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	struct dw7912_priv *pDW = Gdw7912;

	return snprintf(buf, PAGE_SIZE, "calibration: %u\n", pDW->calFreq);
}

static ssize_t dw_haptics_store_calibration(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	int rc;
	u32 val;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val >= 140 && val <= 160) {
		pr_info("%s [VIB] cancel stop timer...\n",__func__);
		hrtimer_cancel(&pDW->stop_timer);
		cancel_work_sync(&pDW->haptics_work);

		mutex_lock(&pDW->play_lock);
		if (atomic_read(&pDW->state)) {
			atomic_set(&pDW->state, 0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x00);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0d, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0e, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x0f, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x10, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x11, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x12, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x13, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x14, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x15, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x16, 0x0);
			i2c_smbus_write_byte_data(pDW->dwclient, 0x17, 0x0);
			gprintk("set state 0\n");
		}
		waveform_freq_set(pDW, val);

		i2c_smbus_write_byte_data(pDW->dwclient, 0x0c, 0x06);// memory set 6 is calibration usage
		i2c_smbus_write_byte_data(pDW->dwclient, 0x14, 0x0F);
		i2c_smbus_write_byte_data(pDW->dwclient, 0x09, 0x01);
		hrtimer_start(&pDW->stop_timer, ktime_set(500 / MSEC_PER_SEC, (500 % MSEC_PER_SEC) * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		atomic_set(&pDW->state, 1);
		mutex_unlock(&pDW->play_lock);
		gprintk("%s: freq: %u\n", __func__, val);
	} else {
		gprintk("%s: incorrect input value %u\n", __func__, val);
	}

	return count;
}

static ssize_t dw_haptics_show_applyCalibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	struct dw7912_priv *pDW = Gdw7912;

	return snprintf(buf, PAGE_SIZE, "calibration: %u\n", pDW->calFreq);
}

static ssize_t dw_haptics_store_applyCalibration(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	int rc;
	u32 val;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val >= 140 && val <= 160) {
		waveform_freq_set(pDW, val);
		pDW->calFreq = val;
	}

	gprintk("Apply calibration freq value %d\n", pDW->calFreq);

	return count;
}

static ssize_t dw_haptics_show_dynamicLoadingWF(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */

	return snprintf(buf, PAGE_SIZE, "DIY\n");
}

static ssize_t dw_haptics_store_dynamicLoadingWF(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	struct file *filp = NULL;
	u8 *cpy_head_data = NULL, *cpy_data = NULL;
	mm_segment_t fs;
	u8 buffer[2];
	u8 temp[1];
	int i = 0;
	int realNum = 0;
	loff_t temp_pos;

	fs = get_fs();
	set_fs(KERNEL_DS);
	if ((filp = filp_open(HEADER_LOCATION, O_RDWR | O_CREAT, 0644)) == NULL) {
		gprintk("Cannot open file %s\n", HEADER_LOCATION);
		return count;
	}
	if (IS_ERR(filp)) {
		gprintk("incorrect filp %d\n", IS_ERR(filp));
		return count;
	}
	set_fs(fs);
	memset(buffer, 0, sizeof(buffer));

	filp->f_op->llseek(filp, 0, SEEK_END);
	temp_pos = filp->f_pos;
	filp->f_pos = 0x00;
	cpy_head_data = kzalloc(sizeof(u8) * ((unsigned long)temp_pos / 4), GFP_KERNEL);
	if (!cpy_head_data) {
		gprintk("head data allocate fail\n");
		filp_close(filp, NULL);
		return count;
	}

	while (filp->f_pos < temp_pos) {
		fs = get_fs();
		set_fs(KERNEL_DS);
		vfs_read(filp, temp, 1, &filp->f_pos);
		set_fs(fs);
		if (i < 2) {
			if ((temp[0] >= '0') && (temp[0] <= '9')) {
				buffer[i] = temp[0] - '0';
			} else if ((temp[0] >= 'A') && (temp[0] <= 'F')) {
				buffer[i] = temp[0] - 'A' + 10;
			}
		}
		if (((i % 3) == 0) && i > 2) {
			cpy_head_data[realNum] = buffer[0] << 4 | buffer[1];
			gprintk("real header data: %X\n", cpy_head_data[realNum]);
			i = 0;
			++realNum;
		} else
			++i;
	}
	gprintk("length: %lu  real num: %d\n", (unsigned long)temp_pos, realNum);

	filp_close(filp, NULL);

	fs = get_fs();
	set_fs(KERNEL_DS);
	if ((filp = filp_open(WAVEFORM_LOCATION, O_RDWR | O_CREAT, 0644)) == NULL) {
		gprintk("Cannot open file %s\n", HEADER_LOCATION);
		goto file_op_fail;
	}
	if (IS_ERR(filp)) {
		gprintk("incorrect filp %d\n", IS_ERR(filp));
		goto file_op_fail;
	}
	set_fs(fs);
	memset(buffer, 0, sizeof(buffer));

	filp->f_op->llseek(filp, 0, SEEK_END);
	temp_pos = filp->f_pos;
	filp->f_pos = 0x00;
	cpy_data = kzalloc(sizeof(u8) * ((unsigned long)temp_pos / 4), GFP_KERNEL);
	if (!cpy_data) {
		gprintk("waveform data allocate fail\n");
		goto memory_allocate_fail;
	}

	realNum = 0;
	i = 0;
	while (filp->f_pos < temp_pos) {
		fs = get_fs();
		set_fs(KERNEL_DS);
		vfs_read(filp, temp, 1, &filp->f_pos);
		set_fs(fs);
		if (i < 2) {
			if ((temp[0] >= '0') && (temp[0] <= '9')) {
				buffer[i] = temp[0] - '0';
			} else if ((temp[0] >= 'A') && (temp[0] <= 'F')) {
				buffer[i] = temp[0] - 'A' + 10;
			}
		}
		if (((i % 3) == 0) && i > 2) {
			cpy_data[realNum] = buffer[0] << 4 | buffer[1];
			gprintk("real waveform data: %X\n", cpy_data[realNum]);
			i = 0;
			++realNum;
		} else
			++i;
	}
	gprintk("length: %lu  real num: %d\n", (unsigned long)temp_pos, realNum);

	if (memory_mode_play(pDW, cpy_head_data, cpy_data, realNum) < 0) {
		gprintk("Waveform setting fail\n");
	} else {
		gprintk("Waveform setting done\n");
	}

	kfree(cpy_data);
memory_allocate_fail:
	filp_close(filp, NULL);
file_op_fail:
	kfree(cpy_head_data);

	return count;
}

static ssize_t dw_haptics_show_bc_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw7912_priv *pDW = Gdw7912;

	return snprintf(buf, PAGE_SIZE, "%d\n", pDW->bc_status);
}

static ssize_t dw_haptics_store_bc_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = Gdw7912;
	int ret;
	u32 value;
	ret = kstrtouint(buf, 0, &value);
	if(ret < 0)
		return ret;

	mutex_lock(&pDW->play_lock);
	if(value) {
		hrtimer_cancel(&pDW->stop_timer);
		cancel_work_sync(&pDW->haptics_work);
		pDW->bc_status = 1;
	} else {
		pDW->bc_status = 0;
	}

	mutex_unlock(&pDW->play_lock);
	gprintk("Set bc_stattus to %d\n", pDW->bc_status);

	return count;
}

static struct device_attribute dw_haptics_attrs[] = {
	__ATTR(state, 0664, dw_haptics_show_state, dw_haptics_store_state),
	__ATTR(duration, 0664, dw_haptics_show_duration, dw_haptics_store_duration),
	__ATTR(activate, 0664, dw_haptics_show_activate, dw_haptics_store_activate),
	__ATTR(enable, 0664, dw_haptics_show_enable, dw_haptics_store_enable),
	__ATTR(waveform, 0664, dw_haptics_show_waveform, dw_haptics_store_waveform),
	__ATTR(regs, 0664, dw_haptics_show_register, dw_haptics_store_register),
	__ATTR(reset, 0664, dw_haptics_show_reset, dw_haptics_store_reset),
	__ATTR(calibration, 0664, dw_haptics_show_calibration, dw_haptics_store_calibration),
	__ATTR(applyK, 0664, dw_haptics_show_applyCalibration, dw_haptics_store_applyCalibration),
	__ATTR(DIYwf, 0664, dw_haptics_show_dynamicLoadingWF, dw_haptics_store_dynamicLoadingWF),
	__ATTR(BC_status,0664, dw_haptics_show_bc_status, dw_haptics_store_bc_status),
};

/* Dummy functions for brightness */
static
enum led_brightness dw_haptics_brightness_get(struct led_classdev *cdev)
{
	return 0;
}

static void dw_haptics_brightness_set(struct led_classdev *cdev,
		enum led_brightness level)
{
}

int real_time_playback_mode_test(struct dw7912_priv *p)
{
//	u8 ack;
//	u8 rdata[8], wdata[8];

//	int rc;
	struct i2c_msg xfer[3];
	//u8  reg_offset = 0x0a;

	struct i2c_client *c = p->dwclient;

#if 0
	ack = i2c_smbus_read_byte_data(c, 0x01);
	gprintk("check status 0x01 = 0x%02x\n", ack);

	if (ack != 0 ) {
		printk("status error\n");
		return -1;
	}
#endif

	i2c_smbus_write_byte_data(c, 0x08, 0x96); //step3 : vd-calmp set 6v
//	i2c_smbus_write_byte_data(c, 0x23, 0x06); //step4 : boost offset set

//	printk("rtp_wave1 size = %lu\n", sizeof(rtp_wave1) );

	//step6 : write rtp data
	// address write
	xfer[0].addr  = c->addr,
	xfer[0].len   = 162; // register address + 2 sign wave data
//	xfer[0].buf   = rtp_wave1;

/*	rc = i2c_transfer(c->adapter, xfer, 1);
	if (rc < 0) {
	        dev_err(&c->dev, "register write failed; reg=0x0a, size=%lu\n", sizeof(rtp_wave1));
		return -EIO;
	}*/

	i2c_smbus_write_byte_data(c, 0x09, 0x01); //step5 : play back

	gprintk("rtp data write done\n");

	return 0;
}

int process_done_check(struct dw7912_priv *p, int msec, int waittime_msec)
{
	u8 ack = 0;
	int ret = 0, limit = 0;

	unsigned long t, w;
	struct i2c_client *c = p->dwclient;

	t = msecs_to_jiffies(msec);

	while (1) {
		schedule_timeout_interruptible(t);

		ack = i2c_smbus_read_byte_data(c, 0x01);
		gprintk("check status 0x01 = 0x%02x\n", ack);
		if (ack & 0x10) {
			printk("process done\n");
			ret = 0;
			break;
		}

		// loop limit event
		if (limit > 1000) { // 1 sec
			limit = 0;
			ret = -1;
			break;
		} else
			limit++;
	}

	i2c_smbus_write_byte_data(c, 0x09, 0x01); //step5 : play back

	if (waittime_msec > 0) {
		w = msecs_to_jiffies(waittime_msec);
		schedule_timeout_interruptible(w);
	}

	return ret;
}

int real_time_playback(struct dw7912_priv *p, u8 *data, u32 size)
{
	//u8 ack;
	int rc;
	//struct i2c_msg xfer[3];
	struct i2c_client *c = p->dwclient;

	uint8_t buf[256];
	struct i2c_msg msg[] = {
		{
			.addr = c->addr,
			.flags = 0,
			.len = size,
			.buf = buf,
		}
	};

	gprintk("REAL TIME PLAYBACK+++\n");

	gprintk("data size = %u, addr: %x\n", size, c->addr);

	//i2c_smbus_write_byte_data(c, 0x08, 0x96); //step3 : vd-calmp set 6v
	//i2c_smbus_write_byte_data(c, 0x23, 0x06); //step4 : boost offset set

	memcpy(buf, data, size);
	rc = i2c_transfer(c->adapter, msg, 1);
	if (rc < 0) {
		gprintk("register write failed; reg=0x0a, size=%u\n", size);
	}
	gprintk("rtp data write done\n");

	return 0;
}

#ifdef CONFIG_INPUT_DW7912_VOLTAGE_SWITCH_WA
void haptics_voltage_switch(bool enabled) {
	struct dw7912_priv *pDW = Gdw7912;
	u8 *cpy_header = NULL;

	if(is_bc_running()) {
		gprintk("BC status is running! Skip vibrator access!");
		return ;
	}

	if (enabled == false) {
#ifdef CONFIG_UCI_NOTIFICATIONS
		boost_reset = true;
#endif
		gprintk("VS: original %x %x\n", mem_header1[7], mem_cali_header6[pDW->defaultFreq - 140][7]);
		if (memory_mode_play(pDW, mem_header1, mem_wave1, sizeof(mem_wave1)) < 0) {
			gprintk("Waveform setting fail\n");
		} else if (pDW->calFreq == 0) {
			waveform_freq_set(pDW, pDW->defaultFreq);
			gprintk("Device did not calibration, using default frequency");
		} else {
			waveform_freq_set(pDW, pDW->calFreq);
		}
	} else if (enabled == true) {
#ifdef CONFIG_UCI_NOTIFICATIONS
		boost_reset = false;
#endif
		gprintk("VS: %x %x\n", pDW->sideKeys_Voltage, pDW->notification_Voltage);
		cpy_header = kzalloc(sizeof(u8) * 8, GFP_KERNEL);
		memcpy(cpy_header, mem_header1, 8);
		cpy_header[7] = pDW->sideKeys_Voltage;
		if (memory_mode_play(pDW, cpy_header, mem_wave1, sizeof(mem_wave1)) < 0 ) {
			gprintk("Waveform setting fail\n");
		} else if (pDW->calFreq == 0) {
			memcpy(cpy_header, mem_cali_header6[pDW->defaultFreq - 140], 8);
			cpy_header[7] = pDW->notification_Voltage;
			if (memory_mode_play( pDW, cpy_header, mem_cali_wave6[pDW->defaultFreq - 140],
					((cpy_header[5] & 0x0F) << 8) + cpy_header[6]) < 0) {
				gprintk("Waveform setting fail\n");
			} else {
				gprintk("Device did not calibration, using default frequency");
			}
		} else {
			memcpy(cpy_header, mem_cali_header6[pDW->calFreq - 140], 8);
			cpy_header[7] = pDW->notification_Voltage;
			if (memory_mode_play( pDW, cpy_header, mem_cali_wave6[pDW->calFreq - 140],
					((cpy_header[5] & 0x0F) << 8) + cpy_header[6]) < 0) {
				gprintk("Waveform setting fail\n");
			}
		}
	}
}
extern void haptics_voltage_switch(bool enabled);
#endif

#ifdef CONFIG_VIB_TRIGGERS
static void dw7912_vib_trigger_enable(struct vib_trigger_enabler *enabler, int value)
{
	struct dw7912_priv *pDW = Gdw7912;
	pDW->vibTime = value;
	gprintk("trg=%d\r\n", value);

	if (value && (pDW->vibTime > 0)) {
		pr_info("%s [VIB] atomic set 1...\n",__func__);
		atomic_set(&pDW->state, 1);
	} else {
		pr_info("%s [VIB] atomic set 0...\n",__func__);
		atomic_set(&pDW->state, 0);
		pDW->vibTime = 0;
	}
	schedule_work(&pDW->haptics_work);

}
#ifdef CONFIG_UCI

static void vibstop_work_func(struct work_struct *vibstop_work) {
	struct dw7912_priv *pDW = Gdw7912;

	pr_info("%s [VIB] cancel stop timer...\n",__func__);
	hrtimer_cancel(&pDW->stop_timer);
	cancel_work_sync(&pDW->haptics_work);

	mutex_lock(&pDW->play_lock);
	pr_info("%s [VIB] atomic set 0...\n",__func__);
	atomic_set(&pDW->state, 0);
	pDW->vibTime = 0;

	schedule_work(&pDW->haptics_work);
	mutex_unlock(&pDW->play_lock);
}
DECLARE_WORK(vibstop_work,vibstop_work_func);

static struct alarm vibstop_rtc;

static enum alarmtimer_restart vibstop_rtc_callback(struct alarm *al, ktime_t now)
{
	schedule_work(&vibstop_work);
        return ALARMTIMER_NORESTART;
}

static void dw7912_vib_trigger_reset_enable(struct vib_trigger_enabler *enabler, int value)
{
	struct dw7912_priv *pDW = Gdw7912;
	pDW->vibTime = value;
	gprintk("trg=%d\r\n", value);

	pr_info("%s [VIB] cancel stop timer...\n",__func__);
	hrtimer_cancel(&pDW->stop_timer);
	cancel_work_sync(&pDW->haptics_work);

	mutex_lock(&pDW->play_lock);
	if (value && (pDW->vibTime > 0)) {
		pr_info("%s [VIB] atomic set 1...\n",__func__);
		atomic_set(&pDW->state, 1);
		vibrator_trigger_count = 3; // set count to 3, so in stop work even if enable seems 0, it will start off work...
	} else {
		pr_info("%s [VIB] atomic set 0...\n",__func__);
		atomic_set(&pDW->state, 0);
		pDW->vibTime = 0;
	}
	schedule_work(&pDW->haptics_work);
	mutex_unlock(&pDW->play_lock);
	{
/*                ktime_t wakeup_time;
                ktime_t curr_time = { .tv64 = 0 };
                wakeup_time = ktime_add_us(curr_time,
                        (sleeptime * 1000LL)); // msec to usec*/
                alarm_cancel(&vibstop_rtc); // stop pending alarm...
                alarm_start_relative(&vibstop_rtc, ktime_set((pDW->vibTime-5) / MSEC_PER_SEC, ((pDW->vibTime-5) % MSEC_PER_SEC) * NSEC_PER_MSEC)); // start new...
	}
}
#endif
#endif

#ifdef CONFIG_OF
static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_priv *p)
{
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;
	struct property *prop;
	int size = 0, i = 0;

	if (!np)
		return -1;
#ifdef CONFIG_UCI_NOTIFICATIONS
	g_p = p;
#endif
	p->en = of_get_named_gpio(np, "dw7912,en-gpio", 0);
	if (p->en < 0) {
		printk("Looking up %s property in node %s failed %d\n",
				"dw7912,en-gpio", dev->of_node->full_name,
				p->en);
		p->en = -1;
		return -1;
	} else if (!gpio_is_valid(p->en)) {
		printk(KERN_ERR "dw7912 en pin(%u) is invalid\n", p->en);
		return -1;
	} else {
		gpio_direction_output(p->en, 0);
		mdelay(10);
		gpio_direction_output(p->en, 1);
		mdelay(10);
		gpio_direction_output(p->en, 0);
		mdelay(10);
		gprintk(" p->en = %u, value %d\n", p->en, gpio_get_value(p->en));
	}

	if (of_property_read_u32(np, "dw7912,freq", &p->defaultFreq) < 0)
	{
		printk("Looking up %s property in node %s failed %d\n",
				"dw7912,freq", dev->of_node->full_name,
				p->defaultFreq);
		p->defaultFreq = -1;
		return -1;
	} else {
		gprintk("Read default freq %u from device tree\n", p->defaultFreq);
	}

#ifdef CONFIG_INPUT_DW7912_VOLTAGE_SWITCH_WA
	if (of_property_read_u32(np, "dw7912,side-keys-VL", &p->sideKeys_Voltage) < 0)
	{
		gprintk("Looking up %s property in node %s failed %X\n",
				"dw7912,side-keys-VL", dev->of_node->full_name,
				p->sideKeys_Voltage);
		p->sideKeys_Voltage = 0;
	} else {
		gprintk("Read sideKeys Voltage %X from device tree\n", p->sideKeys_Voltage);
#ifdef CONFIG_UCI_NOTIFICATIONS
		original_sidekeys_voltage = p->sideKeys_Voltage;
#endif
	}

	if (of_property_read_u32(np, "dw7912,noti-VL", &p->notification_Voltage) < 0)
	{
		gprintk("Looking up %s property in node %s failed %X\n",
				"dw7912,noti-VL", dev->of_node->full_name,
				p->notification_Voltage);
		p->notification_Voltage = 0;
	} else {
		gprintk("Read notification Voltage %X from device tree\n", p->notification_Voltage);
#ifdef CONFIG_UCI_NOTIFICATIONS
		original_notif_voltage = p->notification_Voltage;
#endif
	}
#endif

	prop = of_find_property(np, "memory-mode-header-1", &size);
	if (prop) {
		memcpy(p->mem_header_1, prop->value, size);
		gprintk(" input size: %d, mem_header size = %lu\n", size, sizeof(p->mem_header_1));
		gprintk(" Mem_header_1\n");
		for (i = 0; i < size; i++)
			gprintk("Mem_header_1[%d]: %x\n", i, p->mem_header_1[i]);
	}
	prop = of_find_property(np, "memory-mode-wave-1", &size);
	if (prop) {
		mem_wave_1 = devm_kzalloc(dev, (size)*sizeof(u8), GFP_KERNEL);
		p->waveSize1 = size;
		if (mem_wave_1 == NULL) {
			gprintk(" kzalloc fail\n");
		} else {
			memcpy(mem_wave_1, prop->value, size);
			gprintk(" input size: %d, mem_wave_1 size = %lu\n", p->waveSize1, sizeof(&mem_wave_1));
		}
	}

	prop = of_find_property(np, "memory-mode-header-2", &size);
	if (prop) {
		memcpy(p->mem_header_2, prop->value, size);
		gprintk(" input size: %d, mem_header size = %lu\n", size, sizeof(p->mem_header_2));
		gprintk(" Mem_header_2\n");
		for (i = 0; i < size; i++)
			gprintk("Mem_header_2[%d]: %x\n", i, p->mem_header_2[i]);
	}
	prop = of_find_property(np, "memory-mode-wave-2", &size);
	if (prop) {
		mem_wave_2 = devm_kzalloc(dev, (size)*sizeof(u8), GFP_KERNEL);
		p->waveSize2 = size;
		if (mem_wave_2 == NULL) {
			gprintk(" kzalloc fail\n");
		} else {
			memcpy(mem_wave_2, prop->value, size);
			gprintk(" input size: %d, mem_wave_2 size = %lu\n", p->waveSize2, sizeof(&mem_wave_2));
		}
	}

	prop = of_find_property(np, "memory-mode-header-3", &size);
	if (prop) {
		memcpy(p->mem_header_3, prop->value, size);
		gprintk(" input size: %d, mem_header_3 size = %lu\n", size, sizeof(p->mem_header_3));
		gprintk(" Mem_header_3\n");
		for (i = 0; i < size; i++)
			gprintk("Mem_header_3[%d]: %x\n", i, p->mem_header_3[i]);
	}
	prop = of_find_property(np, "memory-mode-wave-3", &size);
	if (prop) {
		mem_wave_3 = devm_kzalloc(dev, (size)*sizeof(u8), GFP_KERNEL);
		p->waveSize3 = size;
		if (mem_wave_3 == NULL) {
			gprintk(" kzalloc fail\n");
		} else {
			memcpy(mem_wave_3, prop->value, size);
			gprintk(" input size: %d, mem_wave_3 size = %lu\n", p->waveSize3, sizeof(&mem_wave_3));
		}
	}

	return 0;
}
#else
static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_priv *p)
{
	return NULL;
}
#endif

static void __init dw7912_waveform_async(void *unused, async_cookie_t cookie)
{
	struct dw7912_priv *pDW = Gdw7912;

	gprintk("Memory mode, set wave data\n");
	memory_mode_play(pDW, mem_header1, mem_wave1, sizeof(mem_wave1));
	memory_mode_play(pDW, mem_header2, mem_wave2, sizeof(mem_wave2));
	memory_mode_play(pDW, mem_header3, mem_wave3, sizeof(mem_wave3));
	memory_mode_play(pDW, mem_header4, mem_wave4, sizeof(mem_wave4));
	memory_mode_play(pDW, mem_header5, mem_wave5, sizeof(mem_wave5));
	mem_cali_wave6[0] = mem_cali_wave6_140;
	mem_cali_wave6[1] = mem_cali_wave6_141;
	mem_cali_wave6[2] = mem_cali_wave6_142;
	mem_cali_wave6[3] = mem_cali_wave6_143;
	mem_cali_wave6[4] = mem_cali_wave6_144;
	mem_cali_wave6[5] = mem_cali_wave6_145;
	mem_cali_wave6[6] = mem_cali_wave6_146;
	mem_cali_wave6[7] = mem_cali_wave6_147;
	mem_cali_wave6[8] = mem_cali_wave6_148;
	mem_cali_wave6[9] = mem_cali_wave6_149;
	mem_cali_wave6[10] = mem_cali_wave6_150;
	mem_cali_wave6[11] = mem_cali_wave6_151;
	mem_cali_wave6[12] = mem_cali_wave6_152;
	mem_cali_wave6[13] = mem_cali_wave6_153;
	mem_cali_wave6[14] = mem_cali_wave6_154;
	mem_cali_wave6[15] = mem_cali_wave6_155;
	mem_cali_wave6[16] = mem_cali_wave6_156;
	mem_cali_wave6[17] = mem_cali_wave6_157;
	mem_cali_wave6[18] = mem_cali_wave6_158;
	mem_cali_wave6[19] = mem_cali_wave6_159;
	mem_cali_wave6[20] = mem_cali_wave6_160;
	waveform_freq_set(pDW, pDW->defaultFreq);

	gprintk(" Set TRIG 2\n");
	i2c_smbus_write_byte_data(pDW->dwclient, 0x19, 0x01);

}

#ifdef CONFIG_UCI
// TODO
struct vib_trigger_enabler *g_enabler = NULL;

static int set_val = 0;
static void set_vibrate_work_func(struct work_struct *set_vibrate_work) {
    int val = set_val;
    int power_perc = uci_get_vibration_power_percentage();
    pr_info("%s [CLEANSLATE] power_perc = %d val = %d\n",__func__,power_perc,val);

    if (power_perc == 0) return;
    if (val == 0) return;
    if (g_enabler) {
        dw7912_vib_trigger_reset_enable(g_enabler,val);
    }
}
DECLARE_WORK(set_vibrate_work,set_vibrate_work_func);

void set_vibrate(int val)
{
	boost_voltage(false,false);
	if (g_enabler) {
		set_val = val;
		schedule_work(&set_vibrate_work);
	}
}
EXPORT_SYMBOL(set_vibrate);

void set_vibrate_boosted(int val) {
        if (smart_get_boost_on() && !should_not_boost()) { // raise voltage to boosted value in case of notification durations...
		boost_voltage(true,is_call_or_alarm(val));
        } else {
		boost_voltage(false,false);
	}
	if (g_enabler) {
		set_val = val;
		schedule_work(&set_vibrate_work);
	}
}
EXPORT_SYMBOL(set_vibrate_boosted);

#endif
static int dw7912_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	struct dw7912_priv *dw7912;
	int ret = -1;
	int i = 0;

	gprintk("+++Probe+++\n");

	dw7912 = kzalloc(sizeof(struct dw7912_priv), GFP_KERNEL);
	if (dw7912 == NULL)
		return -ENOMEM;

	if (client->dev.of_node) {
		gprintk("Read en pin from device tree\n");
		ret = dw7912_i2c_parse_dt(client, dw7912);
		if (ret < 0) {
			gprintk("en gpio pin dt parse error\n");
			goto get_en_pin_fail;
		}
	} else {
		gprintk("dev.of_node does not exist\n");
		dw7912->en = -1;
		goto get_en_pin_fail;
	}

	i2c_set_clientdata(client, dw7912);
	dw7912->dwclient = client; /* i2c client pointer save */

	INIT_WORK(&dw7912->haptics_work, dw7912_haptics_work);
	mutex_init(&dw7912->play_lock);

	hrtimer_init(&dw7912->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dw7912->stop_timer.function = hap_stop_timer;

	dw7912->cdev.name = "vibrator";
	dw7912->cdev.brightness_get = dw_haptics_brightness_get;
	dw7912->cdev.brightness_set = dw_haptics_brightness_set;
	dw7912->cdev.max_brightness = 100;
	ret = devm_led_classdev_register(&dw7912->dwclient->dev, &dw7912->cdev);
	if (ret < 0) {
		gprintk("Error in registering led class device, rc=%d\n", ret);
		goto register_fail;
	}
	for (i = 0; i < ARRAY_SIZE(dw_haptics_attrs); i++) {
		ret = sysfs_create_file(&dw7912->cdev.dev->kobj, &dw_haptics_attrs[i].attr);
		if (ret < 0) {
			gprintk("Error in creating sysfs file, rc=%d\n", ret);
			//goto sysfs_fail;
		}
	}

#ifdef CONFIG_VIB_TRIGGERS
	dw7912->enabler.name = "haptic-dw7912";
	dw7912->enabler.default_trigger = "vibrator";
	dw7912->enabler.enable = dw7912_vib_trigger_enable;
	dw7912->enabler.trigger_data = dw7912;
	vib_trigger_enabler_register(&dw7912->enabler);
#ifdef CONFIG_UCI
	g_enabler = &dw7912->enabler;
	uci_add_user_listener(uci_user_listener);
	uci_add_sys_listener(uci_sys_listener);
        alarm_init(&vibstop_rtc, ALARM_REALTIME,
            vibstop_rtc_callback);
#endif
#endif

	ret = i2c_smbus_read_byte_data(dw7912->dwclient, 0x00);
	gprintk("chip ID : %x\n", ret);

	ret = i2c_smbus_read_byte_data(dw7912->dwclient, 0x01);
	gprintk("chip status : %x\n", ret);

	i2c_smbus_write_byte_data(dw7912->dwclient, 0x2F, 0x01); //SW reset
	gprintk("Set SW reset, clear all registers data\n");

	i2c_smbus_write_byte_data(dw7912->dwclient, 0x23, 0x06); //boost offset set
	gprintk("Set 0x23(boost_option) to 0x06\n");

	dw7912->bc_status = 0;
	gprintk("Set bc_stattus to %d\n", dw7912->bc_status);

	Gdw7912 = dw7912;
	async_schedule(dw7912_waveform_async, NULL);

	gprintk("---ProbeSuccess---\n");
	return 0;

get_en_pin_fail:
register_fail:
	cancel_work_sync(&dw7912->haptics_work);
	hrtimer_cancel(&dw7912->stop_timer);
	mutex_destroy(&dw7912->play_lock);
	i2c_set_clientdata(client, NULL);
	kfree(dw7912);

	return ret;
}

static int dw7912_i2c_remove(struct i2c_client *client)
{
	struct dw7912_priv *dw7912 = i2c_get_clientdata(client);

	if (dw7912->en > 0)
		gpio_free(dw7912->en);

	cancel_work_sync(&dw7912->haptics_work);
	hrtimer_cancel(&dw7912->stop_timer);
	mutex_destroy(&dw7912->play_lock);
#ifdef CONFIG_VIB_TRIGGERS
	vib_trigger_enabler_unregister(&dw7912->enabler);
#endif
	i2c_set_clientdata(client, NULL);
	kfree(dw7912);

	return 0;
}

#ifdef CONFIG_PM
static int dw7912_suspend(struct device *dev)
{
	return 0;
}

static int dw7912_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(dw7912_pm_ops,
			 dw7912_suspend, dw7912_resume);

#define DW7912_VIBRATOR_PM_OPS (&dw7912_pm_ops)
#else
#define DW7912_VIBRATOR_PM_OPS NULL
#endif


#ifdef CONFIG_OF
static struct of_device_id dw7912_i2c_dt_ids[] = {
	{ .compatible = "dwanatech,dw7912"},
	{ }
};
#endif

static const struct i2c_device_id dw7912_i2c_id[] = {
	{"dwanatech,dw7912", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, dw7912_i2c_id);

static struct i2c_driver dw7912_i2c_driver = {
	.probe = dw7912_i2c_probe,
	.remove = dw7912_i2c_remove,
	.id_table = dw7912_i2c_id,
	.driver = {
		.name = "dw7912-codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw7912_i2c_dt_ids),
#endif
#ifdef CONFIG_PM
		.pm	= DW7912_VIBRATOR_PM_OPS,
#endif
		},

};

static int __init dw7912_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&dw7912_i2c_driver);
	if (ret)
		pr_err("Failed to register dw7912 I2C driver: %d\n", ret);

	return ret;
}

module_init(dw7912_modinit);
//late_initcall(dw7912_modinit);
static void __exit dw7912_exit(void)
{
	i2c_del_driver(&dw7912_i2c_driver);
}

module_exit(dw7912_exit);

MODULE_DESCRIPTION("Vibrator DW7912 codec driver");
MODULE_AUTHOR("ghcstop@gmail.com");
MODULE_LICENSE("GPL");
