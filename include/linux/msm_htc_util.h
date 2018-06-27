/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2007 Google Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef HTC_UTIL_H
#define HTC_UTIL_H

#include <linux/list.h>

struct dsi_panel;
struct backlight_table_v1_0;

struct dsi_status_notifier {
	struct list_head dsi_notifier_link;
	const char *name;
	void (*func)(int status);
};

enum {
	LCM_EARLY_MIPI_OFF_CMD = 0,
	LCM_MIPI_OFF_CMD,
	LCM_EARLY_POWERDOWN,
	LCM_POWERDOWN,
	LCM_REST_EARLY_HIGH,
	LCM_REST_HIGH,
	LCM_REST_EARLY_LOW,
	LCM_REST_LOW,
	LCM_POWERON,
};

extern int dsi_register_notifier(struct dsi_status_notifier*);
extern void send_dsi_status_notify(int status);
extern void check_dsi_panel_data_used(struct dsi_panel *panel );
extern void check_dsi_panel_data_remove(struct dsi_panel *panel );
extern void htc_update_bl_cali_data(struct backlight_table_v1_0 *brt_bl_table);
extern bool panel_set_color_mode(u32 android_color_mode);
extern bool htc_is_burst_bl_on(struct dsi_panel *panel);

#endif /* HTC_UTIL_H */
