/*
 * Copyright (C) 2013 HTC Corporation.
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

#ifndef _NANOHUB_HTC_H
#define _NANOHUB_HTC_H


#define EVT_APP_FROM_HOST                 0x000000F8
#define EVT_NO_FIRST_SENSOR_EVENT         0x00000200
#define EVT_NO_SENSOR_CONFIG_EVENT        0x00000300
#define EVT_RESET_REASON                  0x00000403
#define SENS_TYPE_PROX                    13
#define SENS_TYPE_HALL                    29

#define SENS_TYPE_HTC_BASE        96
#ifdef CONFIG_NANOHUB_HTC_SENSTYPE_OFFSET
#define SENS_TYPE_HTC_EASY_ACCESS (SENS_TYPE_HTC_BASE + 2)
#define SENS_TYPE_HTC_TOUCH       (SENS_TYPE_HTC_BASE + 3)
#define SENS_TYPE_HTC_SECOND_DISP (SENS_TYPE_HTC_BASE + 4)
#define SENS_TYPE_HTC_TOUCH_POINT (SENS_TYPE_HTC_BASE + 7)
#define SENS_TYPE_HTC_EDGE        (SENS_TYPE_HTC_BASE + 8)
#define SENS_TYPE_HTC_EDWK        (SENS_TYPE_HTC_BASE + 9)
#else
#define SENS_TYPE_HTC_EASY_ACCESS         54
#define SENS_TYPE_HTC_TOUCH               55
#define SENS_TYPE_HTC_SECOND_DISP         56
#define SENS_TYPE_HTC_TOUCH_POINT         59
#define SENS_TYPE_HTC_EDGE                60
#define SENS_TYPE_HTC_EDWK                61
#endif
#define SENS_TYPE_HTC_EDGE_ULTRA  (SENS_TYPE_HTC_BASE + 10)
#define SENS_TYPE_HTC_EG_MONITOR  (SENS_TYPE_HTC_BASE + 11)

#define CONFIG_CMD_CFG_DATA               3
#define sensorGetMyEventType(_sensorType) (EVT_NO_FIRST_SENSOR_EVENT + (_sensorType))

#define APP_ID_MAKE(vendor, app)       ((((uint64_t)(vendor)) << 24) | ((app) & 0x00FFFFFF))
#define APP_ID_VENDOR_HTC       0x4854437470ULL /* "HTCtp" */
#define BMI160_APP_ID APP_ID_MAKE(APP_ID_VENDOR_HTC, 2)
#define SNT_APP_ID    APP_ID_MAKE(APP_ID_VENDOR_HTC, 22)

struct __attribute__ ((packed)) hal_cfg_data {
	uint8_t s_pole_near:1;
	uint8_t n_pole_near:1;
};


struct __attribute__ ((packed)) eza_cfg_data {
	uint8_t setting[4];
#if defined(CONFIG_NANOHUB_SECOND_DISP) || defined(CONFIG_NANOHUB_AOD)
	uint8_t lcd_mode;
#endif
};

struct __attribute__ ((packed)) tou_cfg_data {
	uint8_t status;
	uint8_t solution;
#if defined(CONFIG_NANOHUB_SECOND_DISP) || defined(CONFIG_NANOHUB_AOD)
	uint8_t mode;
#endif
};

struct __attribute__ ((packed)) snd_cfg_data {
	uint8_t switch_mcu;
	uint8_t cpu_suspend;
	uint16_t bl_ctrl;
};

struct __attribute__ ((packed)) pnt_cfg_data {
	uint8_t cpu_suspend;
	uint8_t switch_mcu;
	uint8_t lcd_mode;
};

struct __attribute__ ((packed)) edge_cfg_data {
	uint32_t threshold;
	uint8_t  usb_status;
	uint8_t  switch_mcu;
	uint8_t  key_status;
	uint8_t  edge_status; //0: before EVT2; 1: EVT2; 2: after EVT2
	uint8_t  reserved[1];
    uint32_t header;
    uint32_t i2c_switch;
};

#ifdef CONFIG_NANOHUB_EDGE_ULTRA
struct __attribute__ ((packed)) edge_ultra_cfg_data {
	uint8_t  threshold;
	uint8_t  vol_key_can_wake;      //bit mask of vol_wakeup_store
	uint16_t quick_launch_cam;      //msec, timeout for quick launch camera by double tap power keys
	uint8_t  dynamic_power_ctrl : 1;//1: enable; 0: disable
	uint8_t  offmode_charging : 1;	//1: offmode_charging; 0: not offmode_charging
	uint8_t  debug : 1;             //1: dump debug info; 0: do not dump debug info
	uint8_t  normal_mode : 1;       //1: normal boot; 0: boot from hosd
	uint8_t  cancel_force_reset : 1;//1: enable; 0: disable
	uint8_t  display_status : 1;	//1: display on; 0: display off
	uint8_t  reserved : 1;	        //reserved
	uint8_t  power_key_status : 1;	//1: power key pressed; 0: power key released
	uint8_t  usb_status;		//1: plug; 0: un-plug; 255: default
	int32_t  temperature;
};
#endif

#ifdef CONFIG_NANOHUB_EDGE_MONITOR_CONTROL
struct __attribute__ ((packed)) egm_cfg_data {
	uint8_t isRead;
	uint8_t reg;
	uint8_t value;
};
#endif

struct __attribute__ ((packed)) ConfigCmd {
	uint32_t evtType;
	uint64_t latency;
	uint32_t rate;
	uint8_t sensorType;
	uint8_t cmd;
	uint16_t flags;
	uint8_t data[];
};

struct __attribute__ ((packed)) HostHubRawPacket {
	uint64_t appId;
	uint8_t dataLen;
};

struct __attribute__ ((packed)) MsgCmd {
	uint32_t evtType;
	struct HostHubRawPacket msg;
};

struct __attribute__ ((packed)) HostCmd {
	struct MsgCmd command;
	uint8_t data[];
};

#ifdef CONFIG_NANOHUB_BYPASS_MODE
struct nanohub_bypass_notifier {
	uint64_t appId;
	void (*callback)(uint8_t* buf, uint8_t size);
};
#endif

#define NANOHUB_TP_SWITCH_AP            0x00
#define NANOHUB_TP_SWITCH_MCU_NORMAL    0x01
#define NANOHUB_TP_SWITCH_MCU_GLOVE     0x02
#define NANOHUB_TP_SWITCH_DISABLED      0x08

#ifdef CONFIG_NANOHUB_TP_SWITCH
int nanohub_tp_status(uint8_t status);
int nanohub_tp_solution(uint8_t solution);
#else
static inline int nanohub_tp_status(uint8_t status) {
	return 0;
}
static inline int nanohub_tp_solution(uint8_t solution) {
	return 0;
}
#endif

#ifdef CONFIG_NANOHUB_SECOND_DISP
int nanohub_tp_mode(uint8_t mode);
int nanohub_is_switch_operating(void);
#elif defined(CONFIG_NANOHUB_AOD)
int nanohub_tp_mode(uint8_t mode);
#else
static inline int nanohub_tp_mode(uint8_t mode) {
	return 0;
}
static inline int nanohub_is_switch_operating(void) {
	return 0;
}
#endif

#ifdef CONFIG_NANOHUB_EDGE
#define I2C_TO_SHUB 1
#define I2C_TO_ACPU 0
int nanohub_edge_i2c_switch(uint8_t mode);
#else
static inline int nanohub_edge_i2c_switch(uint8_t mode) {
    return 0;
}
#endif

#ifdef CONFIG_NANOHUB_EDGE_ULTRA
int nanohub_edge_vol_wakeup(uint8_t bitmask);
int nanohub_edge_debug(bool enable);
int nanohub_edge_dynamic_power_ctrl(bool enable);
int nanohub_set_drop_buf_flag(char *buf, size_t count);
#else
static inline int nanohub_edge_vol_wakeup(uint8_t bitmask) {
	return 0;
}
static inline int nanohub_edge_debug(bool enable) {
	return 0;
}
static inline int nanohub_edge_dynamic_power_ctrl(bool enable) {
	return 0;
}
static inline int nanohub_set_drop_buf_flag(char *buf, size_t count) {
	return 0;
}
#endif

int nanohub_vbus_status(uint8_t status);
void nanohub_edge_status (uint8_t status);

#ifdef CONFIG_NANOHUB
int nanohub_notifier(uint8_t event_id, void *val);
#else
static inline int nanohub_notifier(uint8_t event_id, void *val) {
	return 0;
}
#endif

#ifdef CONFIG_NANOHUB_FLASH_STATUS_CHECK
uint8_t nanohub_flash_status_check(void);
#endif

enum {
	SECOND_DISP_BL_CTRL,
};

#ifdef CONFIG_NANOHUB_AOD
enum {
	LCD_MODE_U0 = 0,	//Display off
	LCD_MODE_U1,		//Reserve
	LCD_MODE_U2,		//AOD mode
	LCD_MODE_U3,		//Display on (normal mode)
};
#endif

#define NANOHUB_CPU_STATUS_RESUME       0x00
#define NANOHUB_CPU_STATUS_SUSPEND      0x01

#define HTC_PROX_DATA_BUFFER_INDEX_START    24

#endif
