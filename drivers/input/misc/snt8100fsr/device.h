/*****************************************************************************
* File: device.h
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
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>
#include "sonacomm.h"

#ifndef DEVICE_H
#define DEVICE_H

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
static const char * const pctl_names[] = {
    "snt_hostirq_default",
    "snt_wakeirq_default",
    "snt_reset_active",
    "snt_reset_normal",
#if 0//def CONFIG_HTC_FEATURE
    "snt_cs_low",   /* for SPI */
    "snt_cs_high",  /* for SPI */
    "snt_bus_active",
    "snt_bus_sleep",
#endif
};

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
enum {
  BUS_TYPE_NONE = 0,
  BUS_TYPE_SPI  = 1,
  BUS_TYPE_I2C  = 2,
#ifdef CONFIG_HTC_FEATURE
  BUS_TYPE_NANOHUB,
#endif
  BUS_TYPE_MAX,
};

struct snt8100fsr {
    struct device           *dev;
    struct spi_device       *spi;
    struct i2c_client       *i2c;
    struct platform_device  *pdev;
    struct pinctrl          *snt_pinctrl;
    struct pinctrl_state    *pinctrl_state[ARRAY_SIZE(pctl_names)];
    int                     hostirq_gpio;

    int                     wakeirq_gpio;
    int                     rst_gpio;
#ifdef CONFIG_HTC_FEATURE
    struct miscdevice       miscdev;    /* misc device */
    struct sc_command       *sc_cmd_buf;
    struct mutex            sc_cmd_read_lock;
    bool                    sc_cmd_buf_ready;
    int                     hostirq;
    int                     snt1v2_en_gpio;
    int                     snt2v8_en_gpio;
    int                     chip_update_size_remain;

    //For performance check
    struct timespec         time_start, time_end, time_delta;

    //wakeup source
    struct wakeup_source edge_ws;

#ifdef CONFIG_FB
    struct notifier_block   fb_notifier;
    bool                    fb_ready;
#endif
    wait_queue_head_t       transmission_wait;
    atomic_t                transmission_state;
    uint8_t                 bypass_ctrl_state;
    struct mutex            bypass_read_lock;
    struct mutex            bypass_write_lock;
#endif
    int                     bus_type;

    // Rate to sample frames from the sensor
    int                     frame_rate;

    //Rate when device suspend (e.g. probe) is called
    int                     suspended_frame_rate;

    //// true if this i2c dev is to wake up the chip
    bool                    wake_i2c_device;
    struct class            *class;
    struct device           *device;

    // SysFS lock for sysfs calls
    struct mutex            track_report_sysfs_lock;

    // Serial bus lock (i2c/spi)
    struct mutex            sb_lock;
    u32                     spi_freq_khz;
    struct track_report     *track_reports;
    uint16_t                track_reports_count;
    uint16_t                track_reports_frame;

    // Frame streams left to receive from the hardware device for logging
    uint32_t                frame_stream_count;

    uint32_t                get_sys_param_id;
    uint32_t                get_sys_param_val;
    uint32_t                get_sys_param_status;
    struct   sc_command *   get_sys_param_cmd;
    uint32_t                set_sys_param_id;
    uint32_t                set_sys_param_val;
    uint32_t                set_sys_param_status;

    // fwupdate state variables
    struct file*    fwupdate_file;
    int             fwupdate_size;
    int             fwupdate_tx_mtu;
    int             fwupdate_tot_mtu;
    int             fwupdate_major;
    int             fwupdate_address;
    int             fwupdate_status;

    // read_flash_reg_partr variables
    uint16_t    *   reg_part_buf;

    // event log variables
    struct file *   event_log_file;

    // get_reg variables
    uint16_t   *    get_reg_buf;
    int             get_reg_id;
    int             get_reg_num;

    int             active_sc_cmd;

    // get EDGE KEY ID
    int             flash_flag;

    // weak_structure flag
    int             weak_struct_flag;
};


/*==========================================================================*/
/* EXTERNS                                                                  */
/*==========================================================================*/
// The currently enabled SysFS device
extern struct snt8100fsr *snt8100fsr_g;

#ifdef CONFIG_NANOHUB_SILEGO
// The spi device to debug the snt8100fsr
extern struct snt8100fsr *snt8100fsr_spi_g;
#endif

// The i2c device to wake the snt8100fsr
extern struct snt8100fsr *snt8100fsr_wake_i2c_g;

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
#ifdef CONFIG_HTC_FEATURE
int select_pin_ctl(struct snt8100fsr *snt8100fsr, const char *name);
int snt_nanohub_device_init(struct platform_device *pdev,
                        struct snt8100fsr *snt8100fsr);
#endif
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr);

int snt_i2c_device_init(struct i2c_client *i2c,
                        struct snt8100fsr *snt8100fsr);

int snt_suspend(struct device *dev);
int snt_resume(struct device *dev);

void snt_get_hw_id(struct platform_device *pdev,
                        struct snt8100fsr *snt8100fsr);
#endif // PROTOTYPE_H
