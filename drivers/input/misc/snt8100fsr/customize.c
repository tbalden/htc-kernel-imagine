/*****************************************************************************
* File: customize.c
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
*
*
*****************************************************************************/
#include <linux/input.h>
#include <linux/delay.h>
#include "track_report.h"
#include "input_device.h"

#include "config.h"
#include "debug.h"
#include "event.h"
#include "irq.h"
#ifdef CONFIG_HTC_FEATURE
#include "nanohub_bus.h"
#endif

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
#ifdef CONFIG_HTC_FEATURE
/*
 * edge_debug_mask usage:
 * 0x1  Print PRINT_FUNC() if kernel log level >= KERN_DEBUG
 * 0x2  Print PRINT_DEBUG() if kernel log level >= KERN_DEBUG
 * 0x4  Dump touch move event		//#if !defined(USE_NANOHUB_BUS)
 * 0x8  Dump touch down/up event	//#if !defined(USE_NANOHUB_BUS)
 * 0x10 Dump performance infomation	//#if !defined(USE_NANOHUB_BUS)
 * 0x20 n/a
 * 0x40 n/a
 * 0x80 dump chip debug log
 */
uint8_t             edge_debug_mask = 0x00;
bool                showtouch = false;
struct input_dev    *input_dev_g;
#endif

/*==========================================================================*/
/* register_input_events()                                                  */
/* Customize to register which input events we'll be sending                */
/*==========================================================================*/
void register_input_events(struct input_dev *input_dev) {
    PRINT_FUNC();
    /* For the types of events supported, please refer to:
       https://www.kernel.org/doc/Documentation/input/input-programming.txt */
#ifndef CONFIG_HTC_FEATURE
    /* As an example, we'll register for mouse button presses, movement
       and system volume control */
    input_dev->evbit[0] = BIT_MASK(EV_KEY) |
                          BIT_MASK(EV_REL) |
                          BIT_MASK(EV_ABS);

    input_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) |
                                             BIT_MASK(BTN_RIGHT);

    input_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
    input_dev->absbit[0] = BIT_MASK(ABS_VOLUME);
#else    //CONFIG_HTC_FEATURE
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);
#ifdef INPUT_PROP_DIRECT
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    input_set_abs_params(input_dev,
            ABS_MT_POSITION_X,
            0, 1080, 0, 0);
    input_set_abs_params(input_dev,
            ABS_MT_POSITION_Y,
            0, 2560, 0, 0);
    input_set_abs_params(input_dev,
            ABS_MT_PRESSURE,
            0, 255, 0, 0);

    input_set_abs_params(input_dev,
            ABS_MT_TOUCH_MAJOR,
            0, 255, 0, 0);
    input_set_abs_params(input_dev,
            ABS_MT_TOUCH_MINOR,
            0, 255, 0, 0);
#ifdef TYPE_B_PROTOCOL
    input_mt_destroy_slots(input_dev);
    input_mt_init_slots(input_dev, 10/*NUM_OF_SENSORS*/, 0);
#endif
#endif
    input_dev_g = input_dev;
#endif    //CONFIG_HTC_FEATURE
    PRINT_DEBUG("done");
}

/*==========================================================================*/
/* process_track_report()                                                   */
/* Customize to process track reports from the device                       */
/*==========================================================================*/
void process_track_reports(uint16_t frame,
                           struct track_report *tr,
                           size_t count) {
#if !defined(CONFIG_HTC_FEATURE) || defined(USE_NANOHUB_BUS)
    int i;
    PRINT_FUNC();
    /* process a track report */
    /* ...code... */

    for(i = 0; i < count; i++) {
        if (tr[i].bar_id==0 && tr[i].trk_id==0) break; // start of diag data

        if (IS_STG_TRACK_REPORT(tr[i].bar_id)) {
            struct stg_track_report *tg = (struct stg_track_report*) tr;
            // Timestamp, Frame, Bar, Frc0, Frc1, Frc2, Frc3, Frc4, Frc5, Frce6, Frc7
            PRINT_DEBUG("StrainRpt %u: B%u, F0%u, F1%u, F2%u",
                            frame,
                            tg[i].bar_id,
                            tg[i].force_lvl[0],
                            tg[i].force_lvl[1],
                            tg[i].force_lvl[2]
                            );
            PRINT_DEBUG("StrainRpt contd: F3%u, F4%u, F5%u, F6%u",
                            tg[i].force_lvl[3],
                            tg[i].force_lvl[4],
                            tg[i].force_lvl[5],
                            tg[i].force_lvl[6]
                            );

        } else {
            PRINT_DEBUG("Report %u: B%u, T%u, F%u, %u, %u, %u",
                        frame,
                        tr[i].bar_id,
                        tr[i].trk_id,
                        tr[i].force_lvl,
                        tr[i].top,
                        tr[i].center,
                        tr[i].bottom);
        }
    }

    /* do something with the result */
    /* e.g. we'll simulate a system absolute volume setting of 4 units */
    // input_report_abs(get_input_device(), ABS_VOLUME, 4);

    /* we could also send power_on/off, reset, keyboard, etc events,
     * or, we can make system calls to perform operations directly without
     * using the input device subsystem */

    PRINT_DEBUG("done");
#elif defined(CONFIG_HTC_FEATURE) && !defined(USE_NANOHUB_BUS)
    int i;
    struct timespec ts;
    static int8_t pre_state[10] = {-1}; //0 = up, 1 = down, -1 = initial
    static bool firstboot = true;
    u64 boottime;
    PRINT_FUNC();

    /* process a track report */
    /* ...code... */

    if(showtouch == false && firstboot) {
        get_monotonic_boottime(&ts);
        boottime = timespec_to_ns(&ts);
        if(boottime > 30000000000) {
            showtouch = true;
            firstboot = false;
        }
    }

    for(i = 0; i < count; i++) {
        if(showtouch && (IS_STG_TRACK_REPORT(tr[i].bar_id) == 0)) {
            if(tr[i].force_lvl > 0) {
                input_mt_slot(input_dev_g, (5*tr[i].bar_id) +  tr[i].trk_id);
                input_mt_report_slot_state(input_dev_g, MT_TOOL_FINGER, true);
                input_report_abs(input_dev_g, ABS_MT_POSITION_X, (tr[i].bar_id==0)?800:200);
                input_report_abs(input_dev_g, ABS_MT_POSITION_Y, 2260 - (((tr[i].center-80)*216)/100));
                input_report_abs(input_dev_g, ABS_MT_PRESSURE, tr[i].force_lvl);
                input_report_abs(input_dev_g, ABS_MT_TOUCH_MAJOR, tr[i].top - tr[i].bottom);
                input_report_abs(input_dev_g, ABS_MT_TOUCH_MINOR, 1);
            } else {
                input_mt_slot(input_dev_g, (5*tr[i].bar_id) + tr[i].trk_id);
                input_mt_report_slot_state(input_dev_g,
                        MT_TOOL_FINGER, false);
            }
        }
    }
    input_sync(input_dev_g);
#if 0
    if (edge_debug_mask & BIT(4)) {
        getnstimeofday(&snt8100fsr_g->time_end);
        snt8100fsr_g->time_delta.tv_nsec = (snt8100fsr_g->time_end.tv_sec*1000000000+snt8100fsr_g->time_end.tv_nsec) - (snt8100fsr_g->time_start.tv_sec*1000000000+snt8100fsr_g->time_start.tv_nsec);
        PRINT_INFO("Touch latency = %ld us", snt8100fsr_g->time_delta.tv_nsec/1000);
    }
#endif
    for(i = 0; i < count; i++) {
        if(edge_debug_mask & BIT(2)) {
            if (IS_STG_TRACK_REPORT(tr[i].bar_id)) {
                struct stg_track_report *tg = (struct stg_track_report*) tr;
                // Timestamp, Frame, Bar, Frc0, Frc1, Frc2, Frc3, Frc4, Frc5, Frce6, Frc7
                PRINT_INFO("StrainRpt %u: B%u, F0%u, F1%u, F2%u, F3%u",
                                frame,
                                tg[i].bar_id,
                                tg[i].force_lvl[0],
                                tg[i].force_lvl[1],
                                tg[i].force_lvl[2],
                                tg[i].force_lvl[3]
                                );
                PRINT_INFO("StrainRpt contd: F4%u, F5%u, F6%u, F7%u",
                                tg[i].force_lvl[4],
                                tg[i].force_lvl[5],
                                tg[i].force_lvl[6],
                                tg[i].force_lvl[7]
                                );

            } else {
                PRINT_INFO("Report %u: B%u, T%u, F%u, %u, %u, %u",
                            frame,
                            tr[i].bar_id,
                            tr[i].trk_id,
                            tr[i].force_lvl,
                            tr[i].top,
                            tr[i].center,
                            tr[i].bottom);
            }
        }
        if(tr[i].force_lvl > 0) {
            if(pre_state[(5*tr[i].bar_id) +  tr[i].trk_id] == 1 && (edge_debug_mask & BIT(2)))
                PRINT_NOTICE("TP[%s][%u][Move] F:%u Y:%u (Top:%u Bottom:%u)",
                        (tr[i].bar_id==0)? "Right":"Left",
                        (5*tr[i].bar_id) +  tr[i].trk_id,
                        tr[i].force_lvl,
                        1856 - tr[i].center,
                        1856 - tr[i].top,
                        1856 - tr[i].bottom);
            if(pre_state[(5*tr[i].bar_id) +  tr[i].trk_id] != 1) {
                if(edge_debug_mask & BIT(2) || edge_debug_mask & BIT(3))
                    PRINT_NOTICE("TP[%s][%u][Down] F:%u Y:%u (Top:%u Bottom:%u)",
                            (tr[i].bar_id==0)? "Right":"Left",
                            (5*tr[i].bar_id) +  tr[i].trk_id,
                            tr[i].force_lvl,
                            1856 - tr[i].center,
                            1856 - tr[i].top,
                            1856 - tr[i].bottom);
                pre_state[(5*tr[i].bar_id) +  tr[i].trk_id] = 1;
            }

        } else {
            if(edge_debug_mask & BIT(2) || edge_debug_mask & BIT(3))
                PRINT_NOTICE("TP[%s][%u][Up]",
                        (tr[i].bar_id==0)? "Right":"Left",
                        (5*tr[i].bar_id) +  tr[i].trk_id);
                pre_state[(5*tr[i].bar_id) +  tr[i].trk_id] = 0;
        }
    }
#endif    //#if !defined(CONFIG_HTC_FEATURE) || defined(USE_NANOHUB_BUS)
}

#ifdef CONFIG_HTC_FEATURE
#ifndef USE_NANOHUB_BUS
static ssize_t sysfs_event_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf,
                                      size_t count)
{
    unsigned long input;
#ifdef CONFIG_HTC_FEATURE
    if(is_sc_running()) {
        PRINT_CRIT("SC cmd is running! Skip attribute access!");
        return -EBUSY;
    }
#endif
    if (kstrtoul(buf, 10, &input) != 0) {
        PRINT_ERR("Failed to get the input value");
        return -EINVAL;
    }

    if (input > 1) {
        PRINT_ERR("invalid parameter");
        return -EINVAL;
    }

    PRINT_INFO("Unregister any existed irq handler!");
    irq_handler_unregister(snt8100fsr_g);

    if (input == false) {
            PRINT_INFO("Stop event processing");
            stop_event_processing(snt8100fsr_g);
    } else if (input == true) {
            PRINT_INFO("Start event processing");
            start_event_processing(snt8100fsr_g);
    }

    return count;
}

static ssize_t sysfs_input_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    ssize_t ret = 0;
    ret = snprintf(buf, PAGE_SIZE, "showtouch = %s\n", showtouch? "Ture":"False");

    return ret;
}
static ssize_t sysfs_input_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    unsigned long input;
    if (kstrtoul(buf, 10, &input) != 0) {
        PRINT_ERR("Failed to get the input value");
        return -EINVAL;
    }

    if(input > 2) {
        PRINT_ERR("invalid parameter");
        return -EINVAL;
    }
    if(input == 1)
        showtouch = true;
    else
        showtouch = false;

    PRINT_INFO("showtouch = %s", showtouch? "True":"False");

    return count;
}
#endif//#ifndef USE_NANOHUB_BUS

static ssize_t sysfs_debug_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    ssize_t ret = 0;
    ret = snprintf(buf, PAGE_SIZE, "debug mask = 0x%x\n", edge_debug_mask);

    return ret;
}
static ssize_t sysfs_debug_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    unsigned long input;
    if (kstrtoul(buf, 10, &input) != 0) {
        PRINT_ERR("Failed to get the input value");
        return -EINVAL;
    }

    edge_debug_mask = input & 0xFF;
#ifdef CONFIG_NANOHUB_EDGE_ULTRA
    nanohub_edge_debug(!!(edge_debug_mask & BIT(7)));
#endif

    PRINT_INFO("debug mask = 0x%x", edge_debug_mask);

    return count;
}

static ssize_t sysfs_reset_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    sc_cleanup(snt8100fsr_g);

    PRINT_INFO("Hard reset %s", SENTONS_DRIVER_NAME);
    (void)select_pin_ctl(snt8100fsr_g, "snt_reset_active");
    msleep(100);
    (void)select_pin_ctl(snt8100fsr_g, "snt_reset_normal");

    return count;
}

static ssize_t sysfs_chip_update_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    unsigned long input;
#ifdef CONFIG_HTC_FEATURE
    if(is_sc_running()) {
        PRINT_CRIT("SC cmd is running! Skip attribute access!");
        return -EBUSY;
    }
#endif
    if (kstrtoul(buf, 10, &input) != 0) {
        PRINT_ERR("Failed to get the input value");
        return -EINVAL;
    }

    if (input == 1) {
        PRINT_INFO("Unregister fw update irq handler");
        irq_handler_unregister(snt8100fsr_g);
        PRINT_INFO("Upload firmware");
        upload_firmware(snt8100fsr_g, FIRMWARE_LOCATION);
    } else {
        PRINT_INFO("invalid parameter");
    }

    return count;
}

static ssize_t sysfs_chip_update_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    ssize_t ret = 0;
    ret = snprintf(buf, PAGE_SIZE, "%d\n", snt8100fsr_g->chip_update_size_remain);

    return ret;
}

static ssize_t sysfs_enable_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    unsigned long input;
    if (kstrtoul(buf, 10, &input) != 0) {
        PRINT_ERR("Failed to get the input value");
        return -EINVAL;
    }

    if(input > 2) {
        PRINT_ERR("invalid parameter");
        return -EINVAL;
    }

    if(input == 1)
        snt_irq_enable(snt8100fsr_g);
    else
        snt_irq_disable(snt8100fsr_g);

    PRINT_INFO("%s", input?"Enable":"Disable");

    return count;
}

#ifdef CONFIG_NANOHUB_SILEGO
static ssize_t sysfs_snt_bus_type_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    ssize_t ret = 0;
    ret = snprintf(buf, PAGE_SIZE, "snt_bus_type = %d\n", snt8100fsr_g->bus_type);

    return ret;
}
static ssize_t sysfs_snt_bus_type_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    uint8_t state;
    unsigned long input;
    if (kstrtoul(buf, 10, &input) != 0) {
        PRINT_ERR("Failed to get the input value");
        return -EINVAL;
    }

    if(!snt8100fsr_g)
        return count;

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_wake_device(snt8100fsr_g);
    snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
#endif
#endif

    switch(input) {
        case BUS_TYPE_SPI:
            if(snt8100fsr_g->spi) {
                snt8100fsr_g->bus_type = input;
                state = (SPI_MODE_SET | BUS_TYPE_SPI);
                snt_nanohub_bypass_ctrl(snt8100fsr_g, state);
                if (!snt8100fsr_g->edge_ws.active) {
                    __pm_stay_awake(&snt8100fsr_g->edge_ws);
                    PRINT_INFO("bus: SPI, edge_ws: %s",
                            snt8100fsr_g->edge_ws.active ? "enabled" : "disabled");
                }
            }
            break;
        case BUS_TYPE_I2C:
            if(snt8100fsr_g->i2c) {
                snt8100fsr_g->bus_type = input;
                state = (SPI_MODE_SET | BUS_TYPE_I2C);
                snt_nanohub_bypass_ctrl(snt8100fsr_g, state);
            }
            break;
        default:
            snt8100fsr_g->bus_type = BUS_TYPE_NANOHUB;
            state = (SPI_MODE_SET | BUS_TYPE_NANOHUB);
            snt_nanohub_bypass_ctrl(snt8100fsr_g, state);
            if (snt8100fsr_g->edge_ws.active) {
                __pm_relax(&snt8100fsr_g->edge_ws);
                PRINT_INFO("bus: HUB, edge_ws: %s",
                        snt8100fsr_g->edge_ws.active ? "enabled" : "disabled");
            }
    }

    PRINT_INFO("snt_bus_type = %d", snt8100fsr_g->bus_type);

    return count;
}
#endif

#ifndef USE_NANOHUB_BUS
static DEVICE_ATTR(event,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_event_store);

static DEVICE_ATTR(input,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_input_show,
                   sysfs_input_store);
#endif//#ifndef USE_NANOHUB_BUS

static DEVICE_ATTR(debug,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_debug_show,
                   sysfs_debug_store);

static DEVICE_ATTR(reset,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_reset_store);

static DEVICE_ATTR(chip_update,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_chip_update_show,
                   sysfs_chip_update_store);

static DEVICE_ATTR(enable,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_enable_store);

#ifdef CONFIG_NANOHUB_SILEGO
static DEVICE_ATTR(snt_bus_type,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_snt_bus_type_show,
                   sysfs_snt_bus_type_store);
#endif

static struct attribute *sysfs_attrs_htc[] = {
#ifndef USE_NANOHUB_BUS
    &dev_attr_event.attr,
    &dev_attr_input.attr,
#endif
    &dev_attr_debug.attr,
    &dev_attr_reset.attr,
    &dev_attr_chip_update.attr,
    &dev_attr_enable.attr,
#ifdef CONFIG_NANOHUB_SILEGO
    &dev_attr_snt_bus_type.attr,
#endif
    NULL,
};
static const struct attribute_group attr_group_htc = {
    .attrs = sysfs_attrs_htc,
};

int htc_sysfs_init(struct snt8100fsr *snt8100fsr, struct kobject *sysfs_kobj, bool enable)
{
    if (snt8100fsr == NULL || sysfs_kobj == NULL) {
        PRINT_ERR("snt8100fsr or sysfs_kobj is NULL");
        return -ENOMEM;
    }

    if (!enable) {
        sysfs_remove_group(sysfs_kobj, &attr_group_htc);
        PRINT_INFO("HTC sysfs attributes removed");
    } else {
        if (sysfs_create_group(sysfs_kobj, &attr_group_htc) < 0) {
            PRINT_ERR("Failed to create HTC sysfs attributes");
            return -ENODEV;
        } else
            PRINT_DEBUG("done");
    }
    return 0;
}
#endif
