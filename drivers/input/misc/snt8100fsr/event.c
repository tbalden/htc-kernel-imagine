/*****************************************************************************
* File: event.c
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
#include <linux/kernel.h>

#include "config.h"
#include "serial_bus.h"
#include "workqueue.h"
#include "hardware.h"
#include "irq.h"
#include "memory.h"
#include "file.h"
#include "event.h"
#include "track_report.h"
#include "customize.h"
#include "sonacomm.h"
#include "utils.h"
#include "device.h"
#include "debug.h"
#include "crc.h"
#ifdef CONFIG_HTC_FEATURE
#include "nanohub_bus.h"
#endif

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define MAX_FRAME_BUFFER_SIZE    (16 * 1024)
#define FRAME_LENGTH_WIDTH       (4)
#define FRAME_BUFFER_SIZE        (FRAME_LENGTH_WIDTH + MAX_FRAME_BUFFER_SIZE)

// I2C FWUpdate
#define FW_BUFFER_SIZE         256
#define FWImageTrailerSize      16


/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct event_work {
    struct delayed_work my_work;
    struct snt8100fsr *snt8100fsr;
    bool context_fwd;
};

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
static irqreturn_t irq_handler_top(int irq, void *dev);
static void event_wq_func(struct work_struct *work);
static void read_track_reports(struct snt8100fsr *sb_dev);
static void log_track_report(uint16_t frame,
                             struct track_report *tr,
                             size_t count);
static void log_track_report_bin(uint16_t frame,
                             struct track_report *tr,
                             size_t count);
static void log_no_touch_frame(struct snt8100fsr *snt8100fsr);
static void log_d1test(struct snt8100fsr *snt8100fsr);
static void log_frame_cmd(struct snt8100fsr *snt8100fsr);
static void log_frame_stream(struct snt8100fsr *snt8100fsr);
static void set_sys_param_rsp(struct snt8100fsr *snt8100fsr);
static void get_sys_param_rsp(struct snt8100fsr *snt8100fsr);

#ifdef SUPPORT_FLASH
static void fwupdate_rsp_ind(struct snt8100fsr *snt8100fsr);
static void write_flash_reg_part_rsp(struct snt8100fsr *snt8100fsr);
#endif
static void spurious_cmd_event(struct snt8100fsr *snt8100fsr);

static int init_after_reset(struct snt8100fsr *snt8100fsr);
void sc_flush(struct snt8100fsr *snt8100fsr);

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static struct event_work *work_g;

struct file*    log_track_reports_file = NULL;
static int      log_track_reports_file_offset;
struct file*    log_track_reports_bin_file = NULL;
static int      log_track_reports_bin_file_offset;
struct file*    log_d1test_file = NULL;
static int      log_d1test_file_offset;
static int      log_d1test_num_frames;
struct file*    log_frame_file = NULL;
static int      log_frame_file_offset;
struct file*    log_no_touch_frame_file = NULL;
static int      log_no_touch_frame_file_offset;
static struct   sc_command *sc_cmd;
static uint8_t* frame_buffer;
static uint32_t frame_size;
static uint8_t  log_d1test_loop;


/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
#ifdef CONFIG_HTC_FEATURE
bool is_sc_running(void) {
    return (snt8100fsr_g->active_sc_cmd != mc_no_command);
}
EXPORT_SYMBOL(is_sc_running);
#endif

int stop_event_processing(struct snt8100fsr *snt8100fsr) {
    struct register_enable reg_enable;
    int ret;
	PRINT_FUNC();
    // Disable IRQ processing
#ifdef CONFIG_HTC_FEATURE
    irq_handler_unregister(snt8100fsr);
#else
    irq_handler_unregister();
#endif

    // Clear the enable bit to stop processing touch reports
    reg_enable.enable = 0;
    reg_enable.padding = 0;
    ret = write_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
    if (ret) {
        PRINT_CRIT("write_register() failed");
    }

    // Free work object
    if (work_g) {
        workqueue_free_work(work_g);
        work_g = NULL;
    }

    // Free memory for saving last received track reports
    if (snt8100fsr->track_reports) {
        memory_free(snt8100fsr->track_reports);
        snt8100fsr->track_reports = NULL;
    }

    return 0;
}
EXPORT_SYMBOL(stop_event_processing);

int start_event_processing(struct snt8100fsr *snt8100fsr) {
#ifdef CONFIG_HTC_FEATURE
    char product_string[PRODUCT_CONFIG_MAX_LEN] = {0x00};
#else
    char *product_string;
#endif
    struct register_event reg_event;
	// [dy] Always declare register_enable
    struct register_enable reg_enable;

    int ret;

    PRINT_FUNC();

    snt8100fsr->active_sc_cmd = mc_no_command;

    /*
     *  Allocate memory for our track reports. No DMA needed as the underlying
     *  SPI/I2C routines will use a DMA enabled block of memory for transfers
     */
    snt8100fsr->track_reports = memory_allocate(TRACK_REPORTS_MAX_LEN, 0);

    // Allocate a work object for queuing from IRQ handler
    work_g = (void *)workqueue_alloc_work(sizeof(struct event_work),
                                          event_wq_func);
    if (!work_g) {
        PRINT_CRIT(OOM_STRING);
        return -ENOMEM;
    }

    work_g->snt8100fsr = snt8100fsr;
	// [dy] Event queue initializes firmware download context false
	work_g -> context_fwd = false;
    // Read the event register
    ret = read_register(snt8100fsr, REGISTER_EVENT, &reg_event);
    if (ret) {
        PRINT_CRIT("sb_read_register() failed");
        return -1;
    }

    if (reg_event.boot == 0) {
        PRINT_INFO("System already booted");
    } else {
        PRINT_INFO("System booted");
	}

#if !defined(CONFIG_HTC_FEATURE)
    // Get the product configuration string
    product_string = memory_allocate(PRODUCT_CONFIG_MAX_LEN, 0);
    if (product_string == NULL) {
        PRINT_CRIT("memory_allocate(PRODUCT_CONFIG_MAX_LEN) failed");
        return -1;
    }
#endif

    ret = read_product_config(snt8100fsr, product_string);
#if !defined(CONFIG_HTC_FEATURE)
    memory_free(product_string);
#endif
    if (ret) {
        PRINT_WARN("Unable to read product config");
    }

    // Set the frame rate
    PRINT_INFO("Setting frame rate to: %dhz", snt8100fsr->frame_rate);
    ret = write_register(snt8100fsr, REGISTER_FRAME_RATE, &snt8100fsr->frame_rate);
    if (ret) {
        PRINT_CRIT("Unable to set frame rate");
        return -1;
    }

    // Enable IRQ processing
#ifdef CONFIG_HTC_FEATURE
    irq_handler_register(snt8100fsr, irq_handler_top, NULL);
#else
    irq_handler_register(irq_handler_top, NULL);
#endif

	// [dy] 2017-09-01 Touch Enable/Disable was not ifdef'd properly
    // Read the enable bit first
    ret = read_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
    if (ret) {
        PRINT_CRIT("Unable to read register enable");
        return -1;
    }
    PRINT_INFO("Touch processing currently set to %d", reg_enable.enable);
    reg_enable.padding = 0;
#ifdef TOUCH_ENABLED_AT_STARTUP
    // Set the Enable bit to process touch reports
    reg_enable.enable = 1;
#else
    reg_enable.enable = 0;
#endif
    ret = write_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
    if (ret) {
        PRINT_CRIT("sb_read_register() failed");
        return -1;
    }
    ret = read_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
    if (ret) {
        PRINT_CRIT("Unable to read register enable");
        return -1;
    }

    if (reg_enable.enable == 1) {
        PRINT_INFO("Enabled hardware touch processing");
    } else {
#ifdef TOUCH_ENABLED_AT_STARTUP
        PRINT_CRIT("Unable to enable hardware touch processing");
        return -1;
#else
        PRINT_INFO("Hardware touch processing is disabled");
#endif
    }

    // Load any register values on the disk
    load_registers_from_disk(snt8100fsr);

    PRINT_DEBUG("done");
    return 0;
}
EXPORT_SYMBOL(start_event_processing);

#ifdef USE_NANOHUB_BUS
int snt_irq_handler_register(struct snt8100fsr *snt8100fsr) {
    PRINT_FUNC();

    snt8100fsr->active_sc_cmd = mc_no_command;

    /*
     *  Allocate memory for our track reports. No DMA needed as the underlying
     *  SPI/I2C routines will use a DMA enabled block of memory for transfers
     */
    snt8100fsr->track_reports = memory_allocate(TRACK_REPORTS_MAX_LEN, 0);

    // Allocate a work object for queuing from IRQ handler
    work_g = (void *)workqueue_alloc_work(sizeof(struct event_work),
                                          event_wq_func);
    if (!work_g) {
        PRINT_CRIT(OOM_STRING);
        return -ENOMEM;
    }

    work_g->snt8100fsr = snt8100fsr;

    // Regiser IRQ handler for sc_command processing
    irq_handler_register(snt8100fsr, irq_handler_top, NULL);
    // Skip IRQ to ignore edge events
    //snt_nanohub_bypass_ctrl(snt8100fsr, EVENT_REPORT);
    snt_irq_disable(snt8100fsr);

    // Load any register values on the disk
    //load_registers_from_disk(snt8100fsr);

    PRINT_DEBUG("done");
    return 0;
}
EXPORT_SYMBOL(snt_irq_handler_register);
#endif

static int init_after_reset( struct snt8100fsr *snt8100fsr) {
    struct register_event reg_event;
	// [dy] Always declare register_enable
    struct register_enable reg_enable;
	int ret;
	PRINT_FUNC();
	if (is_irq_handler_registered()) {
		PRINT_INFO("irq_handler is registered.");
	} else {
		PRINT_CRIT("irq_handler not registered.");
	}

    // Read the event register
    ret = read_register(snt8100fsr, REGISTER_EVENT, &reg_event);
    if (ret) {
        PRINT_CRIT("sb_read_register() failed");
        return -1;
    }

    if (reg_event.boot == 0) {
        PRINT_INFO("System already booted");
    } else {
        PRINT_INFO("System booted");
	}

    snt8100fsr->active_sc_cmd = mc_no_command;

	// Disable touch reports
#ifdef CONFIG_HTC_FEATURE
    // Read the enable bit first
    ret = read_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
    if (ret) {
        PRINT_CRIT("Unable to read register enable");
        return -1;
    }
#endif
    PRINT_INFO("Touch processing currently set to %d", reg_enable.enable);
    reg_enable.padding = 0;
#ifdef TOUCH_ENABLED_AT_STARTUP
    // Set the Enable bit to process touch reports
    reg_enable.enable = 1;
#else
    reg_enable.enable = 0;
#endif
    ret = write_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
    if (ret) {
        PRINT_CRIT("sb_read_register() failed");
        return -1;
    }
    ret = read_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
    if (ret) {
        PRINT_CRIT("Unable to read register enable");
        return -1;
    }

    if (reg_enable.enable == 1) {
        PRINT_INFO("Enabled hardware touch processing");
    } else {
#ifdef TOUCH_ENABLED_AT_STARTUP
        PRINT_CRIT("Unable to enable hardware touch processing");
        return -1;
#else
        PRINT_INFO("Hardware touch processing is disabled");
#endif
	}
	return 0;
}

int set_context_fwd_done(struct snt8100fsr *snt8100fsr) {
	PRINT_FUNC();
	if (work_g -> context_fwd) {
		work_g -> context_fwd = false;
		PRINT_DEBUG("context_fwd set to false");
	}
	else {
		PRINT_DEBUG("context_fwd already false");
	}
	init_after_reset( snt8100fsr);
	return 0;
}

static irqreturn_t irq_handler_top(int irq, void *dev) {
    PRINT_FUNC();
	/* [dy] If context of firmware download, firmware work queue
	 */
	if (work_g -> context_fwd) {
	   irq_handler_fwd();
	} else {
		workqueue_queue_work(work_g, 0);
	}
    PRINT_DEBUG("done");
    return IRQ_HANDLED;
}

static void event_wq_func(struct work_struct *work) {
    struct register_event reg_event;
    struct event_work *w = (struct event_work *)work;
    struct snt8100fsr *snt8100fsr = w->snt8100fsr;
    int ret;

    PRINT_FUNC();

    do {
        ret = read_register(snt8100fsr, REGISTER_EVENT, &reg_event);
        if (ret) {
            PRINT_CRIT("Unable to read event register");
            return;
        }

		// Check if firmware download requested
		if (1 == reg_event.fwd) {
		  PRINT_INFO("Firmware download requested");
#ifdef UPLOAD_FIRMWARE
		  // [dy] Set event context firmware download
		  work_g -> context_fwd = true;
		  upload_firmware_fwd(snt8100fsr, FIRMWARE_LOCATION);
#endif
		}

        // Do we have any track reports to handle?
        if (reg_event.touch) {
            read_track_reports(snt8100fsr);
        }

        // Any command completions?
        if (reg_event.command) {
            switch (snt8100fsr->active_sc_cmd) {
                case mc_d1_test:        log_d1test(snt8100fsr);                 break;
                case mc_frame_dump:     log_frame_cmd(snt8100fsr);              break;
                case mc_no_touch:       log_no_touch_frame(snt8100fsr);         break;
                case mc_set_sys_param:  set_sys_param_rsp(snt8100fsr);          break;
                case mc_get_sys_param:  get_sys_param_rsp(snt8100fsr);          break;
#ifdef SUPPORT_FLASH
                case mc_fw_update:
                case mc_flash_update:   fwupdate_rsp_ind(snt8100fsr);           break;
                case mc_update_regs:    write_flash_reg_part_rsp(snt8100fsr);   break;
#endif
                default:                spurious_cmd_event(snt8100fsr);         break;
            }
        }

        if (reg_event.stream) {
            log_frame_stream(snt8100fsr);
        }

        /*
         * Don't loop on reg_event.stream because too much data, we'll never
         * leave our while loop and the system may reboot due to watchdog timer
         */
    }
#ifdef USE_NANOHUB_BUS
    while ((snt8100fsr->bypass_ctrl_state & FW_REG_LOG_OP_START)
                    && (reg_event.touch || (reg_event.command && (log_d1test_file == NULL))));
#else
    while (reg_event.touch || (reg_event.command && (log_d1test_file == NULL)));
#endif
    /*
     * If the host bit is set, we are waking up from a sleep mode. We need
     * to set the frame rate back to the last known setting. Least significant byte.
     */
    if (reg_event.host) {
        PRINT_DEBUG("Host event bit set, restoring frame rate to %dhz",
                    snt8100fsr->frame_rate);
        ret = write_register(snt8100fsr, REGISTER_FRAME_RATE,
                             &snt8100fsr->frame_rate);
        if (ret) {
            PRINT_CRIT("write_register(REGISTER_FRAME_RATE) failed");
        }
    }

    PRINT_DEBUG("done");
    return;
}

struct __attribute__((__packed__)) length_and_frame {
    uint16_t len;           // length, in bytes, of FIFO message (including frame field)
    uint16_t frame;         // frame number
};

static void read_track_reports(struct snt8100fsr *snt8100fsr) {
    static const uint16_t tr_size = sizeof(struct track_report);
    struct length_and_frame landf;
    int ret;

    PRINT_FUNC();

    mutex_lock(&snt8100fsr->sb_lock);

    // Read the 4 byte Track Report Header
    ret = sb_read_fifo(snt8100fsr,
                       REGISTER_FIFO_TOUCH,
                       TOUCH_FIFO_LENGTH_WIDTH + TOUCH_FIFO_FRAME_WIDTH,
                       &landf);
    if (ret) {
        PRINT_CRIT("sb_read_fifo() track reports length failed");
        goto cleanup_sb;
    }

    // If the length is greater than our max size, we have a problem
    if (landf.len > TRACK_REPORTS_MAX_LEN) {
#ifdef CONFIG_HTC_FEATURE
        PRINT_CRIT("Track reports length greater than max %lu (%d)",
                   TRACK_REPORTS_MAX_LEN, landf.len);
#else
        PRINT_CRIT("Track reports length greater than max %d (%d)",
                   TRACK_REPORTS_MAX_LEN, landf.len);
#endif
        goto cleanup_sb;
    }

    // Remove frame field width from length to get size of track reports part
    landf.len -= TOUCH_FIFO_FRAME_WIDTH;

    // Fatal if length is not divisble by size of our track report struct
    if (landf.len % tr_size) {
        PRINT_CRIT("Length not multiple of %d", tr_size);
        goto cleanup_sb;
    }

    /*
     * Lock to protect track_reports and track_reports_count for the
     * SysFS interface which uses these fields
     */
    mutex_lock(&snt8100fsr->track_report_sysfs_lock);

    // Calculate the number of track reports
    snt8100fsr->track_reports_count = landf.len / tr_size;
    snt8100fsr->track_reports_frame = landf.frame;
    PRINT_DEBUG("%d total length consisting of %d reports",
                landf.len,
                snt8100fsr->track_reports_count);

    // Read the track reports
    ret = sb_read_fifo(snt8100fsr,
                       REGISTER_FIFO_TOUCH,
                       landf.len,
                       snt8100fsr->track_reports);
    if (ret) {
        PRINT_CRIT("sb_read_fifo() track reports failed");
        goto cleanup_sysfs_and_sb;
    }

    // Pass the reports to the processing routine
    process_track_reports(snt8100fsr->track_reports_frame,
                          snt8100fsr->track_reports,
                          snt8100fsr->track_reports_count);

    // If we are logging track reports, save to the file
    if(log_track_reports_file != NULL) {
        log_track_report(snt8100fsr->track_reports_frame,
                         snt8100fsr->track_reports,
                         snt8100fsr->track_reports_count);
    }
    if (log_track_reports_bin_file != NULL) {
        log_track_report_bin(snt8100fsr->track_reports_frame,
                         snt8100fsr->track_reports,
                         snt8100fsr->track_reports_count);
    }

    PRINT_DEBUG("done");

cleanup_sysfs_and_sb:
    mutex_unlock(&snt8100fsr->track_report_sysfs_lock);
cleanup_sb:
    mutex_unlock(&snt8100fsr->sb_lock);
}

int read_product_config(struct snt8100fsr *snt8100fsr, char *product_string) {
    uint16_t len;
    int ret;

    mutex_lock(&snt8100fsr->sb_lock);

    // Read the length
    ret = sb_read_fifo(snt8100fsr, REGISTER_FIFO_CONFIG, TOUCH_FIFO_LENGTH_WIDTH, &len);
    if (ret) {
        PRINT_CRIT("sb_read_fifo() product config length failed");
        mutex_unlock(&snt8100fsr->sb_lock);
        return ret;
    }

    // If the length is greater than our max size, we have a problem
    if (len > PRODUCT_CONFIG_MAX_LEN) {
        PRINT_CRIT("Product config length greater than max %d (%d)",
                   PRODUCT_CONFIG_MAX_LEN, len);
        mutex_unlock(&snt8100fsr->sb_lock);
        return -1;
    }

    // Read the product config string
    ret = sb_read_fifo(snt8100fsr, REGISTER_FIFO_CONFIG, len, product_string);
    if (ret) {
        PRINT_CRIT("sb_read_fifo() track reports failed");
        mutex_unlock(&snt8100fsr->sb_lock);
        return ret;
    }

    PRINT_NOTICE("Product Info: %s", product_string);
    mutex_unlock(&snt8100fsr->sb_lock);
    return 0;
}

/*==========================================================================*/
/* TRACK REPORT LOGGING                                                     */
/*==========================================================================*/
int enable_track_report_logging(bool enable, int ftype) {
    int ret;
    char log_header[] =
            "Timestamp, Frame No, Bar, Track ID, Force, Pos0, Pos1, Pos2\n";

    if (ftype == 1) {   // bin file
        if (enable) {
            if (log_track_reports_bin_file == NULL) {
                PRINT_DEBUG("Creating track report bin log file: %s",
                            TRACK_REPORT_BIN_LOG_LOCATION);
                log_track_reports_bin_file_offset = 0;
                ret = file_open(TRACK_REPORT_BIN_LOG_LOCATION,
                                O_WRONLY|O_CREAT|O_TRUNC, 0777,
                                &log_track_reports_bin_file);
                if(ret) {
                    PRINT_DEBUG("Unable to create file '%s', error %d",
                                TRACK_REPORT_BIN_LOG_LOCATION, ret);
                    return -1;
                }
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
                PRINT_INFO("log_track_report_bin: +++");
                snt_nanohub_wake_device(snt8100fsr_g);
                snt_nanohub_bypass_ctrl(snt8100fsr_g, LOG_TRACK_REPORT | FW_REG_LOG_OP_START);
#endif
#endif
            } else {
                PRINT_NOTICE("Track report bin logging already enabled");
            }
        } else {
            if (log_track_reports_bin_file != NULL) {
                PRINT_DEBUG("Closing track report log file: %s",
                            TRACK_REPORT_BIN_LOG_LOCATION);
                file_close(log_track_reports_bin_file);
                log_track_reports_bin_file = NULL;
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
                snt_nanohub_bypass_ctrl(snt8100fsr_g, LOG_TRACK_REPORT | FW_REG_LOG_OP_STOP);
                PRINT_INFO("log_track_report_bin: ---");
#endif
#endif
            } else {
                PRINT_NOTICE("Track report bin logging already disabled");
            }
        }
        return 0;
    }

    if (enable) {
        if (log_track_reports_file == NULL) {
            PRINT_DEBUG("Creating track report log file: %s",
                        TRACK_REPORT_LOG_LOCATION);
            log_track_reports_file_offset = 0;
            ret = file_open(TRACK_REPORT_LOG_LOCATION,
                            O_WRONLY|O_CREAT|O_TRUNC, 0777,
                            &log_track_reports_file);
            if(ret) {
                PRINT_DEBUG("Unable to create file '%s', error %d",
                            TRACK_REPORT_LOG_LOCATION, ret);
                return -1;
            }

            // Write a text header so that the log file can be interpreted
            file_write(log_track_reports_file,
                       log_track_reports_file_offset,
                       log_header,
                       sizeof(log_header));
            log_track_reports_file_offset += sizeof(log_header);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
            PRINT_INFO("log_track_report: +++");
            snt_nanohub_wake_device(snt8100fsr_g);
            snt_nanohub_bypass_ctrl(snt8100fsr_g, LOG_TRACK_REPORT | FW_REG_LOG_OP_START);
#endif
#endif
        } else {
            PRINT_NOTICE("Track report logging already enabled");
        }
    } else {
        if (log_track_reports_file != NULL) {
            PRINT_DEBUG("Closing track report log file: %s",
                        TRACK_REPORT_LOG_LOCATION);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
            snt_nanohub_bypass_ctrl(snt8100fsr_g, LOG_TRACK_REPORT | FW_REG_LOG_OP_STOP);
            PRINT_INFO("log_track_report: ---");
#endif
#endif
            file_close(log_track_reports_file);
            log_track_reports_file = NULL;
        } else {
            PRINT_NOTICE("Track report logging already disabled");
        }
    }

    return 0;
}

static int build_tr_diag_report_001(p_tr_diag_vers_001_t p_diag, uint8_t *log_buf, int max_buf)
{
    int i;
    int size = snprintf(log_buf, max_buf,
                 "    vers=%d, gpio=0x%02x, atc=%u, ntm.stress=%u, ntm.nt=%u\n",
                  (unsigned int)p_diag->vers,
                  (unsigned int)p_diag->gpio,
                  (unsigned int)p_diag->atc,
                  (unsigned int)p_diag->ntm>>4,
                  (unsigned int)p_diag->ntm&0xf);
    size += snprintf(log_buf+size, max_buf-size, "    mpa : ");
    for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
      size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->mpa[i]);
    }
    size += snprintf(log_buf+size, max_buf-size, "\n    d_mp: ");
    for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
      size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->d_mp[i]);
    }
    size += snprintf(log_buf+size, max_buf-size, "\n\n");
    return size;
}

static int build_tr_diag_report_002(p_tr_diag_vers_002_t p_diag, uint8_t *log_buf, int max_buf)
{
    int i;
    int size = snprintf(log_buf, max_buf,
                 "    vers=%d, frame_rate=%d, trig=0x%02x, gpio=0x%02x, atc=%u, ntm.stress=%u, ntm.nt=%u\n",
                  (unsigned int)p_diag->vers,
                  (unsigned int)p_diag->frame_rate,
                  (unsigned int)p_diag->trig,
                  (unsigned int)p_diag->gpio,
                  (unsigned int)p_diag->atc,
                  (unsigned int)p_diag->ntm>>4,
                  (unsigned int)p_diag->ntm&0xf);
    size += snprintf(log_buf+size, max_buf-size, "    mpa : ");
    for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
      size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->mpa[i]);
    }
    size += snprintf(log_buf+size, max_buf-size, "\n    d_mp: ");
    for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
      size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->d_mp[i]);
    }
    size += snprintf(log_buf+size, max_buf-size, "\n\n");
    return size;
}

static int build_tr_diag_report(uint8_t* p_diag, uint8_t *log_buf, int max_buf)
{
    if (p_diag) {
        switch (p_diag[1]) {
            case 1: return build_tr_diag_report_001((p_tr_diag_vers_001_t)p_diag, log_buf, max_buf); break;
            case 2: return build_tr_diag_report_002((p_tr_diag_vers_002_t)p_diag, log_buf, max_buf); break;
            default: PRINT_CRIT("Unkown tr_diag version %d", p_diag[1]); return(0); break;
        }
    }
    return(0);
}

static void log_track_report_bin(uint16_t frame,
                      struct track_report *tr,
                      size_t count) {

    // record is tot_len(2) ts(4) reclen(2) frame(2) records(8*x)
    static const uint16_t tr_size = sizeof(struct track_report);
    uint32_t ms = get_time_in_ms();
    uint16_t hdr[5];
    hdr[0] = count*tr_size + 2*sizeof(uint16_t) + sizeof(uint32_t);
    hdr[1] = (uint16_t)(ms&0xffff);
    hdr[2] = (uint16_t)(ms>>16);
    hdr[3] = count*tr_size + sizeof(uint16_t);
    hdr[4] = frame;

    PRINT_FUNC();

    file_write(log_track_reports_bin_file,
                log_track_reports_bin_file_offset,
                (void*)hdr,
                sizeof(hdr));
    log_track_reports_bin_file_offset += sizeof(hdr);
    file_write(log_track_reports_bin_file,
                log_track_reports_bin_file_offset,
                (void*)tr,
                count*tr_size);
    log_track_reports_bin_file_offset += count*tr_size;
}


static void log_track_report(uint16_t frame,
                      struct track_report *tr,
                      size_t count) {
    char log_buf[160];
    uint32_t ms;
    int i, size;

    PRINT_FUNC();

    ms = get_time_in_ms();

    for(i = 0; i < count; i++) {

        if (tr[i].bar_id==0 && tr[i].trk_id==0) { // start of diag data
            int size = 0;
            if (i==0) size += snprintf(log_buf+size, sizeof(log_buf), "%u, %u\n", ms, frame);
            size += build_tr_diag_report((uint8_t*)&tr[i], log_buf+size, sizeof(log_buf)-size);
            file_write(log_track_reports_file,
                       log_track_reports_file_offset,
                       log_buf,
                       size);

            log_track_reports_file_offset += size;
            break;
        }

        if (IS_STG_TRACK_REPORT(tr[i].bar_id)) {
            struct stg_track_report *tg = (struct stg_track_report*) tr;
            // Timestamp, Frame, Bar, Frc0, Frc1, Frc2, Frc3, Frc4, Frc5, Frce6
            size = snprintf(log_buf, sizeof(log_buf),
                            "%u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
                            ms,
                            frame,
                            tg[i].bar_id,
                            tg[i].force_lvl[0],
                            tg[i].force_lvl[1],
                            tg[i].force_lvl[2],
                            tg[i].force_lvl[3],
                            tg[i].force_lvl[4],
                            tg[i].force_lvl[5],
                            tg[i].force_lvl[6]
                            );
        } else {
            // Timestamp, Frame No, Bar, Track ID, Force, Pos0, Pos1, Pos2
            size = snprintf(log_buf, sizeof(log_buf),
                            "%u, %u, %u, %u, %u, %u, %u, %u\n",
                            ms,
                            frame,
                            tr[i].bar_id,
                            tr[i].trk_id,
                            tr[i].force_lvl,
                            tr[i].center,
                            tr[i].bottom,
                            tr[i].top);
        }
        file_write(log_track_reports_file,
                   log_track_reports_file_offset,
                   log_buf,
                   size);

        log_track_reports_file_offset += size;
    }
}

int enable_event_logging(struct snt8100fsr *snt8100fsr) {
    int ret=0;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure */
    mutex_lock(&snt8100fsr->sb_lock);

    if (snt8100fsr->event_log_file == NULL) {
        PRINT_DEBUG("Creating event log file: %s",
                    EVENT_LOG_LOCATION);
        ret = file_open(EVENT_LOG_LOCATION,
                        O_WRONLY|O_CREAT|O_TRUNC, 0777,
                        &snt8100fsr->event_log_file);
        if(ret) {
            PRINT_DEBUG("Unable to create file '%s', error %d",
                        EVENT_LOG_LOCATION, ret);
            goto errexit;
        }
    } else {
        PRINT_NOTICE("Event logging already enabled");
        goto cleanup;
    }

    if (snt8100fsr->active_sc_cmd != mc_no_command) {
        PRINT_CRIT("Cannot retrieve event log. Command (%d) already active", snt8100fsr->active_sc_cmd);
        ret = -1;
        goto errexit;
    }

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
#ifdef CONFIG_HTC_FEATURE
        if (sc_cmd == NULL) {
            ret = -1;
            goto errexit;
        }
#endif
    }

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    PRINT_INFO("log_event: +++");
    snt_nanohub_wake_device(snt8100fsr_g);
    snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_GET | FW_REG_LOG_OP_START);
#endif
#endif

    // request write of flash register init partition
    sc_cmd->magic = SC_MAGIC_NUMBER;
    sc_cmd->address = get_sys_param_event_log;
    sc_cmd->length = 0;
    sc_cmd->major = mc_get_sys_param;
    sc_cmd->minor = min_ok;
    sc_cmd->trans_id = get_time_in_ms();    // unique number

    ret = sb_write_fifo(snt8100fsr,
                        REGISTER_FIFO_COMMAND,
                        SC_COMMAND_HEADER_SIZE + sc_cmd->length,
                        sc_cmd);

    if (ret) {
        PRINT_CRIT("Unable to write to command fifo: %d", ret);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
        snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_GET | FW_REG_LOG_OP_STOP);
#endif
#endif
        goto errexit;
    }
    snt8100fsr->active_sc_cmd = mc_get_sys_param;

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done %d", ret);
    return ret;

 errexit:
    if (snt8100fsr->event_log_file != NULL) {
        file_close(snt8100fsr->event_log_file);
        snt8100fsr->event_log_file = NULL;
    }
    if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }
    goto cleanup;
}

void event_logging_cleanup(struct snt8100fsr *snt8100fsr)
{
    if (snt8100fsr->event_log_file != NULL) {
        file_close(snt8100fsr->event_log_file);
        snt8100fsr->event_log_file = NULL;
    }
}


/*==========================================================================*/
/* Read SC Response                                                          */
/*==========================================================================*/


int snt_read_sc_rsp(struct snt8100fsr *snt8100fsr) {

    int ret = -1;

    if (sc_cmd == NULL) {
        PRINT_CRIT("sc_cmd not initialized!");
        goto cleanup;
    }

    memset(sc_cmd, 0, sizeof(*sc_cmd));
    // Read the entire sonacomm header
    ret = sb_read_fifo(snt8100fsr,
                       REGISTER_FIFO_COMMAND,
                       SC_OVERHEAD, // 16 bytes
                       sc_cmd);

    if (ret) {
        PRINT_CRIT("Unable to read command fifo length: %x", ret);
        goto cleanup;
    }

    if (sc_cmd->length > SC_DATA_PAYLOAD_BYTES) {
        PRINT_CRIT("Command fifo length is too large: %d", sc_cmd->length);
        ret = -1;
        goto cleanup;
    }

    PRINT_DEBUG("sc_cmd->length = %d bytes", sc_cmd->length);

    // Read the rest of the sonacomm data
    ret = sb_read_fifo(snt8100fsr,
                       REGISTER_FIFO_COMMAND,
                       sc_cmd->length,
                       &sc_cmd->data[0]);

    if (ret) {
        PRINT_CRIT("Unable to read command fifo data: %x", ret);
        goto cleanup;
    }

cleanup:
    return ret;
}


/*==========================================================================*/
/* D1Test LOGGING                                                           */
/*==========================================================================*/

static void d1test_cleanup(struct snt8100fsr *snt8100fsr) {

    PRINT_FUNC();

    if (sc_cmd != NULL) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }

    if (log_d1test_file != NULL) {
        PRINT_DEBUG("Closing d1test log file: %s",
                    D1TEST_DATA_LOG_LOCATION);
        file_close(log_d1test_file);
        log_d1test_file = NULL;
    } else {
        PRINT_NOTICE("d1test logging already disabled");
    }
    log_d1test_num_frames = 0;
    log_d1test_loop = 0;
    if (snt8100fsr->active_sc_cmd == mc_d1_test)
        snt8100fsr->active_sc_cmd = mc_no_command;
    PRINT_DEBUG("done. active_sc_cmd = %d", snt8100fsr->active_sc_cmd);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, LOG_D1TEST | FW_REG_LOG_OP_STOP);
    PRINT_INFO("log_d1test: ---");
#endif
#endif
}



int enable_d1test_logging(struct snt8100fsr *snt8100fsr, int num_frames) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    if (num_frames == 65535) {
        log_d1test_loop = 1;
    }
    else {
        log_d1test_loop = 0;
    }
    log_d1test_num_frames = num_frames;

    if (log_d1test_num_frames > 0) {
        if (snt8100fsr->active_sc_cmd != mc_no_command) {
            PRINT_ERR("Cannot start d1Test. SC Cmd %d Active", snt8100fsr->active_sc_cmd);
            mutex_unlock(&snt8100fsr->sb_lock);
            return -1;
        }
        if (log_d1test_loop == 0) {
            if (log_d1test_file == NULL) {
                PRINT_DEBUG("Creating d1test log file: %s",
                            D1TEST_DATA_LOG_LOCATION);
                log_d1test_file_offset = 0;
                ret = file_open(D1TEST_DATA_LOG_LOCATION,
                                O_WRONLY|O_CREAT|O_TRUNC,
                                0777,
                                &log_d1test_file);
                if(ret) {
                    PRINT_DEBUG("Unable to create file '%s', error %d",
                                D1TEST_DATA_LOG_LOCATION, ret);
                    mutex_unlock(&snt8100fsr->sb_lock);
                    return -1;
                }
            } else {
                 PRINT_NOTICE("d1test logging already enabled");
                 mutex_unlock(&snt8100fsr->sb_lock);
                 return 0;
            }
       }
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
        PRINT_INFO("log_d1test: +++");
        snt_nanohub_wake_device(snt8100fsr_g);
        snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
        snt_nanohub_bypass_ctrl(snt8100fsr, LOG_D1TEST | FW_REG_LOG_OP_START);
#endif
#endif
    }

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
#ifdef CONFIG_HTC_FEATURE
        if (sc_cmd == NULL) {
            d1test_cleanup(snt8100fsr);
            mutex_unlock(&snt8100fsr->sb_lock);
            return -1;
        }
#endif
    }
    sc_cmd->magic = SC_MAGIC_NUMBER;
    sc_cmd->address = 0;
    sc_cmd->major = mc_d1_test;
    sc_cmd->trans_id = get_time_in_ms();
    if (log_d1test_num_frames > 0) {
        sc_cmd->data[0] = log_d1test_num_frames;
        sc_cmd->minor = min_ok;
        sc_cmd->length = sizeof(uint32_t);
    } else {
        sc_cmd->minor = min_data_last;
        sc_cmd->length = 0;
    }

    snt8100fsr->active_sc_cmd = mc_d1_test;
    ret = sb_write_fifo(snt8100fsr,
                        REGISTER_FIFO_COMMAND,
                        SC_COMMAND_HEADER_SIZE + sc_cmd->length,
                        sc_cmd);

    if (ret) {
        PRINT_CRIT("Unable to write to command fifo: %d", ret);
        d1test_cleanup(snt8100fsr);
        mutex_unlock(&snt8100fsr->sb_lock);
        return -1;
    }

    if (log_d1test_num_frames == 0) {
        d1test_cleanup(snt8100fsr);
    }

    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return 0;
}

static void log_d1test(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        snt8100fsr->active_sc_cmd = mc_no_command;
        goto cleanup;
    }

    mutex_lock(&snt8100fsr_g->sc_cmd_read_lock);
    if(snt8100fsr->sc_cmd_buf) {
        memcpy(snt8100fsr->sc_cmd_buf, sc_cmd, sizeof(struct sc_command));
        snt8100fsr_g->sc_cmd_buf_ready = true;
    }
    mutex_unlock(&snt8100fsr_g->sc_cmd_read_lock);

    // Write the data portion to our log file
    if (log_d1test_loop == 0) {
        if(log_d1test_file != NULL) {
            file_write(log_d1test_file,
                       log_d1test_file_offset,
                       (unsigned char*)&sc_cmd->data[0],
                       sc_cmd->length);
            log_d1test_file_offset += sc_cmd->length;
        } else {
            PRINT_NOTICE("d1test data received with no open log file");
        }
    }
    log_d1test_num_frames--;
    if (log_d1test_num_frames <= 0 && log_d1test_loop == 0) {
        d1test_cleanup(snt8100fsr);
    }

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");

    if (log_d1test_num_frames <= 0 && log_d1test_loop == 1) {
        if (snt8100fsr->active_sc_cmd == mc_d1_test)
            snt8100fsr->active_sc_cmd = mc_no_command;
        enable_d1test_logging(snt8100fsr_g, 65535);
    }

}

/*==========================================================================*/
/* FRAME DATA LOGGING                                                       */
/*==========================================================================*/

void frame_logging_cleanup(struct snt8100fsr *snt8100fsr)
{
     PRINT_FUNC();

    if (frame_buffer != NULL) {
        memory_free(frame_buffer);
        frame_buffer = NULL;
    }

    if (sc_cmd != NULL) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }

    snt8100fsr->active_sc_cmd = mc_no_command;
    snt8100fsr->frame_stream_count = 0;


    if (log_frame_file != NULL) {
        PRINT_DEBUG("Closing frame data log file: %s",
                    FRAME_DATA_LOG_LOCATION);
        file_close(log_frame_file);
        log_frame_file = NULL;
    }

    PRINT_DEBUG("done");
}

int enable_frame_logging(struct snt8100fsr *snt8100fsr,
                         uint32_t frame_count) {
    int ret;

    // Enable or disable capture based on frame count
    bool enable = (frame_count > 0);

    PRINT_FUNC("%d frames", frame_count);

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure.
     */
    mutex_lock(&snt8100fsr->sb_lock);

    if (enable) {
        if (snt8100fsr->active_sc_cmd != mc_no_command) {
            PRINT_CRIT("Cannot perform Frame Logging. Command (%d) already active", snt8100fsr->active_sc_cmd);
            mutex_unlock(&snt8100fsr->sb_lock);
            return -1;
        }
        if (log_frame_file == NULL) {
            PRINT_DEBUG("Creating frame data log file: %s",
                        FRAME_DATA_LOG_LOCATION);
            log_frame_file_offset = 0;
            ret = file_open(FRAME_DATA_LOG_LOCATION,
                            O_WRONLY|O_CREAT|O_TRUNC,
                            0777,
                            &log_frame_file);
            if(ret) {
                PRINT_DEBUG("Unable to create file '%s', error %d",
                            FRAME_DATA_LOG_LOCATION, ret);
                mutex_unlock(&snt8100fsr->sb_lock);
                return -1;
            }

            /*
             * Until we get our command response with the frame size on stream fifo,
             * set the frame_size to 0
             */
            frame_size = 0;
        } else {
            PRINT_NOTICE("Frame data logging already enabled");
            mutex_unlock(&snt8100fsr->sb_lock);
            return 0;
        }

        if (sc_cmd == NULL) {
            sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
        }

        if (frame_buffer == NULL) {
            frame_buffer = memory_allocate(FRAME_BUFFER_SIZE, GFP_DMA);
        }
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
        PRINT_INFO("log_frames: +++");
        snt_nanohub_wake_device(snt8100fsr_g);
        snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
        snt_nanohub_bypass_ctrl(snt8100fsr, LOG_FRAMES | FW_REG_LOG_OP_START);
#endif
#endif
    }

    // Capture this many frames
    snt8100fsr->frame_stream_count = frame_count;

    // Enable the frame data logging

    if (sc_cmd) {
        // Enable the frame data logging
        sc_cmd->magic = SC_MAGIC_NUMBER;
        sc_cmd->address = 0;
        sc_cmd->length = 0;
        sc_cmd->major = mc_frame_dump;
        sc_cmd->minor = enable ? min_ok : min_data_last;
        sc_cmd->trans_id = get_time_in_ms();
        sc_cmd->length = sizeof(sc_cmd->data[0]);
        sc_cmd->data[0] = frame_count;

        ret = sb_write_fifo(snt8100fsr,
                            REGISTER_FIFO_COMMAND,
                            SC_COMMAND_HEADER_SIZE + sizeof(sc_cmd->data[0]),
                            sc_cmd);

        if (ret) {
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
            snt_nanohub_bypass_ctrl(snt8100fsr, LOG_FRAMES | FW_REG_LOG_OP_STOP);
#endif
#endif
            mutex_unlock(&snt8100fsr->sb_lock);
            PRINT_CRIT("Unable to write to command fifo: %d", ret);
            return -1;
        }
    }
    snt8100fsr->active_sc_cmd = mc_frame_dump;

    if (!enable) {
        /* Temporary fix: don't free the memory
        if (frame_buffer != NULL) {
            memory_free(frame_buffer);
            frame_buffer = NULL;
        }

        if (sc_cmd != NULL) {
            memory_free(sc_cmd);
            sc_cmd = NULL;
        }*/

        snt8100fsr->active_sc_cmd = mc_no_command;

        if (log_frame_file != NULL) {
            PRINT_DEBUG("Closing frame data log file: %s",
                        FRAME_DATA_LOG_LOCATION);
            file_close(log_frame_file);
            log_frame_file = NULL;
        } else {
            PRINT_NOTICE("Frame data logging already disabled");
        }
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
        snt_nanohub_bypass_ctrl(snt8100fsr, LOG_FRAMES | FW_REG_LOG_OP_STOP);
        PRINT_INFO("log_frames: ---");
#endif
#endif
    }

    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return 0;
}

static void log_frame_cmd(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        goto cleanup;
    }

    // sc_cmd->data[0] is the length of the frame to read from stream fifo
    frame_size = sc_cmd->data[0];

    if (frame_size > 16384) {
        PRINT_CRIT("frame_size too large (max %d): %d bytes",
                   16384,
                   frame_size);
        goto cleanup;
    }

    PRINT_INFO("Frame data size %d bytes in stream fifo", frame_size);

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
}

static void log_frame_stream(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    mutex_lock(&snt8100fsr->sb_lock);

    if (frame_size == 0) {
        PRINT_CRIT("Frame size not received on command fifo!");
        goto cleanup;
    }

    if (frame_buffer) {
        // Read the length and frame size
        ret = sb_read_fifo(snt8100fsr,
                           REGISTER_FIFO_STREAM,
                           FRAME_LENGTH_WIDTH + frame_size,
                           frame_buffer);

        if (ret) {
            PRINT_CRIT("Unable to read stream fifo length: %d", ret);
            goto cleanup;
        }

        // Write the frame buffer to our log file, skipping the LENGTH header
        if(log_frame_file != NULL) {
            file_write(log_frame_file,
                       log_frame_file_offset,
                       &frame_buffer[FRAME_LENGTH_WIDTH],
                       frame_size);
            log_frame_file_offset += frame_size;
        } else {
            PRINT_NOTICE("Frame data received with no open log file");
        }

        snt8100fsr->frame_stream_count--;
    } else {
        sc_flush(snt8100fsr);
        PRINT_DEBUG("Cleaning up spurious streamed frame");
        frame_logging_cleanup(snt8100fsr);
        goto cleanup;
    }

    if (snt8100fsr->frame_stream_count < 1) {
        snt8100fsr->active_sc_cmd = mc_no_command;
        PRINT_NOTICE("All frame stream data received, closing log file");
        mutex_unlock(&snt8100fsr->sb_lock);
        enable_frame_logging(snt8100fsr, 0);
        PRINT_DEBUG("done");
        return;
    }

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
}

/*==========================================================================*/
/* NO TOUCH FRAME DUMP                                                      */
/*==========================================================================*/

void no_touch_logging_cleanup(struct snt8100fsr *snt8100fsr)
{
    if (log_no_touch_frame_file) {
        file_close(log_no_touch_frame_file);
        log_no_touch_frame_file = NULL;
    }
    snt8100fsr->active_sc_cmd = mc_no_command;

    if (sc_cmd != NULL) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, LOG_NO_TOUCH_FRAME | FW_REG_LOG_OP_STOP);
    PRINT_INFO("log_no_touch_frame: ---");
#endif
#endif
}

int enable_no_touch_logging(struct snt8100fsr *snt8100fsr) {
    /*
     * This method must not be called from our workerqueue. It will generally
     * be called from sysfs_no_touch_show(). This method will wait for
     * interrupts and use the workerqueue to gather the data, then we will
     * respond with the final result once we have it
     */
    int ret;

    PRINT_FUNC();

    if (snt8100fsr->active_sc_cmd != mc_no_command) {
        PRINT_CRIT("Cannot perform NoTouch Logging. Command (%d) already active", snt8100fsr->active_sc_cmd);
        mutex_unlock(&snt8100fsr->sb_lock);
        return -1;
    }

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure */
    mutex_lock(&snt8100fsr->sb_lock);

    if (log_no_touch_frame_file == NULL) {
        PRINT_DEBUG("Creating no_touch data log file: %s",
                    NO_TOUCH_FRAME_DATA_LOG_LOCATION);
        log_no_touch_frame_file_offset = 0;
        ret = file_open(NO_TOUCH_FRAME_DATA_LOG_LOCATION,
                        O_WRONLY|O_CREAT|O_TRUNC,
                        0777,
                        &log_no_touch_frame_file);
        if(ret) {
            PRINT_DEBUG("Unable to create file '%s', error %d",
                        NO_TOUCH_FRAME_DATA_LOG_LOCATION, ret);
            mutex_unlock(&snt8100fsr->sb_lock);
            return -1;
        }
    } else {
        PRINT_NOTICE("no_touch frame logging already enabled");
    }

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
#ifdef CONFIG_HTC_FEATURE
        if (sc_cmd == NULL) {
            mutex_unlock(&snt8100fsr->sb_lock);
            return -1;
        }
#endif
    }

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    PRINT_INFO("log_no_touch_frame: +++");
    snt_nanohub_wake_device(snt8100fsr_g);
    snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
    snt_nanohub_bypass_ctrl(snt8100fsr, LOG_NO_TOUCH_FRAME | FW_REG_LOG_OP_START);
#endif
#endif

    // Enable the no_touch sensor frame logging
    sc_cmd->magic = SC_MAGIC_NUMBER;
    sc_cmd->address = 0;
    sc_cmd->length = 0;
    sc_cmd->major = mc_no_touch;
    sc_cmd->minor = min_ok;
    sc_cmd->trans_id = get_time_in_ms();
    sc_cmd->length = 0;

    ret = sb_write_fifo(snt8100fsr,
                        REGISTER_FIFO_COMMAND,
                        SC_COMMAND_HEADER_SIZE,
                        sc_cmd);

    if (ret) {
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
        snt_nanohub_bypass_ctrl(snt8100fsr, LOG_NO_TOUCH_FRAME | FW_REG_LOG_OP_STOP);
#endif
#endif
        mutex_unlock(&snt8100fsr->sb_lock);
        PRINT_CRIT("Unable to write to command fifo: %d", ret);
        return -1;
    }
    snt8100fsr->active_sc_cmd = mc_no_touch;

    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return 0;
}

static void log_no_touch_frame(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        goto errexit;
    }

    // Write the data portion to our log file
    if(log_no_touch_frame_file != NULL) {
        file_write(log_no_touch_frame_file,
                   log_no_touch_frame_file_offset,
                   (unsigned char*)&sc_cmd->data[0],
                   sc_cmd->length);
        log_no_touch_frame_file_offset += sc_cmd->length;
    } else {
        PRINT_WARN("no_touch frame data received with no open log file");
    }

    // Check if we are done receiving data
    if (sc_cmd->minor == min_data_last) {
        PRINT_DEBUG("Received last no_touch frame data");
        no_touch_logging_cleanup(snt8100fsr);
    }

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;

errexit:
    no_touch_logging_cleanup(snt8100fsr);
    goto cleanup;
}

#define DIGIT1     0
#define DIGIT2     1
#define FIRSTHEX   2
#define READNUM    3
#define COMMENT    4


int CvtAscii2Digit(uint8_t byte, int radix)
{
  int digit = -1;
  if (byte >= '0' && byte <= '9') {
    digit = byte - '0';
  } else if (radix == 16 && byte >= 'a' && byte <= 'f') {
    digit = byte - 'a' + 10;
  } else if (radix == 16 && byte >= 'A' && byte <= 'F') {
    digit = byte - 'A' + 10;
  }
  return digit;
}

////////////////////////////////////////////////
//
// Reads number of the form: 1234 or 0x12ab or -53
// hex numbers must start with 0x (not 0X)
// can have comments "#"
// RETURNS:
//    0 - success
//   -1 - EOB
//   -2 - Parse error
///////////////////////////////////////////////
int ReadNumFromBuf(const uint8_t **p_in, int *count, uint32_t *ret_val)
{
    uint8_t byte;
    int state = DIGIT1;
    int radix = 10;
#ifdef CONFIG_HTC_FEATURE
    int comment_state = COMMENT;
#else
    int comment_state;
#endif
    int val = 0;
    int sign = 1;

    PRINT_FUNC("%d bytes", *count);

    while (*count > 0) {
        byte = *(*p_in);
        *p_in += 1;         // change the reference value
        *count -= 1;

        if (byte == '#') {
            comment_state = state;
            PRINT_DEBUG("Start Comment\n");
            state = COMMENT;
        }

        PRINT_DEBUG("state=%d, val=0x%08x, byte=%x, radix=%d\n", state, (uint32_t)val, byte, radix);

        switch (state) {

            case DIGIT1: {
                if (byte == '-') {
                    sign = -1;
                    state = READNUM;
                } else {
                    int digit = CvtAscii2Digit(byte, radix);
                    if (digit >= 0) {
                    val = digit;
                    state = DIGIT2;
                    }
                }
            }
            break;

            case DIGIT2: {
                if (byte == 'x') {
                    if (val != 0) {
                        PRINT_ERR("x Embedded in non-Hex number %d", val);
                        return -2;
                    }
                    radix = 16;
                    state = FIRSTHEX;
                    break;
                } else {
                    state = READNUM;
                }
            }

            /* FALLTHROUGH */

            case READNUM: {
                int digit = CvtAscii2Digit(byte, radix);
                if (digit >= 0) {
                    val = val*radix + sign*digit;
                } else {
                    // done. number found
                    *ret_val = (uint32_t) val;
                    PRINT_DEBUG("done. val=%d (0x%x), count=%d", val, *ret_val, *count);
                    return 0;
                }
            }
            break;

            case FIRSTHEX: {
                int digit = CvtAscii2Digit(byte, radix);
                if (digit >= 0) {
                    val = digit;
                    state = READNUM;
                } else {
                    PRINT_DEBUG("No digits following 0x\n");
                    return(-2);
                }
            }
            break;

            case COMMENT: {
                if (byte == 0x0a || byte == 0x0d)  {
                    PRINT_DEBUG("END COMMENT\n");
                    state = comment_state;
                }
            }
            break;

            default:
                PRINT_CRIT("Unknown State %d\n", state);
                return -2;
        }
    }
    PRINT_DEBUG("done. EOB count=%d", *count);
    return -1;
}
#undef DIGIT1
#undef DIGIT2
#undef FIRSTHEX
#undef READNUM
#undef COMMENT


#ifdef SUPPORT_FLASH

/*==========================================================================*/
/* FWUPDATE                                                                 */
/*==========================================================================*/

void fwupdate_cleanup(struct snt8100fsr *snt8100fsr) {
    PRINT_FUNC();

    if (sc_cmd != NULL) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }
    if (snt8100fsr->fwupdate_file != NULL) {
        PRINT_DEBUG("Closing FWUpdate file");
        file_close(snt8100fsr->fwupdate_file);
        snt8100fsr->fwupdate_file = NULL;
    }
#if !defined(CONFIG_HTC_FEATURE)
    snt8100fsr->fwupdate_tx_mtu = 0;
#endif
    snt8100fsr->active_sc_cmd = mc_no_command;

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, FW_FLASH_DOWNLOAD | FW_REG_LOG_OP_STOP);
    PRINT_INFO("fwupdate: ---");
#endif
#endif
    return;
}

static inline int fwupdate_get_method(const char * s) {
    const char *s_base = s;
    const char *s_limit = s + PAGE_SIZE;
    int  ret = mc_fw_update;
    if (s == NULL) return ret;

    while (s < s_limit && *s != '\0' ) {
        s++;
    }

    if ((int)(s - s_base) >= 6 &&
      *s  == '\0' &&
      *(s-1) == 'h' &&
      *(s-2) == 's' &&
      *(s-3) == 'a' &&
      *(s-4) == 'l' &&
      *(s-5) == 'f' &&
      *(s-6) == '.'
      ) {
        PRINT_DEBUG("flash image");
        ret = mc_flash_update;
    } else {
        PRINT_DEBUG("update image");
    }
    return ret;
}


int fwupdate_send_mtu(struct snt8100fsr *snt8100fsr) {
    int ret = 0;

    PRINT_FUNC("tx_mtu %d", snt8100fsr->fwupdate_tx_mtu);

    if (snt8100fsr->fwupdate_tx_mtu) {
        int mtu_bytes = file_read(snt8100fsr->fwupdate_file,
                        snt8100fsr->fwupdate_address,
                        (uint8_t*) &sc_cmd->data[0],
                        FW_BUFFER_SIZE);
        PRINT_DEBUG("fwupdate read address 0x%x, bytes %d", snt8100fsr->fwupdate_address, mtu_bytes);
        if (mtu_bytes < 0) {
            PRINT_CRIT("read of FWUpdate file failed %d", mtu_bytes);
            return mtu_bytes;
        }
        if (mtu_bytes && (mtu_bytes < FW_BUFFER_SIZE))
            mtu_bytes -= FWImageTrailerSize;

        if (mtu_bytes) {
            uint32_t checksum = snt_crc_checksum_buf((uint8_t*)&sc_cmd->data[0], mtu_bytes);
            int      checksum_i = mtu_bytes/sizeof(uint32_t);
            PRINT_DEBUG("crc 0x%08x", checksum);
            sc_cmd->data[checksum_i] = checksum;
            sc_cmd->length = mtu_bytes + sizeof(checksum);
        } else {
            PRINT_DEBUG("Eof reached");
            sc_cmd->length = 0;
        }
        sc_cmd->minor = (snt8100fsr->fwupdate_tx_mtu == 1) ? min_data_last : min_data_frag;
        sc_cmd->address = snt8100fsr->fwupdate_address;
        PRINT_DEBUG("maj=%d, min=%d, addr=%x, len=%d",
                     sc_cmd->major, sc_cmd->minor, snt8100fsr->fwupdate_address, sc_cmd->length);
        ret = sb_write_fifo(snt8100fsr,
                            REGISTER_FIFO_COMMAND,
                            SC_COMMAND_HEADER_SIZE + sc_cmd->length,
                            sc_cmd);

        snt8100fsr->fwupdate_address += mtu_bytes;
        if (ret) {
            PRINT_CRIT("Unable to write to command fifo: %d", ret);
            snt8100fsr->active_sc_cmd = mc_no_command;
            return -1;
        }
    } else {
        PRINT_DEBUG("FWUpdate Complete");
        fwupdate_cleanup(snt8100fsr);
    }
    PRINT_DEBUG("done");
    return ret;
}


int enable_fwupdate(struct snt8100fsr *snt8100fsr, const char *fname) {
    /*
     * This method must not be called from our workerqueue. It will generally
     * be called from sysfs_fwupdate_store(). This method will wait for
     * interrupts and use the workerqueue to trigger when to send update frames.
     */
    int ret = -1;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure */
    mutex_lock(&snt8100fsr->sb_lock);

    if (snt8100fsr->active_sc_cmd != mc_no_command) {
        PRINT_CRIT("Cannot perform firmware/flash update. Command (%d) already active", snt8100fsr->active_sc_cmd);
        snt8100fsr->fwupdate_status = FWUPDATE_STATUS_DRIVER_ERR;
        mutex_unlock(&snt8100fsr->sb_lock);
        return -1;
    }

    if (snt8100fsr->fwupdate_file == NULL) {
        snt8100fsr->fwupdate_status = FWUPDATE_STATUS_OKAY;
        snt8100fsr->fwupdate_tot_mtu = 0;
        snt8100fsr->fwupdate_major = fwupdate_get_method(fname);
        snt8100fsr->fwupdate_address = 0;
        PRINT_DEBUG("Opening fwupdate file %s", fname);
        ret = file_open(fname, O_RDONLY, 0, &snt8100fsr->fwupdate_file);
        if (ret) {
#ifdef CONFIG_HTC_FEATURE
            PRINT_CRIT("Unable to open file %s, error %d", fname, ret);
#else
            PRINT_DEBUG("Unable to open file %s, error %d", fname, ret);
#endif
            snt8100fsr->fwupdate_status = FWUPDATE_STATUS_FILE_ERR;
            goto cleanup;
        }
        ret = file_size(snt8100fsr->fwupdate_file, &snt8100fsr->fwupdate_size);
        if (ret) {
#ifdef CONFIG_HTC_FEATURE
            PRINT_CRIT("Unable to get file size error %d", ret);
#else
            PRINT_DEBUG("Unable to get file size error %d", ret);
#endif
            snt8100fsr->fwupdate_status = FWUPDATE_STATUS_FILE_ERR;
            goto cleanup;
        }
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
        PRINT_INFO("fwupdate: +++");
        snt_nanohub_wake_device(snt8100fsr_g);
//        snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
        snt_nanohub_bypass_ctrl(snt8100fsr, FW_FLASH_DOWNLOAD | FW_REG_LOG_OP_START);
#endif
#endif
        // num tx packets = ceil(filesize-Trailer/BUFF_SIZE) + 1. Last one is for "done" indicator
        snt8100fsr->fwupdate_tx_mtu = ((snt8100fsr->fwupdate_size-FWImageTrailerSize+FW_BUFFER_SIZE-1) / FW_BUFFER_SIZE) + 1;
        snt8100fsr->fwupdate_tot_mtu = snt8100fsr->fwupdate_tx_mtu;
        PRINT_DEBUG("fwupdate file size %d, num mtu %d", snt8100fsr->fwupdate_size, snt8100fsr->fwupdate_tx_mtu);

    } else {
        PRINT_NOTICE("FWUpdate already enabled");
#ifdef CONFIG_HTC_FEATURE
        ret = 0;
#endif
        goto cleanup;
    }

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
    }
    if (sc_cmd == NULL) {
        PRINT_DEBUG("Unable to allocate sc cmd for fwupdate");
        snt8100fsr->fwupdate_status = FWUPDATE_STATUS_DRIVER_ERR;
#ifdef CONFIG_HTC_FEATURE
        ret = -1;
#endif
        goto cleanup;
    }

    sc_cmd->magic = SC_MAGIC_NUMBER;
    sc_cmd->major = snt8100fsr->fwupdate_major;
    sc_cmd->address = snt8100fsr->fwupdate_address;
    sc_cmd->length = 0;
    sc_cmd->trans_id = get_time_in_ms();

    snt8100fsr->active_sc_cmd = snt8100fsr->fwupdate_major;

    ret = fwupdate_send_mtu(snt8100fsr);
    if (ret) {
        PRINT_CRIT("FWUpdate send failed %d", ret);
        snt8100fsr->fwupdate_status = FWUPDATE_STATUS_ABORT;
        fwupdate_cleanup(snt8100fsr);
    }

cleanup:
    PRINT_DEBUG("done");
#ifdef CONFIG_HTC_FEATURE
    PRINT_INFO("tx_mtu=%d, tot_mtu=%d, status=%d",
                    snt8100fsr_g->fwupdate_tx_mtu,
                    snt8100fsr_g->fwupdate_tot_mtu,
                    snt8100fsr_g->fwupdate_status);
    if (ret)
        snt8100fsr->fwupdate_tx_mtu = -1;
#endif
    mutex_unlock(&snt8100fsr->sb_lock);
    return ret;
}


static void fwupdate_rsp_ind(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC("Enter");

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        snt8100fsr->fwupdate_status = FWUPDATE_STATUS_ABORT;
        goto err_exit;
    }

    PRINT_DEBUG("FWUpdate mtu rsp maj=%d, min=%d, crc=0x%08x, address=0x%08x",
                    sc_cmd->major, sc_cmd->minor, sc_cmd->data[0], sc_cmd->address);

    if (sc_cmd->major != snt8100fsr->fwupdate_major) {
        PRINT_CRIT("Unexpected cmd response %d", sc_cmd->major);
        snt8100fsr->fwupdate_status = FWUPDATE_STATUS_DATA_BAD;
        goto err_exit;
    }
    if (sc_cmd->minor == min_data_bad) {
        PRINT_CRIT("Invalid file format");
        snt8100fsr->fwupdate_status = sc_cmd->minor;
        goto err_exit;
    }
    if (sc_cmd->minor == min_bad_part1) {
        PRINT_CRIT("Flash partition error");
        snt8100fsr->fwupdate_status = sc_cmd->minor;
        goto err_exit;
    }
    if (sc_cmd->minor == min_not_supported) {
        PRINT_CRIT("Command not supported by firmware");
        snt8100fsr->fwupdate_status = sc_cmd->minor;
        goto err_exit;
    }

    snt8100fsr->fwupdate_tx_mtu--;
    ret = fwupdate_send_mtu(snt8100fsr);
    if (ret) {
        PRINT_CRIT("FWUpdate send error %d", ret);
        snt8100fsr->fwupdate_status = FWUPDATE_STATUS_ABORT;
        goto err_exit;
    }

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;

err_exit:
#ifdef CONFIG_HTC_FEATURE
    PRINT_INFO("tx_mtu=%d, tot_mtu=%d, status=%d",
                    snt8100fsr_g->fwupdate_tx_mtu,
                    snt8100fsr_g->fwupdate_tot_mtu,
                    snt8100fsr_g->fwupdate_status);
    snt8100fsr->fwupdate_tx_mtu = -1;
#endif
    fwupdate_cleanup(snt8100fsr);
    goto cleanup;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Program SPI Flash register init partition


#define STATE_REGBASE   0
#define STATE_NUMREG    1
#define STATE_REGVALS   2
#define STATE_ERROR     3

//////////////////////////////////////////////////////////////////////////////
//
// RETURNS
//    Number of bytes rounded up to closest multiple of 4 (sizeof uint32_t)
//////////////////////////////////////////////////////////////////////////////
int ReadRegList(const uint8_t *p_in, int in_buf_len, uint8_t *p_out, int out_buf_len)
{
    int       len = 0;
#ifdef CONFIG_HTC_FEATURE
    uint32_t  num_regs = 0;
#else
    uint32_t  num_regs;
#endif
    uint32_t  val;
    int       state = STATE_REGBASE;
    int       ret;

    if (out_buf_len < 2 || (out_buf_len % sizeof(uint32_t)) != 0) {
        PRINT_ERR("ERROR! Bad buffer length for Register Update (%d)\n", out_buf_len);
        state = STATE_ERROR;
        goto ReadRegListDone;
    }
    out_buf_len -= 2;    // remove 2 bytes for "end of file" marker

    while (len < out_buf_len) {
        ret = ReadNumFromBuf(&p_in, &in_buf_len, &val);
        if (ret == -1) {       // end of buffer reached
            PRINT_DEBUG("EOB");
            goto ReadRegListDone;
        }
        if (ret == -2) {      // parse error
            PRINT_ERR("Error processing SPI Register Initialization data\n");
            state = STATE_ERROR;
            goto ReadRegListDone;
        }

        switch (state) {
          case STATE_REGBASE: {
              if (val == 0x200) val = 0x82;
              if (val >= 0x000000ff) {
                PRINT_ERR("ERROR! reg_base out of bounds (%d: 0x%x)\n", len, val);
                state = STATE_ERROR;
                goto ReadRegListDone;
              }
              *p_out++ = (uint8_t)(val&0x000000ff);
              len++;
              state = STATE_NUMREG;
            }
            break;

          case STATE_NUMREG: {
              if (val > 0x000000ff) {
                PRINT_ERR("ERROR! num_regs out of bounds (%d: 0x%x)\n", len, val);
                state = STATE_ERROR;
                goto ReadRegListDone;
              }
              *p_out++ = (uint8_t)(val&0x000000ff);
              len++;
              if (val == 0) {           // no regs to read
                state = STATE_REGBASE;
              } else {
                num_regs = val;
                state = STATE_REGVALS;
              }
            }
            break;

          case STATE_REGVALS: {
              if (val & 0x80000000) val = val & 0x0000ffff;     // negative num trucate to uint16_t
              if (val > 0x0000ffff) {
                PRINT_ERR("ERROR! reg_val %d is out of bounds (%d: 0x%x)\n", num_regs, len, val);
                state=STATE_ERROR;
                goto ReadRegListDone;
              }
              *p_out++ = (uint8_t)(val & 0xff);
              *p_out++ = (uint8_t)((val>>8) & 0xff);
              len += 2;
              if (--num_regs == 0) {
                state = STATE_REGBASE;
              }
            }
            break;

          default: PRINT_ERR("ERROR! Unknown state in ReadRegList (%d)\n", state);
                   state=STATE_ERROR;
                   goto ReadRegListDone;
        }
    }

ReadRegListDone:
    if (state == STATE_ERROR) {
        PRINT_ERR("ERROR! Formatting Error\n");
        return(-1);
    }
    if (state != STATE_REGBASE) {
      PRINT_ERR("ERROR! Premature End of File (state = %d)\n", state);
      return(0);
    }

    // Put End of Data marker
    *p_out++ = 0;                        // reg_base == 0
    *p_out++ = 0;                        // num_regs == 0
    len += 2;
#ifdef CONFIG_HTC_FEATURE
    PRINT_DEBUG("done. len=%d, len32=%zu", len, ((len+(sizeof(uint32_t)-1))/sizeof(uint32_t))*sizeof(uint32_t));
#else
    PRINT_DEBUG("done. len=%d, len32=%d", len, ((len+(sizeof(uint32_t)-1))/sizeof(uint32_t))*sizeof(uint32_t));
#endif
    return ((len+(sizeof(uint32_t)-1))/sizeof(uint32_t))*sizeof(uint32_t); // ceil(number of uint32_t) bytes
}

#undef STATE_REGBASE
#undef STATE_NUMREG
#undef STATE_REGVALS
#undef STATE_ERROR

static void write_flash_reg_part_rsp(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        goto cleanup;
    }

    // Verify return
    if (sc_cmd->minor != min_ok) {
        PRINT_ERR("ERROR! return %d on update_regs_rsp", sc_cmd->minor);
        goto cleanup;
    }


cleanup:
    if (sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }
    snt8100fsr->active_sc_cmd = mc_no_command;
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, FW_REG_DOWNLOAD | FW_REG_LOG_OP_STOP);
    PRINT_INFO("write_flash_reg_part: ---");
#endif
#endif
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;
}




void enable_write_flash_reg_part_req(struct snt8100fsr *snt8100fsr,
                                       const char *buf,
                                       size_t count) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure */
    mutex_lock(&snt8100fsr->sb_lock);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    PRINT_INFO("write_flash_reg_part: +++");
    snt_nanohub_wake_device(snt8100fsr_g);
    snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
    snt_nanohub_bypass_ctrl(snt8100fsr, FW_REG_DOWNLOAD | FW_REG_LOG_OP_START);
#endif
#endif

    if (snt8100fsr->active_sc_cmd != mc_no_command) {
        PRINT_CRIT("Cannot perform firmware/flash update. Command (%d) already active", snt8100fsr->active_sc_cmd);
        goto errexit;
    }

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
#ifdef CONFIG_HTC_FEATURE
        if (sc_cmd == NULL)
            goto errexit;
#endif
    }

    // request write of flash register init partition
    sc_cmd->magic = SC_MAGIC_NUMBER;
    sc_cmd->address = 0;
    sc_cmd->length = ReadRegList(buf, count, (uint8_t*) &sc_cmd->data[0], sizeof(sc_cmd->data));
    sc_cmd->major = mc_update_regs;
    sc_cmd->minor = min_ok;
    sc_cmd->trans_id = get_time_in_ms();    // unique number

    if (sc_cmd->length == 0 || sc_cmd->length > sizeof(sc_cmd->data)) {
        PRINT_CRIT("ERROR processing input file for SPI register update (%d)", sc_cmd->length);
        goto errexit;
    }

    ret = sb_write_fifo(snt8100fsr,
                        REGISTER_FIFO_COMMAND,
                        SC_COMMAND_HEADER_SIZE + sc_cmd->length,
                        sc_cmd);

    if (ret) {
        PRINT_CRIT("Unable to write to command fifo: %d", ret);
        goto errexit;
    }
    snt8100fsr->active_sc_cmd = mc_update_regs;

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;

 errexit:
    if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, FW_REG_DOWNLOAD | FW_REG_LOG_OP_STOP);
#endif
#endif
    goto cleanup;
}
#endif // SUPPORT_FLASH

void enable_set_sys_param(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure */
    mutex_lock(&snt8100fsr->sb_lock);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    PRINT_INFO("set_sys_param: +++");
    snt_nanohub_wake_device(snt8100fsr_g);
    snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
    snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_SET | FW_REG_LOG_OP_START);
#endif
#endif

    if (snt8100fsr->active_sc_cmd != mc_no_command) {
        PRINT_CRIT("Cannot perform SetSysParam. Command (%d) already active", snt8100fsr->active_sc_cmd);
        goto errexit;
    }

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
#ifdef CONFIG_HTC_FEATURE
        if (sc_cmd == NULL)
            goto errexit;
#endif
    }

    // request write of flash register init partition
    sc_cmd->magic = SC_MAGIC_NUMBER;
    sc_cmd->address = snt8100fsr_g->set_sys_param_id;
    sc_cmd->length = 4;
    sc_cmd->major = mc_set_sys_param;
    sc_cmd->minor = min_ok;
    sc_cmd->trans_id = get_time_in_ms();    // unique number
    sc_cmd->data[0] = snt8100fsr_g->set_sys_param_val;

    ret = sb_write_fifo(snt8100fsr,
                        REGISTER_FIFO_COMMAND,
                        SC_COMMAND_HEADER_SIZE + sc_cmd->length,
                        sc_cmd);

    if (ret) {
        PRINT_CRIT("Unable to write to command fifo: %d", ret);
        goto errexit;
    }
    snt8100fsr->active_sc_cmd = mc_set_sys_param;

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;

 errexit:
    if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_SET | FW_REG_LOG_OP_STOP);
#endif
#endif
    goto cleanup;
}

static void set_sys_param_rsp(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        goto cleanup;
    }

    // Verify return
    if (sc_cmd->minor != min_ok) {
        PRINT_ERR("ERROR! return %d on update_regs_rsp", sc_cmd->minor);
        goto cleanup;
    }

    snt8100fsr->set_sys_param_status = sc_cmd->data[0];


cleanup:
    if (sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }
    snt8100fsr->active_sc_cmd = mc_no_command;
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_SET | FW_REG_LOG_OP_STOP);
    PRINT_INFO("set_sys_param: ---");
#endif
#endif
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;
}




void enable_get_sys_param(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure */
    mutex_lock(&snt8100fsr->sb_lock);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    PRINT_INFO("get_sys_param: +++");
    snt_nanohub_wake_device(snt8100fsr_g);
    snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
    snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_GET | FW_REG_LOG_OP_START);
#endif
#endif

    if (snt8100fsr->active_sc_cmd != mc_no_command) {
        PRINT_CRIT("Cannot perform GetSysParam. Command (%d) already active", snt8100fsr->active_sc_cmd);
        goto errexit;
    }

    if (snt8100fsr->get_sys_param_cmd != NULL) {
        memory_free(snt8100fsr->get_sys_param_cmd);
        snt8100fsr->get_sys_param_cmd = NULL;
    }

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
#ifdef CONFIG_HTC_FEATURE
        if (sc_cmd == NULL)
            goto errexit;
#endif
    }

    // request write of flash register init partition
    sc_cmd->magic = SC_MAGIC_NUMBER;
    sc_cmd->address = snt8100fsr_g->get_sys_param_id;
    sc_cmd->length = 0;
    sc_cmd->major = mc_get_sys_param;
    sc_cmd->minor = min_ok;
    sc_cmd->trans_id = get_time_in_ms();    // unique number

    ret = sb_write_fifo(snt8100fsr,
                        REGISTER_FIFO_COMMAND,
                        SC_COMMAND_HEADER_SIZE + sc_cmd->length,
                        sc_cmd);

    if (ret) {
        PRINT_CRIT("Unable to write to command fifo: %d", ret);
        goto errexit;
    }
    snt8100fsr->active_sc_cmd = mc_get_sys_param;

cleanup:
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;

 errexit:
    if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_GET | FW_REG_LOG_OP_STOP);
#endif
#endif
    goto cleanup;
}

static void get_sys_param_rsp(struct snt8100fsr *snt8100fsr) {
    int ret;

    PRINT_FUNC();

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        goto cleanup;
    }

    // Verify return
    if (sc_cmd->minor != min_ok) {
        PRINT_ERR("ERROR! return %d on update_regs_rsp", sc_cmd->minor);
        goto cleanup;
    }

    // event log is special case
    if (sc_cmd->address == get_sys_param_event_log) {
        if (snt8100fsr->event_log_file != NULL){
            file_write(snt8100fsr->event_log_file,
            0,
            (uint8_t*)sc_cmd->data,
            sc_cmd->length);
        }
        goto cleanup;
    } else {
        snt8100fsr->get_sys_param_status = sc_cmd->data[0];
        snt8100fsr->get_sys_param_val = sc_cmd->data[1];

        // deal with Gets that are greater than 4 bytes.
        if (sc_cmd->length > 8) {
            snt8100fsr->get_sys_param_cmd = sc_cmd;
            sc_cmd = NULL;
        }
    }

cleanup:
    if (snt8100fsr->event_log_file != NULL) {
        file_close(snt8100fsr->event_log_file);
        snt8100fsr->event_log_file = NULL;
    }
    if (sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }
    snt8100fsr->active_sc_cmd = mc_no_command;
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(snt8100fsr, SYS_PARAM_GET | FW_REG_LOG_OP_STOP);
    PRINT_INFO("get_sys_param / log_event: ---");
#endif
#endif
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;
}





static void spurious_cmd_event(struct snt8100fsr *snt8100fsr) {
    int ret;

#ifdef CONFIG_HTC_FEATURE
    PRINT_INFO("%s", __func__);
#else
    PRINT_FUNC();
#endif

    /*
     * We must mutex lock before using sc_cmd to protect it from other
     * commands which use the same structure
     */
    mutex_lock(&snt8100fsr->sb_lock);

    if (sc_cmd == NULL) {
        sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
    }

    ret = snt_read_sc_rsp(snt8100fsr);
    if (ret) {
        PRINT_CRIT("Read cmd Fifo rsp failed");
        goto cleanup;
    }

cleanup:
    if (sc_cmd) {
        memory_free(sc_cmd);
        sc_cmd = NULL;
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return;
}

void sc_flush(struct snt8100fsr *snt8100fsr)
{
    struct register_actions reg_actions;
    int ret;

    PRINT_FUNC();
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_wake_device(snt8100fsr_g);
    snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
#endif
#endif
    // reset snt8100fsr sc state machines
    memset((void*)&reg_actions, 0, sizeof(reg_actions));
    reg_actions.fcmd = 1;
    reg_actions.fstr = 1;
    ret = sb_write_fifo(snt8100fsr,
                        REGISTER_ACTIONS,
                        sizeof(reg_actions),
                        &reg_actions);
    if (ret) {
        PRINT_CRIT("write_register() failed");
    }
    PRINT_DEBUG("done");
}

void sc_cleanup(struct snt8100fsr *snt8100fsr)
{
    PRINT_FUNC();
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    PRINT_INFO("sc_reset: +++");
#endif
#endif

    mutex_lock(&snt8100fsr->sb_lock);

    sc_flush(snt8100fsr);

    d1test_cleanup(snt8100fsr);
    no_touch_logging_cleanup(snt8100fsr);

    #ifdef SUPPORT_FLASH
    fwupdate_cleanup(snt8100fsr);
    #endif

    frame_logging_cleanup(snt8100fsr);

    event_logging_cleanup(snt8100fsr);

    mutex_unlock(&snt8100fsr->sb_lock);
#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    PRINT_INFO("sc_reset: ---");
#endif
#endif
    PRINT_DEBUG("done");
    return;
}
