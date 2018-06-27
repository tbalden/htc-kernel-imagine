/*****************************************************************************
* File: firmware.c
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#ifdef CONFIG_HTC_FEATURE
#include <linux/firmware.h>
#endif

#include "config.h"
#include "irq.h"
#include "serial_bus.h"
#include "workqueue.h"
#include "event.h"
#include "file.h"
#include "memory.h"
#include "device.h"
#include "firmware.h"
#include "utils.h"
#include "debug.h"
#ifdef CONFIG_HTC_FEATURE
#include "nanohub_bus.h"
#endif

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
#define MAX_PAYLOAD_NUM     1000 // 1 = Hdr only
#define MAX_PAYLOAD_BYTES   (65 * 1024)

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct upload_work {
  struct delayed_work my_work;
  struct snt8100fsr   *snt8100fsr;
#ifdef CONFIG_HTC_FEATURE
  const struct firmware *fw;
#endif
  char                *filename;
  struct file         *f;
  int                 payload_num;
  int                 file_offset;

  int                 num_write;
  uint32_t            size;
  uint32_t            pay_write;

  bool                final_irq;
  bool                waiting_for_irq;

  uint32_t            firmware_upload_time;
  uint32_t            payload_upload_time;
  uint32_t            total_bytes_written;

  uint8_t             *data_in;
  uint8_t             *data_out;
};

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
static void upload_wq_func(struct work_struct *work);
static void upload_firmware_internal(struct upload_work *w);
static irqreturn_t irq_handler_top(int irq, void *dev);

static void download_wq_func(struct work_struct *work);
static void download_firmware_internal(struct upload_work *w);

static int open_firmware_file(struct upload_work *w);
static error_t firmware_waiting_irq(struct upload_work *w);
static error_t firmware_transfer_payloads(struct upload_work *w);
static void firmware_cleanup(struct upload_work *w, error_t err);
/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t bcr_log_level = 0;
static uint32_t bcr_addr      = 0;
static uint32_t bcr_irpt      = 0;

static struct upload_work *work;

/*==========================================================================*/
/* BOOT FIRMWARE CONFIG                                                     */
/*==========================================================================*/
void set_bcr_addr(uint32_t addr) {
    bcr_addr = (addr & BCR_I2CADDR_MASK) << BCR_I2CADDR_POS;
}

void set_bcr_log_lvl(uint32_t l) {
    bcr_log_level = (l & BCR_LOGLVL_MASK) << BCR_LOGLVL_POS;
}

void set_bcr_irpt_lvl(uint32_t l) {
    bcr_irpt |= (l & BCR_IRPTLVL_MASK) << BCR_IRPTLVL_POS;
}

void set_bcr_irpt_pol(uint32_t p) {
    bcr_irpt |= (p & BCR_IRPTPOL_MASK) << BCR_IRPTPOL_POS;
}

void set_bcr_irpt_dur(uint32_t d) {
    bcr_irpt |= (d & BCR_IRPTDUR_MASK) << BCR_IRPTDUR_POS;
}

/*
 * set_bcr_word()
 *
 * [31:24] - I2C address. valid 0,0x2c,0x2d,0x5c,0x5d (0 means "all 4")
 * [23:11] - Reserved.
 * [10:8]  - Logging Level
 * [7:2]   - Edge Duration in 78MHz tics. "0" means 10 tics (default).
 * [1]     - Interrupt Level. 0 - Level, 1 - Edge
 * [0]     - Interrupt Polarity. 0 - Active High, 1 - Active Low
 */
uint32_t set_bcr_word(void) {

  uint32_t bcr = bcr_addr
               | bcr_log_level
               | bcr_irpt;
  PRINT_DEBUG("Boot Cfg Record = 0x%08x", bcr);
  return bcr;
}

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
#ifdef CONFIG_HTC_FEATURE
static void upload_firmware_func(const struct firmware *fw, void *context) {
    struct upload_work *w = (struct upload_work *)context;
#if 0
    struct delayed_work *delayed_work =
            container_of(context, struct delayed_work, work);
    struct upload_work *w = container_of(delayed_work, struct upload_work, my_work);
#endif
    PRINT_FUNC();

    if (fw == NULL) {
        PRINT_ERR("Firmware does not exist!!");
//            start_event_processing(snt8100fsr_g);
        return;
    }

    w->fw = fw;

    PRINT_DEBUG("Firmware size = %d", (unsigned int)(w->fw->size));
    upload_firmware_internal(w);
    return;
}

static inline int secure_memcpy(unsigned char *dest, unsigned int dest_size,
        const unsigned char *src, unsigned int src_size,
        unsigned int count) {
    if (dest == NULL || src == NULL)
        return -ENOMEM;

    if (count > dest_size || count > src_size)
        return -EINVAL;

    memcpy((void *)dest, (const void *)src, count);
    return 0;
}
#endif

int upload_firmware_fwd(struct snt8100fsr *snt8100fsr, char *filename) {
    PRINT_FUNC();
    // We can't upload the firmware directly as we need to pace
    // ourselves for how fast the hardware can accept data. So we
    // must setup a background thread with a workerqueue and process
    // the upload a piece at a time.
    work = (void *)workqueue_alloc_work(sizeof(struct upload_work),
                                        download_wq_func);
    if (!work) {
        PRINT_CRIT(OOM_STRING);
        return -ENOMEM;
    }

    work->snt8100fsr = snt8100fsr;
    work->filename = FIRMWARE_LOCATION;

    // Allocate our data buffers in contiguous memory for DMA support
    work->data_in = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }

    work->data_out = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }

    // Setup our logging level
    set_bcr_log_lvl(BCR_LOGGING_LEVEL);

    workqueue_queue_work(work, 0);
    return 0;
}

int upload_firmware(struct snt8100fsr *snt8100fsr, char *filename) {
    // We can't upload the firmware directly as we need to pace
    // ourselves for how fast the hardware can accept data. So we
    // must setup a background thread with a workerqueue and process
    // the upload a piece at a time.
    work = (void *)workqueue_alloc_work(sizeof(struct upload_work),
                                        upload_wq_func);
    if (!work) {
        PRINT_CRIT(OOM_STRING);
        return -ENOMEM;
    }

    work->snt8100fsr = snt8100fsr;
    work->filename = FIRMWARE_LOCATION;
#ifdef CONFIG_HTC_FEATURE
    work->file_offset = 0;
    work->payload_num = 0;
#endif

#ifdef USE_NANOHUB_BUS
    // Allocate our data buffers in contiguous memory for DMA support
    work->data_in = memory_allocate(SNT_FWDL_BUF_RX_SIZE, GFP_DMA);
    if (work->data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", SNT_FWDL_BUF_RX_SIZE);
        return -ENOMEM;
    }
#else
    // Allocate our data buffers in contiguous memory for DMA support
    work->data_in = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }
#endif//#ifdef USE_NANOHUB_BUS

    work->data_out = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }

    // Setup our logging level
    set_bcr_log_lvl(BCR_LOGGING_LEVEL);

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_wake_device(snt8100fsr_g);
//    snt_nanohub_bypass_NonSCCmd_ctrl(snt8100fsr_g, DYNAMIC_POWER_CTRL);
    snt_nanohub_bypass_ctrl(work->snt8100fsr, FW_STREAM_UPLOAD | FW_REG_LOG_OP_START);
#endif
#endif
    workqueue_queue_work(work, 500);
    return 0;
}

int irq_handler_fwd( void) {

    int delay;

    // Add a delay in milliseconds if our boot loader is logging output.
    // During it's logging, it can't receive data, so we delay a bit.
    // We have a known amount of delay, so it's always safe.
    if (BCR_LOGGING_LEVEL != BCR_LOGLVL_OFF) {
        delay = FIRMWARE_LOG_DELAY_MS;
    } else {
        delay = 0;
    }

    work->waiting_for_irq = false;
    if (workqueue_mod_work(work, delay) == false) {
      workqueue_queue_work(work, delay);
    }
    return 0;
}

static void upload_wq_func(struct work_struct *work) {
#ifdef CONFIG_HTC_FEATURE
    if (request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, FIRMWARE_LOCATION,
            snt8100fsr_g->dev, GFP_KERNEL, work, upload_firmware_func)) {
        PRINT_ERR("request_firmware_nowait fail!\n");
    }
#else
    upload_firmware_internal((struct upload_work *)work);
#endif
    return;
}

static void download_wq_func(struct work_struct *work) {
    download_firmware_internal((struct upload_work *)work);
    return;
}

static irqreturn_t irq_handler_top(int irq, void *dev) {
    int delay;

    // Add a delay in milliseconds if our boot loader is logging output.
    // During it's logging, it can't receive data, so we delay a bit.
    // We have a known amount of delay, so it's always safe.
    if (BCR_LOGGING_LEVEL != BCR_LOGLVL_OFF) {
        delay = FIRMWARE_LOG_DELAY_MS;
    } else {
        delay = 0;
    }

    work->waiting_for_irq = false;
    if (workqueue_mod_work(work, delay) == false) {
        workqueue_queue_work(work, delay);
    }

    return IRQ_HANDLED;
}

static int open_firmware_file(struct upload_work *w) {
    int ret = 0;
    PRINT_FUNC("0x%p", w);
#ifdef CONFIG_HTC_FEATURE
    if (!w->fw) {
        PRINT_ERR("Unable to request firmware '%s'", w->filename);
        return -ENOMEM;
    }
    if ((w->fw->data == NULL) || (w->fw->size == 0)) {
        PRINT_ERR("No firmware received\n");
        return -ENOMEM;
    }
#else
    // If we haven't opened the firmware file, do so
    if (w->f == 0) {
        PRINT_DEBUG("Opening file: %s", w->filename);
        ret = file_open(w->filename, O_RDONLY, 0, &w->f);
        if(ret) {
            PRINT_ERR("Unable to open firmware file '%s', error %d",
                      w->filename, ret);
        }
    }
#endif
    return ret;
}


static error_t firmware_waiting_irq( struct upload_work *w) {
    PRINT_FUNC();
    /* If we are here, and are waiting for an IRQ,
     * then we have a timeout
     * condition as the IRQ didn't occurr.
     */
    if(FIRMWARE_UPLOAD_WITH_IRQ) {
        if (w->waiting_for_irq) {
            PRINT_CRIT("Timeout waiting for interrupt. Please ensure hardware "
                       "is correctly wired and firmware image is valid.");
            w->waiting_for_irq = false;
            return E_TIMEOUT;
        }

        w->waiting_for_irq = true;

        /* Queue a job to check for an IRQ timeout. The timer includes the
         * time to transfer a payload, so ensure it's long enough.
         */
//        workqueue_queue_work(work, FIRMWARE_IRQ_TIMEOUT_MS);
    }
    return E_SUCCESS;
}

static error_t firmware_transfer_payloads( struct upload_work *w) {
    int         ret;
    int         num_write;
    uint32_t    payload_write;
    uint32_t    payload_size;
    uint32_t    size;
    uint32_t    payload_duration;
    PRINT_FUNC();
    /*
     * Read size of next payload from the file.
     * Actually reading full
     * HW_FIFO_SIZE so will need to write this out
     */
#ifdef CONFIG_HTC_FEATURE
    w->snt8100fsr->chip_update_size_remain = w->fw->size - w->file_offset;
    num_write = min(SPI_FIFO_SIZE, w->snt8100fsr->chip_update_size_remain);
    PRINT_DEBUG("file_offset = %d, remain:%d, num_write:%d",
                w->file_offset, w->snt8100fsr->chip_update_size_remain, num_write);
    if (num_write > 0) {
        ret = secure_memcpy(w->data_out, SNT_FWDL_BUF_SIZE,
                            w->fw->data + w->file_offset, w->snt8100fsr->chip_update_size_remain,
                            num_write);
        if (ret < 0) {
            PRINT_ERR("Fail to copy firmware: offset:%d, remain:%d, num_write:%d",
                      w->file_offset, w->snt8100fsr->chip_update_size_remain, num_write);
            return E_BADREAD;
        }
    } else {
        PRINT_INFO("EOF Reached. Firmware data uploaded.");
        return E_FINISHED;
    }
    PRINT_DEBUG("Write size sector from byte %d to %d (Total %d bytes)",
               w->file_offset, w->file_offset + num_write - 1, num_write);
#else
    num_write = file_read(w->f, w->file_offset, (void *)w->data_out, SPI_FIFO_SIZE);
    if (num_write <= 0) {
        PRINT_INFO("EOF Reached. Firmware data uploaded.");
        return E_FINISHED;
    }
#endif

    w->file_offset += SPI_FIFO_SIZE;		///FIXME
    w->payload_num++;

    // Size is first long word of buffer read
    payload_size = ((uint32_t*)w->data_out)[0];
    size = payload_size;
    PRINT_INFO("Payload %d = %d Bytes (%d inc 'size' field)", w->payload_num, size, size + 4);

    // If this is first segment, then pad word is boot cfg
    if (w->payload_num == 1) {
        ((uint32_t*)w->data_out)[1] = set_bcr_word();

        // Record the start time of the first payload to measure the total
        // time the firmware upload takes to complete.
        w->firmware_upload_time = get_time_in_ms();
    }

    // Record the start of the transfer of this payload
    w->payload_upload_time = get_time_in_ms();

    // Write the size out to chip
    ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
    if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
        PRINT_NOTICE("Existing firmware already loaded...");
        return E_ALREADY;
    } else if (ret) {
        PRINT_ERR("sb_write() failed");
        return E_FAILURE;
    }

    w->total_bytes_written += num_write;
    size -= (SPI_FIFO_SIZE - sizeof(size));

    // Fatal if read_size not /8
    if (size % SPI_FIFO_SIZE) {
        PRINT_ERR("Size not multiple of %d", SPI_FIFO_SIZE);
        return E_BADSIZE;
    }

    // Get payload and write it out in SNT_FWDL_BUF_SIZE chunks
    payload_write = 0;
    while (size != 0 && payload_write < MAX_PAYLOAD_BYTES) {
        int read_size = min((unsigned int)SNT_FWDL_BUF_SIZE, size);

#ifdef CONFIG_HTC_FEATURE
        w->snt8100fsr->chip_update_size_remain = w->fw->size - w->file_offset;
        num_write = min(read_size, w->snt8100fsr->chip_update_size_remain);
        PRINT_DEBUG("file_offset = %d, remain:%d, num_write:%d",
                    w->file_offset, w->snt8100fsr->chip_update_size_remain, num_write);
        if (num_write > 0) {
            ret = secure_memcpy(w->data_out, SNT_FWDL_BUF_SIZE,
                                w->fw->data + w->file_offset, w->snt8100fsr->chip_update_size_remain,
                                num_write);
            if (ret < 0) {
                PRINT_ERR("Fail to copy firmware: offset:%d, remain:%d, read_size:%d",
                          w->file_offset, w->snt8100fsr->chip_update_size_remain, read_size);
                return E_BADREAD;
            }
        } else {
            PRINT_DEBUG("EOF Reached. Stopping...");
            return E_BADREAD;
        }
        PRINT_DEBUG("Write data sector from byte %d to %d (Target %d, Total %d bytes)",
                   w->file_offset, w->file_offset + num_write - 1, read_size, num_write);
#else
        num_write = file_read(w->f, w->file_offset, (void*)w->data_out, read_size);
        if (num_write <= 0) {
            PRINT_DEBUG("EOF Reached. Stopping...");
            return E_BADREAD;
        }
#endif

        w->file_offset += read_size;

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
        memset(w->data_in, 0x00, SNT_FWDL_BUF_RX_SIZE);
#else
        memset(w->data_in, 0x00, SNT_FWDL_BUF_SIZE);
#endif
#endif
        /* Write the data to the bus */
        ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
        if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
            PRINT_NOTICE("Existing firmware already loaded...");
            return E_ALREADY;
        } else if (ret) {
            PRINT_ERR("sb_write() failed");
            return E_BADWRITE;
        }

        w->total_bytes_written += num_write;
        size -= num_write;
        payload_write += num_write;
#ifdef CONFIG_HTC_FEATURE
        PRINT_DEBUG("size = %d, payload_write:%d",
                    size, payload_write);
#endif
    }

    // Calculate how long this total payload took
    payload_duration = get_time_in_ms() - w->payload_upload_time;
    if (payload_duration == 0)
        payload_duration = 1;

    PRINT_DEBUG("Payload %d took %dms at %d kbit/s",
                w->payload_num,
                payload_duration,
                ((payload_size * 8 / payload_duration) * 1000) / 1024);

    if (w->payload_num >= MAX_PAYLOAD_NUM) {
        PRINT_DEBUG("Max Payload Reached. Stopping...");
        return E_TOOMANY;
    }

    if (FIRMWARE_UPLOAD_WITH_IRQ) {
        PRINT_DEBUG("Waiting for IRQ for next payload");
    } else {
        PRINT_DEBUG("workqueue_queue_work()");
        workqueue_queue_work(w, FIRMWARE_UPLOAD_DELAY_MS);
    }
    return E_SUCCESS;
}

static void firmware_cleanup(struct upload_work *w, error_t err) {
    PRINT_FUNC();
    if(err <= E_SUCCESS) {
        int duration;
        duration = get_time_in_ms() - w->firmware_upload_time;
        if (duration == 0)
            duration = 1;

        PRINT_DEBUG("Total %d bytes took %dms at %d kbit/s",
                    w->total_bytes_written,
                    duration,
                    ((w->total_bytes_written * 8 / duration) * 1000) / 1024);
    }

    memory_free(work->data_in);
    memory_free(work->data_out);

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
    snt_nanohub_bypass_ctrl(w->snt8100fsr, FW_STREAM_UPLOAD | FW_REG_LOG_OP_STOP);
#endif
#endif

    if(FIRMWARE_UPLOAD_WITH_IRQ) {
        // Cancel our irq timeout work queue item
#ifdef CONFIG_HTC_FEATURE
        workqueue_cancel_work(work);
#else
        workqueue_cancel_work(w);
#endif
    }

#ifdef CONFIG_HTC_FEATURE
    if (w->fw)
        release_firmware(w->fw);

    workqueue_free_work(w);
//    workqueue_free_work(work);//////////////////////////////////
#else
    if (w->f) {
        filp_close(w->f, NULL);
        w->f = NULL;
    }

    workqueue_free_work(w);
#endif
    PRINT_DEBUG("Finished");
    return;
}

static void upload_firmware_internal(struct upload_work *w) {
    /*
     * insmod driver causes firmware to be uploaded to chip
     */
#ifdef CONFIG_HTC_FEATURE
#if defined(USE_SPI_BUS)
    int  orig_spi_freq_khz = 0;
#endif
#endif
    int         ret;
    error_t err = E_SUCCESS;
    PRINT_FUNC("0x%p", w);

    // If we haven't opened the firmware file, do so
    if (w->f == 0) {
        ret = open_firmware_file(w);
        if (ret) {
            err = E_BADOPEN;
            goto cleanup;
        }
        else {
            // [dy] unique to upload_firmware
            // Register our interrupt handler
            if(FIRMWARE_UPLOAD_WITH_IRQ) {
#ifdef CONFIG_HTC_FEATURE
                irq_handler_register(snt8100fsr_g, irq_handler_top, NULL);
#else
                irq_handler_register(irq_handler_top, NULL);
#endif
            }
        }
    }

    err = firmware_waiting_irq(w);
    if (err >= E_FAILURE) {
        PRINT_CRIT("firmware_waiting_irq err %d", (int)err);
        goto cleanup;
    }

#ifdef CONFIG_HTC_FEATURE
#if defined(USE_SPI_BUS)
    orig_spi_freq_khz = w->snt8100fsr->spi_freq_khz;//Set SPI speed to firmware flash mode
    w->snt8100fsr->spi_freq_khz = SPI_FLASH_SPEED_KHZ;
#endif
#endif
    err = firmware_transfer_payloads(w);
#ifdef CONFIG_HTC_FEATURE
#if defined(USE_SPI_BUS)
    w->snt8100fsr->spi_freq_khz = orig_spi_freq_khz;//Set SPI speed to operation mode
#endif
#endif
    if (err >= E_FAILURE) {
#ifdef CONFIG_HTC_FEATURE
        snt8100fsr_g->chip_update_size_remain = -1;
#endif
        PRINT_CRIT("firmware_transfer_payloads err %d", (int)err);
    }
    else if (err == E_FINISHED) { goto cleanup; }
    return;
cleanup:
#ifdef CONFIG_HTC_FEATURE
    if (err >= E_FAILURE)
        snt8100fsr_g->chip_update_size_remain = -1;
#endif
    firmware_cleanup( w, err);
    if(FIRMWARE_UPLOAD_WITH_IRQ) {
#ifdef CONFIG_HTC_FEATURE
        irq_handler_unregister(snt8100fsr_g); //[dy] unique to upload_firmware
#else
        irq_handler_unregister(); //[dy] unique to upload_firmware
#endif
    }
	start_event_processing(snt8100fsr_g);
    return;
}

static void download_firmware_internal(struct upload_work *w) {
    /*
     * Chip reset sets event register bit FWD so driver must
     * download chip firmware.
     * IRQ already set up by start_event_processing
     */
#ifdef CONFIG_HTC_FEATURE
#if defined(USE_SPI_BUS)
    int  orig_spi_freq_khz = 0;
#endif
#endif
    error_t err = E_SUCCESS;
    int         ret;
    PRINT_FUNC("0x%p", w);

    // If we haven't opened the firmware file, do so
    if (w->f == 0) {
        ret = open_firmware_file(w);
        if(ret) {
            err = E_BADOPEN;
            goto cleanup;
        }
    }

    err = firmware_waiting_irq(w);
    if (err >= E_FAILURE) {
        PRINT_CRIT("firmware_waiting_irq err %d", (int)err);
        goto cleanup;
    }
#ifdef CONFIG_HTC_FEATURE
#if defined(USE_SPI_BUS)
    orig_spi_freq_khz = w->snt8100fsr->spi_freq_khz;//Set SPI speed to firmware flash mode
    w->snt8100fsr->spi_freq_khz = SPI_FLASH_SPEED_KHZ;
#endif
#endif
    err = firmware_transfer_payloads(w);
#ifdef CONFIG_HTC_FEATURE
#if defined(USE_SPI_BUS)
    w->snt8100fsr->spi_freq_khz = orig_spi_freq_khz;//Set SPI speed to operation mode
#endif
#endif
    if (err >= E_FAILURE) {
#ifdef CONFIG_HTC_FEATURE
        snt8100fsr_g->chip_update_size_remain = -1;
#endif
        PRINT_CRIT("firmware_transfer_payloads err %d", (int)err);
    }
    else if (err == E_FINISHED) { goto cleanup; }
    return;
cleanup:
#ifdef CONFIG_HTC_FEATURE
    if (err >= E_FAILURE)
        snt8100fsr_g->chip_update_size_remain = -1;

    set_context_fwd_done(w->snt8100fsr);
#endif
    firmware_cleanup( w, err);
    // [dy] 2017-09-01 fwd done, set context_fwd to false
    // [dy] 2017-09-05 initializations after reset
#if !defined(CONFIG_HTC_FEATURE)
    set_context_fwd_done(w->snt8100fsr);
#endif
    return;
}
