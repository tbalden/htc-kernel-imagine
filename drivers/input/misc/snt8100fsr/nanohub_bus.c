/*****************************************************************************
* File: nanohub_bus.c
*
* (c) 2017 HTC Corporation - All Rights Reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/reboot.h>

#include "config.h"
#include "debug.h"
#include "serial_bus.h"
#include "device.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "memory.h"
#include "utils.h"
#include "main.h"
#include "irq.h"
#include "nanohub_bus.h"
#include <linux/nanohub_htc.h>

#ifdef CONFIG_HTC_FEATURE
#ifdef USE_NANOHUB_BUS
/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define DATA_SIZE            (17 * 1024)
#define TRANSMISSION_TIMEOUT 1000
#define POWER_OFF_FIXED_RATE  (1)
#define POWER_OFF_AUTO_CHANGE (2)
#define POWER_OFF_TRANS_TIMEOUT 300
#define DISABLE_KEY_MASK (BIT(1) | BIT(2) | BIT(3))


/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t *request_out;
static uint8_t  *status_in;
static uint8_t  *data_in;
static uint8_t  *data_out;
static uint8_t  *edge_in_buf;
static uint8_t  edge_in_count = 0;
static struct   nanohub_bypass_notifier *bypass_result_notifier = NULL;
enum {
    EDGE_DATA_WRITE,
    EDGE_DATA_READ,
    EDGE_STATE_CTRL,
    EDGE_MAX_OP_EVENT,
};

///DEBUG
int SNT_FWDL_BUF_SIZE = 100;
module_param(SNT_FWDL_BUF_SIZE, int, S_IRUSR | S_IWUSR);

//Power off sequence
static unsigned int power_off_setting = POWER_OFF_AUTO_CHANGE;
static bool power_off_disable_edge = false;
static unsigned int power_off_slg_retry_cnt = 2;
static unsigned int power_off_freq_l = 2;
static unsigned int power_off_freq_h = 50;
static unsigned int power_off_InactivityTimeout = 2000;
static bool power_off_WakeOnTouch = false;
module_param(power_off_setting, uint, S_IRUSR | S_IWUSR);
module_param(power_off_disable_edge, bool, S_IRUSR | S_IWUSR);
module_param(power_off_slg_retry_cnt, uint, S_IRUSR | S_IWUSR);
module_param(power_off_freq_l, uint, S_IRUSR | S_IWUSR);
module_param(power_off_freq_h, uint, S_IRUSR | S_IWUSR);
module_param(power_off_InactivityTimeout, uint, S_IRUSR | S_IWUSR);
module_param(power_off_WakeOnTouch, bool, S_IRUSR | S_IWUSR);

/*==========================================================================*/
/* PROTOTYPE                                                                */
/*==========================================================================*/
extern void nanohub_edge_bypass_to_app(uint8_t *buf, uint8_t len);
extern void nanohub_edge_notifier_register(struct nanohub_bypass_notifier *notifier);

static int snt8100fsr_open(struct inode *, struct file *);
static ssize_t snt8100fsr_read(struct file *, char *, size_t, loff_t *);
static unsigned int snt8100fsr_poll(struct file *, poll_table *);
static int snt8100fsr_release(struct inode *, struct file *);


/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/

static int power_off_notifier_handler(struct notifier_block *this, unsigned long code, void *data)
{
    uint16_t reg_buf[3] = {power_off_freq_h,
                           power_off_freq_l,
                           ((power_off_WakeOnTouch << 15) | power_off_InactivityTimeout)};
    uint16_t reg_data = 0;
    int ret = 0;
    char drop_buf_flag = '1';
    PRINT_INFO("%s: code: %lu\n", __func__, code);

    nanohub_set_drop_buf_flag(&drop_buf_flag, sizeof(char));

    if(!snt8100fsr_g) {
        PRINT_ERR("Not support snt edge");
        return NOTIFY_DONE;
    }

    switch (power_off_setting) {
        case POWER_OFF_FIXED_RATE:
            snt8100fsr_g->frame_rate = power_off_freq_l;
            PRINT_INFO("Setting frame rate to: %dHz", snt8100fsr_g->frame_rate);
            if (write_register(snt8100fsr_g,
                               REGISTER_FRAME_RATE,
                               &snt8100fsr_g->frame_rate)) {
                PRINT_CRIT("Unable to set frame rate");
            }
            break;
        case POWER_OFF_AUTO_CHANGE:
            //switch to i2c
            snt8100fsr_g->bus_type = BUS_TYPE_NANOHUB;
            if (snt_nanohub_bypass_ctrl(snt8100fsr_g, (SPI_MODE_SET | BUS_TYPE_NANOHUB))) {
                PRINT_CRIT("Unable to switch to i2c mode");
            }

            nanohub_edge_dynamic_power_ctrl(false);

            //Set retry count (009902)
            if (snt_nanohub_bypass_cmd_timeout(snt8100fsr_g, (SLG_RETRY_CNT | power_off_slg_retry_cnt),
                                               POWER_OFF_TRANS_TIMEOUT)) {
                PRINT_CRIT("Unable to set SLG retry count");
            } else {
                PRINT_INFO("Set SLG retry count to %u", power_off_slg_retry_cnt);
            }

            //Enable key: write { 0 } to address[0x3E].bit[3~1]
            if (snt_nanohub_bypass_cmd_timeout(snt8100fsr_g, ACTIVITY_REQUEST,
                                               POWER_OFF_TRANS_TIMEOUT)) {
                PRINT_CRIT("Unable to wake up device");
            }
            ret = read_register(snt8100fsr_g,
                    REGISTER_TCH_FIL_BYPASS,
                    &reg_data);
            if (ret) {
                PRINT_CRIT("Unable to read key setting");
            } else if (!(reg_data & DISABLE_KEY_MASK)) {
                PRINT_INFO("Keys are already enabled (0x%x)", reg_data);
            } else {
                reg_data &= ~DISABLE_KEY_MASK;
                ret = write_register(snt8100fsr_g,
                        REGISTER_TCH_FIL_BYPASS,
                        &reg_data);
                if (ret) {
                    PRINT_CRIT("Unable to enable key setting");
                } else {
                    PRINT_INFO("Set addr[0x3E] to [0x%x]", reg_data);
                }
            }

            //Disable edge: write {0xF8} to address[0x2B]
            if (power_off_disable_edge) {
                snt8100fsr_g->set_sys_param_id  = 100;
                snt8100fsr_g->set_sys_param_val = 0;
                snt8100fsr_g->set_sys_param_status = -1;
                enable_set_sys_param(snt8100fsr_g);

                reg_data = 0xF8;
                if (snt_nanohub_bypass_cmd_timeout(snt8100fsr_g, ACTIVITY_REQUEST,
                                                   POWER_OFF_TRANS_TIMEOUT)) {
                    PRINT_CRIT("Unable to wake up device");
                }
                if (write_register(snt8100fsr_g,
                                  REGISTER_BAR_CTRL,
                                  &reg_data)) {
                    PRINT_CRIT("Unable to disable edge");
                } else {
                    PRINT_INFO("Set addr[0x2B] to [0x%x]", reg_data);
                }
            }

            //Low power setting
            PRINT_INFO("Off mode setting:%dHz, auto-%s(%dHz, timeout:%lums, 0x%x)",
                    reg_buf[0],
                    (reg_buf[2] & BIT(15)) ? "en" : "dis",
                    reg_buf[1],
                    (reg_buf[2] & ~BIT(15)),
                    reg_buf[2]);
            if (snt_nanohub_bypass_cmd_timeout(snt8100fsr_g, ACTIVITY_REQUEST,
                                               POWER_OFF_TRANS_TIMEOUT)) {
                PRINT_CRIT("Unable to wake up device");
            }
            if (write_register(snt8100fsr_g,
                              REGISTER_DYN_HI_RATE,
                              &reg_buf[0]) ||
                write_register(snt8100fsr_g,
                              REGISTER_DYN_LO_RATE,
                              &reg_buf[1]) ||
                write_register(snt8100fsr_g,
                              REFISTER_PWR_CTRL,
                              &reg_buf[2])) {
                PRINT_CRIT("Unable to set frame rate");
            }
            //Clear pending events after setting frame rate
            //    write 0x1a to address[0x05]  -> flush TFifo/CFifo/SFifo
            //    read address[0x02]           -> read to clear Host bit if any
            reg_data = 0x1A;
            if (write_register(snt8100fsr_g,
                    REGISTER_ACTIONS,
                    &reg_data)) {
                PRINT_CRIT("Failed to flush");
            }
            if (read_register(snt8100fsr_g,
                    REGISTER_FRAME_RATE,
                    &reg_data)) {
                PRINT_CRIT("Failed to clear host bit");
            }
            break;
        default:
            //Do nothing
            break;
    }

    return NOTIFY_DONE;
}

static struct notifier_block power_off_notifier = {
    .notifier_call = power_off_notifier_handler,
};

static const struct file_operations snt8100fsr_fops = {
    .owner = THIS_MODULE,
    .open = snt8100fsr_open,
    .read = snt8100fsr_read,
    .poll = snt8100fsr_poll,
    .release = snt8100fsr_release,
};

static int snt8100fsr_open(struct inode *inode, struct file *file)
{
    struct snt8100fsr *snt8100fsr = snt8100fsr_g;

    if(snt8100fsr) {
        file->private_data = snt8100fsr;
        nonseekable_open(inode, file);
        return 0;
    }

    return -ENODEV;
}

static ssize_t snt8100fsr_read(struct file *file, char *buffer, size_t length,
                            loff_t *offset)
{
    struct snt8100fsr *snt8100fsr = file->private_data;
    struct sc_command *sc_buf;
    int ret = 0;
    uint16_t buf_ready_count = 0;

    if(!snt8100fsr->sc_cmd_buf)
        return -ENOMEM;

    while (!snt8100fsr->sc_cmd_buf_ready) {
        buf_ready_count++;
        udelay(100);
        if(buf_ready_count >= 50000)
            return -EFAULT;
    }

    mutex_lock(&snt8100fsr->sc_cmd_read_lock);
    sc_buf = &snt8100fsr->sc_cmd_buf[0];
    ret = copy_to_user(buffer, (unsigned char*)&sc_buf->data[0],
                   sc_buf->length);
    snt8100fsr->sc_cmd_buf_ready = false;
    mutex_unlock(&snt8100fsr->sc_cmd_read_lock);

    if (ret != 0)
        ret = -EFAULT;
    else
        ret = sc_buf->length;

    return ret;
}

static unsigned int snt8100fsr_poll(struct file *file, poll_table *wait)
{
    struct snt8100fsr *snt8100fsr = file->private_data;
    unsigned int mask = POLLOUT | POLLWRNORM;

    if (snt8100fsr->sc_cmd_buf_ready)
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

static int snt8100fsr_release(struct inode *inode, struct file *file)
{
    struct snt8100fsr *snt8100fsr = file->private_data;

    if(snt8100fsr->sc_cmd_buf) {
        memory_free(snt8100fsr->sc_cmd_buf);
        snt8100fsr->sc_cmd_buf = NULL;
    }
    file->private_data = NULL;

    return 0;
}

int snt_nanohub_open(struct platform_device *pdev) {
    // Allocate our data buffers in contiguous memory for DMA support
    status_in = memory_allocate(SPI_FIFO_SIZE, GFP_DMA);
    if (status_in == NULL) {
        PRINT_CRIT("status_in = memory_allocate(%d) failed", SPI_FIFO_SIZE);
        return -1;
    }

    request_out = memory_allocate(SPI_FIFO_SIZE, GFP_DMA);
    if (request_out == NULL) {
        PRINT_CRIT("request_out = memory_allocate(%d) failed", SPI_FIFO_SIZE);
        return -1;
    }

    data_in = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    data_out = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    PRINT_INFO("Registered new platform device");
    return 0;
}

void snt_nanohub_close(void) {
    PRINT_FUNC();

    if (request_out) {
        memory_free(request_out);
        request_out = NULL;
    }

    if (status_in) {
        memory_free(status_in);
        status_in = NULL;
    }

    if (data_in) {
        memory_free(data_in);
        data_in = NULL;
    }

    if (data_out) {
        memory_free(data_out);
        data_out = NULL;
    }

    PRINT_INFO("Unregistering platform device");
    PRINT_DEBUG("done");
}

/*
 * Tranismission sync between ACPU & nanohub
 */
static void nanohub_edge_set_transmission_state(int state)
{
    atomic_set(&snt8100fsr_g->transmission_state, state);
    if (state > 0)
        wake_up(&snt8100fsr_g->transmission_wait);
}

static int nanohub_edge_get_transmission_state(void)
{
    return atomic_read(&snt8100fsr_g->transmission_state);
}

/*
 * 
 */
static int nanohub_edge_tx_rx(uint8_t cmdType,
        const uint8_t *out_buf, int out_count,
        uint8_t *in_buf, int in_count)
{
    uint8_t *buf;
    uint8_t tx_len = 0;
    int ret = 0;
#ifdef DEBUG_BYPASS_MODE
    int i;
#endif

    if (out_count <= 0 || out_count > (SNT_FWDL_BUF_SIZE + 2)
            || in_count < 0 || in_count > SNT_FWDL_BUF_RX_SIZE) {
        PRINT_ERR("invalid tx_len o:%d, i:%d", out_count, in_count);
        ret = -EINVAL;
    } else if ((out_buf == NULL) && (in_buf == NULL)) {
        PRINT_ERR("buf is NULL!");
        ret = -EFAULT;
    } else {
        tx_len = sizeof(cmdType)        /* field length of cmdType */
                + sizeof(uint8_t) * 2   /* field length of out_count & in_count */
                + out_count;            /* data length */
        buf = vmalloc(tx_len);

        if (buf) {
            nanohub_edge_set_transmission_state(0);

            buf[0] = cmdType;
            buf[1] = out_count;
            buf[2] = in_count;
            if (out_count > 0 && out_buf != NULL)
                memcpy(buf + 3, out_buf, out_count);
            if (in_count > 0 && in_buf != NULL) {
                edge_in_buf = in_buf;
                edge_in_count = in_count;
            }

            nanohub_edge_bypass_to_app(buf, tx_len);

#ifdef DEBUG_BYPASS_MODE
            printk(KERN_CONT "[EDGE] >hub len:%d, 0x",tx_len);
            for (i = 0 ; i < tx_len ; i++)
                printk(KERN_CONT "%02X ", buf[i]);
            printk(KERN_CONT "\n");
#endif

            vfree(buf);
        } else {
            PRINT_ERR("buf malloc failed");
            ret = -ENOMEM;
        }
    }

    return ret;
}

static void nanohub_edge_tx_rx_cleanup(void) {
    PRINT_INFO("%s", __func__);
    mutex_lock(&snt8100fsr_g->bypass_read_lock);
    edge_in_count = 0;
    edge_in_buf = NULL;
    mutex_unlock(&snt8100fsr_g->bypass_read_lock);
}

static void nanohub_edge_callback(uint8_t* buf, uint8_t size) {
    uint8_t cmdType;
    uint8_t in_count = 0, temp;
#ifdef DEBUG_BYPASS_MODE
    int i;
#endif

    if (buf == NULL || size == 0) {
        PRINT_ERR("%s", buf == NULL ? "buf is NULL" : "size is 0");
        return;
    }

#ifdef DEBUG_BYPASS_MODE
    printk(KERN_CONT "[EDGE] >cpu len:%d, 0x",size);
    for (i = 0; i < size; i++)
        printk(KERN_CONT "%02X ", buf[i]);
    printk(KERN_CONT "\n");
#endif

    cmdType = buf[0];

    if (cmdType == EDGE_DATA_READ) {
        mutex_lock(&snt8100fsr_g->bypass_read_lock);

        if ((size > (3 * sizeof(uint8_t)))
                    && (edge_in_count > 0)
                    && edge_in_buf) {

            temp = size - 3 * sizeof(uint8_t);
            temp = min(temp, buf[2]);
            in_count = min(temp, edge_in_count);
            memcpy(edge_in_buf, buf + 3, in_count);
        } else
            PRINT_ERR("invalid length(%d,%d,%d) %s",
                            edge_in_count, size, buf[2],
                            edge_in_buf ? "" : "or edge_in_buf=NULL");

        edge_in_count   = 0;
        edge_in_buf     = NULL;
        mutex_unlock(&snt8100fsr_g->bypass_read_lock);

        PRINT_DEBUG("EDGE_DATA_READ len=%d", in_count);
        nanohub_edge_set_transmission_state(in_count);
    } else if (cmdType == EDGE_DATA_WRITE) {
        PRINT_DEBUG("EDGE_DATA_WRITE len=%d", buf[1]);
        nanohub_edge_set_transmission_state(buf[1]);
    } else if (cmdType == EDGE_STATE_CTRL) {
        PRINT_DEBUG("EDGE_STATE_CTRL state:0x%02X", buf[3]);
        nanohub_edge_set_transmission_state(buf[3]);
    } else
        PRINT_ERR("wrong cmdType: 0x%02X", cmdType);
}

/*
 * snt_nanohub_get_boot_status() can only be called during firmware loading
 * prior to the last interrupt saying the system is ready.
 */
uint8_t snt_nanohub_get_boot_status(struct snt8100fsr *snt8100fsr) {
    PRINT_CRIT("Not Implemented");
    return 0xFF;
}


/*
 * Wake the device up over the I2C bus by issuing a write of 4 bytes
 * to the I2C Wake Device which operates on a different I2C address.
 */
int snt_nanohub_wake_device(struct snt8100fsr *snt8100fsr) {
    int count;
    uint8_t state = ACTIVITY_REQUEST;

    mutex_lock(&snt8100fsr_g->bypass_write_lock);
    count = nanohub_edge_tx_rx(EDGE_STATE_CTRL, &state, 1, NULL, 0);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C write failed, error = %d", count);
        return -1;
    }

    count = wait_event_interruptible_timeout(
            snt8100fsr_g->transmission_wait,
            nanohub_edge_get_transmission_state(),
            msecs_to_jiffies(TRANSMISSION_TIMEOUT));
    if (count <= 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("wait failed, %d", count);
        return -1;
    }

    count = nanohub_edge_get_transmission_state();
    if (count != state) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("ctrl state mismatch: 0x%02X, %02X",
                   state, count);
        return -1;
    }

    mutex_unlock(&snt8100fsr_g->bypass_write_lock);
    return 0;
}

int snt_nanohub_bypass_cmd_timeout(struct snt8100fsr *snt8100fsr, uint8_t state, uint16_t timeout_ms) {
    int count;

    mutex_lock(&snt8100fsr_g->bypass_write_lock);
    count = nanohub_edge_tx_rx(EDGE_STATE_CTRL, &state, 1, NULL, 0);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C write failed, error = %d", count);
        return -1;
    }

    count = wait_event_interruptible_timeout(
            snt8100fsr_g->transmission_wait,
            nanohub_edge_get_transmission_state(),
            msecs_to_jiffies(timeout_ms));
    if (count <= 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("wait failed, %d", count);
        return -1;
    }

    count = nanohub_edge_get_transmission_state();
    if (count != state) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("ctrl state mismatch: 0x%02X, %02X",
                   state, count);
        return -1;
    }

    mutex_unlock(&snt8100fsr_g->bypass_write_lock);
    return 0;
}

int snt_nanohub_bypass_NonSCCmd_ctrl(struct snt8100fsr *snt8100fsr, uint8_t state) {
    int count;

    mutex_lock(&snt8100fsr_g->bypass_write_lock);
    count = nanohub_edge_tx_rx(EDGE_STATE_CTRL, &state, 1, NULL, 0);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C write failed, error = %d", count);
        return -1;
    }

    count = wait_event_interruptible_timeout(
            snt8100fsr_g->transmission_wait,
            nanohub_edge_get_transmission_state(),
            msecs_to_jiffies(TRANSMISSION_TIMEOUT));
    if (count <= 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("wait failed, %d", count);
        return -1;
    }

    count = nanohub_edge_get_transmission_state();
    if (count != state) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("ctrl state mismatch: 0x%02X, %02X",
                   state, count);
        return -1;
    }

    mutex_unlock(&snt8100fsr_g->bypass_write_lock);
    return 0;
}

int snt_nanohub_bypass_ctrl(struct snt8100fsr *snt8100fsr, uint8_t state) {
    int count = -1;
    if ((snt8100fsr->bus_type == BUS_TYPE_NANOHUB)
                    || (state == (SPI_MODE_SET | BUS_TYPE_SPI))
                    || (state == (SPI_MODE_SET | BUS_TYPE_I2C))
                    || (state == (SPI_MODE_SET | BUS_TYPE_NANOHUB))) {
        PRINT_INFO("ctrl state: %02X", state);
    } else {
        return 0;
    }

    if ((((state & 0x01) == FW_REG_LOG_OP_STOP) || (state == (SPI_MODE_SET | BUS_TYPE_NANOHUB)))
                    && (snt8100fsr->active_sc_cmd == mc_no_command)
                    && (state != (SPI_MODE_SET | BUS_TYPE_SPI))
                    && (state != (SPI_MODE_SET | BUS_TYPE_I2C))) {
        snt_irq_disable(snt8100fsr);
    }

    mutex_lock(&snt8100fsr_g->bypass_write_lock);
    count = nanohub_edge_tx_rx(EDGE_STATE_CTRL, &state, 1, NULL, 0);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C write failed, error = %d", count);
        return count;
    }

    count = wait_event_interruptible_timeout(
            snt8100fsr_g->transmission_wait,
            nanohub_edge_get_transmission_state(),
            msecs_to_jiffies(TRANSMISSION_TIMEOUT));
    if (count <= 0) {
        PRINT_CRIT("wait failed, %d", count);
        count = -1;
        goto bypass_ctrl_end;
    }

    count = nanohub_edge_get_transmission_state();
    if (count != state) {
        PRINT_CRIT("ctrl state mismatch: 0x%02X, %02X",
                   state, count);
        count = -1;
        goto bypass_ctrl_end;
    }

    count = 0;
    snt8100fsr->bypass_ctrl_state = state;

    if ((((state & 0x01) == FW_REG_LOG_OP_START) && (state != (SPI_MODE_SET | BUS_TYPE_NANOHUB)))
                    || (state == (SPI_MODE_SET | BUS_TYPE_SPI))
                    || (state == (SPI_MODE_SET | BUS_TYPE_I2C))) {
        snt_irq_enable(snt8100fsr);
    }

bypass_ctrl_end:
    if (count != 0 && ((state & 0x01) == FW_REG_LOG_OP_STOP)) {
        snt_irq_enable(snt8100fsr);
    }
    mutex_unlock(&snt8100fsr_g->bypass_write_lock);
    PRINT_DEBUG("done");
    return count;
}

int snt_nanohub_read(struct snt8100fsr *snt8100fsr,
                 int num_read,
                 uint8_t *data_in) {

    int count;
    PRINT_FUNC("%d bytes", num_read);

    mutex_lock(&snt8100fsr_g->bypass_write_lock);
    count = nanohub_edge_tx_rx(EDGE_DATA_READ, NULL, 0, data_in, num_read);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C header read failed len = %d", count);
        return count;
    }

    count = wait_event_interruptible_timeout(
            snt8100fsr_g->transmission_wait,
            nanohub_edge_get_transmission_state(),
            msecs_to_jiffies(TRANSMISSION_TIMEOUT));
    if (count <= 0) {
        nanohub_edge_tx_rx_cleanup();
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("wait failed, %d", count);
        return -1;
    }

    count = nanohub_edge_get_transmission_state();
    if (count != num_read) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C read of %d bytes only read %d bytes",
                   num_read, count);
        return -1;
    }
#if 0
    count = i2c_master_recv(snt8100fsr->i2c, data_in, num_read);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C write failed, error = %d", count);
        return -1;
    } else if (count != num_read) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C read of %d bytes only read %d bytes",
                   num_read, count);
        return -1;
    }
#endif
    mutex_unlock(&snt8100fsr_g->bypass_write_lock);
    PRINT_DEBUG("%d bytes read", num_read);
    return 0;
}

int snt_nanohub_write(struct snt8100fsr *snt8100fsr,
                  int num_write,
                  uint8_t *data_out) {
    int count = -1;
    PRINT_FUNC("%d bytes", num_write);

    while (num_write != 0) {
        uint16_t pkt_len = (num_write > SNT_FWDL_BUF_SIZE) ? SNT_FWDL_BUF_SIZE : num_write;
        num_write -= pkt_len;
        mutex_lock(&snt8100fsr_g->bypass_write_lock);
        count = nanohub_edge_tx_rx(EDGE_DATA_WRITE, data_out, pkt_len, NULL, 0);
        if (count < 0) {
            PRINT_CRIT("I2C write failed, error = %d", count);
            goto write_end;
        }

        count = wait_event_interruptible_timeout(
                snt8100fsr_g->transmission_wait,
                nanohub_edge_get_transmission_state(),
                msecs_to_jiffies(TRANSMISSION_TIMEOUT));
        if (count <= 0) {
            PRINT_CRIT("wait failed, %d", count);
            count = -1;///FIXME, Why sensorhub does not return wrtien number
            goto write_end;
        }

        count = nanohub_edge_get_transmission_state();
        if (count != pkt_len) {
            PRINT_CRIT("I2C write of %d bytes only wrote %d bytes",
                       pkt_len, count);
            count = -1;
            goto write_end;
        }
        count = 0;
write_end:
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        if (count < 0) {
            PRINT_CRIT("i2c pkt write failed at len %d", num_write);
            break;
        }
        data_out += pkt_len;
    }
#if 0
    count = i2c_master_send(snt8100fsr->i2c, data_out, num_write);
    if (count < 0) {
        PRINT_CRIT("I2C write failed, error = %d", count);
        return -1;
    } else if (count != num_write) {
        PRINT_CRIT("I2C write of %d bytes only wrote %d bytes",
                   num_write, count);
        return -1;
    }
#endif

    PRINT_DEBUG("done");
    return count;
}

int snt_nanohub_read_fifo_512(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val) {
    uint8_t addr_phase[2];
    int count;

    PRINT_FUNC("len %d", len);

    addr_phase[0] = (reg >= 0x100) ? (0x80 | (reg >> 8)) : reg;
#ifdef CONFIG_HTC_FEATURE
    if (len > SNT_FWDL_BUF_RX_SIZE) {
        PRINT_CRIT("Warning. Max len for I2C read is %d bytes, truncating.", SNT_FWDL_BUF_RX_SIZE);
        len = SNT_FWDL_BUF_RX_SIZE;
    }
#else
    if (len > 512) {
        PRINT_CRIT("Warning. Max len for I2C read is 512 bytes, truncating.");
        len = 512;
    }
#endif

    if (len == 0) {
        PRINT_CRIT("ERROR: Must read at least 1 word");
        return -1;
    }

    addr_phase[1] = (len / 2) - 1;
    mutex_lock(&snt8100fsr_g->bypass_write_lock);
    count = nanohub_edge_tx_rx(EDGE_DATA_READ, addr_phase, 2, in_val, len);
#if 0
    count = i2c_master_send(snt8100fsr->i2c, addr_phase, 2);
#endif
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C header read failed len = %d", count);
        return -1;
    }

    count = wait_event_interruptible_timeout(
            snt8100fsr_g->transmission_wait,
            nanohub_edge_get_transmission_state(),
            msecs_to_jiffies(TRANSMISSION_TIMEOUT));
    if (count <= 0) {
        nanohub_edge_tx_rx_cleanup();
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("wait failed, %d", count);
        return -1;
    }

    count = nanohub_edge_get_transmission_state();
    if (count != len) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C Read failed len=%d (expected %d)",
                   count, len);
        return -1;
    }
#if 0
    count = i2c_master_recv(snt8100fsr->i2c, in_val, len);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C read failed, error code = %d", count);
        return count;
    } else if (count != len) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C Read failed len=%d (expected %d)",
                   len, count);
        return -1;
    }
#endif

    mutex_unlock(&snt8100fsr_g->bypass_write_lock);
    PRINT_DEBUG("done");
    return 0;
}

int snt_nanohub_read_fifo(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val) {
    int ret = 0;
    uint8_t *offset = in_val;

   PRINT_FUNC("reg 0x%X", reg);

    while (len != 0 && ret == 0) {
#ifdef CONFIG_HTC_FEATURE
        uint16_t pkt_len = (len > SNT_FWDL_BUF_RX_SIZE) ? SNT_FWDL_BUF_RX_SIZE : len;
#else
        uint16_t pkt_len = (len > 512) ? 512 : len;
#endif
        len -= pkt_len;
        ret = snt_nanohub_read_fifo_512(snt8100fsr, reg, pkt_len, offset);
        if (ret != 0) {
            PRINT_CRIT("i2c pkt read failed at len %d, reg: 0x%04X", len, reg);
            break;
        }
        offset += pkt_len;
    }
    PRINT_DEBUG("done");
    return ret;
}


int snt_nanohub_write_fifo_512(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val) {
    int write_len;
    int count;

    PRINT_FUNC("len %d", len);

    data_out[0] = (reg >= 0x100) ? (0x80 | (reg >> 8)) : reg;
#ifdef CONFIG_HTC_FEATURE
    if (len > SNT_FWDL_BUF_SIZE) {
      PRINT_CRIT("Warning. Max len for I2C read is %d bytes, truncating.", SNT_FWDL_BUF_SIZE);
      len = SNT_FWDL_BUF_SIZE;
    }
#else
    if (len > 512) {
      PRINT_CRIT("Warning. Max len for I2C read is 512 bytes, truncating.");
      len = 512;
    }
#endif
    if (len == 0) {
      PRINT_CRIT("ERROR: Must write at least 1 word");
      return -1;
    }

    data_out[1] = (len / 2) - 1;
    memcpy(&data_out[2], out_val, len);
    write_len = len + 2;

    /* Debug output of the hex values of the data
    for(count = 0; count < write_len; count++) {
        PRINT_DEBUG("%02X (%d)", data_out[count], data_out[count]);
    }*/

    mutex_lock(&snt8100fsr_g->bypass_write_lock);
    count = nanohub_edge_tx_rx(EDGE_DATA_WRITE, data_out, write_len, NULL, 0);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C write failed, error code = %d", count);
        return count;
    }

    count = wait_event_interruptible_timeout(
            snt8100fsr_g->transmission_wait,
            nanohub_edge_get_transmission_state(),
            msecs_to_jiffies(TRANSMISSION_TIMEOUT));
    if (count <= 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("wait failed, %d", count);
        return -1;
    }

    count = nanohub_edge_get_transmission_state();
    if (count != write_len) {
      mutex_unlock(&snt8100fsr_g->bypass_write_lock);
      PRINT_CRIT("ERROR: I2C Write failed len=%d (expected %d)", count, write_len);
      return -1;
    }
#if 0
    count = i2c_master_send(snt8100fsr->i2c, data_out, write_len);
    if (count < 0) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("I2C read failed, error code = %d", count);
        return count;
    } else if (count != write_len) {
        mutex_unlock(&snt8100fsr_g->bypass_write_lock);
        PRINT_CRIT("ERROR: I2C Write failed len=%d (expected %d)", count, write_len);
        return -1;
    }
#endif

    mutex_unlock(&snt8100fsr_g->bypass_write_lock);
    PRINT_DEBUG("done");
    return 0;
}

int snt_nanohub_write_fifo(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val) {
    int ret = 0;
    uint8_t *offset = out_val;

    PRINT_FUNC("reg 0x%X", reg);

    while (len != 0 && ret == 0) {
#ifdef CONFIG_HTC_FEATURE
        uint16_t pkt_len = (len > SNT_FWDL_BUF_SIZE) ? SNT_FWDL_BUF_SIZE : len;
#else
        uint16_t pkt_len = (len > 512) ? 512 : len;
#endif
        len -= pkt_len;
        ret = snt_nanohub_write_fifo_512(snt8100fsr, reg, pkt_len, offset);
        if (ret != 0) {
            PRINT_CRIT("i2c pkt write failed at len %d, reg: 0x%04X", len, reg);
            break;
        }
        offset += pkt_len;
    }
    PRINT_DEBUG("done");
    return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id snt_nanohub_dt_id[] = {
    { .compatible = "sentons,snt8100fsr", },
    { },
};

MODULE_DEVICE_TABLE(of, snt_nanohub_dt_id);
#endif

static struct platform_device_id snt_nanohub_id[] = {
    { SENTONS_DRIVER_NAME, 0 },
    { },
};

/*==========================================================================*/
/* Device Probe/Remove/Resume/Suspend                                       */
/*==========================================================================*/
static int snt_nanohub_probe(struct platform_device *pdev)
{
    int ret;
    struct device *dev = &pdev->dev;
    struct snt8100fsr *snt8100fsr;

    PRINT_FUNC();

    snt8100fsr = memory_allocate(sizeof(*snt8100fsr),
                                 GFP_KERNEL);

    if(!snt8100fsr) {
        PRINT_CRIT("failed to allocate memory for struct snt8100fsr");
        return -ENOMEM;
    }

    memset(snt8100fsr, 0, sizeof(*snt8100fsr));

    snt8100fsr->bus_type = BUS_TYPE_NANOHUB;
    snt8100fsr->dev = dev;
    //i2c_set_clientdata(i2c, snt8100fsr);
    platform_set_drvdata(pdev, snt8100fsr);
    snt8100fsr->pdev = pdev;

    // Save this as our main device and to be used for sysFS access
    snt8100fsr_g = snt8100fsr;
#ifdef CONFIG_NANOHUB_SILEGO
    if(snt8100fsr_spi_g->spi)
        snt8100fsr_g->spi = snt8100fsr_spi_g->spi;
#endif

    ret = main_init();
    if (ret) {
        PRINT_CRIT("main_init() failed");
        return ret;
    }

    ret = snt_nanohub_device_init(pdev, snt8100fsr);
    if(ret) {
        PRINT_CRIT("snt_nanohub_device_init() failed");
        return ret;
    }

    snt_get_hw_id(pdev, snt8100fsr);
    printk(KERN_INFO "[EDGE] snt8100fsr flash_flag = %d\n", snt8100fsr->flash_flag);
    nanohub_edge_status(snt8100fsr->flash_flag);

    ret = snt_nanohub_open(snt8100fsr->pdev);
    if (ret) {
        PRINT_CRIT("snt_nanohub_open() failed");
        return ret;
    }

    /* to register as a misc device */
    snt8100fsr->miscdev.minor = MISC_DYNAMIC_MINOR;
    snt8100fsr->miscdev.name = SENTONS_DRIVER_NAME;
    snt8100fsr->miscdev.fops = &snt8100fsr_fops;
    PRINT_INFO("Misc device registration name:%s", SENTONS_DRIVER_NAME);
    if (misc_register(&snt8100fsr->miscdev) != 0)
        PRINT_CRIT("Could not register misc. dev for snt8100fsr");

    mutex_init(&snt8100fsr->sc_cmd_read_lock);
    wakeup_source_init(&snt8100fsr->edge_ws, "edge_ws");

    init_waitqueue_head(&snt8100fsr_g->transmission_wait);
    nanohub_edge_set_transmission_state(0);

    /*
     * Register nanohub bypass notifier for data transmission
     */
    bypass_result_notifier = memory_allocate(sizeof(*bypass_result_notifier),
                                 GFP_KERNEL);
    if (bypass_result_notifier == NULL)
        return -ENOMEM;

    bypass_result_notifier->appId = SNT_APP_ID;
    bypass_result_notifier->callback = nanohub_edge_callback;
    nanohub_edge_notifier_register(bypass_result_notifier);

    snt_irq_handler_register(snt8100fsr);

    // Start our sysfs interface
    snt_sysfs_init(snt8100fsr_g, true);

    register_reboot_notifier(&power_off_notifier);

    PRINT_DEBUG("done");
    return 0;
}

static int snt_nanohub_remove(struct platform_device *pdev)
{
    struct snt8100fsr *snt8100fsr = platform_get_drvdata(pdev);
    PRINT_FUNC();

    if (snt8100fsr) {
        stop_event_processing(snt8100fsr);
        if (snt8100fsr_g) {
            snt_sysfs_init(snt8100fsr_g, false);
        }
    }

    snt_nanohub_close();

    if (bypass_result_notifier) {
        memory_free(bypass_result_notifier);
    }

    if (snt8100fsr) {
        memory_free(snt8100fsr);
    }

    wakeup_source_trash(&snt8100fsr->edge_ws);

    main_exit();
    return 0;
}
#if 0
#ifdef CONFIG_PM_SLEEP
static int snt_nanohub_suspend(struct device *dev)
{
    int ret;
    PRINT_FUNC();
    ret = snt_suspend(dev);
    PRINT_DEBUG("done");
    return ret;
}

static int snt_nanohub_resume(struct device *dev)
{
    int ret;
    PRINT_FUNC();
    ret = snt_resume(dev);
    PRINT_DEBUG("done");
    return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(snt_nanohub_pm_ops, snt_nanohub_suspend, snt_nanohub_resume);
#endif

static struct platform_driver snt_nanohub_device_driver = {
    .probe        = snt_nanohub_probe,
    .remove        = snt_nanohub_remove,
    .id_table    = snt_nanohub_id,
    .driver        = {
        .name    = SENTONS_DRIVER_NAME,
        .owner    = THIS_MODULE,
#if 0
        .pm    = &snt_nanohub_pm_ops,
#endif
        .of_match_table = of_match_ptr(snt_nanohub_dt_id),
    },
};

static int __init snt_nanohub_init(void)
{
    return platform_driver_register(&snt_nanohub_device_driver);
}

static void __exit snt_nanohub_exit(void)
{
    platform_driver_unregister(&snt_nanohub_device_driver);
}

late_initcall(snt_nanohub_init);
module_exit(snt_nanohub_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-keys");
#endif    //#ifdef USE_NANOHUB_BUS
#endif    //#ifdef CONFIG_HTC_FEATURE
