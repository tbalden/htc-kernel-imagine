/*****************************************************************************
* File: i2c-bus.h
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
#include <linux/kernel.h>

#ifndef NANOHUB_H
#define NANOHUB_H

//#define DEBUG_BYPASS_MODE

enum {
    FW_REG_LOG_OP_STOP = 0,
    FW_REG_LOG_OP_START= 1,
    FW_STREAM_UPLOAD   = 2,
    FW_FLASH_DOWNLOAD  = 4,
    FW_UPDATE_DOWNLOAD = 6,
    FW_REG_DOWNLOAD    = 8,
    SYS_PARAM_SET      = 10, //0x0A
    SYS_PARAM_GET      = 12, //0x0C
    LOG_TRACK_REPORT   = 14, //0x0E
    LOG_D1TEST         = 16, //0x10
    LOG_FRAMES         = 18, //0x12
    LOG_NO_TOUCH_FRAME = 20, //0x14
    SLG_RETRY_CNT      = 192,//0xC0 - 0xC3
    EVENT_REPORT       = 240,//0xF0
    ACTIVITY_REQUEST   = 242,//0xF2
    DYNAMIC_POWER_CTRL = 244,//0xF4
    SPI_MODE_SET       = 248,//0xF8
};

enum {
    SLG_SPI_MODE_CPU   = 0,
    SLG_EDGE_RESET     = 1,
    SLG_SPI_MODE_FLASH = 2,
};

uint8_t snt_nanohub_get_boot_status(struct snt8100fsr *snt8100fsr);
int snt_nanohub_wake_device(struct snt8100fsr *snt8100fsr);

int snt_nanohub_bypass_cmd_timeout(struct snt8100fsr *snt8100fsr, uint8_t state, uint16_t delay_ms);

int snt_nanohub_bypass_NonSCCmd_ctrl(struct snt8100fsr *snt8100fsr, uint8_t state);

int snt_nanohub_bypass_ctrl(struct snt8100fsr *snt8100fsr, uint8_t state);

int snt_nanohub_read(struct snt8100fsr *snt8100fsr,
                 int num_read,
                 uint8_t *data_in);

int snt_nanohub_write(struct snt8100fsr *snt8100fsr,
                  int num_write,
                  uint8_t *data_out);

int snt_nanohub_read_fifo(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val);

int snt_nanohub_write_fifo(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val);

#endif // NANOHUB_H

