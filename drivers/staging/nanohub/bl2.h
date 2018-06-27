/*
 * Copyright (C) 2018 STMicroelectonics, Inc.
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

#ifndef _NANOHUB_BL2_H_
#define _NANOHUB_BL2_H_

#include "bl.h"
#include "bl2msg.h"

struct nanohub_bl2 {
	uint32_t    mode;           /* current mode */
	uint32_t    seq;            /* current Frame/seq id */
	uint32_t    io_cur_len;     /* exchanged IO TxRx HW buffer size */
	uint32_t    io_max_len;     /* max IO TxRx HW buffer size in bytes */
	uint32_t    rx_offset;      /* initial IO RX HW offset */
	uint32_t    rx_size;        /* available IO Rx data */
	uint8_t     rx_packet[BL2_MSG_CMD_SIZE]; /* IO Rx packet */
	struct bl2_status_resp rstatus;
};

uint8_t nanohub_bl2_sync(struct nanohub_data *data, uint32_t mlen);

uint8_t nanohub_bl2_info(struct nanohub_data *data,
			 struct bl2_info_resp *rinfo);

uint8_t nanohub_bl2_flashinfo(struct nanohub_data *data,
			      uint32_t sector,
			      struct bl2_memop_resp *rmeminfo);

uint8_t nanohub_bl2_flashcrc32(struct nanohub_data *data,
			       uint32_t addr, uint32_t len,
			       uint32_t *crc, uint32_t *ms);

uint8_t nanohub_bl2_flasherase(struct nanohub_data *data,
			       uint32_t sector, uint32_t len,
			       uint32_t *ms);

uint8_t nanohub_bl2_flashprog(struct nanohub_data *data,
			      uint32_t addr, const uint8_t *image,
			      uint32_t len, uint32_t *ms);

#endif /* _NANOHUB_BL2_H_ */
