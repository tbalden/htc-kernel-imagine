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

#include <linux/platform_data/nanohub.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>

#include "main.h"
#include "bl2.h"
#include "bl2msg.h"

#define BL2_SYNC_TIMEOUT    65536
#define BL2_MIN_DATA_SIZE   256

/* Maximum request number to capture a response
 * and maximum retry iteration to send a TX_BUFFER
 */
#define BL2_RESP_TIMEOUT	 (1024)
#define BL2_MAX_RETRY_TX_BUFFER  (10)

/* Approximate time to:
 *   erase a page (in ms)
 *   calculate CRC32 for 2048 bytes (us/2048)
 *   write 2048 bytes (us/2048)
 */
#define BL2_TIME_ERASE_PAGE_MS (22)
#define BL2_TIME_CRC32_2048_US (140)
#define BL2_TIME_PROG_2048_MS  (20)

/*
 * Internal helper functions
 */

#define _BL2_ERR_CASE(_err) case BL2_ERROR_ ## _err: str = #_err; break;

static const char *nanohub_bl2_err_to_str(uint32_t err)
{
	char *str;

	err = BL2_GET_ERROR(err);

	switch (err) {
	_BL2_ERR_CASE(PARAM);
	_BL2_ERR_CASE(REASON);
	_BL2_ERR_CASE(FRAME);
	_BL2_ERR_CASE(SYNC);
	_BL2_ERR_CASE(GENERIC);
	_BL2_ERR_CASE(NOT_SUPPORTED);
	_BL2_ERR_CASE(PADDRESS);
	_BL2_ERR_CASE(PSIZE);
	_BL2_ERR_CASE(PADDRESS_PSIZE);
	default:
		str = "UNKNOWN";
	}
	return str;
}

static uint8_t nanohub_bl2_tx_create_cmd(struct nanohub_data *data,
					 uint32_t reason,
					 const void *tx, uint16_t len)
{
	const struct nanohub_bl *bl = &data->bl;
	struct nanohub_bl2 *bl2 = &data->bl2;

	struct bl2_packet_header *h = (struct bl2_packet_header *)bl->tx_buffer;
	struct bl2_packet_footer *f;

	if (BL2_PACKET_SIZE(len) > BL2_MSG_CMD_SIZE)
		return CMD_NACK;

	h->nop = BL2_NOP;
	h->sync = BL2_SYNC;
	h->seq = bl2->seq;
	h->reason = reason;
	h->len = len;

	if (tx && len)
		memcpy(h->data, tx, len);

	f = (struct bl2_packet_footer *)(h->data + len);

	f->crc = crc32((uint8_t *)h, sizeof(struct bl2_packet_header) + h->len,
			BL2_CRC32_INIT_DEFAULT);

	/* Pad IO TX buffer with the BL2_NOP value */
	memset(&f->nops[0], BL2_NOP, BL2_POST_NOPS + BL2_MSG_CMD_SIZE
			- BL2_PACKET_SIZE(len));

	bl2->rx_size = 0;
	bl2->io_cur_len = BL2_MSG_CMD_SIZE;
	bl2->rx_offset = BL2_PACKET_SIZE(h->len);

	return CMD_ACK;
}

static uint8_t nanohub_bl2_tx_create_buffer(struct nanohub_data *data,
					    uint32_t reason,
					    const void *tx, uint16_t len,
					    uint32_t flags, uint32_t mdsize)
{
	const struct nanohub_bl *bl = &data->bl;
	struct nanohub_bl2 *bl2 = &data->bl2;

	struct bl2_packet_header *h =
			(struct bl2_packet_header *)bl->tx_buffer;
	struct bl2_packet_footer *f;
	struct bl2_rw_buffer *buffer = (struct bl2_rw_buffer *)&h->data[0];

	mdsize += sizeof(struct bl2_rw_buffer);

	if (BL2_PACKET_SIZE(mdsize) > bl2->io_max_len)
		return CMD_NACK;

	h->nop = BL2_NOP;
	h->sync = BL2_SYNC;
	h->seq = bl2->seq;
	h->reason = reason;
	h->len = len + sizeof(struct bl2_rw_buffer);

	buffer->flags = flags;

	if (tx && len)
		memcpy(buffer->data, tx, len);

	f = (struct bl2_packet_footer *)(h->data + h->len);

	f->crc = crc32((uint8_t *)h, sizeof(struct bl2_packet_header) + h->len,
			BL2_CRC32_INIT_DEFAULT);

	/* Pad IO TX buffer with the BL2_NOP value */
	memset(&f->nops[0], BL2_NOP, BL2_POST_NOPS + BL2_PACKET_SIZE(mdsize)
			- BL2_PACKET_SIZE(h->len));

	bl2->rx_size = 0;
	bl2->io_cur_len = BL2_PACKET_SIZE(mdsize);
	bl2->rx_offset = BL2_PACKET_SIZE(h->len);

	return CMD_ACK;
}

static uint8_t nanohub_bl2_txrx(struct nanohub_data *data, uint8_t to)
{
	int res;
	const struct nanohub_bl *bl = &data->bl;
	struct nanohub_bl2 *bl2 = &data->bl2;

	struct bl2_packet_header *h = (struct bl2_packet_header *)bl->tx_buffer;
	uint32_t psize = BL2_MSG_CMD_SIZE;
	uint32_t tsize = bl2->io_cur_len;

	if (BL2_PACKET_SIZE(h->len) > bl2->io_max_len)
		return CMD_NACK;

	/* Extend the HW TX buffer */
	if (to) {
		uint8_t *pw = bl->tx_buffer + tsize;

		while ((to) && ((tsize + psize) < bl2->io_max_len)) {
			if (bl2->mode == BL2_MODE_CMD)
				memcpy(pw, bl->tx_buffer, psize);
			else
				memset(pw, BL2_NOP, psize);
			tsize += psize;
			pw += psize;
			to--;
		}
	}
	/* Update the exchanged IO TxRx HW buffer size */
	bl2->io_cur_len = tsize;

	/* Reset the IO RX buffer */
	memset(bl->rx_buffer, BL2_NOP, tsize);

	/* Perform the transfer */
	res = bl->txrx(data, tsize);
	if (res != tsize) {
		pr_err("%s: IO error res=%d\n", __func__, res);
		return CMD_NACK;
	};

	return CMD_ACK;
}

static uint8_t nanohub_bl2_verify(struct nanohub_data *data,
				  uint32_t reason,
				  uint32_t dlen)
{
	struct bl2_packet_footer *footer;
	uint16_t len;
	uint32_t crc;

	struct nanohub_bl2 *bl2 = &data->bl2;
	struct bl2_packet_header *packet =
			(struct bl2_packet_header *)bl2->rx_packet;

	len = packet->len;
	if (BL2_PACKET_SIZE(packet->len) > BL2_MSG_CMD_SIZE) {
		pr_err("%s: packet len is not valid %d > %d\n",
		       __func__, (int)BL2_PACKET_SIZE(packet->len),
		       (int)BL2_MSG_CMD_SIZE);
		return CMD_NACK;
	}

	footer = (struct bl2_packet_footer *)(packet->data + len);

	crc = crc32((uint8_t *)packet, sizeof(struct bl2_packet_header) + len,
		    BL2_CRC32_INIT_DEFAULT);

	if (crc != footer->crc) {
		pr_err("%s: CRC32 is not valid - %08x instead %08x\n",
		       __func__, crc, footer->crc);
		return CMD_NACK;
	}

	if ((reason != BL2_REASON_GET_STATUS) &&
	    (packet->reason == BL2_REASON_GET_STATUS)) {
		struct bl2_status_resp *rst =
				(struct bl2_status_resp *)&packet->data[0];

		pr_err("%s: mode=%d status=%d err=%d(%d,%d:%s)\n",
		       __func__, (int)rst->mode, (int)rst->status,
		       BL2_GET_ERROR(rst->error),
		       BL2_GET_MODULE_ID_ERROR(rst->error),
		       BL2_GET_LINE_ERROR(rst->error),
		       nanohub_bl2_err_to_str(rst->error));

		memcpy(&bl2->rstatus, rst, sizeof(struct bl2_status_resp));
		return CMD_NACK;
	}

	if (bl2->seq != packet->seq) {
		pr_err("%s: seq ID is not valid %u instead %u\n",
		       __func__, packet->seq, bl2->seq);
		return CMD_NACK;
	}

	if (reason != packet->reason) {
		pr_err("%s: reason is not valid %u instead %u\n",
		       __func__, packet->reason, reason);
		return CMD_NACK;
	}

	if (dlen != packet->len) {
		pr_err("%s: len is not valid %d instead %d\n",
		       __func__, (int)packet->len, (int)dlen);
		return CMD_NACK;
	}

	return CMD_ACK;
}

static int nanohub_bl2_find_sync(const uint8_t *buf, uint32_t size,
				 uint8_t *last)
{
	uint16_t pos = 0;

	for (pos = 0; pos < size; pos++) {
		if (buf[pos] == BL2_SYNC) {
			if ((pos) && (buf[pos - 1] == BL2_NOP)) {
				return (pos - 1);
			} else {
				/* take account the case where NOP has been
				 * generated at the end of the previous packet
				 */
				if (!pos) {
					if (*last == BL2_NOP)
						return pos;
				}
			}
		}
	}
	*last = buf[size - 1];
	return -1;
}

static uint8_t nanohub_bl2_rx_reply(struct nanohub_data *data, uint32_t rdelay)
{
	int res;
	uint8_t status;
	const struct nanohub_bl *bl = &data->bl;
	struct nanohub_bl2 *bl2 = &data->bl2;
	uint32_t mlen = BL2_MSG_CMD_SIZE;
	uint32_t cs = bl2->io_cur_len - bl2->rx_offset;
	uint32_t rs = mlen;
	uint32_t iter = 0;
	uint8_t *rx = bl->rx_buffer;
	uint8_t last = 0;
	uint32_t to = BL2_RESP_TIMEOUT;

	if (bl2->rx_size) /* Invalid state */
		return CMD_NACK;

	pr_debug("nanohub: %s to=%d rdelay=%dms (txRxSize=%u rxOffset=%u, mlen=%u)\n",
		 __func__, (int)to, (int)rdelay, bl2->io_cur_len,
		 bl2->rx_offset, mlen);

	do {
		if ((bl2->rx_size < mlen) && (cs)) {
			if (!bl2->rx_size)
				res = nanohub_bl2_find_sync(rx + bl2->rx_offset,
							    cs, &last);
			else
				res = 0;
			if (res < 0) {
				pr_debug("nanohub: %s It%d - RX sync not found... res=%d (off=%u,%u)\n",
					 __func__, (int)iter, res,
					 bl2->rx_offset, cs);
			} else {
				cs -= res;
				cs = (cs > mlen) ? mlen : cs;

				if ((res == 0) && (bl2->rx_size == 0)) {
					if (bl->rx_buffer[bl2->rx_offset]
							  == BL2_SYNC) {
						bl2->rx_packet[0] = BL2_NOP;
						bl2->rx_size = 1;
						cs -= 1;
					}
				}

				pr_debug("nanohub: %s It%d - copy RX frame %u/%u (off=%d)\n",
					 __func__, (int)iter, cs, rs, res);

				memcpy(&bl2->rx_packet[bl2->rx_size],
				       &bl->rx_buffer[bl2->rx_offset + res],
				       cs);
				bl2->rx_size += cs;
				rs = mlen - bl2->rx_size;
			}
		}

		if ((iter == 0) && (bl2->rx_size == 0) && (rdelay))
			usleep_range(rdelay * 1000, rdelay * 1000 + 50);

		/* generate a new TX/RX request - repeat the cmd packet */
		if ((rs) && (to)) {
			bl2->io_cur_len = rs;
			memset(bl->rx_buffer, BL2_NOP, rs);
			bl2->rx_offset = 0;

			if (bl2->mode != BL2_MODE_CMD)
				memset(bl->tx_buffer, BL2_NOP, rs);

			cs =  rs; /* bl2->rxOffset = 0 */

			/* Perform the transfer */
			res = bl->txrx(data, cs);
			if (res != cs) {
				pr_err("%s: IO error res=%d\n", __func__, res);
				return CMD_NACK;
			};

#ifdef __KERNEL__
			if (iter % 256 == 0)
				schedule();
#endif
		}

		/* next iteration */
		iter++;

	} while ((rs) && (iter <= to));

	if (rs == 0) {
		/* Complete RX packet found */
		status = CMD_ACK;
	} else {
		pr_err("%s: TIMEOUT (to=%d, rdelay=%dms, %d/%d found!)\n",
		       __func__, (int)to, (int)rdelay, (int)bl2->rx_size,
		       (int)mlen);
		status = CMD_NACK;
	}

	return status;
}

static struct bl2_packet_header *nanohub_bl2_cmd(struct nanohub_data *data,
						 uint32_t reason,
						 const void *tx, uint16_t len,
						 uint16_t rlen, uint8_t wto,
						 uint32_t rdelay)
{
	uint8_t status;
	struct nanohub_bl2 *bl2 = &data->bl2;

	bl2->mode = BL2_MODE_CMD;
	bl2->seq++;

	memset(&bl2->rstatus, 0, sizeof(struct bl2_status_resp));

	status = nanohub_bl2_tx_create_cmd(data, reason, tx, len);
	if (status != CMD_ACK)
		return NULL;

	status = nanohub_bl2_txrx(data, wto);
	if (status != CMD_ACK)
		return NULL;

	status = nanohub_bl2_rx_reply(data, rdelay);
	if (status != CMD_ACK)
		return NULL;

	status = nanohub_bl2_verify(data, reason, rlen);
	if (status != CMD_ACK)
		return NULL;

	return (struct bl2_packet_header *)bl2->rx_packet;
}

static struct bl2_packet_header *nanohub_bl2_txbuffer(struct nanohub_data *data,
						      const void *tx,
						      uint16_t len,
						      uint32_t dlen,
						      uint32_t flags,
						      uint32_t rdelay,
						      uint32_t repeat)
{
	uint8_t status;
	struct nanohub_bl2 *bl2 = &data->bl2;
	struct bl2_packet_header *h =
			(struct bl2_packet_header *)bl2->rx_packet;

	bl2->mode = BL2_MODE_BUFFER_TX;
	if (!repeat)
		bl2->seq++;

	memset(&bl2->rstatus, 0, sizeof(struct bl2_status_resp));

	status = nanohub_bl2_tx_create_buffer(data,
					      BL2_REASON_WRITE_BUFFER,
					      tx, len, flags, dlen);
	if (status != CMD_ACK)
		return NULL;

	/* Delay to be sure that the MCU will be ready to receive a
	 * TX BUFFER msg type. This extra delay (~150us) avoids a potential
	 * retrieve mechanism when MCU frequency is low (vs APE frequency).
	 * Throughput with the large buffers is not impacted.
	 */
	usleep_range(150, 170);

	status = nanohub_bl2_txrx(data, 0);
	if (status != CMD_ACK)
		return NULL;

	status = nanohub_bl2_rx_reply(data, rdelay);
	if (status != CMD_ACK)
		return NULL;

	status = nanohub_bl2_verify(data, BL2_REASON_FLASH_OP,
				    sizeof(struct bl2_memop_resp));
	if (status != CMD_ACK)
		return NULL;

	return h;
}

uint8_t nanohub_bl2_sync(struct nanohub_data *data, uint32_t mlen)
{
	uint8_t status = CMD_ACK;
	uint32_t to = BL2_SYNC_TIMEOUT;
	int res;
	const struct nanohub_bl *bl = &data->bl;
	struct nanohub_bl2 *bl2 = &data->bl2;

	memset(bl2, 0, sizeof(struct nanohub_bl2));

	if (mlen < BL2_PACKET_SIZE(BL2_MIN_DATA_SIZE
			+ sizeof(struct bl2_rw_buffer)))
		return CMD_NACK;

	bl2->io_max_len = mlen;

	bl->tx_buffer[0] = BL2_SYNC_INIT;
	bl->rx_buffer[0] = 0xFF;

	do {
		res = bl->txrx(data, 1);
		if (res != 1) {
			pr_err("%s: IO error res=%d\n", __func__, res);
			return CMD_NACK;
		}
		to--;

#ifdef __KERNEL__
		if ((bl->rx_buffer[0] != BL2_SYNC_ACK) && (to % 256 == 0))
			schedule();
#endif

	} while ((to) && (bl->rx_buffer[0] != BL2_SYNC_ACK));

	pr_debug("nanohub: %s to=%d/%d (mlen=%u)\n", __func__,
		 (int)(BL2_SYNC_TIMEOUT - to),
		 (int)BL2_SYNC_TIMEOUT, mlen);

	if (!to) {
		pr_err("%s Unable to sync with BL2 (TIMEOUT reached)\n",
		       __func__);
		status = CMD_NACK;
	}

	return status;
}

uint8_t nanohub_bl2_info(struct nanohub_data *data,
			 struct bl2_info_resp *rinfo)
{
	struct bl2_packet_header *h;

	h = nanohub_bl2_cmd(data, BL2_REASON_GET_INFO,
			    NULL, 0, sizeof(struct bl2_info_resp), 10, 0);

	if ((h) && (rinfo))
		memcpy(rinfo, &h->data[0], sizeof(struct bl2_info_resp));

	return h ? CMD_ACK : CMD_NACK;
}

static void nanohub_bl2_log_error(const char *fctstr, const char *extra,
				  struct bl2_memop_resp *rmemop)
{
	const char *ex;

	if (!extra)
		ex = "";
	else
		ex = extra;

	pr_err("%s: OP %s failed err=%d(%d,%d:%s)\n",
	       fctstr, extra,
	       BL2_GET_ERROR(rmemop->error),
	       BL2_GET_MODULE_ID_ERROR(rmemop->error),
	       BL2_GET_LINE_ERROR(rmemop->error),
	       nanohub_bl2_err_to_str(rmemop->error));
}

uint8_t nanohub_bl2_flashinfo(struct nanohub_data *data, uint32_t sector,
			      struct bl2_memop_resp *rmeminfo)
{
	struct bl2_packet_header *h;
	struct bl2_memop_cmd memop;
	struct bl2_memop_resp *rmemop;

	memop.op = BL2_FLASH_OP_INFO;
	memop.address = sector;
	memop.size = 0;
	memop.extra = 0;

	h = nanohub_bl2_cmd(data, BL2_REASON_FLASH_OP,
			    &memop, sizeof(struct bl2_memop_cmd),
			    sizeof(struct bl2_memop_resp), 10, 0);

	if (h) {
		rmemop = (struct bl2_memop_resp *)&h->data[0];
		if (rmemop->error == BL2_ERROR_OK) {
			if (rmeminfo)
				memcpy(rmeminfo, rmemop,
				       sizeof(struct bl2_memop_resp));
		} else {
			nanohub_bl2_log_error(__func__, NULL, rmemop);
			h = NULL;
		}
	}

	return h ? CMD_ACK : CMD_NACK;
}

uint8_t nanohub_bl2_flashcrc32(struct nanohub_data *data,
			       uint32_t addr, uint32_t len,
			       uint32_t *crc, uint32_t *ms)
{
	struct bl2_packet_header *h;
	struct bl2_memop_cmd memop;
	struct bl2_memop_resp *rmemop;
	ktime_t ts, te;

	memop.address = addr;
	memop.size = len;
	memop.op = BL2_FLASH_OP_CRC32;
	memop.extra = crc ? *crc : BL2_CRC32_INIT_DEFAULT;

	ts = ktime_get();

	h = nanohub_bl2_cmd(data, BL2_REASON_FLASH_OP,
			    &memop, sizeof(struct bl2_memop_cmd),
			    sizeof(struct bl2_memop_resp),
			    2, (len * BL2_TIME_CRC32_2048_US) / (2048 * 1000));

	if (h) {
		rmemop = (struct bl2_memop_resp *)&h->data[0];
		if (rmemop->error == BL2_ERROR_OK) {
			if (crc)
				*crc = rmemop->extra;
		} else {
			nanohub_bl2_log_error(__func__, NULL, rmemop);
			h = NULL;
		}
	}

	te = ktime_get();
	if (ms)
		*ms = (uint32_t)ktime_to_ms(ktime_sub(te, ts));

	return h ? CMD_ACK : CMD_NACK;
}

uint8_t nanohub_bl2_flasherase(struct nanohub_data *data,
			       uint32_t sector, uint32_t len, uint32_t *ms)
{
	struct bl2_packet_header *h;
	struct bl2_memop_cmd memop;
	struct bl2_memop_resp *rmemop;
	ktime_t ts, te;

	if ((!len) || (sector == ~0))
		len = 1;

	memop.op = BL2_FLASH_OP_ERASE;
	memop.address = sector;
	memop.size = len;
	memop.extra = 0;

	ts = ktime_get();

	h = nanohub_bl2_cmd(data, BL2_REASON_FLASH_OP,
			    &memop, sizeof(struct bl2_memop_cmd),
			    sizeof(struct bl2_memop_resp),
			    255, len * BL2_TIME_ERASE_PAGE_MS);

	if (h) {
		rmemop = (struct bl2_memop_resp *)&h->data[0];
		if (rmemop->error != BL2_ERROR_OK) {
			nanohub_bl2_log_error(__func__, NULL, rmemop);
			h = NULL;
		}
	}

	te = ktime_get();
	if (ms)
		*ms = (uint32_t)ktime_to_ms(ktime_sub(te, ts));

	return h ? CMD_ACK : CMD_NACK;
}

uint8_t nanohub_bl2_flashprog(struct nanohub_data *data,
			      uint32_t addr, const uint8_t *image,
			      uint32_t len, uint32_t *ms)
{
	struct bl2_packet_header *h;
	struct bl2_memop_cmd cmemop;
	struct bl2_memop_resp *rmemop;
	uint32_t bsize;
	struct nanohub_bl2 *bl2 = &data->bl2;

	uint8_t *pr;
	uint32_t tlen, clen;
	uint32_t flags;
	int loop;
	uint32_t pw;
	uint32_t retry;
	uint32_t rdelay;
	ktime_t ts, te;

	if ((!image) || (!len))
		return CMD_NACK;

	bsize = bl2->io_max_len - BL2_PACKET_SIZE(0)
			- sizeof(struct bl2_rw_buffer);
	bsize = (bsize > len) ? len : bsize;

	cmemop.op = BL2_FLASH_OP_WRITE;
	cmemop.address = addr;
	cmemop.size = len;
	cmemop.extra = bsize;

	pr_debug("nanohub: %s init @0x%08x/%u (bsize=%d)...\n",
		 __func__, addr, len, (int)bsize);

	ts = ktime_get();

	h = nanohub_bl2_cmd(data, BL2_REASON_FLASH_OP,
			    &cmemop, sizeof(struct bl2_memop_cmd),
			    sizeof(struct bl2_memop_resp), 0, 0);

	if (!h)
		goto out;

	rmemop = (struct bl2_memop_resp *)&h->data[0];
	if (rmemop->error != BL2_ERROR_OK) {
		nanohub_bl2_log_error(__func__, "INIT", rmemop);
		h = NULL;
		goto out;
	}

	/* Main loop */

	pw = addr;
	bsize = rmemop->extra;
	if (!bsize)
		bsize = 8;

	loop = (int)(len / bsize);
	loop += (int)(len % bsize) ? 1 : 0;

	pr_debug("nanohub: %s starting (#%d, bsize=%d)...\n",
		 __func__, loop, (int)bsize);

	pr = (uint8_t *)image;
	tlen = len;

	loop = 0;
	retry = 0;

	do {
		rdelay = 0;
		clen = (tlen > bsize) ? bsize : tlen;
		if ((tlen - clen) == 0) {
			rdelay = (clen * BL2_TIME_PROG_2048_MS);
			rdelay /= 2048;
			flags = BL2_BUFFER_FLAGS_LAST;
		} else {
			flags = 0;
		}

		h = nanohub_bl2_txbuffer(data, pr, clen, bsize, flags,
					 rdelay, retry);

		if (h) {
			rmemop = (struct bl2_memop_resp *)&h->data[0];
			if (rmemop->error != BL2_ERROR_OK) {
				nanohub_bl2_log_error(__func__, "WRITE",
						      rmemop);
				pr_err("%s: BAD @/size=@0x%08x/%u\n",
				       __func__, rmemop->address,
				       rmemop->size);
				h = NULL;
			} else {
				tlen -= clen;
				pr += clen;
				pw += clen;
				loop++;
			}

			retry = 0;
		} else {
			if (((BL2_GET_ERROR(bl2->rstatus.error) ==
			      BL2_ERROR_SYNC) ||
			    (BL2_GET_ERROR(bl2->rstatus.error) ==
			      BL2_ERROR_FRAME)) &&
			    (retry < BL2_MAX_RETRY_TX_BUFFER)) {
				retry++;
				pr_warn("nanohub: %s repeat the TX_BUFFER msg (%d) err=%d(%d,%d:%s)\n",
					__func__, (int)retry,
					BL2_GET_ERROR(bl2->rstatus.error),
					BL2_GET_MODULE_ID_ERROR(
							   bl2->rstatus.error),
					BL2_GET_LINE_ERROR(
							   bl2->rstatus.error),
					nanohub_bl2_err_to_str(
							   bl2->rstatus.error));
				/* Just to have h!= NULL */
				h = (struct bl2_packet_header *)&h;
				usleep_range(500, 1000);
			} else
				pr_err("%s OP failed iter=%u @0x%08x/%u\n",
				       __func__, (uint32_t)loop, pw, clen);
		}
	} while ((h) && (tlen));

out:
	te = ktime_get();
	if (ms)
		*ms = (uint32_t)ktime_to_ms(ktime_sub(te, ts));

	return h ? CMD_ACK : CMD_NACK;
}

uint8_t nanohub_bl2_status(struct nanohub_data *data,
			   struct bl2_status_resp *rstatus)
{
	struct bl2_packet_header *h;

	h = nanohub_bl2_cmd(data, BL2_REASON_GET_STATUS, NULL, 0,
			    sizeof(struct bl2_status_resp), 10, 0);

	if ((h) && (rstatus))
		memcpy(rstatus, &h->data[0], sizeof(struct bl2_status_resp));

	return h ? CMD_ACK : CMD_NACK;
}

