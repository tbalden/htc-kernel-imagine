/*
 * Copyright (C) 2016 Google, Inc.
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

/*
 * BL2 protocol definition
 *
 * This file is shared between the HOST and MCU code.
 *
 */

#ifndef __BL2MSG_H__
#define __BL2MSG_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>

typedef uint16_t __le16;
typedef uint32_t __le32;
#endif

#define BL2_API_VERSION_MAJOR	(1)
#define BL2_API_VERSION_MINOR	(0)
#define BL2_API_VERSION		(BL2_API_VERSION_MAJOR << 8 \
				| BL2_API_VERSION_MINOR)

/*
 *   BL2 MSG definition
 */

#define BL2_SYNC_INIT      0x5A
#define BL2_SYNC_ACK       0xA5
#define BL2_NOP            0xFF

#define BL2_SYNC           0x31
#define BL2_POST_NOPS      4

struct bl2_packet_header {
	uint8_t nop;
	uint8_t sync;
	__le32  seq;
	__le32  reason;
	__le16  len;
	uint8_t data[0];
} __attribute__((__packed__));

struct bl2_packet_footer {
	__le32  crc;
	uint8_t nops[BL2_POST_NOPS];
} __attribute__((__packed__));

#define BL2_PACKET_SIZE(_len_) \
	(sizeof(struct bl2_packet_header) + (_len_) \
	+ sizeof(struct bl2_packet_footer))

/*
 * BL2 error definition (uint32_t type)
 * ********************
 * Part of the struct Bl2StatusResp and struct Bl2MemOpResp.
 *
 * Error number, Module ID and line number are encoded in a single uint32_t
 * as follow:
 *
 *     b31..b16        line number in the C-file (__LINE__ value)
 *     b15..b8         module ID (BL2_ERROR_MOD_ID define value)
 *     b7..b0          error number
 */

#define BL2_ERROR_OK                (0)

#define BL2_ERROR_PARAM             (1)  /* generic PARAM error */
#define BL2_ERROR_REASON            (2)  /* reason is not valid */
#define BL2_ERROR_FRAME             (3)  /* frame format is not valid */
#define BL2_ERROR_SYNC              (4)  /* sync not found */

#define BL2_ERROR_GENERIC           (5)  /* generic ERROR */
#define BL2_ERROR_NOT_SUPPORTED     (6)  /* CMD not supported */

#define BL2_ERROR_PADDRESS          (10) /* @ parameter is not valid */
#define BL2_ERROR_PSIZE             (11) /* size parameter is not valid */
#define BL2_ERROR_PADDRESS_PSIZE    (12) /* both @/size are not valid */

#if !defined(BL2_ERROR_MOD_ID)
#define BL2_ERROR_MOD_ID (0)        /* default module ID */
#endif

#define BL2_SET_ERROR(_val)         ((__LINE__ << 16) | \
					(BL2_ERROR_MOD_ID << 8) | (_val))

#define BL2_GET_ERROR(_err)           (int)((_err) & 0xFF)
#define BL2_GET_LINE_ERROR(_err)      (int)((_err) >> 16)
#define BL2_GET_MODULE_ID_ERROR(_err) (int)(((_err) >> 8) & 0xFF)

/*
 * CMD/RESPONSE PAYLOAD/REASON definitions
 */

/*
 * BL2_REASON_GET_STATUS
 * *********************
 * Return the slave status
 *
 * MSG.IN
 *     no parameters
 *
 * MSG.OUT
 *     o struct Bl2StatusResp (BL2_REASON_GET_STATUS) with BL2_ERROR_OK if OK.
 *       BL2_ERROR_PARAM is returned, if the IN parameter is not empty
 *
 */

#define BL2_REASON_GET_STATUS   0x00020000

#define BL2_MODE_CMD            (0)
#define BL2_MODE_BUFFER_TX      (1)

#define BL2_STATUS_READY        (0)
#define BL2_STATUS_BUSY         (1)

struct bl2_status_resp {
	__le32 mode;
	__le32 status;
	__le32 error;
	__le32 pad;
} __attribute__((__packed__));

/*
 * BL2_REASON_GET_INFO
 * *******************
 * Return the slave information
 *
 * MSG.IN
 *     no parameters
 *
 * MSG.OUT
 *     o struct Bl2InfoResp (BL2_REASON_GET_INFO) if OK
 *     o struct Bl2StatusResp (BL2_REASON_GET_STATUS) with BL2_ERROR_PARAM is
 *       returned, if the IN payload is not empty
 */

#define BL2_REASON_GET_INFO     0x00010000

struct bl2_info_resp {
	__le32 ver;             /* Major/Minor API b31..b16  */
				/* Implementation b15..b0 version */
	__le32 id;              /* Device ID code   */
	__le32 flash_desc;      /* Short flash device descriptor */
	__le32 max_size;        /* Maximum payload size supported by the BL2 */
} __attribute__((__packed__));

/* ver: Version definition */
#define BL2_INFO_API_VERSION(_ver)      (((_ver) >> 16) & 0xFFFF)
#define BL2_INFO_API_MAJOR(_ver)        (((_ver) >> 24) & 0xFF)
#define BL2_INFO_API_MINOR(_ver)        (((_ver) >> 16) & 0xFF)

#define BL2_INFO_IMP_VERSION(_ver)      ((_ver) & 0xFFFF)
#define BL2_INFO_IMP_MAJOR(_ver)        (((_ver) >> 8) & 0xFF)
#define BL2_INFO_IMP_MINOR(_ver)        (((_ver) >> 0) & 0xFF)

/* flashDesc: Short Flash description */
#define BL2_FLASH_UNIFIED      (1 << 31)    /* Unified sector */
#define BL2_FLASH_PEMPTY       (1 << 30)    /* Program Empty bit value */

#define BL2_FLASH_PUNIT        (0xFF << 16) /* Smallest programmable unit */
					    /* in bytes */
#define BL2_FLASH_SECTOR_COUNT (0xFFFF)     /* Total sector/page number */

#define BL2_FLASH_GET_PUINT(_attr)        (((_attr) & BL2_FLASH_PUNIT) >> 16)
#define BL2_FLASH_GET_SECTOR_COUNT(_attr) ((_attr) & BL2_FLASH_SECTOR_COUNT)
#define BL2_FLASH_IS_UNIFIED(_attr)       (((_attr) & BL2_FLASH_UNIFIED) \
						== BL2_FLASH_UNIFIED)
#define BL2_FLASH_PEMPTY_BIT(_attr)       (((_attr) & BL2_FLASH_PEMPTY) \
						== BL2_FLASH_PEMPTY)

/*
 * BL2_REASON_FLASH_OP
 * *******************
 * Flash memory operation for a large memory chunk
 *
 * MSG.IN
 *     o struct Bl2MemOpCmd
 *
 *     Supported main flash operations
 *
 *       BL2_FLASH_OP_INFO       sector/page info
 *       BL2_FLASH_OP_CRC32      CRC32 calculation
 *       BL2_FLASH_OP_ERASE      erase
 *       BL2_FLASH_OP_WRITE      program
 *
 * MSG.OUT
 *     o struct BlMemOpResp (BL2_REASON_FLASH_OP)
 *       if the operation has been processed. Possible error is returned
 *       in the "error" field
 *     o struct Bl2StatusResp (BL2_REASON_GET_STATUS) with BL2_ERROR_PARAM
 *       is returned, if the IN payload is not empty
 */

#define BL2_REASON_FLASH_OP       0x00400000

#define BL2_FLASH_OP_NO           (0)
#define BL2_FLASH_OP_CRC32        (1)
#define BL2_FLASH_OP_ERASE        (2)
#define BL2_FLASH_OP_WRITE        (3)
#define BL2_FLASH_OP_INFO         (4)

#define BL2_CRC32_INIT_DEFAULT   (~0)

struct bl2_memop_cmd {
	__le32 op;              /* operation type */
	__le32 address;         /* base address */
	__le32 size;            /* total size   */
	__le32 extra;           /* depend of the operation type */
} __attribute__((__packed__));

struct bl2_memop_resp {
	__le32 error;           /* error */
	__le32 address;         /* base address */
	__le32 size;            /* total size   */
	__le32 extra;           /* depend of the requested operation type */
} __attribute__((__packed__));

/*
 * BL2_REASON_FLASH_OP::BL2_FLASH_OP_INFO
 * **************************************
 * - flash sector/page info
 *
 *   MSG.IN (struct BlMemOpCmd)
 *
 *     op          BL2_FLASH_OP_INFO
 *     address     sector/page number
 *                 if (~0UL) return the full flash information
 *     size        unused
 *     extra       unused
 *
 *   MSG.OUT (struct BlMemOpResp)
 *
 *     address     if error == BL2_ERROR_OK
 *                   base @ of the requested sector/page number
 *                 else
 *                   requested sector/page number
 *     size        if error == BL2_ERROR_OK
 *                   size of the expected sector/page number
 *                 else
 *                   0
 *     extra       flash property (see Bl2InfoResp.flashDesc)
 *                 only _UNIFIED & _PUNIT fields are returned.
 *     error       BL2_ERROR_OK
 *                 BL2_ERROR_PADDRESS (sector number is not valid)
 *
 *
 * BL2_REASON_FLASH_OP::BL2_FLASH_OP_CRC32
 * ***************************************
 * - calculate a CRC32 on a flash memory chunk
 *
 *   MSG.IN (struct BlMemOpCmd)
 *
 *     op          BL2_FLASH_OP_CRC32
 *     address     start/base address of the flash memory chunk
 *     size        size of the memory chunk
 *     extra       CRC init (default = ~0UL)
 *
 *   MSG.OUT (struct BlMemOpResp)
 *
 *     address     repeat the requested start/base address
 *     size        repeat the requested size
 *     extra       calculated crc (if BL2_ERROR_OK)
 *     error       BL2_ERROR_OK
 *                 BL2_ERROR_PSIZE, BL2_ERROR_PADDRESS,
 *                 BL2_ERROR_PADDRESS_PSIZE (@/size out of range)
 *                 BL2_ERROR_GENERIC (operation has failed)
 *
 *
 * BL2_REASON_FLASH_OP::BL2_FLASH_OP_ERASE
 * ***************************************
 * - erase a flash memory sector
 *
 *   MSG.IN (struct BlMemOpCmd)
 *
 *     op          BL2_FLASH_OP_ERASE
 *     address     initial sector/page number
 *                 if (~0UL) mass flash erase operation is requested
 *     size        number of sector/page (0 and 1 is equivalent)
 *     extra       unused
 *
 *   MSG.OUT (struct BlMemOpResp)
 *
 *     address     if error == BL2_ERROR_OK
 *                   requested sector/page number
 *                 else
 *                   sector/page number which has failed
 *     size        if error == BL2_ERROR_OK
 *                   number of sector/page
 *                 else
 *                   1
 *     extra       0 - unused
 *     error       BL2_ERROR_OK
 *                 BL2_ERROR_PSIZE, BL2_ERROR_PADDRESS,
 *                 BL2_ERROR_PADDRESS_PSIZE (@/size out of range)
 *                 BL2_ERROR_GENERIC (operation has failed)
 *
 *  BL2_REASON_FLASH_OP::BL2_FLASH_OP_WRITE
 * ***************************************
 * - start/initiate an incremental flash program process
 *
 *   MSG.IN (struct BlMemOpCmd)
 *
 *     op          BL2_FLASH_OP_WRITE
 *     address     start/base address (should be _PUNIT-aligned)
 *     size        total size of the programmed memory chunk
 *     extra       requested size for the exchanged buffers (note1)
 *
 *   MSG.OUT (struct BlMemOpResp)
 *
 *     address     requested start/base address
 *     size        requested size
 *     extra       adjusted size for the exchanged buffers (note1)
 *     error       BL2_ERROR_OK
 *                 BL2_ERROR_PSIZE, BL2_ERROR_PADDRESS,
 *                 BL2_ERROR_PADDRESS_PSIZE (@/size out of range)
 *                 BL2_ERROR_GENERIC (operation has failed)
 *
 *   note1: If BL2_ERROR_OK, the IO RX MSG packets with BL2_REASON_WRITE_BUFFER
 *          reason should be used to download the data until to receive a
 *          packet with the BL2_BUFFER_FLAGS_LAST flag. IO RX MSG size of these
 *          buffers is defined by:
 *
 *               BL2_PACKET_SIZE(BlMemOpResp.extra + sizeof(__le32))
 *
 *
 *   Typical WRITE sequence:
 *
 *     Master                          Slave
 *
 *     1. BL2_REASON_FLASH_OP
 *        (BL2_FLASH_OP_WRITE)
 *
 *                                     r1. BL2_REASON_FLASH_OP
 *
 *     2. BL2_REASON_WRITE_BUFFER (chunk 1)
 *
 *                                     r2. BL2_REASON_FLASH_OP (note2)
 *
 *
 *     N. BL2_REASON_WRITE_BUFFER (chunk N - last memory chunk)
 *       with BL2_BUFFER_FLAGS_LAST
 *
 *                                     rN. BL2_REASON_FLASH_OP
 *
 *   note2: Programming process for the chunk(i) is started after the
 *          associated response. In parallel, the chunk i+1 can be download.
 *          For the chunk i, BlMemOpResp contains the result of the (i-1)
 *          operation. Only for the last chunk, the programming operation
 *          is completed before to send the answer.
 */

/*
 * BL2_REASON_WRITE_BUFFER
 * ***********************
 *
 * Data for incremental flash memory operation.
 *
 * As mentioned in the note1, size of the IO RX MSG packet is fixed but not
 * necessarily full. IO RX MSG packet should be padded with NOP bytes (FF).
 * Provided data size is given by:
 *
 *     PayLoad/data size = Bl2PacketHeader.len - sizeof(struct Bl2RWBuffer)
 *
 * Only the data are provided. Written address is incremented according
 * the previous data size. This implied that the data size should be
 * _PUNIT-aligned. Only the size of the last data can be not aligned
 * (in this case the data are padded with the "00").
 *
 * MSG.IN
 *     o struct Bl2RWBuffer
 *
 * MSG.OUT
 *     o struct BlMemOpResp (BL2_REASON_FLASH_OP)
 *
 *     address     incremented @ which has been programmed
 *     size        programmed size
 *     extra       write sequence number (debug purpose)
 *     error       BL2_ERROR_OK
 *                 BL2_ERROR_GENERIC (operation has failed at @/size) (note3)
 *
 *     o Else struct Bl2StatusResp (BL2_REASON_GET_STATUS)
 *       with BL2_ERROR_SYNC/_FRAME if the slave has not be able
 *       to receive the RX packet (note4)
 *
 *   note3: If BL2_ERROR_GENERIC is returned, complete sequence
 *          should be re-started.
 *
 *   note4: If BL2_ERROR_SYNC/_FRAME is returned, the master must re-send
 *          the same data with the same seq ID. To abort the write sequence,
 *          BL2_REASON_WRITE_BUFFER should be sent with 0-data and
 *          BL2_BUFFER_FLAGS_LAST flag.
 */

#define BL2_REASON_WRITE_BUFFER  0x00410000

#define BL2_BUFFER_FLAGS_LAST    (1 << 0) /* Indicate the last memory chunk */

struct bl2_rw_buffer {
	__le32 flags;
	uint8_t data[0];
} __attribute__((__packed__));

/* CMD/RESP Data/PayLoad */

struct bl2_cmd_resp {
	union {
		struct bl2_memop_cmd   memop_cmd;
		struct bl2_info_resp   info_resp;
		struct bl2_status_resp status_resp;
		struct bl2_memop_resp  memop_resp;
	};
} __attribute__((__packed__));

#define BL2_CMD_MAX_DATA_SIZE  (sizeof(struct bl2_cmd_resp))

/* BUFFER Data/PayLoad */

#if !defined(BL2_BUFF_MAX_SIZE)
#define BL2_BUFF_MAX_SIZE      (1024)      /* max buffer data size */
#endif

#define BL2_PACKET_SIZE_MIN    BL2_PACKET_SIZE(0)
#define BL2_PACKET_SIZE_CMD    BL2_PACKET_SIZE(BL2_CMD_MAX_DATA_SIZE)

#define BL2_PACKET_SIZE_MAX    BL2_PACKET_SIZE(BL2_BUFF_MAX_SIZE + \
				sizeof(struct bl2_rw_buffer))

#define BL2_MSG_CMD_SIZE       BL2_PACKET_SIZE(BL2_CMD_MAX_DATA_SIZE)

#endif /* __BL2MSG_H__ */
