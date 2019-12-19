/*
 * Utility definitions for TUSB544 USB3.0 re-drive
 *
 * Copyright (C) 2017 HTC Corporation.
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

/* General Registers */
#define REG_REVISION		0x08
#define REG_CONFIG_CTRL		0x0A
#define REG_CHANNEL_SWAP_SEL	0x0B
#define REG_VOD_DCGAIN		0x0C
#define REG_DP_TX2		0x10
#define REG_DP_TX1		0x11
#define REG_DP_AUX		0x13
#define REG_USB3_TX2		0x20
#define REG_USB3_TX1		0x21

/* register CONFIG_CTRL value */
#define DISABLE_TX_RX	0x00
/* data pin */
#define USB3_ON_CC1	0x01   /* original is 0x01 on CC1 */
#define USB3_ON_CC2	0x05   /* original is 0x05 on CC2 */
/*
 * For CC1,
 * 4 Lane DP -- 0xA
 * DIR0 - L, DIR1 - L, CTL1 - H, CTL0 - L, FLIP - L
 * URX2P(DP0P) to DRX2P, URX2N(DP0N) to DRX2N
 * UTX2P(DP1P) to DTX2P, UTX2N(DP1N) to DTX2N
 * UTX1P(DP2P) to DTX1P, UTX1N(DP2N) to DTX1N
 * URX1P(DP3P) to DRX1P, URX1N(DP3N) to DRX1N
 *
 * 4 Lane DP -- 0xE
 * DIR0 - L, DIR1 - L, CTL1 - H, CTL0 - L, FLIP - H
 * URX1P(DP0P) to DRX1P, URX1N(DP0N) to DRX1N
 * UTX1P(DP1P) to DTX1P, UTX1N(DP1N) to DTX1N
 * UTX2P(DP2P) to DTX2P, UTX2N(DP2N) to DTX2N
 * URX2P(DP3P) to DRX2P, URX2N(DP3N) to DRX2N
 *
 * DP0 -- RX2 >> 0x0A, DP0 -- RX1 >> 0xE for CC1
 * According HW design, select the settings.
 */
#define DP_ON_CC1_C	0x0A
#define DP_ON_CC2_C	0x0E
#define DP_ON_CC1_D	0x0B
#define DP_ON_CC2_D	0x0F

/* register DP_AUX value */
#define ENABLE_AUX_SNOOP	0x00
#define DISABLE_AUX_SNOOP_C	0x80
#define DISABLE_AUX_SNOOP_D	0x8C
#define AUX_SBU_CC1	0x10	/* AUXP to SBU1 and AUXN to SBU2 */
#define AUX_SBU_CC2	0x20	/* AUXN to SBU2 and AUXP to SBU1 */
#define EQ_OVERRIDE	0x10

enum cc_state {
	CC_STATE_OPEN = 0,
	CC_STATE_USB3,
	CC_STATE_DP,
	CC_STATE_RARA,
};

/* must align with smblib's integer representation of cc orientation. */
enum cc_orientation {
	CC_ORIENTATION_NONE = 0,
	CC_ORIENTATION_CC1 = 1,
	CC_ORIENTATION_CC2 = 2,
};

void tusb544_update_state(enum cc_state cc_state, enum cc_orientation cc_orientation, u8 pin);
