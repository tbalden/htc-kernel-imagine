/*
 * HTC MISC virtual extcon connector
 *
 * Copyright (C) 2017 HTC Corp.
 * Author: JJ Lee <jj_lee@htc.com>
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
#ifndef __EXTCON_HTC_MISC_H__
#define __EXTCON_HTC_MISC_H__ __FILE__

#include <linux/extcon.h>

#ifdef CONFIG_EXTCON_HTC_MISC
// export functions here
int htc_misc_extcon_set_state(unsigned int id, bool state);
#else
static int htc_misc_extcon_set_state(unsigned int id, bool state) {return 0;}
#endif

#endif /* __EXTCON_HTC_MISC_H__ */
