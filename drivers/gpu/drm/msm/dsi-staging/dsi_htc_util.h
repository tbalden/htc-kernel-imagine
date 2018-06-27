/**
 * Copyright (C) 2017 HTC. Corporation.
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
 * \file dsi_htc_util.h
 * \version 1.0
 * \date 廿十七年十一月廿九日
 * \par Compiler:
 * arm-eabi-gcc
 */
#ifndef DSI_HTC_UTIL_H_INCLUDED_
#define DSI_HTC_UTIL_H_INCLUDED_

/*
 * htc display attribute list
 */
struct htc_class_attribute {
	struct class_attribute attr;
	u32 req_value;     /* storage display attribute request value*/
	u32 cur_value;     /* storage display attribute current value*/
	bool (*const set_function)(u32 val); /* return true on success then apply the req_value */
};

ssize_t htc_generic_attr_show(struct class *class, struct class_attribute *class_attr, char *buf);
ssize_t htc_generic_attr_store(struct class *class, struct class_attribute *class_attr, const char *buf, size_t count);

#define HTC_CLASS_ATTR(_name, _mode, req_value, cur_value, set_function) \
        struct htc_class_attribute htc_class_attr_##_name = { __ATTR(_name, _mode, htc_generic_attr_show, htc_generic_attr_store), req_value, cur_value, set_function}
#define HTC_CLASS_ATTR_RW(_name, req_value, cur_value, set_function) \
        struct htc_class_attribute htc_class_attr_##_name = { __ATTR(_name, (S_IWUSR | S_IRUGO), htc_generic_attr_show, htc_generic_attr_store), req_value, cur_value, set_function}
#define HTC_CLASS_ATTR_RO(_name, cur_value, set_function) \
        struct htc_class_attribute htc_class_attr_##_name = { __ATTR(_name, (S_IRUGO), htc_generic_attr_show, NULL), cur_value, cur_value, set_function}

#endif // DSI_HTC_UTIL_H_INCLUDED_

