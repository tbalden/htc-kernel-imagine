/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/msm_htc_util.h>
#include <linux/debugfs.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ESD_DETECTION
#include <linux/msm_drm_notify.h>
#endif
#include <video/mipi_display.h>
#include "dsi_panel.h"
#include "dsi_ctrl.h"
#include "dsi_display.h"
#include "dsi_htc_util.h"
#include "sde_trace.h"

#define UNDEF_VALUE -1U
#define BL_CALI_DEFAULT  10000
#define BL_CALI_MAX      20000
#define BRI_GAIN_CHECK(x) (x>0 && x<=20000)
#define BACKLIGHT_CALI(ori,comp) ((unsigned int)(ori*comp/BL_CALI_DEFAULT))
#define DEFAUTL_AP_LEVLE 142
#define BACKLIGHT_GAIN_RES    "/persist/display/bl_gain"

extern int htc_dsi_panel_parse_dcs_cmds_from_buf(char *buf, int length,
		struct dsi_panel_cmd_set *cmd);
extern int htc_mipi_dsi_dcs_read(struct dsi_panel *panel, u8 cmd, void *data, int len);

static int android_get_supported_color_modes(char *buf, size_t size, bool detail);
static int htc_parse_bl_gain_from_file(const char *filename);
static int dsi_panel_parse_dcs_cmds_from_file(const char *filename, struct dsi_panel_cmd_set *pcmds);

struct dimming_work {
	struct dsi_panel *panel;
	struct delayed_work dimming_on_work;
	bool dimming_on;
};

static struct dimming_work dimming_work;
static void dimming_do_work(struct work_struct *work);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ESD_DETECTION
	typedef struct {
		struct notifier_block drm_notifier;
		bool panel_on;
	}drm_event;

	static drm_event esd_checker;

	extern void lcm_rst_callback_register( void (*lcm_rst_func)(void) );
#endif

/* color mode id must be the same as display hal
 mapping table
 -----------------------------------------------
 android_color_mode_t |   dsi_cmd_set_type
 -----------------------------------------------
 DDIC_COLOR_NATIVE   -->  DSI_CMD_SET_DDIC_COLOR_NATIVE
 DDIC_COLOR_SRGB     -->  DSI_CMD_SET_DDIC_COLOR_SRGB
 DDIC_COLOR_DIC_P3   -->  DSI_CMD_SET_DDIC_COLOR_DCI_P3
 */
typedef enum {
	DDIC_COLOR_NATIVE = 0,
	DDIC_COLOR_SRGB = 7,
	DDIC_COLOR_DCI_P3 = 9,
} android_color_mode_t;

struct ddic_color_mode_info{
	android_color_mode_t color_mode_id;
	enum dsi_cmd_set_type dsi_cmd_id;
	const char* resource_path;
	bool color_mode_supported;
	struct dsi_panel_cmd_set color_cmds;
};

static struct ddic_color_mode_info htc_color_mode_info[] =
{
	{DDIC_COLOR_NATIVE, DSI_CMD_SET_DDIC_COLOR_NATIVE, "/persist/display/ddic_color_native", false, {0}},
	{DDIC_COLOR_SRGB, DSI_CMD_SET_DDIC_COLOR_SRGB, "/persist/display/ddic_color_srgb", false, {0}},
	{DDIC_COLOR_DCI_P3, DSI_CMD_SET_DDIC_COLOR_DCI_P3, "/persist/display/ddic_color_dci_p3", false, {0}}
};

static struct dsi_panel *g_panel;
static struct class *htc_display_class;
static LIST_HEAD(g_dsi_notifier_list);
static DEFINE_MUTEX(dsi_notify_sem);

static ssize_t get_color_modes_show(struct class *dev,
        struct class_attribute *attr, char *buf)
{
	 return android_get_supported_color_modes(buf, PAGE_SIZE, false);
}
static CLASS_ATTR_RO(get_color_modes);

static ssize_t parse_bl_gain_show(struct class *dev,
        struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", htc_parse_bl_gain_from_file(BACKLIGHT_GAIN_RES));
}
static CLASS_ATTR_RO(parse_bl_gain);
/*
 * value:1 --> apply to all nodes of bl_table
 * value:0 --> no apply bl calibration to default AP level,
 * and others node apply BL_CALI_DATA.
 */
static HTC_CLASS_ATTR_RW(bklt_cali_enable, 0, 0, NULL);

/*
 * value: BL_CALI_DATA range is 1 ~ 20000. default is 10000
 * Can use BACKLIGHT_CALI marco for new BL with calibration effect.
 */
static HTC_CLASS_ATTR_RW(bklt_cali_data, 10000, 10000, NULL);

/*
 * value: boolean flag for hal display check
 * if need to enable hal color feature.
 * read only. modify only by device node "htc,hal-color-enable"
 */
static HTC_CLASS_ATTR_RO(hal_color_enable, 0, NULL);

/*
 * value:  android color mode ,range is 0 - 10.
 * color mode information can refer to android_color_mode
 */
static HTC_CLASS_ATTR_RW(ddic_color_mode_ctrl, 0, 0, panel_set_color_mode);

/*
 * value: auto brightness level from hal lights
 * if value >= bl_config.burst_on_level, then enable burst mode else disable burst mode
 */
static HTC_CLASS_ATTR_RW(bklt_burst_switch, 0, 0, NULL);


const static struct class_attribute* const class_attr_list[] =
{
	&class_attr_parse_bl_gain,
	&htc_class_attr_bklt_cali_enable.attr,
	&htc_class_attr_bklt_cali_data.attr,
	&htc_class_attr_hal_color_enable.attr,
	&htc_class_attr_bklt_burst_switch.attr,

};

const static struct class_attribute* const ddic_color_class_attr_list[] =
{
	&class_attr_get_color_modes,
	&htc_class_attr_ddic_color_mode_ctrl.attr,
};

int dsi_register_notifier(struct dsi_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func) {
		pr_err("%s: invalid parameter\n",__func__);
		return -EINVAL;
	}

	mutex_lock(&dsi_notify_sem);
	list_add(&notifier->dsi_notifier_link,
		&g_dsi_notifier_list);
	mutex_unlock(&dsi_notify_sem);
	return 0;
}
EXPORT_SYMBOL(dsi_register_notifier);

void send_dsi_status_notify(int status)
{
	static struct dsi_status_notifier *dsi_notifier;
	SDE_ATRACE_BEGIN(__func__);

	mutex_lock(&dsi_notify_sem);
	list_for_each_entry(dsi_notifier,
		&g_dsi_notifier_list,
		dsi_notifier_link) {
		dsi_notifier->func(status);
	}
	mutex_unlock(&dsi_notify_sem);
	pr_info("%s: status=%d\n",__func__, status);
	SDE_ATRACE_END(__func__);
}

ssize_t htc_generic_attr_show(struct class *class,
        struct class_attribute *class_attr, char *buf)
{
	struct htc_class_attribute* htc_attr = container_of(class_attr, struct htc_class_attribute, attr);

	return scnprintf(buf, PAGE_SIZE, "%d\n", htc_attr->cur_value);
}
EXPORT_SYMBOL_GPL(htc_generic_attr_show);

ssize_t htc_generic_attr_store(struct class *class, struct class_attribute *class_attr,
        const char *buf, size_t count)
{
	u32 res;
	int rc;
	struct htc_class_attribute* htc_attr;

	rc = kstrtou32(buf, 10, &res);
	if (rc) {
		pr_err("invalid parameter, %s %d\n", buf, rc);
		return -EINVAL;
	}

	htc_attr = container_of(class_attr, struct htc_class_attribute, attr);

	htc_attr->req_value = res;
	if (htc_attr->set_function &&
	    htc_attr->cur_value != htc_attr->req_value) {
		if (htc_attr->set_function(res)) {
			htc_attr->cur_value = htc_attr->req_value;
		} else {
			count = -ENOSYS;
		}
	}

	return count;
}
EXPORT_SYMBOL_GPL(htc_generic_attr_store);

/*
 * Display parse color mode follow.
 * 1. hal read kernel file node in bootup, which will run API:android_get_supported_color_modes
 *    and do corresponding read ddic color and bl gain from resource.
 *
 * Display set color mode follow.
 * 1. display hal set color mode to kernel file node
 * 2. remap android color mode id to dsi_cmd_set_type
 * 3. htc_dsi_panel_tx_cmd_set with dsi_cmd_set
 */

static int android_get_supported_color_modes(char *buf, size_t size, bool detail)
{

	int i;
	int len = 0;

	if (!g_panel) {
		pr_err("%s: Invalid argument\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(htc_color_mode_info); ++i) {
		if (htc_color_mode_info[i].color_mode_supported) {
			int rc;

			len += scnprintf(buf + len, size - len, "%d ",
					htc_color_mode_info[i].color_mode_id);

			rc = dsi_panel_parse_dcs_cmds_from_file(htc_color_mode_info[i].resource_path,
					&htc_color_mode_info[i].color_cmds);
			if (rc) {
				pr_warn("Failed to parse color mode '%d' calibration data from path %s\n",
						htc_color_mode_info[i].color_mode_id, htc_color_mode_info[i].resource_path);
			}

			if (detail) {
				len += scnprintf(buf + len, size - len, "file path:%s parse:%s\n",
						htc_color_mode_info[i].resource_path, rc ? "NG" : "OK");
			}
		}
	}


	return len;
}

static void android_set_supported_color_modes(int mode){
	int i = 0;

	for (i = 0; i< ARRAY_SIZE(htc_color_mode_info); i++){
		if (htc_color_mode_info[i].color_mode_id == mode){
			htc_color_mode_info[i].color_mode_supported = true;
			if (g_panel)
				g_panel->support_ddic_color_mode = true;

			break;
		}
	}
}

static int android_colormode_id_to_dsicmd_id(android_color_mode_t id, struct dsi_panel_cmd_set **color_cmds){

	int i;

	for (i = 0; i< ARRAY_SIZE(htc_color_mode_info); i++){
		if (id == htc_color_mode_info[i].color_mode_id){
			if (htc_color_mode_info[i].color_cmds.count && htc_color_mode_info[i].color_cmds.cmds)
				*color_cmds = &htc_color_mode_info[i].color_cmds;

			return (int)htc_color_mode_info[i].dsi_cmd_id;
		}
	}

	return -1;
}

static int htc_dsi_panel_tx_cmd_set(struct dsi_panel *panel, struct dsi_panel_cmd_set *dsi_cmds)
{
	int rc = 0, i = 0;
	ssize_t len;
	struct dsi_cmd_desc *cmds;
	u32 count;
	enum dsi_cmd_set_state state;

	const struct mipi_dsi_host_ops *ops = panel->host->ops;

	if (!panel || !dsi_cmds)
		return -EINVAL;

	cmds = dsi_cmds->cmds;
	count = dsi_cmds->count;
	state = dsi_cmds->state;

	if (count == 0) {
		pr_debug("[%s] No commands to be sent \n", __func__);
		rc = -ENOENT;
		goto error;
	}

	for (i = 0; i < count; i++) {
		/* TODO:  handle last command */
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		len = ops->transfer(panel->host, &cmds->msg);
		if (len < 0) {
			rc = len;
			pr_err("failed to set cmds, rc=%d\n", rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms * 1000,
					cmds->post_wait_ms * 1000 + 50);
		cmds++;
	}
error:
	return rc;
}

/*
@Param buf_in :hex string source
@Param buf_out: byte buffer of parsing result
@Param buf_out_max_size: max size of byte buffer
@Param but_out_len: effect size of byte buffer.
*/
static void parse_hex_string_to_byte_buf(char *buf_in, char *buf_out,
		int buf_out_max_size, int *but_out_len)
{

	char *token,*temp_buf;
	int len = 0;
	unsigned int val;

	if (!buf_in || !buf_out || !buf_out_max_size || !but_out_len){
		pr_err("input argument check fail");
		return;
	}

	temp_buf = buf_in;

	pr_debug("%s, ---------- parsing DCS command start ---------- \n", __func__ );
	while((len < buf_out_max_size) &&
		((token = strsep(&temp_buf, " ")) != NULL)){
		if (sscanf(token, "%02x", &val) == 1){
			buf_out[len] = val;
			len++;
			pr_debug("%s, parse byte[%d]:%x\n", __func__, len, val);
		}
	}
	*but_out_len = len;
	pr_debug("%s, ---------- parsing DCS command end ---------- \n", __func__ );

}

static int htc_parse_bl_gain_from_file(const char *filename)
{

	int size, rc = 0;
	struct file *file = NULL;
	char *buf = NULL;
	unsigned long val;

	file = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(file)) {
		pr_err("%s open file:%s fail\n", __func__, filename);
		return -ENOENT;
	}

	size = i_size_read(file_inode(file));
	if (size <= 0){
		pr_err("%s file:%s size read fail\n", __func__, filename);
		rc = -EIO;
		goto error;
	}
	/* buf is for reading file node */
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		pr_err("unable to allocate memory for buffer data\n");
		rc = -ENOMEM;
		goto error;
	}

	rc = kernel_read(file, 0, buf, size);
	if (rc < 0)
		goto error;

	if (rc != size) {
		rc = -EIO;
		goto error;
	}

	if (sscanf(buf, "%lu\n", &val) == 1) {
		if (BRI_GAIN_CHECK(val)) {
			htc_class_attr_bklt_cali_data.req_value = val;
			pr_info("%s parse bl gain success:%lu\n", __func__, val);
		} else {
			pr_err("%s parse bl gain fail: bl gain out of range:%lu\n", __func__, val);
			rc = -ERANGE;
		}

	} else {
		pr_err("%s parse bl gain fail: mismatch format\n", __func__);
		rc = -EDOM;
	}

error:
	if (buf)
		kfree(buf);

	if (!IS_ERR_OR_NULL(file))
		filp_close(file, NULL);

	return rc < 0 ? rc : 0;
}

static int dsi_panel_parse_dcs_cmds_from_file(const char *filename,
		struct dsi_panel_cmd_set *pcmds)
{

	int size, len, rc = 0;
	struct file *file = NULL;
	char *buf = NULL, *out_buf = NULL;

	file = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(file)) {
		pr_err("%s open file:%s fail\n", __func__, filename);
		return -ENOENT;
	}

	size = i_size_read(file_inode(file));
	if (size <= 0) {
		pr_err("%s file:%s size read fail\n", __func__, filename);
		rc = -EIO;
		goto error;
	}
	/* buf is for reading file node */
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		pr_err("unable to allocate memory for buffer data\n");
		rc = -ENOMEM;
		goto error;
	}
	/* out_buf is for storaging byte data parsed from buf */
	out_buf = kzalloc(size, GFP_KERNEL);
	if (!out_buf) {
		pr_err("unable to allocate memory for out buffer data\n");
		rc = -ENOMEM;
		goto error;
	}

	rc = kernel_read(file, 0, buf, size);
	if (rc < 0)
		goto error;

	if (rc != size) {
		rc = -EIO;
		goto error;
	}

	parse_hex_string_to_byte_buf(buf, out_buf, size, &len);

	rc = htc_dsi_panel_parse_dcs_cmds_from_buf(out_buf, len, pcmds);

	if (rc) {
		pr_err("%s parse dcs cmds error: %d\n", __func__, rc);
	}

error:
	if (buf)
		kfree(buf);

	if (out_buf)
		kfree(out_buf);

	if (!IS_ERR_OR_NULL(file))
		filp_close(file, NULL);

	return rc < 0 ? rc : 0;
}

bool panel_set_color_mode(u32 android_color_mode)
{

	struct dsi_panel_cmd_set *color_cmds = NULL;
	bool result = false;
	int dsi_cmd_id;

	dsi_cmd_id = android_colormode_id_to_dsicmd_id(android_color_mode, &color_cmds);

	if (!g_panel || !g_panel->cur_mode ||
		!g_panel->cur_mode->priv_info ||
		!g_panel->panel_initialized) {
		pr_err("%s: Invalid argument \n", __func__);
		return false;
	}

	if (dsi_cmd_id < 0){
		pr_err("color mode:[%d] do not supported \n", android_color_mode);
		return false;
	}

	if (color_cmds){
		pr_info("use display calibration color mode:[%d]\n", android_color_mode);
	}else{
		pr_info("map android color mode:[%d] to dsi_cmd_id:[%d]\n", android_color_mode, dsi_cmd_id);
		color_cmds = &g_panel->cur_mode->priv_info->cmd_sets[dsi_cmd_id];
	}

	mutex_lock(&g_panel->panel_lock);
	result = htc_dsi_panel_tx_cmd_set(g_panel, color_cmds) >= 0;
	mutex_unlock(&g_panel->panel_lock);

	if (result){
		g_panel->current_color_mode = android_color_mode;
	}

	return result;
}

static ssize_t panel_set_color_mode_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	int rc;
	u32 mode;
	char kBuf[1024];

	count = min(count, ARRAY_SIZE(kBuf) - 1);

	if (copy_from_user(kBuf, buf, count))
		return -EFAULT;
	kBuf[count] = '\0';

	rc = kstrtou32(kBuf, 10, &mode);
	if (rc) {
		pr_err("Invalid color parameter, %s %d\n", buf, rc);
		return -EINVAL;
	}

	return panel_set_color_mode(mode)? count : -ENOSYS;
}

static ssize_t panel_read_color_modes( struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	int len = 0;
	char kBuf[1024];

	if (*ppos) {
		return 0;
	}

	len = android_get_supported_color_modes(kBuf, ARRAY_SIZE(kBuf), true);

	return simple_read_from_buffer(buf, count, ppos, kBuf, len + 1);
}

static const struct file_operations panel_color_mode_fops = {
	.write = panel_set_color_mode_write,
	.read = panel_read_color_modes,
};


static ssize_t htc_mipi_dsi_dcs_write_buffer(struct mipi_dsi_device *dsi,
				  const void *data, size_t len)
{
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_buf = data,
		.tx_len = len,
		.flags = MIPI_DSI_MSG_LASTCOMMAND,
	};

	const struct mipi_dsi_host_ops *ops = dsi->host->ops;

	if (!ops || !ops->transfer)
	return -ENOSYS;

	switch (len) {
	case 0:
		return -EINVAL;

	case 1:
		msg.type = MIPI_DSI_DCS_SHORT_WRITE;
		break;

	case 2:
		msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;

	default:
		msg.type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}

	if (dsi->mode_flags & MIPI_DSI_MODE_LPM)
		msg.flags |= MIPI_DSI_MSG_USE_LPM;

	return ops->transfer(dsi->host, &msg);
}


static ssize_t panel_dsi_write( struct file *file, const char __user *buf,
		size_t count,loff_t *ppos)
{
	int len,rc;
	struct mipi_dsi_device *mipi_dev = NULL;
	char kBuf[1024];
	char payload[255] = {0};
	const char *state;

	count = min(count, ARRAY_SIZE(kBuf) - 1);

	if (copy_from_user(kBuf, buf, count))
		return -EFAULT;
	kBuf[count] = '\0';

	if (!g_panel) {
		pr_err("%s: no global panel data \n", __func__);
		count = -ENODEV;
		goto err_out;
	}

	if (!g_panel->panel_initialized) {
		pr_err("%s: panel is dead \n", __func__);
		count = -ENODEV;
		goto err_out;
	}

	parse_hex_string_to_byte_buf(kBuf, payload, sizeof(payload), &len);

	if (len < 2){
		pr_err("%s input argument is not enough\n", __func__);
		goto err_out;
	}

	mipi_dev = &g_panel->mipi_device;

	/* config MIPI debug interface property */
	state = of_get_property(g_panel->panel_of_node, "qcom,mdss-dsi-default-command-state", NULL);
	if (!state || !strcmp(state, "dsi_lp_mode")) {
		mipi_dev->mode_flags |= MIPI_DSI_MODE_LPM;
	}

	rc = htc_mipi_dsi_dcs_write_buffer(mipi_dev, payload, len);

	if (rc < 0) {
		pr_err("%s send DCS command: return:[%d] fail\n", __func__, rc);
		count = rc;
	} else {
		pr_debug("%s send DCS command: return[%d] \n", __func__, rc);
	}

err_out:
	return count;
}

static const struct file_operations panel_dsi_write_fops = {
	.write = panel_dsi_write,
};

/*
	dcs_read_mesg[0]: address to read
	dcs_read_mesg[1]: desired read count
*/
static u8 dcs_read_mesg[2] = {0};

static ssize_t panel_dsi_read_cmd_set( struct file *file, const char __user *buf,
		size_t count,loff_t *ppos)
{

	unsigned int cmd, read_cout;
	char kBuf[1024];

	count = min(count, ARRAY_SIZE(kBuf) - 1);

	if (copy_from_user(kBuf, buf, count))
		return -EFAULT;
	kBuf[count] = '\0';

	if (sscanf(kBuf, "%x %u", &cmd, &read_cout)!=2) {
		return -EFAULT;
	}

	dcs_read_mesg[0] = (cmd & 0xFF);
	dcs_read_mesg[1] = (read_cout & 0xFF);

	pr_debug("%s dcs_read_mesg->add:%02x, count:%d\n", __func__,
						dcs_read_mesg[0], dcs_read_mesg[1]);

	return count;
}


static ssize_t panel_dsi_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	int i, rc;
	u8 cmd = dcs_read_mesg[0];
	u8 len = dcs_read_mesg[1];

	char read_buf[256] = {0};
	char kBuf[1024];
	int strlen = 0;

	if (*ppos) {
		return 0;
	}

	if (len > ARRAY_SIZE(read_buf))
		return -EINVAL;

	if (!g_panel || !dcs_read_mesg[1]){
		pr_err("%s: no global panel data, or lack of dcs_read_mesg info \n", __func__);
		return -ENODEV;
	}

	if (!g_panel->panel_initialized){
		pr_err("%s: panel is dead \n", __func__);
		return -ENODEV;
	}

	rc = htc_mipi_dsi_dcs_read(g_panel, cmd, read_buf, len);

	if (rc < 0) {
		pr_err("%s read dsi cmd:%02x failed %d\n", __func__, cmd, rc);
		return rc;
	}

	if (!rc) {
		strlen += scnprintf(kBuf + strlen, ARRAY_SIZE(kBuf) - strlen, "%02x: return 0 bytes, please check" \
			"read count size lower than DDIC addres size\n", cmd);
	} else {
		pr_debug("%s read dsi cmd:%02x, return:%d\n", __func__, cmd, rc);
		strlen += scnprintf(kBuf + strlen, ARRAY_SIZE(kBuf) - strlen, "%02x: ", cmd);

		for (i = 0;i < len; i++){
			strlen += scnprintf(kBuf + strlen, ARRAY_SIZE(kBuf) - strlen, "%02x ", read_buf[i]);
		}
	}

	pr_debug("[%s]%s\n", __func__, kBuf);

	return simple_read_from_buffer(buf, count, ppos, kBuf, strlen + 1);
}

static const struct file_operations panel_dsi_read_fops = {
	.read = panel_dsi_read,
	.write = panel_dsi_read_cmd_set,
};

static struct drm_connector *find_drm_connector_from_dsi_panel(struct dsi_panel *panel)
{
	struct drm_device *dev;
	struct drm_connector *connector;
	struct dsi_display *display;

	if (!panel || !panel->host) {
		pr_err("invalid panel\n");
		goto err;
	}

	display = container_of(panel->host, struct dsi_display, host);
	dev = display->drm_dev;
	if (!dev) {
		pr_err("[%s] no drm device!\n", __func__);
		goto err;
	}

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector && connector->connector_type == DRM_MODE_CONNECTOR_DSI)
			return connector;
	}
err:
	return NULL;
}

static void htc_report_panel_dead(void)
{
	struct drm_connector *drm_connector;
	struct drm_event event;
	bool panel_dead = false;

	if (!g_panel) {
		pr_err("invalid panel\n");
		return;
	}

	drm_connector = find_drm_connector_from_dsi_panel(g_panel);
	if (!drm_connector) {
		pr_err("invalid connector\n");
		return;
	}
#ifdef	CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ESD_DETECTION
	if (!esd_checker.panel_on) {
		pr_info("skip esd recovery\n");
		return;
	}
#endif
	pr_err("esd check failed report PANEL_DEAD conn_id: %d\n",
			drm_connector->base.id);
	panel_dead = true;
	event.type = DRM_EVENT_PANEL_DEAD;
	event.length = sizeof(u32);
	msm_mode_object_event_notify(&drm_connector->base,
		drm_connector->dev, &event, (u8 *)&panel_dead);
}

#ifdef	CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ESD_DETECTION
static int drm_notifier_callback(struct notifier_block *self,
					unsigned long event, void *data)
{
	struct msm_drm_notifier *notifier_data = data;
	int next_blank = *(int *)notifier_data->data;

	if ((notifier_data->id != MSM_DRM_PRIMARY_DISPLAY) && event != MSM_DRM_EARLY_EVENT_BLANK)
		return 0;

	esd_checker.panel_on = (next_blank == MSM_DRM_BLANK_SUSPEND || next_blank == MSM_DRM_BLANK_POWERDOWN) ? false : true;
	return 0;
}
#endif

void htc_dsi_parse_esd_params(struct dsi_panel *panel)
{
	if (!panel)
		return;

	if (of_property_read_bool(panel->panel_of_node,
					"htc,esd-status-check")){
#ifdef	CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ESD_DETECTION
		esd_checker.panel_on = false;
		esd_checker.drm_notifier.notifier_call = drm_notifier_callback;
		msm_drm_register_client(&esd_checker.drm_notifier);

		lcm_rst_callback_register(htc_report_panel_dead);
#endif
	} else {
		pr_debug("No valid htc,esd-status-check in DTS \n");
	}
}

static ssize_t panel_dead_write( struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	char kBuf[8];

	count = min(count, ARRAY_SIZE(kBuf) - 1);

	if (copy_from_user(kBuf, buff, count))
		return -EFAULT;

	kBuf[count] = '\0';

	if (kBuf[0] == '1'){
		htc_report_panel_dead();
	}

	return count;
}

static const struct file_operations panel_dead_fops = {
	.write = panel_dead_write,
};

static void htc_create_display_attrs(void)
{
	int i, ret;

	htc_display_class = class_create(THIS_MODULE, "htc_display");
	if (IS_ERR(htc_display_class)){
		ret = PTR_ERR(htc_display_class);
		pr_err("%s: could not allocate htc_display_class, ret = %d\n", __func__, ret);
		return ;
	}

	for (i = 0; i < ARRAY_SIZE(class_attr_list); ++i) {
		ret = class_create_file(htc_display_class, class_attr_list[i]);
		if (ret)
			pr_err("%s class attribute creation failed on %s, ret=%d\n", __func__, class_attr_list[i]->attr.name, ret);
	}

	if (g_panel && g_panel->support_ddic_color_mode){
		for (i = 0; i < ARRAY_SIZE(ddic_color_class_attr_list); ++i) {
			ret = class_create_file(htc_display_class, ddic_color_class_attr_list[i]);
			if (ret)
				pr_err("%s class attribute creation failed on %s, ret=%d\n", __func__, ddic_color_class_attr_list[i]->attr.name, ret);
		}
	}

}

static void htc_debugfs_init(void)
{
	struct dentry *dent = debugfs_create_dir("htc_disp", NULL);

	if (IS_ERR_OR_NULL(dent)) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return;
	}

	if (debugfs_create_file("panel_color_mode_ctrl", 0600, dent, 0, &panel_color_mode_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
	}


	if (debugfs_create_file("dsi_write", 0200, dent, 0, &panel_dsi_write_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
	}

	if (debugfs_create_file("dsi_read", 0600, dent, 0, &panel_dsi_read_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
	}

	if (debugfs_create_file("report_panel_dead", 0200, dent, 0, &panel_dead_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
	}
	return;
}

static void htc_parse_panel_dt(struct dsi_panel *panel)
{

	struct device_node *of_node = panel->panel_of_node;
	int rc = 0;
	u32 val = 0;

	if (of_property_read_bool(of_node,
					"htc,hal-color-enable")) {
		htc_class_attr_hal_color_enable.cur_value =
			htc_class_attr_hal_color_enable.req_value = 1;
	}

	if (of_property_read_bool(of_node,
					"htc,ddic-color-modes-0")){
		android_set_supported_color_modes(0);
	}

	if (of_property_read_bool(of_node,
					"htc,ddic-color-modes-7")){
		android_set_supported_color_modes(7);
	}

	if (of_property_read_bool(of_node,
					"htc,ddic-color-modes-9")){
		android_set_supported_color_modes(9);
	}

	rc = of_property_read_u32(of_node, "htc,burst-on-level",
		&val);
	if (!rc) {
		panel->bl_config.burst_on_level = val;
	}

	rc = of_property_read_u32(of_node, "htc,burst-bl-value",
		&val);
	if (!rc) {
		panel->bl_config.burst_bl_value = val;
	}

}

void check_dsi_panel_data_used(struct dsi_panel *panel )
{

	if (!panel)
		return;

	if (of_property_read_bool(panel->panel_of_node,
					"htc,mdss-dsi-panel-data-used")){
		if (!g_panel){
			g_panel = panel;
			pr_info("use panel data:%s\n", g_panel->name);
			htc_parse_panel_dt(panel);
			htc_debugfs_init();
			htc_create_display_attrs();
		}
	}
}

void check_dsi_panel_data_remove(struct dsi_panel *panel )
{
	if (!panel)
		return;

	if (of_property_read_bool(panel->panel_of_node,
					"htc,mdss-dsi-panel-data-used")) {
		if (g_panel) {
			pr_info("remove panel data:%s\n", g_panel->name);
			g_panel = NULL;
		}
	}
}


void htc_update_bl_cali_data(struct backlight_table_v1_0 *brt_bl_table)
{
	int size = brt_bl_table->size;
	int defaut_level_index = 1, i;
	u16 *bl_data_raw, *bl_data_temp, *bl_ap_level;
	int apply_cali;
	u32 bl_calibration_data;
	struct htc_class_attribute* bl_en_attr = &htc_class_attr_bklt_cali_enable;
	struct htc_class_attribute* bl_data_attr = &htc_class_attr_bklt_cali_data;

	if ((bl_en_attr->cur_value == bl_en_attr->req_value)
		&& (bl_data_attr->cur_value == bl_data_attr->req_value))
		return;

	if (!BRI_GAIN_CHECK(bl_data_attr->req_value)) {
		pr_info("%s bkl=%d out of range\n", __func__, bl_data_attr->req_value);
		return;
	}

	bl_calibration_data = bl_data_attr->cur_value = bl_data_attr->req_value;
	apply_cali = bl_en_attr->cur_value = bl_en_attr->req_value;

	/* Not define brt table */
	if(!size || size < 2 || !brt_bl_table->brt_data || !brt_bl_table->bl_data)
		return;

	memcpy(brt_bl_table->bl_data, brt_bl_table->bl_data_raw, size * sizeof(u16));

	bl_data_raw = brt_bl_table->bl_data_raw;
	bl_data_temp = brt_bl_table->bl_data;
	bl_ap_level = brt_bl_table->brt_data;

	for (i = 0; i < size; i++){
		if (bl_ap_level[i] == DEFAUTL_AP_LEVLE){
			defaut_level_index = i;
			break;
		}
	}

	for (i = 0;i < size; i++){
		/* ap level 142 condition no need to apply calibration gain
		   when calibration apply disable */
		if (!apply_cali && i == defaut_level_index){
			pr_debug("%s apply_cali:%d, ap level:%d,bl raw code:%d," \
			"bl cali code:%d\n",__func__, apply_cali,
			bl_ap_level[i], bl_data_raw[i], bl_data_temp[i]);
			continue;
		}
		bl_data_temp[i] = BACKLIGHT_CALI(bl_data_raw[i], bl_calibration_data);
		pr_debug("%s apply_cali:%d, ap level:%d,bl raw code:%d," \
			"bl cali code:%d\n",__func__, apply_cali,
			bl_ap_level[i], bl_data_raw[i], bl_data_temp[i]);
	}

	/* if bl table no smooth, then set back to original state*/
	for (i = 0; i < size-1; i++){
		if (bl_data_temp[i] > bl_data_temp[i+1]){
			memcpy(bl_data_temp, bl_data_raw, size * sizeof(u16));
			pr_debug("%s fall back to raw bl_table\n", __func__);
			break;
		}
	}
}

void register_dimming_work(struct dsi_panel *panel)
{
	if (!panel)
		return;

	dimming_work.panel = panel;
	dimming_work.dimming_on = false;
	INIT_DELAYED_WORK(&dimming_work.dimming_on_work, dimming_do_work);
}

static void dimming_do_work(struct work_struct *work)
{
	int rc = 0;
	struct dsi_panel_cmd_set *dimming_cmds = NULL;
	struct dsi_panel *panel = dimming_work.panel;

	if (!panel || !panel->cur_mode ||
		!panel->cur_mode->priv_info ||
		!panel->panel_initialized) {

		pr_err("%s: Invalid argument \n", __func__);
		return;
	}

	dimming_cmds = &panel->cur_mode->priv_info->cmd_sets[DSI_CMD_SET_DIMMING_ON];
	mutex_lock(&panel->panel_lock);
	rc = htc_dsi_panel_tx_cmd_set(panel, dimming_cmds);
	mutex_unlock(&panel->panel_lock);

	pr_warn("dimming on %s\n", rc >= 0? "OK":"failed");
}

void htc_dimming_on(void)
{
	struct dsi_panel *panel = dimming_work.panel;

	if (!panel) {
		pr_err("%s no panel \n" , __func__);
		return;
	}

	if (dimming_work.dimming_on)
		return;

	dimming_work.dimming_on = true;
	schedule_delayed_work(&dimming_work.dimming_on_work, msecs_to_jiffies(1000));
}

void htc_dimming_off(void)
{
	struct dsi_panel *panel = dimming_work.panel;

	if (!panel) {
		pr_err("%s no panel \n" , __func__);
		return;
	}
	/* Delete dimming workqueue */
	cancel_delayed_work_sync(&dimming_work.dimming_on_work);
	dimming_work.dimming_on = false;
	pr_debug("dimming off\n");
}

bool htc_is_burst_bl_on(struct dsi_panel *panel)
{

	if (htc_class_attr_bklt_burst_switch.cur_value != htc_class_attr_bklt_burst_switch.req_value){
		htc_class_attr_bklt_burst_switch.cur_value = htc_class_attr_bklt_burst_switch.req_value;
	}

	if (panel->bl_config.burst_on_level &&
		htc_class_attr_bklt_burst_switch.cur_value >= panel->bl_config.burst_on_level){
		return false;
	}

	return false;
}
