/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
//HTC_START, Use vendor poll method
#include "PhoneUpdate.h"
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>

DEFINE_MSM_MUTEX(msm_ois_mutex);
static struct ois_timer ois_timer_t;
//HTC_END


static int32_t msm_ois_get_status(struct cam_ois_ctrl_t *o_ctrl)
{
    uint8_t buf_status[8] = { 0 };
    uint8_t Tri_state = 0;
    int32_t rc = 0;


    rc = cam_camera_cci_i2c_read_seq(o_ctrl->io_master_info.cci_client, 0xF120, &buf_status[0], 
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, 4);
	if (rc != 0) {
		pr_err("[OISDBG] %s : i2c_read_seq fail.\n", __func__);
	} else {
        if (buf_status[2] & 0x40)//1000000
        {
            //CAM_ERR(CAM_OIS, "[OISDBG]Bit[14]: Tripod state = 1");
            Tri_state = 1;
        }
        else
        {
            //CAM_ERR(CAM_OIS, "[OISDBG]Bit[14]: Tripod state = 0");
            Tri_state = 0;
        }
    }
    return Tri_state;
}

/*ioctl from userspace.
*Get ois gyro readout data from ring buffer.
*/
static int32_t msm_ois_get_gyro(struct cam_ois_ctrl_t *o_ctrl, struct cam_packet *csl_packet)
{
	struct msm_ois_readout gyro_data[MAX_GYRO_QUERY_SIZE];
	int i;
	int rc = 0;
	uint64_t              buf_addr;
	size_t                buf_size;
	struct cam_buf_io_cfg *io_cfg;
	uint8_t               *read_buffer;

	#if 0
	uint8_t query_size = gyro->query_size <= MAX_GYRO_QUERY_SIZE ?
		gyro->query_size : MAX_GYRO_QUERY_SIZE;
	#else
	int query_size = MAX_GYRO_QUERY_SIZE;
	#endif

	uint8_t data_get_count = 0;
	uint16_t counter = 0;

	memset(gyro_data, 0, sizeof(gyro_data));

	mutex_lock(&ois_gyro_mutex);

	if (o_ctrl->buf.buffer_tail < MSM_OIS_DATA_BUFFER_SIZE &&
		o_ctrl->buf.buffer_tail != o_ctrl->buf.buffer_head) {
		for (counter = 0; counter < query_size; counter++) {
			gyro_data[counter].ois_x_shift =
				o_ctrl->buf.buffer[o_ctrl->buf.buffer_head]
				.ois_x_shift;
			gyro_data[counter].ois_y_shift =
				o_ctrl->buf.buffer[o_ctrl->buf.buffer_head]
				.ois_y_shift;
			gyro_data[counter].readout_time =
				o_ctrl->buf.buffer[o_ctrl->buf.buffer_head]
				.readout_time;

#if 0
			CAM_INFO(CAM_OIS, "[OISDBG][Get] readout_time = %lld, x_shift = %d, y_shift = %d",
				gyro_data[counter].readout_time,
				gyro_data[counter].ois_x_shift,
				gyro_data[counter].ois_y_shift);
#endif
			o_ctrl->buf.buffer_head++;
			data_get_count++;

			if (o_ctrl->buf.buffer_head >=
				MSM_OIS_DATA_BUFFER_SIZE)
				o_ctrl->buf.buffer_head -=
					MSM_OIS_DATA_BUFFER_SIZE;

			if (o_ctrl->buf.buffer_head ==
				o_ctrl->buf.buffer_tail) {
				//ois_pr_dbg("[OISDBG]:%s head == tail\n",
					//__func__);
				break;
			}
		}
	}
	if (data_get_count != 0 && data_get_count <
		MAX_GYRO_QUERY_SIZE + 1) {

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		for (i = 0; i < csl_packet->num_io_configs; i++) {
			//CAM_ERR(CAM_OIS, "[OISDBG]Direction: %d:", io_cfg->direction);
			if (io_cfg->direction == CAM_BUF_OUTPUT) {
				rc = cam_mem_get_cpu_buf(io_cfg->mem_handle[0],
					(uint64_t *)&buf_addr, &buf_size);
				//CAM_ERR(CAM_OIS, "[OISDBG]buf_addr : %pK, buf_size : %zu\n",
					//(void *)buf_addr, buf_size);

				read_buffer = (uint8_t *)buf_addr;
				if (!read_buffer) {
					CAM_ERR(CAM_OIS,
						"[OISDBG]invalid buffer to copy data");
					mutex_unlock(&ois_gyro_mutex);
					return -EINVAL;
				}
				read_buffer += io_cfg->offsets[0];
#if 0
				if (buf_size < e_ctrl->cal_data.num_data) {
					CAM_ERR(CAM_OIS,
						"[OISDBG]failed to copy, Invalid size");
					return -EINVAL;
				}
#endif
				memcpy(read_buffer, gyro_data, data_get_count * sizeof(struct msm_ois_readout));

			} else {
				CAM_ERR(CAM_EEPROM, "Invalid direction");
				rc = -EINVAL;
			}
		}
	}
	//CAM_ERR(CAM_OIS, "[OISDBG][Get]count = %d", data_get_count);
	mutex_unlock(&ois_gyro_mutex);
	if(rc != 0)
	{
		CAM_ERR(CAM_OIS, "[OISDBG][Get]rc = %d", rc);
		data_get_count = rc;
	}
	return data_get_count;
}


int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


//HTC_START
int msm_stopGyroThread(void)
{
	CAM_INFO(CAM_OIS, "[OISDBG] E");
	if ((ois_timer_t.ois_timer_state == OIS_TIME_ACTIVE) ||
		(ois_timer_t.ois_timer_state == OIS_TIME_ERROR)) {
		pr_info("[OISDBG] %s:timer cancel.\n", __func__);
		hrtimer_cancel(&ois_timer_t.hr_timer);
		destroy_workqueue(ois_timer_t.ois_wq);
		ois_timer_t.ois_timer_state = OIS_TIME_INACTIVE;
	} else
		pr_err("[OISDBG] invalid timer state = %d\n",
			ois_timer_t.ois_timer_state);
	CAM_INFO(CAM_OIS, "[OISDBG] X");
	return 0;
}

/*Enqueue ois gyro readout data via i2c command.*/
bool msm_ois_data_enqueue(int64_t readout_time,
						int16_t x_shift,
						int16_t y_shift,
					struct msm_ois_readout_buffer *o_buf)
{
	bool rc;

	mutex_lock(&ois_gyro_mutex);

	if (o_buf->buffer_tail >= 0 && o_buf->buffer_tail <
		MSM_OIS_DATA_BUFFER_SIZE) {
		o_buf->buffer[o_buf->buffer_tail].ois_x_shift = x_shift;
		o_buf->buffer[o_buf->buffer_tail].ois_y_shift = y_shift;
		o_buf->buffer[o_buf->buffer_tail].readout_time = readout_time;

	#if 0
	CAM_INFO(CAM_OIS, "[OISDBG][EnQ] readout_time = %lld, x_shift = %d, y_shift = %d",
		o_buf->buffer[o_buf->buffer_tail].readout_time,
		o_buf->buffer[o_buf->buffer_tail].ois_x_shift,
		o_buf->buffer[o_buf->buffer_tail].ois_y_shift);
	#endif

		o_buf->buffer_tail++;
		if (o_buf->buffer_tail >= MSM_OIS_DATA_BUFFER_SIZE)
			o_buf->buffer_tail -= MSM_OIS_DATA_BUFFER_SIZE;

		rc = true;

	} else {
		rc = false;
	}
	mutex_unlock(&ois_gyro_mutex);
	return rc;
}

/*Get OIS gyro data via i2c.*/
static void msm_ois_read_work(struct work_struct *work)
{
	uint8_t buf[8] = { 0 };
	int32_t rc = 0;
	int16_t x_shift, y_shift;
	struct timespec ts;
	int64_t readout_time;
	bool result;
	struct ois_timer *ois_timer_in_t;
	//CAM_ERR(CAM_OIS, "[OISDBG] %s ", __func__);

	/*
	struct sched_param param = { .sched_priority = 75 };
	sched_setscheduler(current, SCHED_FIFO, &param);
	*/
	ois_timer_in_t = container_of(work, struct ois_timer, g_work);
	get_monotonic_boottime(&ts);

	rc = cam_camera_cci_i2c_read_seq(ois_timer_in_t->o_ctrl->io_master_info.cci_client, 0xE001, &buf[0], 
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, 8);

	if (rc != 0) {
		ois_timer_t.i2c_fail_count++;
		pr_err("[OISDBG] %s : i2c_read_seq fail. cnt = %d\n",
			__func__, ois_timer_t.i2c_fail_count);
		if (ois_timer_t.i2c_fail_count == MAX_FAIL_CNT) {
			pr_err("[OISDBG] %s : Too many i2c failed. Stop timer.\n",
				__func__);
			ois_timer_t.ois_timer_state = OIS_TIME_ERROR;
		}
	} else {
		ois_timer_t.i2c_fail_count = 0;
		readout_time = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;

		x_shift = (int16_t)(((uint16_t)buf[2] << 8) + (uint16_t)buf[3]);
		y_shift = (int16_t)(((uint16_t)buf[4] << 8) + (uint16_t)buf[5]);
		//CAM_ERR(CAM_OIS, "[OISDBG][E001] readout_time = %lld, x_shift = %d, y_shift = %d", readout_time, x_shift, y_shift);
		//CAM_ERR(CAM_OIS, "[OISDBG][E001] readout_time = %lld, %02X%02X%02X%02X%02X%02X", readout_time, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		//CAM_ERR(CAM_OIS, "[OISDBG][E001] readout_time = %lld, %02X%02X%02X%02X%02X%02X%02X%02X",
            //readout_time, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

		result = msm_ois_data_enqueue(readout_time, x_shift,
			y_shift, &ois_timer_in_t->o_ctrl->buf);
		if (!result)
			pr_err("%s %d ois data enqueue ring buffer failed\n",
				__func__, __LINE__);
	}
}

static enum hrtimer_restart msm_gyro_timer(struct hrtimer *timer)
{
	ktime_t currtime, interval;
	struct ois_timer *ois_timer_in_t;

	ois_timer_in_t = container_of(timer, struct ois_timer, hr_timer);
    //CAM_ERR(CAM_OIS, "[OISDBG] %s, cam_ois_state = %d ", __func__, ois_timer_in_t->o_ctrl->cam_ois_state);
	if ((ois_timer_in_t->o_ctrl->cam_ois_state == CAM_OIS_START)
		&& (ois_timer_t.ois_timer_state != OIS_TIME_ERROR)) {
		queue_work(ois_timer_in_t->ois_wq, &ois_timer_in_t->g_work);
		currtime  = ktime_get();
		interval = ktime_set(0, READ_OUT_TIME);
		hrtimer_forward(timer, currtime, interval);

		return HRTIMER_RESTART;
	} else {
		CAM_ERR(CAM_OIS, "[OISDBG] %s HRTIMER_NORESTART\n", __func__);
		return HRTIMER_NORESTART;
	}
}


int msm_startGyroThread(struct cam_ois_ctrl_t *o_ctrl)
{
	ktime_t  ktime;

	pr_info("[OISDBG] %s:E\n", __func__);
	if (ois_timer_t.ois_timer_state == OIS_TIME_ERROR) {
		pr_err("[OISDBG] %s:Timer error, close befoe create :%d.\n",
			__func__, ois_timer_t.ois_timer_state);
		msm_stopGyroThread();
	}
	ois_timer_t.i2c_fail_count = 0;
	if (ois_timer_t.ois_timer_state != OIS_TIME_ACTIVE) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			I2C_FAST_PLUS_MODE;
		ois_timer_t.o_ctrl = o_ctrl;
		INIT_WORK(&ois_timer_t.g_work, msm_ois_read_work);
		ois_timer_t.ois_wq = create_workqueue("ois_wq");
		if (!ois_timer_t.ois_wq) {
			pr_err("[OISDBG]:%s ois_wq create failed.\n", __func__);
			return -EFAULT;
		}
		ktime = ktime_set(0, READ_OUT_TIME);
		hrtimer_init(&ois_timer_t.hr_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
		ois_timer_t.hr_timer.function = &msm_gyro_timer;
		hrtimer_start(&ois_timer_t.hr_timer, ktime,
			HRTIMER_MODE_REL);
		ois_timer_t.ois_timer_state = OIS_TIME_ACTIVE;
	} else
		pr_err("[OISDBG] invalid timer state = %d.\n",
			ois_timer_t.ois_timer_state);
	pr_info("[OISDBG] %s:X\n", __func__);
	return 0;
}

#include "lc898123F40_htc.h"
int htc_ois_calibration(struct cam_ois_ctrl_t *o_ctrl, int cam_id)
{
	int rc = -1;
	CAM_INFO(CAM_OIS, "[OIS_Cali] E cam_id=%d", cam_id);


	/*GYRO calibration*/
	rc = htc_ext_GyroReCalib(&o_ctrl->io_master_info, cam_id);
	if (rc != 0)
		CAM_ERR(CAM_OIS, "[OIS_Cali]htc_GyroReCalib fail. rc=%d", rc);
	else
	{
		rc = htc_ext_WrGyroOffsetData();
		if (rc != 0)
			CAM_ERR(CAM_OIS, "[OIS_Cali]htc_WrGyroOffsetData fail.\n");
		else
			CAM_INFO(CAM_OIS, "[OIS_Cali]Gyro calibration success.");
		CAM_INFO(CAM_OIS, "[OIS_Cali]htc_GyroReCalib OK");
	}

	return rc;
}
//HTC_END

/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, (void __user *) cmd->handle,
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user((void __user *) cmd->handle, &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc)
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
//HTC_START, Use vendor poll method
#if 0
//HTC_END
	uint32_t i, size;
//HTC_START, Use vendor poll method
#else
	uint8_t UcSndDat = 0;
	uint32_t UlStCnt = 0;
	uint32_t CNT100MS = 1352;
#endif
//HTC_END

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
//HTC_START don't apply empty setting or return failed.
		if (i2c_list->i2c_settings.size == 1 &&
			i2c_list->i2c_settings.reg_setting[0].reg_addr == 0 &&
			i2c_list->i2c_settings.reg_setting[0].reg_data == 0) {
			CAM_ERR(CAM_OIS, "Got an empty setting, skip it");
			continue;
		}
//HTC_END
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
//HTC_START, Use vendor poll method
#if 0
//HTC_END
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
//HTC_START, Use vendor poll method
#else
			do {
				UcSndDat = F40_RdStatus(1);
			} while (UcSndDat != 0 && (UlStCnt++ < CNT100MS ));
			if(UcSndDat == 0)
				CAM_ERR(CAM_OIS,"[CAM_OIS]Poll success, UcSndDat = %d", UcSndDat);
			else
				CAM_ERR(CAM_OIS,"[CAM_OIS]Poll fail, UcSndDat = %d", UcSndDat);
#endif
//HTC_END
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, 32);
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)(
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	page = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)(
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	release_firmware(fw);

	return rc;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uint64_t                        generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uint64_t                        generic_pkt_addr;
	size_t                          pkt_len;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config, (void __user *) ioctl_ctrl->handle,
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		(uint64_t *)&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	if (dev_config.offset > pkt_len) {
		CAM_ERR(CAM_OIS,
			"offset is out of bound: off: %lld len: %zu",
			dev_config.offset, pkt_len);
		return -EINVAL;
	}

	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + dev_config.offset);
	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				(uint64_t *)&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf");
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				return -EINVAL;
			}
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					return rc;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					return rc;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					return rc;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					return rc;
				}
			}
			break;
			}
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				return rc;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply Init settings");
			goto pwr_dwn;
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0)
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
		break;
//HTC_START
	case CAM_OIS_PACKET_OPCODE_OIS_CALIBRATION:
		{
			int32_t *cam_id = NULL, *return_code = NULL;

			offset = (uint32_t *)&csl_packet->payload;
			offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
			cmd_desc = (struct cam_cmd_buf_desc *)(offset);
			rc = cam_mem_get_cpu_buf(cmd_desc[0].mem_handle, (uint64_t *)&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "[OIS_Cali] OIS calibration failed to get cpu buf");
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			cmd_buf += cmd_desc->offset / sizeof(uint32_t);
			cam_id = (int32_t *)cmd_buf;
			return_code = (int32_t *)(cmd_buf + 1);
			*return_code = htc_ois_calibration(o_ctrl, *cam_id);
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_READ_START:
		{
			int32_t *return_code = NULL;
			CAM_INFO(CAM_OIS, "[OISDBG]:CAM_OIS_PACKET_OPCODE_OIS_READ_START");

			offset = (uint32_t *)&csl_packet->payload;
			offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
			cmd_desc = (struct cam_cmd_buf_desc *)(offset);
			rc = cam_mem_get_cpu_buf(cmd_desc[0].mem_handle, (uint64_t *)&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "[OISDBG] OIS calibration failed to get cpu buf");
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			cmd_buf += cmd_desc->offset / sizeof(uint32_t);
			return_code = (int32_t *)cmd_buf;
			*return_code = msm_startGyroThread(o_ctrl);
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_READ_END:
		{
			int32_t *return_code = NULL;
			CAM_INFO(CAM_OIS, "[OISDBG]:CAM_OIS_PACKET_OPCODE_OIS_READ_END");

			offset = (uint32_t *)&csl_packet->payload;
			offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
			cmd_desc = (struct cam_cmd_buf_desc *)(offset);
			rc = cam_mem_get_cpu_buf(cmd_desc[0].mem_handle, (uint64_t *)&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "[OISDBG] OIS calibration failed to get cpu buf");
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			cmd_buf += cmd_desc->offset / sizeof(uint32_t);
			return_code = (int32_t *)cmd_buf;
			*return_code = msm_stopGyroThread();
		}
		break;
     case CAM_OIS_PACKET_OPCODE_OIS_GET_DATA:
		{
			int32_t *query_size = NULL;
			int32_t *OIS_status = NULL;

			offset = (uint32_t *)&csl_packet->payload;
			offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
			cmd_desc = (struct cam_cmd_buf_desc *)(offset);
			rc = cam_mem_get_cpu_buf(cmd_desc[0].mem_handle, (uint64_t *)&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "[OISDBG] OIS calibration failed to get cpu buf");
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			cmd_buf += cmd_desc->offset / sizeof(uint32_t);
			query_size = (int32_t *)cmd_buf;
			OIS_status = (int32_t *)(cmd_buf + 1);

			*query_size = msm_ois_get_gyro(o_ctrl, csl_packet);
			*OIS_status = msm_ois_get_status(o_ctrl);
		}
		break;
//HTC_END

	default:
		break;
	}
	return rc;
pwr_dwn:
	cam_ois_power_down(o_ctrl);
	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user((void __user *) cmd->handle,
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
