/* OIS calibration interface for LC898123 F40
 *
 */

#include "cam_sensor_dev.h"
#include "lc898123F40_htc.h"
//file operation+
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/vmalloc.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
//file operation-

#define SECTOR_UNIT 320
#define SECTOR_DATA 256

static struct camera_io_master *g_io_master_info = NULL;

void RamWrite32A( UINT_16 RamAddr, UINT_32 RamData )
{
//Add 32 bit I2C writing function
	int rc = 0;
	uint8_t data[4] = {0,0,0,0};
	struct camera_io_master *io_master_info = g_io_master_info;
	struct cam_sensor_i2c_reg_setting  i2c_reg_settings;
	struct cam_sensor_i2c_reg_array    i2c_reg_array[4];

	data[0] = (RamData >> 24) & 0xFF;
	data[1] = (RamData >> 16) & 0xFF;
	data[2] = (RamData >> 8)  & 0xFF;
	data[3] = (RamData) & 0xFF;

	i2c_reg_settings.addr_type = 2;
	i2c_reg_settings.data_type = 1;
	i2c_reg_settings.size = 4;

	i2c_reg_array[0].reg_addr = RamAddr;
	i2c_reg_array[0].reg_data = data[0];
	i2c_reg_array[0].delay = 0;

	i2c_reg_array[1].reg_addr = RamAddr;
	i2c_reg_array[1].reg_data = data[1];
	i2c_reg_array[1].delay = 0;

	i2c_reg_array[2].reg_addr = RamAddr;
	i2c_reg_array[2].reg_data = data[2];
	i2c_reg_array[2].delay = 0;

	i2c_reg_array[3].reg_addr = RamAddr;
	i2c_reg_array[3].reg_data = data[3];
	i2c_reg_array[3].delay = 0;

	i2c_reg_settings.reg_setting = &i2c_reg_array[0];

	i2c_reg_settings.delay = 1;

	rc = camera_io_dev_write_continuous(io_master_info, &i2c_reg_settings, 0);

	/*rc = io_master_info->i2c_func_tbl->i2c_write_seq(
		io_master_info, RamAddr, &data[0], 4);*/
	if (rc < 0)
		pr_err("[OIS_Cali] %s : write failed\n", __func__);
}

void RamRead32A( UINT_16 RamAddr, UINT_32 * ReadData )
{
//Add 32 bit I2C writing function
	int rc = 0;
	uint8_t buf[4] = {0,0,0,0};
	struct camera_io_master *io_master_info = g_io_master_info;

	rc = cam_camera_cci_i2c_read_seq(io_master_info->cci_client, RamAddr, &buf[0], 
		    CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, 4);
	/*rc = io_master_info->i2c_func_tbl->i2c_read_seq(
		io_master_info, RamAddr, &buf[0], 4);*/
	if (rc < 0)
		pr_err("[OIS_Cali] %s : read failed\n", __func__);
	else
		*ReadData = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

void WitTim( UINT_16	UsWitTim )
{
	mdelay(UsWitTim);
}

int CntWrt( UINT_8 * PcSetDat, UINT_16 UsDatNum)
{
	int rc = 0;

	struct camera_io_master *io_master_info = g_io_master_info;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;

	uint16_t cnt;
	uint8_t * ptr;
	uint16_t total_bytes = UsDatNum-1;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay= 0 ;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
		kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * total_bytes,
		GFP_KERNEL);
	if (!i2c_reg_setting.reg_setting) {
		pr_err("[OIS_Cali] %s: Failed in allocating i2c_array for fw update", __func__);
		return -ENOMEM;
	}

	//pr_info("[OIS_Cali] PcSetDat0=%d total_bytes=%d i2c_reg_setting.delay=%d io_master_info->master_type=%d", PcSetDat[0], total_bytes, i2c_reg_setting.delay, io_master_info->master_type);

	for (cnt = 0, ptr = (uint8_t *)&(PcSetDat[1]); cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = PcSetDat[0] ;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(io_master_info, &i2c_reg_setting, 1);

	if (rc < 0) {
		pr_err("[OIS_Cali] %s: OIS FW download failed %d", __func__, rc);
	}
	kfree(i2c_reg_setting.reg_setting);

	return rc;
}

int CntRd3( UINT_32 addr, void * PcSetDat, UINT_16 UsDatNum )
{
	int rc = 0;

	/*struct camera_io_master *io_master_info = g_io_master_info;
	rc = cam_camera_cci_i2c_read_seq(io_master_info->cci_client, addr, PcSetDat, 4, UsDatNum);
	//rc = io_master_info->i2c_func_tbl->i2c_read_seq(io_master_info, addr, PcSetDat, UsDatNum);

	if (rc < 0) {
		pr_err("[OIS_Cali] %s:i2c write sequence error:%d\n", __func__, rc);
		return rc;
	}*/
	return rc;
}

void WPBCtrl( UINT_8 UcCtrl )
{
	//do nothing because lc898123F40 uses UnlockCodeSet() to handle WPB by itself
}

unsigned char htc_ext_GyroReCalib(struct camera_io_master *io_master_info, int cam_id)
{
	UINT_8	UcSndDat ;
	struct file*fp = NULL;
	uint8_t *m_path= "/data/misc/camera/GYRO_main_result.txt";
	uint8_t *f_path= "/data/misc/camera/GYRO_front_result.txt";
	char gyro_mem[1024];
	int count = 0;
	stReCalib pReCalib = {0};
	g_io_master_info = io_master_info;
	if (g_io_master_info == NULL)
		return -1;

	//Do gyro offset calibration
	UcSndDat = F40_GyroReCalib(&pReCalib);
	pr_info("[OIS_Cali]%s: %d, pReCalib->SsDiffX = %d (%#x), pReCalib->SsDiffY = %d (%#x)\n", __func__, UcSndDat, pReCalib.SsDiffX, pReCalib.SsDiffX, pReCalib.SsDiffY, pReCalib.SsDiffY);

	//Write calibration result
	if (cam_id == 0)
	{
		fp=msm_fopen (m_path, O_CREAT|O_RDWR|O_TRUNC, 0666);
	} else if (cam_id == 1)
	{
		fp=msm_fopen (f_path, O_CREAT|O_RDWR|O_TRUNC, 0666);
	}else
		pr_info("Can't write result.\n");

	if (fp != NULL)
	{
		count += sprintf(gyro_mem + count,"pReCalib->SsFctryOffX = %d (%#x), pReCalib->SsFctryOffY = %d (%#x) \n", pReCalib.SsFctryOffX, pReCalib.SsFctryOffX, pReCalib.SsFctryOffY, pReCalib.SsFctryOffY);
		count += sprintf(gyro_mem + count,"pReCalib->SsRecalOffX = %d (%#x), pReCalib->SsRecalOffY = %d (%#x) \n", pReCalib.SsRecalOffX, pReCalib.SsRecalOffX, pReCalib.SsRecalOffY, pReCalib.SsRecalOffY);
		count += sprintf(gyro_mem + count,"pReCalib->SsDiffX = %d (%#x), pReCalib->SsDiffY = %d (%#x) \n", pReCalib.SsDiffX, pReCalib.SsDiffX, pReCalib.SsDiffY, pReCalib.SsDiffY);
		msm_fwrite (fp, 0, gyro_mem, strlen(gyro_mem)+1);
		msm_fclose (fp);
	}else
		pr_info("Can't write result.\n");

	if(abs(pReCalib.SsRecalOffX) >= 0x800 || abs(pReCalib.SsRecalOffY) >= 0x800)
	{
		pr_info("[OIS_Cali]%s:Threadhold check failed.\n", __func__);
		return -1;
	}
	else
		return UcSndDat;
}

unsigned char htc_ext_WrGyroOffsetData( void )
{
	UINT_8	ans;
    pr_info("[OIS_Cali]%s: E\n", __func__);
    ans = F40_WrGyroOffsetData();
	return ans;
}

/*
 * version = 0 -> prepare/version 0, read 5 bytes per addr
 * version = 1 -> version 1, read 4 bytes per addr
 */
void htc_ext_FlashSectorRead(struct camera_io_master *io_master_info, unsigned char *data_ptr, UINT_32 address, UINT_32 num_byte)
{
	int i = 0;
	UINT_32 version = 1;
	UINT_32 blk_num = 0;
	g_io_master_info = io_master_info;
	if (g_io_master_info == NULL)
		return;

	//First read to check module version+
/*
	F40_FlashSectorRead_htc(address, data_ptr, 0);
	if(data_ptr[0] != 0xFF)
	{
		version = 0;
		blk_num = num_byte/SECTOR_UNIT;
		pr_err("htc_ext_FlashSectorRead: version = 0, read %d sectors", blk_num);
	}
	else
	{
		version = 1;
		blk_num = num_byte/SECTOR_DATA;
		pr_err("htc_ext_FlashSectorRead: version = 0x%x, read %d sectors", data_ptr[6], blk_num);	//0x1A01 bit[7..0]
	}
	//First read to check module version-
*/
	blk_num = num_byte/SECTOR_DATA;

	for (i = 0; i < blk_num; i++)
	{
		if(version == 0)
			F40_FlashSectorRead_htc_IME(address, data_ptr+SECTOR_UNIT*i, version);
		else
			F40_FlashSectorRead_htc_IME(address, data_ptr+SECTOR_DATA*i, version);
		address += 64;
	}

	//dump data
	//for(i = 0; i < 64*4*blk_num; i+=4)
	//	pr_info("[CAM_PDAF]%s: {0x%x, 0x%x, 0x%x, 0x%x}\n", __func__, data_ptr[i], data_ptr[i+1], data_ptr[i+2], data_ptr[i+3]);
}

void htc_ext_FlashInt32Write(struct camera_io_master *io_master_info, UINT_32 address, UINT_32 data)
{
	g_io_master_info = io_master_info;
	if (g_io_master_info == NULL)
		return;

	// dump write
	pr_info("[EEPROM][write] addr:0x%x data:0x%x\n", address, data);
	F40_FlashInt32Write(address, &data, 1);
}
#define VERNUM 0x04 // LGIT HTC original
#define CALID_CM1 0x00000006 // Calibration ID
#define CALID_CM2 0x00000008 // Calibration ID
int htc_checkFWUpdate(struct camera_io_master *io_master_info)
{
    UINT_8 rc = 0;
    UINT_32 UlFWDat = 0;
    //UINT_32 resultID;
    g_io_master_info = io_master_info;

    pr_info("[OIS_Cali]%s VERNUM = %x\n", __func__, VERNUM);
    WitTim(100);

    RamRead32A(0x8000, &UlFWDat);
    pr_info("[OIS_Cali]%s: FW Ver = %x\n", __func__, UlFWDat);

#if 0 //now disable update fw, after optical confirm, please enable below normal checking version code
    if(0)
#else
    if(((VERNUM&0xFF)>(UlFWDat&0xFF)))
#endif
    {
		//resultID = ReadCalibID();
		//pr_info("[OIS_Cali]%s: resultID = %x, CALID_CM1 = 0x%x, CALID_CM2 = 0x%x\n", __func__, resultID, CALID_CM1, CALID_CM2);

		if((UlFWDat&0xFF00) == 0x0800)
		{
			pr_info("[OIS_Cali]%s: FW update. %x -> %x", __func__, UlFWDat, VERNUM);
			rc = F40_FlashDownload(0, 5, 8);
			if(rc!=0)
				pr_info("[OIS_Cali]%s: FlashUpdate = %d  fail.", __func__, rc);
		}
		else
		{
				pr_info("[OIS_Cali]%s: FW or CALID is invalid", __func__);
		}
    }
    else
        pr_info("[OIS_Cali]%s: no need to update.", __func__);

    pr_info("[OIS_Cali]%s rc = %d\n", __func__, rc);

    return rc;
}

//file operation+
void msm_fclose(struct file* file) {
    filp_close(file, NULL);
}

int msm_fwrite(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

struct file* msm_fopen(const char* path, int flags, int rights) {
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if(IS_ERR(filp)) {
        err = PTR_ERR(filp);
    pr_err("[CAM]File Open Error:%s",path);
        return NULL;
    }
    if(!filp->f_op){
    pr_err("[CAM]File Operation Method Error!!");
    return NULL;
    }

    return filp;
}

int msm_fread(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}
//file operation-
