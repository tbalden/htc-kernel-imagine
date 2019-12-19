#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>

#include "cxd225x.h"

#include <linux/types.h>
#include <linux/htc_flags.h>
#include "cxd225x_mfg.h"

#include <linux/clk.h>

/* Define boot mode for NFC*/
#define NFC_BOOT_MODE_NORMAL 0
#define NFC_BOOT_MODE_FTM 1
#define NFC_BOOT_MODE_DOWNLOAD 2
#define NFC_BOOT_MODE_OFF_MODE_CHARGING 5

#define CXD224X_WAKE_LOCK_TIMEOUT	3		/* wake lock timeout for HOSTINT (sec) */
#define CXD224X_WAKE_LOCK_RF_NFT_TIMEOUT	10		/* wake lock timeout for RF NFT HOSTINT (sec) */
#define CXD224X_WAKE_LOCK_NAME	"cxd224x-i2c"		/* wake lock for HOSTINT */
#define CXD224X_WAKE_LOCK_TIMEOUT_LP	3		/* wake lock timeout for low-power-mode (sec) */
#define CXD224X_WAKE_LOCK_NAME_LP "cxd224x-i2c-lp"	/* wake lock for low-power-mode */

/* do not change below */
#define MAX_BUFFER_SIZE		780

/* Read data */
#define PACKET_HEADER_SIZE_NCI	(3)
#define PACKET_HEADER_SIZE_HCI	(3)
#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)

/* RESET */
#define RESET_ASSERT_MS         (1)

int is_debug = 1;

#define D(x...)	\
	if (is_debug)	\
		printk(KERN_DEBUG "[NFC] " x)
#define I(x...) printk(KERN_INFO "[NFC] " x)
#define E(x...) printk(KERN_ERR "[NFC] " x)
#define CONFIG_CXD224X_NFC_VEN
#define LATCH_ERROR_NO (-110)

struct regulator *pm8998_lvs1;
int nfc_cmd_result;
static int readout_core_reset_ntf = 1;
static int mfc_nfc_cmd_result = 0;

static   unsigned long watchdog_counter;
static   unsigned int watchdogEn;
static   unsigned int watchdog_timeout;
char  NCI_TMP[MAX_BUFFER_SIZE];
char dataresp[MAX_BUFFER_SIZE];
char  FelicaIDm[MAX_BUFFER_SIZE];
#define WATCHDOG_FTM_TIMEOUT_SEC 10

int attr_dbg_value = 0;

static int bcpyrespflag = 0;
static int cpyresplen = 0;
static int edcvalue = 0;

static void cxd224x_hw_reset(void);
static void cxd224x_pon_on(void);
static void cxd224x_pon_off(void);
static void cxd224x_nci_read(void);
static int cxd224x_nci_write(uint8_t *buf, int len);

struct cxd224x_dev {
	struct class	*cxd224x_class;
	struct device	*cxd_dev;
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice cxd224x_device;
	unsigned int en_gpio;	
	unsigned int rst_gpio;	
	unsigned int irq_gpio;	
	unsigned int wake_gpio;	
	unsigned int sony_rfs; // only for debug
	unsigned int clkreq_gpio;
	int boot_mode;
	bool irq_enabled;
	struct mutex lock;
	spinlock_t irq_enabled_lock;
	unsigned int users;
	unsigned int count_irq;
	struct wakeup_source wakelock;	/* wake lock for HOSTINT */
	struct wakeup_source wakelock_lp;	/* wake lock for low-power-mode */
	/* Driver message queue */
	struct workqueue_struct	*wqueue;
	struct work_struct qmsg;
	struct workqueue_struct *p_queue;
	struct work_struct	felica_work;
	/* CLK control */
        bool                    clk_run;
        struct  clk             *s_clk;
	/* read buffer*/
        size_t kbuflen;
        u8 *kbuf;
};

struct cxd224x_dev *cxd224x_info;

void detect_i2c_free(void) {
        struct cxd224x_dev *cxd224x_dev = cxd224x_info;
        I("%s: involve i2c latch recovery\n", __func__);
        queue_work(cxd224x_dev->wqueue, &cxd224x_dev->qmsg);
}
EXPORT_SYMBOL(detect_i2c_free);

/*clock enable function*/
static int nqx_clock_select(struct cxd224x_dev *cxd224x_dev);
/*clock disable function*/
static int nqx_clock_deselect(struct cxd224x_dev *cxd224x_dev);

/******************************************************************************
 *
 *  Function pn544_htc_get_bootmode:
 *  Return  NFC_BOOT_MODE_NORMAL            0
 *          NFC_BOOT_MODE_FTM               1
 *          NFC_BOOT_MODE_DOWNLOAD          2
 *          NFC_BOOT_MODE_OFF_MODE_CHARGING 5
 *  Return 	NFC_BOOT_MODE_NORMAL by default
 *          if there's no bootmode infomation available
 *
 *          Bootmode strig is defined in
 *          bootable/bootloader/lk/app/aboot/aboot.c
 *          bootable/bootloader/lk/app/aboot/htc/htc_board_info_and_setting.c
 *
 ******************************************************************************/
int cxd224x_htc_get_bootmode(void) {
	char sbootmode[30] = "default";
	strcpy(sbootmode,htc_get_bootmode());
	if (strcmp(sbootmode, "offmode_charging") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_OFF_MODE_CHARGING\n",__func__);
		return NFC_BOOT_MODE_OFF_MODE_CHARGING;
	} else if (strcmp(sbootmode, "ftm") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_FTM\n",__func__);
		return NFC_BOOT_MODE_FTM;
	} else if (strcmp(sbootmode, "download") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_DOWNLOAD\n",__func__);
		return NFC_BOOT_MODE_DOWNLOAD;
	} else {
		I("%s: Check bootmode done NFC_BOOT_MODE_NORMAL mode = %s\n",__func__,sbootmode);
		return NFC_BOOT_MODE_NORMAL;
	}
}

#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
static void cxd224x_workqueue(struct work_struct *work)
{
	struct cxd224x_dev *cxd224x_dev = container_of(work, struct cxd224x_dev, qmsg);
	unsigned long flags;

	dev_info(&cxd224x_dev->client->dev, "%s, xrst assert\n", __func__);
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	gpio_set_value(cxd224x_dev->rst_gpio, CXDNFC_RST_ACTIVE);	//for gpio_30 with inverter, trigger reset
	cxd224x_dev->count_irq=0; /* clear irq */
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

	msleep(RESET_ASSERT_MS);
	dev_info(&cxd224x_dev->client->dev, "%s, xrst deassert\n", __func__);
	gpio_set_value(cxd224x_dev->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);	//for gpio_30 with inverter, cancel reset
}

static int __init init_wqueue(struct cxd224x_dev *cxd224x_dev)
{
	INIT_WORK(&cxd224x_dev->qmsg, cxd224x_workqueue);
	cxd224x_dev->wqueue = create_workqueue("cxd224x-i2c_wrokq");
	if (cxd224x_dev->wqueue == NULL)
		return -EBUSY;
	return 0;
}
#endif /* CONFIG_NFC_CXD224X_RST */

static void cxd224x_init_stat(struct cxd224x_dev *cxd224x_dev)
{
	cxd224x_dev->count_irq = 0;
}

static void cxd224x_disable_irq(struct cxd224x_dev *cxd224x_dev)
{
	unsigned long flags;
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->irq_enabled) {
		disable_irq_nosync(cxd224x_dev->client->irq);
		cxd224x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
}

static void cxd224x_enable_irq(struct cxd224x_dev *cxd224x_dev)
{
	unsigned long flags;
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (!cxd224x_dev->irq_enabled) {
		cxd224x_dev->irq_enabled = true;
		enable_irq(cxd224x_dev->client->irq);
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
}

static irqreturn_t cxd224x_dev_irq_handler(int irq, void *dev_id)
{
	struct cxd224x_dev *cxd224x_dev = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	cxd224x_dev->count_irq++;
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
	wake_up(&cxd224x_dev->read_wq);

	return IRQ_HANDLED;
}

static unsigned int cxd224x_dev_poll(struct file *filp, poll_table *wait)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(filp, &cxd224x_dev->read_wq, wait);

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->count_irq > 0)
	{
		cxd224x_dev->count_irq--;
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

	if(mask) 
		__pm_wakeup_event(&cxd224x_dev->wakelock, CXD224X_WAKE_LOCK_TIMEOUT*HZ);

	return mask;
}

static ssize_t cxd224x_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	//unsigned char tmp[MAX_BUFFER_SIZE];
	unsigned char *tmp = NULL;
	int total, len, ret;

	total = 0;
	len = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&cxd224x_dev->read_mutex);

	tmp = cxd224x_dev->kbuf;
        if (!tmp) {
                dev_err(&cxd224x_dev->client->dev,
                        "%s: device doesn't exist anymore\n", __func__);
                return -ENODEV;
        }
        memset(tmp, 0x00, count);

	ret = i2c_master_recv(cxd224x_dev->client, tmp, 3);
	if (ret == 3 && (tmp[0] == 0x00) && (tmp[1] == 0x00) && (tmp[2] == 0x00) ) {
		E("%s : WORKAROUND 000000\n", __func__);
		total = 0;
		ret = 0;
	}
	if (ret == 3 && (tmp[0] != 0xff)) {
		total = ret;

		len = tmp[PACKET_HEADER_SIZE_NCI-1];

		if ( 0x61 == tmp[0] ) {
			__pm_wakeup_event(&cxd224x_dev->wakelock, CXD224X_WAKE_LOCK_RF_NFT_TIMEOUT*HZ);
			I("%s: RF NFT, __pm_wakeup_event(10sec), tmp[0]:%x\n", __func__, tmp[0]);
		}

		/** make sure full packet fits in the buffer **/
		if (len > 0 && (len + total) <= count) {
			/** read the remainder of the packet.
			**/
			ret = i2c_master_recv(cxd224x_dev->client, tmp+total, len);
			if (ret == len)
				total += len;
		}
	} 
	if ( LATCH_ERROR_NO == ret ) {
	    I("%s: sony patch triger\n", __func__);
            queue_work(cxd224x_dev->wqueue, &cxd224x_dev->qmsg);
	}

	mutex_unlock(&cxd224x_dev->read_mutex);

	if (total > count || copy_to_user(buf, tmp, total)) {
		dev_err(&cxd224x_dev->client->dev,
			"[NFC] cxd224x_dev_read failed to copy to user space, total = %d\n", total);
		total = -EFAULT;
	}

	return total;
}

static ssize_t cxd224x_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	//char tmp[MAX_BUFFER_SIZE];
	char *tmp = NULL;
	int ret = 0;

	if (count > MAX_BUFFER_SIZE) {
		dev_err(&cxd224x_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}
	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp)) {
                dev_err(&cxd224x_dev->client->dev, "%s: memdup_user failed\n",
                        __func__);
                ret = PTR_ERR(tmp);
                return ret;
        }

/*	if (copy_from_user(tmp, buf, count)) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to copy from user space\n");
		return -EFAULT;
	}
*/
	mutex_lock(&cxd224x_dev->read_mutex);
	/* Write data */

	ret = i2c_master_send(cxd224x_dev->client, tmp, count);

	if ( LATCH_ERROR_NO == ret ) {
		I("%s: I2C Bus Latch, set ret -EIO\n", __func__);
		ret = -EIO;
	}

	if (ret != count) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to write %d\n", ret);
		ret = -EIO;
	}
	mutex_unlock(&cxd224x_dev->read_mutex);
	kfree(tmp);

	return ret;
}

static int cxd224x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	int call_enable = 0;
	struct cxd224x_dev *cxd224x_dev = container_of(filp->private_data,
                                                       struct cxd224x_dev,
                                                       cxd224x_device);
	filp->private_data = cxd224x_dev;
	mutex_lock(&cxd224x_dev->lock);
	if (!cxd224x_dev->users)
	{
		cxd224x_init_stat(cxd224x_dev);
		call_enable = 1;
	}
	cxd224x_dev->users++;
	mutex_unlock(&cxd224x_dev->lock);
	if (call_enable)
		cxd224x_enable_irq(cxd224x_dev);

	dev_info(&cxd224x_dev->client->dev,
		 "open %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

	return ret;
}

static int cxd224x_dev_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	int call_disable = 0;
	struct cxd224x_dev *cxd224x_dev = filp->private_data;

	mutex_lock(&cxd224x_dev->lock);
	cxd224x_dev->users--;
	if (!cxd224x_dev->users)
	{
		call_disable = 1;
	}
	mutex_unlock(&cxd224x_dev->lock);
	if (call_disable)
		cxd224x_disable_irq(cxd224x_dev);

	dev_info(&cxd224x_dev->client->dev,
		 "release %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

	return ret;
}

static long cxd224x_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;

	switch (cmd) {
	case CXDNFC_RST_CTL:
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
		dev_info(&cxd224x_dev->client->dev, "%s, rst arg=%d\n", __func__, (int)arg);
		return (queue_work(cxd224x_dev->wqueue, &cxd224x_dev->qmsg) ? 0 : 1);
#endif
		break;
	case CXDNFC_POWER_CTL:
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
		if (arg == 0) {
			gpio_set_value(cxd224x_dev->en_gpio, 1);
		} else if (arg == 1) {
			gpio_set_value(cxd224x_dev->en_gpio, 0);  
		} else {
			/* do nothing */
		}
#else
                return 1; /* not support */
#endif
		break;
	case CXDNFC_WAKE_CTL:
		if (arg == 0) {
			__pm_wakeup_event(&cxd224x_dev->wakelock_lp, CXD224X_WAKE_LOCK_TIMEOUT_LP*HZ);
			/* PON HIGH (normal power mode)*/
			gpio_set_value(cxd224x_dev->wake_gpio, 1);  
		} else if (arg == 1) {
			/* PON LOW (low power mode) */
			gpio_set_value(cxd224x_dev->wake_gpio, 0);
			__pm_relax(&cxd224x_dev->wakelock_lp);
		} else {
			/* do nothing */
		}
		break;
	default:
		dev_err(&cxd224x_dev->client->dev,
			"%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
		return 0;
	}

	return 0;
}

static const struct file_operations cxd224x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = cxd224x_dev_poll,
	.read = cxd224x_dev_read,
	.write = cxd224x_dev_write,
	.open = cxd224x_dev_open,
	.release = cxd224x_dev_release,
	.unlocked_ioctl = cxd224x_dev_unlocked_ioctl,
	.compat_ioctl  = cxd224x_dev_unlocked_ioctl,
};

#define RETRY_TIMES 20
static int cxd224x_RxData2(uint8_t *buf, int len)
{
	int ret = 0;
	int i;
	unsigned char *tmp = NULL;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;

	I("cxd224x_RxData2() Start..\r\n");
	I("[%s] : chk addr = 0x%x, len:%d\n",__func__, cxd224x_dev->client->addr, len);
	tmp = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL); 
	for (i = 0; i < 3; i++) {
        	I("%s(%d) len:%d\n", __func__, __LINE__, len);
	        ret = i2c_master_recv(cxd224x_dev->client, tmp, (int)len);
        	if (unlikely(ret < 0)) {
	            E("%s(%d) fail addr:0x%x ret:%d\n", __func__, __LINE__, cxd224x_dev->client->addr, ret);
        	    msleep(1);
	        }
        	else {
	            break;
        	}
	}
	memcpy(buf, tmp, len);

	for (i = 0; i < len; i++) \
		I("%s : Rxbuf[%d] = 0x%x\n",__func__, i, *(buf+i));
	kfree(tmp);

	I("cxd224x_RxData2() End..\r\n");

	return ret;
}

static int cxd224x_TxData2(uint8_t *buf, int len)
{
    int ret = 0;
    int i;
	char *tmp = NULL;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	I("cxd224x_TxData2() Start..\n");
        //tmp = memdup_user(buf, len);
	tmp = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
	memcpy(tmp, buf, len);

	I("[%s] : dump buffer, addr = 0x%x, len:%d\n",__func__, cxd224x_dev->client->addr, len);
	for (i = 0; i < len; i++) \
		I("%s : Txbuf[%d] = 0x%x\n",__func__, i, *(tmp+i));

    for (i = 0; i < 3; i++) {		
        I("%s (%d), len:%d\n", __func__, i, len);
		ret = i2c_master_send(cxd224x_dev->client, tmp, (int)len);
        if (unlikely(ret < 0)) {
            E("%s(%d) failed to write:0x%x ret:%d\n", __func__, __LINE__, cxd224x_dev->client->addr, ret);
			msleep(1);
		}
		else {
			break;
		}
   	}
	kfree(tmp);

	I("cxd224x_TxData2() End.\n");

    return ret;
}

void nfc_nci_dump_data(unsigned char *data, int len) {
	int i = 0, j = 0;
	memset(NCI_TMP, 0x00, MAX_BUFFER_SIZE);
	for (i = 0, j = 0; i < len; i++)
		j += sprintf(NCI_TMP + j, " 0x%02X", data[i]);
	I("%s\n", NCI_TMP);
}

//	nci_reader return conditions:
//	-255: i2c error, break
//	-1: script request condition not reached, stay at current
//	 0: script restart
//	 1: script request condition reached
//	 other n: go to scriptIndex n

int nci_Reader(control_msg_pack *script, unsigned int scriptSize) {
	static control_msg_pack *previous = 0;
	static int res_achieved = 0;
	static int ntf_achieved = 0;
	static char expect_resp_header[2] = {0};
	static char expect_ntf_header[2] = {0};
	uint8_t receiverBuffer[MAX_NFC_DATA_SIZE] ={0};
	/*Read 3 byte to ensure the correct data */
	uint8_t nci_rx_header[3] = {0};
	unsigned char nci_data_len = 0;
	unsigned int GOID = 0;
	int rf_support_len = 0;

	I("%s()+++\n", __func__);
	memset(FelicaIDm, 0, MAX_BUFFER_SIZE);

	if (previous != script) {
		I("new command, reset flags.\n");
		previous = script;
		res_achieved = 0;
		ntf_achieved = 0;
		if (script->exp_resp_content != 0) {
			if (0x20 == (script->cmd[1] & 0xF0))
				expect_resp_header[0] = script->cmd[1] + 0x20;	/* 0x40 */
			else
				expect_resp_header[0] = script->cmd[1];
			expect_resp_header[1] = script->cmd[2];
			I(": expect_resp_header : 0x%02X, 0x%02X\n", expect_resp_header[0], expect_resp_header[1]);
		}

		if (*(script->exp_ntf) != 0) {
			if (0x20 == (script->cmd[1] & 0xF0))
				expect_ntf_header[0] = script->cmd[1] + 0x40;	/* 0x60 */
			else if (0x00 == (script->cmd[1] & 0xF0))
				expect_ntf_header[0] = 0x60;
			I("Expected NTF Header: 0x%02X\n", expect_ntf_header[0]);
		}
	}


	if ( cxd224x_RxData2(nci_rx_header, 3) < 0) {
		I("I2C error while read out the NCI header.\n");
		return -255;
	} else {
		I("@@@ NCI header read: [Header] 0x%02X, 0x%02X [Len] 0x%02X", nci_rx_header[0], nci_rx_header[1], nci_rx_header[2]);
		nci_data_len = nci_rx_header[2];

		mdelay(NFC_READ_DELAY);
		if ( nci_data_len < 0) {
			I("I2C error while read out the NCI data length.\n");
			return -255;
		} else {		
			//I("NCI data length read: %d\n", (int)nci_data_len);
			mdelay(NFC_READ_DELAY);
			if ( cxd224x_RxData2(receiverBuffer, nci_data_len) < 0) {
				I("I2C error while read out the NCI data.\n");
				return -255;
			} else {
				I("NCI data: ");
				nfc_nci_dump_data(receiverBuffer, (int)nci_data_len);
			}
		}
	}
	//I("### NCI header read: 0x%02X, 0x%02X\n", nci_rx_header[0], nci_rx_header[1]);
	//nfc_nci_dump_data(nci_rx_header, (int)2);

	//I("### nci_data_len:%d ", nci_data_len);
	
	//I("### NCI data: ");
	//nfc_nci_dump_data(receiverBuffer, (int)nci_data_len);
	
	/* Bypass separate package first, not used by either source currently */
	/* process responses */
	I("### process responses...\n");
	if (0x40 == (nci_rx_header[0] & 0xF0)) {
		I("### checking GID\n");
		GOID = nci_rx_header[0] & 0x0F;
		GOID = (GOID << 8) | nci_rx_header[1];
		I("GOID: 0x%08X\n", GOID);

		switch (GOID) {
		case 0x0000: /* Case CORE_RESET_RSP */
			I("Response CORE_RESET_RSP received.\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
					I("Response type matched with command. exp_len:%x\n", script->exp_resp_content[0]);
					if (memcmp(&script->exp_resp_content[4], receiverBuffer, script->exp_resp_content[3]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
				}
			}
			gDevice_info.NCI_version = receiverBuffer[1];
			break;
		case 0x0001: /* Case CORE_INIT_RSP */
			I("### Response CORE_INIT_RSP received.\n");
			if (*(script->exp_resp_content)) {
				//I("### CORE_INIT_RSP trace_1.\n");
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\n");
					if (memcmp(&script->exp_resp_content[4], receiverBuffer, script->exp_resp_content[3]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");
						I("### CORE_INIT, res_achieved = 1\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
				}
			}
			rf_support_len = receiverBuffer[5];
			gDevice_info.NFCC_Features = ((unsigned int)receiverBuffer[4]) << 24 | ((unsigned int)receiverBuffer[3]) << 16 | ((unsigned int)receiverBuffer[2]) << 8 | receiverBuffer[1];
			gDevice_info.manufactor = receiverBuffer[12 + rf_support_len];
			gDevice_info.fwVersion = ((unsigned int)receiverBuffer[15 + rf_support_len]) << 8 | ((unsigned int)receiverBuffer[16 + rf_support_len]);
			I("FW Version 0x%07lX\n", gDevice_info.fwVersion);
			mfc_nfc_cmd_result = 1;	//(int)gDevice_info.fwVersion;
			break;
		case 0x0103: /* Case RF_DISCOVER_RSP */
			I("### Response RF_DISCOVER_RSP received.\n");
			if (*(script->exp_resp_content)) {				
				//I("### RF_DISCOVER_RSP trace_1.\n");
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");						
						I("### RF_DISCOVER, res_achieved = 1\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
				}
			}
			if (script->cmd[5] < 0x80)
				I("Start to detect Cards.\n");
			else
				I("Start to listen Reader.\n");
			/* Set target NTF as RF_INTF_ACTIVATED_NTF */
			expect_ntf_header[1] = 0x05;
			break;
		case 0x0200: /* Case NFCEE_DISCOVER_RSP */
			I("Response NFCEE_DISCOVER_RSP received.\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");
						I("### NFCEE_DISCOVER_RSP, res_achieved = 1\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
				}
			}
			/* Set target NTF as NFCEE_DISCOVER_NTF */
			expect_ntf_header[1] = 0x00;
			break;
		case 0x0201: /* Case NFCEE_MODE_SET_RSP */
			I("Response NFCEE_MODE_SET_RSP received.\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");
						I("### NFCEE_MODE_SET_RSP, res_achieved = 1\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
				}
			}
			/* Set target NTF as NFCEE_DISCOVER_NTF */
			expect_ntf_header[1] = 0x00;
			break;
		case 0x0F20: /* Case SONY  */
			I("SONY specific packert.\n");
			I("### Response FW_VERSION_CMD received.\n");
                        if(*(script->exp_resp_content)) {
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0) {
					I("Response type matched with command.\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");
						I("### FW_VERSION_CMD, res_achieved = 1\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
					return -255;
				}
			}

			gDevice_info.fwVersion = ((unsigned int)receiverBuffer[3]) << 16 | \
						 ((unsigned int)receiverBuffer[2]) << 8 | \
						 (unsigned int)receiverBuffer[1];
			I("FW Version 0x%06lX\n", gDevice_info.fwVersion);
			gDevice_info.FW_Minor = gDevice_info.fwVersion & 0xFFFF;
			gDevice_info.FW_Major = (gDevice_info.fwVersion >> 16) & 0xFFFF;
			mfc_nfc_cmd_result = 1; //(int)gDevice_info.fwVersion;

			break;
		case 0x0F1B: /* Case SONY  */
			I("SONY specific packert.\n");
			I("### Response PATCH_VERSION_CMD received.\n");
			if (*(script->exp_resp_content)) {
				//I("### PATCH_VERSION_CMD trace_1.\n");
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\n");
					if (memcmp(&script->exp_resp_content[4], receiverBuffer, script->exp_resp_content[3]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");
						I("### PATCH_VERSION_CMD, res_achieved = 1\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
				}
			}
			gDevice_info.fwVersion = ((unsigned int)receiverBuffer[5]) << 24 | ((unsigned int)receiverBuffer[4]) << 16 | ((unsigned int)receiverBuffer[3]) << 8 | receiverBuffer[2];
			I("FW Version 0x%08lX\n", gDevice_info.fwVersion);
			mfc_nfc_cmd_result = 1;	//(int)gDevice_info.fwVersion;
			break;
		default:
			I("Response not defined.\r\n");
			if (*(script->exp_resp_content)) {
				//I("### default trace_1.\r\n");
				if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\n");						
						I("### default, res_achieved = 1\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\n");
				}
			} else
				I("No response requirement.\n");
		}
	}

	/* process felica IDm */
	//E("### process Felica IDm...\n");
	if ( (0x02 == nci_rx_header[0]) && (0x00 == nci_rx_header[1]) ) {
		E("Felica IDm+++ : \n");
		nfc_nci_dump_data(&receiverBuffer[2], 8);
		memcpy(FelicaIDm,NCI_TMP,MAX_BUFFER_SIZE);
		E("Felica IDm--- : %s\n", FelicaIDm);
		res_achieved = 1;
		return 1;
	}

	/* process data packets */
	I("### process data packets...\n");
	if (0x00 == (nci_rx_header[0] & 0xF0)) {
		I("Data Packet, Connection ID:0x%02X\n", (nci_rx_header[0] & 0x0F));
		if (*(script->exp_resp_content)) {
			//I("### data packets trace_1.\n");
			if (memcmp(nci_rx_header, expect_resp_header, 2) == 0){
				I("Response type matched with command.\n");
				if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
					I("Response matched with expected response, res_achieved set.\n");
					I("### process data packets, res_achieved = 1\n");
					res_achieved = 1;
				} else {
					I("Not expected response! Quit now.\n");
					return -255;
				}
			} else {
				I("### Command-Response type not matched, ignore.\n");
			}
		} else
			I("No response requirement.\n");
		if (0x00 == (nci_rx_header[0] & 0xF0))
			expect_ntf_header[1] = 0x06;
	}

	/* process notifications */
	I("### process notifications...\n");
	if (0x60 == (nci_rx_header[0] & 0xF0)) {
		I("### notifications, checking GOID...\n");
		GOID = nci_rx_header[0] & 0x0F;
		GOID = (GOID << 8) | nci_rx_header[1];
		I("GOID: 0x%08X\n", GOID);

		switch (GOID) {
		case 0x0103:
			I("Notification RF_DISCOVER_NTF received.\n");
			if (*(script->exp_ntf)) { /*Case when expected multiple remote SE NTF coming*/
				if (memcmp(nci_rx_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\n");
						return -1;
					}
				} else {
					//I("Command-Notification type not matched, ignore.\n");
					/* Case when waiting for RF_INTF_ACTIVATED_NTF but multiple NTF detected */
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_ID = receiverBuffer[0];
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Protocol = receiverBuffer[1];
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Technology = receiverBuffer[2];
					if (gDevice_info.target_rf_id == 255 &&
					 gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Protocol == gDevice_info.protocol_set)
						gDevice_info.target_rf_id = gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_ID;

					if (receiverBuffer[nci_data_len - 1] == 0) {
						I("Last INTF_NTF reached.\n");
						I("Card detected!\n");

//						select_rf_target[0].cmd[4] = gDevice_info.target_rf_id;
//						select_rf_target[0].cmd[5] = gDevice_info.protocol_set;
//						select_rf_target[0].cmd[6] = gDevice_info.intf_set;
//						if (script_processor(select_rf_target, sizeof(select_rf_target)) == 0)
//							return 1;
//						else
//							return -1;
					}
				}
			}
			gDevice_info.NTF_count++;
			break;
		case 0x0105: /* Case RF_INTF_ACTIVATED_NTF */
			I("Notification RF_INTF_ACTIVATED_NTF received.\n");
			if (*(script->exp_ntf)) {
				//I("### RF_INTF_ACTIVATED_NTF trace_1.\n");
				if (memcmp(nci_rx_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\n");
						gDevice_info.activated_INTF = receiverBuffer[0];
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\n");
				}
			}
			if (receiverBuffer[3] < 0x80)
				I("Card detected!\n");
			else
				I("Reader detected!\n");
			break;
		case 0x0200: /* Case NFCEE_DISCOVER_NTF */
			I("Notification NFCEE_DISCOVER_NTF received.\n");
			if (*(script->exp_ntf)) {
				//I("### NFCEE_DISCOVER_NTF trace_1.\n");
				if (memcmp(nci_rx_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\n");
				}
			}
			gDevice_info.HW_model = receiverBuffer[1];
			break;
		case 0x010A: /* Case NFCEE_DISCOVER_NTF */
			I("RF_NFCEE_DISCOVERY_REQ_NTF+++ received.\n");
			if (*(script->exp_ntf)) {
				//I("### RF_NFCEE_DISCOVERY_REQ_NTF trace_1.\n");
				if (memcmp(nci_rx_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\n");
						return -1;
					}
				} else {
					I("Felica Command-Notification type not matched, ignore to next command\n");
					ntf_achieved = 1;
				}
			}
			I("Felica RF_NFCEE_DISCOVERY_REQ_NTF---\n");
		break;
		default:
			I("Notification not defined.\n");
			if (*(script->exp_ntf)) {
				if (memcmp(nci_rx_header, expect_ntf_header, 1) == 0){
					I("Notification type matched with command.\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\n");
				}
			} else
				I("No Notification requirement.\n");
		}
	}

	//I("### if (*(script->exp_resp_content) != 0)\r\n");
	if (*(script->exp_resp_content) != 0) {
		if (res_achieved) {
			//I("### if (res_achieved)\r\n");
			if (*(script->exp_ntf) != 0) {
				if (ntf_achieved) {
					return 1;
				} else {
					I("Notification requirement not achieve, stay at current command_dbg1.\n");
					//if (watchdogEn == 1)
					//	watchdog_counter = 0;

					return -1;
				}
			} else {
				I("No NTF requirement, step to next command.\n");
				return 1;
			}
		} else {	
			//I("### else (res_achieved)\r\n");
			I("Response requirement not achieve, stay at current command.\n");

			//if (watchdogEn == 1)
			//	watchdog_counter = 0;

			return -1;
		}
	} else if (*(script->exp_ntf) != 0) {
		if (ntf_achieved) {
			return 1;
		} else {
			I("Notification requirement not achieve, stay at current command_dbg2.\n");

			//if (watchdogEn == 1)
			//	watchdog_counter = 0;

			return -1;
		}
	} else {
		I("No requirement, step to next command.\n");
		return 1;
	}
}


#define CHECK_READER(void) \
do { \
	if (watchdogEn == 1) {\
		if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {\
			I("watchdog timeout, command fail.\n");\
			goto TIMEOUT_FAIL; \
		} \
	} \
	if (!gpio_get_value(cxd224x_dev->irq_gpio)) { \
		reader_resp = nci_Reader(&script[scriptIndex], scriptSize); \
		/*I("%d returned.\r\n", reader_resp);*/ \
		switch(reader_resp) { \
		case -255: \
			/* I2C error, break*/ \
			goto I2C_FAIL; \
			break; \
		case -1: \
			/* stay at current command.*/ \
			break; \
		case 0: \
			/* script restart */ \
			scriptIndex = 0; \
			break; \
		case 1: \
			/* step to next command. */ \
			scriptIndex++; \
			break; \
		default: \
			scriptIndex = reader_resp; \
		} \
	} \
} while(0)


int script_processor(control_msg_pack *script, unsigned int scriptSize) {
	int ret = 0;
	int scriptIndex, reader_resp;
	int last_scriptIndex;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;

	scriptSize = scriptSize/sizeof(control_msg_pack);
	
	scriptIndex = 0;
	last_scriptIndex = -1;
	reader_resp = 1;

	I("%s: script_processor script index:%d, size: %d.\n",__func__,scriptIndex, scriptSize);

	do {
		if (reader_resp == -1) {
			CHECK_READER();
			mdelay(NFC_READ_DELAY);

			if (watchdogEn == 1)
				if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {
					I("watchdog timeout, command fail.\n");
					goto TIMEOUT_FAIL;
				}

			continue;
		}

		if ( last_scriptIndex != scriptIndex) {
			I("script_processor cxd224x_TxData2()+\n");
			ret = cxd224x_TxData2(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
			I("script_processor cxd224x_TxData2()-\n");
			if (ret < 0) {
				E("%s, i2c Tx error!\n", __func__);
				nfc_nci_dump_data(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
				goto I2C_FAIL;
				break;
			}
			else {
					I("i2c wrote: ");
					nfc_nci_dump_data(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
					mdelay(NFC_READ_DELAY + 20);
					last_scriptIndex = scriptIndex;
					I("script_processor CHECK_IRQ value :%d\n", gpio_get_value(cxd224x_dev->irq_gpio));
					CHECK_READER();
				}
		} else {
			CHECK_READER();

			if (watchdogEn == 1)
				if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {
					I("watchdog timeout, command fail.\n");
					goto TIMEOUT_FAIL;
				}

		}
		mdelay(NFC_READ_DELAY);
	} while(scriptIndex < scriptSize);

	I("%s()---\n", __func__);

	return 0;
I2C_FAIL:
	E("%s, I2C_FAIL!\n", __func__);
	mfc_nfc_cmd_result = -2;
	return 1;
TIMEOUT_FAIL:
	mfc_nfc_cmd_result = 0;
	return 1;
}

static void cxd224x_hw_reset(void)
{
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	//int ret = 0;

	I("%s()+++\n", __func__);

	I("%s: 1_set RST rst_gpio pin [Low/HIGH]\n", __func__);
	gpio_set_value(cxd224x_dev->rst_gpio, CXDNFC_RST_ACTIVE);	//for gpio_30 with inverter, trigger reset
	//ret = gpio_direction_output(cxd224x_dev->rst_gpio, 0);
	I("%s: 2_wait for 50 ms, chk irq:%d, rst:%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->rst_gpio)); 
	mdelay(50);
	I("%s: 3_set RST rst_gpio pin [High/LOW]\n", __func__);
	//ret = gpio_direction_output(cxd224x_dev->rst_gpio, 1);
	gpio_set_value(cxd224x_dev->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);	//for gpio_30 with inverter, cancel reset
	I("%s: 4_wait for 50 ms, chk irq:%d, rst:%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->rst_gpio)); 
	mdelay(50);

    I("%s()---\n", __func__);
}

static void cxd224x_core_version(void)
{
	int ret = 0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	uint8_t version_cmd[] = {0x2F, 0x20, 0x00}; // GET_VERSION_CMD
	int i = 0;

	// CORE_INIT_CMD
	I("%s: before TX check NFC_IRQ pin value %d, (wake: %d)\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->wake_gpio));
	I("%s: [reset_cmd] (pon: %d)\n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));
	for (i = 0; i < 100; i++) {
		I("%s: cxd224x_TxData2 check i=%d, NFC_IRQ value %d\n", __func__, i, gpio_get_value(cxd224x_dev->irq_gpio));
		if(1 == gpio_get_value(cxd224x_dev->irq_gpio)) {		
			ret = cxd224x_TxData2(version_cmd, 3);
			if (ret < 0)
				E("%s, i2c Tx error!\n", __func__);
			break;
		}
		mdelay(5);
	}
	I("%s: after TX check NFC_IRQ pin value %d, ret=%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), ret);

}

static void cxd224x_patch_version(void)
{
	int ret =0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	uint8_t version_cmd[] = {0x2F, 0x1B, 0x00}; // GET_VERSION_CMD
	int i = 0;

	// CORE_INIT_CMD
	I("%s: before TX check NFC_IRQ pin value %d, (wake: %d)\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->wake_gpio));
	I("%s: [reset_cmd] (pon: %d)\n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));
	for (i = 0; i < 100; i++) {
		I("%s: cxd224x_TxData2 check i=%d, NFC_IRQ value %d\n", __func__, i, gpio_get_value(cxd224x_dev->irq_gpio));
		if(1 == gpio_get_value(cxd224x_dev->irq_gpio)) {		
			ret = cxd224x_TxData2(version_cmd, 3);
			if (ret < 0)
				E("%s, i2c Tx error!\n", __func__);
			break;
		}
		mdelay(5);
	}
	I("%s: after TX check NFC_IRQ pin value %d, ret=%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), ret);

}

static void cxd224x_core_reset(void)
{
	int ret = 0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	uint8_t reset_cmd[] = {0x20, 0x00, 0x01, 0x01}; // CORE_RESET_CMD
	int i = 0;

	// CORE_RESET_CMD
	I("%s: before TX check NFC_IRQ pin value %d, (wake: %d)\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->wake_gpio));
	I("%s: [reset_cmd] (pon: %d)\n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));
	for (i = 0; i < 100; i++) {
		I("%s: cxd224x_TxData2 check i=%d, NFC_IRQ value %d\n", __func__, i, gpio_get_value(cxd224x_dev->irq_gpio));
		if(1 == gpio_get_value(cxd224x_dev->irq_gpio)) {		
			ret = cxd224x_TxData2(reset_cmd, 4);
			if (ret < 0)
				E("%s, i2c Tx error!\n", __func__);
			break;
		}
		mdelay(5);
	}
	I("%s: after TX check NFC_IRQ pin value %d, ret=%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), ret);

}


static void cxd224x_core_init(void)
{
	int ret = 0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	uint8_t init_cmd[] = {0x20, 0x01, 0x00};	// CORE_INIT_CMD
	int i = 0;

	// CORE_INIT_CMD
	I("%s: before TX check NFC_IRQ pin value %d, (wake: %d)\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->wake_gpio));
	I("%s: [reset_cmd] (pon: %d)\n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));
	for (i = 0; i < 100; i++) {
		I("%s: cxd224x_TxData2 check i=%d, NFC_IRQ value %d\n", __func__, i, gpio_get_value(cxd224x_dev->irq_gpio));
		if(1 == gpio_get_value(cxd224x_dev->irq_gpio)) {		
			ret = cxd224x_TxData2(init_cmd, 3);
			if (ret < 0)
				E("%s, i2c Tx error!\n", __func__);
			break;
		}
		mdelay(5);
	}
	I("%s: after TX check NFC_IRQ pin value %d, ret=%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), ret);

}

static void cxd224x_pon_on(void)
{
	int ret = 0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
 
    I("%s()+++\n", __func__);

	I("%s : before set fel_pon output HIGH (%d) \n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));
	ret = gpio_direction_output(cxd224x_dev->wake_gpio, 1);
	I("%s : after set fel_pon output HIGH (%d) DELAY 100\n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));
	mdelay(100);

	I("%s()---\n", __func__);
}

static void cxd224x_pon_off(void)
{
	int ret = 0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
 
    I("%s()+++\n", __func__);

	I("%s : before set fel_pon output LOW (%d) \n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));
	ret = gpio_direction_output(cxd224x_dev->wake_gpio, 0);	
	mdelay(50);
	I("%s : after set fel_pon output LOW (%d) \n", __func__, gpio_get_value(cxd224x_dev->wake_gpio));

	I("%s()---\n", __func__);
}

static void cxd224x_reset_pon(void)
{
    I("%s()+++\n", __func__);
	cxd224x_pon_off();
	cxd224x_pon_on();
	I("%s()---\n", __func__);
}

static void cxd224x_nci_read(void)
{
	int i;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	uint8_t receiverBuffer[MAX_NFC_DATA_SIZE] ={0};
	uint8_t nci_read_header[3] = {0};
 	unsigned char nci_data_len = 0;
 
	 I("%s()+++\n", __func__);
	 memset(dataresp, 0, MAX_NFC_DATA_SIZE);

	/* read out HW_REST notification */
	for (i = 0; i < 10; i++) {	
		I("%s: cxd224x_RxData2 check i=%d, NFC_IRQ %d\n", __func__, i, gpio_get_value(cxd224x_dev->irq_gpio));
		if (!gpio_get_value(cxd224x_dev->irq_gpio)) {
			I("INTR be triggered, NFC_IRQ to LOW!\r\n");
			if ( cxd224x_RxData2(nci_read_header, 3) < 0) {
				I("I2C error while read out the NCI header.\n");
			} else {
				I("NCI header read: 0x%02X, 0x%02X, 0x%02X\n", 
                                   nci_read_header[0], nci_read_header[1], nci_read_header[2]);
                                nci_data_len = nci_read_header[2];

				mdelay(NFC_READ_DELAY);
				if ( cxd224x_RxData2(receiverBuffer, nci_data_len) < 0) {
					I("I2C error while read out the NCI data.\n");
				} else {
					I("NCI data: ");
					nfc_nci_dump_data(receiverBuffer, (int)nci_data_len);
				        if ( bcpyrespflag ) {
						cpyresplen = (int)nci_data_len;
						memcpy(dataresp,NCI_TMP,MAX_BUFFER_SIZE);
						edcvalue = receiverBuffer[1];
						bcpyrespflag = 0;
					}
				}
			}
			
			/* process responses */
			if (0x40 == (nci_read_header[0] & 0xF0)) {
				I("NCI CMD Resp");
			}
			
			/* process notifications */
			if (0x60 == (nci_read_header[0] & 0xF0)) {
				I("NCI NTF!!");

				if ((1 == nci_data_len) && (0xee == receiverBuffer[0])) {
					I("### received CORE_RESET_NTF_ERROR!! ###");
					readout_core_reset_ntf = 0xEE;
				}

				if ((2 == nci_data_len) && (0x00 == receiverBuffer[0]) && (0x01 == receiverBuffer[1]))
					readout_core_reset_ntf = 0;
			}
		}
		else
		{
			break;
		}
	
		mdelay(100);
	}	
	I("%s()---\n", __func__);
}

//////////////////////////////////////////////
#if 0
static void cxd224x_felica_work(struct work_struct *work)
{
	I("%s()+++\n", __func__);
	if (cxd224x_write_eeprom(nfc_eeprom0_script, sizeof(nfc_eeprom0_script)) == 0) {
		I("%s: flash nfc_eeprom_script succeed", __func__); 	
		mfc_nfc_cmd_result = 1;
	}
	mdelay(500);
	if (cxd224x_write_eeprom(nfc_eeprom1_script, sizeof(nfc_eeprom1_script)) == 0) {
		I("%s: flash nfc_eeprom_script succeed", __func__); 	
		mfc_nfc_cmd_result = 1;
	}
	I("%s()---\n", __func__);

}
#endif
static ssize_t debug_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("debug_enable_show\n");

	ret = sprintf(buf, "is_debug=%d\n", is_debug);
	return ret;
}
static ssize_t debug_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%d", &is_debug);
	return count;
}
static DEVICE_ATTR(debug_enable, 0664, debug_enable_show, debug_enable_store);

static ssize_t debug_cmd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	I("%s() nfc_cmd_result:%d\n", __func__, nfc_cmd_result);

	cxd224x_core_reset();

	cxd224x_reset_pon();

	if (nfc_cmd_result > 0) {
		return scnprintf(buf, PAGE_SIZE,
			"NFC firmware version: 0x%07x\n", nfc_cmd_result);
	}
	else if (nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,nfc_cmd_result);
	}
}

#define i2cw_size (20)

static ssize_t debug_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code = -1;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;

	sscanf(buf, "%d", &code);

	I("%s()+++\n", __func__);
	I("%s: irq = %d, rst = %d,  wake_gpio = %d +\n", __func__, \
		gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->rst_gpio), \
		gpio_get_value(cxd224x_dev->wake_gpio));

	I("%s: store value = %d\n", __func__, code);

	switch (code) {
	case 0:
			I("%s: cxd224x_hw_reset\n", __func__);
			cxd224x_hw_reset();
			break;
	case 1:
			I("%s: cxd224x_pon_off\n", __func__);
			cxd224x_pon_off();
			break;
	case 2:
			I("%s: cxd224x_pon_on\n", __func__);
			cxd224x_pon_on();
			break;
	case 3:
			I("%s: cxd224x_nci_read\n", __func__);
			cxd224x_nci_read();
			break;
	case 4:
			I("%s: cxd224x_core_version\n", __func__);
			cxd224x_core_version();
			break;
	case 5:
			I("%s: cxd224x_core_reset\n", __func__);
			cxd224x_core_reset();
			break;
	case 6:
			I("%s: cxd224x_core_init\n", __func__);
			cxd224x_core_init();
			break;
	case 7:
			I("%s: cxd224x_patch_version\n", __func__);
			cxd224x_patch_version();
			break;
	default:
			E("%s: case default\n", __func__);
			I("%s: cxd224x_nci_read\n", __func__);
			cxd224x_nci_read();
			break;
	}

	I("%s()---\n", __func__);

	return count;
}

static DEVICE_ATTR(debug_cmd, 0664, debug_cmd_show, debug_cmd_store);

/* mfg test commamd attribute file */
static int mfg_nfc_test(int code)
{
	int ret = 0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
#if 0
	uint8_t fn_cup_cmd[8] = {0x00, 0x00, 0x05, 0x05, 0xF0, 0x00, 0x12, 0x34};
#endif
	watchdog_counter = 0;
	watchdogEn = 1;

	I("%s()+++\n", __func__);
	I("%s: irq = %d, wake_gpio = %d +\n", __func__, \
		gpio_get_value(cxd224x_dev->irq_gpio), \
		gpio_get_value(cxd224x_dev->wake_gpio));

	I("%s: store value = %d\n", __func__, code);

	switch (code) {
	case 0:
			I("%s: nfc version\n", __func__);
			cxd224x_pon_on();
			if (script_processor(nfc_version_script, sizeof(nfc_version_script)) == 0) {
				I("%s: get nfcversion succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
			cxd224x_pon_off();
			break;
	case 1:
			I("%s: type-A polling\n", __func__);
			cxd224x_pon_on();
			if (script_processor(nfc_reader_A_script, sizeof(nfc_reader_A_script)) == 0) {
				I("%s: type-A polling succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
			cxd224x_pon_off();
			break;
	case 2:
			I("%s: type-B polling\n", __func__);
			cxd224x_pon_on();
			if (script_processor(nfc_reader_B_script, sizeof(nfc_reader_B_script)) == 0) {
				I("%s: type-B polling succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
			cxd224x_pon_off();
			break;
	case 3:
			I("%s: type-F polling\n", __func__);
			cxd224x_pon_on();
			if (script_processor(nfc_reader_F_script, sizeof(nfc_reader_F_script)) == 0) {
				I("%s: type-F polling succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
#if 0
			I("%s, Launch CUP command()+++++\n", __func__);
			ret = cxd224x_nci_write(fn_cup_cmd, sizeof(fn_cup_cmd));
			cxd224x_nci_read();
			I("%s, Launch CUP command()-----\n", __func__);
#endif
			cxd224x_pon_off();
			break;
	case 4:
			I("%s: type-A listen\n", __func__);
			cxd224x_pon_on();
			if (script_processor(nfc_card_script, sizeof(nfc_card_script)) == 0) {
				I("%s: type-A listen succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
			cxd224x_pon_off();
			break;
	case 5:
			I("%s: nfc_card_without_sim listen\n", __func__);
			cxd224x_pon_on();
			if (script_processor(nfc_card_without_sim, sizeof(nfc_card_without_sim)) == 0) {
				I("%s: nfc_card_without_sim listen succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
			cxd224x_pon_off();
			break;
	case 6:
			I("%s: nfc version for tset, no hw reset!\n", __func__);
			cxd224x_pon_on();
			if (script_processor(nfc_version_script_test, sizeof(nfc_version_script_test)) == 0) {
				I("%s: get nfcversion succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
			cxd224x_pon_off();
			break;
	case 7:
			I("%s: reset pon pin\n", __func__);
			cxd224x_reset_pon();
			break;
	case 8:
			I("%s: cancel cxd_reset pin\n", __func__);
			I("%s: set RST rst_gpio pin [High/LOW]\n", __func__);
			gpio_set_value(cxd224x_dev->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);	//for gpio_30 with inverter, cancel reset
		//	ret = gpio_direction_output(cxd224x_dev->rst_gpio, 1);
			I("%s: wait for 50 ms, rst:%d\n", __func__, gpio_get_value(cxd224x_dev->rst_gpio));
			mdelay(50);
			break;
	case 9:
			I("%s: set cxd_reset pin\n", __func__);
			I("%s: set RST gpio pin [Low/HIGH]\n", __func__);
			gpio_set_value(cxd224x_dev->rst_gpio, CXDNFC_RST_ACTIVE);	//for gpio_30 with inverter, trigger reset
		//	ret = gpio_direction_output(cxd224x_dev->rst_gpio, 0);
			I("%s: wait for 50 ms, rst:%d\n", __func__, gpio_get_value(cxd224x_dev->rst_gpio));
			mdelay(50);
			break;
	case 10:
			I("%s: felica_ese_without_sim listen\n", __func__);
			cxd224x_pon_on();
			if (script_processor(felica_ese_without_sim, sizeof(felica_ese_without_sim)) == 0) {
				I("%s: felica_ese_without_sim succeed", __func__);
				mfc_nfc_cmd_result = 1;
			}
			cxd224x_pon_off();
			break;
	case 11:
			I("%s: get felica_ese_IDm\n", __func__);
			memset(FelicaIDm, 0, MAX_BUFFER_SIZE);
			cxd224x_pon_on();
			if (script_processor(felica_ese_idm, sizeof(felica_ese_idm)) == 0) {
				I("%s: get felica_ese_IDm succeed", __func__);
			}
			cxd224x_pon_off();
			break;
        case 12:
                        I("%s: cxd2252 polling (128 times)\n", __func__);
                        cxd224x_pon_on();
                        if (script_processor(nfc_cxd2252_polling, sizeof(nfc_cxd2252_polling)) == 0) {
                                I("%s: polling 128 times succeed", __func__);
                                mfc_nfc_cmd_result = 1;
                        }
                        cxd224x_pon_off();
                        break;
	default:
			E("%s: case default\n", __func__);
			break;
	}

	I("%s()---\n", __func__);
	return ret;
}


static ssize_t mfg_nfcversion(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s watchdogEn is %d\n", __func__, watchdogEn);
	if (nqx_clock_select(cxd224x_info) < 0)
                E("%s unable to enable clock\n", __func__);
        msleep(20);

	ret = mfg_nfc_test(0);
	if (nqx_clock_deselect(cxd224x_info) < 0)
                E("%s unable to disable clock\n",__func__);

	if (mfc_nfc_cmd_result > 0) {
		return scnprintf(buf, PAGE_SIZE,
			"NFC firmware version: %x.%04x\n", (int)gDevice_info.FW_Major, (int)gDevice_info.FW_Minor);
	}
	else if (mfc_nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,mfc_nfc_cmd_result);
	}
}
static DEVICE_ATTR(mfg_nfcversion, 0440, mfg_nfcversion, NULL);

static ssize_t mfg_nfcreader(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s watchdogEn is %d\n", __func__, watchdogEn);
	if (nqx_clock_select(cxd224x_info) < 0)
                E("%s unable to enable clock\n", __func__);
        msleep(20);

	ret = mfg_nfc_test(3);	// polling type_F
	if (nqx_clock_deselect(cxd224x_info) < 0)
                E("%s unable to disable clock\n",__func__);

	if (mfc_nfc_cmd_result == 1) {
		return scnprintf(buf, PAGE_SIZE,
			"%s Succeed\n",__func__);
	}
	else if (mfc_nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,mfc_nfc_cmd_result);
	}

	return ret;
}
static DEVICE_ATTR(mfg_nfcreader, 0440, mfg_nfcreader, NULL);

static ssize_t mfg_nfccard(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s watchdogEn is %d\n", __func__, watchdogEn);
	if (nqx_clock_select(cxd224x_info) < 0)
                E("%s unable to enable clock\n", __func__);
        msleep(20);
	ret = mfg_nfc_test(10);
	if (nqx_clock_deselect(cxd224x_info) < 0)
                E("%s unable to disable clock\n",__func__);

	if (mfc_nfc_cmd_result == 1) {
		return scnprintf(buf, PAGE_SIZE,
			"%s Succeed\n",__func__);
	}
	else if (mfc_nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,mfc_nfc_cmd_result);
	}

	return ret;
}
static DEVICE_ATTR(mfg_nfccard, 0440, mfg_nfccard, NULL);

#if 0
static ssize_t mfg_felicacard(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s watchdogEn is %d\n", __func__, watchdogEn);
	if (nqx_clock_select(cxd224x_info) < 0)
                E("%s unable to enable clock\n", __func__);
        msleep(20);
	ret = mfg_nfc_test(10);
	if (mfc_nfc_cmd_result == 1) {
		return scnprintf(buf, PAGE_SIZE,
			"%s Succeed\n",__func__);
	}
	else if (mfc_nfc_cmd_result == 0) {
		return scnprintf(buf, PAGE_SIZE,
			"%s watchdog timeout fail\n",__func__);
	}
	else {
		return scnprintf(buf, PAGE_SIZE,
			"%s Fail %d\n",__func__,mfc_nfc_cmd_result);
	}

	return ret;
}
static DEVICE_ATTR(mfg_felicacard, 0440, mfg_felicacard, NULL);
#endif
static ssize_t mfg_felicaidm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s+ watchdogEn is %d\n", __func__, watchdogEn);
	if (nqx_clock_select(cxd224x_info) < 0)
                E("%s unable to enable clock\n", __func__);
        msleep(20);
	ret = mfg_nfc_test(11);
	
	if (nqx_clock_deselect(cxd224x_info) < 0)
                E("%s unable to disable clock\n",__func__);
//	memcpy(FelicaIDm,dataresp,16);

	I("%s IDm:%s\n", __func__, FelicaIDm);
	ret = sprintf(buf, "[NFC] Felica IDm : %s\n", FelicaIDm);
	I("%s-\n", __func__);
	return ret;
}
static DEVICE_ATTR(mfg_felicaidm, 0440, mfg_felicaidm, NULL);

static ssize_t mfg_nfc_ctrl_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s mfc_nfc_cmd_result is %d\n", __func__, nfc_cmd_result);
		ret = sprintf(buf, "%d\n\n", nfc_cmd_result);
	return ret;
}
static ssize_t mfg_nfc_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	int code = -1;
	sscanf(buf, "%d", &code);
	if (nqx_clock_select(cxd224x_info) < 0)
                E("%s unable to enable clock\n", __func__);
        msleep(20);
	ret = mfg_nfc_test(code);
	if (nqx_clock_deselect(cxd224x_info) < 0)
                E("%s unable to disable clock\n",__func__);
	return count;
}
static DEVICE_ATTR(mfg_nfc_ctrl, 0660, mfg_nfc_ctrl_show, mfg_nfc_ctrl_store);

/* set eepeom setting value attribute file */
static ssize_t hw_reset(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	if (nqx_clock_select(cxd224x_info) < 0)
                E("%s unable to enable clock\n", __func__);
        msleep(20);
	cxd224x_hw_reset();
	cxd224x_pon_on();
	msleep(20);
	cxd224x_nci_read();

	I("%s is %d\n", __func__, attr_dbg_value);
	ret = sprintf(buf, "%d\n", attr_dbg_value);

	I("%s()---\n", __func__);
	return ret;
}
static DEVICE_ATTR(hw_reset, 0440, hw_reset, NULL);

static int cxd224x_nci_write(uint8_t *buf, int len) {
	int ret = 0;
	struct cxd224x_dev *cxd224x_dev = cxd224x_info;
	int i = 0;
	I("%s()+++\n", __func__);

	I("%s: before TX check NFC_IRQ pin value %d, (pon:%d), len:%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), gpio_get_value(cxd224x_dev->wake_gpio), len);
	for (i = 0; i < 100; i++) {
		I("%s: cxd224x_TxData2 check i=%d, NFC_IRQ value %d\n", __func__, i, gpio_get_value(cxd224x_dev->irq_gpio));
		if(1 == gpio_get_value(cxd224x_dev->irq_gpio)) {
			ret = cxd224x_TxData2(buf, len);
			if (ret < 0)
				E("%s, i2c Tx error!\n", __func__);
			break;
		}
		else
			cxd224x_nci_read();
		mdelay(5);
	}
	I("%s: after TX check NFC_IRQ pin value %d, ret=%d\n", __func__, gpio_get_value(cxd224x_dev->irq_gpio), ret);
	I("%s()---\n", __func__);
    return ret;
}
//erase eflash sector 95
static ssize_t erase_sec95(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    uint8_t eflash_erase_sec95[12] = {0x2F, 0x3A, 0x09, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0xF8, 0x02, 0x00};

    I("%s()+++\n", __func__);
    if (nqx_clock_select(cxd224x_info) < 0)
    	E("%s unable to enable clock\n", __func__);
    msleep(20);

    ret = cxd224x_nci_write(eflash_erase_sec95, sizeof(eflash_erase_sec95));
    mdelay(10);
    I("%s() delay 10ms\n", __func__);
    cxd224x_nci_read();
    
    I("%s()---\n", __func__);
    if(ret < 0)
    	return scnprintf(buf, PAGE_SIZE,"%s Fail \n",__func__);
    else
    	return scnprintf(buf, PAGE_SIZE,"%s Success \n",__func__);
}
static DEVICE_ATTR(erase_sec95, 0440, erase_sec95, NULL);

//erase eflash NVR1
static ssize_t erase_NVR(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    uint8_t eflash_erase_NVR[12] = {0x2F, 0x3A, 0x09, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x10, 0x00};

    I("%s()+++\n", __func__);
    if (nqx_clock_select(cxd224x_info) < 0)
        E("%s unable to enable clock\n", __func__);
    msleep(20);

    ret = cxd224x_nci_write(eflash_erase_NVR, sizeof(eflash_erase_NVR));
    mdelay(10);
    I("%s() delay 10ms\n", __func__);
    cxd224x_nci_read();

    I("%s()---\n", __func__);
    if(ret < 0)
        return scnprintf(buf, PAGE_SIZE,"%s Fail \n",__func__);
    else
        return scnprintf(buf, PAGE_SIZE,"%s Success \n",__func__);
}
static DEVICE_ATTR(erase_NVR, 0440, erase_NVR, NULL);

/* write eflash setting value attribute file */
static ssize_t seteflash_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s()+++\n", __func__);

	I("%s is %d\n", __func__, attr_dbg_value);
	ret = sprintf(buf, "%d\n", attr_dbg_value);

	I("%s()---\n", __func__);
	return ret;
}
static ssize_t seteflash_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint8_t eflash_configure[16] = {0x2F, 0x3A, 0x0D, 0x01, 0x01, 0x00, 0x04, 0x00, 
                                        0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00, 0x00, 0x00};
	int ret = 0;
	unsigned long addr;
	unsigned long value;

	I("%s()+++\n", __func__);

	sscanf(buf, "%lx %lx", &addr, &value);
	I("%s: chk addr :%lx, value :%lx\n", __func__, addr, value);
	eflash_configure[8] = (uint8_t)addr & 0xFF;
	eflash_configure[9] = (uint8_t)(addr >> 8) & 0xFF;
        eflash_configure[10] = (uint8_t)(addr >> 16) & 0xFF;
        eflash_configure[11] = (uint8_t)(addr >> 24) & 0xFF;
	eflash_configure[12] = (uint8_t)(value >> 24) & 0xFF;
	eflash_configure[13] = (uint8_t)(value >> 16) & 0xFF;
	eflash_configure[14] = (uint8_t)(value >> 8) & 0xFF;
	eflash_configure[15] = (uint8_t)value & 0xFF;
      
	I("%s: addr:%x-%x-%x-%x, value:%x-%x-%x-%x \n", __func__, 
          eflash_configure[8], eflash_configure[9], eflash_configure[10], eflash_configure[11], 
          eflash_configure[12], eflash_configure[13], eflash_configure[14], eflash_configure[15]);
	ret = cxd224x_nci_write(eflash_configure, sizeof(eflash_configure));
	mdelay(10);
	I("%s() delay 10ms\n", __func__);
	cxd224x_nci_read();
	I("%s()---\n", __func__);
	return count;
}
static DEVICE_ATTR(seteflash, 0664, seteflash_show, seteflash_store);

/* get eflash setting value attribute file */
static ssize_t geteflash_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s()+++\n", __func__);

	I("%s dataresp:%s\n", __func__, dataresp);
	ret = sprintf(buf, "eflash value : %s\n", dataresp);

	I("%s()---\n", __func__);
	return ret;
}
static ssize_t geteflash_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
        uint8_t eflash_configure[12] = {0x2F, 0x3A, 0x09, 0x01, 0x02, 0x00, 0x04, 0x00,
                                        0x00, 0x00, 0x00, 0x00};
        int ret = 0;
        unsigned long addr;
	bcpyrespflag = 1;

        I("%s()+++\n", __func__);

        sscanf(buf, "%lx", &addr);

        I("%s: chk addr :%lx\n", __func__, addr);
        eflash_configure[8] = (uint8_t)addr & 0xFF;
        eflash_configure[9] = (uint8_t)(addr >> 8) & 0xFF;
        eflash_configure[10] = (uint8_t)(addr >> 16) & 0xFF;
        eflash_configure[11] = (uint8_t)(addr >> 24) & 0xFF;

	I("%s: addr:%x-%x-%x-%x \n", __func__,
          eflash_configure[8], eflash_configure[9], eflash_configure[10], eflash_configure[11]);
        ret = cxd224x_nci_write(eflash_configure, sizeof(eflash_configure));
        mdelay(10);
        I("%s() delay 10ms\n", __func__);
        cxd224x_nci_read();

	I("%s : flag:%d, resp : %s NFC_TMP:%s\n", __func__, bcpyrespflag, dataresp, NCI_TMP);

	cpyresplen = 0;
	bcpyrespflag = 0;
	I("%s()---\n", __func__);
	return count;
}
static DEVICE_ATTR(geteflash, 0664, geteflash_show, geteflash_store);

/* write reg setting value attribute file */
static ssize_t setreg_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        int ret = 0;
        I("%s()+++\n", __func__);

        I("%s is %d\n", __func__, attr_dbg_value);
        ret = sprintf(buf, "%d\n", attr_dbg_value);

        I("%s()---\n", __func__);
        return ret;
}
static ssize_t setreg_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
        uint8_t reg_configure[16] = {0x2F, 0x3A, 0x0D, 0x02, 0x01, 0x00, 0x04, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00, 0x00, 0x00};
        int ret = 0;
        unsigned long addr;
        unsigned long value;

        I("%s()+++\n", __func__);

        sscanf(buf, "%lx %lx", &addr, &value);
        I("%s: chk addr :%lx, value :%lx\n", __func__, addr, value);
        reg_configure[8] = (uint8_t)addr & 0xFF;
        reg_configure[9] = (uint8_t)(addr >> 8) & 0xFF;
        reg_configure[10] = (uint8_t)(addr >> 16) & 0xFF;
        reg_configure[11] = (uint8_t)(addr >> 24) & 0xFF;
        reg_configure[12] = (uint8_t)(value >> 24) & 0xFF;
        reg_configure[13] = (uint8_t)(value >> 16) & 0xFF;
        reg_configure[14] = (uint8_t)(value >> 8) & 0xFF;
        reg_configure[15] = (uint8_t)value & 0xFF;

        I("%s: addr:%x-%x-%x-%x, value:%x-%x-%x-%x \n", __func__, 
          reg_configure[8], reg_configure[9], reg_configure[10], reg_configure[11], 
          reg_configure[12], reg_configure[13], reg_configure[14], reg_configure[15]);
        ret = cxd224x_nci_write(reg_configure, sizeof(reg_configure));
        mdelay(50);
        I("%s() delay 50ms\n", __func__);
        cxd224x_nci_read();
        I("%s()---\n", __func__);
        return count;
}
static DEVICE_ATTR(setreg, 0664, setreg_show, setreg_store);

/* get reg setting value attribute file */
static ssize_t getreg_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        int ret = 0;
        I("%s()+++\n", __func__);

        I("%s dataresp:%s\n", __func__, dataresp);
        ret = sprintf(buf, "reg value : %s\n", dataresp);

        I("%s()---\n", __func__);
        return ret;
}
static ssize_t getreg_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
        uint8_t reg_configure[12] = {0x2F, 0x3A, 0x09, 0x02, 0x02, 0x00, 0x04, 0x00,
                                        0x00, 0x00, 0x00, 0x00};
        int ret = 0;
        unsigned long addr;
        bcpyrespflag = 1;

        I("%s()+++\n", __func__);

        sscanf(buf, "%lx", &addr);

        I("%s: chk addr :%lx\n", __func__, addr);
        reg_configure[8] = (uint8_t)addr & 0xFF;
        reg_configure[9] = (uint8_t)(addr >> 8) & 0xFF;
        reg_configure[10] = (uint8_t)(addr >> 16) & 0xFF;
        reg_configure[11] = (uint8_t)(addr >> 24) & 0xFF;

        I("%s: addr:%x-%x-%x-%x \n", __func__,
          reg_configure[8], reg_configure[9], reg_configure[10], reg_configure[11]);
        ret = cxd224x_nci_write(reg_configure, sizeof(reg_configure));
        mdelay(50);
        I("%s() delay 50ms\n", __func__);
        cxd224x_nci_read();

        I("%s : flag:%d, resp : %s NFC_TMP:%s\n", __func__, bcpyrespflag, dataresp, NCI_TMP);

        cpyresplen = 0;
        bcpyrespflag = 0;
        I("%s()---\n", __func__);
        return count;
}
static DEVICE_ATTR(getreg, 0664, getreg_show, getreg_store);

/* auto-tunning attribute file */
static ssize_t chkrstnft_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("%s()+++\n", __func__);
	I("%s chk readout_core_reset_ntf: 0x%x\n", __func__, readout_core_reset_ntf);
	ret = sprintf(buf, "core_reset_ntf is 0x%x\n", readout_core_reset_ntf);

	I("%s()---\n", __func__);
	return ret;
}
static DEVICE_ATTR(chkrstnft, 0440, chkrstnft_show, NULL);

/*
        * Routine to enable clock.
        * this routine can be extended to select from multiple
        * sources based on clk_src_name.
*/
static int nqx_clock_select(struct cxd224x_dev *cxd224x_dev)
{
        int r = 0;
	cxd224x_dev->clk_run = true;
	//clock is enabled in bootloader to pin control mode
	//do not enable clock again to sw control mode

        cxd224x_dev->s_clk = clk_get(&cxd224x_dev->client->dev, "ref_clk");

        if (cxd224x_dev->s_clk == NULL)
                goto err_clk;

        if (cxd224x_dev->clk_run == false)
                r = clk_prepare_enable(cxd224x_dev->s_clk);

        if (r)
                goto err_clk;

        cxd224x_dev->clk_run = true;

        return r;

err_clk:
        r = -1;
        return r;
}
/*
        * Routine to disable clocks
*/
static int nqx_clock_deselect(struct cxd224x_dev *cxd224x_dev)
{
        int r = -1;
	cxd224x_dev->clk_run = false;

        if (cxd224x_dev->s_clk != NULL) {
                if (cxd224x_dev->clk_run == true) {
                        clk_disable_unprepare(cxd224x_dev->s_clk);
                        cxd224x_dev->clk_run = false;
                }
                return 0;
        }
        return r;
}

static int cxd224x_parse_dt(struct device *dev, struct cxd224x_platform_data *pdata)
{
	struct device_node *dt = dev->of_node;
	I("%s: Start\n", __func__);

	/* irq, ven, firm gpio info */
	pdata->irq_gpio = of_get_named_gpio(dt, "sony,irq_gpio", 0);
	if (!gpio_is_valid(pdata->irq_gpio))
		E("DT:pdata->irq_gpio value is not valid\n");
	else
		I("DT:pdata->irq_gpio=%d\n", pdata->irq_gpio);

#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	pdata->en_gpio = of_get_named_gpio(dt, "sony,en_gpio", 0);
	if (!gpio_is_valid(pdata->en_gpio))
		E("DT:pdata->en_gpio value is not valid\n");
	else
		I("DT:pdata->en_gpio=%d\n", pdata->en_gpio);
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	pdata->rst_gpio = of_get_named_gpio(dt, "sony,rst_gpio", 0);
	if (!gpio_is_valid(pdata->rst_gpio))
		E("DT:pdata->rst_gpio value is not valid\n");
	else
		I("DT:pdata->rst_gpio=%d\n", pdata->rst_gpio);
#endif
	pdata->wake_gpio = of_get_named_gpio(dt, "sony,wake_gpio", 0);
	if (!gpio_is_valid(pdata->wake_gpio))
		E("DT:pdata->wake_gpio PON value is not valid\n");
	else
		I("DT:pdata->wake_gpio PON=%d\n", pdata->wake_gpio);

	of_property_read_string(dt, "qcom,clk-src", &pdata->clk_src_name);

        pdata->clkreq_gpio = of_get_named_gpio(dt, "qcom,nq-clkreq", 0);

	return 0;
}

static int cxd224x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct cxd224x_platform_data *platform_data;
	struct cxd224x_dev *cxd224x_dev = NULL;

	int irq_gpio_ok  = 0;
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	int en_gpio_ok   = 0;
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	int rst_gpio_ok = 0;
#endif
	int wake_gpio_ok = 0;
	
	int clkreq_gpio_ok = 0;

	I("%s: Start, bootmode:%d\n", __func__, cxd224x_htc_get_bootmode());

	if (cxd224x_htc_get_bootmode() != NFC_BOOT_MODE_OFF_MODE_CHARGING){
		pm8998_lvs1 = regulator_get(&client->dev, "pm8998_lvs1");
		if (pm8998_lvs1< 0) {
			E("%s : pm8998_lvs1 regulator_get fail\n", __func__);
			return -ENODEV;
		}
		ret = regulator_enable(pm8998_lvs1);
		I("%s : pm8998_lvs1 regulator_enable\n", __func__);
		I("%s : pm8998_lvs1 regulator_is_enabled = %d\n", __func__, regulator_is_enabled(pm8998_lvs1));
		if (ret < 0) {
			E("%s : pm8998_lvs1 regulator_enable fail\n", __func__);
			return -ENODEV;
		}
	}
	else{
		I("%s: OFF mode charging, not Enable HVDD and return exit!\n", __func__);
		return -ENODEV;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}
        
	if (client->dev.of_node) {
		 platform_data = kzalloc(sizeof(*platform_data), GFP_KERNEL);
		 if (platform_data == NULL) {
			 E("%s : nfc probe fail because platform_data \
				 is NULL\n", __func__);
			 return  -ENODEV;
		 }
		 ret = cxd224x_parse_dt(&client->dev, platform_data);
		 if (ret) {
	                 E("%s : cxd224x_parse_dt fail\n", __func__);
	                 ret = -ENODEV;
	                 goto err_exit;
	         }
	} else {
		 platform_data = client->dev.platform_data;
		 if (platform_data == NULL) {
			 E("%s : nfc probe fail because platform_data \
				 is NULL\n", __func__);
			 return  -ENODEV;
		 }
	}

	/* IRQ_GPIO */
	ret = gpio_request_one(platform_data->irq_gpio, GPIOF_IN, "nfc_int");
	if (ret) {
		E("%s : request gpio%d fail\n",
			__func__, platform_data->irq_gpio);
		ret = -ENODEV;
	}
	irq_gpio_ok=1;

#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	ret = gpio_request_one(platform_data->en_gpio, GPIOF_OUT_INIT_LOW, "nfc_cen");
	if (ret)
		goto err_exit;
	en_gpio_ok=1;
        ret = gpio_direction_output(platform_data->en_gpio, 0);
        if (ret)
            return -ENODEV;
#endif

#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	ret = gpio_request_one(platform_data->rst_gpio, GPIOF_OUT_INIT_LOW, "nfc_rst");
	if (ret)
		goto err_exit;
	rst_gpio_ok=1;
        ret = gpio_direction_output(platform_data->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);
        if (ret)
            return -ENODEV;
	dev_info(&client->dev, "%s, xrst deassert\n", __func__);

#endif

	ret = gpio_request_one(platform_data->wake_gpio, GPIOF_OUT_INIT_HIGH, "nfc_wake");
	if (ret)
		goto err_exit;
	wake_gpio_ok=1;
        ret = gpio_direction_output(platform_data->wake_gpio,1);

//Clk
        ret = gpio_request(platform_data->clkreq_gpio, "nfc_clkreq_gpio");
        if (ret) {
                dev_err(&client->dev,
                        "%s: unable to request nfc clkreq gpio [%d]\n",
                        __func__, platform_data->clkreq_gpio);
                goto err_exit;
        }
        ret = gpio_direction_input(platform_data->clkreq_gpio);
        if (ret) {
                dev_err(&client->dev,
                "%s: cannot set direction for nfc clkreq gpio [%d]\n",
                __func__, platform_data->clkreq_gpio);
                goto err_exit;
        }
        I("%s : clkreq_gpio set input %d \n", __func__,ret);
        clkreq_gpio_ok = 1;
//end

	cxd224x_dev = kzalloc(sizeof(*cxd224x_dev), GFP_KERNEL);
	if (cxd224x_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	cxd224x_dev->kbuflen = MAX_BUFFER_SIZE;
        cxd224x_dev->kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
        if (!cxd224x_dev->kbuf) {
                dev_err(&client->dev,
                        "failed to allocate memory for cxd224x_dev->kbuf\n");
                ret = -ENOMEM;
                goto err_exit;
        }

	cxd224x_dev->irq_gpio = platform_data->irq_gpio;
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	cxd224x_dev->en_gpio = platform_data->en_gpio;
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	cxd224x_dev->rst_gpio = platform_data->rst_gpio;
#endif
	cxd224x_dev->wake_gpio = platform_data->wake_gpio;
	cxd224x_dev->clkreq_gpio = platform_data->clkreq_gpio;
	cxd224x_dev->client = client;
	cxd224x_dev->boot_mode = cxd224x_htc_get_bootmode();

	ret = gpio_direction_input(cxd224x_dev->irq_gpio);
	I("%s : irq_gpio set input %d \n", __func__,ret);
	ret = gpio_direction_output(cxd224x_dev->wake_gpio, 0);
	I("%s : wake_gpio set 0 %d \n", __func__,ret);

	cxd224x_info = cxd224x_dev;

	wakeup_source_init(&cxd224x_dev->wakelock, CXD224X_WAKE_LOCK_NAME);
	wakeup_source_init(&cxd224x_dev->wakelock_lp, CXD224X_WAKE_LOCK_NAME_LP);
	cxd224x_dev->users =0;

	/* init mutex and queues */
	init_waitqueue_head(&cxd224x_dev->read_wq);
	mutex_init(&cxd224x_dev->read_mutex);
	mutex_init(&cxd224x_dev->lock);
	spin_lock_init(&cxd224x_dev->irq_enabled_lock);
#if 0
	INIT_WORK(&cxd224x_dev->felica_work, cxd224x_felica_work);
	cxd224x_dev->p_queue = create_singlethread_workqueue("htc_felica");
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	if (init_wqueue(cxd224x_dev) != 0) {
		dev_err(&client->dev, "init workqueue failed\n");
		goto err_exit;
	}
#endif

	cxd224x_dev->cxd224x_device.minor = MISC_DYNAMIC_MINOR;
	cxd224x_dev->cxd224x_device.name = "cxd224x-i2c";
	cxd224x_dev->cxd224x_device.fops = &cxd224x_dev_fops;

	ret = misc_register(&cxd224x_dev->cxd224x_device);
	if (ret) {
		E("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
    	client->irq = gpio_to_irq(platform_data->irq_gpio);
	I("%s : requesting IRQ %d\n", __func__, client->irq);
	cxd224x_dev->irq_enabled = true;
	ret = request_irq(client->irq, cxd224x_dev_irq_handler,
			  IRQF_TRIGGER_FALLING, client->name, cxd224x_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	cxd224x_disable_irq(cxd224x_dev);
//clkreq	
	ret = nqx_clock_select(cxd224x_dev);
        if (ret < 0) {
                dev_err(&client->dev,
                        "%s: nqx_clock_select failed\n", __func__);
                goto err_exit;
        }
        I("%s : Clock enable %d\n", __func__, ret);
//end
	i2c_set_clientdata(client, cxd224x_dev);

	cxd224x_dev->cxd224x_class = class_create(THIS_MODULE, "SONY_FELICA");
	if (IS_ERR(cxd224x_dev->cxd224x_class)) {
		ret = PTR_ERR(cxd224x_dev->cxd224x_class);
		cxd224x_dev->cxd224x_class = NULL;
		E("%s : class_create failed\n", __func__);
		goto err_create_class;
	}

	cxd224x_dev->cxd_dev = device_create(cxd224x_dev->cxd224x_class, NULL, 0, "%s", "cxd224x");
	if (unlikely(IS_ERR(cxd224x_dev->cxd_dev))) {
		ret = PTR_ERR(cxd224x_dev->cxd_dev);
		cxd224x_dev->cxd_dev = NULL;
		E("%s : device_create failed\n", __func__);
		goto err_create_pn_device;
	}

	/* register the attributes */
	ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_debug_cmd);
	if (ret) {
		E("%s : device_create_file dev_attr_debug_cmd failed\n", __func__);
		goto err_create_pn_file;
	}

	ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_debug_enable);
	if (ret) {
		E("pn544_probe device_create_file dev_attr_debug_enable failed\n");
		goto err_create_pn_file;
	}

	ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_mfg_nfcversion);
	if (ret) {
		E("device_create_file dev_attr_mfg_nfcversion failed\n");
	}
	
    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_mfg_nfcreader);
    if (ret) {
	    E("%s : device_create_file dev_attr_mfg_nfcreader failed\n", __func__);
	    goto err_create_pn_file;
    }

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_mfg_nfccard);
    if (ret) {
	    E("%s : device_create_file dev_attr_mfg_nfccard failed\n", __func__);
	    goto err_create_pn_file;
    }
#if 0
    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_mfg_felicacard);
    if (ret) {
	    E("%s : device_create_file dev_attr_mfg_felicacard failed\n", __func__);
	    goto err_create_pn_file;
    }
#endif
    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_mfg_felicaidm);
    if (ret) {
	    E("%s : device_create_file dev_attr_mfg_felicaidm failed\n", __func__);
	    goto err_create_pn_file;
    }
	/*  register the attributes */
	ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_mfg_nfc_ctrl);
	if (ret) {
		E("pn544_probe device_create_file dev_attr_mfg_nfc_ctrl failed\n");
	}

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_hw_reset);
    if (ret) {
	    E("%s : device_create_file dev_attr_hw_reset failed\n", __func__);
	    goto err_create_pn_file;
    }
    
    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_erase_sec95);
    if (ret) {
            E("%s : device_create_file dev_attr_erase_sec95 failed\n", __func__);
            goto err_create_pn_file;
    }

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_erase_NVR);
    if (ret) {
            E("%s : device_create_file dev_attr_erase_NVR failed\n", __func__);
            goto err_create_pn_file;
    }

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_seteflash);
    if (ret) {
	    E("%s : device_create_file dev_attr_seteflash failed\n", __func__);
	    goto err_create_pn_file;
    }

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_geteflash);
    if (ret) {
	    E("%s : device_create_file dev_attr_geteflash failed\n", __func__);
	    goto err_create_pn_file;
    }

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_setreg);
    if (ret) {
            E("%s : device_create_file dev_attr_setreg failed\n", __func__);
            goto err_create_pn_file;
    }

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_getreg);
    if (ret) {
            E("%s : device_create_file dev_attr_getreg failed\n", __func__);
            goto err_create_pn_file;
    }

    ret = device_create_file(cxd224x_dev->cxd_dev, &dev_attr_chkrstnft);
    if (ret) {
	    E("%s : device_create_file dev_attr_chkrstnft failed\n", __func__);
	    goto err_create_pn_file;
    }
	watchdog_timeout = WATCHDOG_FTM_TIMEOUT_SEC; //set default timeout value 20 sec

	I("%s(), XRST in driver probe\n", __func__);
	cxd224x_hw_reset();

	I("%s, probing cxd224x driver exited successfully\n", __func__);
	return 0;

err_create_pn_file:
err_create_pn_device:
	device_unregister(cxd224x_dev->cxd_dev);
err_create_class:
	class_destroy(cxd224x_dev->cxd224x_class);
err_request_irq_failed:
	misc_deregister(&cxd224x_dev->cxd224x_device);
err_misc_register:
	mutex_destroy(&cxd224x_dev->read_mutex);
	mutex_destroy(&cxd224x_dev->lock);
err_exit:
	if(irq_gpio_ok)
		gpio_free(platform_data->irq_gpio);
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	if(en_gpio_ok)
		gpio_free(platform_data->en_gpio);
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	if(rst_gpio_ok)
		gpio_free(platform_data->rst_gpio);
#endif
	if(wake_gpio_ok)
		gpio_free(platform_data->wake_gpio);
	if(clkreq_gpio_ok)
                gpio_free(platform_data->clkreq_gpio);
	if(cxd224x_dev->kbuf)
		kfree(cxd224x_dev->kbuf);
	if(cxd224x_dev)
		kfree(cxd224x_dev);
	return ret;
}

static int cxd224x_remove(struct i2c_client *client)
{
	struct cxd224x_dev *cxd224x_dev;

	cxd224x_dev = i2c_get_clientdata(client);
	wakeup_source_trash(&cxd224x_dev->wakelock);
	wakeup_source_trash(&cxd224x_dev->wakelock_lp);
	free_irq(client->irq, cxd224x_dev);
	misc_deregister(&cxd224x_dev->cxd224x_device);
	mutex_destroy(&cxd224x_dev->read_mutex);
	mutex_destroy(&cxd224x_dev->lock);
	gpio_free(cxd224x_dev->irq_gpio);
	gpio_free(cxd224x_dev->rst_gpio);
	gpio_free(cxd224x_dev->wake_gpio);
	gpio_free(cxd224x_dev->clkreq_gpio);
	kfree(cxd224x_dev->kbuf);	
	nqx_clock_deselect(cxd224x_dev);

	kfree(cxd224x_dev);

	return 0;
}

#ifdef CONFIG_PM
static int cxd224x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cxd224x_dev *cxd224x_dev = i2c_get_clientdata(client);

	if (cxd224x_dev->users > 0 && device_may_wakeup(&client->dev)) {
		int irq = gpio_to_irq(cxd224x_dev->irq_gpio);
		enable_irq_wake(irq);
	}
	return 0;
}

static int cxd224x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cxd224x_dev *cxd224x_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		int irq = gpio_to_irq(cxd224x_dev->irq_gpio);
		disable_irq_wake(irq);
	}
	return 0;
}

static const struct dev_pm_ops cxd224x_pm_ops = {
	.suspend	= cxd224x_suspend,
	.resume		= cxd224x_resume,
};
#endif

static const struct i2c_device_id cxd224x_id[] = {
	{"cxd224x-i2c", 0},
	{}
};
static struct of_device_id cxd224x_match_table[] = {
	{ .compatible = "sony,nfccxd224x",},
	{ },
};

static struct i2c_driver cxd224x_driver = {
	.id_table = cxd224x_id,
	.probe = cxd224x_probe,
	.remove = cxd224x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cxd224x-i2c",
                .of_match_table = cxd224x_match_table,
#ifdef CONFIG_PM
		.pm	= &cxd224x_pm_ops,
#endif
	},
};

/*
 * module load/unload record keeping
 */

static int __init cxd224x_dev_init(void)
{
	return i2c_add_driver(&cxd224x_driver);
}
module_init(cxd224x_dev_init);

static void __exit cxd224x_dev_exit(void)
{
	i2c_del_driver(&cxd224x_driver);
}
module_exit(cxd224x_dev_exit);

MODULE_AUTHOR("Sony");
MODULE_DESCRIPTION("NFC cxd224x driver");
MODULE_LICENSE("GPL");
