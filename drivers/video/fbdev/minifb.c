#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <linux/msm_ion.h>
#include <linux/minifb.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>

#define DEVICE_NAME "minifb"

static int major;       /*  default is dynamic major device number */
static struct class *minifb_class;
static struct device *minifb_dev;

struct minifb_data
{
	struct minifb_req buf_info;
	struct list_head entry;
	struct ion_handle *ionhdl;
	int state;
	int info;
};

struct minifb_ctrl
{
	struct mutex lock;
	struct kref refcnt;

	struct list_head busy_queue;
	struct list_head free_queue;
	struct list_head ready_queue;
	struct minifb_data *retired;
	wait_queue_head_t wait_q; /* TBD */
	u32 state;
	uint32_t width;
	uint32_t height;
	uint32_t frame_cnt;
	uint32_t drop_cnt;
	uint32_t retry_cnt;
	uint32_t lock_cnt;
	unsigned long log_time;

	struct ion_client *iclient;
	struct dentry *debug_root;
};

static DEFINE_MUTEX(minifb_lock);
static struct minifb_ctrl *sMinifb_ctrl; // Access to sMinifb_ctrl is protected by minifb_lock

static struct minifb_ctrl *get_ctrl(void)
{
	struct minifb_ctrl *ctrl;

	mutex_lock(&minifb_lock);

	ctrl = sMinifb_ctrl;
	if (ctrl) {
		kref_get(&ctrl->refcnt);
	}

	mutex_unlock(&minifb_lock);

	return ctrl;
}

static void minifb_release_ctrl_locked(struct kref *ref)
{
	struct minifb_data *node, *tmp;
	struct minifb_ctrl *ctrl;

	pr_info("%s\n", __func__);

	ctrl = container_of(ref, struct minifb_ctrl, refcnt);

	debugfs_remove_recursive(ctrl->debug_root);

	list_for_each_entry_safe(node, tmp, &ctrl->busy_queue, entry) {
		pr_warn("%s: Should not here! node#%d\n", __func__, node->info);
		ion_unmap_kernel(ctrl->iclient, node->ionhdl);
		list_del(&node->entry);
		ion_free(ctrl->iclient, node->ionhdl);
		kfree(node);
	}

	list_for_each_entry_safe(node, tmp, &ctrl->ready_queue, entry) {
		list_del(&node->entry);
		ion_free(ctrl->iclient, node->ionhdl);
		kfree(node);
	}

	list_for_each_entry_safe(node, tmp, &ctrl->free_queue, entry) {
		list_del(&node->entry);
		ion_free(ctrl->iclient, node->ionhdl);
		kfree(node);
	}

	ion_client_destroy(ctrl->iclient);

	kfree(ctrl);
	sMinifb_ctrl = NULL;
}

static void put_ctrl(struct minifb_ctrl *ctrl)
{
	mutex_lock(&minifb_lock);

	kref_put(&ctrl->refcnt, minifb_release_ctrl_locked);

	mutex_unlock(&minifb_lock);
}

static ssize_t minifb_frame_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int ret = 0, copy_len = 0;
	static void *start = NULL;
	static unsigned long size = 0;
	int count_int = count;

	if (size && (*ppos >= size)) {
		return 0;
	}

	if (start == NULL) {
		char *ptr;
		if (minifb_lockbuf(&start, &size, MINIFB_NOREPEAT) < 0)
			return 0;
		ptr = start;
	}

	copy_len = size - *ppos;
	if (copy_len > count)
		copy_len = count;
	ret = copy_to_user(buff, start + *ppos, copy_len);
	pr_debug("%s: dump from %p with %lld/%lu bytes, count=%d, copylen=%d, ret = %d\n",
		__func__, start, *ppos, size, count_int, copy_len, ret);

	*ppos += copy_len;

	if (*ppos >= size) {
		minifb_unlockbuf();
		start = NULL;
	}

	return copy_len;
}

static const struct file_operations minifb_frame_fops = {
	.read = minifb_frame_read,
};

static ssize_t minifb_info_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int tot = 0;

	struct minifb_ctrl *fbctrl = get_ctrl();

	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	tot = snprintf(buff, count, "%dx%d, %d frame, %d drop, %d consume, %d again\n",
		fbctrl->width, fbctrl->height,
		fbctrl->frame_cnt, fbctrl->drop_cnt, fbctrl->lock_cnt, fbctrl->retry_cnt);
	if (*ppos >= tot)
		tot = 0;
	*ppos += tot;

	put_ctrl(fbctrl);
	return tot;
}

static const struct file_operations minifb_info_fops = {
	.read = minifb_info_read,
};

static int minifb_init(struct minifb_ctrl * const ctrl, struct minifb_session * const sess)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	mutex_lock(&ctrl->lock);

	ctrl->state = 0;
	ctrl->width = sess->width;
	ctrl->height = sess->height;
	ctrl->frame_cnt = 0;
	ctrl->drop_cnt = 0;
	ctrl->retry_cnt = 0;
	ctrl->lock_cnt = 0;
	ctrl->log_time = 0;

	mutex_unlock(&ctrl->lock);
	pr_info("%s done\n", __func__ );

	return ret;
}

static int minifb_terminate(struct minifb_ctrl * const ctrl, struct minifb_session * const sess)
{
	pr_info("%s\n", __func__);

	return 0;
}

static int minifb_queuebuf(struct minifb_ctrl * const ctrl, struct minifb_req * const data)
{
	static int sCount = 0;
	struct minifb_data *node, *tmp, *tmp2;

	if (!ctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s\n", __func__);
	/* empty ready list and put new buf into ready list */
	node = kzalloc(sizeof(struct minifb_data), GFP_KERNEL);
	if (!node) {
		pr_err("%s: kzalloc fail\n", __func__);
		return -ENOMEM;
	}

	memcpy(&node->buf_info, data, sizeof(node->buf_info));
	node->ionhdl = ion_import_dma_buf_fd(ctrl->iclient, node->buf_info.memory_id);
	if (IS_ERR_OR_NULL(node->ionhdl)) {
		pr_warn("%s: import ion buf fail, fd=%d\n", __func__, node->buf_info.memory_id);
	}
	node->info = sCount++;

	mutex_lock(&ctrl->lock);
	if (!list_empty(&ctrl->ready_queue)) {
		list_for_each_entry_safe(tmp, tmp2, &ctrl->ready_queue, entry) {
			list_move(&tmp->entry, &ctrl->free_queue);
			ctrl->drop_cnt++;
			pr_debug("%s: drop frame#%d from ready queue\n", __func__, node->info);
		}
	}

	pr_debug("%s: queue frame#%d\n", __func__, node->info);
	list_add_tail(&node->entry, &ctrl->ready_queue);
	ctrl->frame_cnt++;

	if (time_after(jiffies, ctrl->log_time + 5 * HZ) || !ctrl->log_time) {
		pr_info("[MiniFB]: screen size=%dx%d, stat: %d frame, %d drop, "
			"%d consume, %d again\n",
			ctrl->width, ctrl->height,
			ctrl->frame_cnt, ctrl->drop_cnt,
			ctrl->lock_cnt, ctrl->retry_cnt);

		ctrl->log_time = jiffies;
	}

	mutex_unlock(&ctrl->lock);

	return 0;
}

/* blocking/non-blocking operation */
static int minifb_dequeuebuf(struct minifb_ctrl * const ctrl, struct minifb_req * const data)
{
	int ret = 0;

	if (!ctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s\n", __func__);
	mutex_lock(&ctrl->lock);

	/* find a buf from free list and return it. */
	if (!list_empty(&ctrl->free_queue)) {
		struct minifb_data *node;

		node = list_first_entry(&ctrl->free_queue, struct minifb_data, entry);
		pr_debug("%s: dequeue frame#%d\n", __func__, node->info);
		list_del(&node->entry);
		memcpy(data, &node->buf_info, sizeof(*data));
		ion_free(ctrl->iclient, node->ionhdl);
		if (node == ctrl->retired)
			ctrl->retired = NULL;
		kfree(node);
	} else {
		ret = -EBUSY;
	}

	mutex_unlock(&ctrl->lock);

	return ret;
}

int minifb_lockbuf(void **vaddr, unsigned long *ptr_size, int repeat)
{
	int ret = 0;
	struct minifb_data *node;
	struct minifb_ctrl *fbctrl = get_ctrl();

	/* move buf in ready list to busy state */
	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	if (!list_empty(&fbctrl->busy_queue)) {
		pr_err("%s: already lock buf\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s:\n", __func__);
	mutex_lock(&fbctrl->lock);

	*vaddr = NULL;
	*ptr_size = 0;
	if (!list_empty(&fbctrl->ready_queue)) {
		node = list_first_entry(&fbctrl->ready_queue, struct minifb_data, entry);
		list_move(&node->entry, &fbctrl->busy_queue);
		pr_debug("%s: lock frame#%d from fd%d\n", __func__, node->info, node->buf_info.memory_id);

		/* map ion memory */
		*vaddr = ion_map_kernel(fbctrl->iclient, node->ionhdl);
		ion_handle_get_size(fbctrl->iclient, node->ionhdl, ptr_size);
		fbctrl->lock_cnt++;
	} else if (repeat && fbctrl->retired) {
		node = fbctrl->retired;
		fbctrl->retired = NULL;
		list_move(&node->entry, &fbctrl->busy_queue);
		pr_debug("%s: lock frame#%d from retired fd%d\n", __func__, node->info, node->buf_info.memory_id);

		/* map ion memory */
		*vaddr = ion_map_kernel(fbctrl->iclient, node->ionhdl);
		ion_handle_get_size(fbctrl->iclient, node->ionhdl, ptr_size);
		fbctrl->lock_cnt++;
	} else {
		pr_debug("%s: no buffer\n", __func__);
		fbctrl->retry_cnt++;
		ret = -EAGAIN;
	}

	mutex_unlock(&fbctrl->lock);
	put_ctrl(fbctrl);

	return ret;
}

void minifb_unlockbuf(void)
{
	struct minifb_data *node;
	struct minifb_ctrl *fbctrl = get_ctrl();

	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return;
	}

	pr_debug("%s:\n", __func__);
	mutex_lock(&fbctrl->lock);

	/* move fb buf from busy state to free list */
	if (!list_empty(&fbctrl->busy_queue)) {
		node = list_first_entry(&fbctrl->busy_queue, struct minifb_data, entry);
		fbctrl->retired = node;
		pr_debug("%s: unlock node#%d\n", __func__, node->info);
		list_move_tail(&node->entry, &fbctrl->free_queue);
		ion_unmap_kernel(fbctrl->iclient, node->ionhdl);
	} else {
		pr_warn("%s: no buffer was in busy!\n", __func__);
	}

	mutex_unlock(&fbctrl->lock);
	put_ctrl(fbctrl);

	return;
}

static long minifb_ioctl_handler(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -ENOSYS;
	struct minifb_session sess;
	struct minifb_req data;
	void __user *argp = (void __user *)arg;
	struct minifb_ctrl * const ctrl = file->private_data;

	switch (cmd) {
	case MINIFB_INIT:
		ret = copy_from_user(&sess, argp, sizeof(sess));
		if (ret)
			return ret;
		ret = minifb_init(ctrl, &sess);
		if (!ret)
			ret = copy_to_user(argp, &sess, sizeof(sess));
		break;

	case MINIFB_TERMINATE:
		ret = copy_from_user(&sess, argp, sizeof(sess));
		if (ret)
			return ret;
		ret = minifb_terminate(ctrl, &sess);
		break;

	case MINIFB_QUEUE_BUFFER:
		ret = copy_from_user(&data, argp, sizeof(data));
		if (ret)
			return ret;
		ret = minifb_queuebuf(ctrl, &data);
		break;

	case MINIFB_DEQUEUE_BUFFER:
		ret = copy_from_user(&data, argp, sizeof(data));
		if (ret)
			return ret;
		ret = minifb_dequeuebuf(ctrl, &data);

		if (!ret)
			ret = copy_to_user(argp, &data, sizeof(data));
		break;

	default:
		break;
	}

	return ret;
}

static int minifb_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct minifb_ctrl* ctrl = NULL;

	pr_info("%s\n", __func__);
	mutex_lock(&minifb_lock);

	ctrl = sMinifb_ctrl;
	if (!ctrl) {
		ctrl = kzalloc(sizeof(struct minifb_ctrl), GFP_KERNEL);
		if (!ctrl) {
			pr_err("minifb_ctrl allocate fail\n");
			goto err;
		}

		mutex_init(&ctrl->lock);
		kref_init(&ctrl->refcnt);

		INIT_LIST_HEAD(&ctrl->free_queue);
		INIT_LIST_HEAD(&ctrl->busy_queue);
		INIT_LIST_HEAD(&ctrl->ready_queue);
		init_waitqueue_head(&ctrl->wait_q);

		/* FIXME: get rid of msm_ion_.. */
		ctrl->iclient = msm_ion_client_create("minifb");

		ctrl->debug_root = debugfs_create_dir("minifb", NULL);
		if (IS_ERR_OR_NULL(ctrl->debug_root)) {
			pr_err("debugfs_create_dir fail, error %ld\n",
			       PTR_ERR(ctrl->debug_root));
			ret = -ENODEV;
			kfree(ctrl);
			goto err;
		}

		debugfs_create_file("framedump", 0440, ctrl->debug_root, NULL, &minifb_frame_fops);
		debugfs_create_file("info", 0440, ctrl->debug_root, NULL, &minifb_info_fops);

		sMinifb_ctrl = ctrl;
	} else {
		pr_warn("minifb already initialized\n");
		kref_get(&ctrl->refcnt);
	}

	file->private_data = ctrl;

err:
	mutex_unlock(&minifb_lock);
	return ret;
}

static int  minifb_release(struct inode *inode, struct file *file)
{
	struct minifb_ctrl * const ctrl = file->private_data;
	if (!ctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	put_ctrl(ctrl);
	return 0; // SUCCESS
}

static const struct file_operations minifb_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = minifb_ioctl_handler,
	.open =		minifb_open,
	.release =	minifb_release,
};

static int __init minifb_register(void)
{
	int ret;
	ret = register_chrdev(major, DEVICE_NAME, &minifb_fops);
	if (ret < 0) {
		printk("unable to get major %d for %s devs\n", major, DEVICE_NAME);
		goto reg_err;
	}

	if (major == 0) {
		major = ret;
	}

	minifb_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(minifb_class)) {
		ret = PTR_ERR(minifb_class);
		printk(KERN_WARNING "Unable to create %s class; errno = %d\n", DEVICE_NAME, ret);
		goto class_err;
	}

	/* register your own device in sysfs, and this will cause udevd to create corresponding device node */
	minifb_dev = device_create(minifb_class, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(minifb_dev)) {
		ret = PTR_ERR(minifb_dev);
		printk(KERN_WARNING "Unable to create device for %s; errno = %d\n", DEVICE_NAME, ret);
		goto dev_err;
	}

	return 0;

dev_err:
	minifb_dev = NULL;
	class_destroy(minifb_class);
class_err:
	minifb_class = NULL;
	unregister_chrdev(major, "minifb");
reg_err:
	major = 0;
	return ret;
}

static void __exit minifb_unregister(void)
{
	if (major) {
		device_destroy(minifb_class, major);
		class_destroy(minifb_class);
		unregister_chrdev(major, "minifb");
	}
}

module_init(minifb_register);
module_exit(minifb_unregister);
