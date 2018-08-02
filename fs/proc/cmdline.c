#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/setup.h>

#if 1
#include <linux/slab.h>
#include <linux/spinlock.h>

static bool done = false;

static DEFINE_SPINLOCK(show_lock);
static bool magisk = true;

extern bool is_magisk(void);
extern bool is_magisk_sync(void);
extern void init_custom_fs(void);

#endif

static char new_command_line[COMMAND_LINE_SIZE];
char command_line_vanilla[COMMAND_LINE_SIZE];
char* get_command_line_vanilla(void) {
	return command_line_vanilla;
}
EXPORT_SYMBOL(get_command_line_vanilla);

static int cmdline_proc_show(struct seq_file *m, void *v)
{
#if 1
	spin_lock(&show_lock);
	if (done) {
	} else {
		magisk = is_magisk();
		done = true;
	}
	spin_unlock(&show_lock);
#endif
	seq_printf(m, "%s\n", new_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if 0
static void remove_flag(char *cmd, const char *flag)
{
	char *start_addr, *end_addr;

	/* Ensure all instances of a flag are removed */
	while ((start_addr = strstr(cmd, flag))) {
		end_addr = strchr(start_addr, ' ');
		if (end_addr)
			memmove(start_addr, end_addr + 1, strlen(end_addr));
		else
			*(start_addr - 1) = '\0';
	}
}

static void remove_safetynet_flags(char *cmd)
{
	remove_flag(cmd, "androidboot.enable_dm_verity=");
	remove_flag(cmd, "androidboot.secboot=");
	remove_flag(cmd, "androidboot.verifiedbootstate=");
	remove_flag(cmd, "androidboot.veritymode=");
}
#endif

#if 1
static void replace_flag(char *cmd, const char *flag, const char *new_flag)
{
#if 1
	char *start_addr;

	/* Ensure all instances of a flag are removed */
	while ((start_addr = strstr(cmd, flag))) {
                                while ((*new_flag)!='\0') {
                                        *start_addr = *new_flag;
                                        new_flag++;
                                        start_addr++;
                                }
	}
#endif
}

static void replace_safetynet_flags(char *cmd)
{
//	replace_flag(cmd, "androidboot.enable_dm_verity=");
//	replace_flag(cmd, "androidboot.secboot=");
//	replace_flag(cmd, "androidboot.veritymode=enforcing");

	replace_flag(cmd, "androidboot.verifiedbootstate=orange" , "androidboot.verifiedbootstate=green ");

// you shouldn't replace vbmeta lock state: won't boot on an unlocked vbmeta
//	replace_flag(cmd, "androidboot.vbmeta.device_state=unlocked" , 
//			  "androidboot.vbmeta.device_state=locked  ");
}
#endif

static int __init proc_cmdline_init(void)
{
#if 1
	init_custom_fs();
#endif
	strcpy(new_command_line, saved_command_line);
	strcpy(command_line_vanilla, saved_command_line);

	/*
	 * Remove various flags from command line seen by userspace in order to
	 * pass SafetyNet CTS check.
	 */
#if 0
	remove_safetynet_flags(new_command_line);
#else
	/* replace boot state flags for CTS check */
	replace_safetynet_flags(new_command_line);
#endif

	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
