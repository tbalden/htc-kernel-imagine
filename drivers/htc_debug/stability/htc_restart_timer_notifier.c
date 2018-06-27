#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/stacktrace.h>
#include <asm/stacktrace.h>

#define RESTART_TIMER_TIMEOUT 12

static char* blocker = NULL;

#ifdef CONFIG_STACKTRACE
static int save_trace(struct stackframe *frame, void *d)
{
	struct stack_trace *trace = d;
	unsigned long addr = frame->pc;

	trace->entries[trace->nr_entries++] = addr;
	return trace->nr_entries >= trace->max_entries;
}

static void print_block_stack(struct task_struct *tsk)
{
	static char blocker_buf[64];
	unsigned long addrs[32];
	struct stack_trace trace;
	int i;
	struct stackframe frame;

	if (!try_get_task_stack(tsk)) {
		pr_err("Failed to get tsk %p\n", tsk);
		return;
	}

	trace.nr_entries = 0;
	trace.max_entries = ARRAY_SIZE(addrs);
	trace.entries = addrs;

	frame.fp = thread_saved_fp(tsk);
	frame.sp = thread_saved_sp(tsk);
	frame.pc = thread_saved_pc(tsk);

	walk_stackframe(tsk, &frame, save_trace, &trace);
	put_task_stack(tsk);

	pr_err("Call trace:\n");
	for (i = 0; i < trace.nr_entries; i++) {
		unsigned long addr = addrs[i];
		char symname[64];
		snprintf(symname, sizeof(symname), "%pS", (void*) addr);
		pr_emerg("%2d: [<%p>] %s\n", i, (void *) addr, symname);
		/* i!=0 to skip 1st `__switch_to' */
		if (i && !blocker && !in_sched_functions(addr)) {
			strcpy(blocker_buf, symname);
			blocker = blocker_buf;
		}
	}
}
#endif

static void restart_timeout(unsigned long data)
{
	pr_emerg("\n--------------Restart timer timeout----------------\n");
#ifdef CONFIG_STACKTRACE
	print_block_stack((struct task_struct *) data);
#else
	show_stack((void *) data,NULL);
#endif
	pr_emerg("### Show All Blocked State ###\n");
	show_state_filter(TASK_UNINTERRUPTIBLE);
	panic("%s: %s", __func__, blocker ? : "unknown-blocker");
}

int restart_timer_add(struct notifier_block *this,unsigned long code,void *data)
{
	static struct timer_list *restart_timer;
	struct task_struct *tsk;

	tsk = get_current();
	restart_timer = kmalloc(sizeof(struct timer_list),GFP_KERNEL);
	init_timer(restart_timer);
	restart_timer->function = restart_timeout;
	restart_timer->expires = jiffies + HZ * RESTART_TIMER_TIMEOUT;
	restart_timer->data = (unsigned long) tsk;
	add_timer(restart_timer);

	return NOTIFY_DONE;
}

static struct notifier_block restart_notifier ={
	.notifier_call = restart_timer_add,
};

static int __init timer_notifier_add(void)
{
	register_reboot_notifier(&restart_notifier);//add a notifier block in reboot notifier list

	return 0;
}
module_init(timer_notifier_add);
