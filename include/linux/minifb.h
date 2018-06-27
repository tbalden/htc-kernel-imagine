#ifndef _MINI_FB_H_
#define _MINI_FB_H_

#include <linux/ioctl.h>
#include <linux/types.h>
#include <uapi/linux/minifb.h>

#ifdef CONFIG_FB_MINIFB

#define MINIFB_NOREPEAT 0
#define MINIFB_REPEAT   1

int minifb_lockbuf(void **, unsigned long *, int);
void minifb_unlockbuf(void);
#else
int minifb_lockbuf(void **, unsigned long *)
{
	return -ENODEV;
}

void minifb_unlockbuf(void);
{

}

#endif /* CONFIG_FB_MINIFB */
#endif /* _MINI_FB_H_ */

