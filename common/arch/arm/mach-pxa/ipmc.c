/*
 * linux/arch/arm/mach-pxa/ipmc.c
 *
 * Provide ioctl interface to application, to specify more detailed info and
 * change system's behaviour.
 *
 * Copyright (C) 2003-2004 Intel Corporation.
 *
 * Author:  Cain Yuan <cain.yuan@intel.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * If wana use it please use "mknod /dev/ipmc c 10 90" to create the dev file.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/cpufreq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/semaphore.h>
#include <linux/slab.h>

#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/current.h>
#include <asm/system.h>
#include <mach/ipmc.h>
#include <mach/prm.h>
#include <mach/dvfm.h>

#define DEBUG

#ifdef DEBUG
#define DPRINTK(fmt, args...) \
do { printk(KERN_DEBUG "%s: " fmt, __func__ , ## args); } while (0)
#else
#define DPRINTK(fmt, args...) \
do {} while (0)
#endif

extern unsigned int pm_sleeptime;
extern int (*event_notify) (int, int, void *, unsigned int);

int (*pipm_start_pmu) (void *) = NULL;
EXPORT_SYMBOL(pipm_start_pmu);
int (*pipm_stop_pmu) (void) = NULL;
EXPORT_SYMBOL(pipm_stop_pmu);

static struct ipme_queue ipme_queue;

static spinlock_t ipmq_lock = __SPIN_LOCK_UNLOCKED(ipmq_lock);

/* PRM related staff */
#define IPMC_CLI_NAME	"IPM Control Device"	/* name as PMU client */
#define IPMC_CLI_PRI	PRI_IPMC		/* priority as a client of PMU */
#define GROUP_COP	0			/* resource group: Core Operating Point related resources */
static unsigned int ipmc_client_id;		/* IPMC's client ID as a client of PMU */
static unsigned int rsrcs_avail;		/* resources available now */
static wait_queue_head_t cop_wait_q;		/* queue of waiting for resources available */
static void cop_event_handler(prm_event, unsigned int, void *);

static void cop_event_handler(prm_event event, unsigned int group_id,
			void *data)
{
	if (event == PRM_RES_READY) {

		DPRINTK("event PRM_RES_READY received!\n");
		/* commite resource again */
		if (prm_commit_resources(ipmc_client_id, group_id)) {
			DPRINTK("Failed to commit PRM resources again\n");
			return;
		}

		/* if the PMU resources are available again, wake up the
		 * policy maker thread
		 */
		rsrcs_avail = 1;
		wake_up_interruptible(&cop_wait_q);
	} else if (event == PRM_RES_APPROPRIATED) {
		/* if the PMU resources are appropriated, notify policy maker
		 * thread to enter sleep
		 */
		DPRINTK("event PRM_RES_APPROPRIATED received!\n");
		rsrcs_avail = 0;
	} else {
		;
	}

	return;
}

int ipmc_open(struct inode *inode, struct file *filep)
{
	int ret;

	printk(KERN_DEBUG "ipmc_open: device opened!\n");

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	/* init PRM resource wait queue */
	init_waitqueue_head(&cop_wait_q);

	/* open a session to access PRM COP resources */
	ret = prm_open_session(IPMC_CLI_PRI, IPMC_CLI_NAME,
			cop_event_handler, (void *) NULL);
	if (ret < 0) {
		printk(KERN_ERR "unable to open session of PMU access: %d\n",
				ret);
		module_put(THIS_MODULE);
		return ret;
	}
	ipmc_client_id = ret;

	/* allocate necessary COP resources */
	ret = prm_allocate_resource(ipmc_client_id, PRM_COP, GROUP_COP);
	if (ret) {
		DPRINTK("prm_allocate_resource:PRM_COP failed(%d)\n", ret);
		prm_close_session(ipmc_client_id);
		module_put(THIS_MODULE);
		return ret;
	}

	/* commit PRM resources */
	if (prm_commit_resources(ipmc_client_id, GROUP_COP)) {
		DPRINTK("PRM resources are appropriated, enter sleep...\n");
		wait_event_interruptible(cop_wait_q, (rsrcs_avail != 0));
		DPRINTK("PRM resources are available again, wake up\n");
	} else {
		rsrcs_avail = 1;
	}

	return 0;		/* success */
}

int ipmc_close(struct inode *inode, struct file *filep)
{
	printk(KERN_DEBUG "ipmc_open: device closed!\n");

	/* stop pmu */
	if (pipm_stop_pmu != NULL)
		pipm_stop_pmu();

	/* free resources */
	prm_free_resources(ipmc_client_id, GROUP_COP);

	/* close session */
	prm_close_session(ipmc_client_id);
	module_put(THIS_MODULE);

	return 0;
}

/* getting and handling events are only used by IPM */
static int ipmq_get(struct ipm_event *ipme)
{
	unsigned long flags;

	spin_lock_irqsave(&ipmq_lock, flags);

	if (!ipme_queue.len) {
		spin_unlock_irqrestore(&ipmq_lock, flags);
		return -1;
	}
	memcpy(ipme, ipme_queue.ipmes + ipme_queue.tail,
			sizeof(struct ipm_event));
	ipme_queue.len--;
	ipme_queue.tail = (ipme_queue.tail + 1) % MAX_IPME_NUM;

	spin_unlock_irqrestore(&ipmq_lock, flags);

	return 0;
}

static int ipmq_clear(void)
{
	unsigned long flags;

	spin_lock_irqsave(&ipmq_lock, flags);

	ipme_queue.head = ipme_queue.tail = 0;
	ipme_queue.len = 0;

	spin_unlock_irqrestore(&ipmq_lock, flags);

	return 0;
}

static int ipmq_put(struct ipm_event *ipme)
{
	unsigned long flags;

	spin_lock_irqsave(&ipmq_lock, flags);

	if (ipme_queue.len == MAX_IPME_NUM) {
		spin_unlock_irqrestore(&ipmq_lock, flags);
		return -1;
	}
	memcpy(ipme_queue.ipmes + ipme_queue.head, ipme,
			sizeof(struct ipm_event));
	ipme_queue.len++;
	ipme_queue.head = (ipme_queue.head + 1) % MAX_IPME_NUM;

	spin_unlock_irqrestore(&ipmq_lock, flags);

	/* wake up the waiting process */
	wake_up_interruptible(&ipme_queue.waitq);

	return 0;
}

static inline int ipmq_empty(void)
{
	return (ipme_queue.len > 0) ? 0 : 1;
}

/* Return a ipm event or get blocked until there is a event. */
ssize_t ipmc_read(struct file *filep, char __user *buf, size_t count,
		  loff_t *f_pos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct ipm_event data;
	ssize_t retval;

	if (count < sizeof(struct ipm_event))
		return -EINVAL;

	add_wait_queue(&ipme_queue.waitq, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	for (;;) {
		if (filep->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		}

		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}
		/* Get the event out. If no, blocked. */
		if (!ipmq_empty()) {
			ipmq_get(&data);
			break;
		}
		/* No events and no error, block it. */
		schedule();
	}

	/* pass the event to user space. */
	retval = copy_to_user(buf, &data, sizeof(struct ipm_event));
	if (retval) {
		retval = -EFAULT;
		goto out;
	}
	retval = sizeof(struct ipm_event);

out:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&ipme_queue.waitq, &wait);
	return retval;
}

/*
 * Write will do nothing. If need to pass down some information use ioctl
 * interface instead.
 */
ssize_t ipmc_write(struct file *filep, const char __user *buf, size_t count,
		loff_t *f_pos)
{
	return 0;
}

/* poll for ipm event */
unsigned int ipmc_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &ipme_queue.waitq, wait);
	if (!ipmq_empty())
		return POLLIN | POLLRDNORM;

	return 0;
}

int ipmc_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	unsigned int cpu_mode;

	int ret = 0;

	switch (cmd) {
	case IPMC_IOCTL_GET_FV_OP:
		{
			unsigned int op;
			struct pxa95x_fv_info param;

			ret = cop_get_cur_cop(ipmc_client_id, &op, &param);
			if (ret)
				break;

			if (copy_to_user((unsigned long *) arg, &op,
						sizeof(op))) {
				ret = -EFAULT;
				break;
			}
			break;
		}
	case IPMC_IOCTL_SET_FV_OP:
		{
			struct ipmc_ioctl_setfvop setop;
			if (copy_from_user(&setop, (unsigned int *) arg,
						sizeof(setop))) {
				printk(KERN_ERR "Error from copy_from_user.\n");
				ret = -EFAULT;
				break;
			}
			ret = cop_set_cop(ipmc_client_id, setop.op,
					setop.mode);

			break;
		}
	case IPMC_IOCTL_GET_FV_OP_COUNT:
		{
			unsigned int count;
			count = cop_get_num_of_cops();
			if (copy_to_user((unsigned long *) arg, &count,
						sizeof(count)))
				ret = -EFAULT;
			break;
		}
	case IPMC_IOCTL_GET_FV_OP_INFO:
		{
			struct ipmc_ioctl_getfvopinfo getinfo;
			if (copy_from_user(&getinfo, (unsigned int *) arg,
						sizeof(getinfo))) {
				printk(KERN_ERR "Error from copy_from_user.\n");
				ret = -EFAULT;
				break;
			}
			ret = cop_get_cop(ipmc_client_id, getinfo.op,
					&(getinfo.info));
			if (ret)
				break;
			if (copy_to_user((unsigned long *) arg, &getinfo,
						sizeof(getinfo)))
				ret = -EFAULT;
			break;
		}

	case IPMC_IOCTL_SET_CPU_MODE:
		if (copy_from_user(&cpu_mode, (unsigned int *) arg,
					sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}
		pm_suspend(cpu_mode);
		break;

	case IPMC_IOCTL_SET_SLEEPTIME:
		if (copy_from_user(&pm_sleeptime, (unsigned int *) arg,
					sizeof(unsigned int)))
			ret = -EFAULT;
		break;

	case IPMC_IOCTL_GET_SLEEPTIME:
		if (copy_to_user((unsigned int *) arg, &pm_sleeptime,
					sizeof(unsigned int)))
			ret = -EFAULT;
		break;

		/* get rid of those event before. */
	case IPMC_IOCTL_CLEAR_EVENTLIST:
		ipmq_clear();
		break;

	case IPMC_IOCTL_STARTPMU:
		{
			unsigned int len;
			void *pmu_arg = NULL;
			/* get size from first word */
			if (copy_from_user(&len, (unsigned int *) arg,
						sizeof(unsigned int))) {
				ret = -EFAULT;
				break;
			}
			if (len > 0) {
				pmu_arg = kmalloc(len, GFP_KERNEL);
				if (pmu_arg == NULL) {
					printk(KERN_ERR "can't allocate enough"
							"memory\n");
					ret = -ENOMEM;
					break;
				}
				if (copy_from_user(pmu_arg,
						(unsigned int *) arg, len)) {
					ret = -EFAULT;
					kfree(pmu_arg);
					break;
				}
			} else {
				ret = -EFAULT;
				break;
			}
			if (pipm_start_pmu != NULL)
				pipm_start_pmu(pmu_arg);
			kfree(pmu_arg);
			break;
		}

	case IPMC_IOCTL_STOPPMU:
		if (pipm_stop_pmu != NULL)
			pipm_stop_pmu();
		break;

	case IPMC_IOCTL_SET_WAKEUPSRC:
		if (copy_from_user(&wakeup_src, (pm_wakeup_src_t *) arg,
					sizeof(pm_wakeup_src_t)))
			ret = -EFAULT;
		break;

	case IPMC_IOCTL_GET_WAKEUPSRC:
		if (copy_to_user((pm_wakeup_src_t *) arg, &wakeup_src,
					sizeof(pm_wakeup_src_t)))
			ret = -EFAULT;
		break;

	case IPMC_IOCTL_SET_DEEPIDLE:
		if (copy_from_user(&enable_deepidle, (unsigned int *) arg,
					sizeof(unsigned int)))
			ret = -EFAULT;
		break;

#if defined(CONFIG_FB_PXA) && defined(DEBUG_BKLIGHT)
	case IPMC_IOCTL_GET_BKLIGHT:
		{
			int bright;
			bright = pxafb_get_backlight();
			if (copy_to_user((int *) arg, &bright, sizeof(int)))
				ret = -EFAULT;
		}
		break;

	case IPMC_IOCTL_SET_BKLIGHT:
		{
			int bright;
			if (copy_from_user(&bright, (int *) arg, sizeof(int)))
				ret = -EFAULT;
			pxafb_set_backlight(bright);
		}
		break;
#endif

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	if (ret == EACCES || !rsrcs_avail) {
		/* if PRM resources are appropriated, let IPM policy maker
		 * thread enter sleep
		 */
		DPRINTK("PRM resources are appropriated, enter sleep...\n");
		rsrcs_avail = 0;
		wait_event_interruptible(cop_wait_q, (rsrcs_avail != 0));
		DPRINTK("PRM resources are available again, wake up\n");
	}

	return ret;
}

static const struct file_operations ipmc_fops = {
	.owner = THIS_MODULE,
	.open = ipmc_open,
	.read = ipmc_read,
	.write = ipmc_write,
	.poll = ipmc_poll,
	.ioctl = ipmc_ioctl,
	.release = ipmc_close,
};

static struct miscdevice ipmc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ipmc",
	.fops = &ipmc_fops
};

/*
 * Below is the event list maintain functions, the same as keypad.c
 */
static int ipmq_init(void)
{
	/* init ipme queue */
	ipme_queue.head = ipme_queue.tail = 0;
	ipme_queue.len = 0;
	init_waitqueue_head(&ipme_queue.waitq);
	return 0;
}

static int __init ipmc_init(void)
{
	int rc;

	/* Clear event queue. */
	ipmq_init();

	rc = misc_register(&ipmc_misc_device);
	if (rc != 0) {
		printk(KERN_ERR "Could not register device ipmc, res = %d.\n",
				rc);
		return -EBUSY;
	}
	printk(KERN_INFO "Register device ipmc successgul.\n");

	event_notify = ipm_event_notify;
	return 0;
}

static void ipmc_exit(void)
{
	event_notify = NULL;
	misc_deregister(&ipmc_misc_device);
}

/*
 * IPM event can be posted by this function.
 * If we need to do some pre-processing of those events we can add code here.
 * Also attach the processed result to info.
 */
int ipm_event_notify(int type, int kind, void *info, unsigned int info_len)
{
	struct ipm_event events;
	int len = 0;

	if (info_len > INFO_SIZE)
		len = INFO_SIZE;
	else if ((info_len < INFO_SIZE) && (info_len > 0))
		len = info_len;
	memset(&events, 0, sizeof(struct ipm_event));
	events.type = type;
	events.kind = kind;
	if ((len > 0) && (info != NULL)) {
		memcpy(events.info, info, len);
	}
	ipmq_put(&events);

	return len;
}
EXPORT_SYMBOL(ipm_event_notify);

module_init(ipmc_init);
module_exit(ipmc_exit);

MODULE_DESCRIPTION("IPM control device");
MODULE_LICENSE("GPL");
