/*
 * DVFM Statistic Driver
 *
 * Copyright (C) 2007 Marvell Corporation
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/notifier.h>
#include <linux/jiffies.h>
#include <linux/relay.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <mach/dvfm.h>
#include <mach/hardware.h>
#include <mach/pxa95x_dvfm.h>

#define OP_NUM			20
#define JIFFIES_VALVE		2

#define EVENT_PAGE_NUM		32
#define OP_STATS_PAGE_NUM	16
#define EVENT_ARRAY_SIZE	(EVENT_PAGE_NUM * PAGE_SIZE)
#define OP_STATS_ARRAY_SIZE	(OP_STATS_PAGE_NUM * PAGE_SIZE)

#define EVENT_BUF_LEN		64
#define STATS_BUF_LEN		256

struct op_switch {
	unsigned int last_tick;
	unsigned int min_tick;
	unsigned int max_tick;
	unsigned int timestamp;
	unsigned int count;
};

static struct op_switch op_switch[OP_NUM][OP_NUM];

struct event_info {
	unsigned int event_idx;
	unsigned int op_idx;
};

struct event_type {
	unsigned int event;	/* high halfword is src, low is dest */
	unsigned int timestamp;	/* record time when the event happen */
};

struct op_stats_type {
	unsigned int timestamp;
	unsigned int op_idx;
	unsigned int runtime;
	unsigned int idletime;
	unsigned int overflow;	/* high halfword is run, low is idle */
};

struct op_cycle_type {
	unsigned int op_idx;
	unsigned int runtime;
	unsigned int idletime;
	unsigned int count;
};

/* relayfs interface */
static struct rchan *event_chan, *op_stats_chan;

/* Run/Idle state change, OPs change are all stored in event table. */
static struct event_type *event_table_head, *event_p;
/* Detail time cost on all OPs are stored in op_stats table */
static struct op_stats_type *op_stats_table_head, *op_stats_p;

static struct event_info event_array[2 * OP_NUM];
static int event_num;		/* stores the number of all events */

/* Indicates the first timeslot should be skipped */
static int dvfm_timeslot_init;

/* Total ticks cost on all OPs. */
static struct op_cycle_type op_ticks_array[OP_NUM];
static spinlock_t stats_lock = __SPIN_LOCK_UNLOCKED(stats_lock);

extern int cur_op;

static int cap_init(void);
static int cap_deinit(void);
static void write_event_chan(void);
static void write_stats_chan(void);

/* Interface under SYSFS */

/* Display duty cycles on all operating points */
static ssize_t duty_cycle_show(struct sys_device *sys_dev,
			       struct sysdev_attribute *attr, char *buf)
{
	int len, i;
	u64 total_ticks;
	unsigned int result, fraction, shift;
	struct dvfm_md_opt *temp_op;
	struct op_info *op_entry = NULL;
	char op_name[OP_NAME_LEN];

	total_ticks = 0;
	for (i = 0; i < (event_num / 2); i++) {
		total_ticks += op_ticks_array[i].runtime
		    + op_ticks_array[i].idletime;
	}
	if (total_ticks == 0) {
		len = sprintf(buf, "No OP change, no duty cycle info\n");
		return len;
	}
	len = sprintf(buf, "Duty cycle of operating point list:\n");

	result = total_ticks >> 32;
	if (result) {
		shift = fls(result);
		total_ticks >>= shift;
	} else
		shift = 0;

	for (i = 0; i < (event_num / 2); i++) {
		read_lock(&dvfm_op_list->lock);
		/* op list shouldn't be empty because op_nums is valid */
		list_for_each_entry(op_entry, &dvfm_op_list->list, list) {
			if (op_entry->index == i) {
				temp_op = (struct dvfm_md_opt *)(op_entry->op);
				strcpy(op_name, temp_op->name);
			}
		}
		read_unlock(&dvfm_op_list->lock);
		len += sprintf(buf + len, "OP %2d [%10s]  ", i, op_name);

		/* The following algorithm prints a float number
			with an accuracy of 2 digits after the dot */
		result = (unsigned int)div_u64_rem(
					(((u64)op_ticks_array[i].runtime)*100)
						>> shift,
					(unsigned int)total_ticks,
					&fraction);
		fraction = fraction * 100 / (unsigned int)total_ticks;
		len += sprintf(buf + len, "run:%2u.", result);
		if (fraction < 10)
			len += sprintf(buf + len, "0");
		len += sprintf(buf + len, "%u%%   ", fraction);

		result = (unsigned int) div_u64_rem(
					(((u64)op_ticks_array[i].idletime)*100)
						>> shift,
					(unsigned int)total_ticks,
					&fraction);
		fraction = fraction * 100 / (unsigned int)total_ticks;
		len += sprintf(buf + len, "idle:%2u.", result);
		if (fraction < 10)
			len += sprintf(buf + len, "0");
		len += sprintf(buf + len, "%u%%\n", fraction);
	}

	return len;
}

SYSDEV_ATTR(duty_cycle, 0444, duty_cycle_show, NULL);

/* Display costed time on all operating points */
static ssize_t ticks_show(struct sys_device *sys_dev,
			  struct sysdev_attribute *attr, char *buf)
{
	int len, i;
	struct dvfm_md_opt *temp_op;
	struct op_info *op_entry = NULL;
	char op_name[OP_NAME_LEN];

	len = sprintf(buf, "\nTicks of operating point list:\n\n");

	len += sprintf(buf + len, "OP#   |  OP name  | run ticks |"
		" idle ticks | run second | idle second |   count\n");

	len += sprintf(buf + len, "-------------------------"
		"---------------------------------------------------------\n");
	for (i = 0; i < (event_num / 2); i++) {
		read_lock(&dvfm_op_list->lock);
		/* op list shouldn't be empty because op_nums is valid */
		list_for_each_entry(op_entry, &dvfm_op_list->list, list) {
			if (op_entry->index == i) {
				temp_op = (struct dvfm_md_opt *)(op_entry->op);
				strcpy(op_name, temp_op->name);
			}
		}
		read_unlock(&dvfm_op_list->lock);

		len += sprintf(buf + len,
			"OP %2d | %9s |  %8u | %10u | %10u |  %10u | %10u\n", i,
			op_name,
			op_ticks_array[i].runtime,
			op_ticks_array[i].idletime,
			dvfm_driver->ticks_to_sec(op_ticks_array[i].runtime),
			dvfm_driver->ticks_to_sec(op_ticks_array[i].idletime),
			op_ticks_array[i].count);
	}
	return len;
}

SYSDEV_ATTR(ticks, 0444, ticks_show, NULL);

/* Display costed time on switching operating point */
static ssize_t switch_time_show(struct sys_device *sys_dev,
				struct sysdev_attribute *attr, char *buf)
{
	int i, j, len, op_num;

	/* event_num is double size of op_num */
	op_num = event_num / 2;
	len = sprintf(buf, "Time cost on switching operating point:");
	len += sprintf(buf + len, "(Unit is usec)\n");
	len += sprintf(buf + len, "(Left is source OP, right is dest OP)\n");
	for (i = 0; i < op_num; i++) {
		len += sprintf(buf + len, "\tOP%d", i);
	}
	len += sprintf(buf + len, "\n");
	for (i = 0; i < op_num; i++) {
		len += sprintf(buf + len, "OP%d\t", i);
		for (j = 0; j < op_num; j++) {
			len += sprintf(buf + len, "%d(%d,%d)\t",
				dvfm_driver->ticks_to_usec(
					op_switch[i][j].last_tick),
				dvfm_driver->ticks_to_usec(
					op_switch[i][j].min_tick),
				dvfm_driver->ticks_to_usec(
					op_switch[i][j].max_tick));
		}
		len += sprintf(buf + len, "\n");
	}
	return len;
}

SYSDEV_ATTR(switch_time, 0444, switch_time_show, NULL);

/* Re-collect static information */
static ssize_t stats_store(struct sys_device *sys_dev,
			   struct sysdev_attribute *attr, const char *buf,
			   size_t len)
{
	unsigned int cap_flag;

	sscanf(buf, "%u", &cap_flag);
	if (cap_flag == 1) {
		cap_init();
	} else if (cap_flag == 0) {
		cap_deinit();
		write_event_chan();
		write_stats_chan();
	}
	return len;
}

SYSDEV_ATTR(stats, 0200, NULL, stats_store);

static struct attribute *dvfm_stats_attr[] = {
	&attr_duty_cycle.attr,
	&attr_ticks.attr,
	&attr_switch_time.attr,
	&attr_stats.attr,
};

static void update_op_cycle(int op_idx, unsigned int runtime,
			    unsigned int idletime, int inc)
{
	op_ticks_array[op_idx].runtime += runtime;
	op_ticks_array[op_idx].idletime += idletime;
	if (inc)
		op_ticks_array[op_idx].count++;
}

/* Events are divided into two parts. One is the set of run mode of operating
 * points, the other is the set of idle mode of operating points.
 * The event number of these two sets is equal.
 */
static int dvfm_event_init(struct info_head *op_table)
{
	struct op_info *entry = NULL;
	int num = 0, i;

	if (!op_table)
		goto out;
	memset(&event_array, 0, sizeof(struct event_info) * 2 * OP_NUM);
	read_lock(&op_table->lock);
	if (list_empty(&op_table->list)) {
		read_unlock(&op_table->lock);
		goto out;
	}
	list_for_each_entry(entry, &op_table->list, list) {
#if 1
		num++;
#else
		if (entry->op.flag == OP_FLAG_FACTORY) {
			num++;
			event_array[i].op_idx = num;
			event_array[i].event_idx = num;
		}
#endif
	}
	for (i = 0; i < num; i++) {
		event_array[i + num].op_idx = event_array[i].op_idx;
		event_array[i + num].event_idx = event_array[i].event_idx +
		    num;
	}
	read_unlock(&op_table->lock);
out:
	event_num = 2 * num;
	return event_num;
}

static struct dentry *dvfm_create_buf(const char *filename,
				      struct dentry *parent, int mode,
				      struct rchan_buf *buf,
				      int *is_global)
{
	return debugfs_create_file(filename, S_IRUGO | S_IWUSR, parent,
				   buf, &relay_file_operations);
}

static int dvfm_remove_buf(struct dentry *dentry)
{
	debugfs_remove(dentry);
	return 0;
}

static struct rchan_callbacks event_cb = {
	.subbuf_start = NULL,
	.buf_mapped = NULL,
	.buf_unmapped = NULL,
	.create_buf_file = dvfm_create_buf,
	.remove_buf_file = dvfm_remove_buf,
};

static int dvfm_event_table_init(void)
{
	event_table_head = kzalloc(EVENT_ARRAY_SIZE, GFP_KERNEL);
	if (!event_table_head)
		goto out_table;
	event_p = event_table_head;
	return 0;

out_table:
	printk(KERN_WARNING "Not enough memory of event table.\n");
	event_p = NULL;
	event_table_head = NULL;
	return -ENOMEM;
}

static void dvfm_event_table_deinit(void)
{
	/* kfree(NULL) is safe, so NULL check is not required */
	kfree(event_table_head);
}

static int dvfm_timeslot_table_init(void)
{
	op_stats_table_head = kzalloc(OP_STATS_ARRAY_SIZE, GFP_KERNEL);
	if (!op_stats_table_head)
		goto out_table;
	op_stats_p = op_stats_table_head;
	return 0;

out_table:
	printk(KERN_WARNING "Not enough memory of op stats table.\n");
	op_stats_table_head = NULL;
	op_stats_p = NULL;
	return -ENOMEM;
}

static void dvfm_timeslot_table_deinit(void)
{
	/* kfree(NULL) is safe, so NULL check is not required */
	kfree(op_stats_table_head);
}

/* write event to relay channel */
static void write_event_chan(void)
{
	struct event_type *p = event_table_head;
	int len, size;
	char buf[EVENT_BUF_LEN];

	if (p == NULL) {
		printk(KERN_WARNING "%s: no event\n", __func__);
		return;
	}

	/*
	 * Close the relay channel in order to save new data.
	 */
	if (event_chan)
		relay_close(event_chan);
	event_chan = relay_open("event_table", NULL,
				PAGE_SIZE, EVENT_PAGE_NUM, &event_cb,
				NULL);
	if (event_chan == NULL) {
		printk(KERN_WARNING "Can't open event_chan.\n");
		return;
	}

	relay_reset(event_chan);
	size = sizeof(struct event_type);
	spin_lock(&stats_lock);
	for (;;) {
		memset(buf, 0, EVENT_BUF_LEN);
		len = sprintf(buf, "E, %x, T, %u\n", p->event, p->timestamp);
		relay_write(event_chan, buf, len);
		p++;
		if (p > (event_table_head + (EVENT_ARRAY_SIZE - size)
			 / size))
			break;
	}
	spin_unlock(&stats_lock);
	/* Just flush, don't use relay_close() at here. */
	relay_flush(event_chan);
}

/* write stats to relay channel */
static void write_stats_chan(void)
{
	struct op_stats_type *p = op_stats_table_head;
	int len, size;
	char buf[STATS_BUF_LEN];

	if (p == NULL) {
		printk(KERN_WARNING "%s: no stats\n", __func__);
		return;
	}

	/*
	 * Close the relay channel in order to save new data.
	 */
	if (op_stats_chan)
		relay_close(op_stats_chan);
	op_stats_chan = relay_open("op_stats_table", NULL,
				   PAGE_SIZE, OP_STATS_PAGE_NUM, &event_cb,
				   NULL);
	if (op_stats_chan == NULL) {
		printk(KERN_WARNING "Can't open stats_chan.\n");
		return;
	}
	relay_reset(op_stats_chan);
	size = sizeof(struct op_stats_type);
	spin_lock(&stats_lock);
	for (;;) {
		memset(buf, 0, STATS_BUF_LEN);
		len = sprintf(buf, "T, %u, OP, %d, RT, %u, IT, %u, OV, %x\n",
			    p->timestamp, p->op_idx, p->runtime,
			    p->idletime, p->overflow);
		relay_write(op_stats_chan, buf, len);
		p++;
		if (p > (op_stats_table_head + (OP_STATS_ARRAY_SIZE - size)
			 / size))
			break;
	}
	spin_unlock(&stats_lock);
	/* Just flush, don't use relay_close() at here. */
	relay_flush(op_stats_chan);
}

/*
 * Add this event into event table when new event changes.
 * New event can be transition between run mode and idle mode in same operating
 * point, transition between different operating point.
 * Event table will record two things. One is event, the other is time stamp.
 *
 * The format of event is in below.
 * source event,	dest event
 * <high 16-bit>	<low 16-bit>
 */
int dvfm_add_event(int src_op_idx, int src_cpu_state, int dst_op_idx,
		   int dst_cpu_state)
{
	unsigned int event, size;
	unsigned long flags;
	if (event_table_head == NULL || event_p == NULL)
		return -EINVAL;
	if (src_cpu_state == CPU_STATE_IDLE)
		src_op_idx += (event_num >> 1);
	if (dst_cpu_state == CPU_STATE_IDLE)
		dst_op_idx += (event_num >> 1);
	event = (src_op_idx << 16) + dst_op_idx;
	spin_lock_irqsave(&stats_lock, flags);
	event_p->event = event;
	event_p->timestamp = dvfm_driver->read_time();
	/* increase the pointer to next one */
	event_p++;
	size = sizeof(struct event_type);
	if (event_p >= (event_table_head +
			(EVENT_ARRAY_SIZE - size) / size)) {
		event_p = event_table_head;
	}
	spin_unlock_irqrestore(&stats_lock, flags);
	return 0;
}

/*
 * Add this information into timeslot table.
 * This table records the time cost on different cpu state and different
 * operating points.
 */
int dvfm_add_timeslot(int op_idx, int cpu_state)
{
	static unsigned int prev_timestamp;
	unsigned int timestamp, time, size;
	unsigned int idle_overflow = 0, run_overflow = 0;
	unsigned long flags;

	spin_lock_irqsave(&stats_lock, flags);
	if (op_stats_table_head == NULL || op_stats_p == NULL
	    || op_idx < 0) {
		spin_unlock_irqrestore(&stats_lock, flags);
		return -EINVAL;
	}
	timestamp = dvfm_driver->read_time();
	time = (timestamp >= prev_timestamp) ? timestamp - prev_timestamp
	    : 0xFFFFFFFF - prev_timestamp + timestamp;
	prev_timestamp = timestamp;
	if (!dvfm_timeslot_init) {
		/* It's the first snapshot */
		dvfm_timeslot_init = 1;
		spin_unlock_irqrestore(&stats_lock, flags);
		return -EAGAIN;
	}

	if (op_idx != op_stats_p->op_idx) {
		/* new operating point */
		update_op_cycle(op_stats_p->op_idx, 0, 0, 1);
		op_stats_p++;	/* Point to next one */
		size = sizeof(struct op_stats_type);
		if (op_stats_p >= (op_stats_table_head +
				   (OP_STATS_ARRAY_SIZE - size) / size))
			op_stats_p = op_stats_table_head;
		op_stats_p->op_idx = op_idx;
		op_stats_p->timestamp = dvfm_driver->read_time();
		op_stats_p->runtime = 0;
		op_stats_p->idletime = 0;
		op_stats_p->overflow = 0;
	}
	if (cpu_state == CPU_STATE_IDLE) {
		if (op_stats_p->idletime > (0xFFFFFFFF - time))
			idle_overflow = 1;
		op_stats_p->idletime += time;
		update_op_cycle(op_idx, 0, time, 0);
	} else {
		if (op_stats_p->runtime > (0xFFFFFFFF - time))
			run_overflow = 1;
		op_stats_p->runtime += time;
		update_op_cycle(op_idx, time, 0, 0);
	}
	op_stats_p->overflow = (run_overflow << 16) + idle_overflow;
	spin_unlock_irqrestore(&stats_lock, flags);
	return 0;
}

/* Calculate time when begin to switch new operating point. */
int calc_switchtime_start(int old, int new, unsigned int ticks)
{
	if (old == new)
		return -EINVAL;
	op_switch[old][new].timestamp = ticks;
	return 0;
}

/* Calculate time when end to switch new operating point. */
int calc_switchtime_end(int old, int new, unsigned int ticks)
{
	unsigned int time;
	struct op_switch *p = &op_switch[old][new];

	if (old == new)
		return -EINVAL;
	time = (ticks > p->timestamp) ? ticks - p->timestamp
	    : 0xFFFFFFFF - p->timestamp + ticks;
	p->last_tick = time;
	if (time > p->max_tick)
		p->max_tick = time;
	if (time < p->min_tick || p->min_tick == 0)
		p->min_tick = time;
	return 0;
}

static int cap_init(void)
{
	struct op_info *info = NULL;
	unsigned long flags;

	spin_lock_irqsave(&stats_lock, flags);
	/* Initialize the pointer to tables */
	event_p = event_table_head;
	op_stats_p = op_stats_table_head;
	memset((char *) event_p, 0, EVENT_ARRAY_SIZE);
	memset((char *) op_stats_p, 0, OP_STATS_ARRAY_SIZE);

	/* Update the op_idx in first entry of op_stats_table */
	op_stats_p->op_idx = dvfm_get_op(&info);
	op_stats_p->timestamp = dvfm_driver->read_time();
	spin_unlock_irqrestore(&stats_lock, flags);

	/* Clear op_cycle array */
	memset(&op_ticks_array, 0, sizeof(struct op_cycle_type) * OP_NUM);

	dvfm_timeslot_init = 0;
	/* make a timestamp for the first snapshot */
	dvfm_add_timeslot(cur_op, CPU_STATE_RUN);
	dvfm_add_event(cur_op, CPU_STATE_RUN, cur_op, CPU_STATE_RUN);

	return 0;
}

static int cap_deinit(void)
{
	/* make a timestamp for the last snapshot */
	dvfm_add_timeslot(cur_op, CPU_STATE_RUN);
	dvfm_add_event(cur_op, CPU_STATE_RUN, cur_op, CPU_STATE_RUN);
	dvfm_timeslot_init = 0;
	return 0;
}

static int stats_add(struct sys_device *sys_dev)
{
	int i, n, ret;
	n = ARRAY_SIZE(dvfm_stats_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj),
					dvfm_stats_attr[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int stats_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(dvfm_stats_attr);
	for (i = 0; i < n; i++) {
		sysfs_remove_file(&(sys_dev->kobj), dvfm_stats_attr[i]);
	}
	return 0;
}

static int stats_suspend(struct sys_device *sysdev, pm_message_t pmsg)
{
	return 0;
}

static int stats_resume(struct sys_device *sysdev)
{
	return 0;
}

static struct sysdev_driver dvfm_stats_driver = {
	.add = stats_add,
	.remove = stats_rm,
	.suspend = stats_suspend,
	.resume = stats_resume,
};

int __init dvfm_stats_init(void)
{
	int ret;

	dvfm_event_init(dvfm_op_list);
	dvfm_event_table_init();
	dvfm_timeslot_table_init();
	cap_init();
	ret = sysdev_driver_register(&cpu_sysdev_class, &dvfm_stats_driver);
	if (ret)
		printk(KERN_ERR "Can't register DVFM STATS in sysfs\n");
	return ret;
}

void __exit dvfm_stats_exit(void)
{
	sysdev_driver_unregister(&cpu_sysdev_class, &dvfm_stats_driver);
	dvfm_timeslot_table_deinit();
	dvfm_event_table_deinit();
}

module_init(dvfm_stats_init);
module_exit(dvfm_stats_exit);
