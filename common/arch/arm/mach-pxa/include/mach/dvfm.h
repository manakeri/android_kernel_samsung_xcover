/*
 * Copyright (C) 2003-2004 Intel Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef DVFM_H
#define DVFM_H

#ifdef __KERNEL__
enum {
	FV_NOTIFIER_QUERY_SET = 1,
	FV_NOTIFIER_PRE_SET = 2,
	FV_NOTIFIER_POST_SET = 3,
};


#define MAXTOKENS			80
#define CONSTRAINT_NAME_LEN		20

#define DVFM_MAX_NAME			32
#define DVFM_MAX_DEVICE			32

#define DVFM_FREQUENCY_NOTIFIER		0
#define	DVFM_LOWPOWER_NOTIFIER		1

#define DVFM_FREQ_PRECHANGE		0
#define DVFM_FREQ_POSTCHANGE		1

#define DVFM_LOWPOWER_PRECHANGE		0
#define DVFM_LOWPOWER_POSTCHANGE	1

#define APPS_COMM_D2_THRESHOLD 326

/* set the lowest operating point that is equal or higher than specified */
#define RELATION_LOW			0
/* set the highest operating point that is equal or lower than specified */
#define RELATION_HIGH			1
/* set the specified operating point */
#define RELATION_STICK			2

/* Both of these states are used in statistical calculation */
#define CPU_STATE_RUN			1
#define CPU_STATE_IDLE			2

/*
 * operating point definition
 */

struct op_info {
	void *op;
	struct list_head list;
	unsigned int index;
	unsigned int device;	/* store the device ID blocking OP */
};

struct dvfm_freqs {
	unsigned int old;	/* operating point index */
	unsigned int new;	/* operating point index */
	struct op_info old_info;
	struct op_info new_info;
	unsigned int flags;
};

struct info_head {
	struct list_head list;
	rwlock_t lock;
	unsigned int device;	/* store the registerred device ID */
};

struct head_notifier {
	spinlock_t lock;
	struct notifier_block *head;
};

struct dvfm_lock {
	spinlock_t lock;
	unsigned long flags;
	int dev_idx;
	int count;
};

/*
 * Store the dev_id and dev_name.
 * Registered device number can't be larger than 32.
 */
struct dvfm_trace_info {
	struct list_head list;
	int index;		/* index is [0,31] */
	unsigned int dev_id;	/* dev_id == 1 << index */
	char name[DVFM_MAX_NAME];
};


struct dvfm_driver {
	int (*get_opinfo) (void *driver_data, void *info);
	int (*count) (void *driver_data, struct info_head * op_table);
	int (*set) (void *driver_data, struct dvfm_freqs * freq,
		    unsigned int new, unsigned int relation);
	int (*dump) (void *driver_data, struct op_info * md, char *buf);
	char *(*name) (void *driver_data, struct op_info * md);
	int (*request_set) (void *driver_data, int index);
	int (*enable_dvfm) (void *driver_data, int dev_id);
	int (*disable_dvfm) (void *driver_data, int dev_id);
	int (*enable_op) (void *driver_data, int index, int relation);
	int (*disable_op) (void *driver_data, int index, int relation);
	int (*volt_show) (void *driver_data, char *buf);
	unsigned int (*ticks_to_usec) (unsigned int);
	unsigned int (*ticks_to_sec) (unsigned int);
	unsigned int (*read_time) (void);
	void *priv;
};

extern struct dvfm_driver *dvfm_driver;
extern struct info_head *dvfm_op_list;
extern unsigned int op_nums;
extern int PowerDisabled;
extern int DvfmDisabled;

#ifdef CONFIG_DVFM
extern int dvfm_notifier_frequency(struct dvfm_freqs *freqs,
				   unsigned int state);
extern int dvfm_notifier_lowpower(struct dvfm_freqs *freqs,
				  unsigned int state);
extern int dvfm_register_notifier(struct notifier_block *nb,
				  unsigned int list);
extern int dvfm_unregister_notifier(struct notifier_block *nb,
				    unsigned int list);
extern int dvfm_register_driver(struct dvfm_driver *driver_data,
				struct info_head *op_list);
extern int dvfm_unregister_driver(struct dvfm_driver *driver);
extern int dvfm_register(const char *name, int *);
extern int dvfm_unregister(const char *name, int *);
extern int dvfm_query_device_num(void);
extern int dvfm_query_device_list(void *, int);

extern int dvfm_find_index(char *name, int *id);

extern int dvfm_enable_op(int, int);
extern int dvfm_disable_op(int, int);
extern int dvfm_enable(int);
extern int dvfm_disable(int);
extern int dvfm_enable_op_name(char *, int);
extern int dvfm_disable_op_name(char *, int);
extern int dvfm_enable_op_name_no_change(char *, int);
extern int dvfm_disable_op_name_no_change(char *, int);

extern void dvfm_disable_lowpower(int dev_idx);
extern void dvfm_enable_lowpower(int dev_idx);
extern void dvfm_disable_global(int dev_idx);
extern void dvfm_enable_global(int dev_idx);

extern int dvfm_set_op(struct dvfm_freqs *, unsigned int, unsigned int);
extern int dvfm_get_op(struct op_info **);
extern int dvfm_get_defop(void);
extern int dvfm_get_opinfo(int, struct op_info **);
extern int dvfm_request_op(int);
extern int dvfm_op_count(void);
extern int dvfm_find_op(int, struct op_info **);
extern int dvfm_trace(char *);

extern int dvfm_add_event(int, int, int, int);
extern int dvfm_add_timeslot(int, int);
extern int calc_switchtime_start(int, int, unsigned int);
extern int calc_switchtime_end(int, int, unsigned int);

/* handling comm apps sync */
extern int dvfm_notify_next_comm_wakeup_time(unsigned int);
extern unsigned int dvfm_get_next_comm_wakeup_time(void);
extern int dvfm_is_comm_wakep_near(void);
extern void set_mipi_reference_control(void);
extern void clear_mipi_reference_control(void);

#else				/* CONFIG_DVFM */
static inline int dvfm_register_notifier(struct notifier_block *nb,
					 unsigned int list)
{
	return 0;
}

static inline int dvfm_unregister_notifier(struct notifier_block *nb,
					   unsigned int list)
{
	return 0;
}

static inline int dvfm_enable_op(int index, int dev_idx)
{
	return 0;
}

static inline int dvfm_disable_op(int index, int dev_idx)
{
	return 0;
}

static inline int dvfm_enable(int dev_idx)
{
	return 0;
}

static inline int dvfm_disable(int dev_idx)
{
	return 0;
}

static inline int dvfm_enable_op_name(char *name, int dev_idx)
{
	return 0;
}

static inline int dvfm_disable_op_name(char *name, int dev_idx)
{
	return 0;
}

static inline int dvfm_enable_op_name_no_change(char *name, int dev_idx)
{
	return 0;
}

static inline int dvfm_disable_op_name_no_change(char *name, int dev_idx)
{
	return 0;
}

static inline int dvfm_register(char *name, int *id)
{
	return 0;
}

static inline int dvfm_unregister(char *name, int *id)
{
	return 0;
}

static inline void dvfm_disable_lowpower(int dev_idx)
{
}

static inline void dvfm_enable_lowpower(int dev_idx)
{
}

static inline void dvfm_disable_global(int dev_idx)
{
}

static inline void dvfm_enable_global(int dev_idx)
{
}
#endif
#endif
#endif
