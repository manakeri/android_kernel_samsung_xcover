/*
 *  linux/arch/arm/mach-pxa/samsung_debug.c
 *
 *  Samsung specific error handling
 *
 *  Author:     Yung Leem (yung.leem@samsung.com)
 *  Created:    Sep. 21, 2010
 *  Copyright:  (C) Copyright 2010 Samsung Electronics Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/workqueue.h>

#include <linux/fs.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/file.h>

struct kobject *samsung_debug_kobj;
unsigned int samsung_debug_enable_flag = 0; // 0-disabled ,1-enabled
unsigned int samsung_debug_mode_flag = 0; // 0 by default
unsigned int samsung_debug_mode_flag2 = 0; // 0 by default
EXPORT_SYMBOL(samsung_debug_enable_flag); // used by ramdump.c
EXPORT_SYMBOL(samsung_debug_mode_flag); // used by ramdump.c
EXPORT_SYMBOL(samsung_debug_mode_flag2); // used by ramdump.c

#define SSDMODE_CP_RESET_ENABLE (1<<0) // CP assert to cause system ramdump
#define SSDMODE_USERLAND_DEBUG (1<<1) // Before ramdump, run userland debug first (e.g. dumpstate)
#define SSDMODE_GET_RAMDUMP_SDCARD (1<<2) // 0 - Get the ramdump by USB(RDX), 1 - Get the ramdump to SDCard.
//#define SSDMODE_RESERVED1 (1<<2) // for future use
#define SSDMODE_RESERVED2 (1<<3) // for future use
#define SSDMODE_LAST_MARKER (1<<4) // for future use
#define SSDMODE2_LOGCAT_TO_DIAG (1<<0)

// 8 bytes : [0-3]=deSAMSUNG_DEBUG_SETTING_FILEbug_enable_flag, [4-7]=ssd_mode_flag
#define SAMSUNG_DEBUG_SETTING_FILE "/mnt/.lfs/samsung_debug_setting.inf"
//TODO: when lfs write problem is fixed, restore to use lfs
//#define SAMSUNG_DEBUG_SETTING_FILE "/data/samsung_debug_setting.inf"
#define DUMPSTATELOG "/data/dumpstate/dumplog"
#define USER_TOMBSTONE "/mnt/.lfs/tombstone"
#define POWEROFF_CAUSE "/mnt/.lfs/poweroff"

#define samsung_debug_attr(_name)	\
static struct kobj_attribute _name##_attr = {	\
	.attr = {									\
		.name = __stringify(_name),				\
		.mode = 0664,							\
	},											\
	.show	= _name##_show,						\
	.store	= _name##_store,					\
}

#define DEBUG_READ 0
#define DEBUG_WRITE 1
#define VAR_DEBUG_ENABLE 0
#define VAR_SSD_FLAG 1
#define VAR_SSD_FLAG2 2
// rw_op     : 0 - read, 1 - write
// which_var : 0 - debug_enable, 1 - mode_flag
// value     : if rw_op==1, set var to value
int kernel_ssd_lfs_access(int rw_op, int which_var, unsigned int value)
{
	int ret=0, f_flags=0;
	struct file *filp;
	mm_segment_t mm_seg_fs;

	if (rw_op == DEBUG_READ)
		f_flags = (int)(O_RDONLY);
	else
		f_flags = (int)(O_RDWR|O_SYNC);

	filp = filp_open(SAMSUNG_DEBUG_SETTING_FILE, f_flags, 0);

	if (IS_ERR(filp)) {
		pr_err("%s: filp_open failed. (%ld)\n", __FUNCTION__,
					PTR_ERR(filp));
		return -1;
	}

	mm_seg_fs = get_fs();
	set_fs( get_ds() );

	if (rw_op == DEBUG_READ) {
		if (which_var == VAR_DEBUG_ENABLE) {
			ret = filp->f_op->read(filp,  (char __user *)&samsung_debug_enable_flag,
					sizeof(int), &filp->f_pos);
		} else if (which_var == VAR_SSD_FLAG) {
			ret = filp->f_op->llseek(filp, 4, SEEK_SET);//first 4bits for enable flag
			ret = filp->f_op->read(filp,  (char __user *)&samsung_debug_mode_flag,
					sizeof(int), &filp->f_pos);
		} else if (which_var == VAR_SSD_FLAG2) {
			ret = filp->f_op->llseek(filp, 8, SEEK_SET);
			ret = filp->f_op->read(filp,  (char __user *)&samsung_debug_mode_flag2,
					sizeof(int), &filp->f_pos);
		}
	} else if (rw_op == DEBUG_WRITE) {
		if (which_var == VAR_DEBUG_ENABLE) {
			ret = filp->f_op->write(filp,  (char __user *)&samsung_debug_enable_flag,
					sizeof(int), &filp->f_pos);
		} else if (which_var == VAR_SSD_FLAG) {
			ret = filp->f_op->llseek(filp, 4, SEEK_SET);//first 4bits for enable flag
			ret = filp->f_op->write(filp,  (char __user *)&samsung_debug_mode_flag,
					sizeof(int), &filp->f_pos);
		} else if (which_var == VAR_SSD_FLAG2) {
			ret = filp->f_op->llseek(filp, 8, SEEK_SET);
			ret = filp->f_op->write(filp,  (char __user *)&samsung_debug_mode_flag2,
					sizeof(int), &filp->f_pos);
		}
	}

	set_fs(mm_seg_fs);
	filp_close(filp, NULL);

	return ret;
}
/* SAMSUNG_PPS: Added this function to write a file to lfs */  
static int kernel_ssd_lfs_file_write( const char *filename ,char *buf, unsigned int size)
{
	int ret=0, f_flags=0;
	struct file *filp;
	mm_segment_t mm_seg_fs;

	f_flags = (int)(O_RDWR|O_SYNC|O_CREAT);

	filp = filp_open(filename, f_flags, 0);

	if (IS_ERR(filp)) {
		pr_err("%s: filp_open failed. (%ld)\n", __FUNCTION__,
					PTR_ERR(filp));
		return -1;
	}
	ret = filp->f_op->llseek(filp, 0, SEEK_END);


	mm_seg_fs = get_fs();
	set_fs( get_ds() );

	ret = filp->f_op->write(filp,  (char __user *)buf,size, &filp->f_pos);

	set_fs(mm_seg_fs);
	filp_close(filp, NULL);

	return ret;
}


static ssize_t
samsung_debug_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
char *buf)
{
	if (kernel_ssd_lfs_access(DEBUG_READ, VAR_DEBUG_ENABLE, 0) == -1)
		pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");
	printk(KERN_WARNING "\n samsung_debug_enable=%d : 0-disable, 1-enable\n",
			 	samsung_debug_enable_flag);
	return sprintf(buf, "%d\n", samsung_debug_enable_flag);
}

static ssize_t
samsung_debug_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
char *buf, size_t n)
{
	int val = -EINVAL;

	if(sscanf(buf, "%d", &val) != 1) {
		printk(KERN_WARNING "\n samsung_debug_enable: wrong input received. 0-disable, 1-enable\n");
		return n;
	}

	switch (val) {
	case 1:
		samsung_debug_enable_flag = 1;
		if (kernel_ssd_lfs_access(DEBUG_WRITE, VAR_DEBUG_ENABLE, 0) == -1)
			pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");
		printk(KERN_WARNING "\n samsung_debug_enable: 1 - Now enabled\n");
		break;
	case 0:
		samsung_debug_enable_flag = 0;
		if (kernel_ssd_lfs_access(DEBUG_WRITE, VAR_DEBUG_ENABLE, 0) == -1)
			pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");
		printk(KERN_WARNING "\n samsung_debug_enable: 0 - Now disabled\n");
		break;
	default:
		printk(KERN_WARNING "\n samsung_debug_enable: wrong input received. 0-disable, 1-enable\n");
		break;
	}
	return n;
}

samsung_debug_attr(samsung_debug_enable);

static void dumpstate_dump()
{
	char *argv_dumpsys[] = { "/system/bin/dumpstate", "-o", DUMPSTATELOG, "-z", NULL };
	char *envp_dumpsys[] = { 
		"HOME=/",
		"TERM=linux",
		"PATH=/sbin:/system/sbin:/system/bin:/system/xbin:/marvell/tel",
		"ANDROID_ROOT=/system",
		NULL };
	call_usermodehelper( argv_dumpsys[0], argv_dumpsys,
						envp_dumpsys, UMH_WAIT_PROC ); // blocking call until work is done
	    				//envp_dumpsys, UMH_NO_WAIT );
						//envp_dumpsys, UMH_WAIT_EXEC );
}

static ssize_t
force_operation_show(struct kobject *kobj, struct kobj_attribute *attr,
char *buf)
{
	return sprintf(buf, "force panic      : echo 1 > force_operation\n"
					    "force data abort : echo 2 > force_operation\n"
						"dumpstate panic  : echo 3 > force_operation\n");
}

static ssize_t
force_operation_store(struct kobject *kobj, struct kobj_attribute *attr,
char *buf, size_t n)
{
	int val = -EINVAL;
	char *p1 = NULL;

	if (!samsung_debug_enable_flag) {
		printk(KERN_WARNING "\nSamsung debug is disabled. Do nothing.\n");
		return n;
	}

	if(sscanf(buf, "%d", &val) != 1) {
		printk(KERN_WARNING "\n Wrong input, cat force_operation to view commands.\n");
		return n;
	}

	switch (val) {
	case 1:
		panic("Forced panic by samsung_debug\n");
		break;
	case 2:
		*p1 = "I am so dead"; // This should generate segmentation fault
		break;
	case 3:
		dumpstate_dump();
		panic("Panic induced by dumpstate_panic\n");
		break;
	default:
		printk(KERN_WARNING "\n Wrong input, cat force_operation to view commands.\n");
		break;
	}
	return n;
}

samsung_debug_attr(force_operation);


static ssize_t
ssdebug_mode_flag_show(struct kobject *kobj, struct kobj_attribute *attr,
char *buf)
{
	if (kernel_ssd_lfs_access(DEBUG_READ, VAR_SSD_FLAG, 0) == -1)
		pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");
	printk(KERN_INFO "\nCP_RESET_ENABLE=%d\nUSERLAND_DEBUG=%d"
					 "\nSSDMODE_GET_RAMDUMP_SDCARD=%d\nSSDMODE_RESERVED2=%d\n",
			samsung_debug_mode_flag & SSDMODE_CP_RESET_ENABLE,
		   	samsung_debug_mode_flag & SSDMODE_USERLAND_DEBUG,
		   	samsung_debug_mode_flag & SSDMODE_GET_RAMDUMP_SDCARD,
		   	//samsung_debug_mode_flag & SSDMODE_RESERVED1,
		   	samsung_debug_mode_flag & SSDMODE_RESERVED2
			);
	return sprintf(buf, "%.8x\n", samsung_debug_mode_flag);
}

static ssize_t
ssdebug_mode_flag_store(struct kobject *kobj, struct kobj_attribute *attr,
char *buf, size_t n)
{
	int ret = 0;
	unsigned int val = -EINVAL;

	if(ret=sscanf(buf, "%x", &val) != 1 || val > SSDMODE_LAST_MARKER-1) {
		//printk(KERN_WARNING "\n LEEMY_DEBUG: ret = %d\tval = %.8x\n", ret, val);
		printk(KERN_WARNING "\n ssdebug_mode_flag: Wrong input. Expected between %.8x to %.8x\n",
				0, SSDMODE_LAST_MARKER - 1);
		return n;
	}

	samsung_debug_mode_flag = val;

	if (kernel_ssd_lfs_access(DEBUG_WRITE, VAR_SSD_FLAG, 0) == -1)
		pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");

	printk(KERN_INFO "\nCP_RESET_ENABLE=%d\nUSERLAND_DEBUG=%d"
					 "\nSSDMODE_GET_RAMDUMP_SDCARD=%d\nSSDMODE_RESERVED2=%d\n",
			samsung_debug_mode_flag & SSDMODE_CP_RESET_ENABLE,
		   	samsung_debug_mode_flag & SSDMODE_USERLAND_DEBUG,
		   	samsung_debug_mode_flag & SSDMODE_GET_RAMDUMP_SDCARD,
		   	//samsung_debug_mode_flag & SSDMODE_RESERVED1,
		   	samsung_debug_mode_flag & SSDMODE_RESERVED2
		   	);

	return n;
}

samsung_debug_attr(ssdebug_mode_flag);

static ssize_t
ssdebug_mode_flag2_show(struct kobject *kobj, struct kobj_attribute *attr,
char *buf)
{
	if (kernel_ssd_lfs_access(DEBUG_READ, VAR_SSD_FLAG2, 0) == -1)
		pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");
	printk(KERN_WARNING "\n samsung_debug_mode_flag2=%d : 0-disable, 1-enable\n",
			 	samsung_debug_mode_flag2);
	return sprintf(buf, "%d\n", samsung_debug_mode_flag2);
}

static ssize_t
ssdebug_mode_flag2_store(struct kobject *kobj, struct kobj_attribute *attr,
char *buf, size_t n)
{
	int val = -EINVAL;

	if(sscanf(buf, "%d", &val) != 1) {
		printk(KERN_WARNING "\n samsung_mode_flag2: wrong input received. 0-disable, 1-enable\n");
		return n;
	}
	
	samsung_debug_mode_flag2 = val;

	if (kernel_ssd_lfs_access(DEBUG_WRITE, VAR_SSD_FLAG2, 0) == -1)
		pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");

	return n;
}
samsung_debug_attr(ssdebug_mode_flag2);

static ssize_t
user_tombstone_file_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{

	printk(KERN_WARNING "\n user_tombstone_file_show not implemented\n");
	return 0;
}

static ssize_t
user_tombstone_file_store(struct kobject *kobj, struct kobj_attribute *attr,char *buf, size_t n)
{
        if( (buf != NULL) || (n !=0))
	{
		if (kernel_ssd_lfs_file_write(USER_TOMBSTONE,buf,n) == -1)
			pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");
	}
	return n;
}
samsung_debug_attr(user_tombstone_file);

static ssize_t
poweroff_cause_file_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{

	printk(KERN_WARNING "\n power_cause not implemented\n");
	return 0;
}

static ssize_t
poweroff_cause_file_store(struct kobject *kobj, struct kobj_attribute *attr,char *buf, size_t n)
{
        if( (buf != NULL) || (n !=0))
	{
		if (kernel_ssd_lfs_file_write(POWEROFF_CAUSE,buf,n) == -1)
			pr_err("SSD FATAL ERROR.  Operation is likely meaningless.\n");
	}
	return n;
}
samsung_debug_attr(poweroff_cause_file);



static struct attribute * samsung_debug_g[] = {
	&samsung_debug_enable_attr.attr,
	&force_operation_attr.attr,
	&ssdebug_mode_flag_attr.attr,
	&ssdebug_mode_flag2_attr.attr,
	&user_tombstone_file_attr.attr,
	&poweroff_cause_file_attr.attr,
	NULL,
};

static struct attribute_group samsung_debug_attr_group = {
	.attrs = samsung_debug_g,
};

static int __init samsung_debug_init(void)
{
	samsung_debug_kobj = kobject_create_and_add("samsung_debug",NULL);
	if(!samsung_debug_kobj)
		return -ENOMEM;
	return sysfs_create_group(samsung_debug_kobj, &samsung_debug_attr_group);
}
core_initcall(samsung_debug_init); /*TBD: option early_initcall*/
