/*
 * pxa9xx_freeze
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sysdev.h>
#include <linux/pxa95x_freeze.h>


extern struct kobject *power_kobj;

int android_freezer_debug;
static LIST_HEAD(unfreezable_task_list);
static void unfreezable_task_list_print(void);

#define frz_attr(_name)				\
    static struct kobj_attribute _name##_attr = {		\
    .attr	= {					\
    .name = __stringify(_name),		\
    .mode = 0644,				\
},						\
    .show	= _name##_show,				\
    .store = _name##_store,			\
}

/* check non freezeable tasks list PF_NOFREEZE is set */
int FRZ_process_unfreezable_list_check(void)
{
	struct unfreezable_task_entry *entry;
	struct task_struct *g, *p;
	int rc = 0;

	read_lock(&tasklist_lock);
	list_for_each_entry(entry, &unfreezable_task_list, link) {
	do_each_thread(g, p) {
		if (strcmp(entry->name, p->comm) == 0) {
			if (!(p->flags & PF_NOFREEZE)) {
				printk(KERN_INFO "ERROR: process = %s pid = %d has to be unfeezeable but PF_NOFREEZE is clear\n", p->comm, p->pid);
				rc = -1;
			}
		}
		} while_each_thread(g, p);
	}
	read_unlock(&tasklist_lock);
	return rc;
}


void FRZ_process_show(int freeze_only)
{

	struct task_struct *g, *p;
	int count = 0;

	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		if ((freeze_only == 0) || (p->flags & PF_NOFREEZE)) {
			printk(KERN_INFO "[%d] pid = %d tgid = %d name = %s",
				count, p->pid, p->tgid, p->comm);
			if (p->flags & PF_NOFREEZE)
				printk(KERN_INFO "\tPF_NOFREEZE");
			printk(KERN_INFO "\n");
			count++;
			}
	} while_each_thread(g, p);
	read_unlock(&tasklist_lock);
}

static int __frz_procees_store(unsigned char *task_name, int op)
{
	struct task_struct *g, *p;
	int rc, count = 0, found = 0;

	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		if ((strncmp(p->comm, task_name, TASK_COMM_LEN)) != 0)
			continue;
		found++;
		printk(KERN_INFO "[%d] pid = %d name = %s\t",
			count, p->pid, p->comm);
		if (op) {
			p->flags |= PF_NOFREEZE;
			printk(KERN_INFO "set to be PF_NOFREEZE");
		} else {
			p->flags &= (~PF_NOFREEZE);
			printk(KERN_INFO "set to be no PF_NOFREEZE");
		}
		printk(KERN_INFO "\n");
	} while_each_thread(g, p);
	read_unlock(&tasklist_lock);
	if (found) {
		printk(KERN_INFO "OK found %d process %s name\n", found, task_name);
		rc = 0;
	} else {
		printk(KERN_INFO "proccess %s name did not find\n", task_name);
		rc = -1;
	}
	return rc;
}

static ssize_t frz_all_process_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t len)
{
	return 0;
}

static ssize_t frz_all_process_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	FRZ_process_show(0);
	return 0;
}
frz_attr(frz_all_process);


static ssize_t frz_process_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	FRZ_process_show(1);
	return 0;
}

static ssize_t frz_process_store(struct kobject *kobj, struct kobj_attribute * attr, const char *buf, size_t len)
{
	char task_name[MAX_LINE_LENGTH];
	int op;

	memset(task_name, 0, TASK_COMM_LEN);
	sscanf(buf, "%s%d", (char *)task_name, &op);

	switch (op) {
	case 0:
		printk(KERN_INFO "try set the process %s as freezeable\n", task_name);
		break;
	case 1:
		printk(KERN_INFO "try set the process %s as non freezeable\n", task_name);
		break;
	default:
		printk(KERN_INFO "Type:  process name 1   .... for set process as non freezeable\n"
			"Type:  process name 0   .... for set process as freezeable\n");
		return len;
	}
	__frz_procees_store(task_name, op);
	return len;
}
frz_attr(frz_process);

static ssize_t frz_dbg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	printk(KERN_INFO "android freezer debug = %d\n", android_freezer_debug);
	return 0;
}

static ssize_t frz_dbg_store(struct kobject *kobj, struct kobj_attribute * attr, const char *buf, size_t len)
{
	int op;

	sscanf(buf, "%d", &op);
	switch (op) {
	case 0:
	case FRZ_DEBUG_DISPLAY_TASK:
	case FRZ_DEBUG_CHECK_FREEZE_TASKS:
	case (FRZ_DEBUG_DISPLAY_TASK | FRZ_DEBUG_CHECK_FREEZE_TASKS):
			android_freezer_debug  = op;
		break;
	default:
		printk(KERN_INFO "for display procees each suspend set val 0x%x \n"
			"for check process at freeze list status set val 0x%x\n"
			"for disable debug set 0 \n", FRZ_DEBUG_DISPLAY_TASK, FRZ_DEBUG_CHECK_FREEZE_TASKS);
		break;
	}
	return len;
}
frz_attr(frz_dbg);

static ssize_t frz_list_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unfreezable_task_list_print();
	return 0;
}

static ssize_t frz_list_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t len)
{
	return 0;
}
frz_attr(frz_list);

static struct attribute *frz_ops[] = {
	&frz_process_attr.attr,
	&frz_dbg_attr.attr,
	&frz_list_attr.attr,
	&frz_all_process_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name	 = "freeze_process",
	.attrs = frz_ops,
};

int FRZ_set_nonfreezeable_procees(char *task_name)
{
	int rc = __frz_procees_store(task_name, 1);
	if (rc != 0)
		printk(KERN_INFO "%s task name = %s faild\n", __func__, task_name);

	return rc;
}


/* for debug build list of non freezable process */
static void unfreezable_task_entry_add(struct unfreezable_task_entry *entry, const char *name)
{
	if (name)
		strncpy(entry->name, name, (sizeof(entry->name) - 1));
	list_add(&entry->link, &unfreezable_task_list);
}

static void unfreezable_task_entry_add_by_name(const char *name)
{
	struct unfreezable_task_entry *entry;
	entry = kzalloc(sizeof(struct unfreezable_task_entry), GFP_KERNEL);
	unfreezable_task_entry_add(entry, name);
	return;
}

/* prints all unfreezable tasks */
static void unfreezable_task_list_print(void)
{
	struct unfreezable_task_entry *entry;
	list_for_each_entry(entry, &unfreezable_task_list, link) {
		printk(KERN_INFO "%s\n", entry->name);
	}
}

/* finds the first entry with the given name and removes it from the list */
static void unfreezable_task_entry_remove_by_name(const char *name)
{
	struct unfreezable_task_entry *entry, *found_entry = NULL;
	list_for_each_entry(entry, &unfreezable_task_list, link) {
		if (!strcmp(entry->name, name))
			found_entry = entry;
	}
	if (found_entry) {
		list_del(&found_entry->link);
		kfree(found_entry);
	}
	return;
}

static int freezer_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	data_struct reg_data;

	switch (cmd) {
	case PROCESS_NON_FREEZE: {
		unsigned char task_name[MAX_LINE_LENGTH];
		memset(task_name, 0 , TASK_COMM_LEN);
		if (copy_from_user(&reg_data, (data_struct *)arg, sizeof(data_struct))) {
			printk(KERN_INFO "%s (%d) failed copy data_struct from user\n", __func__, __LINE__);
			return -EFAULT;
		}
		if (copy_from_user(task_name, (data_struct *)reg_data.data_arg1, TASK_COMM_LEN)) {
			printk(KERN_INFO "%s (%d) failed copy task_name from user\n", __func__, __LINE__);
			return -EFAULT;
		}
		FRZ_set_nonfreezeable_procees(task_name);
		break;
	}
	default:
		printk(KERN_INFO "%s get from user unkowon command = %d\n", __func__, cmd);
		err = -1;
		break;
	}
    return err;
}

static const struct file_operations freezer_fops = {
owner:	THIS_MODULE,
ioctl :	freezer_ioctl,
};

static struct miscdevice freezer_misc_device = {
minor:	MISC_DYNAMIC_MINOR,
name :	"freezer_device",
fops :	&freezer_fops,
};

static int __init unfreezable_task_entry_init(void)
{
	int ret;

	ret = misc_register(&freezer_misc_device);
	if (ret != 0) {
		printk(KERN_ERR "register freezer device failed, ret = %d.\n ", ret);
		return -EFAULT;
	}
	if (sysfs_create_group(power_kobj, &attr_group)) {
		printk(KERN_INFO "%s failed\n", __func__);
		return -EFAULT;
	}

	/* for debug build list of non freezable process */
	unfreezable_task_entry_add_by_name("rild");
	unfreezable_task_entry_add_by_name("mtsd");
	unfreezable_task_entry_add_by_name("mtilatcmd");
	unfreezable_task_entry_add_by_name("serial_client");
	return 0;
}

static void __exit unfreezable_task_entry_exit(void)
{
	unfreezable_task_entry_remove_by_name("rild");
	unfreezable_task_entry_remove_by_name("mtsd");
	unfreezable_task_entry_remove_by_name("mtilatcmd");
	unfreezable_task_entry_remove_by_name("serial_client");
}

module_init(unfreezable_task_entry_init);
module_exit(unfreezable_task_entry_exit);
