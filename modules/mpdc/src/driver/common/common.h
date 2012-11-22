#ifndef __PX_DRIVER_CMN_H__
#define __PX_DRIVER_CMN_H__

#include <linux/mm.h>
#include <linux/module.h>
#include <linux/file.h>

#define LINUX_APP_BASE_LOW	0x00008000
//#define LINUX_APP_BASE_HIGH	0x20000000
#define LINUX_APP_BASE_HIGH	ELF_ET_DYN_BASE

extern int get_thread_group_id(struct task_struct *task);
extern unsigned long get_arm_cpu_id(void);
extern bool is_valid_module(struct vm_area_struct *mmap);
extern bool is_exe_module(unsigned long address);
extern struct task_struct * px_find_task_by_pid(pid_t pid);
extern char * px_d_path(struct file *file, char *buf, int buflen);


#endif // __PX_DRIVER_CMN_H__
