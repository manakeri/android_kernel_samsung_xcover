/*
** (C) Copyright 2009 Marvell International Ltd.
**  		All Rights Reserved

** This software file (the "File") is distributed by Marvell International Ltd. 
** under the terms of the GNU General Public License Version 2, June 1991 (the "License"). 
** You may use, redistribute and/or modify this File in accordance with the terms and 
** conditions of the License, a copy of which is available along with the File in the 
** license.txt file or by writing to the Free Software Foundation, Inc., 59 Temple Place, 
** Suite 330, Boston, MA 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
** THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED WARRANTIES 
** OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY DISCLAIMED.  
** The License provides additional details about this warranty disclaimer.
*/
#include <linux/sched.h>
#include <linux/pid_namespace.h>
#include <linux/preempt.h>
#include <linux/version.h>
#include <linux/file.h>
#include <linux/fs.h>

#include "common.h"

int get_thread_group_id(struct task_struct *task)
{
	int pid;
	void *proc_vm, *p_proc_vm;
	struct task_struct *orig_task;

	read_lock(&tasklist_lock);

	if (task->pid != task->tgid)
	{
		pid = task->tgid;
	}
	else
	{
		proc_vm = task->mm;
		p_proc_vm = task->real_parent->mm;
		
		while (proc_vm == p_proc_vm)
		{
			orig_task = task;
			task = task->real_parent;

			if (orig_task == task)
			break;
		
			proc_vm = task->mm;
			p_proc_vm = task->real_parent->mm;
		}
		pid = task->pid;	
	}

	read_unlock(&tasklist_lock);

	return pid;
}

// Get the CPU ID
unsigned long get_arm_cpu_id(void)
{
	unsigned long v1 = 0;
	__asm__ __volatile__ ("mrc  p15, 0, %0, c0, c0, 0\n\t" : "=r"(v1));
	return v1;
}

bool is_valid_module(struct vm_area_struct *mmap)
{
	if (mmap == NULL)
		return false;
	
	if (mmap->vm_file == NULL)
		return false;
	
	if (mmap->vm_flags & VM_EXECUTABLE)                               // .exe
		return true;
		
	//if ((mmap->vm_flags & VM_EXEC) && !(mmap->vm_flags & VM_WRITE))   // .so
	if (mmap->vm_flags & VM_EXEC)   // .so
		return true;
		
	return false;
}

bool is_exe_module(unsigned long address)
{
	if ((address >= LINUX_APP_BASE_LOW) && (address < LINUX_APP_BASE_HIGH))
		return true;
	else
		return false;	
}

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30))
static struct task_struct * px_find_task_by_pid_ns(pid_t nr, struct pid_namespace *ns)
{
	return pid_task(find_pid_ns(nr, ns), PIDTYPE_PID);
}

struct task_struct * px_find_task_by_pid(pid_t pid)
{
	struct task_struct * result;

	rcu_read_lock();
	result = px_find_task_by_pid_ns(pid, current->nsproxy->pid_ns);
	rcu_read_unlock();

	return result;
}
#elif defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))
struct task_struct * px_find_task_by_pid(pid_t pid)
{
	return find_task_by_vpid(pid);
}
#else
struct task_struct * px_find_task_by_pid(pid_t pid)
{
	return find_task_by_pid(pid);
}
#endif


#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25))
char * px_d_path(struct file *file, char *buf, int buflen)
{
	return d_path(&file->f_path, buf, buflen);
}
#else
char * px_d_path(struct file *file, char *buf, int buflen)
{
	return d_path(file->f_dentry, file->f_vfsmnt, buf, buflen);
}
#endif
