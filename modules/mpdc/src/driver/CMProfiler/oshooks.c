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

#include <linux/types.h>
#include <linux/linkage.h>
#include <linux/ptrace.h> 
#include <linux/file.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <asm/mman.h>
#include <linux/mm.h>

#include "PXD_cm.h"
#include "cm_drv.h"
#include "cm_dsa.h"
#include "CMProfilerSettings.h"
#include "ring_buffer.h"
#include "common.h"

static unsigned long g_buffer[sizeof(PXD32_CMProcessCreate) + PATH_MAX];
extern struct RingBufferInfo g_buffer_info[];

struct mmap_arg_struct {
	unsigned long add;
	unsigned long len;
	unsigned long prot;
	unsigned long flags;
	unsigned long fd;
	unsigned long offset;
};

bool g_is_app_exit_set   = false;
pid_t g_launch_app_pid = 0;
bool g_launched_app_exit = false;
char name[PATH_MAX];

extern void write_thread_create_info(unsigned int pid, unsigned int tid, unsigned long long ts);

extern void** system_call_table;
extern pid_t g_data_collector_pid;
extern wait_queue_head_t pxcm_kd_wait;
extern unsigned long g_mode;

typedef asmlinkage int (*sys_fork_t) (struct pt_regs *regs);
typedef asmlinkage int (*sys_vfork_t) (struct pt_regs *regs);
typedef asmlinkage int (*sys_clone_t) (unsigned long clone_flgs, unsigned long newsp, struct pt_regs *regs);
typedef asmlinkage int (*sys_execve_t) (char *filenameei, char **argv, char **envp, struct pt_regs *regs);
typedef asmlinkage int (*sys_exit_t) (int error_code);
typedef asmlinkage int (*sys_exit_group_t) (int error_code);
typedef asmlinkage long (*sys_kill_t) (int pid, int sig);
typedef asmlinkage long (*sys_tkill_t) (int pid, int sig);
typedef asmlinkage long (*sys_tgkill_t) (int pid, int tid, int sig);


extern bool is_specific_object_mode(void);
extern unsigned long long get_timestamp(void);
extern PXD32_DWord convert_to_PXD32_DWord(unsigned long long n);

#define ALIGN_UP(x, align)  ((x + align - 1) & ~(align - 1))

/* formally we use register IP to calculate SP value, which is incorrect. 
 * code optimization will use IP register for temp usage, thus mess up.
 * here we use fp to cal SP value.
 */
#define INIT_STACK_FRAME \
	char *originstack = NULL;\
	char *stackbuf = NULL;\
	do{\
		__asm__("add %0, fp, #4\n\t":"=r"(originstack):);\
	} while(0)\

#define APPEND_STACK_FRAME \
	do{\
	__asm__(\
		"sub sp, sp, #84\n\t"\
		"str sp, %0"\
		:"=m"(stackbuf):"m"(stackbuf)\
	);\
	memcpy(stackbuf, originstack, 84);\
	} while(0)\

#define CUTTAIL_STACK_FRAME \
	do{\
		memcpy(originstack, stackbuf, 84);\
		__asm__("add sp, sp, #84");\
	} while(0)

static sys_vfork_t          px_original_sys_vfork  = NULL;
static sys_fork_t           px_original_sys_fork   = NULL;
static sys_clone_t          px_original_sys_clone  = NULL;
static sys_execve_t         px_original_sys_execve = NULL;

static sys_exit_t           px_original_sys_exit = NULL;
static sys_exit_group_t     px_original_sys_exit_group = NULL;
static sys_kill_t           px_original_sys_kill = NULL;
static sys_tkill_t          px_original_sys_tkill = NULL;
static sys_tgkill_t         px_original_sys_tgkill = NULL;

static bool gb_enable_os_hooks = false; 

//DECLARE_MUTEX(fork_mutex);
DEFINE_MUTEX(fork_mutex);

static void launched_app_exit_notif(void)
{
	g_launched_app_exit = true;

	if (waitqueue_active(&pxcm_kd_wait))
		wake_up_interruptible(&pxcm_kd_wait);
}

static void check_launched_app_exit(pid_t pid)
{
	if (g_is_app_exit_set && (pid == g_launch_app_pid))
	{
		launched_app_exit_notif();
		g_is_app_exit_set = false;
	}
}

static char* get_cmdline_from_task(struct task_struct *ptask, bool *is_full_path)
{
	bool find = false;
	struct mm_struct *mm;
	char *pname = NULL;
	struct vm_area_struct *mmap;	

	mm = get_task_mm(ptask);
	if (mm != NULL)
	{
		down_read(&mm->mmap_sem);
		
		for (mmap = mm->mmap; mmap; mmap = mmap->vm_next)
		{
			if (is_valid_module(mmap))
			{
				memset (name, 0, PATH_MAX * sizeof(char));
				pname = px_d_path(mmap->vm_file, name, PATH_MAX);
				
				if (pname != NULL)
				{
					find = true;
					break;
				}
		            
		        }
		}
		
		up_read(&mm->mmap_sem);
		mmput(mm);
	}

	if (!find)
	{
		/* we can't get the full path name of the process */
		*is_full_path = false;

		return ptask->comm;	
	}
	else
	{
		/* we can get the full path name of the process */
		*is_full_path = true;
		return pname;
	}
}

static char* get_cmdline_from_pid(pid_t pid, bool *is_full_path)
{
	struct task_struct *ptask;
	
	ptask = px_find_task_by_pid(pid);
	
	if (ptask != NULL)
	{
		return get_cmdline_from_task(ptask, is_full_path);
	}
	else
	{
		*is_full_path = false;
		return NULL;
	}
}

static unsigned long get_filename_offset(char * full_path)
{
	unsigned long i;
	unsigned long offset;
	
	if (full_path == NULL)
		return 0;
		
	if (full_path[0] != '/')
		return 0;
	
	offset = 0;
	
	for (i=0; full_path[i] != '\0'; i++)
	{
		if (full_path[i] == '/')
			offset = i+1;
	}
	
	return offset;      
}

pid_t get_idle_task_pid(void)
{
	return 0;
}

char * get_idle_task_name(void)
{
	return "PXIdle";
}


static void write_process_info(pid_t pid, char* cmd_line, bool is_full_path, unsigned long long ts)
{
	bool need_flush;
	bool buffer_full;
	unsigned int name_len;
	unsigned int extra_len; /* length of the name which exceeds to size of PXD32_CMProcessCreate */
	unsigned int offset;

	PXD32_CMProcessCreate *p_info;

	if (is_specific_object_mode())
	{
		return;
	}

	memset(g_buffer, 0, sizeof(g_buffer));
	
	p_info = (PXD32_CMProcessCreate *)&g_buffer;

	p_info->timestamp = convert_to_PXD32_DWord(ts);
	p_info->pid       = pid;
	p_info->flag      = 0;

	memcpy(p_info->pathName, cmd_line, strlen(cmd_line) + 1);


	/* if it is the idle process, set the corresponding flag */
	if (p_info->pid == get_idle_task_pid())
	{
		p_info->flag |= PROCESS_FLAG_IDLE_PROCESS;

		if (p_info->pathName[0] == '\0')
		{
			strcpy(p_info->pathName, get_idle_task_name());
		}
	}
	
	name_len  = (strlen(p_info->pathName) + 1) * sizeof(char);
	
	/* calculate the offset of pathName[0] from the start address of structure PXD32_CMProcessCreate */
	offset = (char *)&(p_info->pathName[0]) - (char *)p_info;
	
	if (name_len <= sizeof(PXD32_CMProcessCreate) - offset)
	{
		extra_len = 0;
	}
	else
	{
		extra_len = name_len - (sizeof(PXD32_CMProcessCreate) - offset);
	}

	if (is_full_path)
	{
		p_info->nameOffset = get_filename_offset(cmd_line);    
	}
	else
	{
		p_info->nameOffset = 0;
	}
	
	p_info->recLen =   sizeof(PXD32_CMProcessCreate) 
			         + ALIGN_UP(extra_len, sizeof(unsigned long));

	write_ring_buffer(&g_buffer_info[CM_BUFFER_ID_PROCESS_CREATE].buffer, p_info, p_info->recLen, &buffer_full, &need_flush);

	if (need_flush && !g_buffer_info[CM_BUFFER_ID_PROCESS_CREATE].is_full_event_set)
	{
		g_buffer_info[CM_BUFFER_ID_PROCESS_CREATE].is_full_event_set = true;

		if (waitqueue_active(&pxcm_kd_wait))
			wake_up_interruptible(&pxcm_kd_wait);
	}

}

static void write_process_info_by_pid(pid_t pid, unsigned long long ts)
{
	char* cmd_line;
	bool is_full_path;

	cmd_line = get_cmdline_from_pid(pid, &is_full_path);

	if (cmd_line == NULL)
	{
		return;
	}

	write_process_info(pid, cmd_line, is_full_path, ts);    
}


asmlinkage int px_sys_fork(struct pt_regs *regs)
{
	int ret = 0;
	unsigned long long ts;

	INIT_STACK_FRAME; 
	mutex_lock(&fork_mutex);

	APPEND_STACK_FRAME;

	ts = get_timestamp();

	ret = px_original_sys_fork(NULL);

	mutex_unlock(&fork_mutex);
	CUTTAIL_STACK_FRAME;

	if ((ret >= 0) && (gb_enable_os_hooks))	
	{
		write_process_info_by_pid(ret, ts);	
	}
	
	//mutex_unlock(&fork_mutex);

	return ret;
}

asmlinkage int px_sys_vfork(struct pt_regs *regs)
{
	int ret = 0;
	unsigned long long ts;

	INIT_STACK_FRAME; 
	mutex_lock(&fork_mutex);
	mutex_unlock(&fork_mutex);
	
	APPEND_STACK_FRAME;

	ts = get_timestamp();

	ret = px_original_sys_vfork(regs);

	CUTTAIL_STACK_FRAME;

	if ((ret >= 0) && (gb_enable_os_hooks))
	{
		write_process_info_by_pid(ret, ts);
	}

	return ret;
}

asmlinkage int px_sys_clone(unsigned long clone_flgs, unsigned long newsp, struct pt_regs *regs)
{
	int ret = 0;
	unsigned long long ts;

	INIT_STACK_FRAME; 

	mutex_lock(&fork_mutex);

	APPEND_STACK_FRAME;

	ts = get_timestamp();

	ret = px_original_sys_clone(clone_flgs, newsp, regs);
	
	mutex_unlock(&fork_mutex);
	CUTTAIL_STACK_FRAME;

	if ((ret >= 0) && (gb_enable_os_hooks))
	{
		if (!(clone_flgs & CLONE_THREAD))
		{
		    write_process_info_by_pid(ret, ts);
		}
	}

	//mutex_unlock(&fork_mutex);

	return ret;
}

asmlinkage int px_sys_execve(char *filenameei, char **argv, char **envp, struct pt_regs *regs)
{
	int ret;
	unsigned long long ts;

	INIT_STACK_FRAME; 

	/** 
	 * here we just use the mutex to make sure the fork call is finished, 
	 *   no need to keep the mutex till the call finished 
	 **/
	mutex_lock(&fork_mutex);
	mutex_unlock(&fork_mutex);

	APPEND_STACK_FRAME;

	ts = get_timestamp();

	ret = px_original_sys_execve(filenameei, argv, envp, regs);

	CUTTAIL_STACK_FRAME;

	if ((ret >= 0) && (gb_enable_os_hooks))
	{
		write_process_info_by_pid(current->tgid, ts);
		write_thread_create_info(current->tgid, current->pid, ts);
	}

	return ret;
}

asmlinkage int px_sys_exit(int error_code)
{
	long ret = 0;
	/**
	 * We must save the original sys_exit function here. If the current pid
	 * is the application the user launched and user selected "stop sampling
	 * on application unload", then the function "enum_module_for_exit_check"
	 * will terminate the sampling session, which in turn will remove all the
	 * syscall hooks. On 2.4 kernel it is not going to happen, because the 
	 * utility has no chance to run until this function returns; but on 2.6
	 * kernel with preempt turned on, the utility maybe scheduled run before 
	 * the following call to vt_original_sys_exit, so at that time the module
	 * tracker is already unloaded and the vt_original_sys_exit becomes NULL
	 * pointer. The following vt_sys_exit_group has the same situation.
	 */
	sys_exit_t saved_sys_exit = px_original_sys_exit;

	if (gb_enable_os_hooks) 
	{
		check_launched_app_exit(current->tgid);
	}

	ret = saved_sys_exit(error_code);

	return ret;
}

asmlinkage int px_sys_exit_group(int error_code)
{
	long ret = 0;
	sys_exit_group_t saved_sys_exit_group = px_original_sys_exit_group;

	if (gb_enable_os_hooks) 
	{
		check_launched_app_exit(current->tgid);
	}
	
	ret = saved_sys_exit_group(error_code);

	return ret;
}

static bool is_kill_sig(int sig)
{
	switch (sig)
	{
	case SIGINT:
	case SIGKILL:
	case SIGTERM:
	case SIGABRT:
	case SIGFPE:
	case SIGILL:
	case SIGQUIT:
	case SIGSEGV:
	case SIGBUS:
		return true;
	default:
		return false;
	}
}

asmlinkage long px_sys_kill(int pid, int sig)
{
	long ret = 0;

	ret = px_original_sys_kill(pid, sig);
	
	if (!ret && gb_enable_os_hooks & is_kill_sig(sig))
	{
		check_launched_app_exit(pid);
	}
	
	return ret;
}

asmlinkage long px_sys_tkill(int pid, int sig)
{
	long ret = 0;

	ret = px_original_sys_tkill(pid, sig);
	
	if (!ret && gb_enable_os_hooks & is_kill_sig(sig))
	{
		check_launched_app_exit(pid);
	}
	
	return ret;
}

asmlinkage long px_sys_tgkill(int pid, int tid, int sig)
{
	long ret = 0;

	ret = px_original_sys_tgkill(pid, tid, sig);
	
	if (!ret && gb_enable_os_hooks & is_kill_sig(sig))
	{
		check_launched_app_exit(pid);
	}
	
	return ret;
}


void static_enum_processes(void)
{
	struct task_struct *p = &init_task;

	write_process_info(0, "swapper", false, 0);

	do 
	{
		write_process_info_by_pid(p->tgid, 0);

		read_lock(&tasklist_lock);

		p = next_task(p);

		read_unlock(&tasklist_lock);
	} while (p != &init_task);
}

int install_os_hooks(void)
{
	if (g_mode != CM_MODE_SYSTEM)
	{
		px_original_sys_fork = (sys_fork_t) xchg(&system_call_table[__NR_fork - __NR_SYSCALL_BASE], px_sys_fork);

		px_original_sys_vfork = (sys_vfork_t) xchg(&system_call_table[__NR_vfork - __NR_SYSCALL_BASE], px_sys_vfork);

		px_original_sys_clone = (sys_clone_t) xchg(&system_call_table[__NR_clone - __NR_SYSCALL_BASE], px_sys_clone);

		px_original_sys_execve = (sys_execve_t) xchg(&system_call_table[__NR_execve - __NR_SYSCALL_BASE], px_sys_execve);
	}

	px_original_sys_exit = (sys_exit_t) xchg(&system_call_table[__NR_exit - __NR_SYSCALL_BASE], px_sys_exit);

	px_original_sys_exit_group = (sys_exit_group_t) xchg(&system_call_table[__NR_exit_group - __NR_SYSCALL_BASE], px_sys_exit_group);

	px_original_sys_kill = (sys_kill_t) xchg(&system_call_table[__NR_kill - __NR_SYSCALL_BASE], px_sys_kill);
	
	px_original_sys_tkill = (sys_tkill_t) xchg(&system_call_table[__NR_tkill - __NR_SYSCALL_BASE], px_sys_tkill);

	px_original_sys_tgkill = (sys_tgkill_t) xchg(&system_call_table[__NR_tgkill - __NR_SYSCALL_BASE], px_sys_tgkill);

	gb_enable_os_hooks = true;
	
	return 0;
}

int remove_os_hooks(void)
{
	void *orgFn;

	/* only remove the os hooks when install_os_hooks() has been called */
	if (!gb_enable_os_hooks)
		return 0;

	if (g_mode != CM_MODE_SYSTEM)
	{
		if ((orgFn = xchg(&px_original_sys_fork, 0)))
			orgFn = xchg(&system_call_table[__NR_fork - __NR_SYSCALL_BASE], orgFn);

		if ((orgFn = xchg(&px_original_sys_vfork, 0)))
			orgFn = xchg(&system_call_table[__NR_vfork - __NR_SYSCALL_BASE], orgFn);

		if ((orgFn = xchg(&px_original_sys_clone, 0)))
			orgFn = xchg(&system_call_table[__NR_clone - __NR_SYSCALL_BASE], orgFn);

		if ((orgFn = xchg(&px_original_sys_execve, 0)))
			orgFn = xchg(&system_call_table[__NR_execve - __NR_SYSCALL_BASE], orgFn);
	}

	if ((orgFn = xchg(&px_original_sys_exit, 0)))
		orgFn = xchg(&system_call_table[__NR_exit - __NR_SYSCALL_BASE], orgFn);

	if ((orgFn = xchg(&px_original_sys_exit_group, 0)))
		orgFn = xchg(&system_call_table[__NR_exit_group - __NR_SYSCALL_BASE], orgFn);

	if ((orgFn = xchg(&px_original_sys_kill, 0)))
		orgFn = xchg(&system_call_table[__NR_kill - __NR_SYSCALL_BASE], orgFn);
		
	if ((orgFn = xchg(&px_original_sys_tkill, 0)))
		orgFn = xchg(&system_call_table[__NR_tkill - __NR_SYSCALL_BASE], orgFn);

	if ((orgFn = xchg(&px_original_sys_tgkill, 0)))
		orgFn = xchg(&system_call_table[__NR_tgkill - __NR_SYSCALL_BASE], orgFn);

	gb_enable_os_hooks = false;
	
	return 0;
}

int start_os_hooks(void)
{
	int ret;

	if (g_mode != CM_MODE_SYSTEM)
	{
		static_enum_processes();
	}
	
	ret = install_os_hooks();
	
	return ret;
}

int stop_os_hooks(void)
{
	int ret;
	
	ret = remove_os_hooks();
	
	return ret;
}
