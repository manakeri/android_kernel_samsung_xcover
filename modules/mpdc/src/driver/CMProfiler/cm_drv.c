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

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/pagemap.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/preempt.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/cpufreq.h>
#include <asm/io.h>

#ifdef PRM_SUPPORT
#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))
	#include <mach/prm.h>
#else
	#include <asm/arch/prm.h>
#endif	
#else
	extern unsigned get_clk_frequency_khz(int info);
#endif 

#include "cm_drv.h"
#include "cm_dsa.h"
#include "vdkiocode.h"
#include "CMProfilerSettings.h"
#include "common.h"

MODULE_LICENSE("GPL");

//extern struct miscdevice pxcm_dsa_d;
extern struct miscdevice pxcm_proc_create_buf_d;
extern struct miscdevice pxcm_thrd_create_buf_d;
extern struct miscdevice pxcm_thrd_switch_buf_d;
extern struct miscdevice px_cm_d;

/* start/stop/pause/resume thread preemptive monitor */
extern int start_thread_monitor(void);
extern int stop_thread_monitor(void);
extern int pause_thread_monitor(void);
extern int resume_thread_monitor(void);

/* start or stop os hooks */
extern int start_os_hooks(void);
extern int stop_os_hooks(void);

extern unsigned long long get_timestamp(void);

struct CMCounterConfigs g_counter_config;
pid_t g_data_collector_pid;

unsigned long g_mode;
pid_t g_specific_pid;
pid_t g_specific_tid;

extern wait_queue_head_t pxcm_kd_wait;
extern u64 spt_cv[MAX_CM_COUNTER_NUM];

static ulong param_system_call_table_addr = 0;  
module_param(param_system_call_table_addr, ulong, 0);

void** system_call_table = NULL;  

//struct px_cm_dsa *g_dsa = NULL;

static int g_client_count = 0;
static bool g_profiler_running = false;

extern bool g_launched_app_exit;
extern bool g_is_app_exit_set;
extern pid_t g_launch_app_pid;

struct cm_ctr_op_mach *cm_ctr_op = NULL;

struct RingBufferInfo g_buffer_info[CM_BUFFER_NUMBER];

static void init_func_for_cpu(void)
{
#ifdef PX_CPU_PXA2
	cm_ctr_op = &cm_op_pxa2;
#endif

#ifdef PX_CPU_PJ1
	cm_ctr_op = &cm_op_pj1;
#endif

#ifdef PX_CPU_PJ4
	cm_ctr_op = &cm_op_pj4;
#endif
}

#if 0
static void free_dsa(void)
{
	if (g_dsa != NULL)
	{
		vfree(g_dsa);
		g_dsa = NULL;
	}
}

static int alloc_dsa(void)
{
	free_dsa();

	g_dsa = __vmalloc(sizeof(struct px_cm_dsa),
			GFP_KERNEL,
			pgprot_noncached(PAGE_KERNEL));

	if (g_dsa == NULL)
	{
		return -ENOMEM; 
	}

	memset(g_dsa, 0, sizeof(struct px_cm_dsa));

	return 0;
}

static void init_dsa(void)
{
    int i;
	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
		g_dsa->buffer_info[i].buffer.is_data_lost = false;
		g_dsa->buffer_info[i].buffer.address      = 0;
		g_dsa->buffer_info[i].buffer.size         = 0;
		g_dsa->buffer_info[i].buffer.read_offset  = 0;
		g_dsa->buffer_info[i].buffer.write_offset = 0;
		g_dsa->buffer_info[i].is_full_event_set   = false;
	}
}
#endif

/* 
 * reset the dsa 
 */
/* 
static void reset_dsa(void)
{
    int i;
	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
	    g_dsa->buffer_info[i].buffer.is_data_lost = false;
		g_dsa->buffer_info[i].buffer.read_offset  = 0;
		g_dsa->buffer_info[i].buffer.write_offset = 0;
		g_dsa->buffer_info[i].is_full_event_set   = false;
	}    
}
*/

static int __init px_cm_init(void)
{
	int ret;

	/* register counter monitor thread collector device */
	ret = misc_register(&px_cm_d);

	if (ret != 0){
		printk(KERN_ERR "Unable to register \"%s\" misc device\n", PX_CM_DRV_NAME);
		goto init_err_1;
	}
#if 0
	/* register counter monitor thread collector DSA device */
	ret = misc_register(&pxcm_dsa_d);

	if (ret != 0){
		printk(KERN_ERR "Unable to register \"pxcm_dsa_d\" misc device\n");
		goto init_err_2;
	}
#endif
	/* register counter monitor buffer device */
	ret = misc_register(&pxcm_proc_create_buf_d);

	if (ret != 0){
		printk(KERN_ERR "Unable to register \"%s\" misc device\n", PX_CM_PC_DRV_NAME);
		goto init_err_3;
	}

	ret = misc_register(&pxcm_thrd_create_buf_d);

	if (ret != 0){
		printk(KERN_ERR "Unable to register \"%s\" misc device\n", PX_CM_TC_DRV_NAME);
		goto init_err_4;
	}

	ret = misc_register(&pxcm_thrd_switch_buf_d);

	if (ret != 0){
		printk(KERN_ERR "Unable to register \"%s\" misc device\n", PX_CM_TS_DRV_NAME);
		goto init_err_5;
	}

#ifdef SYS_CALL_TABLE_ADDRESS
	//system_call_table =(void**) SYS_CALL_TABLE_ADDRESS;
#endif

	if (param_system_call_table_addr != 0) {
		system_call_table = (void**) param_system_call_table_addr;
	}

	if (system_call_table == NULL) {
		printk(KERN_ALERT"*ERROR* Can't find sys_call_table address\n");
		goto init_err_6;
	}

#if 0
	if (alloc_dsa() != 0){
		printk(KERN_ERR "Unable to create dsa in kernel\n");
		goto init_err_7;
	}

	init_dsa();
#endif	
	init_func_for_cpu();
	
	return 0;
init_err_6:
	misc_deregister(&pxcm_thrd_switch_buf_d);
init_err_5:
	misc_deregister(&pxcm_thrd_create_buf_d);
init_err_4:
	misc_deregister(&pxcm_proc_create_buf_d);
init_err_3:
//	misc_deregister(&pxcm_dsa_d);
//init_err_2:
	misc_deregister(&px_cm_d);    
init_err_1:
	return ret;
}

static void __exit px_cm_exit(void)
{

//	free_dsa();
	misc_deregister(&pxcm_proc_create_buf_d);
	misc_deregister(&pxcm_thrd_create_buf_d);
	misc_deregister(&pxcm_thrd_switch_buf_d);
//	misc_deregister(&pxcm_dsa_d);
	misc_deregister(&px_cm_d);

	return;
}

module_init(px_cm_init);
module_exit(px_cm_exit);

static int allocate_buffer(struct RingBufferInfo *rb, unsigned int size)
{
	void * address;
	
	address = vmalloc(size);
	
	if (address == NULL)
		return -ENOMEM;
	
	rb->buffer.address      = address;
	rb->buffer.size         = size;
	rb->buffer.read_offset  = 0;
	rb->buffer.write_offset = 0;
	rb->is_full_event_set   = false;
	
	return 0;
}

static int free_buffer(struct RingBufferInfo *rb)
{
	if (rb->buffer.address != NULL)
		vfree(rb->buffer.address);
	
	rb->buffer.address = NULL;
	rb->buffer.size    = 0;
	
	return 0;
}

static int free_all_buffers(void)
{
	int i;

#if 0
	if (g_dsa->buffer_info[0].buffer.address != 0)
	{
		vfree(g_dsa->buffer_info[0].buffer.address);
	}

	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
		g_dsa->buffer_info[i].buffer.address = 0;
		g_dsa->buffer_info[i].buffer.size    = 0;
	}
#else
	for (i=0; i<CM_BUFFER_NUMBER; i++)
		free_buffer(&g_buffer_info[i]);
#endif
	return 0;
}

/*
 *  We allocate a large continous buffer, even if there are several small buffers needed
 */
static int allocate_all_buffers(unsigned int *size)
{
	int   i;
	unsigned int buffer_size[CM_BUFFER_NUMBER];
#if 0
	void *address;
	unsigned int total_buffer_size ;
	unsigned int last_buffer_size;
	void* last_buffer_addr;
#endif
	if (copy_from_user(buffer_size, size, sizeof(unsigned int) * CM_BUFFER_NUMBER) != 0)
	{
		return -EFAULT;
	}

	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
		buffer_size[i] = PAGE_ALIGN(buffer_size[i]);
	}
#if 0
	/* free kernel buffers if it is already allocated */
	if (g_dsa->buffer_info[0].buffer.address != 0)
	{
		free_all_buffers();
	}

	/* allocate one continous large buffer containing several small buffers */
	total_buffer_size = 0;

	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
		total_buffer_size += buffer_size[i];
	}

	address = __vmalloc(total_buffer_size, GFP_KERNEL, pgprot_noncached(PAGE_KERNEL));

	if (address == NULL)
		return -ENOMEM;

	/* now we calculate the address and size of each buffer */
	last_buffer_addr = address;
	last_buffer_size = 0;

	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
		g_dsa->buffer_info[i].buffer.address      = (void *)((char *)last_buffer_addr + last_buffer_size);
		g_dsa->buffer_info[i].buffer.size         = buffer_size[i];
		g_dsa->buffer_info[i].buffer.read_offset  = 0;
		g_dsa->buffer_info[i].buffer.write_offset = 0;
		g_dsa->buffer_info[i].is_full_event_set   = false;

		last_buffer_addr = g_dsa->buffer_info[i].buffer.address;
		last_buffer_size = g_dsa->buffer_info[i].buffer.size;
	}
#else
	free_all_buffers();
	
	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
		int ret;

		if ((ret = allocate_buffer(&g_buffer_info[i], buffer_size[i])) != 0)
			return ret;
	}
#endif
	if (copy_to_user(size, buffer_size, sizeof(unsigned int) * CM_BUFFER_NUMBER) != 0)
	{
		return -EFAULT;
	}

	return 0;    
}

static int set_counter(struct CMCounterConfigs *counter_config)
{
	if (copy_from_user(&g_counter_config, counter_config, sizeof(struct CMCounterConfigs)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int start_profiling(bool * is_start_paused)
{
	int ret;

	g_profiler_running = true;

//	reset_dsa();

	ret = cm_ctr_op->start();
	if (ret != 0)
	{
		return ret;
	}

	ret = start_os_hooks();

	if (ret != 0)
		return ret;
	
	if (g_mode != CM_MODE_SYSTEM)
	{

		ret = start_thread_monitor();

		if (ret != 0)
			return ret;
	}

	return 0;
}

static int stop_profiling(void)
{
	int ret;

	if (g_profiler_running)
	{
		g_is_app_exit_set = false;
		
		ret = cm_ctr_op->stop();
		if (ret != 0)
		{
			return ret;
		}

		if (g_mode != CM_MODE_SYSTEM)
		{
			ret = stop_thread_monitor();

			if (ret != 0)
			{
				return ret;
			}
		}

		ret = stop_os_hooks();

		if (ret != 0)
		{
			return ret;
		}

		g_profiler_running = false;
	}
	
	return 0;
}

static int pause_profiling(void)
{
	int ret;

	ret = cm_ctr_op->pause();
	if (ret != 0)
	{
		return ret;
	}

	if (g_mode != CM_MODE_SYSTEM)
	{
		ret = pause_thread_monitor();
		if (ret != 0)
			return ret;
	}
	
	return 0;
}

static int resume_profiling(void)
{
	int ret;

	ret = cm_ctr_op->resume();
	if (ret != 0)
	{
		return ret;
	}

	if (g_mode != CM_MODE_SYSTEM)
	{
		ret = resume_thread_monitor();
		if (ret != 0)
			return ret;
	}

	return 0;
}

static int query_request(unsigned int *request)
{
	unsigned int mask;

	mask = 0;

	if (g_buffer_info[CM_BUFFER_ID_PROCESS_CREATE].is_full_event_set)
	{
		mask |= KDR_PROCESS_CREATE_BUFFER_FULL;        
	}

	if (g_buffer_info[CM_BUFFER_ID_THREAD_CREATE].is_full_event_set)
	{
		mask |= KDR_THREAD_CREATE_BUFFER_FULL;        
	}

	if (g_buffer_info[CM_BUFFER_ID_THREAD_SWITCH].is_full_event_set)
	{
		mask |= KDR_THREAD_SWITCH_BUFFER_FULL;        
	}
	
	if (g_launched_app_exit)	
	{	
		mask |= KDR_LAUNCHED_APP_EXIT;	
		g_launched_app_exit = false;	
	}	

	if (copy_to_user(request, &mask, sizeof(unsigned int)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int get_cpu_id(unsigned long *cpu_id)
{
	unsigned int id;

	id = get_arm_cpu_id();

	if (copy_to_user(cpu_id, &id, sizeof(unsigned long)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int get_cpu_freq(unsigned long *cpu_freq)
{
	unsigned long freq = 0;

#ifdef CONFIG_CPU_FREQ
	freq = cpufreq_quick_get(0);
#else // CONFIG_CPU_FREQ

#ifdef PRM_SUPPORT
	int client_id;
	char* client_name = "frequency queryer";
	unsigned int cop;

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 21))
	struct pxa3xx_fv_info fv_info;
#else
	struct mhn_fv_info fv_info;
#endif

	/* open a PRM session to query current core frequency */
	client_id = prm_open_session(PRI_LOWEST, client_name, NULL, NULL);
	if (client_id < 0) {
		printk (KERN_ALERT"*STATUS* open prm session failed!\n");
		return -1;
	}

	if (cop_get_cur_cop(client_id, &cop, &fv_info))
	{
		printk(KERN_ERR "Can't get core clock frequency\n");
		return -1;
	}

	//check if in ring 0 mode, frequency is always 60MHz when in ring 0 mode
	//	BSP releases older than alpha4 don't support ring 0 mode
	if (fv_info.d0cs)
		freq = 60 * 1000;
	else 
		freq = fv_info.xl *  fv_info.xn * 13000;

	prm_close_session(client_id);

#else
	freq = 0;
#endif
#endif // CONFIG_CPU_FREQ

	if (copy_to_user(cpu_freq, &freq, sizeof(unsigned long)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int get_time_stamp(unsigned long long * ts)
{
	unsigned long long timestamp;

	timestamp = get_timestamp();

	if (copy_to_user(ts, &timestamp, sizeof(unsigned long long)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int get_timestamp_freq(unsigned long long *freq)
{
	unsigned long long frequency;

	frequency = 1000 * 1000;

	if (copy_to_user(freq, &frequency, sizeof(unsigned long long)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int get_os_timer_freq(unsigned long long *freq)
{
	unsigned long long frequency;

	frequency = 1000 * 1000;

	if (copy_to_user(freq, &frequency, sizeof(unsigned long long)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int read_counter_value(struct read_counter_data* op)
{
	struct read_counter_data rcop;

	if (copy_from_user(&rcop, op, sizeof(rcop)) != 0)
	{
		return -EFAULT;
	}

	if (!cm_ctr_op->read_counter(rcop.cid, &(rcop.value)))
	{
		return -EFAULT;
	}

	if (copy_to_user(op, &rcop, sizeof(rcop)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int set_mode(struct set_mode_data *data)
{
	struct set_mode_data smd;

	if (copy_from_user(&smd, data, sizeof(struct set_mode_data)) != 0)
	{
		return -EFAULT;
	}

	g_mode         = smd.mode;    
	g_specific_pid = smd.specific_pid;
	g_specific_tid = smd.specific_tid;

	return 0;
}

static int read_counter_in_specific_mode(unsigned long long *cv_array)
{
	if (copy_to_user(cv_array, &spt_cv, MAX_CM_COUNTER_NUM * sizeof(unsigned long long)) != 0)
	{
		return -EFAULT;
	}

	return 0;    
}

static int set_auto_launch_app_pid(pid_t *pid)	
{	
	if (copy_from_user(&g_launch_app_pid, pid, sizeof(pid_t)) != 0)	
	{	
		return -EFAULT;	
	}	
		
	g_is_app_exit_set = true;	
		
	return 0;	
}

static int reset_buffer_full(unsigned int *buffer_id)
{
	unsigned int id;
	
	if (copy_from_user(&id, buffer_id, sizeof(unsigned int)) != 0)
	{
		return -EFAULT;
	}
		
	switch (id)
	{
	case CM_BUFFER_ID_PROCESS_CREATE:
	case CM_BUFFER_ID_THREAD_CREATE:
	case CM_BUFFER_ID_THREAD_SWITCH:
		g_buffer_info[id].is_full_event_set = false;
		return 0;
	
	default:
		return -EINVAL;
	}
}

static int px_cm_d_ioctl( struct inode *inode, struct file *fp, unsigned int cmd, unsigned long arg)
{
	switch (cmd)
	{
		case PX_CM_CMD_GET_CPU_ID:
			return get_cpu_id((unsigned long *)arg);

		case PX_CM_CMD_GET_CPU_FREQ:
			return get_cpu_freq((unsigned long *)arg);    

		case PX_CM_CMD_READ_COUNTER:
			return read_counter_value((struct read_counter_data *)arg);

		case PX_CM_CMD_ALLOC_BUFFER:
			return allocate_all_buffers((unsigned int*)arg);

		case PX_CM_CMD_FREE_BUFFER:
			return free_all_buffers();

		case PX_CM_CMD_START_KERNEL_DRIVER:
			return start_profiling((bool *)arg); 

		case PX_CM_CMD_STOP_KERNEL_DRIVER:
			return stop_profiling();

		case PX_CM_CMD_PAUSE_KERNEL_DRIVER:
			return pause_profiling();

		case PX_CM_CMD_RESUME_KERNEL_DRIVER:
			return resume_profiling();

		case PX_CM_CMD_SET_COUNTER:
			return set_counter((struct CMCounterConfigs *)arg); 

		case PX_CM_CMD_QUERY_REQUEST:
			return query_request((unsigned int *)arg);

		case PX_CM_CMD_GET_TIMESTAMP_FREQ:
			return get_timestamp_freq((unsigned long long *)arg);

		case PX_CM_CMD_GET_OS_TIMER_FREQ:
			return get_os_timer_freq((unsigned long long *)arg);

		case PX_CM_CMD_GET_TIMESTAMP:
			return get_time_stamp((unsigned long long *)arg);

		case PX_CM_CMD_SET_MODE:
			return set_mode((struct set_mode_data *)arg);

		case PX_CM_CMD_READ_COUNTER_IN_SPECIFIC_MODE:
			return read_counter_in_specific_mode((unsigned long long *)arg);
		
		case PX_CM_CMD_SET_AUTO_LAUNCH_APP_PID:
			return set_auto_launch_app_pid((pid_t *)arg);
		
		case PX_CM_CMD_RESET_BUFFER_FULL:
			return reset_buffer_full((unsigned int *)arg);

		default:
			return -EINVAL;
	}
}

static unsigned int px_cm_d_poll(struct file *fp, struct poll_table_struct *wait)
{
	int i;
	unsigned int mask = 0;

	poll_wait(fp, &pxcm_kd_wait, wait);

	for (i=0; i<CM_BUFFER_NUMBER; i++)
	{
		if (g_buffer_info[i].is_full_event_set)
			mask |= POLLIN | POLLRDNORM;
	}
	
	if (g_launched_app_exit)
	{	
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

static ssize_t px_cm_d_write(struct file *fp, const char *buf, size_t count, loff_t *ppos)
{
	return ENOSYS;
}

static ssize_t px_cm_d_read(struct file *fp, char *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static int px_cm_d_open(struct inode *inode, struct file *fp)
{
	if (g_client_count > 0)
		return -EBUSY;

	g_client_count++;

	return 0;
}

static int px_cm_d_release(struct inode *inode, struct file *fp)
{
	g_client_count--;

	if (g_client_count == 0)
	{
		/* stop profiling in case it is still running */
		stop_profiling();

		/* free buffers in case they are not freed */
		free_all_buffers();
	}

	return 0;
}

static struct file_operations px_cm_d_fops = {
	.owner   = THIS_MODULE,
	.read    = px_cm_d_read,
	.write   = px_cm_d_write,
	.poll    = px_cm_d_poll,
	.ioctl   = px_cm_d_ioctl,
	.open    = px_cm_d_open,
	.release = px_cm_d_release,
};

struct miscdevice px_cm_d = {
	MISC_DYNAMIC_MINOR,
	"px_cm_d",
	&px_cm_d_fops
};


