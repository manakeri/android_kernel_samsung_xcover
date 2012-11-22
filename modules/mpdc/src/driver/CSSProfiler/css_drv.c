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
#include <linux/errno.h>

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))
#include <mach/hardware.h>
#else
#include <asm/arch/hardware.h>
#endif

#ifdef PRM_SUPPORT
#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))
#include <mach/prm.h>
#else
#include <asm/arch/prm.h>
#endif
#else /* PRM_SUPPORT */
	extern unsigned get_clk_frequency_khz(int info);
#endif /* PRM_SUPPORT */

#include "css_drv.h"
#include "css_dsa.h"	
#include "vdkiocode.h"	
#include "CSSProfilerSettings.h"
#include "common.h"
	
MODULE_LICENSE("GPL");	
	
//extern void __iomem	*pml_base;	
extern int start_module_tracking(void);	
extern int stop_module_tracking(void);	

extern unsigned long get_timestamp_freq(void);

extern int add_module_record(struct add_module_data *data);

static ulong param_system_call_table_addr = 0;  	
module_param(param_system_call_table_addr, ulong, 0);	
	
void** system_call_table = NULL;  	

extern struct px_css_dsa *g_dsa;	
	
struct CSSTimerSettings g_tbs_settings;	
struct CSSEventSettings g_ebs_settings;	
bool g_calibration_mode;

extern bool g_launched_app_exit;	
extern bool g_is_image_load_set;	
extern bool g_is_app_exit_set;	
	
/* the pid of auto launched application */	
extern pid_t g_launch_app_pid;	

static bool g_image_loaded;	
/* the full path of the wait image */
extern char g_image_load_path[PATH_MAX];	
	
extern struct miscdevice px_css_d;	
extern struct miscdevice pxcss_sample_d;	
extern struct miscdevice pxcss_module_d;	
extern struct miscdevice pxcss_dsa_d;	
	
static unsigned int g_client_count = 0;	

/* if the profiler is stopped or not */
static bool g_profiler_stopped = true;

DECLARE_WAIT_QUEUE_HEAD(pxcss_kd_wait);
static DEFINE_MUTEX(op_mutex);

struct css_sampling_op_mach *css_samp_op;

static void init_func_for_cpu(void)
{
#ifdef PX_CPU_PXA2
	css_samp_op = &css_sampling_op_pxa2;
#endif

#ifdef PX_CPU_PJ1
	css_samp_op = &css_sampling_op_pj1;
#endif

#ifdef PX_CPU_PJ4
	css_samp_op = &css_sampling_op_pj4;
#endif
}

int handle_image_loaded(void)
{
	/* if the profiler is paused, we can't resume sampling when image is loaded */
	g_image_loaded = true;

	if (waitqueue_active(&pxcss_kd_wait))
		wake_up_interruptible(&pxcss_kd_wait);
	
	return 0;
}

static void free_dsa(void)	
{	
	if (g_dsa != NULL)	
	{	
		vfree(g_dsa);	
		g_dsa = NULL;	
	}	
}	
	
static int allocate_dsa(void)	
{	
	free_dsa();	
	
	g_dsa = __vmalloc(sizeof(struct px_css_dsa),	
                          GFP_KERNEL,	
                          pgprot_noncached(PAGE_KERNEL));	
	
	if (g_dsa == NULL)	
	{	
		return -ENOMEM; 	
	}	
	
	memset(g_dsa, 0, sizeof(struct px_css_dsa));	
		
	return 0;	
}	

struct RingBufferInfo g_sample_buffer;
struct RingBufferInfo g_module_buffer;

static void init_dsa(void)	
{	
	g_dsa->sample_count = 0;
	
	g_sample_buffer.buffer.address = 0;	
	g_sample_buffer.buffer.size           = 0;	
	g_sample_buffer.buffer.read_offset    = 0;	
	g_sample_buffer.buffer.write_offset   = 0;	
	g_sample_buffer.is_full_event_set     = false;	
//	g_sample_buffer.buffer.is_data_lost   = false;	
	
	g_module_buffer.buffer.address = 0;	
	g_module_buffer.buffer.size           = 0;	
	g_module_buffer.buffer.read_offset    = 0;	
	g_module_buffer.buffer.write_offset   = 0;	
	g_module_buffer.is_full_event_set     = false;
//	g_module_buffer.buffer.is_data_lost   = false;	
	
}	

/*
static void reset_dsa(void)	
{	
	g_sample_buffer.buffer.read_offset    = 0;	
	g_sample_buffer.buffer.write_offset   = 0;	
	g_sample_buffer.buffer.is_data_lost   = false;	
	g_sample_buffer.is_full_event_set     = false;	
	
	g_module_buffer.buffer.read_offset    = 0;	
	g_module_buffer.buffer.write_offset   = 0;	
	g_module_buffer.buffer.is_data_lost   = false;	
	g_module_buffer.is_full_event_set     = false;	
}
*/
	
static int __init px_css_init(void)	
{	
	int ret;	
	
/*	pml_base = ioremap(0x4600ff00, 32);	
	if (pml_base == NULL) 	
	{	
		printk(KERN_ERR"Unable to map PML registers!\n");	
		ret = -ENOMEM;	
		goto init_err_0;	
	}	
*/		
	/* register pmu device */	
	ret = misc_register(&px_css_d);	
	if (ret) {	
		   printk(KERN_ERR"Unable to register \"%s\" misc device: ret = %d\n", PX_CSS_DRV_NAME, ret);	
		   goto init_err_0;	
	}	
	
	/* register counter monitor thread collector DSA device */	
	ret = misc_register(&pxcss_dsa_d);	
		
	if (ret != 0){	
		printk(KERN_ERR "Unable to register \"%s\" misc device: ret = %d\n", PX_CSS_DSA_DRV_NAME, ret);	
		goto init_err_1;	
	}	
	
	/* register call stack sampling sample buffer device */	
	ret = misc_register(&pxcss_sample_d);	
		
	if (ret != 0){	
		printk(KERN_ERR "Unable to register \"%s\" misc device: ret = %d\n", PX_CSS_SAMPLE_DRV_NAME, ret);	
		goto init_err_2;	
	}	
		
	/* register call stack sampling module buffer device */	
	ret = misc_register(&pxcss_module_d);	
		
	if (ret != 0){	
		printk(KERN_ERR "Unable to register \"%s\" misc device: ret = %d\n", PX_CSS_MODULE_DRV_NAME, ret);
		goto init_err_3;	
	}	
	
#ifdef SYS_CALL_TABLE_ADDRESS
	//system_call_table =(void**) SYS_CALL_TABLE_ADDRESS;
#endif

	if (param_system_call_table_addr != 0) {	
		system_call_table = (void**) param_system_call_table_addr;	
	}	
	
	if (system_call_table == NULL) {	
		printk(KERN_ALERT"*ERROR* Can't find sys_call_table address\n");	
		goto init_err_4;	
	}	
		
	if (allocate_dsa() != 0){	
		printk(KERN_ERR "Unable to create dsa in kernel\n");	
		goto init_err_4;	
	}	
		
	init_dsa();

	init_func_for_cpu();	
	
	return 0;	
init_err_4:	
	misc_deregister(&pxcss_module_d);	
init_err_3:	
	misc_deregister(&pxcss_sample_d);		
init_err_2:	
	misc_deregister(&pxcss_dsa_d);	
init_err_1:	
	misc_deregister(&px_css_d);		
init_err_0:	
	return ret;	
}	
	
static void __exit px_css_exit(void)	
{	
	free_dsa();

	misc_deregister(&pxcss_sample_d);	
	misc_deregister(&pxcss_module_d);
	misc_deregister(&pxcss_dsa_d);	
	misc_deregister(&px_css_d);	
	
	return;	
}	
	
module_init(px_css_init);	
module_exit(px_css_exit);	
	
/*	
 * free sample buffer	
 */	
static int free_sample_buffer(void)	
{	
	if (g_sample_buffer.buffer.address != 0)	
	{	
		vfree(g_sample_buffer.buffer.address);	
	}	
	
	g_sample_buffer.buffer.address = 0;	
	g_sample_buffer.buffer.size    = 0;	
	
	return 0;	
}	
	
/* 	
 * free module buffer	
 */	
static int free_module_buffer(void)	
{	
	if (g_module_buffer.buffer.address != 0)	
	{	
		vfree(g_module_buffer.buffer.address);	
	}	
	
	g_module_buffer.buffer.address = 0;	
	g_module_buffer.buffer.size    = 0;	
	
	return 0;	
}	
	
/* 	
 * allocate sample buffer	
 */	
static int allocate_sample_buffer(unsigned int *size)	
{	
	unsigned int buffer_size;	
	void * address;	
		
	if (copy_from_user(&buffer_size, size, sizeof(unsigned int)) != 0)	
	{	
		return -EFAULT;	
	}	
		
	buffer_size = PAGE_ALIGN(buffer_size);

	/* free kernel buffers if it is already allocated */	
	if (g_sample_buffer.buffer.address != 0)	
	{	
		free_sample_buffer();	
	}	
	
	//address = __vmalloc(buffer_size, GFP_KERNEL, pgprot_noncached(PAGE_KERNEL));	
	address = vmalloc(buffer_size);
	
	if (address == NULL)	
	{	
		return -ENOMEM;	
	}	
	
	g_sample_buffer.buffer.address      = address;	
	g_sample_buffer.buffer.size         = buffer_size;	
	g_sample_buffer.buffer.read_offset  = 0;	
	g_sample_buffer.buffer.write_offset = 0;	
//	g_sample_buffer.buffer.is_data_lost = false;
	g_sample_buffer.is_full_event_set   = false;	
		
	if (copy_to_user(size, &buffer_size, sizeof(unsigned int)) != 0)
	{
		return -EFAULT;
	}

	return 0;	
}	

/* 	
 * allocate module buffer	
 */	
static int allocate_module_buffer(unsigned int *size)	
{	
	unsigned int buffer_size;	
	void * address;	
		
	if (copy_from_user(&buffer_size, size, sizeof(unsigned int)) != 0)	
	{	
		return -EFAULT;	
	}	

	buffer_size = PAGE_ALIGN(buffer_size);
		
	/* free kernel buffers if it is already allocated */	
	if (g_module_buffer.buffer.address != 0)	
	{	
		free_module_buffer();	
	}	
	
	//address = __vmalloc(buffer_size, GFP_KERNEL, pgprot_noncached(PAGE_KERNEL));
	address = vmalloc(buffer_size);
	
	if (address == NULL)	
		return -ENOMEM;	
	
	g_module_buffer.buffer.address = address;	
	g_module_buffer.buffer.size           = buffer_size;	
	g_module_buffer.buffer.read_offset    = 0;	
	g_module_buffer.buffer.write_offset   = 0;	
//	g_module_buffer.buffer.is_data_lost   = false;
	g_module_buffer.is_full_event_set     = false;	
		
	if (copy_to_user(size, &buffer_size, sizeof(unsigned int)) != 0)
	{
		return -EFAULT;
	}

	return 0;	
}	

static int start_sampling(bool *paused)	
{	
	int ret;	
	bool is_start_paused;	
	
	if (copy_from_user(&is_start_paused, paused, sizeof (bool)) != 0)	
	{	
		return -EFAULT;	
	}	
	
	//reset_dsa();	

	g_dsa->sample_count = 0;

#if 0
	ret = start_module_tracking();	
	if (ret != 0)	
	{	
		return ret;	
	}	
#endif	

	ret = css_samp_op->start(is_start_paused);
	if (ret != 0)	
	{	
		return ret;	
	}	

	g_profiler_stopped = false;

	return 0;	
}	
	
int stop_profiling(void)	
{
	if (!g_profiler_stopped)
	{
		g_is_image_load_set = false;	
		g_is_app_exit_set   = false;	

		mutex_lock(&op_mutex);

		css_samp_op->stop();
		
		mutex_unlock(&op_mutex);

		g_profiler_stopped = true;
	}

	stop_module_tracking();	
	
	return 0;	
}	
	
static int pause_profiling(void)	
{
	int ret;
	
	mutex_lock(&op_mutex);

	ret = css_samp_op->pause();

	mutex_unlock(&op_mutex);

	return ret;	
}	
	
static int resume_profiling(void)	
{	
	int ret;

	mutex_lock(&op_mutex);
	
	ret = css_samp_op->resume();

	mutex_unlock(&op_mutex);

	return ret;	
}	
	
static int query_request(unsigned int *request)	
{	
	unsigned int mask;	
	
	mask = 0;	
	
	if (g_sample_buffer.is_full_event_set)	
	{	
		mask |= KDR_HS_SAMPLE_BUFFER_FULL;	
	}	
	
	if (g_module_buffer.is_full_event_set)	
	{	
		mask |= KDR_HS_MODULE_BUFFER_FULL;	
	}	
		
	if (g_launched_app_exit)	
	{	
		mask |= KDR_HS_LAUNCHED_APP_EXIT;	
		g_launched_app_exit = false;	
	}	
		
	
	if (g_image_loaded)	
	{	
		mask |= KDR_HS_WAIT_IMAGE_LOADED;	
		g_image_loaded = false;	
	}
	
	
	if (copy_to_user(request, &mask, sizeof(unsigned int)) != 0)	
	{	
		return -EFAULT;	
	}	
	
	return 0;	
}	
	
static int get_calibration_result(struct calibration_result *result)	
{	
	struct calibration_result cb_result;	
	
	if (copy_from_user(&cb_result, result, sizeof(struct calibration_result)) != 0)	
	{	
		return -EFAULT;	
	}	
	
	cb_result.event_count = css_samp_op->get_count_for_cal(cb_result.register_id);
	
	if (copy_to_user(result, &cb_result, sizeof(struct calibration_result)) != 0)	
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
	
static int set_wait_image_load_name(char *path)	
{	
	unsigned int length;

	length = strlen_user(path);
	
	if (length > PATH_MAX)
		return -EINVAL;

	if (strncpy_from_user(g_image_load_path, path, length) == -EFAULT)
	{	
		return -EFAULT;	
	}	
		
	g_is_image_load_set = true;	
		
	return 0;	
}	
	
static int set_tbs_settings(struct CSSTimerSettings *settings)	
{	
	memset(&g_tbs_settings, 0, sizeof(g_tbs_settings));

	if (settings == NULL)	
	{	
		g_tbs_settings.interval = 0;	
	}	
	else	
	{	
		if (copy_from_user(&g_tbs_settings, settings, sizeof(struct CSSTimerSettings)) != 0)	
		{	
			printk(KERN_ERR "set_tbs_interval fails\n");	
			return -EFAULT;	
		}	
	}	
		
	return 0;		
}	
	
static int set_ebs_settings(struct CSSEventSettings *settings)	
{	
	memset(&g_ebs_settings, 0, sizeof(g_ebs_settings));

	if (settings == NULL)	
	{	
		g_ebs_settings.eventNumber = 0;	
	}	
	else	
	{
		if (copy_from_user(&g_ebs_settings, settings, sizeof(struct CSSEventSettings)) != 0)	
		{	
			return -EFAULT;	
		}	
	}	
		
	return 0;	
}	
	
static int set_calibration_mode(bool *calibration_mode)
{
	if (copy_from_user(&g_calibration_mode, calibration_mode, sizeof(bool)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int get_cpu_id(unsigned int *cpu_id)	
{	
	unsigned int id;
	
	id = get_arm_cpu_id();

	if (copy_to_user(cpu_id, &id, sizeof(unsigned long)) != 0)
	{
		return -EFAULT;
	}

	return 0;	
}	
	
static int get_cpu_freq(unsigned int *cpu_freq)	
{	
	unsigned long freq = 0;
    
#ifdef CONFIG_CPU_FREQ
	freq = cpufreq_quick_get(0);
#else // CONFIG_CPU_FREQ

#if defined(PRM_SUPPORT) && !defined(PJ1)
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

	// check if in ring 0 mode, frequency is always 60MHz when in ring 0 mode
	// BSP releases older than alpha4 don't support ring 0 mode
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

static int get_timestamp_frequency(unsigned long *freq)
{
	unsigned long f;

	f = get_timestamp_freq();
	
	if (copy_to_user(freq, &f, sizeof(unsigned long)) != 0)
	{
		return -EFAULT;
	}

	return 0;
}

static int reset_sample_buffer_full(bool *when_stop)
{
	int ret = 0;
	bool stop;

	if (copy_from_user(&stop, when_stop, sizeof(bool)))
	{
		return -EFAULT;
	}

	g_sample_buffer.is_full_event_set = false;

	if (!stop)
	{
		ret = resume_profiling();
	}

	return ret;
}

static int reset_module_buffer_full(bool *when_stop)
{
	bool stop;

	if (copy_from_user(&stop, when_stop, sizeof(bool)))
	{
		return -EFAULT;
	}

	g_module_buffer.is_full_event_set = false;

	return 0;
}

static int px_css_d_ioctl( struct inode *inode, 	
                               struct file *fp, 	
                               unsigned int cmd, 	
                               unsigned long arg)	
{	

	switch (cmd)	
	{	
	case PX_CSS_CMD_START_MODULE_TRACKING:	
		return start_module_tracking();	
	
	case PX_CSS_CMD_START_SAMPLING:	
		return start_sampling((bool *)arg);	
	
	case PX_CSS_CMD_STOP_PROFILING:	
		return stop_profiling();	
		
	case PX_CSS_CMD_PAUSE_PROFILING:	
		return pause_profiling();	
	
	case PX_CSS_CMD_RESUME_PROFILING:	
		return resume_profiling();	
	
	case PX_CSS_CMD_ALLOC_SAMPLE_BUFFER:	
		return allocate_sample_buffer((unsigned int *)arg);	
			
	case PX_CSS_CMD_ALLOC_MODULE_BUFFER:	
		return allocate_module_buffer((unsigned int *)arg);	
	
	case PX_CSS_CMD_FREE_SAMPLE_BUFFER:	
		return free_sample_buffer();	
			
	case PX_CSS_CMD_FREE_MODULE_BUFFER:	
		return free_module_buffer();	
	
	case PX_CSS_CMD_SET_AUTO_LAUNCH_APP_PID:	
		return set_auto_launch_app_pid((pid_t *)arg);	
			
	case PX_CSS_CMD_SET_WAIT_IMAGE_LOAD_NAME:	
		return set_wait_image_load_name((char *)arg);	
			
	case PX_CSS_CMD_SET_TBS_SETTINGS:	
		return set_tbs_settings((struct CSSTimerSettings *)arg);	
		
	case PX_CSS_CMD_SET_EBS_SETTINGS:	
		return set_ebs_settings((struct CSSEventSettings *)arg);	
	
	case PX_CSS_CMD_SET_CALIBRATION_MODE:
		return set_calibration_mode((bool *)arg);
	
	case PX_CSS_CMD_QUERY_REQUEST:	
		return query_request((unsigned int *)arg);	
	
	case PX_CSS_CMD_GET_CALIBRATION_RESULT:	
		return get_calibration_result((struct calibration_result *)arg);	
	
	case PX_CSS_CMD_GET_CPU_ID:	
		return get_cpu_id((unsigned int *)arg);	
		
	case PX_CSS_CMD_GET_CPU_FREQ:	
		return get_cpu_freq((unsigned int *)arg);	

	case PX_CSS_CMD_GET_TIMESTAMP_FREQ:
		return get_timestamp_frequency((unsigned long *)arg);
	
	case PX_CSS_CMD_ADD_MODULE_RECORD:
		return add_module_record((struct add_module_data *)arg);

	case PX_CSS_CMD_RESET_SAMPLE_BUFFER_FULL:
		return reset_sample_buffer_full((bool *)arg);

	case PX_CSS_CMD_RESET_MODULE_BUFFER_FULL:
		return reset_module_buffer_full((bool *)arg);
		
	default:	
		return -EINVAL;	
	}	
}	
	
static unsigned int px_css_d_poll(struct file *fp, struct poll_table_struct *wait)	
{	
	unsigned int mask = 0;	
	
	poll_wait(fp, &pxcss_kd_wait, wait);	
	
	if ((g_dsa != NULL) && (g_sample_buffer.is_full_event_set))
		mask |= POLLIN | POLLRDNORM;
	
	if ((g_dsa != NULL) && (g_module_buffer.is_full_event_set))
		mask |= POLLIN | POLLRDNORM;
	
	if (g_launched_app_exit)	
		mask |= POLLIN | POLLRDNORM;
	
	if (g_image_loaded)	
		mask |= POLLIN | POLLRDNORM;
	

	return mask;	
}	
	
static ssize_t px_css_d_write(struct file *fp, 	
                                  const char *buf, 	
                                  size_t count, 	
                                  loff_t *ppos)	
{	
	return ENOSYS;	
}	
	
static ssize_t px_css_d_read(struct file *fp, 	
                                 char *buf, 	
                                 size_t count, 	
                                 loff_t *ppos)	
{	
	return 0;	
}	
	
static int px_css_d_open(struct inode *inode, struct file *fp)	
{	
	if (g_client_count > 0)
		return -EBUSY;

	g_client_count++;

	return 0;	
}	
	
static int px_css_d_release(struct inode *inode, struct file *fp)	
{	
	g_client_count--;	
		
	if (g_client_count == 0)
	{
		/* stop profiling in case it is still running */
		stop_profiling();	

		/* free buffers in case they are not freed */
		free_sample_buffer();
		
		free_module_buffer();
	}
			
	return 0;	
}	
	
static struct file_operations px_css_d_fops = {	
	.owner   = THIS_MODULE,	
	.read    = px_css_d_read,	
	.write   = px_css_d_write,	
	.poll	 = px_css_d_poll,	
	.ioctl   = px_css_d_ioctl,	
	.open    = px_css_d_open,	
	.release = px_css_d_release,	
};	
	
struct miscdevice px_css_d = {	
	MISC_DYNAMIC_MINOR,	
	"px_css_d",	
	&px_css_d_fops	
};	
	
