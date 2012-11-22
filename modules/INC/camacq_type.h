/*.......................................................................................................
. COPYRIGHT (C)  SAMSUNG Electronics CO., LTD (Suwon, Korea). 2009           
. All rights are reserved. Reproduction and redistiribution in whole or 
. in part is prohibited without the written consent of the copyright owner.
. 
.   Developer:
.   Date:
.   Description:  
..........................................................................................................
*/

#if !defined(_CAMACQ_TYPES_H_)
#define _CAMACQ_TYPES_H_

#undef GLOBAL

#if !defined(_CAMACQ_CORE_C_)
#define GLOBAL extern
#else
#define GLOBAL
#endif

/* Include */
#include "camacq_model.h"

#if defined(WIN32)
#include "cmmfile.h"
#elif defined(_LINUX_)
#include <linux/types.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/videodev.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/mfp.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#endif /* WIN32 */

/* Global */

/* Definition */
#if defined(CAMACQ_MODEL_JETTA)
#define CAMACQ_SENSOR_MAX      2
#elif defined(CAMACQ_MODEL_ALKON) || defined(CAMACQ_MODEL_GFORCE)
#define CAMACQ_SENSOR_MAX       1
#elif defined(CAMACQ_MODEL_VOLGA)
#define CAMACQ_SENSOR_MAX      2
#else
#define CAMACQ_SENSOR_MAX      1
#endif

#define CAMACQ_SENSOR_MAIN 		0       // main sensor
#define CAMACQ_SENSOR_SUB  		1       // sub sensor 

#define CAMACQ_SENSOR_OPEN 		1
#define CAMACQ_SENSOR_CLOSE  	0
#define CAMACQ_ON               1
#define CAMACQ_OFF              0
#define CAMACQ_NAME_MAX         32

#define CAMACQ_LOG_ERR 0x01
#define CAMACQ_LOG_DBG 0x02
#define CAMACQ_LOG_MUX (CAMACQ_LOG_ERR | CAMACQ_LOG_DBG)
#define CAMACQ_LOG_LVL CAMACQ_LOG_ERR

#if defined(WIN32)
#define CAMACQ_RDONLY "r"
#define __FUNCTION__ __FILE__
#define camacq_malloc(uisz) malloc(uisz)
#define camacq_zalloc(uisz) malloc(uisz)
#define camacq_vmalloc(uisz) malloc(uisz)
#define camacq_free(pvarg) free(pvarg)
#define camacq_vfree(pvarg) free(pvarg)
#define camacq_open(pcpath, pcmode) (S32)fopen(pcpath, pcmode)
#define camacq_close(ifd) fclose((FILE*)ifd)
#define camacq_read(ifd, pvbuf, uisz) fread(pvbuf, sizeof(char), uisz, (FILE*)ifd)
#define camacq_write(ifd, pvbuf, uisz) fwrite(pvbuf, sizeof(char), uisz, (FILE*)ifd)
#define camacq_seek(ifd, uioffset, uiwhence) fseek((FILE*)ifd, uioffset, uiwhence)
#define camacq_tell(ifd) ftell((FILE*)ifd)
#define camacq_eof(ifd) feof((FILE*)ifd)
#define IS_ERR fd < 0

#define camacq_trace_err(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_ERR)\
        printf( "%s"fmt"\n", __FUNCTION__, ##args)
#define camacq_trace_dbg(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG)\
        printf( "%s"fmt"\n", __FUNCTION__, ##args)
#define camacq_trace_dbg_in(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG)\
        printf( "%s IN"fmt"\n", __FUNCTION__, ##args)
#define camacq_trace_dbg_out(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG)\
        printf( "%s OUT"fmt"\n", __FUNCTION__, ##args)
        
#elif defined(_LINUX_)
#define CAMACQ_RDONLY 0
#define CamacqMalloc(uisz) kmalloc(uisz, GFP_KERNEL)
#define CamacqZalloc(uisz) kzalloc(uisz, GFP_KERNEL)
#define CamacqVmalloc(uisz) vmalloc(uisz)
#define CamacqFree(pvarg) kfree(pvarg)
#define CamacqVfree(pvarg) vfree(pvarg)
#define CamacqOpen(pcpath, pcmode) (struct file*)filp_open(pcpath, 0, (int)pcmode)
#define CamacqClose(ifd) filp_close((struct file*)ifd, current->files)
#define CamacqRead(ifd, pvbuf, uisz) vfs_read((struct file*)ifd, pvbuf,uisz, (loff_t*)&stpos)
#define CamacqWrite(ifd, pvbuf, uisz) vfs_fwrite((struct file*)ifd, pvbuf, uisz, (loff_t*)&stpos)
#define CamacqSeek(ifd, uioffset, uiwhence) vfs_llseek((struct file*)ifd, uioffset, uiwhence)
#define CamacqTell(ifd) ((struct file*)ifd)->f_dentry->d_inode->i_size

#if 0
#define CamacqTraceErr(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_ERR)\
        printk( "%s"fmt"\n", __FUNCTION__, ##args)
#define CamacqTraceDbg(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG)\
        printk( "%s"fmt"\n", __FUNCTION__, ##args)
#define CamacqTraceIN(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG)\
        printk( "%s IN"fmt"\n", __FUNCTION__, ##args)
#define CamacqTraceOUT(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG)\
        printk( "%s OUT"fmt"\n", __FUNCTION__, ##args)
#define CamacqTraceDbg_v(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG)\
        printk( "%s OUT"fmt"\n", __FUNCTION__, ##args)
#else
static struct timeval g_stDbgTimeval;
#define CAMACQ_DBG_MSEC ( ((g_stDbgTimeval.tv_sec)*1000000 + g_stDbgTimeval.tv_usec) ) 

#define CamacqTraceErr(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_ERR) {\
        do_gettimeofday( &g_stDbgTimeval); printk(KERN_ERR " %lus %s "fmt"\n", CAMACQ_DBG_MSEC, __FUNCTION__, ##args); }
#define CamacqTraceDbg(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG) {\
         do_gettimeofday( &g_stDbgTimeval); printk(KERN_ERR " %luus %s "fmt"\n", CAMACQ_DBG_MSEC, __FUNCTION__, ##args); }
#define CamacqTraceIN(fmt, args...) \
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG) {\
        do_gettimeofday( &g_stDbgTimeval); printk(KERN_ERR " %luus %s IN "fmt"\n", CAMACQ_DBG_MSEC, __FUNCTION__, ##args); }
#define CamacqTraceOUT(fmt, args...)\
    if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG) {\
        do_gettimeofday( &g_stDbgTimeval); printk(KERN_ERR " %luus %s OUT "fmt"\n", CAMACQ_DBG_MSEC, __FUNCTION__, ##args); }
#define CamacqTraceDbg_v(fmt, args...)\
     if(CAMACQ_LOG_LVL & CAMACQ_LOG_DBG) {\
     do_gettimeofday( &g_stDbgTimeval); printk(KERN_ERR " %luus %s"fmt"\n", CAMACQ_DBG_MSEC, __FUNCTION__, ##args); }
#endif 
#endif /* WIN32 */


/* Type Definition */
typedef unsigned long UL32;
typedef unsigned int U32;
typedef unsigned short U16;
typedef unsigned char U8;

typedef int S32;
typedef short S16;
typedef char S8;

/* Enumeration */

/* Global Value */

/* Function */


/* Sensor type */
#if defined(CAMACQ_MODEL_JETTA)
#include "camacq_isx012.h"
#include "camacq_mt9v114_mipi.h"
#include "camacq_pxa950.h"
#elif defined(CAMACQ_MODEL_ALKON) || defined(CAMACQ_MODEL_GFORCE)
#include "camacq_s5k5ccgx_mipi.h"
#include "camacq_pxa950.h"
#elif defined(CAMACQ_MODEL_GFORCE_OLD)
#include "camacq_s5k4ecgx_mipi.h"
#include "camacq_pxa950.h"
#elif defined(CAMACQ_MODEL_BENI)
#include "camacq_isx006.h"
#include "camacq_pxa950.h"
#elif defined(CAMACQ_MODEL_VOLGA)
#include "camacq_isx006.h"
#include "camacq_s5k6aafx13.h"
#include "camacq_pxa950.h"
#else
#include "camacq_isx006.h"
#include "camacq_s5k6aafx13.h"
#include "camacq_pxa950.h"
#endif

#undef GLOBAL

#endif /* _CAMACQ_TYPES_H_ */
