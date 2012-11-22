/*
 *  linux/drivers/media/video/pxa95x-camrea.c - PXA9XX MMCI driver
 *
 *  Based on linux/drivers/media/video/cafe_ccic.c
 *
 *  Copyright:	(C) Copyright 2008 Marvell International Ltd.
 *              Mingwei Wang <mwwang@marvell.com>
 *
 * A driver for the CMOS camera controller in the Marvell 88ALP01 "cafe"
 * multifunction chip.  Currently works with the Omnivision OV7670
 * sensor.
 *
 * The data sheet for this device can be found at:
 *    http://www.marvell.com/products/pcconn/88ALP01.jsp
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Written by Jonathan Corbet, corbet@lwn.net.
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include <linux/uaccess.h>
#include <linux/io.h>

#include <linux/clk.h>
#include <mach/pxa95x-regs.h>
#include <mach/dma.h>
#include <mach/camera.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>

#include <linux/leds.h>
#include "../../staging/android/timed_output.h"

#include <mach/dvfm.h>

#include "camacq_api.h"


#if 0
static struct timeval g_stDbgTimeval;
#define DBG_MSEC ( ((g_stDbgTimeval.tv_sec)*1000000 + g_stDbgTimeval.tv_usec) ) 
// #define DBG_MSEC 0
#define pxa950_camera_dbg_in(fmt, args...)    do_gettimeofday( &g_stDbgTimeval); printk(KERN_ALERT " %luus %s IN "fmt"\n", DBG_MSEC, __FUNCTION__, ##args)
#define pxa950_camera_dbg_out(fmt, args...)    do_gettimeofday( &g_stDbgTimeval); printk(KERN_ALERT" %luus %s OUT "fmt"\n", DBG_MSEC, __FUNCTION__, ##args)
#define pxa950_camera_dbg(fmt, args...)    do_gettimeofday( &g_stDbgTimeval); printk(KERN_ALERT "%luus %s "fmt"\n", DBG_MSEC, __FUNCTION__, ##args)
#define pxa950_camera_dbg_v(fmt, args...)  // do_gettimeofday( &g_stDbgTimeval); printk(KERN_ALERT "%luus %s "fmt"\n", DBG_MSEC, __FUNCTION__, ##args)
#define pxa950_camera_err(fmt, args...) do_gettimeofday( &g_stDbgTimeval); printk(KERN_ALERT " %luus %s "fmt"\n", DBG_MSEC, __FUNCTION__, ##args)

#else
#define pxa950_camera_dbg_in(fmt, args...)   
#define pxa950_camera_dbg_out(fmt, args...)    
#define pxa950_camera_dbg(fmt, args...)    
#define pxa950_camera_dbg_v(fmt, args...)
#define pxa950_camera_err(fmt, args...) printk(KERN_ALERT " %s "fmt"\n", __FUNCTION__, ##args)
#endif

// for debugging
#define CSI_REG_DUMP        0
#define FPS_DUMP            0
#define CCIC_OFO_DUMP       0

#if CSI_REG_DUMP
#define CSI_DEBUG_LENGTH        1000

static unsigned int g_iCsiDebugCount;
static unsigned int g_rgiREG_CSxINST[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSSCR[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSGCR[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSxCR0[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSxSR[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSxINEN[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSxTIM0[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSxTIM1[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSxGENDAT[CSI_DEBUG_LENGTH];
static unsigned int g_rgiREG_CSxPHYCAL[CSI_DEBUG_LENGTH];
#endif /* CSI_REG_DUMP */

#if FPS_DUMP
#define FPS_MAX     100
struct timeval g_stFps[FPS_MAX] = {0, };
static unsigned int g_iFpsCnt = 0;
static unsigned int g_iOverflowCnt = 0;
#endif /* FPS_DUMP */

#if CCIC_OFO_DUMP
#define MAX_OFO_COUNT 50
static unsigned int g_uiOFOCount = 0;
struct timeval g_stOFOStartTime = {0, };
struct timeval g_stOFOEndTime = {0, };
#endif /* CCIC_OFO_DUMP */ 

static int giSwitchCamera = 1;

#define CCIC_VERSION 0x000001
/*
 * Parameters.
 */
MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("Marvell 88ALP01 CMOS Camera Controller driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("Video");

/*
 * Registers
 */

/*
 * CSI register base: 0x50020000
 */
#define REG_CSSCR		0x0000
#define REG_CSGCR		0x000C
#define REG_CSxCR0		0x0010
#define REG_CSxSR		0x0014
#define REG_CSxINEN		0x0018
#define REG_CSxINST		0x001C
#define REG_CSxTIM0		0x0020
#define REG_CSxTIM1		0x0024
#define REG_CSxGENDAT		0x0028
#define REG_CSxPHYCAL		0x002C
#define REG_CSxDTINTLV		0x0030

/* CSxCR0 */
#define CSxCR0_CSIEN	0x0001

/* CSxSR */
#define CSxSR_CSIEN	0x0001

/* CAM_CSI_CONT_CSxCR0 bits */
#define CAM_CSI_CONT_CSI_EN            (0x1u<<0)  /* CSI controller Enable/Disable*/
#define CAM_CSI_CONT_NOL(n)            ((n)<<2)   /* Number of Active Lanes (2 bits)*/
#define CAM_CSI_CONT_VC0_CFG(n)        ((n)<<5)   /* Chan 0 Addr Conf (2 bits)*/
#define CAM_CSI_CONT_VC1_CFG(n)        ((n)<<7)   /* Chan 1 Addr Conf (2 bits)*/
#define CAM_CSI_CONT_VC2_CFG(n)        ((n)<<9)   /* Chan 2 Addr Conf (2 bits)*/
#define CAM_CSI_CONT_VC3_CFG(n)        ((n)<<11)  /* Chan 3 Addr Conf (2 bits)*/


/*CAM_CSI_CONT_CSxTIM0 bits*/
#define CAM_CSI_CONT_CLTERMEN(n)       ((n)<<0)   /* Time to wait before enabling clock HS termination (8 bits)*/
#define CAM_CSI_CONT_CLSETTLE(n)       ((n)<<8)   /* Time to wait before HS clock is valid (8 bits)*/
#define CAM_CSI_CONT_CLMISS(n)         ((n)<<16)  /* Time to detect that the clock has stopped toggling (8 bits)*/
#define CAM_CSI_CONT_HSTERMEN(n)       ((n)<<24)  /* Time to wait before enabling data HS termination (8 bits)*/

/* CAM_CSI_CONT_CSxTIM1 bits */
#define CAM_CSI_CONT_HS_Rx_TO(n)       ((n)<<0)   /* Time to wait before declaring an error on a packet reception. This counter counts on the escape mode clock which is 52 MHz (16 bits)*/
#define CAM_CSI_CONT_HSTSETTLE(n)      ((n)<<16)  /* Timeout at RX end to neglect transition effects (8 bits)*/

/* CAM_CSI_CONT_CSxPHYCAL bits */
#define CAM_CSI_CONT_MIPI_BG_VREF_EN   (0x1u<<0)  /* See DPHY specifications for details*/
#define CAM_CSI_CONT_MIPI_RCOMP_CLKSEL (0x1u<<1)  /* Used to control the source of the PHY calibration clock (External/Internal).*/
#define CAM_CSI_CONT_MIPI_RCOMP_LOAD   (0x1u<<2)  /* Used to enable loading the MIPI_REN bypass value*/
#define CAM_CSI_CONT_MIPI_REN_BYPASS(n) ((n)<<3)  /* An 8-bit value to bypass the internal MIPI_REN value.*/
#define CAM_CSI_CONT_MIPI_RESET        (0x1u<<10) /* Mipi Reset Bit for pxa955 instead of 8th bit of REN_BYPASS*/
#define CAM_CSI_CONT_MIPI_RCOMP_CLK_EXT (0x1u<<31) /* Used tp provide a SW generated extternal clock to the PHY calibration block.*/

/* CAM_CSI_MCLK_CSGCR bits */
#define CAM_CSI_CLK_DIV(n)             ((n)<<0)  /* Generated Clock Divisor. GCLK = 1/(DIV+2). 12 bits*/
#define CAM_CSI_CLK_GCLK_EN            (0x1u<<16)/* Generated Clock Enable*/

/* CAM_CSI_CONT_CSxINEN bits */
#define CAM_CSI_CONT_CSI_EN_INT        (0x1u<<0)  /* ENABLE Interrupt Enable*/
#define CAM_CSI_CONT_SoF_INT_EN        (0x1u<<1)  /* Start of Frame Interrupt Enable*/
#define CAM_CSI_CONT_EoF_INT_EN        (0x1u<<2)  /* End of Frame Interrupt Enable*/
#define CAM_CSI_CONT_OFLOW_INT_EN      (0x1u<<3)  /* Overflow Interrupt Enable*/
#define CAM_CSI_CONT_PHY_ERR_EN        (0x1u<<4)  /* D-PHY-related Error Interrupt Enable*/
#define CAM_CSI_CONT_GEN_PACK_INT_EN   (0x1u<<5)  /* Generic Packet Interrupt Enable*/
#define CAM_CSI_CONT_TIMEOUT_EN        (0x1u<<8)  /* Timeout Interrupt Enable*/
#define CAM_CSI_CONT_PROT_EN           (0x1u<<12) /* Protection Interupt Enable*/

/*
 * SCI register
 * SCI0 base: 0x50000000
 */
#define REG_SCICR0	0x0000
#define REG_SCICR1	0x0004
#define REG_SCISR	0x0008
#define REG_SCIMASK	0x000C
#define REG_SCIFIFO	0x00F8
#define REG_SCIFIFOSR	0x00FC
#define REG_SCIDADDR0	0x0200
#define REG_SCISADDR0	0x0204
#define REG_SCITADDR0	0x0208
#define REG_SCIDCMD0	0x020C
#define REG_SCIDADDR1	0x0210
#define REG_SCISADDR1	0x0214
#define REG_SCITADDR1	0x0218
#define REG_SCIDCMD1	0x021C
#define REG_SCIDADDR2	0x0220
#define REG_SCISADDR2	0x0224
#define REG_SCITADDR2	0x0228
#define REG_SCIDCMD2	0x022C
#define REG_SCIDBR0	0x0300
#define REG_SCIDCSR0	0x0304
#define REG_SCIDBR1	0x0310
#define REG_SCIDCSR1	0x0314
#define REG_SCIDBR2	0x0320
#define REG_SCIDCSR2	0x0324

#define	IRQ_EOFX	0x00000001
#define	IRQ_EOF		0x00000002
#define	IRQ_DIS		0x00000004
#define	IRQ_OFO		0x00000008
#define	IRQ_DBS		0x00000200
#define	IRQ_SOFX	0x00000400
#define	IRQ_SOF		0x00000800
#define	IRQ_FRAME 	(IRQ_EOF|IRQ_EOFX|IRQ_SOF|IRQ_SOFX)


/* FMT_YUV420 & FMT_RGB666 only for input format without output format */
#define   FMT_RAW8	  0x0000
#define   FMT_RAW10	  0x0002
#define   FMT_YUV422	  0x0003
#define   FMT_YUV420	  0x0004
#define   FMT_RGB565	  0x0005
#define   FMT_RGB666	  0x0006
#define   FMT_RGB888	  0x0007
#define   FMT_JPEG	  0x0008
#define   FMT_YUV422PACKET	0x0009

#define SCICR1_FMT_OUT(n)       ((n) <<12)	/* Output Pixel Format (4 bits) */
#define SCICR1_FMT_IN(n)        ((n) <<28)	/* Input Pixel Format (4 bits) */

/* REG_SCIDCSR0 */
#define SCIDCSR_DMA_RUN        0x80000000  	/* DMA Channel Enable */

/* REG_SCICR0 */
#define SCICR0_CAP_EN          0x40000000 	/* Capture Enable */
#define SCICR0_CI_EN           0x80000000 	/* Camera Interface Enable */

/* REG_SCIFIFO */
#define SCIFIFO_Fx_EN        0x0010	/* FIFO 0 Enable */

#define SCIFIFO_F0_EN        0x0010	/* FIFO 0 Enable */
#define SCIFIFO_F1_EN        0x0020	/* FIFO 1 Enable */
#define SCIFIFO_F2_EN        0x0040	/* FIFO 2 Enable */
#define SCIFIFO_FX_EN(n)  ((n) <<4)		/* Input Pixel Format (4 bits) */

/* REG_SCIDBRx*/
#define SCIDBR_EN 		(0x1u<<1) /*DMA Branch Enable*/

#define VGA_WIDTH       640
#define VGA_HEIGHT      480

#define SENSOR_MAX 2

/* Define following macro to change arbitration register
 * for high camera priority on CI AXI arbitration */
/*#define _ARB_CHANGE_*/
#ifdef _ARB_CHANGE_
#define ARB_CNTRL_AXI   0x55D10000
#define ARB_CNTRL_CI1   0x55D10020
#define ARB_CNTRL_CI2   0x55D10040
#define ARB_CNTRL_GCU   0x55D10060
static unsigned int *pri_axi, *pri_ci1, *pri_ci2, *pri_gcu;
#endif

/* During sensor format change, its output may get unstable, if controller
 * is turned on immediately, some unreasonable input may drive controller
 * into a deadloop reporting errors repeatedly. */
/* Define following macro to enable feature "controller reset when deadloop"
 * Add reset mechanism to recover controller from the deadloop*/
#define _CONTROLLER_DEADLOOP_RESET_

/*
 * Internal DMA buffer management.  Since the controller cannot do S/G I/O,
 * we must have physically contiguous buffers to bring frames into.
 * These parameters control how many buffers we use, whether we
 * allocate them at load time (better chance of success, but nails down
 * memory) or when somebody tries to use the camera (riskier), and,
 * for load-time allocation, how big they should be.
 *
 * The controller can cycle through three buffers.  We could use
 * more by flipping pointers around, but it probably makes little
 * sense.
 */
#define CAM_NAME "pxa95x-camera"
#define CSI_NAME "pxa95x-camera-csi"
#define DMA_POOL 0

/* csi only accept 422 and transfer to 420 */
#define CONVERT_420_422 1
#define JPEG_DISCARD_WORKAROUND
#define JPEG_COMPRESS_RATIO_HIGH 20
#define CHANNEL_NUM 3 		/*YUV*/
#define MAX_DMA_BUFS 4
#define MIN_DMA_BUFS 1
#define MAX_DMA_SIZE (1920*1080*3/2)
#define FULL_JPEG_BUFFER_SIZE (1024*1024*3/2)
#define DMA_DUMMY_SIZE (1024*4) /*4k bytes as dummy buf*/
static int dvfm_dev_idx;

static int alloc_bufs_at_read = 0;
static int n_dma_bufs = MAX_DMA_BUFS;	/*frame buffers number*/
static int dma_buf_size = MAX_DMA_SIZE;  /* Worst case for JPEG */
static int min_buffers = MIN_DMA_BUFS;
static int max_buffers = MAX_DMA_BUFS;
static int flip = 0;
struct cam_platform_data *init_platform_ops = NULL;

module_param(alloc_bufs_at_read, bool, 0444);
MODULE_PARM_DESC(alloc_bufs_at_read,
		"Non-zero value causes DMA buffers to be allocated when the "
		"video capture device is read, rather than at module load "
		"time.  This saves memory, but decreases the chances of "
		"successfully getting those buffers.");

module_param(n_dma_bufs, uint, 0644);
MODULE_PARM_DESC(n_dma_bufs,
		"The number of DMA buffers to allocate.  Can be either two "
		"(saves memory, makes timing tighter) or three.");

module_param(dma_buf_size, uint, 0444);
MODULE_PARM_DESC(dma_buf_size,
		"The size of the allocated DMA buffers.  If actual operating "
		"parameters require larger buffers, an attempt to reallocate "
		"will be made.");

module_param(min_buffers, uint, 0644);
MODULE_PARM_DESC(min_buffers,
		"The minimum number of streaming I/O buffers we are willing "
		"to work with.");

module_param(max_buffers, uint, 0644);
MODULE_PARM_DESC(max_buffers,
		"The maximum number of streaming I/O buffers an application "
		"will be allowed to allocate.  These buffers are big and live "
		"in vmalloc space.");

module_param(flip, bool, 0444);
MODULE_PARM_DESC(flip,
		"If set, the sensor will be instructed to flip the image "
		"vertically.");


enum ccic_state {
	S_NOTREADY,	/* Not yet initialized */
	S_IDLE,		/* Just hanging around */
	S_FLAKED,	/* Some sort of problem */
	S_SINGLEREAD,	/* In read() */
	S_SPECREAD,   	/* Speculative read (for future read()) */
	S_STREAMING	/* Streaming data */
};

/*
 * Tracking of streaming I/O buffers.
 */
struct ccic_sio_buffer {
	struct list_head list;
	struct v4l2_buffer v4lbuf;
	char *buffer;   /* Where it lives in kernel space */
	int mapcount;
	struct ccic_camera *cam;
	dma_addr_t dma_handles;
	struct page *page;
};

/* descriptor needed for the PXA DMA engine */
struct pxa_cam_dma {
	dma_addr_t		sg_dma;
	struct pxa_dma_desc	*sg_cpu;
	size_t			sg_size;
	int                     sglen;
};

struct pxa_buf_node {
	struct list_head list;
	int order;	/* Internal buffer addresses */
	void *dma_bufs;	/* Internal buffer addresses */
	dma_addr_t dma_handles; /* Buffer bus addresses */
	struct page *page;
	struct pxa_cam_dma dma_desc[CHANNEL_NUM];
};

struct ccic_csi
{
	unsigned char __iomem *regs;
	int irq;

	spinlock_t dev_lock;  /* Access to device */
};

struct ccic_csi *g_csi;

struct stCamacqSensor_t;
struct stCamacqSensorManager_t;
volatile struct pxa_buf_node * g_pstFrame;
/*
 * A description of one of our devices.
 * Locking: controlled by s_mutex.  Certain fields, however, require
 * 	    the dev_lock spinlock; they are marked as such by comments.
 *	    dev_lock is also required for access to device registers.
 */
struct ccic_camera
{
	enum ccic_state state;
	unsigned long flags;   		/* Buffer status, mainly (dev_lock) */
	int users;			/* How many open FDs */
	struct file *owner;		/* Who has data access (v4l2) */

	/*
     *   Sensor driver 
     */
    struct stCamacqSensorManager_t* pstSensorManager;
    int                             iSelectedSensor;

    // bool                            bIsPowerOn;

	/*
	 * Subsystem structures.
	 */
	int irq;
	struct platform_device *pdev;
	struct video_device v4ldev;
	struct i2c_adapter i2c_adapter;

	unsigned char __iomem *regs;
	struct list_head dev_list;	/* link to other devices */

	/* DMA buffers */
	unsigned int nbufs;		/* How many bufs used by HW DMA chain (the bufs in dma_buf_list)*/
	unsigned int n_mapbufs;		/* how many bufs we have allocated, as buf pool for mmap method */
	struct pxa_buf_node *next_buf;	/* Next to consume (dev_lock) */
	unsigned int dma_buf_size;  	/* allocated size */
	struct list_head dma_buf_list; 	/*dma buffer list, the list member is buf_node*/
	struct pxa_buf_node *buf_node; 	/*a frame buf, with all data info used by DMA */
	struct pxa_buf_node *buf_dummy;	/*a dummy buf for only one buf case, for DMA dummy loop*/
	struct pxa_buf_node *buf_mmap;	/*the buf which is allocated by driver, usr mmap this buf*/
	int channels;
	unsigned int channel_size[CHANNEL_NUM];

	unsigned int specframes;	/* Unconsumed spec frames (dev_lock) */

	/* Streaming buffers */
	unsigned int nskipbufs;
	unsigned int n_sbufs;		/* How many bufs usr has required (the bufs in sb_avail and sb_full lists)  */
	struct ccic_sio_buffer *sb_bufs; /* The array of housekeeping structs */
	struct list_head sb_avail;	/* Available for data (we own) (dev_lock) */
	struct list_head sb_full;	/* With data (user space owns) (dev_lock) */

	/* Current operating parameters */
	struct v4l2_pix_format pix_format;

	/* Locks */
	struct mutex s_mutex; /* Access to this structure */
	spinlock_t dev_lock;  /* Access to device */

	/* Misc */
	wait_queue_head_t iowait;	/* Waiting on frame data */


	struct clk *clk;

	/* platform data */
	struct cam_platform_data *platform_ops;

	unsigned int discard;
	unsigned int requestbufs;
	unsigned long io_type;

#ifdef _CONTROLLER_DEADLOOP_RESET_
	/* A timer to detect controller error, on which controller will be reset */
	struct timer_list reset_timer;
	struct work_struct reset_wq;
	/* This var tells anyone who trys to modify reset timer
	 * it is being killed, don't modify it. otherwise, it'll restart again*/
	int killing_reset_timer;
#endif
};

/* Flash related constants */
enum flash_arg_format {
	FLASH_ARG_TIME_OFF	= 0UL,
	FLASH_ARG_MODE_0	= 0x00,
	FLASH_ARG_MODE_1	= 0x10,
	FLASH_ARG_MODE_2	= 0x20,
	FLASH_ARG_MODE_3	= 0x30,
	FLASH_ARG_MODE_EVER	= FLASH_ARG_MODE_0, /* Always on till send off  */
	FLASH_ARG_MODE_TIMED	= FLASH_ARG_MODE_1, /* On for some time */
	FLASH_ARG_MODE_REPEAT	= FLASH_ARG_MODE_2, /* Repeatedlly on */
	FLASH_ARG_MODE_BLINK	= FLASH_ARG_MODE_3, /* On for some jeffies defined by H/W */
	FLASH_ARG_MODE_MAX,	/* Equal or greater mode considered invalid */
	FLASH_ARG_LUMI_OFF	= 0x00, /* Flash off */
	FLASH_ARG_LUMI_FULL	= 0x01,
	FLASH_ARG_LUMI_DIM1	= 0x02,	/* One fifth luminance*/
	FLASH_ARG_LUMI_DIM2	= 0x03, /* One third luminance, not supported in H/W configuration */
	FLASH_ARG_LUMI_MAX	= 0x03, /* Equal or greater luminance considered invalid */
};

/* Len should 8 bytes align, bit[2:0] should be 0 */
#define SINGLE_DESC_TRANS_MAX   (1 << 24)


/*
 * Status flags.  Always manipulated with bit operations.
 */
#define CF_BUF0_VALID	 0	/* Buffers valid - first three */
#define CF_BUF1_VALID	 1
#define CF_BUF2_VALID	 2
#define CF_DMA_ACTIVE	 3	/* A frame is incoming */
#define CF_CONFIG_NEEDED 4	/* Must configure hardware */

static int detected_high = 0;
static int detected_low = 0;
static int sensor_selected = SENSOR_NONE;
static void csi_config(struct ccic_csi *csi, int iResType);

#ifdef _CONTROLLER_DEADLOOP_RESET_
#define MIPI_RESET_TIMEOUT (msecs_to_jiffies(300))	/* For jpeg fps=10fps, so max polling time could be 200ms, */
static void ccic_timeout_handler(unsigned long);
#endif
#define _CAM_DEBUG_V4L2OPEN_
#ifdef _CAM_DEBUG_V4L2OPEN_
/* Flash sysfs interface, will invoke sensor function thru __ccic_cam_cmd */
static struct led_classdev camflash_led_cdev;

struct cam_classdev {
	int detect_fail;
	int vdev_reg;
	int vdev_unreg;
	int s_input_fail;
};

struct cam_classdev g_cam_cdev;

EXPORT_SYMBOL(g_cam_cdev);

#endif
/*
 * Start over with DMA buffers - dev_lock needed.
 */
static void ccic_reset_buffers(struct ccic_camera *cam)
{
    pxa950_camera_dbg_v(""); 
	cam->next_buf = NULL;
	cam->specframes = 0;
}

static inline int ccic_needs_config(struct ccic_camera *cam)
{
    pxa950_camera_dbg_v(""); 
	return test_bit(CF_CONFIG_NEEDED, &cam->flags);
}

static void ccic_set_config_needed(struct ccic_camera *cam, int needed)
{
    pxa950_camera_dbg_v("");
    
	if (needed)
		set_bit(CF_CONFIG_NEEDED, &cam->flags);
	else
		clear_bit(CF_CONFIG_NEEDED, &cam->flags);
}




/*
 * Debugging and related.
 */
#define cam_err(cam, fmt, arg...) \
	dev_err(&(cam)->pdev->dev, fmt, ##arg);
#define cam_warn(cam, fmt, arg...) \
	dev_warn(&(cam)->pdev->dev, fmt, ##arg);
#define cam_dbg(cam, fmt, arg...) \
	dev_dbg(&(cam)->pdev->dev, fmt, ##arg);


/* ---------------------------------------------------------------------*/
/*
 * We keep a simple list of known devices to search at open time.
 */
static LIST_HEAD(ccic_dev_list);
static DEFINE_MUTEX(ccic_dev_list_lock);
static int __ccic_cam_cmd(struct ccic_camera *cam, int cmd, void *arg);
static int __ccic_cam_reset(struct ccic_camera *cam);

static void ccic_add_dev(struct ccic_camera *cam)
{
    pxa950_camera_dbg_v("");

	mutex_lock(&ccic_dev_list_lock);
	list_add_tail(&cam->dev_list, &ccic_dev_list);
	mutex_unlock(&ccic_dev_list_lock);
}

static void ccic_remove_dev(struct ccic_camera *cam)
{
    pxa950_camera_dbg_v("");
    
	mutex_lock(&ccic_dev_list_lock);
	list_del(&cam->dev_list);
	mutex_unlock(&ccic_dev_list_lock);
}

static struct ccic_camera *ccic_find_dev(int minor)
{
	struct ccic_camera *cam;

    pxa950_camera_dbg_v("");
    
	mutex_lock(&ccic_dev_list_lock);
	list_for_each_entry(cam, &ccic_dev_list, dev_list) {
		if (cam->v4ldev.minor == minor)
			goto done;
	}
	cam = NULL;
  done:
	mutex_unlock(&ccic_dev_list_lock);
	return cam;
}

static struct ccic_camera *ccic_find_by_pdev(struct platform_device *pdev)
{
	struct ccic_camera *cam;

    pxa950_camera_dbg_v("");

	mutex_lock(&ccic_dev_list_lock);
	list_for_each_entry(cam, &ccic_dev_list, dev_list) {
		if (cam->pdev == pdev)
			goto done;
	}
	cam = NULL;
  done:
	mutex_unlock(&ccic_dev_list_lock);
	return cam;
}


/* ------------------------------------------------------------------------ */
/*
 * Device register I/O
 */
static inline void ccic_reg_write(struct ccic_camera *cam, unsigned int reg,
		unsigned int val)
{
    pxa950_camera_dbg_v("");
    
	__raw_writel(val, cam->regs + reg);
}

static inline unsigned int ccic_reg_read(struct ccic_camera *cam,
		unsigned int reg)
{
    pxa950_camera_dbg_v("");
    
	return __raw_readl(cam->regs + reg);
}


static inline void ccic_reg_write_mask(struct ccic_camera *cam, unsigned int reg,
		unsigned int val, unsigned int mask)
{
	unsigned int v = ccic_reg_read(cam, reg);

    pxa950_camera_dbg_v("");

	v = (v & ~mask) | (val & mask);
	ccic_reg_write(cam, reg, v);
}

static inline void ccic_reg_clear_bit(struct ccic_camera *cam,
		unsigned int reg, unsigned int val)
{
    pxa950_camera_dbg_v("");
    
	ccic_reg_write_mask(cam, reg, 0, val);
}

static inline void ccic_reg_set_bit(struct ccic_camera *cam,
		unsigned int reg, unsigned int val)
{
    pxa950_camera_dbg_v("");
    
	ccic_reg_write_mask(cam, reg, val, val);
}

/*provided for sensor calling to enable clock at begining of probe*/
void ccic_set_clock(unsigned int reg, unsigned int val)
{
	struct ccic_camera *cam;

    pxa950_camera_dbg_v("");
    
	cam = ccic_find_dev(0);
	ccic_reg_write(cam, reg, val);
}

unsigned int ccic_get_clock(unsigned int reg)
{
        struct ccic_camera *cam;

    pxa950_camera_dbg_v("");
    
        cam = ccic_find_dev(0);
	return ccic_reg_read(cam, reg);
}
/*
	1. APMU CCIC clock reset control
	2. CCIC power on
	3. enable MCLK
*/

/* Somebody is on the bus */
static int __attribute__ ((unused)) ccic_cam_init(struct ccic_camera *cam);
static void ccic_ctlr_stop_dma(struct ccic_camera *cam);

static void csi_start(struct ccic_csi *csi, int camif);
static void csi_stop(struct ccic_csi *csi);
static void csi_reset(struct ccic_csi *csi);
static void csi_mipi_config(struct ccic_csi *csi, unsigned int width, int iresType);
static void __attribute__ ((unused)) dump_csi_registers(struct ccic_csi *csi);
static void __attribute__ ((unused)) dump_registers(struct ccic_camera *cam);
static void __attribute__ ((unused)) dump_dma_desc(struct ccic_camera *cam);
static void __attribute__ ((unused))dump_dma_buf_list(struct ccic_camera *cam);
static void __attribute__ ((unused))dump_avail_buf_list(struct ccic_camera *cam);
static void __attribute__ ((unused))dump_dummy_buf_desc(struct ccic_camera *cam);

int ccic_sensor_attach( struct stCamacqSensorManager_t *pstSensorManager )
{
	struct ccic_camera *cam = NULL;
	int ret = 0;
	int i = 0;
    struct stCamacqSensor_t* pstSensor = NULL;
	
	pxa950_camera_dbg_in();

    if( pstSensorManager == NULL )
    {
        printk(KERN_ERR "pstSensorManager == NULL\n");
        return -ENODEV;
    }

	for( i = 0; i < CCIC_NUM; i++ ) 
    {
		cam = ccic_find_dev(i);
		if( cam == NULL ) 
        {
			printk(KERN_ERR "didn't find camera device!\n");
			return -ENODEV;
		}
		pxa950_camera_dbg( " : i[%d] cam[%x]", i, (U32)cam );

		mutex_lock(&cam->s_mutex);

        cam->pstSensorManager = pstSensorManager;
        cam->iSelectedSensor = i; // 0 is main sensor, 1 is sub sensor in camacq driver
		pxa950_camera_dbg( " : cam->pstSensorManager [%x]", (U32)cam->pstSensorManager );

        pstSensor = cam->pstSensorManager->GetSensor( cam->pstSensorManager, i );
        pxa950_camera_dbg( " : pstSensor [%x]", (u32)pstSensor );

        if( pstSensor != NULL )
        {
            if( pstSensor->m_uiResType == 1 )
            {
                pxa950_camera_dbg( " : attached LOW, i[%d], pstSensor->iResType[%d]", i, pstSensor->m_uiResType );
                detected_low = 1;
            }
            else if( pstSensor->m_uiResType == 0 )
            {
                pxa950_camera_dbg( " : attached HIGH, i[%d], pstSensor->iResType[%d]", i, pstSensor->m_uiResType );
                detected_high = 1;
            }
            else
            {
                pxa950_camera_dbg( " : attach ERROR!!, i[%d], pstSensor->iResType[%d]", i, pstSensor->m_uiResType );
                return -ENODEV;
            }
        }
        else
        {
            pxa950_camera_err("pstSensor is NULL, i=%d", i);
        }
        		
		/* Get/set parameters? */
		cam->state = S_IDLE;
		mutex_unlock(&cam->s_mutex);
	}

    pstSensorManager->m_stCamBlock.pdev = cam->pdev;

    pxa950_camera_dbg_out();
	return ret;
	
}
EXPORT_SYMBOL(ccic_sensor_attach);

void force_v4l2_unregister()
{
    struct ccic_camera *cam;
	int i, ret;

	pxa950_camera_dbg_in();

	for (i = 0; i < CCIC_NUM; i++)
	{
		cam = ccic_find_dev(i);
		if (cam == NULL) {
			cam_err(cam, "[cam]: faild to find cam in detach!\n");
			continue;
		}		
	}

    if( cam )
    {
        video_unregister_device(&cam->v4ldev);     
#ifdef _CAM_DEBUG_V4L2OPEN_
	g_cam_cdev.vdev_unreg++;
#endif  
        pxa950_camera_err("video_unregister_device");
    }

	pxa950_camera_dbg_out();
}
EXPORT_SYMBOL(force_v4l2_unregister);

/* UL4D 20100506 s*/
int ccic_sensor_detach(struct i2c_client *client)
//static int __attribute__ ((unused)) ccic_sensor_detach(struct i2c_client *client)
/* UL4D 20100506 e*/
{
#if 0
	struct ccic_camera *cam;
	int i;

	pxa950_camera_dbg_in();

	for (i = 0; i < CCIC_NUM; i++)
	{
		cam = ccic_find_dev(i);
		if (cam == NULL) {
			cam_err(cam, "[cam]: faild to find cam in detach!\n");
			continue;
		}
		if (cam->psensor->pclient == client) {
			ccic_ctlr_stop_dma(cam);
			cam_err(cam, "lost the sensor!\n");
			cam->psensor = NULL;  /* Bummer, no camera */
			cam->state = S_NOTREADY;
		}
	}

	pxa950_camera_dbg_out();
#endif	
	return 0;
}


static void __attribute__ ((unused))dump_dma_desc(struct ccic_camera *cam)
{
	int i, k;
	struct pxa_buf_node *buf_node;
	printk(KERN_ERR "cam: dump_dma_desc ************+\n");

	printk(KERN_ERR "dma_buf_list head 0x%x\n",(unsigned int)&cam->dma_buf_list);
	list_for_each_entry(buf_node, &cam->dma_buf_list, list)  {
			printk(KERN_ERR "buf_node 0x%x\n",(unsigned int)buf_node);

		for (i = 0; i < cam->channels; i++){
			printk(KERN_ERR "chnnl %d, chnnl_size %d, sglen %d, sgdma 0x%x, sgsze %d\n",
				i,cam->channel_size[i],buf_node->dma_desc[i].sglen,buf_node->dma_desc[i].sg_dma,buf_node->dma_desc[i].sg_size);
			for (k = 0; k < buf_node->dma_desc[i].sglen; k++) {
				printk(KERN_ERR "dcmd [%d]  0x%x\n",k,buf_node->dma_desc[i].sg_cpu[k].dcmd);
				printk(KERN_ERR "ddadr[%d]  0x%x\n",k,buf_node->dma_desc[i].sg_cpu[k].ddadr);
				printk(KERN_ERR "dsadr[%d]  0x%x\n",k,buf_node->dma_desc[i].sg_cpu[k].dsadr);
				printk(KERN_ERR "dtadr[%d]  0x%x\n\n",k,buf_node->dma_desc[i].sg_cpu[k].dtadr);
			}
		}
	}

	printk(KERN_ERR "cam: dump_dma_desc ************-\n\n");
}

static void __attribute__ ((unused))dump_dma_buf_list(struct ccic_camera *cam)
{
	struct pxa_buf_node *buf_node;
	printk(KERN_ERR "cam: dump_dma_buf_list ************+\n");
	list_for_each_entry(buf_node, &cam->dma_buf_list, list)
		printk(KERN_ERR "cam: buf_node 0x%x, pa 0x%x, va 0x%x\n",
			(unsigned int)buf_node, (unsigned int)buf_node->dma_handles, (unsigned int)buf_node->dma_bufs);

	printk(KERN_ERR "cam: dump_dma_buf_list ************-\n\n");
}
static void __attribute__ ((unused))dump_avail_buf_list(struct ccic_camera *cam)
{
	struct ccic_sio_buffer *sbuf;

	printk(KERN_ERR "cam: dump_avail_buf_list ************+\n");
	printk(KERN_ERR "cam: head 0x%x\n", (unsigned int)&cam->sb_avail);
	list_for_each_entry(sbuf, &cam->sb_avail, list) {
		printk(KERN_ERR "cam: sbuf 0x%x, pa 0x%x\n", (unsigned int)sbuf, (unsigned int)sbuf->dma_handles);
		printk(KERN_ERR "cam: list 0x%x, next 0x%x, prev 0x%x\n",
			(unsigned int)&sbuf->list, (unsigned int)sbuf->list.next, (unsigned int)sbuf->list.prev);
	}

	printk(KERN_ERR "cam: dump_avail_buf_list ************-\n\n");
}

static void __attribute__ ((unused))dump_dummy_buf_desc(struct ccic_camera *cam)
{
	int i, k;
	struct pxa_buf_node *buf_node;

	printk(KERN_ERR "cam: dump_dummy_buf_desc ************+\n");
	buf_node = cam->buf_dummy;
	for (i = 0; i < cam->channels; i++){
		printk(KERN_ERR "chnnl %d, chnnl-size %d, sglen %d, sgdma 0x%x, sgsze %d\n",
			i,cam->channel_size[i],buf_node->dma_desc[i].sglen,buf_node->dma_desc[i].sg_dma,buf_node->dma_desc[i].sg_size);
		for (k = 0; k < buf_node->dma_desc[i].sglen; k++) {
			printk(KERN_ERR "dcmd [%d]  0x%x\n",k,buf_node->dma_desc[i].sg_cpu[k].dcmd);
			printk(KERN_ERR "ddadr[%d]  0x%x\n",k,buf_node->dma_desc[i].sg_cpu[k].ddadr);
			printk(KERN_ERR "dsadr[%d]  0x%x\n",k,buf_node->dma_desc[i].sg_cpu[k].dsadr);
			printk(KERN_ERR "dtadr[%d]  0x%x\n\n",k,buf_node->dma_desc[i].sg_cpu[k].dtadr);
		}
	}

	printk(KERN_ERR "cam: dump_dummy_buf_desc ************-\n\n");
}

const u8 dma_mark[4] = {0xff,0xff,0x00,0x00};

/*
 * Only handle in irq context
 */
static struct pxa_buf_node* ccic_take_frame(struct ccic_camera *cam)
{
	struct pxa_buf_node *frame = NULL;
	int node_num = 0;
	unsigned long flags;
	int j;
	u8 *magic_bit;
	bool buf_filled = false;

    pxa950_camera_dbg_v();

	spin_lock_irqsave(&cam->dev_lock, flags);
	if (cam->requestbufs != MIN_DMA_BUFS) {
		list_for_each_entry(frame, &cam->dma_buf_list, list) {
			node_num++;
		}
		if (node_num > 1) {
			/*get the first node of dma_list, it must have been filled by dma*/
			frame = list_entry(cam->dma_buf_list.next, struct pxa_buf_node, list);
		} else {
			/*if there is only dummy-node left in dma_list, drop it!*/
			frame = NULL;
			//pxa950_camera_dbg_v("cam: drop a frame!\n");
			printk("cam: drop a frame====================\n");
		}
	} else { /* handle one buf case */
		if (!list_empty(&cam->dma_buf_list)) {
			frame = list_entry(cam->dma_buf_list.next, struct pxa_buf_node, list);
			pxa950_camera_dbg_v("cam: ccic_take_frame dma 0x%x\n", frame->dma_handles);
			dma_sync_single_for_device(&cam->pdev->dev,
									frame->dma_handles,
									cam->channel_size[0],
									DMA_FROM_DEVICE);
			magic_bit = (u8*)frame->dma_bufs;

			for (j = 0; j < sizeof(dma_mark); j++) {
				if (dma_mark[j] != magic_bit[j])
					buf_filled = true;
			}

			if ((!buf_filled)
				|| ((cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG)
				&& ((((char *)frame->dma_bufs)[0] != 0xff)
				|| (((char *)frame->dma_bufs)[1] != 0xd8)))) {
				pxa950_camera_err("cam: bad jpeg or buf no filled!\n");
				frame = NULL;
			}
		}
	}
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return frame;
}

static void ccic_append_dma_desc(struct ccic_camera* cam, struct pxa_buf_node* node1, struct pxa_buf_node* node2)
{
	int i = 0;
	struct pxa_cam_dma *pxa_dma1 = NULL, *pxa_dma2 = NULL;
	/*chain node2 to tail of node1*/
	for (i = 0; i < cam->channels; i++) {
		pxa_dma1 = &node1->dma_desc[i];
		pxa_dma2 = &node2->dma_desc[i];
		pxa_dma1->sg_cpu[pxa_dma1->sglen-1].ddadr = (u32)pxa_dma2->sg_dma;
	}
	//printk(KERN_DEBUG "cam: append new dma 0x%x to 0x%x\n",pxa_dma2->sg_dma, pxa_dma1->sg_dma);
    pxa950_camera_dbg_v("cam: append new dma 0x%x to 0x%x\n",pxa_dma2->sg_dma, pxa_dma1->sg_dma);
}
static void ccic_switch_desc_onebuf(struct ccic_camera *cam, struct pxa_buf_node *frame)
{
	struct ccic_sio_buffer *sbuf = NULL;
	unsigned long flags;
	cam->next_buf = frame;

    pxa950_camera_dbg_v("IN");
	spin_lock_irqsave(&cam->dev_lock, flags);

	/*if the buf has been filled, make sure dma is not polluting it, so we skip some frames, or submit it*/
	if (frame != NULL) {
		/*skip some frames, and loop back dummy, make sure dma will not point to this buf*/
		if (cam->nskipbufs) {
			cam->nskipbufs--;
			ccic_append_dma_desc(cam, cam->buf_dummy, cam->buf_dummy);
			printk(KERN_ERR "cam: drop a frame to make sure dma is not working on it.\n");

			spin_unlock_irqrestore(&cam->dev_lock, flags);
			return;
		} else {
		/*after we skip some frames, submit to usr this buf*/

			printk(KERN_ERR "cam: a valide frame!\n");
			sbuf = list_entry(cam->sb_avail.next,
				struct ccic_sio_buffer, list);
			sbuf->v4lbuf.bytesused = cam->pix_format.sizeimage;
			sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
			sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;

			list_move_tail(&sbuf->list, &cam->sb_full);
			list_del_init(&frame->list);
			if (! list_empty(&cam->sb_full)){
				wake_up(&cam->iowait);
			}
		}
	} else {
	/*if the buf is not filled, get a buf from avail-list, chain this buf to dummy*/
		if ((!list_empty(&cam->sb_avail)) && (list_empty(&cam->dma_buf_list))) {
			list_add_tail(&cam->buf_node[0].list, &cam->dma_buf_list);

			printk(KERN_ERR "cam: append buf node 0x%x(dma 0x%x) to dummy.\n",
				(unsigned int)&cam->buf_node[0], cam->buf_node[0].dma_handles);
			ccic_append_dma_desc(cam, cam->buf_dummy, &cam->buf_node[0]);
			ccic_append_dma_desc(cam, &cam->buf_node[0], cam->buf_dummy);
			cam->nskipbufs = 1;
		}
	}
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}

static void ccic_switch_desc(struct ccic_camera *cam, struct pxa_buf_node *frame)
{
	struct ccic_sio_buffer *sbuf_tmp = NULL, *sbuf;
	struct pxa_buf_node *buf_node;
	struct pxa_buf_node *tail_node;
	unsigned long flags;
	unsigned int regval;
	int i;
	bool exist_buf = false;
	cam->next_buf = frame;
    pxa950_camera_dbg_v("IN");

	spin_lock_irqsave(&cam->dev_lock, flags);

	/*step 1: indicate upper layer a new frame buf of sbuf-node*/
	if (frame != NULL)
	{
		list_for_each_entry(sbuf, &cam->sb_avail, list) {
			if (frame->dma_handles == sbuf->dma_handles) {
				if (cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) {
					dma_map_page(&cam->pdev->dev,
							sbuf->page,
							0,
							sbuf->v4lbuf.length,
							DMA_FROM_DEVICE);
					if ((((char *)frame->dma_bufs)[0] != 0xff)
					|| (((char *)frame->dma_bufs)[1] != 0xd8)) {
						printk(KERN_ERR "cam: JPEG ERROR !!! dropped this frame.\n");
						list_move_tail(&sbuf->list, &cam->sb_avail);
						list_del_init(&frame->list);
						break;		//drop current JPEG frame because of wrong header
					} else {
						printk(KERN_ERR "cam: JPEG done!\n");
                    }
				}
				sbuf->v4lbuf.bytesused = cam->pix_format.sizeimage;
				sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
				sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;
				sbuf_tmp = sbuf;/*this sbuf node will be moved from avail list to full list*/
				break;
			}
		}
	}

	/*step 2: add the avail buf to the tail of dma list, and route the dma desc chain to the new bufs*/
	list_for_each_entry(sbuf, &cam->sb_avail, list) {
		exist_buf = false;
		/* 1) search if this sbuf has existed in dma_buf_list, if yes, ignore this buf, do not append it*/
		list_for_each_entry(buf_node, &cam->dma_buf_list, list){
			if (sbuf->dma_handles == buf_node->dma_handles) {
				exist_buf = true;
				pxa950_camera_dbg_v("cam: the buf_node of 0x%x has exist in dma_buf_list, ignore it.\n",(unsigned int)buf_node);
				break;
			}
		}
		/* 2) we got a avail buf which is not in dma_buf_list, append it!*/
		if (exist_buf == false) {

			/*find the tail of the dma buf list, and then append an avail buf to the tail */
			list_for_each_entry(tail_node, &cam->dma_buf_list, list){
				if (tail_node->list.next == &cam->dma_buf_list)
					break;
			}
			/*search in buf_node pool, find out the buf_node, according to sbuf node*/
			for (i = 0; i < cam->n_sbufs; i++)
			{
				if (sbuf->dma_handles == cam->buf_node[i].dma_handles) {
					regval = ccic_reg_read(cam, REG_SCITADDR0);
					/* NOTE!!! only one desc for one frame buffer, if not in this way, need change
					* here, as phy addr might not located between the begin and end, phy addr
					* might not continuous between different desc of one frame buffer.
					*/
					if ((regval >= tail_node->dma_handles)
						&& (regval < (tail_node->dma_handles + cam->channel_size[0]))) {
						/* if we find DMA is looping in the last buf, and there is new coming buf,
						* (DMA target address shows that DMA is working in the tail buffer)
						* we SHOULD set DMA branch reg, force DMA move to the new buf descriptor
						* in the next frame, so we can pick up this buf when next irq comes.
						*/
						pxa950_camera_dbg_v("cam: REG_SCITADDR0 0x%x\n", regval);
						ccic_reg_write(cam, REG_SCIDBR0, (cam->buf_node[i].dma_desc[0].sg_dma | SCIDBR_EN));
						if(cam->channels == 3) {
							ccic_reg_write(cam, REG_SCIDBR1, (cam->buf_node[i].dma_desc[1].sg_dma | SCIDBR_EN));
							ccic_reg_write(cam, REG_SCIDBR2, (cam->buf_node[i].dma_desc[2].sg_dma | SCIDBR_EN));
						}
					} else {
						/* if it is not the last buf which DMA looping in, just append desc to the tail of the DMA chain*/
                        pxa950_camera_dbg_v("ccic_append_dma_desc");
						ccic_append_dma_desc(cam, tail_node, &cam->buf_node[i]);
					}
					ccic_append_dma_desc(cam, &cam->buf_node[i], &cam->buf_node[i]);
					list_add_tail(&cam->buf_node[i].list, &cam->dma_buf_list);
					/*dump_dma_buf_list(cam);*/
					break;
				}
			}
		}
	}

	/*step 3: remove the sbuf node from avail list, remove buf_node from dma_buf list*/
	if (sbuf_tmp) {
		pxa950_camera_dbg_v("cam: move dma of 0x%x to full_list.\n", sbuf_tmp->dma_handles);
		list_move_tail(&sbuf_tmp->list, &cam->sb_full);
		list_del_init(&frame->list);
	}
	if (! list_empty(&cam->sb_full)){
		wake_up(&cam->iowait);
	}

	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return;
}

/* ------------------------------------------------------------------- */
/*
 * Deal with the controller.
 */
static int ccic_alloc_dma_desc(struct ccic_camera *cam, int index)
{
	int i;
	unsigned int len = 0, len_tmp = 0;

	pxa_dma_desc 	*dma_desc_tmp;
	unsigned long 	dma_desc_phy_tmp;
	unsigned long srcphyaddr, dstphyaddr;
	struct pxa_cam_dma *pxa_dma;

    pxa950_camera_dbg_in("");

	srcphyaddr = 0;	/* TBD */
	dstphyaddr = cam->buf_node[index].dma_handles;

	for (i = 0; i < cam->channels; i++) {
		pxa950_camera_dbg(KERN_DEBUG "cam: index %d, channels %d\n",index, i);
		pxa_dma = &cam->buf_node[index].dma_desc[i];
		len = cam->channel_size[i];
		pxa_dma->sglen = (len + SINGLE_DESC_TRANS_MAX - 1) / SINGLE_DESC_TRANS_MAX;
		pxa_dma->sg_size = (pxa_dma->sglen) * sizeof(struct pxa_dma_desc);
		if (pxa_dma->sg_cpu == NULL){
			pxa_dma->sg_cpu = dma_alloc_coherent(&(cam->pdev->dev), pxa_dma->sg_size,
					     &pxa_dma->sg_dma, GFP_KERNEL);
		}
		pxa950_camera_dbg(KERN_DEBUG "cam: sglen %d, size %d, sg_cpu 0x%x\n",pxa_dma->sglen, pxa_dma->sg_size, (unsigned int)pxa_dma->sg_cpu);
		if (!pxa_dma->sg_cpu){
			pxa950_camera_err( "cam: dma_alloc_coherent failed at chnnl %d!\n", i);
			goto err;
		}

		dma_desc_tmp = pxa_dma->sg_cpu;
		dma_desc_phy_tmp = pxa_dma->sg_dma;

		while (len) {
			len_tmp = len > SINGLE_DESC_TRANS_MAX ?
				SINGLE_DESC_TRANS_MAX : len;

			if ((dstphyaddr & 0xf) != 0) {
				pxa950_camera_err("cam: error: at least we need 16bytes align for DMA!\n");
				goto err;
			}
			dma_desc_tmp->ddadr = dma_desc_phy_tmp + sizeof(pxa_dma_desc);
			dma_desc_tmp->dsadr = srcphyaddr; /* TBD */
			dma_desc_tmp->dtadr = dstphyaddr;
			dma_desc_tmp->dcmd = len_tmp;

			len -= len_tmp;
			dma_desc_tmp++;
			dma_desc_phy_tmp += sizeof(pxa_dma_desc);
			dstphyaddr += len_tmp;

		}
	}
	return 0;

    pxa950_camera_dbg_out("");
err:
	for (i = 0; i < cam->channels; i++) {
		pxa_dma = &cam->buf_node[index].dma_desc[i];
		if (pxa_dma->sg_cpu) {
			dma_free_coherent(&cam->pdev->dev, pxa_dma->sg_size,
				    pxa_dma->sg_cpu, pxa_dma->sg_dma);
			pxa_dma->sg_cpu = 0;
		}
	}
    pxa950_camera_err("err");
	return -ENOMEM;
}

static int ccic_init_dma_desc(struct ccic_camera *cam)
{
	int i = 0;
	int index;
	unsigned int len = 0, len_tmp = 0;
	int ret;
	pxa_dma_desc 	*dma_desc_tmp;
	unsigned long 	dma_desc_phy_tmp;
	struct pxa_cam_dma *pxa_dma;

    //pxa950_camera_dbg_in("");

	if (cam->requestbufs == MIN_DMA_BUFS) {
		/*init the dma desc for dummy dma buf, 4k size*/
		for (i = 0; i < cam->channels; i++) {

			pxa_dma = &cam->buf_dummy->dma_desc[i];
			len = cam->channel_size[i];
			pxa_dma->sglen = (len + DMA_DUMMY_SIZE - 1) / DMA_DUMMY_SIZE;
			pxa_dma->sg_size = (pxa_dma->sglen) * sizeof(struct pxa_dma_desc);
			pxa_dma->sg_cpu = dma_alloc_coherent(&(cam->pdev->dev), pxa_dma->sg_size,
							&pxa_dma->sg_dma, GFP_KERNEL);

		    pxa950_camera_dbg_v("cam: dummy: sglen %d, size %d, sg_cpu 0x%x\n",pxa_dma->sglen, pxa_dma->sg_size, (unsigned int)pxa_dma->sg_cpu);
			if (!pxa_dma->sg_cpu){
				pxa950_camera_err("cam: dummy: dma_alloc_coherent failed at chnnl %d!\n", i);
				return -ENOMEM;
			}

			dma_desc_tmp = pxa_dma->sg_cpu;
			dma_desc_phy_tmp = pxa_dma->sg_dma;

			while (len) {
				len_tmp = len > DMA_DUMMY_SIZE ? DMA_DUMMY_SIZE : len;

				dma_desc_tmp->ddadr = dma_desc_phy_tmp + sizeof(pxa_dma_desc);
				dma_desc_tmp->dsadr = 0; /* TBD */
				dma_desc_tmp->dtadr = cam->buf_dummy->dma_handles;
				dma_desc_tmp->dcmd = len_tmp;

				len -= len_tmp;
				dma_desc_tmp++;
				dma_desc_phy_tmp += sizeof(pxa_dma_desc);
			}
		}
	}

	/*init the dma desc for dma buf, size is cam->channel_size[i]*/
	for (index = 0; index < cam->nbufs; index++) {
		ret = ccic_alloc_dma_desc(cam, index);
		if (ret) {
			pxa950_camera_err(" ret:%d, cam: failed in alloc dma desc!\n", ret);
        }
	}

    //pxa950_camera_dbg_out("");

	return 0;
}

static int ccic_free_dma_desc(struct ccic_camera *cam)
{
	int i = 0;
	int index;
	struct pxa_cam_dma *pxa_dma;

    //pxa950_camera_dbg_in("");

	for (index = 0; index<cam->nbufs; index++) {
		for (i = 0; i < cam->channels; i++) {
			pxa_dma = &cam->buf_node[index].dma_desc[i];
			if (pxa_dma->sg_cpu){
				dma_free_coherent(&cam->pdev->dev, pxa_dma->sg_size,
					    pxa_dma->sg_cpu, pxa_dma->sg_dma);
				pxa_dma->sg_cpu = 0;
			}
		}
	}
/*
	for (i = 0; i < cam->channels; i++) {
		pxa_dma = &cam->buf_dummy->dma_desc[i];
		if (pxa_dma->sg_cpu) {
			dma_free_coherent(&cam->pdev->dev, pxa_dma->sg_size,
					    pxa_dma->sg_cpu, pxa_dma->sg_dma);
			pxa_dma->sg_cpu = 0;
		}
	}
*/
    //pxa950_camera_dbg_out("");

	return -ENOMEM;
}

static int ccic_link_dma_desc(struct ccic_camera *cam)
{
	struct ccic_sio_buffer *sbuf;
	int i = 0;
	int index = 0;
	struct pxa_cam_dma *dma_cur = NULL, *dma_pre = NULL;
	struct pxa_buf_node *buf_cur = NULL, *buf_pre = NULL;
	struct pxa_buf_node *tail_node;

    //pxa950_camera_dbg_in("");

	if (list_empty(&cam->sb_avail)) {
		pxa950_camera_err("cam: there is no buffer en-queue, failed to link dma desc!\n");
		return -EPERM;
	}

	/*chain the buffers in the sb_avail list*/
	list_for_each_entry(sbuf, &cam->sb_avail, list) {
		for (index = 0; index < cam->nbufs; index++) {

			if (sbuf->dma_handles == cam->buf_node[index].dma_handles) {
				list_add_tail(&cam->buf_node[index].list, &cam->dma_buf_list);
				buf_cur = &cam->buf_node[index];

				for (i = 0; i < cam->channels; i++) {
					dma_cur = &buf_cur->dma_desc[i];
					if (buf_pre)
						dma_pre = &buf_pre->dma_desc[i];

					/*head of the dma_buf_list, this is the first dma desc*/
					if (sbuf->list.prev == &cam->sb_avail) {
						ccic_reg_write(cam, REG_SCIDADDR0 + i*0x10, dma_cur->sg_dma);
					} else {
						/* link to prev descriptor */
						if(dma_pre)
							dma_pre->sg_cpu[dma_pre->sglen-1].ddadr = (u32)dma_cur->sg_dma;
					}

					/* find the tail, loop back to the tail itself */
					if (&cam->sb_avail == sbuf->list.next) {
						dma_cur->sg_cpu[dma_cur->sglen-1].ddadr = (u32)dma_cur->sg_dma;
					}
				}
				buf_pre = buf_cur;
			}
		}
	}

	if (cam->requestbufs == MIN_DMA_BUFS) {
		tail_node = list_entry(cam->dma_buf_list.next,
				struct pxa_buf_node, list);
		ccic_append_dma_desc(cam, tail_node, cam->buf_dummy);
		ccic_append_dma_desc(cam, cam->buf_dummy, cam->buf_dummy);
		cam->nskipbufs = 1;
		/*dump_dma_desc(cam);*/
	}

    //pxa950_camera_dbg_out("");
    
	return 0;
}

static int ccic_ctlr_dma(struct ccic_camera *cam)
{
	int ret;
    //pxa950_camera_dbg("");
	ccic_init_dma_desc(cam);
	ret = ccic_link_dma_desc(cam);
	return ret;
}

static void ccic_ctlr_image(struct ccic_camera *cam)
{
	struct v4l2_pix_format *fmt = &cam->pix_format;
	unsigned int size = fmt->width*fmt->height;
	//pxa950_camera_dbg_in();
    pxa950_camera_dbg( " fmt->width : %d, fmt->height : %d, size : %d", fmt->width, fmt->height, size );

	switch (fmt->pixelformat) {

	case V4L2_PIX_FMT_RGB565:
        //pxa950_camera_dbg( " V4L2_PIX_FMT_RGB565 ");
		cam->channels = 1;
		cam->channel_size[0] = size*2;
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_RGB565) | SCICR1_FMT_OUT(FMT_RGB565));
	    break;

	case V4L2_PIX_FMT_JPEG:
        //pxa950_camera_dbg( " V4L2_PIX_FMT_JPEG ");
		cam->channels = 1;
		/* cam->channel_size[0] = size; */
		/* use size get from sensor */
		cam->channel_size[0] = fmt->sizeimage;
        //pxa950_camera_dbg("cam->channel_size[0] : %d", cam->channel_size[0]);
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_JPEG) | SCICR1_FMT_OUT(FMT_JPEG));
	    break;

	case V4L2_PIX_FMT_YUV422P:
        //pxa950_camera_dbg( " V4L2_PIX_FMT_YUV422P ");
		cam->channels = 3;
		cam->channel_size[0] = size;
		cam->channel_size[1] = cam->channel_size[2] = size/2;
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV422));
		break;

	case V4L2_PIX_FMT_UYVY:
        //pxa950_camera_dbg( " V4L2_PIX_FMT_UYVY ");
		cam->channels = 1;
		cam->channel_size[0] = size*2;
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV422PACKET));
		break;

	case V4L2_PIX_FMT_YUV420:
        //pxa950_camera_dbg( " V4L2_PIX_FMT_YUV420 ");
		/* only accept YUV422 as input */
		cam->channels = 3;
		cam->channel_size[0] = size;
		cam->channel_size[1] = cam->channel_size[2] = size/4;
#if CONVERT_420_422
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV420));
#else
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV420) | SCICR1_FMT_OUT(FMT_YUV420));
#endif
		break;

	default:
		pxa950_camera_err("cam: error can not support fmt!\n");
		break;
	}

	//pxa950_camera_dbg_out();

}


/*
 * Configure the controller for operation; caller holds the
 * device mutex.
 */
static int ccic_ctlr_configure(struct ccic_camera *cam)
{
	int ret;
    //pxa950_camera_dbg("");
	ccic_ctlr_image(cam);
	ret = ccic_ctlr_dma(cam);
	return ret;
}

static void ccic_sci_irq_enable(struct ccic_camera *cam)
{
	/*
	 * Clear any pending interrupts, since we do not
	 * expect to have I/O active prior to enabling.
	 */
    //pxa950_camera_dbg_v("");
	ccic_reg_write(cam, REG_SCISR, ccic_reg_read(cam, REG_SCISR));
	ccic_reg_clear_bit(cam, REG_SCIMASK, IRQ_FRAME);
}

static void ccic_sci_irq_disable(struct ccic_camera *cam)
{
    //pxa950_camera_dbg_v("");
	ccic_reg_set_bit(cam, REG_SCIMASK, IRQ_FRAME);
}

/*
 * Make the controller start grabbing images.  Everything must
 * be set up before doing this.
 */
static void ccic_sci_start(struct ccic_camera *cam)
{
	int i = 0;
	unsigned int val = 0;

    //pxa950_camera_dbg_in("");

	/* start_fifo */
	for (i = 0; i < cam->channels; i++) {
		val = SCIFIFO_F0_EN << i;
		ccic_reg_set_bit(cam, REG_SCIFIFO, val);
	}

	/* start_dma */
	for (i = 0; i < cam->channels; i++)
		ccic_reg_set_bit(cam, REG_SCIDCSR0 + i*0x10, SCIDCSR_DMA_RUN);

	/* start sci */
	ccic_reg_set_bit(cam, REG_SCICR0, SCICR0_CAP_EN | SCICR0_CI_EN);

    //pxa950_camera_dbg_out("");
}

static void ccic_sci_stop(struct ccic_camera *cam)
{
	int i = 0;
	unsigned int val = 0;

    //pxa950_camera_dbg_in("");

	/* stop_fifo */
	for (i = 0; i < cam->channels; i++) {
		val = SCIFIFO_F0_EN << i;
		ccic_reg_clear_bit(cam, REG_SCIFIFO, val);
	}

	/* stop_dma */
	for (i = 0; i < cam->channels; i++) {
		ccic_reg_clear_bit(cam, REG_SCIDCSR0 + i*0x10, SCIDCSR_DMA_RUN);
		ccic_reg_clear_bit(cam, REG_SCIDBR0 + i*0x10, SCIDBR_EN);
	}
	/* stop sci */
	ccic_reg_clear_bit(cam, REG_SCICR0, SCICR0_CAP_EN | SCICR0_CI_EN);

    //pxa950_camera_dbg_out("");
}

void ccic_ctlr_init(struct ccic_camera *cam)
{
	unsigned long flags;

    //pxa950_camera_dbg_in("");

	spin_lock_irqsave(&cam->dev_lock, flags);

	/*
	 * Turn off the enable bit.  It sure should be off anyway,
	 * but it's good to be sure.
	 */
	ccic_reg_clear_bit(cam, REG_SCICR0, SCICR0_CI_EN);
	/*
	 * Mask all interrupts.
	 */
	ccic_reg_write(cam, REG_SCIMASK, ~0);
	/*
	 * Clock the sensor appropriately.  Controller clock should
	 * be 48MHz, sensor "typical" value is half that.
	 */
	spin_unlock_irqrestore(&cam->dev_lock, flags);

    //pxa950_camera_dbg_out("");
}
/*
 * Stop the controller, and don't return until we're really sure that no
 * further DMA is going on.
 */
#if 0
static void ccic_ctlr_stop_dma(struct ccic_camera *cam)
{
	unsigned long flags;

    pxa950_camera_dbg_in("");

	/*
	 * Theory: stop the camera controller (whether it is operating
	 * or not).  Delay briefly just in case we race with the SOF
	 * interrupt, then wait until no DMA is active.
	 */

	wait_event_timeout(cam->iowait,
			!test_bit(CF_DMA_ACTIVE, &cam->flags), HZ);
	if (test_bit(CF_DMA_ACTIVE, &cam->flags))
		cam_err(cam, "Timeout waiting for DMA to end\n");

	spin_lock_irqsave(&cam->dev_lock, flags);
	csi_stop(g_csi);
	ccic_sci_stop(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);

	/* This would be bad news - what now? */
	spin_lock_irqsave(&cam->dev_lock, flags);

	cam->state = S_IDLE;
	ccic_sci_irq_disable(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);

	ccic_free_dma_desc(cam);

    pxa950_camera_dbg_out("");
}
#else // 6.0D
static void ccic_ctlr_stop_dma(struct ccic_camera *cam)
{
	unsigned long flags;

	/*
	 * Theory: stop the camera controller (whether it is operating
	 * or not).  Delay briefly just in case we race with the SOF
	 * interrupt, then wait until no DMA is active.
	 */
	if (cam->state != S_IDLE) {
		/* for 30fps, we only wait for one frame time to stop, 33ms */
		wait_event_timeout(cam->iowait,
				!test_bit(CF_DMA_ACTIVE, &cam->flags), msecs_to_jiffies(33));
		if (test_bit(CF_DMA_ACTIVE, &cam->flags))
			printk(KERN_DEBUG "cam: timeout waiting for DMA to end!\n");

		spin_lock_irqsave(&cam->dev_lock, flags);
		csi_stop(g_csi);
		ccic_sci_stop(cam);
		spin_unlock_irqrestore(&cam->dev_lock, flags);

		/* This would be bad news - what now? */
		spin_lock_irqsave(&cam->dev_lock, flags);

		cam->state = S_IDLE;
		ccic_sci_irq_disable(cam);
		spin_unlock_irqrestore(&cam->dev_lock, flags);

		ccic_free_dma_desc(cam);
	}
}
#endif

/* -------------------------------------------------------------------- */
/*
 * Communications with the sensor.
 */

static int __ccic_cam_cmd(struct ccic_camera *cam, int cmd, void *arg)
{
	int ret = 0;
    struct stCamacqSensor_t *pstSensor;
    //pxa950_camera_dbg_in();

    if (cam->pstSensorManager == NULL)
    {
        pxa950_camera_err( "cam->pstSensorManager == NULL" );
        return -EINVAL;
    }
		
    pstSensor = cam->pstSensorManager->SetSensor( cam->pstSensorManager, cam->iSelectedSensor );

    pxa950_camera_dbg( " : cam->iSelectedSensor[ %d ], pstSensor[%x]", cam->iSelectedSensor, pstSensor );

    if( pstSensor == NULL )
    {
        pxa950_camera_err( "pstSensor == NULL" );
        return -EINVAL;
    }
    
    switch( cmd ) 
    {
        case VIDIOC_INT_INIT:
            ret = pstSensor->m_pstAPIs->IntInit( pstSensor );
            break;

        case VIDIOC_ENUM_FMT:
            ret = pstSensor->m_pstAPIs->EnumPixFmt( pstSensor, (struct v4l2_fmtdesc *)arg );
            break;

         case VIDIOC_ENUM_FRAMESIZES:
            ret = pstSensor->m_pstAPIs->EnumFrameSizes( pstSensor, (struct v4l2_frmsizeenum*)arg );
            break;

        case VIDIOC_TRY_FMT:
            ret = pstSensor->m_pstAPIs->TryFmt( pstSensor, (struct v4l2_format *)arg, NULL, NULL );
            break;

        case VIDIOC_S_FMT:
            ret = pstSensor->m_pstAPIs->SetFmt( pstSensor, (struct v4l2_format *)arg );
            break;

        case VIDIOC_S_PARM:
            ret = pstSensor->m_pstAPIs->SetParm( pstSensor, (struct v4l2_streamparm *)arg );
            break;
               
        case VIDIOC_G_PARM:
            ret = pstSensor->m_pstAPIs->GetParm( pstSensor, (struct v4l2_streamparm *)arg );
            break;
        
        case VIDIOC_S_INPUT:
            ret = pstSensor->m_pstAPIs->SetInput( pstSensor, (int*)arg );
            break;
        
        case VIDIOC_STREAMON:
            ret = pstSensor->m_pstAPIs->StreamOn( pstSensor );
            break;
        
        case VIDIOC_STREAMOFF:
            ret = pstSensor->m_pstAPIs->StreamOff( pstSensor ); 
            break;

        case VIDIOC_QUERYCTRL:
			ret = pstSensor->m_pstAPIs->QueryControl( pstSensor, (struct v4l2_queryctrl*)arg );
            break;
            
		case VIDIOC_S_CTRL:
            ret = pstSensor->m_pstAPIs->SetControl( pstSensor, (struct v4l2_control*)arg );
            break;
			
		case VIDIOC_G_CTRL:
            ret = pstSensor->m_pstAPIs->GetControl( pstSensor, (struct v4l2_control*)arg );
            break;

        case VIDIOC_S_EFX:
            ret = pstSensor->m_pstAPIs->SetEfx( pstSensor, (struct v4l2_efx*)arg );
            break;

        case VIDIOC_G_EFX:
            ret = pstSensor->m_pstAPIs->GetEfx( pstSensor, (struct v4l2_efx*)arg );
            break;

        case VIDIOC_QUERY_EFX:
            ret = pstSensor->m_pstAPIs->QueryEfx( pstSensor, (struct v4l2_query_efx*)arg );
            break;

        case VIDIOC_S_WBMODE:
            ret = pstSensor->m_pstAPIs->SetWB( pstSensor, (struct v4l2_wb_mode*)arg );
            break;

        case VIDIOC_G_WBMODE:
            ret = pstSensor->m_pstAPIs->GetWB( pstSensor, (struct v4l2_wb_mode*)arg );
            break;

        case VIDIOC_QUERY_WBMODE:
            ret = pstSensor->m_pstAPIs->QueryWB( pstSensor, (struct v4l2_query_wb_mode*)arg );
            break;

        case VIDIOC_S_JPEGCOMP:
            ret = pstSensor->m_pstAPIs->SetJpegQuality( pstSensor, (struct v4l2_jpegcompression*)arg );
            break;

        case VIDIOC_G_JPEGCOMP:
            ret = pstSensor->m_pstAPIs->GetJpegQuality( pstSensor, (struct v4l2_jpegcompression*)arg );
            break;

        case VIDIOC_S_FLASH:
            ret = pstSensor->m_pstAPIs->SetFlash( pstSensor, (struct v4l2_flash*)arg );
            break;

        default:
            pxa950_camera_dbg( " : Not supported command!!,  cmd : %d", cmd ); 
            break;
                
    }

    //pxa950_camera_dbg_out();

	return ret;
}


static int __ccic_cam_reset(struct ccic_camera *cam)
{
	int zero = 0;
	int ret = 0;
	
	//pxa950_camera_dbg_in();

	ret = __ccic_cam_cmd(cam, VIDIOC_INT_RESET, &zero);
	
	//pxa950_camera_dbg_out();
	return ret;
}

/*
 * We have found the sensor on the i2c.  Let's try to have a
 * conversation.
 */
static int __attribute__ ((unused)) ccic_cam_init(struct ccic_camera *cam)
{
	int ret = 0;
	struct v4l2_dbg_chip_ident chip = { {}, 0, 0 };
	chip.match.type = V4L2_CHIP_MATCH_I2C_ADDR;
	chip.match.addr = 0;
	pxa950_camera_dbg_in();

 #if 0

	mutex_lock(&cam->s_mutex);
	if (cam->state != S_NOTREADY)
		cam_warn(cam, "Cam init with device in funky state %d",
				cam->state);
	ret = __ccic_cam_reset(cam);
	if (ret)
		goto out;
	chip.match.type = V4L2_CHIP_MATCH_I2C_ADDR;
	ret = __ccic_cam_cmd(cam, VIDIOC_DBG_G_CHIP_IDENT, &chip);
	if (ret)
		goto out;
	cam->sensor_type = chip.ident;
	if ((cam->sensor_type != V4L2_IDENT_OV7690)
		&&(cam->sensor_type != V4L2_IDENT_OV3640)
		&&(cam->sensor_type != V4L2_IDENT_OV5642)) {
		cam_err(cam, "cam: Unsupported sensor type %d", cam->psensor->pclient->addr);
		ret = -EINVAL;
		goto out;
	}
	/* Get/set parameters? */
	ret = 0;
	cam->state = S_IDLE;
	
out:
	mutex_unlock(&cam->s_mutex);

#endif

    cam->state = S_IDLE;
	pxa950_camera_dbg_out();
	
	return ret;
}

/*
 * Configure the sensor to match the parameters we have.  Caller should
 * hold s_mutex
 */
static int __attribute__ ((unused)) ccic_cam_set_flip(struct ccic_camera *cam)
{
	struct v4l2_control ctrl;

	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = V4L2_CID_VFLIP;
	ctrl.value = flip;
	return __ccic_cam_cmd(cam, VIDIOC_S_CTRL, &ctrl);
}

static int ccic_cam_configure(struct ccic_camera *cam)
{
	struct v4l2_format fmt;
	int ret, zero = 0;

	//pxa950_camera_dbg_in();

	if (cam->state != S_IDLE)
		return -EINVAL;
	fmt.fmt.pix = cam->pix_format;

#if CONVERT_420_422
	if (V4L2_PIX_FMT_YUV420 == fmt.fmt.pix.pixelformat)
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
#endif
#ifdef JPEG_DISCARD_WORKAROUND
	if (V4L2_PIX_FMT_JPEG == fmt.fmt.pix.pixelformat)
		cam->discard = 8;
#endif
	ret = __ccic_cam_cmd(cam, VIDIOC_S_FMT, &fmt);

	//pxa950_camera_dbg_out();

	return ret;
}

/* -------------------------------------------------------------------- */
/*
 * DMA buffer management.  These functions need s_mutex held.
 */

/* FIXME: this is inefficient as hell, since dma_alloc_coherent just
 * does a get_free_pages() call, and we waste a good chunk of an orderN
 * allocation.  Should try to allocate the whole set in one chunk.
 */
#if 0
static int ccic_alloc_dma_bufs(struct ccic_camera *cam, int loadtime)
{
	int i;

    pxa950_camera_dbg("in");

	ccic_set_config_needed(cam, 1);

	/* 2) determine the DMA buf size*/
	if (loadtime)
		cam->dma_buf_size = dma_buf_size;
	else
		cam->dma_buf_size = cam->pix_format.sizeimage;
	cam->dma_buf_size = PAGE_ALIGN(cam->dma_buf_size);


	/* 3) allocate dummy buf, for one buf case*/
	cam->buf_dummy = kzalloc(sizeof(struct pxa_buf_node), GFP_KERNEL);
	if (cam->buf_dummy == NULL) {
		printk(KERN_ERR "cam: allocate buf_dummy failed!!!\n");
		return -ENOMEM;
	}

	cam->buf_dummy->order = get_order(DMA_DUMMY_SIZE);
	cam->buf_dummy->dma_bufs = (void *)__get_free_pages(GFP_KERNEL, cam->buf_dummy->order);
	cam->buf_dummy->dma_handles = __pa(cam->buf_dummy->dma_bufs);
	if (cam->buf_dummy->dma_bufs == NULL) {
		cam_warn(cam, "cam: Failed to allocate dummy DMA buffer\n");
		return -ENOMEM;
	}
	pxa950_camera_dbg( "cam: init cam dummy_node 0x%x, size %d, dma-phy 0x%x\n", (unsigned int)cam->buf_dummy, DMA_DUMMY_SIZE, cam->buf_dummy->dma_handles);


	/* 4) allocate mmap-buf, for mmap buf usage, the max buf num is MAX_DMA_BUFS*/
	if (n_dma_bufs > MAX_DMA_BUFS)
		n_dma_bufs = MAX_DMA_BUFS;
	cam->n_mapbufs = 0;
	cam->buf_mmap = kzalloc(n_dma_bufs*sizeof(struct pxa_buf_node), GFP_KERNEL);
	if (cam->buf_mmap == NULL) {

		printk(KERN_ERR "cam: allocate buf_mmap failed!!!\n");
		return -ENOMEM;
	}
	for (i = 0; i < n_dma_bufs; i++) {

		pxa950_camera_dbg("cam: init cam buf_node[%d] 0x%x, size %d\n", i, (unsigned int)&cam->buf_mmap[i], cam->dma_buf_size);
		cam->buf_mmap[i].order = get_order(cam->dma_buf_size);
		cam->buf_mmap[i].dma_bufs = (void *)__get_free_pages(GFP_KERNEL, cam->buf_mmap[i].order);
		cam->buf_mmap[i].dma_handles = __pa(cam->buf_mmap[i].dma_bufs);

		if (cam->buf_mmap[i].dma_bufs == NULL) {
			cam_warn(cam, "cam: Failed to allocate DMA buffer\n");
			break;
		}

		(cam->n_mapbufs)++;
		pxa950_camera_dbg("cam: allocate buf_mmap[%d] 0x%x, pa 0x%x, va 0x%x\n",
			i, (unsigned int)&cam->buf_mmap[i],cam->buf_mmap[i].dma_handles, (unsigned int)cam->buf_mmap[i].dma_bufs);
	}

	if (cam->n_mapbufs < n_dma_bufs) {
		for (i = 0; i < cam->n_mapbufs; i++) {
			free_pages((unsigned long)cam->buf_mmap[i].dma_bufs, cam->buf_mmap[i].order);
		}
		cam->n_mapbufs = 0;
		cam_err(cam, "cam: Insufficient DMA buffers, cannot operate\n");
		return -ENOMEM;
	}

    pxa950_camera_dbg("out");
    
	return 0;
}
#else 6.0D
static int ccic_alloc_dma_bufs(struct ccic_camera *cam, int loadtime)
{
	return 0;
	int i;

    pxa950_camera_dbg_in();
	
	/* 2) determine the DMA buf size*/
	if (loadtime)
		cam->dma_buf_size = dma_buf_size;
	else
		cam->dma_buf_size = cam->pix_format.sizeimage;
	cam->dma_buf_size = PAGE_ALIGN(cam->dma_buf_size);


	/* 3) allocate dummy buf, for one buf case*/
	cam->buf_dummy = kzalloc(sizeof(struct pxa_buf_node), GFP_KERNEL);
	if (cam->buf_dummy == NULL) {
		printk(KERN_ERR "cam: allocate buf_dummy failed!!!\n");
		return -ENOMEM;
	}

	cam->buf_dummy->order = get_order(DMA_DUMMY_SIZE);
	cam->buf_dummy->page = alloc_pages(GFP_KERNEL, cam->buf_dummy->order);
	if (cam->buf_dummy->page == NULL){
		printk(KERN_ERR "cam: allocate buf_dummy failed!!!\n");
		return -ENOMEM;
	}

	cam->buf_dummy->dma_bufs = (void *)page_address(cam->buf_dummy->page);
	cam->buf_dummy->dma_handles = __pa(cam->buf_dummy->dma_bufs);
	if (cam->buf_dummy->dma_bufs == NULL) {
		cam_warn(cam, "cam: Failed to allocate dummy DMA buffer\n");
		return -ENOMEM;
	}
	printk(KERN_DEBUG "cam: init cam dummy_node 0x%x, size %d, dma-phy 0x%x\n", (unsigned int)cam->buf_dummy, DMA_DUMMY_SIZE, cam->buf_dummy->dma_handles);


	/* 4) allocate mmap-buf, for mmap buf usage, the max buf num is MAX_DMA_BUFS*/
	if (n_dma_bufs > MAX_DMA_BUFS)
		n_dma_bufs = MAX_DMA_BUFS;
	cam->n_mapbufs = 0;
	cam->buf_mmap = kzalloc(n_dma_bufs*sizeof(struct pxa_buf_node), GFP_KERNEL);
	if (cam->buf_mmap == NULL) {

		printk(KERN_ERR "cam: allocate buf_mmap failed!!!\n");
		return -ENOMEM;
	}
	for (i = 0; i < n_dma_bufs; i++) {

		printk(KERN_DEBUG "cam: init cam buf_node[%d] 0x%x, size %d\n", i, (unsigned int)&cam->buf_mmap[i], cam->dma_buf_size);
		cam->buf_mmap[i].order = get_order(cam->dma_buf_size);
		cam->buf_mmap[i].page = alloc_pages(GFP_KERNEL, cam->buf_mmap[i].order);
		if (cam->buf_mmap[i].page == NULL){
			printk(KERN_ERR "cam: allocate buf_mmap failed!!!\n");
			return -ENOMEM;
		}

		cam->buf_mmap[i].dma_bufs = (void *)page_address(cam->buf_mmap[i].page);
		cam->buf_mmap[i].dma_handles = __pa(cam->buf_mmap[i].dma_bufs);

		if (cam->buf_mmap[i].dma_bufs == NULL) {
			cam_warn(cam, "cam: Failed to allocate DMA buffer\n");
			break;
		}

		(cam->n_mapbufs)++;
		printk(KERN_DEBUG "cam: allocate buf_mmap[%d] 0x%x, pa 0x%x, va 0x%x\n",
			i, (unsigned int)&cam->buf_mmap[i],cam->buf_mmap[i].dma_handles, (unsigned int)cam->buf_mmap[i].dma_bufs);
	}

	if (cam->n_mapbufs < n_dma_bufs) {
		for (i = 0; i < cam->n_mapbufs; i++) {
			free_pages((unsigned long)cam->buf_mmap[i].dma_bufs, cam->buf_mmap[i].order);
		}
		cam->n_mapbufs = 0;
		cam_err(cam, "cam: Insufficient DMA buffers, cannot operate\n");
		return -ENOMEM;
	}

    pxa950_camera_dbg_out();
	return 0;
}

#endif 

static void ccic_free_dma_bufs(struct ccic_camera *cam)
{
	return 0;
	int i;
    pxa950_camera_dbg("in");
    
	for (i = 0; i < cam->n_mapbufs; i++) {
		free_pages((unsigned long)cam->buf_mmap[i].dma_bufs, cam->buf_mmap[i].order);
		cam->buf_mmap[i].dma_bufs = NULL;
	}
	cam->n_mapbufs = 0;

    pxa950_camera_dbg("out");
}


/* ----------------------------------------------------------------------- */
/*
 * Here starts the V4L2 interface code.
 */

/*
 * Read an image from the device.
 */
static ssize_t ccic_deliver_buffer(struct ccic_camera *cam,
		char __user *buffer, size_t len, loff_t *pos)
{
	struct pxa_buf_node *bufnode;
	unsigned long flags;

    pxa950_camera_dbg("in");

	spin_lock_irqsave(&cam->dev_lock, flags);
	if (cam->next_buf == NULL) {
		cam_err(cam, "cam: deliver_buffer: No next buffer\n");
		spin_unlock_irqrestore(&cam->dev_lock, flags);
		return -EIO;
	}
	bufnode = cam->next_buf;
	cam->next_buf = NULL;
	cam->specframes = 0;
	spin_unlock_irqrestore(&cam->dev_lock, flags);

	if (len > cam->pix_format.sizeimage)
		len = cam->pix_format.sizeimage;
	dma_sync_single_for_device(&(cam->pdev->dev), bufnode->dma_handles, len, DMA_FROM_DEVICE);
	if (copy_to_user(buffer, bufnode->dma_bufs, len))
		return -EFAULT;
	(*pos) += len;

    pxa950_camera_dbg("out");
	return len;
}

/*
 * Get everything ready, and start grabbing frames.
 */

#define AQMEMORYDEBUG	ioremap_nocache(0x54000414,4)  /* GC outstanding request Limiter */
static unsigned int gMemDebRegValue = 9999;

unsigned int ofo_cnt;
unsigned int ofo_tm;
static int ccic_read_setup(struct ccic_camera *cam, enum ccic_state state)
{
	int ret;
	unsigned long flags;
	int idx = cam->v4ldev.minor;
    unsigned long clk;
	pxa950_camera_dbg_in( " idx[%d]", idx );
    
#if defined(_WELLER_)
    unsigned int * pMemDebReg = NULL;
	unsigned int value;
	printk (KERN_ERR "Weller: Inside ccic_read_setup\n");

    pMemDebReg = AQMEMORYDEBUG;

#if 0
    if(pMemDebReg != NULL){
        value = *pMemDebReg;
		gMemDebRegValue = value;
		printk(KERN_ERR "Weller: Before AQMEMORYDEBUG  0x%x\n", value);

		value = (value & 0xFFFFFF00) | 1;
		*pMemDebReg = value;
        value = *pMemDebReg;
		mdelay(500);
		printk(KERN_ERR "Weller: After AQMEMORYDEBUG  0x%x\n", value);
    }
#else
    if(pMemDebReg != NULL){
        value = *pMemDebReg;
        gMemDebRegValue = value;
        printk(KERN_ERR "Weller: Before AQMEMORYDEBUG  0x%x ", value);

        value = 0;
        gMemDebRegValue = value;
        *pMemDebReg = value;
        msleep(100);
        printk(KERN_ERR "Weller: Before Force to Zero  0x%x ", value);

        value = (value & 0xFFFFFF00) | 1;
        *pMemDebReg = value;
        value = *pMemDebReg;
        msleep(100);
        printk(KERN_ERR "Weller: After AQMEMORYDEBUG  0x%x ", value);
    }
#endif
#endif /* _WELLER_ */
	//20111011 Marvell's patch
	if( (176 == cam->pix_format.width) && (144 == cam->pix_format.height)) {
		printk("cam: wait stable for QCIF ============================= \n");
		msleep(20);
	}

	/*
	 * Configuration.  If we still don't have DMA buffers,
	 * make one last, desperate attempt.
	 */
	if (cam->n_mapbufs == 0)
		if (ccic_alloc_dma_bufs(cam, 0))
			return -ENOMEM;
	if (cam->buf_node == NULL) {
		printk(KERN_ERR "cam: internal buffers are not ready!\n");
		return -EPERM;
	}

	csi_mipi_config(g_csi, cam->pix_format.width, sensor_selected);
	if (ccic_needs_config(cam)) {
		ret = ccic_ctlr_configure(cam);
		ccic_set_config_needed(cam, 0);
		if (ret)
			return ret;
	}

	/*
	 * Turn it loose.
	 */
	spin_lock_irqsave(&cam->dev_lock, flags);
	ccic_reset_buffers(cam);
	ccic_sci_irq_enable(cam);
	cam->state = state;

    pxa950_camera_dbg( " : cam state [%d]", cam->state );

	csi_start(g_csi, idx);// configure enable which camera interface controller
	ccic_sci_start(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	__ccic_cam_cmd(cam, VIDIOC_STREAMON, NULL);

#ifdef _CONTROLLER_DEADLOOP_RESET_
	mod_timer(&cam->reset_timer, jiffies + MIPI_RESET_TIMEOUT);
	cam->killing_reset_timer = 0;
#endif

	pxa950_camera_dbg_out();

    ofo_tm = 0;
	ofo_cnt = 0;

	/*dump_registers(cam);*/
	/*dump_csi_registers(g_csi);*/

	return 0;
}


static ssize_t ccic_v4l_read(struct file *filp,
		char __user *buffer, size_t len, loff_t *pos)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;

    pxa950_camera_dbg_v("in");

	/*
	 * Perhaps we're in speculative read mode and already
	 * have data?
	 */
	mutex_lock(&cam->s_mutex);
	if (cam->state == S_SPECREAD) {
		if (cam->next_buf != NULL) {
			ret = ccic_deliver_buffer(cam, buffer, len, pos);
			if (ret != 0)
				goto out_unlock;
		}
	} else if (cam->state == S_FLAKED || cam->state == S_NOTREADY) {
		ret = -EIO;
		goto out_unlock;
	} else if (cam->state != S_IDLE) {
		ret = -EBUSY;
		goto out_unlock;
	}

	/*
	 * v4l2: multiple processes can open the device, but only
	 * one gets to grab data from it.
	 */
	if (cam->owner && cam->owner != filp) {
		ret = -EBUSY;
		goto out_unlock;
	}
	cam->owner = filp;

	/*
	 * Do setup if need be.
	 */
	if (cam->state != S_SPECREAD) {
		ret = ccic_read_setup(cam, S_SINGLEREAD);
		if (ret)
			goto out_unlock;
	}
	/*
	 * Wait for something to happen.  This should probably
	 * be interruptible (FIXME).
	 */
	wait_event_timeout(cam->iowait, cam->next_buf != NULL, HZ);
	if (cam->next_buf == NULL) {
		cam_err(cam, "cam: read() operation timed out\n");
		ccic_ctlr_stop_dma(cam);
		ret = -EIO;
		goto out_unlock;
	}
	/*
	 * Give them their data and we should be done.
	 */
	ret = ccic_deliver_buffer(cam, buffer, len, pos);

    pxa950_camera_dbg_v("out");
    
out_unlock:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Streaming I/O support.
 */

static unsigned int csi_reg_read(struct ccic_csi *csi, unsigned int reg);

static int ccic_vidioc_streamon(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

#if CCIC_OFO_DUMP
    g_uiOFOCount = 0;
    memset( &g_stOFOStartTime, 0x00, sizeof(struct timeval));
    memset( &g_stOFOEndTime, 0x00, sizeof(struct timeval));
#endif /* CCIC_OFO_DUMP */

	pxa950_camera_dbg_in();

    pxa950_camera_dbg("REG_CSGCR : [%x]", csi_reg_read( g_csi, REG_CSGCR ));
    pxa950_camera_dbg( " : ACCR[0x%08x]", ACCR );
    pxa950_camera_dbg( " : ACSR[0x%08x]", ACSR );

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_IDLE || cam->n_sbufs == 0)
		goto out_unlock;

	ret = ccic_read_setup(cam, S_STREAMING);
out_unlock:
	mutex_unlock(&cam->s_mutex);
out:
	pxa950_camera_dbg_out();
	return ret;
}



static int ccic_vidioc_streamoff(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;
	pxa950_camera_dbg_in();

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_STREAMING)
		goto out_unlock;

	__ccic_cam_cmd(cam, VIDIOC_STREAMOFF, NULL);
	ccic_ctlr_stop_dma(cam);

#ifdef _CONTROLLER_DEADLOOP_RESET_
	/* Announce reset timer is being killed */
	cam->killing_reset_timer = 1;
	del_timer(&cam->reset_timer);
#endif

	ret = 0;

#if defined(_WELLER_)
	if( gMemDebRegValue != 9999)
	{
		unsigned int * pMemDebReg = NULL;
		pMemDebReg = AQMEMORYDEBUG;
        *pMemDebReg = gMemDebRegValue;
	}
#endif /* _WELLER_ */

	pxa950_camera_dbg_out();

out_unlock:
	mutex_unlock(&cam->s_mutex);
out:
	return ret;
}

static unsigned long uva_to_pa(unsigned long addr, struct page **page)
{
	unsigned long ret = 0UL;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(current->mm, addr);
	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte = pte_offset_map(pmd, addr);
				if (!pte_none(*pte) && pte_present(*pte)) {
					(*page) = pte_page(*pte);
					ret = page_to_phys(*page);
					ret |= (addr & (PAGE_SIZE-1));
				}
			}
		}
	}
	return ret;
}

struct page* va_to_page(unsigned long user_addr)
{
	struct page *page = NULL;
	unsigned int vaddr = PAGE_ALIGN(user_addr);

	if (uva_to_pa(vaddr, &page) != 0)
		return page;

	return 0;
}
unsigned long va_to_pa(unsigned long user_addr, unsigned int size)
{
	unsigned long  paddr, paddr_tmp;
	unsigned long  size_tmp = 0;
	struct page *page = NULL;
	int page_num = PAGE_ALIGN(size) / PAGE_SIZE;
	unsigned int vaddr = PAGE_ALIGN(user_addr);
	int i = 0;

	if(vaddr == 0)
		return 0;

	paddr = uva_to_pa(vaddr, &page);

	for (i = 0; i < page_num; i++) {
		paddr_tmp = uva_to_pa(vaddr, &page);
		if ((paddr_tmp - paddr) != size_tmp)
			return 0;
		vaddr += PAGE_SIZE;
		size_tmp += PAGE_SIZE;
	}
	return paddr;
}

static int ccic_prepare_buffer_node(struct ccic_camera *cam,
	struct ccic_sio_buffer *sbuf, unsigned long userptr,
                unsigned int size, unsigned int index)
{
	struct pxa_cam_dma *pxa_dma;
	unsigned int vaddr = PAGE_ALIGN(userptr);

	pxa950_camera_dbg_in();
	if (vaddr != userptr) {
		printk(KERN_ERR "cam: error: the memory is not page align!\n");
		return -EPERM;
	}
	sbuf->dma_handles = va_to_pa(vaddr, size);
	if (!sbuf->dma_handles) {
		printk(KERN_ERR "cam: mem is not contiguous!\n");
		return -ENOMEM;
	}
	sbuf->page = va_to_page(vaddr);
	if (!sbuf->page) {
		printk(KERN_ERR "cam: fail to get page info!\n");
		return -EFAULT;
	}
	/* Map the PA to kernel space */
	sbuf->buffer = ioremap(sbuf->dma_handles, PAGE_ALIGN(size));
	if (NULL == sbuf->buffer) {
		/* This implies that the kernal don't have enough space to map
		 * the userptr buffer to kernel space, usually this is because
		 * the requested size is too big
		 */
		printk(KERN_ERR "cam: failed to map userptr buffer to kernel"
				", buffer size: %d\n", size);
		return -EINVAL;
	}
	memset(sbuf->buffer, 0, size);

	INIT_LIST_HEAD(&sbuf->list);
	sbuf->v4lbuf.length = PAGE_ALIGN(size);
	sbuf->mapcount = 0;

	sbuf->v4lbuf.index = index;
	sbuf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	sbuf->v4lbuf.field = V4L2_FIELD_NONE;
	sbuf->v4lbuf.memory = V4L2_MEMORY_USERPTR;
	sbuf->v4lbuf.m.offset = 2 * index * sbuf->v4lbuf.length;

	if (cam->buf_node) {
		/* init the struct which is finally used by DMA chain*/
		cam->buf_node[index].dma_bufs = sbuf->buffer;
		cam->buf_node[index].dma_handles = sbuf->dma_handles;
	} else {
		printk(KERN_ERR "cam: camera internal buf is not ready!\n");
		return -EPERM;
	}
	pxa_dma = &cam->buf_node[index].dma_desc[0];
	if (pxa_dma->sg_cpu == NULL) {
		ccic_alloc_dma_desc(cam, index);
	}
    
	pxa950_camera_dbg_out();
	return 0;
}

static void ccic_free_buffer_node(struct ccic_sio_buffer *sbuf)
{
	/*
	 * vunmap will do TLB flush for us.
	 * We map uncachable memory, so needn't cache invalid operation here.
	 */

    pxa950_camera_dbg("in");

	if(V4L2_MEMORY_USERPTR != sbuf->v4lbuf.memory)
		return;

	if (sbuf->buffer) {
		iounmap(sbuf->buffer);
		sbuf->buffer = NULL;
	}

    pxa950_camera_dbg("out");
}

static int ccic_setup_siobuf(struct ccic_camera *cam, int index)
{
	struct ccic_sio_buffer *sbuf = cam->sb_bufs + index;
    pxa950_camera_dbg("in");

	INIT_LIST_HEAD(&sbuf->list);
	sbuf->v4lbuf.length = PAGE_ALIGN(cam->pix_format.sizeimage);
	sbuf->buffer = cam->buf_mmap[index].dma_bufs;
	sbuf->dma_handles = cam->buf_mmap[index].dma_handles;
	sbuf->page = cam->buf_mmap[index].page;

	printk(KERN_DEBUG "cam: ccic_setup_siobuf: sio_buf 0x%x, dma_handle 0x%x\n", (unsigned int)sbuf, sbuf->dma_handles);
	if (sbuf->buffer == NULL)
		return -ENOMEM;
	sbuf->mapcount = 0;
	sbuf->cam = cam;

	sbuf->v4lbuf.index = index;
	sbuf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	sbuf->v4lbuf.field = V4L2_FIELD_NONE;
	sbuf->v4lbuf.memory = V4L2_MEMORY_MMAP;
	/*
	 * Offset: must be 32-bit even on a 64-bit system.  videobuf-dma-sg
	 * just uses the length times the index, but the spec warns
	 * against doing just that - vma merging problems.  So we
	 * leave a gap between each pair of buffers.
	 */
	sbuf->v4lbuf.m.offset = 2 * index * sbuf->v4lbuf.length;
	/* add the sbuf into avail_buf list, then when stremon, can chain this sbuf into HW DMA chain*/
	list_add_tail(&sbuf->list, &cam->sb_avail);
    pxa950_camera_dbg("out");
    
	return 0;
}

static int ccic_free_sio_buffers(struct ccic_camera *cam)
{
	int i;
	pxa950_camera_dbg_in();
	/*
	 * If any buffers are mapped, we cannot free them at all.
	 */
	if (cam->sb_bufs) {
		for (i = 0; i < cam->n_sbufs; i++) {
			if (cam->sb_bufs[i].mapcount > 0)
				return -EBUSY;
			ccic_free_buffer_node(&cam->sb_bufs[i]);
			//printk("cam: ccic_free_sio_buffers: freed streaming buffer in 0x%08X\n", &cam->sb_bufs[i]);
		}
	}
	/*
	 * OK, let's do it.
	 */

	cam->nbufs = 0;
	cam->n_sbufs = 0;
	if (cam->sb_bufs)
		kfree(cam->sb_bufs);
              //printk("cam: ccic_free_sio_buffers: free buffer pool at 0x%08X\n", cam->sb_bufs);
	cam->sb_bufs = NULL;
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	INIT_LIST_HEAD(&cam->dma_buf_list);

	pxa950_camera_dbg_out();
	return 0;
}

static int ccic_vidioc_reqbufs(struct file *filp, void *priv,
		struct v4l2_requestbuffers *req)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;  /* Silence warning */
	//printk("cam: ccic_vidioc_reqbufs+, filp=0x%08X, priv=0x%08X, cnt=%d\n", filp, priv, req->count);
    pxa950_camera_dbg_in();

	/* Make sure it's something we can do. we support usr-point and mmap method*/
	if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE){
		printk(KERN_ERR "cam: invalid type, only support VIDEO_CAPTURE!\n");
		return -EINVAL;
	}
	if ((req->memory != V4L2_MEMORY_MMAP) && (req->memory != V4L2_MEMORY_USERPTR)){
		printk(KERN_ERR "cam: invalid memory type, only support MMAP or USERPTR!\n");
		return -EINVAL;
	}

	cam->nbufs = 0;
	cam->owner = filp;
	cam->io_type = req->memory;

	if (req->count == 0) {
		mutex_lock(&cam->s_mutex);
		/*
		 * If they ask for zero buffers, they really want us to stop streaming
		 * (if it's happening) and free everything.  Should we check owner?
		 */
		if (cam->state == S_STREAMING)
			ccic_ctlr_stop_dma(cam);
		ret = ccic_free_sio_buffers (cam);
		kzfree(cam->buf_node);
		cam->buf_node = NULL;
		mutex_unlock(&cam->s_mutex);
		if (ret)
			printk(KERN_ERR "cam: err: user request zero buf?! err = %d!\n",ret);
		return ret;
	}

	/*
	 * Device needs to be idle and working.  We *could* try to do the
	 * right thing in S_SPECREAD by shutting things down, but it
	 * probably doesn't matter.
	 */
	if (cam->state != S_IDLE || (cam->owner && cam->owner != filp)) {
		printk(KERN_ERR "cam: already in working, or invalide user!\n");
		ret = -EBUSY;
		return ret;
	}

	if (cam->n_sbufs > 0) {

		mutex_lock(&cam->s_mutex);
		ret = ccic_free_sio_buffers(cam);
		kzfree(cam->buf_node);
		cam->buf_node = NULL;
		mutex_unlock(&cam->s_mutex);
		if (ret) {
			printk(KERN_ERR "cam: mapped buffers remaining, flush first!\n");
			return ret;
		}
	}

	cam->sb_bufs = kzalloc(req->count * sizeof(struct ccic_sio_buffer), GFP_KERNEL);
	//printk("cam: cam->sb_bufs 0x%x, request buf %d\n", (unsigned int)cam->sb_bufs, req->count);
	if (cam->sb_bufs == NULL) {
		printk(KERN_ERR "cam: failed to init sb_bufs!!!\n");
		ret = -ENOMEM;
		return ret;
	}

	if (cam->io_type == V4L2_MEMORY_USERPTR){
		cam->requestbufs = req->count;

		/* allocate buf_node, which is both for user-point and mmap method, maintain real DMA chian list of dma_buf_list*/
		if (cam->buf_node == NULL)
			cam->buf_node = kzalloc((req->count)*sizeof(struct pxa_buf_node), GFP_KERNEL);
		if (cam->buf_node == NULL) {
			printk(KERN_ERR "cam: failed to init %d of buf_node!!!\n", req->count);
			ret = -ENOMEM;
			return ret;
		}
	} else if (cam->io_type == V4L2_MEMORY_MMAP ) {

		if (req->count < min_buffers)
			req->count = min_buffers;
		/*we only allocate n_mapbufs in camera driver, do not request more than it*/
		else if (req->count > cam->n_mapbufs)
			req->count = cam->n_mapbufs;

		/* allocate buf_node, which is both for user-point and mmap method, maintain real DMA chian list of dma_buf_list*/
		if (cam->buf_node == NULL)
			cam->buf_node = kzalloc((req->count)*sizeof(struct pxa_buf_node), GFP_KERNEL);
		if (cam->buf_node == NULL) {
			printk(KERN_ERR "cam: failed to init %d of buf_node!!!\n", req->count);
			ret = -ENOMEM;
			return ret;
		}

		mutex_lock(&cam->s_mutex);

		for (cam->n_sbufs = 0; cam->n_sbufs < req->count; (cam->n_sbufs++)) {
			ret = ccic_setup_siobuf(cam, cam->n_sbufs);
			if (ret)
				break;
		}

		if (cam->n_sbufs == 0) {  /* no luck at all - ret already set */
			printk(KERN_ERR "cam: we'd allocate zero buf?!!\n");
			kfree(cam->sb_bufs);
			cam->sb_bufs = NULL;
		}
		req->count = cam->n_sbufs;  /* In case of partial success */
		cam->nbufs = cam->n_sbufs;	/* we really use *nbufs* num of bufs */
		cam->requestbufs = cam->n_sbufs;
		memcpy(cam->buf_node, cam->buf_mmap, (cam->nbufs * sizeof(struct pxa_buf_node)));

		mutex_unlock(&cam->s_mutex);

	}

    pxa950_camera_dbg_out();    
    //printk("ccic_vidioc_reqbufs-\n");
	return ret;
}


static int ccic_vidioc_querybuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;
    pxa950_camera_dbg_in();

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (buf->index >= cam->n_sbufs)
		goto out;
	*buf = cam->sb_bufs[buf->index].v4lbuf;
	ret = 0;
out:
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_qbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	struct ccic_sio_buffer *sbuf;
	struct ccic_sio_buffer *sbuf_tmp;
	bool is_in_bufq = false;
	int ret = -EINVAL;
	unsigned long flags;
	u8 *magic_bit;
    pxa950_camera_dbg_v("in");

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		printk(KERN_ERR "cam: qbuf: type != V4L2_BUF_TYPE_VIDEO_CAPTURE.\n");
		ret = -EINVAL;
		goto out;
	}
	if (cam->sb_bufs == NULL) {
		printk(KERN_ERR "cam: qbuf: need request buf!\n");
		ret = -EINVAL;
		goto out;
	}
	if (buf->index >= cam->requestbufs) {
		printk(KERN_ERR "cam: qbuf: request buf index invalid!\n");
		ret = -EINVAL;
		goto out;
	}

	sbuf = cam->sb_bufs + buf->index;
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_QUEUED) {
		/* already queued?? */
		ret = 0;
		printk(KERN_ERR "cam: qbuf: an already queued buf!\n");
		goto out;
	}
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_DONE) {
		printk(KERN_ERR "cam: qbuf: an already done buf!\n");
		/* Spec doesn't say anything, seems appropriate tho */
		ret = -EBUSY;
		goto out;
	}

	/* allocate n_sbufs bufs for userptr method*/
	if ((buf->memory == V4L2_MEMORY_USERPTR)
		&& (buf->index == cam->n_sbufs)) {
		/* For JPEG format, the size of the image is variable, but should not be too small */
		if ( (V4L2_PIX_FMT_JPEG == cam->pix_format.pixelformat)
		   &&(cam->pix_format.sizeimage < cam->pix_format.width
						* cam->pix_format.height
						/ JPEG_COMPRESS_RATIO_HIGH) ) {
			printk(KERN_ERR "cam: userspace set image size(%d bytes), "
					" might be too small for resolution(%d, %d)\n",
					cam->pix_format.sizeimage,
					cam->pix_format.width,
					cam->pix_format.height);
		}

        /* For userptr mode, the buffer is allocate by user, kernel should check its size */
        pxa950_camera_dbg( "buf->length : %d", buf->length );
        pxa950_camera_dbg( "cam->pix_format.sizeimage : %d", cam->pix_format.sizeimage );
        
		if (buf->length < cam->pix_format.sizeimage) {
			printk(KERN_ERR "cam: in userptr qubf: buffer is too small for image\n");
			ret = -EINVAL;
			goto out;
		}

		if (ccic_prepare_buffer_node(cam, sbuf, buf->m.userptr, buf->length, buf->index)){
			ret = -EINVAL;
			goto out;
		}
		cam->n_sbufs++;
		cam->nbufs = cam->n_sbufs;/*dma buf num always same with usr bufs*/
	} else {/*V4L2_MEMORY_MMAP*/
		if (buf->index >= cam->n_sbufs) {
			printk(KERN_ERR "cam: qbuf: invalid buf index %d!\n", buf->index);
			goto out;
		}
	}

	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	if (cam->requestbufs == MIN_DMA_BUFS) {
		magic_bit = (u8*)sbuf->buffer;
		memcpy(magic_bit, dma_mark, sizeof(dma_mark));
		dma_sync_single_for_device(&cam->pdev->dev,
						sbuf->dma_handles,
						sbuf->v4lbuf.length,
						DMA_TO_DEVICE);
	}

	spin_lock_irqsave(&cam->dev_lock, flags);

	list_for_each_entry(sbuf_tmp, &cam->sb_avail, list){
		if (sbuf_tmp == sbuf) {
			printk(KERN_NOTICE "cam: warn: the buf of %d has already in the queue!\n",buf->index);
			is_in_bufq = true;
			break;
		}
	}
	if (is_in_bufq == false){
		// printk(KERN_DEBUG "cam: qbuf, dma 0x%x, index %d\n", sbuf->dma_handles, buf->index);
		pxa950_camera_dbg_v("cam: qbuf, dma 0x%x, index %d\n", sbuf->dma_handles, buf->index);
		list_add_tail(&sbuf->list, &cam->sb_avail);
	}
	/*dump_avail_buf_list(cam);*/
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	ret = 0;
out:
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_v("out");
	return ret;
}

static int ccic_vidioc_dqbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	struct ccic_sio_buffer *sbuf;
	int ret = -EINVAL;
	unsigned long flags;
    pxa950_camera_dbg_v("in");

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE){
		printk(KERN_ERR "cam: err: type != V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		goto out_unlock;
	}
	if (cam->state != S_STREAMING) {
		printk(KERN_ERR "cam: err: state != S_STREAMING\n");
		goto out_unlock;
	}
	if (list_empty(&cam->sb_full) && filp->f_flags & O_NONBLOCK) {
		printk(KERN_ERR "cam: err: buf-list empty & NONBLOCK\n");
		ret = -EAGAIN;
		goto out_unlock;
	}

	while (list_empty(&cam->sb_full) && cam->state == S_STREAMING) {
		mutex_unlock(&cam->s_mutex);
		if (wait_event_interruptible(cam->iowait,
						!list_empty(&cam->sb_full))) {

			printk(KERN_ERR "cam: err: buf-list empty & STREAMING\n");
			ret = -ERESTARTSYS;
			goto out;
		}
		mutex_lock(&cam->s_mutex);
	}

	if (cam->state != S_STREAMING) {
		printk(KERN_ERR "cam: err: it is not STREAMING\n");
		ret = -EINTR;
	}
	else {
		spin_lock_irqsave(&cam->dev_lock, flags);
		/* Should probably recheck !list_empty() here */
		sbuf = list_entry(cam->sb_full.next,
				struct ccic_sio_buffer, list);
		list_del_init(&sbuf->list);

		// printk(KERN_DEBUG "cam: dqbuf dma 0x%x\n", sbuf->dma_handles);
		pxa950_camera_dbg_v("cam: dqbuf dma 0x%x\n", sbuf->dma_handles);
        
		spin_unlock_irqrestore(&cam->dev_lock, flags);

		dma_map_page(&cam->pdev->dev,
				sbuf->page,
				0,
				sbuf->v4lbuf.length,
				DMA_FROM_DEVICE);
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		*buf = sbuf->v4lbuf;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&cam->s_mutex);
out:
    pxa950_camera_dbg_v("out");
	return ret;
}

static void ccic_v4l_vm_open(struct vm_area_struct *vma)
{
	struct ccic_sio_buffer *sbuf = vma->vm_private_data;

    pxa950_camera_dbg_in("");
	/*
	 * Locking: done under mmap_sem, so we don't need to
	 * go back to the camera lock here.
	 */
	sbuf->mapcount++;

    pxa950_camera_dbg_out("");
}

static void ccic_v4l_vm_close(struct vm_area_struct *vma)
{
	struct ccic_sio_buffer *sbuf = vma->vm_private_data;

    pxa950_camera_dbg_in("");

	mutex_lock(&sbuf->cam->s_mutex);
	sbuf->mapcount--;
	/* Docs say we should stop I/O too... */
	if (sbuf->mapcount == 0)
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_MAPPED;
	mutex_unlock(&sbuf->cam->s_mutex);

    pxa950_camera_dbg_out("");
}

static struct vm_operations_struct ccic_v4l_vm_ops = {
	.open = ccic_v4l_vm_open,
	.close = ccic_v4l_vm_close
};


static int ccic_v4l_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct ccic_camera *cam = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL;
	int i;
	struct ccic_sio_buffer *sbuf = NULL;

    pxa950_camera_dbg_in();
    
	/*
	 * Find the buffer they are looking for.
	 */
	mutex_lock(&cam->s_mutex);
	for (i = 0; i < cam->n_sbufs; i++)
		if (cam->sb_bufs[i].v4lbuf.m.offset == offset) {
			sbuf = cam->sb_bufs + i;
			break;
		}
	if (sbuf == NULL)
		goto out;

	//ret = remap_vmalloc_range(vma, sbuf->buffer, 0);
	ret =  remap_pfn_range(vma, vma->vm_start,
		sbuf->dma_handles >> PAGE_SHIFT,
		(vma->vm_end - vma->vm_start),
		vma->vm_page_prot);
	if (ret)
		goto out;
	vma->vm_flags |= VM_RESERVED;   /* Don't swap */
	vma->vm_flags |= VM_DONTEXPAND;	/* Don't remap */
	vma->vm_private_data = sbuf;
	vma->vm_ops = &ccic_v4l_vm_ops;
	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_MAPPED;
	ccic_v4l_vm_open(vma);
	ret = 0;
out:
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
	return ret;
}



static int ccic_v4l_open(struct file *filp)
{
	struct ccic_camera *cam;
	int i;
	bool sensor_exist = false;

	printk("cam: ccic_v4l_open+, filp = 0x%08X\n", filp);	
    pxa950_camera_dbg_in();

	cam = ccic_find_dev(video_devdata(filp)->minor);
	if (cam == NULL) {
		printk(KERN_ERR "cam: no camera driver, failed to open!\n");
		return -ENODEV;
	}

	filp->private_data = cam;

	mutex_lock(&cam->s_mutex);
	if (cam->users == 0) {
		/* Disable 1GHz */
		dvfm_disable_op_name("988M", dvfm_dev_idx);
		dvfm_disable_op_name("156M", dvfm_dev_idx);
		dvfm_disable_op_name("156M_HF", dvfm_dev_idx);
		dvfm_disable_op_name("208M", dvfm_dev_idx);
		dvfm_disable_op_name("208M_HF", dvfm_dev_idx);
#ifdef _ARB_CHANGE_
		*pri_axi = 1;
		*pri_gcu = 2;
		*pri_ci1 = 0;
		*pri_ci2 = 0;
		printk("************************************************\n" \
			"cam: change AXI = 0x%X, GCU = 0x%X, " \
			"CI1 = 0x%X, CI2 = 0x%X\n" \
			"************************************************\n", \
			*pri_axi, *pri_gcu, *pri_ci1, *pri_ci2);
#endif

	/* FIXME make sure this is complete */
	}
	(cam->users)++;
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
	printk("cam: ccic_v4l_open-\n");    
	return 0;
}


static int ccic_v4l_release(struct file *filp)
{
	struct ccic_camera *cam = filp->private_data;
	//printk("cam: ccic_v4l_release+, filp=0x%08X\n", filp);
    pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	(cam->users)--;
	if (filp == cam->owner) {
#ifdef _CONTROLLER_DEADLOOP_RESET_
		/* Announce reset timer is being killed */
		cam->killing_reset_timer = 1;
		del_timer(&cam->reset_timer);
#endif
		ccic_ctlr_stop_dma(cam);
		ccic_free_sio_buffers(cam);
		kzfree(cam->buf_node);
		cam->buf_node = NULL;
		cam->owner = NULL;
	}
	if (cam->users == 0) {
	    //pxa950_camera_dbg( " sensor_selected[%d], cam->iSelectedSensor[%d]", sensor_selected, cam->iSelectedSensor );
        // cam->pstSensorManager->SensorPowerOff( cam->pstSensorManager, 1 ); // SUB Power-Off

        // if( cam->bIsPowerOn == true )
        // {   
            // clk off, TO DO : move to camacq power control
            // init_platform_ops = cam->platform_ops;
	        // init_platform_ops->power_set(0);
            
            cam->pstSensorManager->SensorPowerOff( cam->pstSensorManager, cam->iSelectedSensor); // MAIN Power-Off
            // cam->bIsPowerOn = false;
        // }        
        
        
#if CSI_REG_DUMP
        {
            for( g_iCsiDebugCount = 0; g_iCsiDebugCount < CSI_DEBUG_LENGTH; g_iCsiDebugCount++ )
            {
                printk( KERN_ALERT " REG_CSxINST[ 0x%08x ]\n", g_rgiREG_CSxINST[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSSCR[ 0x%08x ]\n", g_rgiREG_CSSCR[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSGCR[ 0x%08x ]\n", g_rgiREG_CSGCR[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSxCR0[ 0x%08x ]\n", g_rgiREG_CSxCR0[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSxSR[ 0x%08x ]\n", g_rgiREG_CSxSR[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSxINEN[ 0x%08x ]\n", g_rgiREG_CSxINEN[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSxTIM0[ 0x%08x ]\n", g_rgiREG_CSxTIM0[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSxTIM1[ 0x%08x ]\n", g_rgiREG_CSxTIM1[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSxGENDAT[ 0x%08x ]\n", g_rgiREG_CSxGENDAT[g_iCsiDebugCount] );
                printk( KERN_ALERT " REG_CSxPHYCAL[ 0x%08x ]\n", g_rgiREG_CSxPHYCAL[g_iCsiDebugCount] );
                printk( KERN_ALERT "\n" );  
            }

            // initial
            memset( g_rgiREG_CSxINST, 0x00, sizeof(g_rgiREG_CSxINST) );
            memset( g_rgiREG_CSSCR, 0x00, sizeof(g_rgiREG_CSSCR) );
            memset( g_rgiREG_CSGCR, 0x00, sizeof(g_rgiREG_CSGCR) );
            memset( g_rgiREG_CSxCR0, 0x00, sizeof(g_rgiREG_CSxCR0) );
            memset( g_rgiREG_CSxSR, 0x00, sizeof(g_rgiREG_CSxSR) );
            memset( g_rgiREG_CSxINEN, 0x00, sizeof(g_rgiREG_CSxINEN) );
            memset( g_rgiREG_CSxTIM0, 0x00, sizeof(g_rgiREG_CSxTIM0) );
            memset( g_rgiREG_CSxTIM1, 0x00, sizeof(g_rgiREG_CSxTIM1) );
            memset( g_rgiREG_CSxGENDAT, 0x00, sizeof(g_rgiREG_CSxGENDAT) );
            memset( g_rgiREG_CSxPHYCAL, 0x00, sizeof(g_rgiREG_CSxPHYCAL) );
            g_iCsiDebugCount = 0;
        }
#endif /* CSI_REG_DUMP */

#if CCIC_OFO_DUMP
    {
        unsigned int gap = ((g_stOFOEndTime.tv_sec*1000)+(g_stOFOEndTime.tv_usec/1000))-((g_stOFOStartTime.tv_sec*1000)+(g_stOFOStartTime.tv_usec/1000));
        printk( KERN_ALERT "======== CCIC OFO RESULT =========\n" );
        printk( KERN_ALERT " g_uiOFOCount = %d \n", g_uiOFOCount );
        printk( KERN_ALERT " StartTime Sec[%lu], Usec[%lu]\n", g_stOFOStartTime.tv_sec, g_stOFOStartTime.tv_usec );
        printk( KERN_ALERT " EndTime Sec[%lu], Usec[%lu]\n", g_stOFOEndTime.tv_sec, g_stOFOEndTime.tv_usec );
        printk( KERN_ALERT " Gap : %dms\n",  gap);
        printk( KERN_ALERT "======== CCIC OFO RESULT =========\n" );
    }
#endif /* CCIC_OFO_DUMP */
       
		if (alloc_bufs_at_read)
			ccic_free_dma_bufs(cam);
		dvfm_enable_op_name("156M", dvfm_dev_idx);
		dvfm_enable_op_name("156M_HF", dvfm_dev_idx);
		dvfm_enable_op_name("208M", dvfm_dev_idx);
		dvfm_enable_op_name("208M_HF", dvfm_dev_idx);
		dvfm_enable_op_name("988M", dvfm_dev_idx);

#ifdef _ARB_CHANGE_
		*pri_axi = 0;
		*pri_gcu = 0;
		*pri_ci1 = 0;
		*pri_ci2 = 0;
		printk(KERN_NOTICE "CI AXI Fabric RR Arbitration recovered\n");
#endif
	}
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
   // printk("cam: ccic_v4l_release-\n");
    return 0;
}

static unsigned int ccic_v4l_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct ccic_camera *cam = filp->private_data;
    pxa950_camera_dbg_v();

	poll_wait(filp, &cam->iowait, pt);
	if (!list_empty(&cam->sb_full))
		return POLLIN | POLLRDNORM;
	return 0;
}



static int ccic_vidioc_queryctrl(struct file *filp, void *priv,
		struct v4l2_queryctrl *qc)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;

    pxa950_camera_dbg_in();

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERYCTRL, qc);
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
	return ret;
}


static int ccic_vidioc_g_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;

    pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
    
	return ret;
}


static int ccic_vidioc_s_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;
	pxa950_camera_dbg_in();

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
	pxa950_camera_dbg_out();
	
	return ret;
}

static int ccic_vidioc_s_ext_ctrls(struct file *filp, void *priv, struct v4l2_ext_controls *a)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;
	pxa950_camera_dbg_in();
	
	pxa950_camera_dbg_out();
	return ret;
}

#if 0
static int ccic_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
    pxa950_camera_dbg_in();
    
	strcpy(cap->driver, "pxa950_camera");
	strcpy(cap->card, "pxa950_camera");
	cap->version = CCIC_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
    pxa950_camera_dbg_out();
	return 0;
}
#else
static int ccic_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	struct ccic_camera *cam = file->private_data;
	int ret = 0;

    pxa950_camera_dbg_in();

    strcpy(cap->card, "MG1");

#if 0 /* WINGI */
    strcpy(cap->driver, "sensor N/A");  // org
#else
    strcpy(cap->driver, "s5k4ecgx");  // temp
#endif /* WINGI */

	cap->version = CCIC_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERYCAP, cap);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg("cam: ccic_vidioc_querycap-driver = %s\n", cap->driver);

    pxa950_camera_dbg_out();
	return ret;
}
#endif


/*
 * The default format we use until somebody says otherwise.
 */
static struct v4l2_pix_format ccic_def_pix_format = {
	.width		= VGA_WIDTH,
	.height		= VGA_HEIGHT,
	.pixelformat	= V4L2_PIX_FMT_YVU420,
	.field		= V4L2_FIELD_NONE,
	.bytesperline	= VGA_WIDTH*2,
	.sizeimage	= VGA_WIDTH*VGA_HEIGHT*2,
};

static int ccic_vidioc_enum_fmt_cap(struct file *filp,
		void *priv, struct v4l2_fmtdesc *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;

    pxa950_camera_dbg_in();

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_ENUM_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
	return ret;
}


static int ccic_vidioc_try_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;
    pxa950_camera_dbg_in();
    
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_TRY_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
    
	return ret;
}

static int ccic_vidioc_s_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;
    pxa950_camera_dbg_in();
	{
	int fourcc = fmt->fmt.pix.pixelformat;
	char *ch = &fourcc;
	printk(KERN_NOTICE "cam: set format %c%c%c%c(width %d, height %d)\n", ch[0], ch[1], ch[2], ch[3], fmt->fmt.pix.width, fmt->fmt.pix.height);
	}
	/*
	 * Can't do anything if the device is not idle
	 * Also can't if there are streaming buffers in place.
	 */
	if (cam->state != S_IDLE)
		return -EBUSY;
	/*
	 * See if the formatting works in principle.
	 */
	ret = ccic_vidioc_try_fmt_cap(filp, priv, fmt);
	if (ret)
		return ret;
	/*
	 * Now we start to change things for real, so let's do it
	 * under lock.
	 */
	mutex_lock(&cam->s_mutex);
	cam->pix_format = fmt->fmt.pix;
	/*
	 * Make sure we have appropriate DMA buffers.
	 */
	ret = -ENOMEM;

	if (cam->nbufs > 0 && cam->dma_buf_size < cam->pix_format.sizeimage) {
		printk(KERN_ERR "cam: image size has exceeded the max size allocated in camera driver!!\n");
		ccic_free_dma_bufs(cam);
		goto out;
	}

	ccic_sci_stop(cam);

	/*
	 * It looks like this might work, so let's program the sensor.
	 */
	ret = ccic_cam_configure(cam);
	if (!ret) {
		ccic_ctlr_image(cam);
		ccic_set_config_needed(cam, 1);
	}
out:
	mutex_unlock(&cam->s_mutex);
    pxa950_camera_dbg_out();
	return ret;
}

/*
 * Return our stored notion of how the camera is/should be configured.
 * The V4l2 spec wants us to be smarter, and actually get this from
 * the camera (and not mess with it at open time).  Someday.
 */
static int ccic_vidioc_g_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *f)
{
	struct ccic_camera *cam = priv;

    pxa950_camera_dbg_in();
	f->fmt.pix = cam->pix_format;
    pxa950_camera_dbg_out();
	return 0;
}

/*
 * We only have one input - the sensor - so minimize the nonsense here.
 */
static int ccic_vidioc_enum_input(struct file *filp, void *priv,
		struct v4l2_input *input)
{
	if (input->index != 0 && input->index != 1)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_ALL; /* Not sure what should go here */
	strcpy(input->name, "Camera");
	return 0;
}

static int ccic_vidioc_g_input(struct file *filp, void *priv, unsigned int *i)
{
	*i = sensor_selected;
	return 0;
}

static int ccic_vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;

              printk("cam: ccic_s_input+, i=%d\n", i);
	pxa950_camera_dbg_in( " i : %d", i );

	// 0 is high sensor, 1 is low sensor
    // i = 1;
    // i = giSwitchCamera;    

    cam->iSelectedSensor = i;
    sensor_selected = i;

    pxa950_camera_dbg( "sensor_selected : %d, cam->iSelectedSensor : %d ", sensor_selected, cam->iSelectedSensor );
	
    mutex_lock(&cam->s_mutex);

    ret = cam->pstSensorManager->SensorPowerOn( cam->pstSensorManager, cam->iSelectedSensor );
    if( ret < 0 ) 
    {
        pxa950_camera_err( " : ret=%d",  ret );
#ifdef _CAM_DEBUG_V4L2OPEN_
	g_cam_cdev.s_input_fail++;
#endif
        mutex_unlock(&cam->s_mutex);
        return ret;
    }        
    
    ccic_ctlr_init(cam);
	csi_config(g_csi, sensor_selected);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_INPUT, &(cam->iSelectedSensor)); 
    if( ret < 0 ) 
    {
        pxa950_camera_err( " : ret=%d",  ret );
#ifdef _CAM_DEBUG_V4L2OPEN_
	g_cam_cdev.s_input_fail++;
#endif
        mutex_unlock(&cam->s_mutex);
        return ret;
    } 

    ret = __ccic_cam_cmd(cam, VIDIOC_INT_INIT, &(cam->iSelectedSensor)); // sensor init. temp
    if( ret < 0 ) 
    {
        pxa950_camera_err( " : ret=%d",  ret );
#ifdef _CAM_DEBUG_V4L2OPEN_
	g_cam_cdev.s_input_fail++;
#endif
        mutex_unlock(&cam->s_mutex);
        return ret;
    } 

    mutex_unlock(&cam->s_mutex);

   	pxa950_camera_dbg_out();
	//printk("cam: ccic_s_input-\n");

	return ret;
}

/* from vivi.c */
static int ccic_vidioc_s_std(struct file *filp, void *priv, v4l2_std_id *a)
{
	return 0;
}

/*
 * G/S_PARM.  Most of this is done by the sensor, but we are
 * the level which controls the number of read buffers.
 */
static int ccic_vidioc_g_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct ccic_camera *cam = priv;
	int ret;

    pxa950_camera_dbg_in();
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_s_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct ccic_camera *cam = priv;
	int ret;
    pxa950_camera_dbg_in();
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_enum_framesizes(struct file *file, void *fh,
                                         struct v4l2_frmsizeenum *fsize)
{
	struct ccic_camera *cam = fh;
	int ret = 0;
    pxa950_camera_dbg_in();

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_ENUM_FRAMESIZES, fsize);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_s_efx(struct file *file, void *priv, struct v4l2_efx *efx)
{
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();
    
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_EFX, efx);
	mutex_unlock(&cam->s_mutex);    

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_g_efx(struct file *file, void *priv, struct v4l2_efx *efx)
{
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_EFX, efx);
	mutex_unlock(&cam->s_mutex);
    	
	pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_queryefx(struct file *file, void *priv, struct v4l2_query_efx *queryefx)
{   
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERY_EFX, queryefx);
	mutex_unlock(&cam->s_mutex);
	
    pxa950_camera_dbg_out();
    return ret;
}

static int ccic_vidioc_s_wbmode(struct file *file, void *priv, struct v4l2_wb_mode *wb)
{
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_WBMODE, wb);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_g_wbmode(struct file *file, void *priv, struct v4l2_wb_mode *wb)
{
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_WBMODE, wb);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_querywbmode(struct file *file, void *priv, struct v4l2_query_wb_mode *querywb)
{
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERY_WBMODE, querywb);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_s_flash(struct file *file, void *priv, struct v4l2_flash *flash)
{
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_FLASH, flash);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_g_flash(struct file *file, void *priv, struct v4l2_flash *flash)
{
    struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_FLASH, flash);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_s_jpegcomp(struct file *file, void *priv, struct v4l2_jpegcompression *jpegcomp)
{
    struct ccic_camera *cam = priv;
    int ret;
    pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_JPEGCOMP, jpegcomp);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
    return ret;
}

static int ccic_vidioc_g_jpegcomp(struct file *file, void *priv, struct v4l2_jpegcompression *jpegcomp)
{
    struct ccic_camera *cam = priv;
    int ret;
    pxa950_camera_dbg_in();

    mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_JPEGCOMP, jpegcomp);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
    return ret;
}

/*for register access*/
static int ccic_vidioc_g_register(struct file *filp, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct ccic_camera *cam = priv;
	int ret;
    pxa950_camera_dbg_in();
	
	mutex_lock(&cam->s_mutex);
	ret =  __ccic_cam_cmd(cam, VIDIOC_DBG_G_REGISTER, reg);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static int ccic_vidioc_s_register(struct file *filp, void *priv,
                struct v4l2_dbg_register *reg)
{
	struct ccic_camera *cam = priv;
	int ret;
	pxa950_camera_dbg_in();
    
	mutex_lock(&cam->s_mutex);
	ret =  __ccic_cam_cmd(cam, VIDIOC_DBG_S_REGISTER, reg);
	mutex_unlock(&cam->s_mutex);

    pxa950_camera_dbg_out();
	return ret;
}

static void ccic_v4l_dev_release(struct video_device *vd)
{
	struct ccic_camera *cam = container_of(vd, struct ccic_camera, v4ldev);

	kfree(cam);
}


static long ccic_v4l_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct video_device *vdev = video_devdata(file);
	struct ccic_camera *cam = container_of(vdev, struct ccic_camera, v4ldev);
	int ret;

    pxa950_camera_dbg_v(" cmd : %d ", cmd );

	/* Handle some specific cmds, to avoid permission check in video_ioctl2 */
	switch (cmd) {
		case VIDIOC_ENUM_FRAMESIZES: {
			ret = ccic_vidioc_enum_framesizes(file, (void *)cam, (struct v4l2_frmsizeenum *)arg);
			return ret;
		}
		case VIDIOC_DBG_S_REGISTER: {
			ret = ccic_vidioc_s_register(file, (void *)cam, (struct v4l2_dbg_register *)arg);
			return ret;
		}
		case VIDIOC_DBG_G_REGISTER:{
			ret = ccic_vidioc_g_register(file, (void *)cam, (struct v4l2_dbg_register *)arg);
			return ret;
		}
		default:
			break;
	}
	/* Handle other ioctl cmds with standard interface */
	ret = video_ioctl2(file, cmd, arg);
	return ret;
}


/*
 * This template device holds all of those v4l2 methods; we
 * clone it for specific real devices.
 */

static const struct v4l2_file_operations ccic_v4l_fops = {
	.owner = THIS_MODULE,
	.open = ccic_v4l_open,
	.release = ccic_v4l_release,
	.read = ccic_v4l_read,
	.poll = ccic_v4l_poll,
	.mmap = ccic_v4l_mmap,
	.ioctl = ccic_v4l_ioctl,
};
/* upgrade changes from .25 to .28 */
struct v4l2_ioctl_ops ccic_ioctl_ops = {
	.vidioc_querycap        = ccic_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap    = ccic_vidioc_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap     = ccic_vidioc_try_fmt_cap,
	.vidioc_s_fmt_vid_cap       = ccic_vidioc_s_fmt_cap,
	.vidioc_g_fmt_vid_cap       = ccic_vidioc_g_fmt_cap,
	.vidioc_enum_input      = ccic_vidioc_enum_input,
	.vidioc_g_input         = ccic_vidioc_g_input,
	.vidioc_s_input         = ccic_vidioc_s_input,
	.vidioc_s_std           = ccic_vidioc_s_std,
	.vidioc_reqbufs         = ccic_vidioc_reqbufs,
	.vidioc_querybuf        = ccic_vidioc_querybuf,
	.vidioc_qbuf            = ccic_vidioc_qbuf,
	.vidioc_dqbuf           = ccic_vidioc_dqbuf,
	.vidioc_streamon        = ccic_vidioc_streamon,
	.vidioc_streamoff       = ccic_vidioc_streamoff,
	.vidioc_queryctrl       = ccic_vidioc_queryctrl,
	.vidioc_g_ctrl          = ccic_vidioc_g_ctrl,
	.vidioc_s_ctrl          = ccic_vidioc_s_ctrl,
    .vidioc_s_ext_ctrls     = ccic_vidioc_s_ext_ctrls,
	.vidioc_g_parm          = ccic_vidioc_g_parm,
	.vidioc_s_parm          = ccic_vidioc_s_parm,
	.vidioc_s_efx           = ccic_vidioc_s_efx,
    .vidioc_g_efx           = ccic_vidioc_g_efx,
    .vidioc_queryefx        = ccic_vidioc_queryefx,
    .vidioc_s_wbmode        = ccic_vidioc_s_wbmode, 
    .vidioc_g_wbmode        = ccic_vidioc_g_wbmode,
    .vidioc_querywbmode     = ccic_vidioc_querywbmode,
    .vidioc_s_jpegcomp      = ccic_vidioc_s_jpegcomp,
    .vidioc_g_jpegcomp      = ccic_vidioc_g_jpegcomp,
    .vidioc_s_flash         = ccic_vidioc_s_flash,
    .vidioc_g_flash         = ccic_vidioc_g_flash,

};

static struct video_device ccic_v4l_template = {
	.name = CAM_NAME,
	.minor = -1, 				/* Get one dynamically */
	.tvnorms = V4L2_STD_NTSC_M,
	.current_norm = V4L2_STD_NTSC_M,	/* make mplayer happy */
	.fops = &ccic_v4l_fops,
	.release = ccic_v4l_dev_release,
	.ioctl_ops	= &ccic_ioctl_ops,
};

static void __attribute__ ((unused)) dump_registers(struct ccic_camera *cam)
{
	printk(KERN_ERR "SCICR0 0x%x\n", ccic_reg_read(cam, REG_SCICR0));
	printk(KERN_ERR "SCICR1 0x%x\n", ccic_reg_read(cam, REG_SCICR1));
	printk(KERN_ERR "SCISR 0x%x\n", ccic_reg_read(cam, REG_SCISR));
	printk(KERN_ERR "SCIMASK 0x%x\n", ccic_reg_read(cam, REG_SCIMASK));
	printk(KERN_ERR "SCIFIFO 0x%x\n", ccic_reg_read(cam, REG_SCIFIFO));
	printk(KERN_ERR "SCIFIFOSR 0x%x\n", ccic_reg_read(cam, REG_SCIFIFOSR));

	printk(KERN_ERR "SCIDADDR0 0x%x\n", ccic_reg_read(cam, REG_SCIDADDR0));
	printk(KERN_ERR "SCISADDR0 0x%x\n", ccic_reg_read(cam, REG_SCISADDR0));
	printk(KERN_ERR "SCITADDR0 0x%x\n", ccic_reg_read(cam, REG_SCITADDR0));
	printk(KERN_ERR "SCIDCMD0 0x%x\n", ccic_reg_read(cam, REG_SCIDCMD0));

	printk(KERN_ERR "SCIDADDR1 0x%x\n", ccic_reg_read(cam, REG_SCIDADDR1));
	printk(KERN_ERR "SCISADDR1 0x%x\n", ccic_reg_read(cam, REG_SCISADDR1));
	printk(KERN_ERR "SCITADDR1 0x%x\n", ccic_reg_read(cam, REG_SCITADDR1));
	printk(KERN_ERR "SCIDCMD1 0x%x\n", ccic_reg_read(cam, REG_SCIDCMD1));

	printk(KERN_ERR "SCIDADDR2 0x%x\n", ccic_reg_read(cam, REG_SCIDADDR2));
	printk(KERN_ERR "SCISADDR2 0x%x\n", ccic_reg_read(cam, REG_SCISADDR2));
	printk(KERN_ERR "SCITADDR2 0x%x\n", ccic_reg_read(cam, REG_SCITADDR2));
	printk(KERN_ERR "SCIDCMD2 0x%x\n", ccic_reg_read(cam, REG_SCIDCMD2));

	printk(KERN_ERR "SCIDBR0 0x%x\n", ccic_reg_read(cam, REG_SCIDBR0));
	printk(KERN_ERR "SCIDCSR0 0x%x\n", ccic_reg_read(cam, REG_SCIDCSR0));

	printk(KERN_ERR "SCIDBR1 0x%x\n", ccic_reg_read(cam, REG_SCIDBR1));
	printk(KERN_ERR "SCIDCSR1 0x%x\n", ccic_reg_read(cam, REG_SCIDCSR1));

	printk(KERN_ERR "SCIDBR2 0x%x\n", ccic_reg_read(cam, REG_SCIDBR2));
	printk(KERN_ERR "SCIDCSR2 0x%x\n", ccic_reg_read(cam, REG_SCIDCSR2));
}

static void ccic_frame_complete(struct ccic_camera *cam, struct pxa_buf_node *frame)
{
    pxa950_camera_dbg_v();
    
	/*
	 * Basic frame housekeeping.
	 */
	if (cam->next_buf == NULL)
		cam->next_buf = frame;

	switch (cam->state) {
		/*
		 * If in single read mode, try going speculative.
		 */
		case S_SINGLEREAD:
			cam->state = S_SPECREAD;
			cam->specframes = 0;
			wake_up(&cam->iowait);
			break;

			/*
			 * If we are already doing speculative reads,
			 * and nobody is reading them, just stop.
			 */
		case S_SPECREAD:
			if (++(cam->specframes) >= cam->nbufs) {
				ccic_sci_stop(cam);
				ccic_sci_irq_disable(cam);
				cam->state = S_IDLE;
			}
			wake_up(&cam->iowait);
			break;
			 /*
			 * FIXME: if the app is not consuming the buffers,
			 * we should eventually put things on hold and
			 * restart in vidioc_dqbuf().
			 */
		case S_STREAMING:
            pxa950_camera_dbg_v("S_STREAMING");
			if (cam->requestbufs != MIN_DMA_BUFS)
				ccic_switch_desc(cam, frame);
			else
				ccic_switch_desc_onebuf(cam, frame);
			break;

		default:
			cam_err(cam, "Frame intin non-operational state\n");
			break;
	}
}

static void ccic_frame_irq(struct ccic_camera *cam, unsigned int irqs)
{
	struct pxa_buf_node *buf_node;
    pxa950_camera_dbg_v();

	if (irqs & IRQ_EOFX)
	{
#if FPS_DUMP
        do_gettimeofday( g_stFps + g_iFpsCnt );
        g_iFpsCnt++;

        if( g_iFpsCnt == FPS_MAX )
        {
            int i;
            for( i = 0; i < g_iFpsCnt; i++ )
            {
                printk( " FPS : index[%d], sec[%lu], usec[%lu] \n", i, g_stFps[i].tv_sec, g_stFps[i].tv_usec );
            }
            printk( " ===== FPS OVER FLOW COUNT [  %d  ]\n\n",  ++g_iOverflowCnt );

            g_iFpsCnt = 0;
        }
#endif /* FPS_DUMP */
#ifdef _CONTROLLER_DEADLOOP_RESET_
		if (!cam->killing_reset_timer)
			mod_timer(&cam->reset_timer, jiffies + MIPI_RESET_TIMEOUT);
#endif
		clear_bit(CF_DMA_ACTIVE, &cam->flags);
		pxa950_camera_dbg_v("cam: IRQ_EOFX\n");
		buf_node = ccic_take_frame(cam);
		ccic_frame_complete(cam, buf_node);
	}
	/*
	 * If a frame starts, note that we have DMA active.  This
	 * code assumes that we won't get multiple frame interrupts
	 * at once; may want to rethink that.
	 */
	if (irqs & IRQ_SOFX)
		set_bit(CF_DMA_ACTIVE, &cam->flags);
}

static irqreturn_t ccic_irq(int irq, void *data)
{
	struct ccic_camera *cam = data;
	unsigned int irqs = 0;
    unsigned int ofo_time;

	spin_lock(&cam->dev_lock);
	irqs = ccic_reg_read(cam, REG_SCISR);
	ccic_reg_write(cam, REG_SCISR, irqs);		/*clear irqs here*/

    // pxa950_camera_err(" REG_SCISR [ 0x%08x ]\n", irqs);
    
	if (irqs & IRQ_OFO) {
        pxa950_camera_err(" OVERFLOW !!!!!!!!!!!!!!!!!!!!!!!!\n", irqs);
#if CCIC_OFO_DUMP
        if( g_uiOFOCount < MAX_OFO_COUNT )
        {
            if( g_uiOFOCount != 0 )
            {
                g_uiOFOCount++;
                if( g_uiOFOCount == MAX_OFO_COUNT )
                {
                    do_gettimeofday( &g_stOFOEndTime );  
                }
            }
            else // first occur, g_uiCount == 0
            {
                g_uiOFOCount++;
                do_gettimeofday( &g_stOFOStartTime );
            }
        }   
#endif /* CCIC_OFO_DUMP */

#if defined(_WELLER_)
        printk(KERN_ERR "Weller: ccic over flow error!ofo_cnt = %d\n",ofo_cnt);
		ofo_cnt++;
		if (ofo_cnt == 1)
			ofo_tm = OSCR4;
		//ofo_time = OSCR4 -ofo_tm;
		if (ofo_cnt >= 50 ){
			ofo_time = OSCR4 -ofo_tm;

			printk (KERN_ERR "Weller: ofo cnt=%d, curtime = %d ticks, %d s\n", ofo_cnt, ofo_time, ofo_time/32768);	 

			ofo_cnt = 0;
			ofo_tm = 0;
		}
		//printk(KERN_ERR "cam: ccic over flow error!ofo_cnt = %d\n",ofo_cnt);
#endif /* _WELLER_ */

		csi_stop(g_csi);
		ccic_sci_stop(cam);
		csi_start(g_csi, cam->v4ldev.minor);
		ccic_sci_start(cam);
		spin_unlock(&cam->dev_lock);
		return IRQ_HANDLED;
	}

	if (irqs & IRQ_FRAME)
		ccic_frame_irq(cam, irqs);
	spin_unlock(&cam->dev_lock);
	return IRQ_HANDLED;
}

#ifdef _CONTROLLER_DEADLOOP_RESET_
static void ccic_timeout_handler(unsigned long data)
{
	static unsigned int reset_cnt = 1;
	struct ccic_camera *cam = (struct ccic_camera *)data;

	schedule_work(&cam->reset_wq);
	printk(KERN_INFO "cam: mipi_err, call reset workqueue, " \
		"INST=0x%08X----------------<%d>\n", \
		csi_reg_read(g_csi, REG_CSxINST), reset_cnt++);
	if (reset_cnt >= 100)
		printk(KERN_WARNING "cam: Too many reset workqueue triggerd,"\
			"try configure _TURN_OFF_SENSOR_ or _RESET_SENSOR_"\
			"in ccic_reset_handler()\n");

}

static void ccic_reset_handler(struct work_struct *work)
{
#if 0 /* Don't turn off sensor now to save time */
#define _TURN_OFF_SENSOR_
#endif
#ifdef _TURN_OFF_SENSOR_
/*#define _RESET_SENSOR_*/
#endif
	struct ccic_camera *cam = container_of(work, struct ccic_camera, reset_wq);
	int val = 0;
	unsigned long flags;

	if (cam->killing_reset_timer)
		return;

	/* CSI erroneous state usually caused by abnormal sensor state
	 * If the CSI reset is commited frequently, must check sensor!*/

#ifdef _TURN_OFF_SENSOR_
	/* Turn off sensor output, required by spec, but works well without it,
	 * disable it to save a lot of time in reset routing */
	__ccic_cam_cmd(cam, VIDIOC_STREAMOFF, NULL);
	msleep(2);
	/* Assume after 2 ms, sensor output is off, both data and clock lane
	 * is in LP11 state now, this is a pre-condition for CSI reset */
#endif
#ifdef _RESET_SENSOR_
	/* If sensor enters a unrecoverable error state, a sensor reset must
	 * be committed, which however, will take several millie-sec. */
	__ccic_cam_cmd(cam, VIDIOC_INT_RESET, NULL);
#endif
	spin_lock_irqsave(&cam->dev_lock, flags);
	csi_reset(g_csi);
	csi_stop(g_csi);
	ccic_sci_stop(cam);
	/* Wait until CSI is acutally turned off */
	while (val & CSxSR_CSIEN)
		val = csi_reg_read(g_csi, REG_CSxSR);

	csi_config(g_csi, sensor_selected);
	csi_mipi_config(g_csi, cam->pix_format.width, sensor_selected);
	ccic_sci_start(cam);
	csi_start(g_csi, cam->v4ldev.minor);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
#ifdef _TURN_OFF_SENSOR_
	__ccic_cam_cmd(cam, VIDIOC_STREAMON, NULL);
#endif
	/* After streamon sensor, controller should receive a valid frame
	 * before time out, otherwise handle mipi error again*/
	if (!cam->killing_reset_timer)
		mod_timer(&cam->reset_timer, jiffies + MIPI_RESET_TIMEOUT);
}
#endif

/* Camera flash sysfs interface functions */
static enum led_brightness cam_flash_get_lum(struct led_classdev *led_cdev)
{
	struct v4l2_control ctrl={V4L2_CID_FLASH_LUMINANCE, 12345};

#ifdef _CAM_DEBUG_V4L2OPEN_
	printk(KERN_ERR "cam: debug: detect_fail = %d, vdev_reg = %d, " \
		"vdev_unreg = %d, s_input_fail = %d\n", \
		g_cam_cdev.detect_fail, g_cam_cdev.vdev_reg, \
		g_cam_cdev.vdev_unreg, g_cam_cdev.s_input_fail);
#endif
	return g_cam_cdev.detect_fail;
}

static void cam_flash_set_lum(struct led_classdev *led_cdev,
	unsigned int value)
{
#if 0
	struct v4l2_control ctrl={V4L2_CID_FLASH_LUMINANCE, value};
	struct ccic_camera *cam = container_of(led_cdev, \
				  struct ccic_camera, camflash_led_cdev);

	if (value == 0) {
		/* Turn off the flash = mode1, strobe request off */
		printk(KERN_NOTICE "CamFlash: Will turn off the flash\n");
		__ccic_cam_cmd(cam, VIDIOC_S_CTRL, &ctrl);
		return;
	}

	if (value >= FLASH_ARG_LUMI_MAX) {
		printk(KERN_NOTICE "CamFlash: Invalid flash brightness\n");
		return;
	}

	printk(KERN_DEBUG "CamFlash: Flash brightness set to %d\n", value);
	__ccic_cam_cmd(cam, VIDIOC_S_CTRL, &ctrl);
#endif
	return;
}
/* -------------------------------------------------------------------------- */
static int pxa95x_camera_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = -ENOMEM;
	struct ccic_camera *cam;
    pxa950_camera_dbg_in();

	/*
	 * Start putting together one of our big camera structures.
	 */

	cam = kzalloc(sizeof(struct ccic_camera), GFP_KERNEL);
	if (cam == NULL)
		goto out;
	memset(cam, 0, sizeof(struct ccic_camera));

	mutex_init(&cam->s_mutex);
	mutex_lock(&cam->s_mutex);
	spin_lock_init(&cam->dev_lock);
	cam->state = S_NOTREADY;
	init_waitqueue_head(&cam->iowait);
	cam->pdev = pdev;

	cam->pix_format = ccic_def_pix_format;
	INIT_LIST_HEAD(&cam->dev_list);
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	INIT_LIST_HEAD(&cam->dma_buf_list);

	cam->platform_ops = pdev->dev.platform_data;
	cam->io_type = V4L2_MEMORY_MMAP;

	if (cam->platform_ops == NULL) {
		printk(KERN_ERR "cam: camera no platform data defined\n");
		return -ENODEV;
	}

	cam->irq = platform_get_irq(pdev, 0);
	if (cam->irq < 0)
		return -ENXIO;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "cam: no IO memory resource defined\n");
		return -ENODEV;
	}

	ret = -EIO;
	cam->regs = ioremap(res->start, SZ_4K);
	if (!cam->regs) {
		printk(KERN_ERR "cam: Unable to ioremap pxa95x-camera regs\n");
		goto out_free;
	}
	ret = request_irq(cam->irq, ccic_irq,
		IRQF_SHARED, CAM_NAME, cam);
	if (ret)
		goto out_iounmap;

#ifdef _CONTROLLER_DEADLOOP_RESET_
	init_timer(&cam->reset_timer);
	cam->reset_timer.function = ccic_timeout_handler;
	cam->reset_timer.data = (unsigned long)cam;
	INIT_WORK(&cam->reset_wq, ccic_reset_handler);
#endif


#ifdef _ARB_CHANGE_
	pri_axi = ioremap_nocache(ARB_CNTRL_AXI, 0x20);
	pri_ci1 = ioremap_nocache(ARB_CNTRL_CI1, 0x20);
	pri_ci2 = ioremap_nocache(ARB_CNTRL_CI2, 0x20);
	pri_gcu = ioremap_nocache(ARB_CNTRL_GCU, 0x20);
#endif
	/* Power on, clock enable before ccic controller initialize */
	/* Set platform operation pointer to the */
	/* probed sensor's platform operation pointer */
	init_platform_ops = cam->platform_ops;
	init_platform_ops->power_set(1);
	ccic_ctlr_init(cam);
	init_platform_ops->power_set(0);
	/*
	 * Set up I2C/SMBUS communications.  We have to drop the mutex here
	 * because the sensor could attach in this call chain, leading to
	 * unsightly deadlocks.
	 */
	mutex_unlock(&cam->s_mutex);  /* attach can deadlock */
	/*
	 * Get the v4l2 setup done.
	 */
	mutex_lock(&cam->s_mutex);
	cam->v4ldev = ccic_v4l_template;
	cam->v4ldev.debug = 0;
	cam->v4ldev.dev = pdev->dev;
	cam->v4ldev.index = pdev->id;

//	printk(KERN_NOTICE "cam: pxa95x_camera_probe id %d\n", cam->v4ldev.index);
	ret = video_register_device(&cam->v4ldev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto out_freeirq;
	/*
	 * If so requested, try to get our DMA buffers now.
	 */
	if (!alloc_bufs_at_read) {
		if (ccic_alloc_dma_bufs(cam, 1))
			cam_warn(cam, "Unable to alloc DMA buffers at load"
					" will try again later.");
	}
	mutex_unlock(&cam->s_mutex);
	ccic_add_dev(cam);
#ifdef _CAM_DEBUG_V4L2OPEN_
	g_cam_cdev.vdev_reg++;
#endif
	/* Setup leds-class obj, assume other members is zeroed by kzalloc */
	camflash_led_cdev.name = "spotlight";
	camflash_led_cdev.brightness_get = cam_flash_get_lum;
	camflash_led_cdev.brightness_set = cam_flash_set_lum;
	/* Camera flash dev registed in /sys/class/leds/ */
	ret = led_classdev_register(&pdev->dev, &camflash_led_cdev);
	if (ret < 0) {
		printk(KERN_ERR "CamFlash: flash led dev registration failure\n");
		led_classdev_unregister(&camflash_led_cdev);
	}

	pxa950_camera_dbg_out();
	return 0;

out_freeirq:
	free_irq(cam->irq, cam);
out_iounmap:
	iounmap(cam->regs);
out_free:
	kfree(cam);
out:
	return ret;
}


/*
 * Shut down an initialized device
 */
static void ccic_shutdown(struct ccic_camera *cam)
{
    pxa950_camera_dbg_v("");
    
/* FIXME: Make sure we take care of everything here */
	if (cam->n_sbufs > 0)
		/* What if they are still mapped?  Shouldn't be, but... */
		ccic_free_sio_buffers(cam);
	ccic_remove_dev(cam);
	ccic_ctlr_stop_dma(cam);
	ccic_free_dma_bufs(cam);
	if (cam->buf_node){
		kzfree(cam->buf_node);
		cam->buf_node = NULL;
	}
	free_irq(cam->irq, cam);
	iounmap(cam->regs);
	video_unregister_device(&cam->v4ldev);
#ifdef _CAM_DEBUG_V4L2OPEN_
	g_cam_cdev.vdev_unreg++;
#endif
	/* kfree(cam); done in v4l_release () */
}


static int pxa95x_camera_remove(struct platform_device *pdev)
{
	struct ccic_camera *cam = ccic_find_by_pdev(pdev);

    pxa950_camera_dbg_in();

	if (cam == NULL) {
		printk(KERN_WARNING "cam: remove on unknown pdev %p\n", pdev);
		return -EIO;
	}
	mutex_lock(&cam->s_mutex);
	if (cam->users > 0)
		cam_warn(cam, "cam: Removing a device with users!\n");
	ccic_shutdown(cam);

	led_classdev_unregister(&camflash_led_cdev);

    pxa950_camera_dbg_out();
	return 0;
	/* No unlock - it no longer exists */
}


static struct platform_driver pxa95x_camera_driver = {
	.driver = {
		.name = CAM_NAME
	},
	.probe 		= pxa95x_camera_probe,
	.remove 	= pxa95x_camera_remove,

};

/* -------------------------------------------------------------------------- */

/* CSI */

/* ------------------------------------------------------------------------ */
/*
 * Device register I/O
 */
static inline void csi_reg_write(struct ccic_csi *csi, unsigned int reg,
		unsigned int val)
{
	__raw_writel(val, csi->regs + reg);
}

static inline unsigned int csi_reg_read(struct ccic_csi *csi,
		unsigned int reg)
{
	return __raw_readl(csi->regs + reg);
}

typedef enum
{
	CAM_PCLK_104=0,
	CAM_PCLK_156,
	CAM_PCLK_208,
	CAM_PCLK_78,
}CAM_PCLK_t;

#define BBU_ACSR_ALUF_MASK 0x00600000L
#define BBU_ACSR_ALUF_OFFSET 21
static void csi_mipi_config(struct ccic_csi *csi, unsigned int width, int iresType)
{
	unsigned int val = 0;
	unsigned int clk;

    pxa950_camera_dbg_in();
    pxa950_camera_dbg("iresType : %d", iresType);

	clk = (ACSR&BBU_ACSR_ALUF_MASK)>>BBU_ACSR_ALUF_OFFSET;
	val = csi_reg_read(csi, REG_CSGCR);
	val &= 0xfffff000;/*bit 11:0 clk divider*/

    pxa950_camera_dbg( " width : %d, val : %d, clk : %d", width, val, clk );

    pxa950_camera_dbg( " : ACCR[0x%08x]", ACCR );
    pxa950_camera_dbg( " : ACSR[0x%08x]", ACSR );


	switch(clk)
	{
		case CAM_PCLK_104:
			/* divide clock by 4. 26Mhz.*/
			val |= CAM_CSI_CLK_DIV(0x3);
			break;

		case CAM_PCLK_156:
			/* divide clock by 6. 26Mhz.*/
			val |= CAM_CSI_CLK_DIV(0x5);
			break;

		case CAM_PCLK_208:
			/* divide clock by 8. 26Mhz.*/
			val |= CAM_CSI_CLK_DIV(0x3);/* in test result, the devider should be 3, no 7*/
			break;

		default:
			/* divide clock by 8. 26Mhz.*/
			val |= CAM_CSI_CLK_DIV(0x7);
			break;
	}

    pxa950_camera_dbg( "val [%x]", val );   
	csi_reg_write(csi, REG_CSGCR, val);
	pxa950_camera_dbg("REG_CSGCR : [%x]", csi_reg_read( csi, REG_CSGCR ));

	switch (width) {
		case 1920:
			/* for 1080p(1920x1080) mipi timing, it is fatster*/
			val = CAM_CSI_CONT_HS_Rx_TO(0xffff) |
				CAM_CSI_CONT_HSTSETTLE(0x0f);
			csi_reg_write(csi, REG_CSxTIM1, val);
			/* 2 lanes*/
			val = CAM_CSI_CONT_NOL(0x01) |
				  CAM_CSI_CONT_VC0_CFG(0x0) |
				  CAM_CSI_CONT_VC1_CFG(0x1);
			csi_reg_write(csi, REG_CSxCR0, val);
			break;

        default:
        {
            if( iresType == 0 ) // 0 is main sensor
            {
                val =   CAM_CSI_CONT_HS_Rx_TO(0xffff) |
                CAM_CSI_CONT_HSTSETTLE(0x0e); /* for isx006 & s5k4ecgx 2-lane */
                // CAM_CSI_CONT_HSTSETTLE(0x1a); /* for s5k6aafx MIPI timming */
            }
            else if( iresType == 1 ) // 1 is sub sensor
            {
                val =  CAM_CSI_CONT_HS_Rx_TO(0xffff) |
                CAM_CSI_CONT_HSTSETTLE(0x0e);  /* for s5k6aafx MIPI timming */
                // CAM_CSI_CONT_HSTSETTLE(0x40); /* for 0v5642 */
            }
            csi_reg_write(csi, REG_CSxTIM1, val);

             /* set lane */
            if( iresType == 0 ) // 0 is main sensor
            {
#if defined(CONFIG_CAMERA_S5K4ECGX) || defined(CONFIG_CAMERA_ISX012)
                pxa950_camera_dbg("2LANE ===================");
                /*  2 lanes */ 
                val = CAM_CSI_CONT_NOL(0x01) |	/* 2 lanes */
                CAM_CSI_CONT_VC0_CFG(0x0) |
                CAM_CSI_CONT_VC1_CFG(0x1);
#else
                /* 1 lane */
                val = CAM_CSI_CONT_NOL(0x00) |
                CAM_CSI_CONT_VC0_CFG(0x0) |
                CAM_CSI_CONT_VC1_CFG(0x1) |
                CAM_CSI_CONT_VC2_CFG(0x2) |
                CAM_CSI_CONT_VC3_CFG(0x3);
#endif          
            }
            else if( iresType == 1 ) // 1 is sub sensor
            {
                /* 1 lane */
                val = CAM_CSI_CONT_NOL(0x00) |
                CAM_CSI_CONT_VC0_CFG(0x0) |
                CAM_CSI_CONT_VC1_CFG(0x1) |
                CAM_CSI_CONT_VC2_CFG(0x2) |
                CAM_CSI_CONT_VC3_CFG(0x3);
            }
            csi_reg_write(csi, REG_CSxCR0, val); 
        }
        break;
	}
    
#if 0
		default:
			/* for all the other resolution */
			val = CAM_CSI_CONT_HS_Rx_TO(0xffff) |
				CAM_CSI_CONT_HSTSETTLE(0x48);
			csi_reg_write(csi, REG_CSxTIM1, val);
			/* 1 lane */
			val = CAM_CSI_CONT_NOL(0x00) |
				 CAM_CSI_CONT_VC0_CFG(0x0) |
				 CAM_CSI_CONT_VC1_CFG(0x1) |
				 CAM_CSI_CONT_VC2_CFG(0x2) |
				 CAM_CSI_CONT_VC3_CFG(0x3);
			csi_reg_write(csi, REG_CSxCR0, val);
			break;
	}
#endif

    pxa950_camera_dbg("REG_CSGCR : [%x]", csi_reg_read( csi, REG_CSGCR ));
}

void csi_config(struct ccic_csi *csi, int iresType)
{
	unsigned int val = 0;
	unsigned int clk = 0;
	unsigned int csi_irq_val = 0;

#ifdef CONFIG_PXA95x
        unsigned int *calibration_p;
#endif

	pxa950_camera_dbg_in();

	/* Before operating CSI, make sure CSI is NOT in the reset state */
	val = CAM_CSI_CONT_MIPI_RESET;
	csi_reg_write(csi, REG_CSxPHYCAL, val);

	/* using default sys video clock */
	clk = (ACSR&BBU_ACSR_ALUF_MASK)>>BBU_ACSR_ALUF_OFFSET;
	pxa950_camera_dbg( " : clk [%d]", clk );

    pxa950_camera_dbg( " : ACCR[0x%08x]", ACCR );
    pxa950_camera_dbg( " : ACSR[0x%08x]", ACSR );

	switch(clk)
	{
		case CAM_PCLK_104:
			/* divide clock by 4. 26Mhz.*/
			val = CAM_CSI_CLK_DIV(0x3) | CAM_CSI_CLK_GCLK_EN;
			break;

		case CAM_PCLK_156:
			/* divide clock by 6. 26Mhz.*/
			val = CAM_CSI_CLK_DIV(0x5) | CAM_CSI_CLK_GCLK_EN;
			break;

		case CAM_PCLK_208:
			/* divide clock by 8. 26Mhz.*/
			val = CAM_CSI_CLK_DIV(0x3) | CAM_CSI_CLK_GCLK_EN;/* in test result, the devider should be 3, no 7*/
			break;

		default:
			/* divide clock by 8. 26Mhz.*/
			val = CAM_CSI_CLK_DIV(0x7) | CAM_CSI_CLK_GCLK_EN;
			break;
	}
    pxa950_camera_dbg( "val [%x]", val );   
	csi_reg_write(csi, REG_CSGCR, val);

    pxa950_camera_dbg("REG_CSGCR : [%x]", csi_reg_read( csi, REG_CSGCR ));

	/* default: csi controller channel 0 data route to camera interface 0, channel 1 data route to camera interface 1. */
	val = 0x01;
	csi_reg_write(csi, REG_CSSCR, val);

	val = CAM_CSI_CONT_CLTERMEN(0x00) |
		  CAM_CSI_CONT_CLSETTLE(0x0C) |
		  CAM_CSI_CONT_CLMISS(0x00)   |
		  // CAM_CSI_CONT_HSTERMEN(0x08);
		  CAM_CSI_CONT_HSTERMEN(0x04);
	csi_reg_write(csi, REG_CSxTIM0, val);

	/* set time out before declarating an error to maximum */

#if 0
#if defined(CONFIG_VIDEO_OV7690_BRIDGE)
	val =  CAM_CSI_CONT_HS_Rx_TO(0xffff) |
		  /* CAM_CSI_CONT_HSTSETTLE(0x0e); *//* for ov3640 MIPI timming */
		  CAM_CSI_CONT_HSTSETTLE(0x40);/* for ov5642 MIPI timing, 0x40 is proper for OV7690 bridged via OV5642*/
#else

	val =  CAM_CSI_CONT_HS_Rx_TO(0xffff) |
		  /* CAM_CSI_CONT_HSTSETTLE(0x0e); *//* for ov3640 MIPI timming */
		  CAM_CSI_CONT_HSTSETTLE(0x48); /* for ov5642 MIPI timing, both work on MG1-JIL and PV2*/
#endif
	csi_reg_write(csi, REG_CSxTIM1, val);
#endif

    if( iresType == 0 ) // 0 is main sensor
    {
        val =   CAM_CSI_CONT_HS_Rx_TO(0xffff) |
                CAM_CSI_CONT_HSTSETTLE(0x0e); /* for isx006 & s5k4ecgx 2-lane */
                // CAM_CSI_CONT_HSTSETTLE(0x1a); /* for s5k6aafx MIPI timming */
    }
    else if( iresType == 1 ) // 1 is sub sensor
    {
        val =  CAM_CSI_CONT_HS_Rx_TO(0xffff) |
    		   CAM_CSI_CONT_HSTSETTLE(0x0e);  /* for s5k6aafx MIPI timming */
                // CAM_CSI_CONT_HSTSETTLE(0x40); /* for 0v5642 */
    }
	csi_reg_write(csi, REG_CSxTIM1, val);

    /* set lane */
    if( iresType == 0 ) // 0 is main sensor
    {
#if defined(CONFIG_CAMERA_S5K4ECGX) || defined(CONFIG_CAMERA_ISX012)
        pxa950_camera_dbg("2Lane ===================");
        /*  2 lanes */ 
        val = CAM_CSI_CONT_NOL(0x01) |	/* 2 lanes */
        CAM_CSI_CONT_VC0_CFG(0x0) |
        CAM_CSI_CONT_VC1_CFG(0x1);
#else
        /* 1 lane */
        val = CAM_CSI_CONT_NOL(0x00) |
        CAM_CSI_CONT_VC0_CFG(0x0) |
        CAM_CSI_CONT_VC1_CFG(0x1) |
        CAM_CSI_CONT_VC2_CFG(0x2) |
        CAM_CSI_CONT_VC3_CFG(0x3);
#endif         
    }
    else if( iresType == 1 ) // 1 is sub sensor
    {
       /* 1 lane */
    	val = CAM_CSI_CONT_NOL(0x00) |
    		  CAM_CSI_CONT_VC0_CFG(0x0) |
    		  CAM_CSI_CONT_VC1_CFG(0x1) |
    		  CAM_CSI_CONT_VC2_CFG(0x2) |
        	  CAM_CSI_CONT_VC3_CFG(0x3);
    }
	csi_reg_write(csi, REG_CSxCR0, val); 


//#if 1 /* Enable CSI IRQ */
#if 0 /* Enable CSI IRQ */ //20111011 Marvell's patch
	csi_irq_val =  CAM_CSI_CONT_CSI_EN_INT |        /* ENABLE Interrupt Enable*/
					 CAM_CSI_CONT_SoF_INT_EN |         /* Start of Frame Interrupt Enable*/
					 CAM_CSI_CONT_EoF_INT_EN |         /* End of Frame Interrupt Enable*/
					 CAM_CSI_CONT_OFLOW_INT_EN |     /* Overflow Interrupt Enable*/
					 CAM_CSI_CONT_PHY_ERR_EN |          /* D-PHY-related Error Interrupt Enable*/
					 CAM_CSI_CONT_GEN_PACK_INT_EN |     /* Generic Packet Interrupt Enable*/
					 CAM_CSI_CONT_TIMEOUT_EN |         /* Timeout Interrupt Enable*/
					 CAM_CSI_CONT_PROT_EN;            /* Protection Interupt Enable*/

	csi_reg_write( csi, REG_CSxINEN, csi_irq_val );
#endif

	val = 0;
#ifdef CONFIG_PXA95x
        calibration_p = ioremap_nocache(0x42404078, 4);
        if(calibration_p != NULL){
                val = *calibration_p | (5<<8);
                val &= ~(1<<9);
                *calibration_p = val;
                iounmap(calibration_p);
        }
	val = CAM_CSI_CONT_MIPI_RESET;
#endif
	val |= CAM_CSI_CONT_MIPI_REN_BYPASS(0x0) |
		CAM_CSI_CONT_MIPI_BG_VREF_EN;
	csi_reg_write(csi, REG_CSxPHYCAL, val);

    pxa950_camera_dbg_out();
}

static void __attribute__ ((unused)) dump_csi_registers(struct ccic_csi *csi)
{
	printk(KERN_ERR "CSSCR  0x%x\n", csi_reg_read(csi, REG_CSSCR));
	printk(KERN_ERR "CSGCR  0x%x\n", csi_reg_read(csi, REG_CSGCR));
	printk(KERN_ERR "CSxCR0 0x%x\n", csi_reg_read(csi, REG_CSxCR0));
	printk(KERN_ERR "CSxSR  0x%x\n", csi_reg_read(csi, REG_CSxSR));
	printk(KERN_ERR "CSxINEN 0x%x\n", csi_reg_read(csi, REG_CSxINEN));
	printk(KERN_ERR "CSxINST 0x%x\n", csi_reg_read(csi, REG_CSxINST));

	printk(KERN_ERR "CSxTIM0 0x%x\n", csi_reg_read(csi, REG_CSxTIM0));
	printk(KERN_ERR "CSxTIM1 0x%x\n", csi_reg_read(csi, REG_CSxTIM1));
	printk(KERN_ERR "CSxGENDAT 0x%x\n", csi_reg_read(csi, REG_CSxGENDAT));
	printk(KERN_ERR "CSxPHYCAL 0x%x\n", csi_reg_read(csi, REG_CSxPHYCAL));

}

static void csi_start(struct ccic_csi *csi, int camif)
{
	unsigned int val = 0;
    pxa950_camera_dbg("camif : %d", camif);
	switch(camif)// we assume sensor send all its data with packet 00, we have config packet 00 goto channel 0 in csi_config()
	{
        case CCIC_1:// channel 0 data goto camera interface 1
			if(cpu_is_pxa968())
				val = 0x01;
			else
				val = 0x10;

			csi_reg_write(csi, REG_CSSCR, val);
            break;

        case CCIC_0:// channel 1 data goto camera interface 0
        default:
			if(cpu_is_pxa968())
				val = 0x10;
			else
				val = 0x1;
			csi_reg_write(csi, REG_CSSCR, val);
			break;
	}
	val = csi_reg_read(csi, REG_CSxCR0);
	val |= CSxCR0_CSIEN;
	csi_reg_write(csi, REG_CSxCR0, val);
	if(cpu_is_pxa968())
	{
		val = 0x0;
		csi_reg_write(csi,REG_CSxDTINTLV,val);
	}
}

static void csi_stop(struct ccic_csi *csi)
{
	unsigned int val = csi_reg_read(csi, REG_CSxCR0);

    pxa950_camera_dbg_v("");

	val &= ~CSxCR0_CSIEN;
	csi_reg_write(csi, REG_CSxCR0, val);
}

/* csi_reset put CSI into reset state, must call csi_config
 * or manually set CSxPHYCAL[CAM_CSI_CONT_MIPI_RESET] later */
static void csi_reset(struct ccic_csi *csi)
{
	int val;
	/* Clear CSxPHYCAL[CAM_CSI_CONT_MIPI_RESET] to put CSI to reset state*/
	val = csi_reg_read(csi, REG_CSxPHYCAL);
	csi_reg_write(g_csi, REG_CSxPHYCAL, val&(~CAM_CSI_CONT_MIPI_RESET));
}

static irqreturn_t csi_irq(int irq, void *data)
{
	struct ccic_csi *csi = data;
	unsigned int irqs;

	spin_lock(&csi->dev_lock);
	irqs = csi_reg_read(csi, REG_CSxINST);
	csi_reg_write(csi, REG_CSxINST, irqs);
    
    // pxa950_camera_dbg_v(" REG_CSxINST[ 0x%08x ]", irqs );
    // pxa950_camera_err(" REG_CSxINST [ 0x%08x ]\n", irqs);

#if CSI_REG_DUMP // csi regs
    g_rgiREG_CSxINST[g_iCsiDebugCount] = irqs;
	g_rgiREG_CSSCR[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSSCR );
    g_rgiREG_CSGCR[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSGCR );
    g_rgiREG_CSxCR0[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSxCR0 );
    g_rgiREG_CSxSR[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSxSR );
    g_rgiREG_CSxINEN[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSxINEN );
    g_rgiREG_CSxTIM0[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSxTIM0 );
    g_rgiREG_CSxTIM1[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSxTIM1 );
    g_rgiREG_CSxGENDAT[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSxGENDAT );
    g_rgiREG_CSxPHYCAL[g_iCsiDebugCount] = csi_reg_read( csi, REG_CSxPHYCAL );
    g_iCsiDebugCount++;
    if( g_iCsiDebugCount == CSI_DEBUG_LENGTH )
        g_iCsiDebugCount = 0;
#endif /* CSI_REG_DUMP */

	spin_unlock(&csi->dev_lock);
	return IRQ_HANDLED;
}


static int pxa95x_csi_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = -ENOMEM;
	struct ccic_csi *csi;

    pxa950_camera_dbg_in("");

	csi = kzalloc(sizeof(struct ccic_csi), GFP_KERNEL);
	if (csi == NULL)
		goto out;

	g_csi = csi;

	csi->irq = platform_get_irq(pdev, 0);
	if (csi->irq < 0) {
		ret = -ENXIO;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "cam: no IO memory resource defined\n");
		ret = -ENODEV;
		goto out;
	}

	ret = -EIO;
	csi->regs = ioremap(res->start, SZ_4K);
	if (!csi->regs) {
		printk(KERN_ERR "cam: Unable to ioremap pxa95x-camera regs\n");
		goto out;
	}

	if (csi->irq > 0) {
		ret = request_irq(csi->irq, csi_irq,
			IRQF_SHARED, CSI_NAME, csi);
		if (ret)
			goto out;
	}

	spin_lock_init(&csi->dev_lock);

	/* After CSI configuring the sensor, disable the clock to save power */
	/* Meanwhile, the initial platform operation pointer should never be */
	/* accessed later! */
	if (init_platform_ops == NULL) {
		printk(KERN_ERR "cam: No defined camera, stop CSI probe\n");
		ret = -ENODEV;
		goto out;
	}
	init_platform_ops->power_set(1);
		csi_config(csi, 1);
	init_platform_ops->power_set(0);
	//init_platform_ops = NULL;
	ret = 0;

    pxa950_camera_dbg_out("");
	return 0;

 out:

	if (csi) {
		if (csi->irq > 0)
			free_irq(csi->irq, csi);

		if (csi->regs)
			iounmap(csi->regs);

		kfree(csi);
	}

	return ret;
}

static int pxa95x_csi_remove(struct platform_device *pdev)
{
	struct ccic_csi *csi = g_csi;

    pxa950_camera_dbg_v("");

	if (csi) {
		if (csi->irq > 0)
			free_irq(csi->irq, csi);

		if (csi->regs)
			iounmap(csi->regs);

		kfree(csi);
	}

	return 0;
}


static struct platform_driver pxa95x_csi_driver = {
	.driver = {
		.name = CSI_NAME
	},
	.probe 		= pxa95x_csi_probe,
	.remove 	= pxa95x_csi_remove,

};

static int __devinit pxa95x_camera_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&pxa95x_camera_driver);
	ret |= platform_driver_register(&pxa95x_csi_driver);
#ifdef CONFIG_PXA95x_DVFM
	dvfm_register("PXA95X-Camera", &dvfm_dev_idx);
#endif
	return ret;
}

static void __exit pxa95x_camera_exit(void)
{
	platform_driver_unregister(&pxa95x_camera_driver);
	platform_driver_unregister(&pxa95x_csi_driver);
#ifdef CONFIG_PXA95x_DVFM
	dvfm_unregister("PXA95X-Camera", &dvfm_dev_idx);
#endif
}

module_init(pxa95x_camera_init);
module_exit(pxa95x_camera_exit);

