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
	 * Subsystem structures.
	 */
	int irq;
	struct platform_device *pdev;
	struct video_device v4ldev;
	struct i2c_adapter i2c_adapter;
	struct i2c_client *sensor;
	struct i2c_client *sensors[SENSOR_MAX];

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
	u32 sensor_type;		/* Currently ov5642 only */
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

	/* Flash sysfs interface, will invoke sensor function thru __ccic_cam_cmd */
	struct led_classdev camflash_led_cdev;
	struct timed_output_dev camflash_timed_dev;
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
static void csi_config(struct ccic_csi *csi);

/*
 * Start over with DMA buffers - dev_lock needed.
 */
static void ccic_reset_buffers(struct ccic_camera *cam)
{
	cam->next_buf = NULL;
	cam->specframes = 0;
}

static inline int ccic_needs_config(struct ccic_camera *cam)
{
	return test_bit(CF_CONFIG_NEEDED, &cam->flags);
}

static void ccic_set_config_needed(struct ccic_camera *cam, int needed)
{
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
	mutex_lock(&ccic_dev_list_lock);
	list_add_tail(&cam->dev_list, &ccic_dev_list);
	mutex_unlock(&ccic_dev_list_lock);
}

static void ccic_remove_dev(struct ccic_camera *cam)
{
	mutex_lock(&ccic_dev_list_lock);
	list_del(&cam->dev_list);
	mutex_unlock(&ccic_dev_list_lock);
}

static struct ccic_camera *ccic_find_dev(int minor)
{
	struct ccic_camera *cam;

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
	__raw_writel(val, cam->regs + reg);
}

static inline unsigned int ccic_reg_read(struct ccic_camera *cam,
		unsigned int reg)
{
	return __raw_readl(cam->regs + reg);
}


static inline void ccic_reg_write_mask(struct ccic_camera *cam, unsigned int reg,
		unsigned int val, unsigned int mask)
{
	unsigned int v = ccic_reg_read(cam, reg);

	v = (v & ~mask) | (val & mask);
	ccic_reg_write(cam, reg, v);
}

static inline void ccic_reg_clear_bit(struct ccic_camera *cam,
		unsigned int reg, unsigned int val)
{
	ccic_reg_write_mask(cam, reg, 0, val);
}

static inline void ccic_reg_set_bit(struct ccic_camera *cam,
		unsigned int reg, unsigned int val)
{
	ccic_reg_write_mask(cam, reg, val, val);
}

/*provided for sensor calling to enable clock at begining of probe*/
void ccic_set_clock(unsigned int reg, unsigned int val)
{
	struct ccic_camera *cam;
	cam = ccic_find_dev(0);
	ccic_reg_write(cam, reg, val);
}

unsigned int ccic_get_clock(unsigned int reg)
{
        struct ccic_camera *cam;
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
static void csi_mipi_config(struct ccic_csi *csi, unsigned int width);
static void __attribute__ ((unused)) dump_csi_registers(struct ccic_csi *csi);
static void __attribute__ ((unused)) dump_registers(struct ccic_camera *cam);
static void __attribute__ ((unused)) dump_dma_desc(struct ccic_camera *cam);
static void __attribute__ ((unused))dump_dma_buf_list(struct ccic_camera *cam);
static void __attribute__ ((unused))dump_avail_buf_list(struct ccic_camera *cam);
static void __attribute__ ((unused))dump_dummy_buf_desc(struct ccic_camera *cam);

int ccic_sensor_attach(struct i2c_client *client)
{
	struct ccic_camera *cam;
	int ret;
	int i;

	for (i = 0; i < CCIC_NUM; i++) {
		struct v4l2_dbg_chip_ident chip = { {}, 0, 0 };
		chip.match.type = V4L2_CHIP_MATCH_I2C_ADDR;
		chip.match.addr = 0;

		cam = ccic_find_dev(i);

		if (cam == NULL) {
			printk(KERN_ERR "cam: didn't find camera device!\n");
			return -ENODEV;
		}
		/*
		* Don't talk to chips we don't recognize.
		*/

		mutex_lock(&cam->s_mutex);
		cam->sensor = client;
		ret = __ccic_cam_reset(cam);
		if (ret)
			goto out;
		chip.match.type = V4L2_CHIP_MATCH_I2C_ADDR;
		chip.match.addr = client->addr;
		ret = __ccic_cam_cmd(cam, VIDIOC_DBG_G_CHIP_IDENT, &chip);
		if (ret)
			goto out;
		cam->sensor_type = chip.ident;

		if ((cam->sensor_type == V4L2_IDENT_OV7690)
			|| (cam->sensor_type == V4L2_IDENT_OV7670)) {
			cam->sensors[SENSOR_LOW] = client;
			detected_low = 1;
		} else if ((cam->sensor_type == V4L2_IDENT_OV3640)
		|| (cam->sensor_type == V4L2_IDENT_OV5642)) {
			cam->sensors[SENSOR_HIGH] = client;
			detected_high = 1;
		} else {
			cam_err(cam, "cam: Unsupported sensor type %d\n", cam->sensor_type);
			ret = -EINVAL;
			goto out;
		}
		/* Get/set parameters? */
		ret = 0;
		cam->state = S_IDLE;
out:
		cam->sensor = NULL;
		mutex_unlock(&cam->s_mutex);

	}
	return ret;
}

int ccic_sensor_detach(struct i2c_client *client)
{
	struct ccic_camera *cam;
	int i;
	for (i = 0; i < CCIC_NUM; i++)
	{
		cam = ccic_find_dev(i);
		if (cam == NULL) {
			printk(KERN_ERR "cam: faild to find cam in detach!\n");
			continue;
		}
		if (cam->sensor == client) {
			ccic_ctlr_stop_dma(cam);
			cam_err(cam, "lost the sensor!\n");
			cam->sensor = NULL;  /* Bummer, no camera */
			cam->state = S_NOTREADY;
		}
	}
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
			printk(KERN_DEBUG "cam: drop a frame!\n");
		}
	} else { /* handle one buf case */
		if (!list_empty(&cam->dma_buf_list)) {
			frame = list_entry(cam->dma_buf_list.next, struct pxa_buf_node, list);
			printk(KERN_DEBUG "cam: ccic_take_frame dma 0x%x\n", frame->dma_handles);
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
	printk(KERN_DEBUG "cam: append new dma 0x%x to 0x%x\n",pxa_dma2->sg_dma, pxa_dma1->sg_dma);
}
static void ccic_switch_desc_onebuf(struct ccic_camera *cam, struct pxa_buf_node *frame)
{
	struct ccic_sio_buffer *sbuf = NULL;
	unsigned long flags;
	cam->next_buf = frame;

	spin_lock_irqsave(&cam->dev_lock, flags);

	/*if the buf has been filled, make sure dma is not polluting it, so we skip some frames, or submit it*/
	if (frame != NULL) {
		/*skip some frames, and loop back dummy, make sure dma will not point to this buf*/
		if (cam->nskipbufs) {
			cam->nskipbufs--;
			ccic_append_dma_desc(cam, cam->buf_dummy, cam->buf_dummy);
			printk(KERN_DEBUG "cam: drop a frame to make sure dma is not working on it.\n");

			spin_unlock_irqrestore(&cam->dev_lock, flags);
			return;
		} else {
		/*after we skip some frames, submit to usr this buf*/

			printk(KERN_DEBUG "cam: a valide frame!\n");
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

			printk(KERN_DEBUG "cam: append buf node 0x%x(dma 0x%x) to dummy.\n",
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
	spin_lock_irqsave(&cam->dev_lock, flags);

	/*step 1: indicate upper layer a new frame buf of sbuf-node*/
	if (frame != NULL)
	{
		list_for_each_entry(sbuf, &cam->sb_avail, list) {
			if (frame->dma_handles == sbuf->dma_handles) {
				dma_unmap_page(&cam->pdev->dev,
						frame->dma_handles,
						cam->channel_size[0],
						DMA_FROM_DEVICE);

				if ((cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG)
					&& ((((char *)frame->dma_bufs)[0] != 0xff)
					|| (((char *)frame->dma_bufs)[1] != 0xd8))) {
						printk(KERN_ERR "cam: JPEG ERROR !!! dropped this frame.\n");
						list_move_tail(&sbuf->list, &cam->sb_avail);
						list_del_init(&frame->list);
						break;		//drop current JPEG frame because of wrong header
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
				printk(KERN_DEBUG "cam: the buf_node of 0x%x has exist in dma_buf_list, ignore it.\n",(unsigned int)buf_node);
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
						printk(KERN_DEBUG "cam: REG_SCITADDR0 0x%x\n", regval);
						ccic_reg_write(cam, REG_SCIDBR0, (cam->buf_node[i].dma_desc[0].sg_dma | SCIDBR_EN));
						if(cam->channels == 3) {
							ccic_reg_write(cam, REG_SCIDBR1, (cam->buf_node[i].dma_desc[1].sg_dma | SCIDBR_EN));
							ccic_reg_write(cam, REG_SCIDBR2, (cam->buf_node[i].dma_desc[2].sg_dma | SCIDBR_EN));
						}
					} else {
						/* if it is not the last buf which DMA looping in, just append desc to the tail of the DMA chain*/
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
		printk(KERN_DEBUG "cam: move dma of 0x%x to full_list.\n", sbuf_tmp->dma_handles);
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

	srcphyaddr = 0;	/* TBD */
	dstphyaddr = cam->buf_node[index].dma_handles;

	for (i = 0; i < cam->channels; i++) {
		printk(KERN_DEBUG "cam: index %d, channels %d\n",index, i);
		pxa_dma = &cam->buf_node[index].dma_desc[i];
		len = cam->channel_size[i];
		pxa_dma->sglen = (len + SINGLE_DESC_TRANS_MAX - 1) / SINGLE_DESC_TRANS_MAX;
		pxa_dma->sg_size = (pxa_dma->sglen) * sizeof(struct pxa_dma_desc);
		if (pxa_dma->sg_cpu == NULL){
			pxa_dma->sg_cpu = dma_alloc_coherent(&(cam->pdev->dev), pxa_dma->sg_size,
					     &pxa_dma->sg_dma, GFP_KERNEL);
		}
		printk(KERN_DEBUG "cam: sglen %d, size %d, sg_cpu 0x%x\n",pxa_dma->sglen, pxa_dma->sg_size, (unsigned int)pxa_dma->sg_cpu);
		if (!pxa_dma->sg_cpu){
			printk(KERN_ERR "cam: dma_alloc_coherent failed at chnnl %d!\n", i);
			goto err;
		}

		dma_desc_tmp = pxa_dma->sg_cpu;
		dma_desc_phy_tmp = pxa_dma->sg_dma;

		while (len) {
			len_tmp = len > SINGLE_DESC_TRANS_MAX ?
				SINGLE_DESC_TRANS_MAX : len;

			if ((dstphyaddr & 0xf) != 0) {
				printk(KERN_ERR "cam: error: at least we need 16bytes align for DMA!\n");
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

err:
	for (i = 0; i < cam->channels; i++) {
		pxa_dma = &cam->buf_node[index].dma_desc[i];
		if (pxa_dma->sg_cpu) {
			dma_free_coherent(&cam->pdev->dev, pxa_dma->sg_size,
				    pxa_dma->sg_cpu, pxa_dma->sg_dma);
			pxa_dma->sg_cpu = 0;
		}
	}
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

	if (cam->requestbufs == MIN_DMA_BUFS) {
		/*init the dma desc for dummy dma buf, 4k size*/
		for (i = 0; i < cam->channels; i++) {

			pxa_dma = &cam->buf_dummy->dma_desc[i];
			len = cam->channel_size[i];
			pxa_dma->sglen = (len + DMA_DUMMY_SIZE - 1) / DMA_DUMMY_SIZE;
			pxa_dma->sg_size = (pxa_dma->sglen) * sizeof(struct pxa_dma_desc);
			pxa_dma->sg_cpu = dma_alloc_coherent(&(cam->pdev->dev), pxa_dma->sg_size,
							&pxa_dma->sg_dma, GFP_KERNEL);
			printk(KERN_DEBUG "cam: dummy: sglen %d, size %d, sg_cpu 0x%x\n",pxa_dma->sglen, pxa_dma->sg_size, (unsigned int)pxa_dma->sg_cpu);
			if (!pxa_dma->sg_cpu){
				printk(KERN_ERR "cam: dummy: dma_alloc_coherent failed at chnnl %d!\n", i);
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
		if (ret)
			printk(KERN_ERR "cam: failed in alloc dma desc!\n");
	}

	return 0;
}

static int ccic_free_dma_desc(struct ccic_camera *cam)
{
	int i = 0;
	int index;
	struct pxa_cam_dma *pxa_dma;

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

	for (i = 0; i < cam->channels; i++) {
		pxa_dma = &cam->buf_dummy->dma_desc[i];
		if (pxa_dma->sg_cpu) {
			dma_free_coherent(&cam->pdev->dev, pxa_dma->sg_size,
					    pxa_dma->sg_cpu, pxa_dma->sg_dma);
			pxa_dma->sg_cpu = 0;
		}
	}

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

	if (list_empty(&cam->sb_avail)) {
		printk(KERN_ERR "cam: there is no buffer en-queue, failed to link dma desc!\n");
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
	return 0;
}

static int ccic_ctlr_dma(struct ccic_camera *cam)
{
	int ret;
	ccic_init_dma_desc(cam);
	ret = ccic_link_dma_desc(cam);
	return ret;
}

static void ccic_ctlr_image(struct ccic_camera *cam)
{
	struct v4l2_pix_format *fmt = &cam->pix_format;
	unsigned int size = fmt->width*fmt->height;

	switch (fmt->pixelformat) {

	case V4L2_PIX_FMT_RGB565:
		cam->channels = 1;
		cam->channel_size[0] = size*2;
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_RGB565) | SCICR1_FMT_OUT(FMT_RGB565));
	    break;

	case V4L2_PIX_FMT_JPEG:
		cam->channels = 1;
		/* cam->channel_size[0] = size; */
		/* use size get from sensor */
		cam->channel_size[0] = fmt->sizeimage;
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_JPEG) | SCICR1_FMT_OUT(FMT_JPEG));
	    break;

	case V4L2_PIX_FMT_YUV422P:

		cam->channels = 3;
		cam->channel_size[0] = size;
		cam->channel_size[1] = cam->channel_size[2] = size/2;
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV422));
		break;

	case V4L2_PIX_FMT_UYVY:
		cam->channels = 1;
		cam->channel_size[0] = size*2;
		ccic_reg_write(cam, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV422PACKET));
		break;

	case V4L2_PIX_FMT_YUV420:
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
		printk(KERN_ERR "cam: error can not support fmt!\n");
		break;
	}

}


/*
 * Configure the controller for operation; caller holds the
 * device mutex.
 */
static int ccic_ctlr_configure(struct ccic_camera *cam)
{
	int ret;
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

	ccic_reg_write(cam, REG_SCISR, ccic_reg_read(cam, REG_SCISR));
	ccic_reg_clear_bit(cam, REG_SCIMASK, IRQ_FRAME);
}

static void ccic_sci_irq_disable(struct ccic_camera *cam)
{
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
}

static void ccic_sci_stop(struct ccic_camera *cam)
{
	int i = 0;
	unsigned int val = 0;

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
}

void ccic_ctlr_init(struct ccic_camera *cam)
{
	unsigned long flags;

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
}
/*
 * Stop the controller, and don't return until we're really sure that no
 * further DMA is going on.
 */
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

/* -------------------------------------------------------------------- */
/*
 * Communications with the sensor.
 */

static int __ccic_cam_cmd(struct ccic_camera *cam, int cmd, void *arg)
{
	struct i2c_client *sc = cam->sensor;
	int ret;

	if (sc == NULL || sc->driver == NULL || sc->driver->command == NULL)
		return -EINVAL;
	ret = sc->driver->command(sc, cmd, arg);
	if (ret == -EPERM) /* Unsupported command */
		return 0;
	return ret;
}

static int __ccic_cam_reset(struct ccic_camera *cam)
{
	int zero = 0;
	return __ccic_cam_cmd(cam, VIDIOC_INT_RESET, &zero);
}

/*
 * We have found the sensor on the i2c.  Let's try to have a
 * conversation.
 */
static int __attribute__ ((unused)) ccic_cam_init(struct ccic_camera *cam)
{
	int ret;
	struct v4l2_dbg_chip_ident chip = { {}, 0, 0 };
	chip.match.type = V4L2_CHIP_MATCH_I2C_ADDR;
	chip.match.addr = 0;

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
		cam_err(cam, "cam: Unsupported sensor type %d", cam->sensor->addr);
		ret = -EINVAL;
		goto out;
	}
	/* Get/set parameters? */
	ret = 0;
	cam->state = S_IDLE;
out:
	mutex_unlock(&cam->s_mutex);
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
	int ret;

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
	if (!ret) {
#if defined (CONFIG_VIDEO_PXA95X_OV7690)
		/* small resolution use high resolution as bridge in hardware, though it is not modulabe */
		if (SENSOR_LOW == sensor_selected) {
			struct i2c_client *sc = cam->sensors[SENSOR_HIGH];

			if (sc == NULL || sc->driver == NULL
					|| sc->driver->command == NULL) {
				printk(KERN_ERR "no briged could be used\n");
				return -EINVAL;
			}

			ret = sc->driver->command(sc, VIDIOC_SET_BRIDGE, &fmt);
			if (ret == -EPERM) {
				printk(KERN_ERR "command no briged could be used\n");
				return -EINVAL;
			}
		}
#endif
	}
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
static int ccic_alloc_dma_bufs(struct ccic_camera *cam, int loadtime)
{
	int i;

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
	return 0;
}

static void ccic_free_dma_bufs(struct ccic_camera *cam)
{
	int i;
	for (i = 0; i < cam->n_mapbufs; i++) {
		free_pages((unsigned long)cam->buf_mmap[i].dma_bufs, cam->buf_mmap[i].order);
		cam->buf_mmap[i].dma_bufs = NULL;
	}
	cam->n_mapbufs = 0;
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
	return len;
}

/*
 * Get everything ready, and start grabbing frames.
 */
static int ccic_read_setup(struct ccic_camera *cam, enum ccic_state state)
{
	int ret;
	unsigned long flags;
	int idx = cam->v4ldev.minor;
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

	csi_mipi_config(g_csi, cam->pix_format.width);
	if (ccic_needs_config(cam)) {
		ccic_cam_configure(cam);
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

	csi_start(g_csi, idx);// configure enable which camera interface controller
	ccic_sci_start(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	__ccic_cam_cmd(cam, VIDIOC_STREAMON, NULL);

	/*dump_registers(cam);*/
	/*dump_csi_registers(g_csi);*/

	return 0;
}


static ssize_t ccic_v4l_read(struct file *filp,
		char __user *buffer, size_t len, loff_t *pos)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;

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

out_unlock:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Streaming I/O support.
 */

static int ccic_vidioc_streamon(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	printk(KERN_NOTICE "cam: ccic_vidioc_streamon+\n");
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_IDLE || cam->n_sbufs == 0)
		goto out_unlock;

	ret = ccic_read_setup(cam, S_STREAMING);
out_unlock:
	mutex_unlock(&cam->s_mutex);
out:

	printk(KERN_NOTICE "cam: ccic_vidioc_streamon-\n");
	return ret;
}

static unsigned int csi_reg_read(struct ccic_csi *csi, unsigned int reg);

static int ccic_vidioc_streamoff(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	printk(KERN_NOTICE "cam: ccic_vidioc_streamoff+\n");
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_STREAMING)
		goto out_unlock;

	__ccic_cam_cmd(cam, VIDIOC_STREAMOFF, NULL);
	ccic_ctlr_stop_dma(cam);

	ret = 0;
	printk(KERN_NOTICE "cam: ccic_vidioc_streamoff-\n");
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

static int ccic_prepare_buffer_node(struct ccic_camera *cam,
	struct ccic_sio_buffer *sbuf, unsigned long userptr,
                unsigned int size, unsigned int index)
{
	struct pxa_cam_dma *pxa_dma;
	unsigned int vaddr = PAGE_ALIGN(userptr);

	printk(KERN_DEBUG "cam: ccic_prepare_buffer_node+\n");
	if (vaddr != userptr) {
		printk(KERN_ERR "cam: error: the memory is not page align!\n");
		return -EPERM;
	}
	if ((size & 0x1f) != 0) {
		printk(KERN_ERR "camera: the memory size 0x%08x is not 32 bytes align\n", size);
		return -EPERM;
	}
	sbuf->page = va_to_page(vaddr);
	if (!sbuf->page) {
		printk(KERN_ERR "cam: fail to get page info!\n");
		return -EFAULT;
	}
	sbuf->dma_handles = dma_map_page(&cam->pdev->dev,
				sbuf->page,
				0,
				sbuf->v4lbuf.length,
				DMA_FROM_DEVICE);
	if (!sbuf->dma_handles) {
		printk(KERN_ERR "cam: mem is not contiguous!\n");
		return -ENOMEM;
	}
	/* Map the PA to kernel space */
	if (PageHighMem(sbuf->page))
		sbuf->buffer = ioremap(sbuf->dma_handles, PAGE_ALIGN(size));
	else
		sbuf->buffer = page_address(sbuf->page);

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
	printk(KERN_DEBUG "cam: ccic_prepare_buffer_node-\n");
	return 0;
}

static void ccic_free_buffer_node(struct ccic_sio_buffer *sbuf)
{
	/*
	 * vunmap will do TLB flush for us.
	 * We map uncachable memory, so needn't cache invalid operation here.
	 */
	printk(KERN_DEBUG "cam: ccic_free_buffer_node+\n");
	if(V4L2_MEMORY_USERPTR != sbuf->v4lbuf.memory)
		return;

	if (sbuf->buffer) {
		if (PageHighMem(sbuf->page))
			iounmap(sbuf->buffer);
		sbuf->buffer = NULL;
	}
	printk(KERN_DEBUG "cam: ccic_free_buffer_node-\n");
}

static int ccic_setup_siobuf(struct ccic_camera *cam, int index)
{
	struct ccic_sio_buffer *sbuf = cam->sb_bufs + index;

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
	return 0;
}

static int ccic_free_sio_buffers(struct ccic_camera *cam)
{
	int i;
	/*
	 * If any buffers are mapped, we cannot free them at all.
	 */
	if (cam->sb_bufs) {
		for (i = 0; i < cam->n_sbufs; i++) {
			if (cam->sb_bufs[i].mapcount > 0)
				return -EBUSY;
			ccic_free_buffer_node(&cam->sb_bufs[i]);
		}
	}
	/*
	 * OK, let's do it.
	 */

	cam->nbufs = 0;
	cam->n_sbufs = 0;
	if (cam->sb_bufs)
		kfree(cam->sb_bufs);
	cam->sb_bufs = NULL;
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	INIT_LIST_HEAD(&cam->dma_buf_list);
	return 0;
}

static int ccic_vidioc_reqbufs(struct file *filp, void *priv,
		struct v4l2_requestbuffers *req)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;

	printk(KERN_NOTICE "cam: ccic_vidioc_reqbufs+ (count %d)\n", req->count);
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
	printk(KERN_DEBUG "cam: cam->sb_bufs 0x%x, request buf %d\n", (unsigned int)cam->sb_bufs, req->count);
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

	printk(KERN_NOTICE "cam: ccic_vidioc_reqbufs-\n");
	return ret;
}


static int ccic_vidioc_querybuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (buf->index >= cam->n_sbufs)
		goto out;
	*buf = cam->sb_bufs[buf->index].v4lbuf;
	ret = 0;
out:
	mutex_unlock(&cam->s_mutex);
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

		dma_map_page(&cam->pdev->dev,
				sbuf->page,
				0,
				sbuf->v4lbuf.length,
				DMA_FROM_DEVICE);
	}

	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	if (cam->requestbufs == MIN_DMA_BUFS) {
		magic_bit = (u8*)sbuf->buffer;
		memcpy(magic_bit, dma_mark, sizeof(dma_mark));
		/* clean and invalidate cache*/
		dma_map_single(&cam->pdev->dev,
						sbuf->buffer,
						sizeof(dma_mark),
						DMA_BIDIRECTIONAL);
	}

	spin_lock_irqsave(&cam->dev_lock, flags);

	list_for_each_entry(sbuf_tmp, &cam->sb_avail, list){
		if (sbuf_tmp == sbuf) {
			is_in_bufq = true;
			break;
		}
	}
	if (is_in_bufq == false){
		printk(KERN_DEBUG "cam: qbuf, dma 0x%x, index %d\n", sbuf->dma_handles, buf->index);
		list_add_tail(&sbuf->list, &cam->sb_avail);
	}
	/*dump_avail_buf_list(cam);*/
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	ret = 0;
out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_dqbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	struct ccic_sio_buffer *sbuf;
	int ret = -EINVAL;
	unsigned long flags;

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

		printk(KERN_DEBUG "cam: dqbuf dma 0x%x\n", sbuf->dma_handles);
		spin_unlock_irqrestore(&cam->dev_lock, flags);

		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		*buf = sbuf->v4lbuf;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&cam->s_mutex);
out:
	return ret;
}

static void ccic_v4l_vm_open(struct vm_area_struct *vma)
{
	struct ccic_sio_buffer *sbuf = vma->vm_private_data;
	/*
	 * Locking: done under mmap_sem, so we don't need to
	 * go back to the camera lock here.
	 */
	sbuf->mapcount++;
}


static void ccic_v4l_vm_close(struct vm_area_struct *vma)
{
	struct ccic_sio_buffer *sbuf = vma->vm_private_data;

	mutex_lock(&sbuf->cam->s_mutex);
	sbuf->mapcount--;
	/* Docs say we should stop I/O too... */
	if (sbuf->mapcount == 0)
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_MAPPED;
	mutex_unlock(&sbuf->cam->s_mutex);
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
	return ret;
}



static int ccic_v4l_open(struct file *filp)
{
	struct ccic_camera *cam;
	int i;
	bool sensor_exist = false;

	cam = ccic_find_dev(video_devdata(filp)->minor);
	if (cam == NULL) {
		printk(KERN_ERR "cam: no camera driver, failed to open!\n");
		return -ENODEV;
	}
	for (i = 0; i < SENSOR_MAX; i++ ) {
		if (cam->sensors[i] != NULL) {
			sensor_exist = true;
			break;
		}
	}
	if (sensor_exist == false) {
		printk(KERN_ERR "cam: no sensor detected, failed to open!\n");
		return -ENODEV;
	}
	filp->private_data = cam;

	mutex_lock(&cam->s_mutex);
	if (cam->users == 0) {
		/* Disable 1GHz */
		dvfm_disable_op_name("988M", dvfm_dev_idx);
		dvfm_disable(dvfm_dev_idx);
#ifdef _ARB_CHANGE_
		*pri_axi = 1;
		*pri_gcu = 2;
		*pri_ci1 = 0;
		*pri_ci2 = 0;
		printk(KERN_NOTICE "cam: change AXI = 0x%X, GCU = 0x%X, "\
			"CI1 = 0x%X, CI2 = 0x%X\n", \
			*pri_axi, *pri_gcu, *pri_ci1, *pri_ci2);
#endif

	/* FIXME make sure this is complete */
	}
	(cam->users)++;
	mutex_unlock(&cam->s_mutex);
	return 0;
}


static int ccic_v4l_release(struct file *filp)
{
	struct ccic_camera *cam = filp->private_data;
	struct sensor_platform_data *pdata = NULL;
	if (cam->sensor)
		pdata = cam->sensor->dev.platform_data;

	printk(KERN_NOTICE "cam: ccic_v4l_release+\n");
	mutex_lock(&cam->s_mutex);
	(cam->users)--;
	if (filp == cam->owner) {
		ccic_ctlr_stop_dma(cam);
		ccic_free_sio_buffers(cam);
		kzfree(cam->buf_node);
		cam->buf_node = NULL;
		cam->owner = NULL;
	}
	if (cam->users == 0) {
		if (pdata)
			pdata->power_set(sensor_selected, SENSOR_CLOSE);
		if (alloc_bufs_at_read)
			ccic_free_dma_bufs(cam);
		dvfm_enable(dvfm_dev_idx);
		dvfm_enable_op_name("988M", dvfm_dev_idx);
		sensor_selected = SENSOR_NONE;
		cam->sensor = NULL;
#ifdef _ARB_CHANGE_
		*pri_axi = 0;
		*pri_gcu = 0;
		*pri_ci1 = 0;
		*pri_ci2 = 0;
		printk(KERN_NOTICE "CI AXI Fabric RR Arbitration recovered\n");
#endif
	}
	mutex_unlock(&cam->s_mutex);
	printk(KERN_NOTICE "cam: ccic_v4l_release-\n");
	return 0;
}

static unsigned int ccic_v4l_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct ccic_camera *cam = filp->private_data;

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

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERYCTRL, qc);
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int ccic_vidioc_g_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int ccic_vidioc_s_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	struct ccic_camera *cam = file->private_data;
	int ret = 0;

	printk(KERN_NOTICE "cam: ccic_vidioc_querycap+\n");
	strcpy(cap->card, "MG1");
	strcpy(cap->driver, "sensor N/A");
	cap->version = CCIC_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERYCAP, cap);
	mutex_unlock(&cam->s_mutex);
	printk(KERN_NOTICE "cam: ccic_vidioc_querycap-driver = %s\n", cap->driver);
	return ret;
}


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

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_ENUM_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int ccic_vidioc_try_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_TRY_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_s_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;
	printk(KERN_NOTICE "cam: ccic_vidioc_s_fmt_cap+ (width %d, height %d)\n", fmt->fmt.pix.width, fmt->fmt.pix.height);
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
	printk(KERN_NOTICE "cam: ccic_vidioc_s_fmt_cap-\n");
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

	f->fmt.pix = cam->pix_format;
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
	int ret;

	struct sensor_platform_data *pdata;

	if ( !( ((i == SENSOR_LOW) && (detected_low == 1))
		|| ((i == SENSOR_HIGH) && (detected_high == 1)) ) ) {
		printk(KERN_ERR "cam: requested sensor %d is NOT attached!\n", i);
		return -1;
	}

	mutex_lock(&cam->s_mutex);
	/* Before select new sensor, make sure power off current sensor */
	if (sensor_selected != SENSOR_NONE) {
		pdata = cam->sensor->dev.platform_data;
		pdata->power_set(sensor_selected, SENSOR_CLOSE);
	}
	/* Select new sensor */
	cam->sensor = cam->sensors[i];
	sensor_selected = i;
	/* enable clock */
	pdata = cam->sensor->dev.platform_data;
	pdata->power_set(sensor_selected, SENSOR_OPEN);
	__ccic_cam_reset(cam);
	ccic_ctlr_init(cam);
	csi_config(g_csi);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_INPUT, &i);
	mutex_unlock(&cam->s_mutex);

#if 0 /* TODO!for ov7690 bridge solution, at some boards, the settings can not work here, need investigate!*/
#if ! defined(CONFIG_VIDEO_OV7690)
	//To enable OV5642 bridge +OV7690, we got to enable OV7690 entire setting first.
	/* small resolution use high resolution as bridge in hardware, though it is not modulabe */

	if (SENSOR_LOW == sensor_selected) {
		struct i2c_client *sc = cam->sensors[SENSOR_HIGH];

		if (sc == NULL || sc->driver == NULL
				|| sc->driver->command == NULL) {
			printk(KERN_ERR "no briged could be used\n");
			return -EINVAL;
		}

		ret = sc->driver->command(sc, VIDIOC_SET_BRIDGE, NULL);
		if (ret == -EPERM) {
			printk(KERN_ERR "command no briged could be used\n");
			return -EINVAL;
		}

	}
#endif
#endif
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

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
	return ret;
}

static int ccic_vidioc_s_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct ccic_camera *cam = priv;
	int ret;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
	return ret;
}

static int ccic_vidioc_enum_framesizes(struct file *file, void *fh,
                                         struct v4l2_frmsizeenum *fsize)
{
	struct ccic_camera *cam = fh;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_ENUM_FRAMESIZES, fsize);
	mutex_unlock(&cam->s_mutex);

	/* due to SCI_FIFO limitation, earlier steppings don't support
	 * HD resolutions for YUV*/
	if (cpu_is_pxa955_Cx() || cpu_is_pxa955_Dx()) {
		struct v4l2_frmsizeenum frmsize;
		if (copy_from_user(&frmsize, fsize, sizeof(frmsize)))
			return -EFAULT;
		/* If pixel format is YUV, and resolution is HD res*/
		if (((frmsize.pixel_format == V4L2_PIX_FMT_YUV422P)
		  || (frmsize.pixel_format == V4L2_PIX_FMT_YUV420)
		  || (frmsize.pixel_format == V4L2_PIX_FMT_UYVY))
		 && ((frmsize.discrete.height == 720)
		  || (frmsize.discrete.height == 1280)))
			return -EINVAL;
	}

	return ret;
}

static void ccic_v4l_dev_release(struct video_device *vd)
{
	struct ccic_camera *cam = container_of(vd, struct ccic_camera, v4ldev);
	kfree(cam);
}

/*for register access*/
static int ccic_vidioc_g_register(struct file *filp, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct ccic_camera *cam = priv;
	int ret;
	mutex_lock(&cam->s_mutex);
	ret =  __ccic_cam_cmd(cam, VIDIOC_DBG_G_REGISTER, reg);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_s_register(struct file *filp, void *priv,
                struct v4l2_dbg_register *reg)
{
	struct ccic_camera *cam = priv;
	int ret;
	mutex_lock(&cam->s_mutex);
	ret =  __ccic_cam_cmd(cam, VIDIOC_DBG_S_REGISTER, reg);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static long ccic_v4l_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct video_device *vdev = video_devdata(file);
	struct ccic_camera *cam = container_of(vdev, struct ccic_camera, v4ldev);
	int ret;

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
	.vidioc_g_parm          = ccic_vidioc_g_parm,
	.vidioc_s_parm          = ccic_vidioc_s_parm,
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

	if (irqs & IRQ_EOFX)
	{
		clear_bit(CF_DMA_ACTIVE, &cam->flags);
		printk(KERN_DEBUG "cam: IRQ_EOFX\n");
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

	spin_lock(&cam->dev_lock);
	irqs = ccic_reg_read(cam, REG_SCISR);
	ccic_reg_write(cam, REG_SCISR, irqs);		/*clear irqs here*/
	if (irqs & IRQ_OFO) {

		printk(KERN_ERR "cam: ccic over flow error!\n");
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

/* Camera flash sysfs interface functions */
static enum led_brightness cam_flash_get_lum(struct led_classdev *led_cdev)
{
	struct v4l2_control ctrl={V4L2_CID_FLASH_LUMINANCE, 12345};
	struct ccic_camera *cam = container_of(led_cdev, \
				  struct ccic_camera, camflash_led_cdev);

	__ccic_cam_cmd(cam, VIDIOC_G_CTRL, &ctrl);
	printk(KERN_NOTICE "CamFlash: Current flash brightness: %d (0->OFF, 1->100%%, 2->20%%(N/A)\n", ctrl.value);
	return ctrl.value;
}

static void cam_flash_set_lum(struct led_classdev *led_cdev,
	unsigned int value)
{
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
	return;
}

static void cam_flash_set_drtn(struct timed_output_dev *sdev, int timeout)
{
	struct v4l2_control ctrl={V4L2_CID_FLASH_DURATION, timeout};
	struct ccic_camera *cam = container_of(sdev, struct ccic_camera, \
					       camflash_timed_dev);
	if (timeout < 0)
		return;
	__ccic_cam_cmd(cam, VIDIOC_S_CTRL, &ctrl);
	__ccic_cam_cmd(cam, VIDIOC_G_CTRL, &ctrl);
	printk(KERN_NOTICE "CamFlash: Set flash duration: %dms\n", ctrl.value);
	return;
}

static int cam_flash_get_drtn(struct timed_output_dev *sdev)
{
	struct v4l2_control ctrl={V4L2_CID_FLASH_DURATION, 12345};
	struct ccic_camera *cam = container_of(sdev, struct ccic_camera, \
					       camflash_timed_dev);

	__ccic_cam_cmd(cam, VIDIOC_G_CTRL, &ctrl);
	printk(KERN_NOTICE "CamFlash: Current flash duration: %dms\n", ctrl.value);
	return ctrl.value;
}

/* -------------------------------------------------------------------------- */
static int pxa95x_camera_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = -ENOMEM;
	struct ccic_camera *cam;

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

	printk(KERN_NOTICE "cam: pxa95x_camera_probe id %d\n", cam->v4ldev.index);
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

	/* Setup leds-class obj, assume other members is zeroed by kzalloc */
	cam->camflash_led_cdev.name = "spotlight";
	cam->camflash_led_cdev.brightness_set = cam_flash_set_lum;
	cam->camflash_led_cdev.brightness_get = cam_flash_get_lum;
	/* Camera flash dev registed in /sys/class/leds/ */
	ret = led_classdev_register(&pdev->dev, &cam->camflash_led_cdev);
	if (ret < 0) {
		printk(KERN_ERR "CamFlash: flash led dev registration failure\n");
		led_classdev_unregister(&cam->camflash_led_cdev);
	}

	/* Setup timed_output obj */
	cam->camflash_timed_dev.name = "flash";
	cam->camflash_timed_dev.enable = cam_flash_set_drtn;
	cam->camflash_timed_dev.get_time = cam_flash_get_drtn;
	/* Camera flash dev register in /sys/class/timed_output/ */
	ret = timed_output_dev_register(&cam->camflash_timed_dev);
	if (ret < 0) {
		printk(KERN_ERR "CamFlash: flash timed_output dev registration failure\n");
		timed_output_dev_unregister(&cam->camflash_timed_dev);
	}

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
	/* kfree(cam); done in v4l_release () */
}


static int pxa95x_camera_remove(struct platform_device *pdev)
{
	struct ccic_camera *cam = ccic_find_by_pdev(pdev);

	if (cam == NULL) {
		printk(KERN_WARNING "cam: remove on unknown pdev %p\n", pdev);
		return -EIO;
	}
	mutex_lock(&cam->s_mutex);
	if (cam->users > 0)
		cam_warn(cam, "cam: Removing a device with users!\n");
	ccic_shutdown(cam);

	led_classdev_unregister(&cam->camflash_led_cdev);
	timed_output_dev_unregister(&cam->camflash_timed_dev);

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
static void csi_mipi_config(struct ccic_csi *csi, unsigned int width)
{
	unsigned int val = 0;
	unsigned int clk;
	clk = (ACSR&BBU_ACSR_ALUF_MASK)>>BBU_ACSR_ALUF_OFFSET;
	val = csi_reg_read(csi, REG_CSGCR);
	val &= 0xfffff000;/*bit 11:0 clk divider*/
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

	csi_reg_write(csi, REG_CSGCR, val);

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
			/* for all the other resolution */
			if (SENSOR_LOW == sensor_selected) {
				val = CAM_CSI_CONT_HS_Rx_TO(0xffff) |
				CAM_CSI_CONT_HSTSETTLE(0x48);
			} else 	if (SENSOR_HIGH == sensor_selected) {
				val = CAM_CSI_CONT_HS_Rx_TO(0xffff) |
				CAM_CSI_CONT_HSTSETTLE(0x48);
			} else {
				printk(KERN_ERR "cam: no sensor selected\n");
				return;
			}
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

}

static void csi_config(struct ccic_csi *csi)
{
	unsigned int val = 0;
	unsigned int clk = 0;
#ifdef CONFIG_PXA95x
        unsigned int *calibration_p;
#endif

	/* using default sys video clock */
	clk = (ACSR&BBU_ACSR_ALUF_MASK)>>BBU_ACSR_ALUF_OFFSET;

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
	csi_reg_write(csi, REG_CSGCR, val);

	/* default: csi controller channel 0 data route to camera interface 0, channel 1 data route to camera interface 1. */
	val = 0x01;
	csi_reg_write(csi, REG_CSSCR, val);

	val = CAM_CSI_CONT_CLTERMEN(0x00) |
		  CAM_CSI_CONT_CLSETTLE(0x0C) |
		  CAM_CSI_CONT_CLMISS(0x00)   |
		  CAM_CSI_CONT_HSTERMEN(0x04);
	csi_reg_write(csi, REG_CSxTIM0, val);

	/* set time out before declarating an error to maximum */

	if (SENSOR_LOW == sensor_selected) {
		val = CAM_CSI_CONT_HS_Rx_TO(0xffff) |
		      CAM_CSI_CONT_HSTSETTLE(0x48);
	} else if (SENSOR_HIGH == sensor_selected ) {
		val = CAM_CSI_CONT_HS_Rx_TO(0xffff) |
		      /* CAM_CSI_CONT_HSTSETTLE(0x0e); *//* for ov3640 MIPI timming */
		      CAM_CSI_CONT_HSTSETTLE(0x48); /* for ov5642 MIPI timing, both work on MG1-JIL and PV2*/
	} else {
		printk(KERN_ERR "cam: no sensor selected\n");
		return;
	}
	csi_reg_write(csi, REG_CSxTIM1, val);

#if 1 /* for ov5642, 1 lane */
	/* 1 lane */
	val = CAM_CSI_CONT_NOL(0x00) |
		  CAM_CSI_CONT_VC0_CFG(0x0) |
		  CAM_CSI_CONT_VC1_CFG(0x1) |
		  CAM_CSI_CONT_VC2_CFG(0x2) |
		  CAM_CSI_CONT_VC3_CFG(0x3);
#else /* for ov3640, 2 lanes */
	/* packet data with 00 goto channel 0, packet data with 01 goto channel 1*/
	/* channel 2 and channel 3 are reserved for future use */

	val = CAM_CSI_CONT_NOL(0x01) |	/* 2 lanes */
		  CAM_CSI_CONT_VC0_CFG(0x0) |
		  CAM_CSI_CONT_VC1_CFG(0x1);
#endif

	csi_reg_write(csi, REG_CSxCR0, val);
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
	switch(camif)// we assume sensor send all its data with packet 00, we have config packet 00 goto channel 0 in csi_config()
	{
        case CCIC_1:// channel 0 data goto camera interface 1
			/* for pxa955, Ax Bx steppings are not supported */
			if (cpu_is_pxa955_Cx() || cpu_is_pxa955_Dx())
				val = 0x10;
			else
				val = 0x01;

			csi_reg_write(csi, REG_CSSCR, val);
            break;

        case CCIC_0:// channel 1 data goto camera interface 0
        default:
			if (cpu_is_pxa955_Cx() || cpu_is_pxa955_Dx())
				val = 0x01;
			else
				val = 0x10;
			csi_reg_write(csi, REG_CSSCR, val);
			break;
	}
	val = csi_reg_read(csi, REG_CSxCR0);
	val |= CSxCR0_CSIEN;
	csi_reg_write(csi, REG_CSxCR0, val);
	if (!(cpu_is_pxa955_Cx() || cpu_is_pxa955_Dx())) {
		val = 0x0;
		csi_reg_write(csi,REG_CSxDTINTLV,val);
	}
}

static void csi_stop(struct ccic_csi *csi)
{
	unsigned int val = csi_reg_read(csi, REG_CSxCR0);

	val &= ~CSxCR0_CSIEN;
	csi_reg_write(csi, REG_CSxCR0, val);
}


static irqreturn_t csi_irq(int irq, void *data)
{
	struct ccic_csi *csi = data;
	unsigned int irqs;

	spin_lock(&csi->dev_lock);
	irqs = csi_reg_read(csi, REG_CSxINST);
	csi_reg_write(csi, REG_CSxINST, irqs);
	spin_unlock(&csi->dev_lock);
	return IRQ_HANDLED;
}


static int pxa95x_csi_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = -ENOMEM;
	struct ccic_csi *csi;

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
	csi_config(csi);
	init_platform_ops->power_set(0);
	//init_platform_ops = NULL;
	ret = 0;

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
	dvfm_register("PXA95X-Camera", &dvfm_dev_idx);

	return ret;
}

static void __exit pxa95x_camera_exit(void)
{
	platform_driver_unregister(&pxa95x_camera_driver);
	platform_driver_unregister(&pxa95x_csi_driver);
	dvfm_unregister("PXA95X-Camera", &dvfm_dev_idx);
}

module_init(pxa95x_camera_init);
module_exit(pxa95x_camera_exit);

