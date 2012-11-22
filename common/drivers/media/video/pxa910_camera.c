/*
 *  linux/drivers/media/video/pxa910_camera.c - PXA9XX MMCI driver
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
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/videodev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/pm_qos_params.h>

#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

#include <linux/clk.h>
#include "pxa910_camera.h"

#include <mach/camera.h>
#include <mach/regs-apmu.h>
#include <asm/mach-types.h>
#include <mach/cputype.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>

static struct wake_lock idle_lock;
static struct pm_qos_request_list *pxa910_camera_qos_req_min;
static struct pm_qos_request_list *pxa910_camera_qos_disable_cpufreq;
static void set_power_constraint(void)
{
	pm_qos_update_request(pxa910_camera_qos_req_min, 624);
	pm_qos_update_request(pxa910_camera_qos_disable_cpufreq, 1);
}

static void unset_power_constraint(void)
{
	pm_qos_update_request(pxa910_camera_qos_req_min, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(pxa910_camera_qos_disable_cpufreq, PM_QOS_DEFAULT_VALUE);
}

#ifdef CONFIG_PM
#include <plat/mfp.h>
#include <mach/gpio.h>
#include <mach/regs-apbc.h>
static unsigned long GPIO[14];
static void camera_pin_power_opt(void)
{
	unsigned int pin_index = 0, i = 0;
	for (pin_index = MFP_PIN_GPIO67; pin_index <= MFP_PIN_GPIO80; pin_index++)
		GPIO[i++] = mfp_read(pin_index);

	/* save VCC_IO_GPIO2 0.05mA */
	mfp_write(MFP_PIN_GPIO67, 0xb8c1);
	mfp_write(MFP_PIN_GPIO68, 0xb8c1);
	mfp_write(MFP_PIN_GPIO69, 0xb8c1);
	mfp_write(MFP_PIN_GPIO70, 0xb8c1);
	mfp_write(MFP_PIN_GPIO71, 0xb8c1);
	mfp_write(MFP_PIN_GPIO72, 0xb8c1);
	mfp_write(MFP_PIN_GPIO73, 0xb8c1);
	mfp_write(MFP_PIN_GPIO74, 0xb8c1);
	mfp_write(MFP_PIN_GPIO75, 0xb8c1);
	mfp_write(MFP_PIN_GPIO76, 0xb8c1);
	mfp_write(MFP_PIN_GPIO77, 0xb8c1);
	mfp_write(MFP_PIN_GPIO78, 0xb8c1);
	mfp_write(MFP_PIN_GPIO79, 0xc8c0);
	mfp_write(MFP_PIN_GPIO80, 0x1880);

	__raw_writel(FIRST_SECURITY_VALUE, APBC_PXA910_ASFAR);
	__raw_writel(SECOND_SECURITY_VALUE, APBC_PXA910_ASSAR);
	__raw_writel(AIB_POWER_SHUTDOWN, AIB_GPIO2_IO);
}

static void camera_pin_restore(void)
{
	unsigned int pin_index = 0, i = 0;
	for (pin_index = MFP_PIN_GPIO67; pin_index <= MFP_PIN_GPIO80; pin_index++)
		mfp_write(pin_index, GPIO[i++]);
	/*turn on GPIO2 power domain*/
	__raw_writel(FIRST_SECURITY_VALUE, APBC_PXA910_ASFAR);
	__raw_writel(SECOND_SECURITY_VALUE, APBC_PXA910_ASSAR);
	__raw_writel(AIB_POWER_TURNON, AIB_GPIO2_IO);

	return 0;
}
#endif

#define CCIC_VERSION 0x000000
/*
 * Parameters.
 */
MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("Marvell 88ALP01 CMOS Camera Controller driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("Video");

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

#define DMA_POOL 0
#define MAX_DMA_BUFS 3
/* max dma buffer can be access by user point*/

static int alloc_bufs_at_read = 0;
module_param(alloc_bufs_at_read, bool, 0444);
MODULE_PARM_DESC(alloc_bufs_at_read,
		"Non-zero value causes DMA buffers to be allocated when the "
		"video capture device is read, rather than at module load "
		"time.  This saves memory, but decreases the chances of "
		"successfully getting those buffers.");

static int n_dma_bufs = 3;	//3 frame buffers
module_param(n_dma_bufs, uint, 0644);
MODULE_PARM_DESC(n_dma_bufs,
		"The number of DMA buffers to allocate.  Can be either two "
		"(saves memory, makes timing tighter) or three.");

static int dma_buf_size = 2048 * 1536 * 2;  /* Worst case */

module_param(dma_buf_size, uint, 0444);
MODULE_PARM_DESC(dma_buf_size,
		"The size of the allocated DMA buffers.  If actual operating "
		"parameters require larger buffers, an attempt to reallocate "
		"will be made.");

static int min_buffers = 1;
module_param(min_buffers, uint, 0644);
MODULE_PARM_DESC(min_buffers,
		"The minimum number of streaming I/O buffers we are willing "
		"to work with.");

static int max_buffers = 10;
module_param(max_buffers, uint, 0644);
MODULE_PARM_DESC(max_buffers,
		"The maximum number of streaming I/O buffers an application "
		"will be allowed to allocate.  These buffers are big and live "
		"in vmalloc space.");

static int flip = 0;
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

struct yuv_pointer_t
{
	dma_addr_t y;
	dma_addr_t u;
	dma_addr_t v;
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
	struct vm_area_struct *svma;
	dma_addr_t dma_handles;
	struct yuv_pointer_t yuv_p;
	struct page *page;
};

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
	unsigned int bus_type[SENSOR_MAX];	/* parrallel or MIPI */

	unsigned char __iomem *regs;
	struct list_head dev_list;	/* link to other devices */

	/* DMA buffers */
	unsigned int nbufs;		/* How many are alloc'd */
	int next_buf;			/* Next to consume (dev_lock) */
	unsigned int dma_buf_size;  	/* allocated size */
	int order[MAX_DMA_BUFS];	/* Internal buffer addresses */
	void *dma_bufs[MAX_DMA_BUFS];	/* Internal buffer addresses */
	dma_addr_t dma_handles[MAX_DMA_BUFS]; /* Buffer bus addresses */
	void *rubbish_buf_virt;
	dma_addr_t rubbish_buf_phy;

	/* Streaming buffers */
	unsigned int n_sbufs;		/* How many we have */
	struct ccic_sio_buffer *sb_bufs; /* The array of housekeeping structs */
	struct list_head sb_avail;	/* Available for data (we own) (dev_lock) */
	struct list_head sb_full;	/* With data (user space owns) (dev_lock) */
	struct list_head sb_dma;	/* dma list (dev_lock) */
	struct tasklet_struct s_tasklet;

	/* Current operating parameters */
	u32 sensor_type;		/* Currently ov7670 only */
	struct v4l2_pix_format pix_format;

	/* Locks */
	struct mutex s_mutex; /* Access to this structure */
	spinlock_t dev_lock;  /* Access to device */

	/* Misc */
	wait_queue_head_t iowait;	/* Waiting on frame data */
	unsigned long io_type;
	unsigned int n_map_bufs; 	/* How many buffer from user point*/
	struct early_suspend	ccic_early_suspend;
	struct clk *lcd_clk;
};

#define BUS_PARALLEL 0x01
#define BUS_MIPI 0x02
#define BUS_IS_PARALLEL(b) (b & BUS_PARALLEL)
#define BUS_IS_MIPI(b) (b & BUS_MIPI)

/*
 * Status flags.  Always manipulated with bit operations.
 */
#define CF_DMA_ACTIVE	 3	/* A frame is incoming */

static int detected_high = 0;
static int detected_low = 0;
static int sensor_selected = 2;

struct clk *rst_clk;
struct clk *pxa168_ccic_gate_clk;
static DEFINE_SPINLOCK(reg_lock);
EXPORT_SYMBOL(pxa168_ccic_gate_clk);

static int ccic_enable_clk(struct ccic_camera *cam, struct sensor_platform_data *pdata);

/*
 * Start over with DMA buffers - dev_lock needed.
 */
static void ccic_reset_buffers(struct ccic_camera *cam)
{
	cam->next_buf = -1;
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

/* only for debug */
#if 0
static int dump_register(struct ccic_camera *cam)
{
	unsigned int irqs;
	unsigned long flags;
	spin_lock_irqsave(&cam->dev_lock, flags);
	irqs = ccic_reg_read(cam, REG_IRQSTAT);
	printk("CCIC: REG_IRQSTAT is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IRQSTATRAW);
	printk("CCIC: REG_IRQSTATRAW is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IRQMASK);
	printk("CCIC: REG_IRQMASK is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IMGPITCH);
	printk("CCIC: REG_IMGPITCH is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IMGSIZE);
	printk("CCIC: REG_IMGSIZE is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IMGOFFSET);
	printk("CCIC: REG_IMGOFFSET is 0x%08x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_CTRL0);
	printk("CCIC: REG_CTRL0 is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_CTRL1);
	printk("CCIC: REG_CTRL1 is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_CLKCTRL);
	printk("CCIC: REG_CLKCTRL is 0x%08x\n\n", irqs);

	irqs = ccic_reg_read(cam, REG_CSI2_DPHY3);
	printk("CCIC: REG_CSI2_DPHY3 is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_CSI2_DPHY5);
	printk("CCIC: REG_CSI2_DPHY5 is 0x%08x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_CSI2_DPHY6);
	printk("CCIC: REG_CSI2_DPHY6 is 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_CSI2_CTRL0);
	printk("CCIC: REG_CSI2_CTRL0 is 0x%08x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_Y0BAR );
	printk(KERN_ERR"REG_Y0BAR 0x%08x \n", irqs);
	irqs = ccic_reg_read(cam, REG_Y0BAR + 4);
	printk(KERN_ERR"REG_Y0BAR 0x%08x \n", irqs);
	irqs = ccic_reg_read(cam, REG_Y0BAR + 8);
	printk(KERN_ERR"REG_Y0BAR 0x%08x \n", irqs);
	spin_unlock_irqrestore(&cam->dev_lock, flags);

	return 0;
}
#endif

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
void ccic_set_clock_mipi(void)
{
	unsigned long flags;
	/*
	 *  bit6 = 0 select 312MHz clock as MCLK divider
	 *  bit6 = 1 select 52MHz clock as MCLK divider
	 *
	 *  PHYCLK
	 *  bit7 = 0 select 104Mhz for PHY
	 *  bit7 = 1 select 52MHz for PHY
	 */
	clk_set_rate(rst_clk, 0x6a3f);
	spin_lock_irqsave(&reg_lock, flags);
	//enable MIPI by setting bit 25/26 of apmu_debug register
	__raw_writel(0x06000000 | __raw_readl(APMU_CCIC_DBG), APMU_CCIC_DBG);
	spin_unlock_irqrestore(&reg_lock, flags);

	clk_enable(pxa168_ccic_gate_clk);

	/* use DMA burst 64bytes
	 * REG_CTRL1[26:25] = 0
	 *
	 * CCIC ping-pong buffer select:
	 * REG_CTRL1[27]: 0 means 3 Frames; 1 means 2 Frames
	 *
	 * keep CCIC non-posted and IRE posted write for TD
	 * REG_CTRL1[30] should be 0
	 */

	ccic_set_clock(REG_CTRL1, (1<<27) | 0x3c);          //CCIC power on. bit28 (PWRDNEN) must be cleared.
	/*
	 *  0x3 Core CLK(312Mhz) as CCIC clock
	 *  0x2 AXI CLK(156Mhz@624, 201@806) as CCIC clock
	 */
	ccic_set_clock(REG_CLKCTRL, (0x2<<29 | 13));	//enable 24MCLK = 312M/13

}

void ccic_set_clock_parallel(void)
{
	/*
	 *	bit6 = 0 select 312MHz clock as MCLK divider
	 *	bit6 = 1 select 52MHz clock as MCLK divider
	 */
	clk_set_rate(rst_clk, 0x01f);  // only parallel related register will be enabled
	clk_enable(pxa168_ccic_gate_clk);

	/* use DMA burst 64bytes
	 * REG_CTRL1[26:25] = 0
	 *
	 * CCIC ping-pong buffer select:
	 * REG_CTRL1[27]: 0 means 3 Frames; 1 means 2 Frames
	 *
	 * keep CCIC non-posted and IRE posted write for TD
	 * REG_CTRL1[30] should be 0
	 */
	ccic_set_clock(REG_CTRL1, (1<<27) | 0x3c);          //CCIC power on. bit28 (PWRDNEN) must be cleared.
	/*
	 *  0x3 Core CLK(312Mhz) as CCIC clock
	 *  0x2 AXI CLK(156Mhz@624, 201@806) as CCIC clock
	 */
	ccic_set_clock(REG_CLKCTRL, (0x2<<29) | 13);	//enable 24MCLK = 312M/13
	ccic_set_clock(0x1ec, 0x00004);		//undocumented register???
}
EXPORT_SYMBOL (ccic_set_clock_parallel);

void ccic_disable_clock(void)
{
	unsigned long flags;

	clk_set_rate(rst_clk, 0x6800);
	ccic_set_clock(REG_CLKCTRL, 0x0);
	clk_set_rate(pxa168_ccic_gate_clk, 0);
	ccic_set_clock(REG_CTRL1, 0x0);
	/* disable MIPI */
	spin_lock_irqsave(&reg_lock, flags);
	__raw_writel((~0x06000000) & __raw_readl(APMU_CCIC_DBG), APMU_CCIC_DBG);
	spin_unlock_irqrestore(&reg_lock, flags);
}
EXPORT_SYMBOL (ccic_disable_clock);

static int ccic_enable_clk(struct ccic_camera *cam, struct sensor_platform_data *pdata)
{
	/*TODO need to do below reset for stress test*/
	ccic_disable_clock();

	if (BUS_IS_MIPI(cam->bus_type[sensor_selected]))
		ccic_set_clock_mipi();
	else
		ccic_set_clock_parallel();
	if (pdata->platform_set)
		pdata->platform_set(sensor_selected, rst_clk);//set specific platform clk
	return 0;
}

static void ccic_ctlr_power_down(struct ccic_camera *cam);

int ccic_sensor_attach(struct i2c_client *client)
{
	struct ccic_camera *cam;
	int ret;
	struct v4l2_dbg_chip_ident chip;

	cam = ccic_find_dev(0);		//TODO - only support one camera controller
	if (cam == NULL){
		printk("didn't find camera device!\n");
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
	chip.match.addr = cam->sensor->addr;
        chip.ident = V4L2_IDENT_NONE;
	ret = __ccic_cam_cmd(cam, VIDIOC_DBG_G_CHIP_IDENT, &chip);
        if (ret < 0)
                goto out;
        cam->sensor_type = chip.ident;

	if ((cam->sensor_type == V4L2_IDENT_OV7660) || (cam->sensor_type == V4L2_IDENT_OV7670)) {
		cam->sensors[SENSOR_LOW] = client;
		cam->bus_type[SENSOR_LOW] = BUS_PARALLEL;
		detected_low = 1;
		sensor_selected = SENSOR_LOW; /* Set default sensor */
	} else if (cam->sensor_type == V4L2_IDENT_OV5642 || cam->sensor_type == V4L2_IDENT_OV3640) {
        cam->sensors[SENSOR_HIGH] = client;
		cam->bus_type[SENSOR_HIGH] = BUS_MIPI;
        detected_high = 1;
		if (cam->sensor_type == V4L2_IDENT_OV5642)
			cam->bus_type[SENSOR_HIGH] = BUS_PARALLEL;
		sensor_selected = SENSOR_HIGH;	/* Set default sensor */
	} else {
		cam_err(cam, "Unsupported sensor type %d at addr 0x%x", cam->sensor_type, cam->sensor->addr);
		ret = -EINVAL;
		goto out;
	}
/* Get/set parameters? */
        ret = 0;
        cam->state = S_IDLE;
  out:
        ccic_ctlr_power_down(cam);
	cam->sensor = NULL;
        mutex_unlock(&cam->s_mutex);
        return ret;

}
EXPORT_SYMBOL (ccic_sensor_attach);
static inline int check_jpeg_header(char *jpeg_buff)
{
	/* JPEG Header in buffer
		offset	size 	description
		0		2		JPEG SOI marker (FFD8 hex)
	*/
	u16 jpeg_header;
	if(NULL == jpeg_buff){
		printk(KERN_ERR"JPEG buff is NULL \n" );
		return -EINVAL;
	}
	jpeg_header = jpeg_buff[0] << 8 | jpeg_buff[1];

	if(0xffd8 != jpeg_header){
		printk(KERN_ERR"JPEG:0x%x\n", jpeg_header);
		return -EINVAL;
	}
	return 0;
}

/* ------------------------------------------------------------------- */
/*
 * Deal with the controller.
 */
static inline void ccic_switch_dma(struct ccic_camera *cam, int frame)
{
	struct ccic_sio_buffer *sbuf = NULL, *newsbuf = NULL;
	dma_addr_t phy = 0;
	unsigned long flags = 0;
	/*
	 * if the buffer is dequeued, try to fetch one buffer from avail list
	 * if the buffer is in avail list, move form avail list to full list,
	 *	dequeue the buffer at the same time to protect
	 */
	spin_lock_irqsave(&cam->dev_lock, flags);

	phy = ccic_reg_read(cam, REG_Y0BAR + (frame<<2));
	/* rubbish got data */
	if (cam->rubbish_buf_phy == phy){
		/* the buffer is dequeued */
		if (list_empty(&cam->sb_avail))
			goto out;

		sbuf = list_entry(cam->sb_avail.next,
				struct ccic_sio_buffer, list);
		ccic_reg_write(cam, REG_Y0BAR + (frame<<2), sbuf->yuv_p.y);
		ccic_reg_write(cam, REG_U0BAR + (frame<<2), sbuf->yuv_p.u);
		ccic_reg_write(cam, REG_V0BAR + (frame<<2), sbuf->yuv_p.v);
		list_move_tail(&sbuf->list, &cam->sb_dma);

	} else {  // got the real data
		list_for_each_entry(sbuf, &cam->sb_dma, list) {
			if (phy == sbuf->dma_handles) { // find the match sbuf
				if (cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG){
					dma_map_page(&cam->pdev->dev, sbuf->page,
							0,
							64,
							DMA_FROM_DEVICE);
					if(check_jpeg_header(sbuf->buffer) < 0)
						goto out;
				}

				if (list_empty(&cam->sb_avail)) {  // no more dma buffer
					printk(KERN_DEBUG "CCIC link to rubbish buffer\n");
					ccic_reg_write(cam, REG_Y0BAR + (frame<<2), cam->rubbish_buf_phy);
					ccic_reg_write(cam, REG_U0BAR + (frame<<2), cam->rubbish_buf_phy);
					ccic_reg_write(cam, REG_V0BAR + (frame<<2), cam->rubbish_buf_phy);

				} else {

					newsbuf = list_entry(cam->sb_avail.next,
							struct ccic_sio_buffer, list);
					ccic_reg_write(cam, REG_Y0BAR + (frame<<2), newsbuf->yuv_p.y);
					ccic_reg_write(cam, REG_U0BAR + (frame<<2), newsbuf->yuv_p.u);
					ccic_reg_write(cam, REG_V0BAR + (frame<<2), newsbuf->yuv_p.v);
					list_move_tail(&newsbuf->list, &cam->sb_dma);

				}
				sbuf->v4lbuf.bytesused = cam->pix_format.sizeimage;
				sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
				sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;

				cam->next_buf = frame;
				list_move_tail(&sbuf->list, &cam->sb_full);
				if (! list_empty(&cam->sb_full)){
					wake_up(&cam->iowait);
				}

				goto out;
			}
		}

	}
out:
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return;
}

/* should be protected by spin_lock */
static int ccic_ctlr_dma_mmap(struct ccic_camera *cam)
{
	struct v4l2_pix_format *fmt = &cam->pix_format;

	if (cam->nbufs <= 0){
		printk(KERN_ERR"no buffer for dma in mmap mode %s %d \n", __func__, __LINE__);
		return -ENOMEM;
	}
	ccic_reg_write(cam, REG_Y0BAR, cam->dma_handles[0]);
	ccic_reg_write(cam, REG_Y1BAR, cam->dma_handles[1]);

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P) {
		ccic_reg_write(cam, REG_U0BAR, cam->dma_handles[0] + fmt->width*fmt->height);
		ccic_reg_write(cam, REG_V0BAR, cam->dma_handles[0] + fmt->width*fmt->height + fmt->width*fmt->height/2);
		ccic_reg_write(cam, REG_U1BAR, cam->dma_handles[1] + fmt->width*fmt->height);
		ccic_reg_write(cam, REG_V1BAR, cam->dma_handles[1] + fmt->width*fmt->height + fmt->width*fmt->height/2);
	}
        if (fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
                ccic_reg_write(cam, REG_U0BAR, cam->dma_handles[0] + fmt->width*fmt->height);
                ccic_reg_write(cam, REG_V0BAR, cam->dma_handles[0] + fmt->width*fmt->height + fmt->width*fmt->height/4);
                ccic_reg_write(cam, REG_U1BAR, cam->dma_handles[1] + fmt->width*fmt->height);
                ccic_reg_write(cam, REG_V1BAR, cam->dma_handles[1] + fmt->width*fmt->height + fmt->width*fmt->height/4);
        }

	/*
	 * Store the first two Y buffers (we aren't supporting
	 * planar formats for now, so no UV bufs).  Then either
	 * set the third if it exists, or tell the controller
	 * to just use two.
	 */
	if (cam->nbufs > 2) {
		ccic_reg_write(cam, REG_Y2BAR, cam->dma_handles[2]);
		if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P) {
			ccic_reg_write(cam, REG_U2BAR, cam->dma_handles[2] + fmt->width*fmt->height);
			ccic_reg_write(cam, REG_V2BAR, cam->dma_handles[2] + fmt->width*fmt->height + fmt->width*fmt->height/2);
		}
                if (fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
                        ccic_reg_write(cam, REG_U2BAR, cam->dma_handles[2] + fmt->width*fmt->height);
                        ccic_reg_write(cam, REG_V2BAR, cam->dma_handles[2] + fmt->width*fmt->height + fmt->width*fmt->height/4);
                }

		ccic_reg_clear_bit(cam, REG_CTRL1, C1_TWOBUFS);
	}
	else
		ccic_reg_set_bit(cam, REG_CTRL1, C1_TWOBUFS);

	return 0;
}

/* should be protected by spin_lock */
static int ccic_ctlr_dma(struct ccic_camera *cam)
{
        int frame = 0;
        struct ccic_sio_buffer *sbuf;
	if(cam->n_map_bufs < 2) {
		printk(KERN_ERR "ccic at least 2 dma buffers\n");
                return -ENOMEM;
        }
        for(frame = 0; frame < cam->n_map_bufs; frame++) {
                /* suppose only 3 buffers queued */
                if(MAX_DMA_BUFS == frame)
                        break;
                sbuf = cam->sb_bufs + frame;
                ccic_reg_write(cam, REG_Y0BAR + (frame<<2), sbuf->yuv_p.y);
                ccic_reg_write(cam, REG_U0BAR + (frame<<2), sbuf->yuv_p.u);
                ccic_reg_write(cam, REG_V0BAR + (frame<<2), sbuf->yuv_p.v);

		list_move_tail(&sbuf->list, &cam->sb_dma);
        }
	/* prefer 2 DMA channels */
       //if (cam->n_map_bufs > 2)
       //        ccic_reg_clear_bit(cam, REG_CTRL1, C1_TWOBUFS);
      // else
             ccic_reg_set_bit(cam, REG_CTRL1, C1_TWOBUFS);

	return 0;
}

int ccic_config_dma_base(struct ccic_camera *cam)
{
	int ret = 0;
	unsigned long flags = 0;
	spin_lock_irqsave(&cam->dev_lock, flags);
	if(V4L2_MEMORY_USERPTR == cam->io_type){
		ret = ccic_ctlr_dma(cam);
	}else if(V4L2_MEMORY_MMAP == cam->io_type){
		ret = ccic_ctlr_dma_mmap(cam);
	}else{
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return ret;
}

static int ccic_config_image(struct ccic_camera *cam)
{
	int ret = 0;

	int imgsz;
	unsigned int temp;
	struct v4l2_pix_format *fmt = &cam->pix_format;
	int widthy = 0, widthuv = 0;

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV420){
		imgsz = ((fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK) | (((fmt->bytesperline)*4/3) & IMGSZ_H_MASK);
	}else if(fmt->pixelformat == V4L2_PIX_FMT_JPEG){
		imgsz = (((fmt->sizeimage/fmt->bytesperline)<< IMGSZ_V_SHIFT) & IMGSZ_V_MASK) | (fmt->bytesperline & IMGSZ_H_MASK);
		printk(KERN_ERR" %s %d bytesperline %d height %d\n", __func__, __LINE__,
				fmt->bytesperline, fmt->sizeimage / fmt->bytesperline);

	}else {
		imgsz = ((fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK) | (fmt->bytesperline & IMGSZ_H_MASK);
	}
		/* YPITCH just drops the last two bits */
	//ccic_reg_write_mask(cam, REG_IMGPITCH, fmt->bytesperline,
	//		IMGP_YP_MASK);
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
            widthy = fmt->width*2;
            widthuv = fmt->width*2;
	    break;
	case V4L2_PIX_FMT_RGB565:
	    widthy = fmt->width*2;
	    widthuv = 0;
	    break;
	case V4L2_PIX_FMT_JPEG:
		#if 0
		if (BUS_IS_MIPI(cam->bus_type[sensor_selected])){
			/* fix me do not need this*/
			widthy = 0x1000;
			widthuv = 0;
			imgsz = 0x18001000;
		}else{/* same as 422pack for parallel */
		#endif
			widthy = fmt->bytesperline;
			widthuv = fmt->bytesperline;
	//	}
		break;
	case V4L2_PIX_FMT_YUV422P:
                widthy = fmt->width;
                widthuv = fmt->width/2;
		break;
	case V4L2_PIX_FMT_YUV420:
		widthy = fmt->width;
		widthuv = fmt->width/2;
		break;
	default:
		break;
	}

	ccic_reg_write(cam, REG_IMGPITCH, widthuv << 16 | widthy);
	ccic_reg_write(cam, REG_IMGSIZE, imgsz);
	ccic_reg_write(cam, REG_IMGOFFSET, 0x0);
/*
	 * Tell the controller about the image format we are using.
	 */
	switch (cam->pix_format.pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
	    ccic_reg_write_mask(cam, REG_CTRL0,
			    C0_DF_YUV|C0_YUV_PLANAR|C0_YUVE_YVYU,	/*the endianness of sensor output is UYVY(Y1CrY0Cb)*/
			    C0_DF_MASK);
	    break;
        case V4L2_PIX_FMT_YUV420:
            ccic_reg_write_mask(cam, REG_CTRL0,
                            C0_DF_YUV|C0_YUV_420PL|C0_YUVE_YVYU,	/*the endianness of sensor output is UYVY(Y1CrY0Cb)*/
                            C0_DF_MASK);
            break;

	case V4L2_PIX_FMT_YUYV:
	    ccic_reg_write_mask(cam, REG_CTRL0,
			    C0_DF_YUV|C0_YUV_PACKED|C0_YUVE_YUYV,	/*the endianness of sensor output is UYVY(Y1CrY0Cb)*/
			    C0_DF_MASK);
	    break;
	case V4L2_PIX_FMT_UYVY:
		ccic_reg_write_mask(cam, REG_CTRL0,
				C0_DF_YUV|C0_YUV_PACKED|C0_YUVE_UYVY,	/*the endianness of sensor output is UYVY(Y1CrY0Cb)*/
				C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_JPEG:		//C0_YUV_PACKED must be set for JPEG?!
	    if (BUS_IS_MIPI(cam->bus_type[sensor_selected])){
		    ccic_reg_write_mask(cam, REG_CTRL0,
				    C0_DF_RGB|C0_RGB_BGR|C0_RGB4_BGRX,	/* Set CTRL0 as 0x10a8 for JPEG */
				    C0_DF_MASK);
	    }else{
		    ccic_reg_write_mask(cam, REG_CTRL0,
				    C0_DF_YUV|C0_YUV_PACKED|C0_YUVE_YUYV,
				    C0_DF_MASK);
	    }
	    break;

	case V4L2_PIX_FMT_RGB444:
	    ccic_reg_write_mask(cam, REG_CTRL0,
			    C0_DF_RGB|C0_RGBF_444|C0_RGB4_XRGB,
			    C0_DF_MASK);
		/* Alpha value? */
	    break;

	case V4L2_PIX_FMT_RGB565:
	    ccic_reg_write_mask(cam, REG_CTRL0,
			    C0_DF_RGB|C0_RGBF_565|C0_RGB5_BGGR,
			    C0_DF_MASK);
	    break;

	default:
	    cam_err(cam, "Unknown format %x\n", cam->pix_format.pixelformat);
	    break;
	}
	/*
	 * Make sure it knows we want to use hsync/vsync.
	 */
	ccic_reg_write_mask(cam, REG_CTRL0, C0_SIF_HVSYNC, C0_SIFM_MASK);
	/*
	 * This field controls the generation of EOF(internal signal)
	 */
	temp = ccic_reg_read(cam, REG_CTRL0);
	temp |=  CO_EOF_VSYNC | C0_VEDGE_CTRL;
	ccic_reg_write(cam, REG_CTRL0, temp);
	return ret;
}

static void ccic_ctlr_irq_enable(struct ccic_camera *cam)
{
	/*
	 * Clear any pending interrupts, since we do not
	 * expect to have I/O active prior to enabling.
	 */
	ccic_reg_write(cam, REG_IRQSTAT, FRAMEIRQS);
	ccic_reg_set_bit(cam, REG_IRQMASK, FRAMEIRQS);
}

static void ccic_ctlr_irq_disable(struct ccic_camera *cam)
{
	ccic_reg_clear_bit(cam, REG_IRQMASK, FRAMEIRQS);
}

/*
 * Make the controller start grabbing images.  Everything must
 * be set up before doing this.
 */
static void ccic_ctlr_start(struct ccic_camera *cam)
{
	/* set_bit performs a read, so no other barrier should be
	   needed here */
	ccic_reg_set_bit(cam, REG_CTRL0, C0_ENABLE);
}

static void ccic_ctlr_stop(struct ccic_camera *cam)
{
	ccic_reg_clear_bit(cam, REG_CTRL0, C0_ENABLE);
}

void ccic_ctlr_init(struct ccic_camera *cam)
{


	/*
	 * Make sure it's not powered down.
	 */
	ccic_reg_clear_bit(cam, REG_CTRL1, C1_PWRDWN);
	/*
	 * Turn off the enable bit.  It sure should be off anyway,
	 * but it's good to be sure.
	 */
	ccic_reg_clear_bit(cam, REG_CTRL0, C0_ENABLE);
	/*
	 * Mask all interrupts.
	 */
	ccic_reg_write(cam, REG_IRQMASK, 0);
	/*
	 * Clock the sensor appropriately.  Controller clock should
	 * be 48MHz, sensor "typical" value is half that.
	 */
}
/*
 * Stop the controller, and don't return until we're really sure that no
 * further DMA is going on.
 */
static void ccic_ctlr_stop_dma(struct ccic_camera *cam)
{

	/*
	 * Theory: stop the camera controller (whether it is operating
	 * or not).  Delay briefly just in case we race with the SOF
	 * interrupt, then wait until no DMA is active.
	 */

	ccic_ctlr_stop(cam);
	/*CSI2/DPHY need to be cleared, or no EOF will be received*/
	ccic_reg_write(cam, REG_CSI2_DPHY3, 0x0);
	ccic_reg_write(cam, REG_CSI2_DPHY6, 0x0);
	ccic_reg_write(cam, REG_CSI2_DPHY5, 0x0);
	ccic_reg_write(cam, REG_CSI2_CTRL0, 0x0);

	cam->state = S_IDLE;
	ccic_ctlr_irq_disable(cam);
}

/*
 * Power up and down.
 */
void ccic_ctlr_power_up(struct ccic_camera *cam)
{
	/*
	 * Part one of the sensor dance: turn the global
	 * GPIO signal on.
	 */
	ccic_reg_clear_bit(cam, REG_CTRL1, C1_PWRDWN);
}

static void ccic_ctlr_power_down(struct ccic_camera *cam)
{
	ccic_reg_set_bit(cam, REG_CTRL1, C1_PWRDWN);
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

static int ccic_set_sensor_fmt(struct ccic_camera *cam)
{
	struct v4l2_format fmt;
	int ret = 0;

	if (cam->state != S_IDLE)
	{
		return -EINVAL;
	}
	fmt.fmt.pix = cam->pix_format;
		ret = __ccic_cam_cmd(cam, VIDIOC_S_FMT, &fmt);
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

	if (loadtime)
		cam->dma_buf_size = dma_buf_size;
	else
		cam->dma_buf_size = cam->pix_format.sizeimage;
	if (n_dma_bufs > MAX_DMA_BUFS)
		n_dma_bufs = MAX_DMA_BUFS;

	cam->nbufs = 0;
	for (i = 0; i < n_dma_bufs; i++) {
#if DMA_POOL
		cam->dma_bufs[i] = dma_alloc_coherent(&cam->pdev->dev,
				cam->dma_buf_size, cam->dma_handles + i,
				GFP_KERNEL);
#else
		cam->order[i] = get_order(cam->dma_buf_size);
		cam->dma_bufs[i] = (void *)__get_free_pages(GFP_KERNEL, cam->order[i]);
		cam->dma_handles[i] = __pa(cam->dma_bufs[i]);
#endif
		if (cam->dma_bufs[i] == NULL) {
			cam_warn(cam, "Failed to allocate DMA buffer\n");
			break;
		}
		/* For debug, remove eventually */
		memset(cam->dma_bufs[i], 0xcc, cam->dma_buf_size);
		(cam->nbufs)++;
	}

	switch (cam->nbufs) {
	case 1:
#if DMA_POOL
	    dma_free_coherent(&cam->pdev->dev, cam->dma_buf_size,
			    cam->dma_bufs[0], cam->dma_handles[0]);
#else
	    free_pages((unsigned long)cam->dma_bufs[0], cam->order[0]);
#endif
	    cam->nbufs = 0;
	case 0:
	    cam_err(cam, "Insufficient DMA buffers, cannot operate\n");
	    return -ENOMEM;

	case 2:
	    if (n_dma_bufs > 2)
		    cam_warn(cam, "Will limp along with only 2 buffers\n");
	    break;
	}
	return 0;
}

static void ccic_free_dma_bufs(struct ccic_camera *cam)
{
	int i;
	for (i = 0; i < cam->nbufs; i++) {
#if DMA_POOL
		dma_free_coherent(&cam->pdev->dev, cam->dma_buf_size,
				cam->dma_bufs[i], cam->dma_handles[i]);
#else
		if(cam->dma_bufs[i]){
			free_pages((unsigned long)cam->dma_bufs[i], cam->order[i]);
#endif
			cam->dma_bufs[i] = NULL;
		}
	}
	cam->nbufs = 0;
}


/*
 * Get everything ready, and start grabbing frames.
 */
static int ccic_config_phy(struct ccic_camera *cam, enum ccic_state state)
{

	/*
	 * Turn it loose.
	 */
	ccic_reset_buffers(cam);
	ccic_ctlr_irq_enable(cam);
	cam->state = state;

	if (BUS_IS_MIPI(cam->bus_type[sensor_selected])){
		//TODO DPHY clock tunning

		if(cam->sensor_type == V4L2_IDENT_OV3640){
			ccic_reg_write(cam, REG_CSI2_DPHY3, 0x0a06);
			ccic_reg_write(cam, REG_CSI2_DPHY6, 0x0a00);
		}
		else if(cam->sensor_type == V4L2_IDENT_OV5642){
			ccic_reg_write(cam, REG_CSI2_DPHY3, 0x1b0b);
			ccic_reg_write(cam, REG_CSI2_DPHY6, 0x1a03);
		}
#define ENABLE_TWO_LANES
#ifdef ENABLE_TWO_LANES
		ccic_reg_write(cam, REG_CSI2_DPHY5, 0x33);		/* 0x33 for 2 lanes. 0x11 for 1 lane */
		ccic_reg_write(cam, REG_CSI2_CTRL0, 0x43);		/* 0x43 for 2 lanes, 0x41 for 1 lane. */
#else
		ccic_reg_write(cam, REG_CSI2_DPHY5, 0x11);		/* 0x33 for 2 lanes. 0x11 for 1 lane */
		ccic_reg_write(cam, REG_CSI2_CTRL0, 0x41);		/* 0x43 for 2 lanes, 0x41 for 1 lane. */
#endif

	} else {
		ccic_reg_write(cam, REG_CSI2_DPHY3, 0x0);
		ccic_reg_write(cam, REG_CSI2_DPHY6, 0x0);
		ccic_reg_write(cam, REG_CSI2_DPHY5, 0x0);
		ccic_reg_write(cam, REG_CSI2_CTRL0, 0x0);
	}
	ccic_ctlr_start(cam);
	return  __ccic_cam_cmd(cam, VIDIOC_STREAMON, NULL);
}

/*
 * Streaming I/O support.
 */

static int ccic_vidioc_streamon(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	ret = ccic_config_dma_base(cam);
	if(ret < 0){
		printk(KERN_ERR" ccic_config_dma_base failed %s %d \n", __FUNCTION__, __LINE__);
		goto out_unlock;
	}
	if (cam->state != S_IDLE || cam->n_sbufs == 0)
		goto out_unlock;

	ret = ccic_config_phy(cam, S_STREAMING);
	wake_lock(&idle_lock);

  out_unlock:
	mutex_unlock(&cam->s_mutex);
  out:
	return ret;
}

static int __stop_ccicandsesnor(struct ccic_camera *cam)
{
	/*
	 * workaround when stop DMA controller!!!
	 * 1) ccic controller must be stopped first,
	 * and it shoud delay for one frame transfer time at least
	 * 2)and then stop the camera sensor's output
	 *
	 * Fix me! need sillcion to add DMA stop/start bit
	 */
	ccic_ctlr_stop_dma(cam);
	/* 166 = 1/6, jpeg fps is 8, that is why we set timeout as 300 */
	mdelay(200);
	return  __ccic_cam_cmd(cam, VIDIOC_STREAMOFF, NULL);
}

static int ccic_vidioc_streamoff(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_STREAMING)
		goto out_unlock;

	ret = __stop_ccicandsesnor(cam);
	wake_unlock(&idle_lock);
out_unlock:
	mutex_unlock(&cam->s_mutex);
out:
	return ret;
}

static int ccic_setup_siobuf(struct ccic_camera *cam, int index)
{
	struct ccic_sio_buffer *buf = cam->sb_bufs + index;

	INIT_LIST_HEAD(&buf->list);
	buf->v4lbuf.length = PAGE_ALIGN(cam->pix_format.sizeimage);
	buf->buffer = vmalloc_user(buf->v4lbuf.length);
	if (buf->buffer == NULL)
		return -ENOMEM;
	buf->mapcount = 0;
	buf->cam = cam;

	buf->v4lbuf.index = index;
	buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->v4lbuf.field = V4L2_FIELD_NONE;
	buf->v4lbuf.memory = V4L2_MEMORY_MMAP;
	/*
	 * Offset: must be 32-bit even on a 64-bit system.  videobuf-dma-sg
	 * just uses the length times the index, but the spec warns
	 * against doing just that - vma merging problems.  So we
	 * leave a gap between each pair of buffers.
	 */
	buf->v4lbuf.m.offset = 2*index*buf->v4lbuf.length;
	return 0;

}
extern struct page* va_to_page(unsigned long user_addr);
extern unsigned long va_to_pa(unsigned long user_addr, unsigned int size);

static int ccic_prepare_buffer_node(struct ccic_camera *cam,
	struct ccic_sio_buffer *buf, unsigned long userptr,
                unsigned int size, unsigned int index)
{
	unsigned int vaddr = PAGE_ALIGN(userptr);
	struct v4l2_pix_format *fmt = &cam->pix_format;
	if (vaddr != userptr) {
			printk(KERN_ERR "camera: the memory base 0x%08lx is not page align %s %d \n", userptr,  __func__, __LINE__);
			BUG_ON(1);
			return -EPERM;
	}
	if(0 != size% 32){
			printk(KERN_ERR "camera: the memory size 0x%08x is not 32 bytes align\n", size);
			BUG_ON(1);
			return -EPERM;
	}
	buf->dma_handles = va_to_pa(vaddr, size);
	if (!buf->dma_handles) {
		printk(KERN_ERR"mem is not contiguous %s %d\n",__func__, __LINE__);
		BUG_ON(1);
		return -ENOMEM;
	}
	buf->page= va_to_page(vaddr);
	if (!buf->page) {
		printk(KERN_ERR "camera: fail to get page %s %d info\n",  __func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}
	if(PageHighMem(buf->page)){
		printk(KERN_ERR "camera: HIGHMEM is not supported %s %d info\n",  __func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}

	buf->buffer = page_address(buf->page);
	if (!buf->buffer) {
		printk(KERN_ERR "camera: fail to get buffer info %s %d\n", __func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}

	INIT_LIST_HEAD(&buf->list);
	buf->v4lbuf.length = size;
	buf->mapcount = 0;

	buf->v4lbuf.index = index;
	buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->v4lbuf.field = V4L2_FIELD_NONE;
	buf->v4lbuf.memory = V4L2_MEMORY_USERPTR;
	buf->v4lbuf.m.offset = 2*index*buf->v4lbuf.length;

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P) {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = buf->yuv_p.y + fmt->width*fmt->height;
		buf->yuv_p.v = buf->yuv_p.u + fmt->width*fmt->height/2;
	} else if (fmt->pixelformat == V4L2_PIX_FMT_YUV420){
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = buf->yuv_p.y + fmt->width*fmt->height;
		buf->yuv_p.v = buf->yuv_p.u + fmt->width*fmt->height/4;
	} else {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = cam->rubbish_buf_phy;
		buf->yuv_p.v = cam->rubbish_buf_phy;
	}
	return 0;
}

static void ccic_free_buffer_node(struct ccic_sio_buffer *sbuf)
{
	/*
	 * vunmap will do TLB flush for us.
	 * We map uncachable memory, so needn't cache invalid operation here.
	 */

	if(V4L2_MEMORY_USERPTR != sbuf->v4lbuf.memory)
		return;
}

static int ccic_free_sio_buffers(struct ccic_camera *cam)
{
	int i;
	unsigned long flags = 0;

	/*
	 * If any buffers are mapped, we cannot free them at all.
	 */
	for (i = 0; i < cam->n_sbufs; i++){
		if (cam->sb_bufs[i].mapcount > 0)
			return -EBUSY;
		ccic_free_buffer_node(&cam->sb_bufs[i]);
	}
	cam->n_map_bufs = 0;
	/*
	 * OK, let's do it.
	 */
	for (i = 0; i < cam->n_sbufs; i++){
		if(V4L2_MEMORY_MMAP == cam->sb_bufs[i].v4lbuf.memory && cam->sb_bufs[i].buffer){
			vfree(cam->sb_bufs[i].buffer);
			cam->sb_bufs[i].buffer = NULL;
		}
	}
	cam->n_sbufs = 0;
	if(cam->sb_bufs){
		kfree(cam->sb_bufs);
		cam->sb_bufs = NULL;
	}
	spin_lock_irqsave(&cam->dev_lock, flags);
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	INIT_LIST_HEAD(&cam->sb_dma);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return 0;
}

static int ccic_vidioc_reqbufs(struct file *filp, void *priv,
		struct v4l2_requestbuffers *req)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;  /* Silence warning */

	if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&cam->s_mutex);
	/*
	 * If they ask for zero buffers, they really want us to stop streaming
	 * (if it's happening) and free everything.  Should we check owner?
	 */
	if (req->count == 0) {
		if (cam->state == S_STREAMING)
			__stop_ccicandsesnor(cam);
		ret = ccic_free_sio_buffers (cam);
		goto out;
	}

	if (req->memory == V4L2_MEMORY_USERPTR){
		if (cam->state != S_IDLE || (cam->owner && cam->owner != filp)) {
			ret = -EBUSY;
			goto out;
		}
		cam->io_type = V4L2_MEMORY_USERPTR;
		cam->owner = filp;
		/* we do not need kernel to alloc the DAM buffer */
		ccic_free_dma_bufs(cam);
		ret = ccic_free_sio_buffers (cam);
		goto out;
	}

	/* Below handle MMAP method */
	if (req->memory != V4L2_MEMORY_MMAP){
		ret = -EINVAL;
		goto out;
	}

	/*
	 * Device needs to be idle and working.  We *could* try to do the
	 * right thing in S_SPECREAD by shutting things down, but it
	 * probably doesn't matter.
	 */
	if (cam->state != S_IDLE || (cam->owner && cam->owner != filp)) {
		ret = -EBUSY;
		goto out;
	}
	cam->owner = filp;
	cam->io_type = V4L2_MEMORY_MMAP;

	if (req->count < min_buffers)
		req->count = min_buffers;
	else if (req->count > max_buffers)
		req->count = max_buffers;
	if (cam->n_sbufs > 0) {
		ret = ccic_free_sio_buffers(cam);
		if (ret)
			goto out;
	}

	cam->sb_bufs = kzalloc(req->count*sizeof(struct ccic_sio_buffer),
			GFP_KERNEL);
	if (cam->sb_bufs == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	for (cam->n_sbufs = 0; cam->n_sbufs < req->count; (cam->n_sbufs++)) {
		ret = ccic_setup_siobuf(cam, cam->n_sbufs);
		if (ret)
			break;
	}
	if (cam->n_sbufs == 0)  /* no luck at all - ret already set */
		kfree(cam->sb_bufs);
	req->count = cam->n_sbufs;  /* In case of partial success */

  out:
	mutex_unlock(&cam->s_mutex);
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
	if (buf->index < 0 || buf->index >= cam->n_sbufs)
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
	struct ccic_sio_buffer *sbuf = NULL;
	int ret = -EINVAL;
	unsigned long flags;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;

	if(NULL == cam->sb_bufs){
		cam->sb_bufs = kzalloc(max_buffers * sizeof(struct ccic_sio_buffer),
				GFP_KERNEL);
		if (cam->sb_bufs == NULL) {
			ret = -ENOMEM;
			goto out;
		}

	}
	sbuf = cam->sb_bufs + buf->index;
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_QUEUED) {
		ret = 0; /* Already queued?? */
		goto out;
	}
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_DONE) {
		/* Spec doesn't say anything, seems appropriate tho */
		ret = -EBUSY;
		goto out;
	}

	if ((buf->memory == V4L2_MEMORY_USERPTR)
			&& (buf->index == cam->n_sbufs)){
		if(buf->length < cam->pix_format.sizeimage) {
			printk(KERN_ERR "prepare buffer, size is not enough, buf->length %d, cam->pix_format.sizeimage %d\n",
				buf->length, cam->pix_format.sizeimage );
			goto out;
		}

		if(buf->index > max_buffers) {
			printk(KERN_ERR "Only %d buffers are supported\n", max_buffers);
			goto out;
		}

		if (ccic_prepare_buffer_node(cam, sbuf,
					buf->m.userptr, buf->length, buf->index)){
			ret = -EINVAL;
			goto out;
		}
		cam->n_map_bufs ++;
		cam->n_sbufs ++;
	} else {
		if (buf->index < 0 || buf->index >= cam->n_sbufs)
			goto out;
	}
	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	spin_lock_irqsave(&cam->dev_lock, flags);
	list_add_tail(&sbuf->list, &cam->sb_avail);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	ret = 0;
	if(buf->memory == V4L2_MEMORY_MMAP){
		flush_cache_range(sbuf->svma, (unsigned long)sbuf->buffer, (unsigned long)(sbuf->buffer + cam->pix_format.sizeimage));
	}
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_dqbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	struct ccic_sio_buffer *sbuf = NULL;
	int ret = -EINVAL;
	unsigned long flags = 0;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out_unlock;
	if (cam->state != S_STREAMING)
		goto out_unlock;
	if (list_empty(&cam->sb_full) && filp->f_flags & O_NONBLOCK) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	while (list_empty(&cam->sb_full) && cam->state == S_STREAMING) {
		mutex_unlock(&cam->s_mutex);
		if (wait_event_interruptible(cam->iowait,
						!list_empty(&cam->sb_full))) {
			ret = -ERESTARTSYS;
			goto out;
		}
		mutex_lock(&cam->s_mutex);
	}

	if (cam->state != S_STREAMING)
		ret = -EINTR;
	else {
		spin_lock_irqsave(&cam->dev_lock, flags);
		/* Should probably recheck !list_empty() here */
		sbuf = list_entry(cam->sb_full.next,
				struct ccic_sio_buffer, list);
		list_del_init(&sbuf->list);
		if(buf->memory == V4L2_MEMORY_USERPTR){
			dma_map_page(&cam->pdev->dev,
					sbuf->page,
					0,
					sbuf->v4lbuf.length,
					DMA_FROM_DEVICE);
		}
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

	/* FIXME:
	 * Workaround only. The svma now could only be set
	 * by the first process opens the driver.
	 */
	if (!sbuf->svma)
		sbuf->svma = vma;
}


static void ccic_v4l_vm_close(struct vm_area_struct *vma)
{
	struct ccic_sio_buffer *sbuf = vma->vm_private_data;

	mutex_lock(&sbuf->cam->s_mutex);
	sbuf->mapcount--;
	/* Docs say we should stop I/O too... */
	if (sbuf->mapcount == 0) {
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_MAPPED;
		sbuf->svma = 0;
	}
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

	mutex_lock(&cam->s_mutex);
	for (i = 0; i < cam->n_sbufs; i++)
		if (cam->sb_bufs[i].v4lbuf.m.offset == offset) {
			sbuf = cam->sb_bufs + i;
			break;
		}
	if (sbuf == NULL)
		goto out;

	ret = remap_vmalloc_range(vma, sbuf->buffer, 0);
	if (ret)
		goto out;
	vma->vm_flags |= VM_DONTEXPAND;
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
	struct video_device *vd = video_devdata(filp);
	struct ccic_camera *cam = container_of(vd, struct ccic_camera, v4ldev);
	int ret = 0;
	if (cam == NULL)
		return -ENODEV;
	/* sensor_selected should be 0 or 1 */

	if(2 == sensor_selected){
		printk(KERN_ERR"no sensor detected !\n");
		return -ENODEV;
	}

	set_power_constraint();
#ifdef CONFIG_PM
	camera_pin_restore();
#endif

	filp->private_data = cam;

	mutex_lock(&cam->s_mutex);
	if (cam->users == 0) {
		clk_enable(cam->lcd_clk);
		ccic_ctlr_power_up(cam);
	/* FIXME make sure this is complete */
	}

	cam->sensor = cam->sensors[sensor_selected];

	if (cam->users == 0) {
		if (cam->sensor) {
			ret = ccic_enable_clk(cam, cam->sensor->dev.platform_data);
			((struct sensor_platform_data *)cam->sensor->dev.platform_data)->power_on(1, sensor_selected);
			__ccic_cam_reset(cam);
			ret = __ccic_cam_cmd(cam, VIDIOC_S_INPUT, &sensor_selected);
		} else {
			printk(KERN_ERR "weird, could not find default sensor\n");
			ret = -ENODEV;
		}
	}

	(cam->users)++;
	mutex_unlock(&cam->s_mutex);
	dev_dbg(&(cam)->pdev->dev, "%s %d \n", __FUNCTION__, __LINE__);
	return ret;
}

static int ccic_v4l_release(struct file *filp)
{
	struct ccic_camera *cam = filp->private_data;
	struct sensor_platform_data *pdata;
	pdata = cam->sensor->dev.platform_data;
	mutex_lock(&cam->s_mutex);
	(cam->users)--;
	if (filp == cam->owner) {
		__stop_ccicandsesnor(cam);
		ccic_free_sio_buffers(cam);
		cam->owner = NULL;
	}
	if (cam->users == 0) {
		ccic_disable_clock();
		pdata->power_on(0, sensor_selected);
		ccic_ctlr_power_down(cam);
		ccic_free_dma_bufs(cam);
		clk_disable(cam->lcd_clk);
	}
	mutex_unlock(&cam->s_mutex);

	unset_power_constraint();
#ifdef CONFIG_PM
	camera_pin_power_opt();
#endif

	dev_dbg(&(cam)->pdev->dev, "%s %d \n", __FUNCTION__, __LINE__);
	return 0;
}

static unsigned int ccic_v4l_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct ccic_camera *cam = filp->private_data;
	int ret  = 0;
	poll_wait(filp, &cam->iowait, pt);
	if (! list_empty(&cam->sb_full))
		ret =  POLLIN | POLLRDNORM;
	return ret;
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
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERYCAP, cap);
	mutex_unlock(&cam->s_mutex);
	cap->version = CCIC_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return ret;
}


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


static int ccic_vidioc_try_fmt_cap (struct file *filp, void *priv,
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
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	/*
	 * Can't do anything if the device is not idle
	 * Also can't if there are streaming buffers in place.
	 */
	if (cam->state != S_IDLE) {
		return -EBUSY;
	}
	/*
	 * See if the formatting works in principle.
	 */
	if(pix->pixelformat == V4L2_PIX_FMT_JPEG){
		if(pix->sizeimage <= 0){
			dev_err(&(cam)->pdev->dev, "Error pix->sizeimage = %d %s %d \n",
				pix->sizeimage,  __FUNCTION__, __LINE__);
			return -EINVAL;
		}
	}
	ret = ccic_vidioc_try_fmt_cap(filp, priv, fmt);
	if (ret < 0){
		printk(KERN_ERR" try_fmt error: pix->width %d pix->height %d \n",
			 pix->width, pix->height);
		return ret;
	}
	/*
	 * Now we start to change things for real, so let's do it
	 * under lock.
	 */
	mutex_lock(&cam->s_mutex);
	cam->pix_format = fmt->fmt.pix;
	/*
	 * It looks like this might work, so let's program the sensor.
	 */
	ret = ccic_set_sensor_fmt(cam);
	if(0 != ret)
		goto out;
	/*
	 * configure ccic controller
	 */
	ret = ccic_config_image(cam);
	if(0 != ret)
		goto out;

out:
	mutex_unlock(&cam->s_mutex);
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

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix = cam->pix_format;
	return 0;
}

/*
 * We only have one input - the sensor - so minimize the nonsense here.
 */
static int ccic_vidioc_enum_input(struct file *filp, void *priv,
		struct v4l2_input *input)
{
	int ret = 0;
	struct ccic_camera *cam = priv;
	mutex_lock(&cam->s_mutex);

	if ((input->index == 0 && detected_low == 0) ||
			(input->index == 1 && detected_high == 0) ||
			(input->index != 0 && input->index != 1)){
		ret = -EINVAL;
		goto out_unlock;
	}
	input->type = V4L2_INPUT_TYPE_CAMERA;
	/* input->std = V4L2_STD_ALL; */ /* Not sure what should go here */
	strcpy(input->name, "Camera");

out_unlock:
	mutex_unlock(&cam->s_mutex);

	return ret;
}

static int ccic_vidioc_g_input(struct file *filp, void *priv, unsigned int *i)
{
	struct ccic_camera *cam = priv;
	mutex_lock(&cam->s_mutex);
	*i = sensor_selected;
	mutex_unlock(&cam->s_mutex);
	return 0;
}

static int ccic_vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	struct ccic_camera *cam = filp->private_data;
        int ret = 0;

	/* If the required sensor is the same as the current
	 * active one, return immediately.
	 */
	if (i == sensor_selected)
		return ret;

        mutex_lock(&cam->s_mutex);
	if (((i == SENSOR_LOW) && (detected_low == 1)) || ((i == SENSOR_HIGH) && (detected_high == 1))) {
		/* switch off the previous sensor power */
		((struct sensor_platform_data *)cam->sensor->dev.platform_data)->power_on(0, sensor_selected);
		cam->sensor = cam->sensors[i];
		sensor_selected = i;
	} else {
		printk(KERN_ERR "requested sensor %d is NOT attached!\n", i);
		ret = -EINVAL;
		goto out;
	}
	ret = ccic_enable_clk(cam, cam->sensor->dev.platform_data);

	((struct sensor_platform_data *)cam->sensor->dev.platform_data)->power_on(1, sensor_selected);
	__ccic_cam_reset(cam);

        ret = __ccic_cam_cmd(cam, VIDIOC_S_INPUT, &sensor_selected);
out:
        mutex_unlock(&cam->s_mutex);
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

static int ccic_vidioc_cropcap(struct file *file, void *fh,
					struct v4l2_cropcap *a)
{
	struct ccic_camera *cam = fh;
	struct v4l2_cropcap *ccap = a;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_CROPCAP, ccap);
	mutex_unlock(&cam->s_mutex);
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
	return ret;
}

static void ccic_v4l_dev_release(struct video_device *vd)
{
	struct ccic_camera *cam = container_of(vd, struct ccic_camera, v4ldev);

	kfree(cam);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
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
#endif

static long ccic_v4l_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct video_device *vdev = video_devdata(file);
	struct ccic_camera *cam = container_of(vdev, struct ccic_camera, v4ldev);
	int ret;

	/* Handle some specific cmds */
	switch (cmd) {
	case VIDIOC_ENUM_FRAMESIZES:
	{
		ret = ccic_vidioc_enum_framesizes(file, (void *)cam, (struct v4l2_frmsizeenum *)arg);
		return ret;
	}
#ifdef CONFIG_VIDEO_ADV_DEBUG
	case VIDIOC_DBG_S_REGISTER:
	{
		ret = ccic_vidioc_s_register(file, (void *)cam,	(struct v4l2_dbg_register *)arg);
		return ret;
	}
	case VIDIOC_DBG_G_REGISTER:
	{
		ret = ccic_vidioc_g_register(file, (void *)cam, (struct v4l2_dbg_register *)arg);
		return ret;
	}
#endif
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
	.poll = ccic_v4l_poll,
	.mmap = ccic_v4l_mmap,
	.ioctl = ccic_v4l_ioctl,
};
/* upgrade changes from .25 to .28 */
struct v4l2_ioctl_ops ccic_ioctl_ops = {
	.vidioc_querycap	= ccic_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap	= ccic_vidioc_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap	= ccic_vidioc_try_fmt_cap,
	.vidioc_s_fmt_vid_cap	= ccic_vidioc_s_fmt_cap,
	.vidioc_g_fmt_vid_cap	= ccic_vidioc_g_fmt_cap,
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
	.vidioc_cropcap		= ccic_vidioc_cropcap,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register      = ccic_vidioc_g_register,
	.vidioc_s_register      = ccic_vidioc_s_register,
#endif
};

static struct video_device ccic_v4l_template = {
	.name = "pxa910-camera",
	.vfl_type = VID_TYPE_CAPTURE,	/* upgrade changes from .25 to .28 */
	.minor = -1, /* Get one dynamically */
	.tvnorms = V4L2_STD_NTSC_M,
	.current_norm = V4L2_STD_NTSC_M,  /* make mplayer happy */
	.fops = &ccic_v4l_fops,
	.release = ccic_v4l_dev_release,
	.ioctl_ops		= &ccic_ioctl_ops,	/* upgrade changes from .25 to .28 */
};

static void ccic_frame_tasklet(unsigned long data)
{
	struct ccic_camera *cam = (struct ccic_camera *) data;
	int i;
	unsigned long flags;
	struct ccic_sio_buffer *sbuf;
	spin_lock_irqsave(&cam->dev_lock, flags);
	for (i = 0; i < cam->nbufs; i++) {
		int bufno = cam->next_buf;
		if (bufno < 0) {  /* "will never happen" */
			cam_err(cam, "No valid bufs in tasklet!\n");
			break;
		}
		if (++(cam->next_buf) >= cam->nbufs)
			cam->next_buf = 0;
		if (list_empty(&cam->sb_avail))
			break;  /* Leave it valid, hope for better later */
		if (cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) {
			if(check_jpeg_header(cam->dma_bufs[bufno]) < 0){
				goto out;
			/* drop current JPEG frame because of wrong header */
			}
		}

		sbuf = list_entry(cam->sb_avail.next,
				struct ccic_sio_buffer, list);
		/*
		 * Drop the lock during the big copy.  This *should* be safe...
		 */
		spin_unlock_irqrestore(&cam->dev_lock, flags);
		memcpy(sbuf->buffer, cam->dma_bufs[bufno],
				cam->pix_format.sizeimage);
#if !DMA_POOL
/*		dmac_inv_range(cam->dma_bufs[bufno],\
			cam->dma_bufs[bufno] + cam->pix_format.sizeimage); */
#endif
		sbuf->v4lbuf.bytesused = cam->pix_format.sizeimage;
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
		sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;
		spin_lock_irqsave(&cam->dev_lock, flags);
		list_move_tail(&sbuf->list, &cam->sb_full);
		break;		/* just exit once current frame done */
	}
	if (!list_empty(&cam->sb_full))
		wake_up(&cam->iowait);
out:
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}

/* #define IRQ_DEBUG */
#ifdef IRQ_DEBUG
static int overfl_count;
static inline void ccic_irq_debug(struct ccic_camera *cam)
{
	/*
	int line_num = ccic_reg_read(cam, REG_LNNUM);
	printk(KERN_ERR" line_num = %d\n", line_num);
	*/
}
#endif

static irqreturn_t ccic_irq(int irq, void *data)
{
	struct ccic_camera *cam = data;
	unsigned int irqs;
	short frame = 0x0f; /* means on buffer */
	irqs = ccic_reg_read(cam, REG_IRQSTAT);
	ccic_reg_write(cam, REG_IRQSTAT, irqs);		/* clear irqs here */

#ifdef IRQ_DEBUG
	if (irqs & IRQ_OVERFLOW)
		printk(KERN_ERR" overfl_count = %d\n", overfl_count++);
	/*
	printk(KERN_ERR" irqs = 0x%08x\n", irqs);
	*/
#endif

	if (irqs & IRQ_EOF0)
		frame = 0;
	else if (irqs & IRQ_EOF1)
		frame = 1;
	else if (irqs & IRQ_EOF2)
		frame = 2;
	else
		frame = 0x0f;
	if (0x0f != frame) {
#ifdef IRQ_DEBUG
		ccic_irq_debug(cam);
#endif
		if (cam->state == S_STREAMING) {
			if (V4L2_MEMORY_MMAP == cam->io_type)
				tasklet_schedule(&cam->s_tasklet);
			else if (V4L2_MEMORY_USERPTR == cam->io_type)
				ccic_switch_dma(cam, frame);
		}
	}

	return IRQ_HANDLED;
}
static void ccic_sleep_early_suspend(struct early_suspend *h)
{
	struct ccic_camera *cam = container_of(h, \
			struct ccic_camera, ccic_early_suspend);
	clk_disable(cam->lcd_clk);
}

static void ccic_normal_late_resume(struct early_suspend *h)
{
	struct ccic_camera *cam = container_of(h,\
			struct ccic_camera, ccic_early_suspend);
	clk_enable(cam->lcd_clk);
}

static int pxa910_camera_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	struct ccic_camera *cam;
	/*
	 * Start putting together one of our big camera structures.
	 */
	ret = -ENOMEM;
	rst_clk = clk_get(&pdev->dev, "CCICRSTCLK");
	if (IS_ERR(rst_clk)) {
		dev_err(&pdev->dev, "unable to get CCICRSTCLK");
		return PTR_ERR(rst_clk);
	}

	pxa168_ccic_gate_clk = clk_get(&pdev->dev, "CCICGATECLK");
	if (IS_ERR(pxa168_ccic_gate_clk)) {
		dev_err(&pdev->dev, "unable to get CCICGATECLK");
		return PTR_ERR(pxa168_ccic_gate_clk);
	}

	cam = kzalloc(sizeof(struct ccic_camera), GFP_KERNEL);
	if (cam == NULL)
		goto out;
	cam->lcd_clk = clk_get(NULL, "LCDCLK");
	if (IS_ERR(cam->lcd_clk)) {
		dev_err(&pdev->dev, "unable to get LCDCLK");
		return PTR_ERR(cam->lcd_clk);
	}
	clk_enable(cam->lcd_clk);

	platform_set_drvdata(pdev, cam);
	mutex_init(&cam->s_mutex);
	/* do not need lock mutex in probe for \
	   we bring up sensor in late_init */
	spin_lock_init(&cam->dev_lock);
	cam->state = S_NOTREADY;
	init_waitqueue_head(&cam->iowait);
	cam->pdev = pdev;
	INIT_LIST_HEAD(&cam->dev_list);
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	tasklet_init(&cam->s_tasklet, ccic_frame_tasklet, (unsigned long) cam);

	INIT_LIST_HEAD(&cam->sb_dma);

	cam->irq = platform_get_irq(pdev, 0);
	if (cam->irq < 0)
		return -ENXIO;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "no IO memory resource defined\n");
		return -ENODEV;
	}

	ret = -EIO;
	cam->regs = ioremap(res->start, SZ_4K);
	if (!cam->regs) {
		printk(KERN_ERR "Unable to ioremap pxa910-camera regs\n");
		goto out_free;
	}
	ret = request_irq(cam->irq, ccic_irq, \
			IRQF_SHARED, "pxa910-camera", cam);
	if (ret)
		goto out_iounmap;
	/*
	 * Initialize the controller and leave it powered up.  It will
	 * stay that way until the sensor driver shows up.
	 */
	ccic_ctlr_init(cam);
	ccic_ctlr_power_up(cam);
	/*
	 * Set up I2C/SMBUS communications.  We have to drop the mutex here
	 * because the sensor could attach in this call chain, leading to
	 * unsightly deadlocks.
	 */
	/*
	 * Get the v4l2 setup done.
	 */
	cam->v4ldev = ccic_v4l_template;
	cam->v4ldev.debug = 0;
	cam->v4ldev.dev = pdev->dev;	/* upgrade changes from .25 to .28 */
	cam->v4ldev.index = pdev->id;
	printk(KERN_NOTICE "camera_probe id %d\n", cam->v4ldev.index);

	cam->v4ldev.parent = &pdev->dev;
	ret = video_register_device(&cam->v4ldev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto out_freeirq;
	/*
	 * If so requested, try to get our DMA buffers now.
	 */
	if (alloc_bufs_at_read) {
		if (ccic_alloc_dma_bufs(cam, 1))
			cam_warn(cam, "Unable to alloc DMA buffers at load"
					" will try again later.");
	}

	/* allocate rubbish buffer */
	cam->rubbish_buf_virt = (void *)__get_free_pages(GFP_KERNEL, \
			get_order(dma_buf_size));
	if (!cam->rubbish_buf_virt) {
		printk(KERN_ERR "Can't get memory for rubbish buffer\n");
		ret = -ENOMEM;
		goto out_free;
	} else {
		cam->rubbish_buf_phy = __pa(cam->rubbish_buf_virt);
	}
	cam->ccic_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	cam->ccic_early_suspend.suspend = ccic_sleep_early_suspend;
	cam->ccic_early_suspend.resume = ccic_normal_late_resume;
	register_early_suspend(&cam->ccic_early_suspend);

	ccic_add_dev(cam);
/*	ccic_ctlr_power_down(cam);	//for power optimization */
#ifdef CONFIG_PM
	camera_pin_power_opt();
#endif
	return 0;

out_freeirq:
	ccic_ctlr_power_down(cam);
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
	__stop_ccicandsesnor(cam);
	ccic_ctlr_power_down(cam);
	ccic_free_dma_bufs(cam);
	free_irq(cam->irq, cam);
	iounmap(cam->regs);
	video_unregister_device(&cam->v4ldev);
	/* kfree(cam); done in v4l_release () */
}

static int pxa910_camera_remove(struct platform_device *pdev)
{
	struct ccic_camera *cam = ccic_find_by_pdev(pdev);

	if (cam == NULL) {
		printk(KERN_WARNING "remove on unknown pdev %p\n", pdev);
		return -ENODEV;
	}
	mutex_lock(&cam->s_mutex);
	if (cam->users > 0)
		cam_warn(cam, "Removing a device with users!\n");
	ccic_shutdown(cam);
	/* free rubbish buffer */
	if (cam->rubbish_buf_virt)
		free_pages((unsigned long)cam->rubbish_buf_virt,\
				get_order(dma_buf_size));
	mutex_unlock(&cam->s_mutex);

	return 0;
}


#if 0
/* #ifdef CONFIG_PM */
/*
 * Basic power management.
 */
static int ccic_suspend(struct platform_device *dev, pm_message_t state)
{
	struct ccic_camera *cam = platform_get_drvdata(dev);
	struct sensor_platform_data *pdata;
	enum ccic_state cstate;

	cstate = cam->state; /* HACK - stop_dma sets to idle */
	pdata = cam->sensor->dev.platform_data;
	__stop_ccicandsesnor(cam)
	/* we just keep sensor in idle state in first state. \
	   reset sensor need more configuration */
	/* pdata->power_on(0, sensor_selected); */
	ccic_ctlr_power_down(cam);
	ccic_disable_clock();
	cam->state = cstate;
	return 0;
}


static int ccic_resume(struct platform_device *dev)
{
	struct ccic_camera *cam = platform_get_drvdata(dev);
	int ret = 0;

	ccic_ctlr_init(cam);
	ccic_ctlr_power_down(cam);

	mutex_lock(&cam->s_mutex);
	if (cam->users > 0) {
		ccic_ctlr_power_up(cam);
		ccic_enable_clk(cam, cam->sensor->dev.platform_data);
	}
	mutex_unlock(&cam->s_mutex);

	if (cam->state == S_SPECREAD)
		cam->state = S_IDLE;  /* Don't bother restarting */
	else if (cam->state == S_SINGLEREAD || cam->state == S_STREAMING)
		ret = ccic_read_setup(cam, cam->state);
	return ret;
}
#else
#define ccic_suspend NULL
#define ccic_resume  NULL
#endif  /* CONFIG_PM */

static struct platform_driver pxa910_camera_driver = {
	.driver = {
		.name = "pxa910-camera"
	},
	.probe		= pxa910_camera_probe,
	.remove		= pxa910_camera_remove,
#ifdef CONFIG_PM
	.suspend	= ccic_suspend,
	.resume		= ccic_resume,
#endif

};

static int __devinit pxa910_camera_init(void)
{
	spin_lock_init(&reg_lock);

	wake_lock_init(&idle_lock, WAKE_LOCK_IDLE, "pxa910_camera_idle");
	pxa910_camera_qos_req_min = pm_qos_add_request(PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);
	pxa910_camera_qos_disable_cpufreq = pm_qos_add_request(
		PM_QOS_CPUFREQ_DISABLE, PM_QOS_DEFAULT_VALUE);

	return platform_driver_register(&pxa910_camera_driver);
}

static void __exit pxa910_camera_exit(void)
{
	platform_driver_unregister(&pxa910_camera_driver);
	wake_lock_destroy(&idle_lock);
}

module_init(pxa910_camera_init);
module_exit(pxa910_camera_exit);
