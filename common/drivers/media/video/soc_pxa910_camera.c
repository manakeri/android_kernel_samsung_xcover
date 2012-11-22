/*
 * V4L2 Driver for PXA910 camera host
 *
 * Based on linux/drivers/media/video/pxa_camera.c
 *
 * Copyright (C) 2010, Marvell International Ltd.
 *              Kassey Lee <ygli@marvell.com>
 *				Angela Wan <jwan@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/pm_qos_params.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-contig.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-chip-ident.h>

#include <linux/videodev2.h>
#include <mach/regs-apmu.h>
#include <mach/dma.h>
#include "pxa910_camera.h"

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

#define PXA_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5)
#define PXA910_CAM_DRV_NAME "pxa910-camera"
#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
	((x) >> 24) & 0xff
#define BUS_PARALLEL 0x01
#define BUS_MIPI 0x02
#define BUS_IS_PARALLEL(b) (b & BUS_PARALLEL)
#define BUS_IS_MIPI(b) (b & BUS_MIPI)

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

static int dma_buf_size = 2048 * 1536 * 2;	/* Worst case */

struct yuv_pointer_t {
	dma_addr_t y;
	dma_addr_t u;
	dma_addr_t v;
};

/* buffer for one video frame */
struct pxa910_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;
	enum v4l2_mbus_pixelcode code;
	struct yuv_pointer_t yuv_p;
	struct page *page;
	dma_addr_t dma_handles;
	char *buffer;		/* Where it lives in kernel space */
};

struct pxa910_camera_dev {
	struct soc_camera_host soc_host;

	struct soc_camera_device *icd;
	struct clk *rst_clk;
	struct clk *gateclk;
	struct clk *lcd_clk;

	unsigned int irq;
	void __iomem *base;

	struct pxacamera_platform_data *pdata;
	struct resource *res;
	unsigned long platform_flags;
	unsigned long ciclk;
	unsigned long mclk;

	struct list_head capture;
	struct list_head sb_dma;	/* dma list (dev_lock) */

	spinlock_t list_lock;
	struct v4l2_pix_format pix_format;
	/*
	 * internal use only
	 * rubbisth_buf is used when available
	 * for DMA is less than 3
	 */
	void *rubbish_buf_virt;
	dma_addr_t rubbish_buf_phy;

	/* PM */
	struct early_suspend ccic_early_suspend;
	unsigned int state;
	unsigned int bus_type;
};

struct videobuf_dma_contig_memory {
	u32 magic;
	void *vaddr;
	dma_addr_t dma_handle;
	unsigned long size;
	int is_userptr;
};

/*
 * Device register I/O
 */
static inline void ccic_reg_write(struct pxa910_camera_dev *pcdev,
				  unsigned int reg, unsigned int val)
{
	__raw_writel(val, pcdev->base + reg);
}

static inline unsigned int ccic_reg_read(struct pxa910_camera_dev *pcdev,
					 unsigned int reg)
{
	return __raw_readl(pcdev->base + reg);
}

static inline void ccic_reg_write_mask(struct pxa910_camera_dev *pcdev,
				       unsigned int reg, unsigned int val,
				       unsigned int mask)
{
	unsigned int v = ccic_reg_read(pcdev, reg);

	v = (v & ~mask) | (val & mask);
	ccic_reg_write(pcdev, reg, v);
}

static inline void ccic_reg_clear_bit(struct pxa910_camera_dev *pcdev,
				      unsigned int reg, unsigned int val)
{
	ccic_reg_write_mask(pcdev, reg, 0, val);
}

static inline void ccic_reg_set_bit(struct pxa910_camera_dev *pcdev,
				    unsigned int reg, unsigned int val)
{
	ccic_reg_write_mask(pcdev, reg, val, val);
}

/* only for debug */
#if 0
static int dump_register(struct pxa910_camera_dev *pcdev)
{
	unsigned int irqs;
	irqs = ccic_reg_read(pcdev, REG_IRQSTAT);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_IRQSTAT is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_IRQSTATRAW);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_IRQSTATRAW is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_IRQMASK);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_IRQMASK is %x\n\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_IMGPITCH);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_IMGPITCH is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_IMGSIZE);
	dev_dbg(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_IMGSIZE is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_IMGOFFSET);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_IMGOFFSET is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_CTRL0);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_CTRL0 is %x\n", irqs);
	irqs = ccic_reg_read(pcdev, REG_CTRL1);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_CTRL1 is %x\n", irqs);
	irqs = ccic_reg_read(pcdev, REG_CLKCTRL);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_CLKCTRL is %x\n",
		irqs);

	irqs = ccic_reg_read(pcdev, REG_CSI2_DPHY3);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_CSI2_DPHY3 is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_CSI2_DPHY5);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_CSI2_DPHY5 is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_CSI2_DPHY6);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_CSI2_DPHY6 is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_CSI2_CTRL0);
	dev_err(pcdev->soc_host.v4l2_dev.dev, "CCIC: REG_CSI2_CTRL0 is %x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_Y0BAR);
	dev_err(pcdev->soc_host.v4l2_dev.dev, KERN_ERR "REG_Y0BAR 0x%08x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_Y0BAR + 4);
	dev_err(pcdev->soc_host.v4l2_dev.dev, KERN_ERR "REG_Y0BAR 0x%08x\n",
		irqs);
	irqs = ccic_reg_read(pcdev, REG_Y0BAR + 8);
	dev_err(pcdev->soc_host.v4l2_dev.dev, KERN_ERR "REG_Y0BAR 0x%08x\n",
		irqs);
	return 0;
}
#endif

static void ccic_set_clock(struct pxa910_camera_dev *pcdev, unsigned int reg,
			   unsigned int val)
{
	ccic_reg_write(pcdev, reg, val);
}

/*
   1. APMU CCIC clock reset control
   2. CCIC power on
   3. enable MCLK
   */
static DEFINE_SPINLOCK(reg_lock);
void ccic_set_clock_mipi(struct pxa910_camera_dev *pcdev)
{
	unsigned long flags;
	/*
	 *  bit6 = 0 select 312MHz clock as MCLK divider
	 *  bit6 = 1 select 52MHz clock as MCLK divider
	 */
	clk_set_rate(pcdev->rst_clk, 0x6abf);
	spin_lock_irqsave(&reg_lock, flags);
	/* enable MIPI by setting bit 25/26 of apmu_debug register */
	__raw_writel(0x06000000 | __raw_readl(APMU_CCIC_DBG), APMU_CCIC_DBG);
	spin_unlock_irqrestore(&reg_lock, flags);

	clk_enable(pcdev->gateclk);

	/* use DMA burst 64bytes
	 * REG_CTRL1[26:25] = 0
	 *
	 * CCIC ping-pong buffer select:
	 * REG_CTRL1[27]: 0 means 3 Frames; 1 means 2 Frames
	 *
	 * keep CCIC non-posted and IRE posted write for TD
	 * REG_CTRL1[30] should be 0
	 */

	ccic_set_clock(pcdev, REG_CTRL1, (1 << 27) | 0x3c);
	/*
	 *  0x3 Core CLK(312Mhz) as CCIC clock
	 *  enable 24MCLK = 312M/13
	 */
	ccic_set_clock(pcdev, REG_CLKCTRL, (0x2 << 29 | 13));

}

void ccic_set_clock_parallel(struct pxa910_camera_dev *pcdev)
{
	/*
	 *  bit6 = 0 select 312MHz clock as MCLK divider
	 *  bit6 = 1 select 52MHz clock as MCLK divider
	 */
	clk_set_rate(pcdev->rst_clk, 0x01f);
	clk_enable(pcdev->gateclk);

	/* use DMA burst 64bytes
	 * REG_CTRL1[26:25] = 0
	 *
	 * CCIC ping-pong buffer select:
	 * REG_CTRL1[27]: 0 means 3 Frames; 1 means 2 Frames
	 *
	 * keep CCIC non-posted and IRE posted write for TD
	 * REG_CTRL1[30] should be 0
	 */
	ccic_set_clock(pcdev, REG_CTRL1, (1 << 27) | 0x3c);
	/*
	 *  0x3 Core CLK(312Mhz) as CCIC clock
	 *  enable 24MCLK = 312M/13
	 */
	ccic_set_clock(pcdev, REG_CLKCTRL, (0x2 << 29) | 13);
	ccic_set_clock(pcdev, 0x1ec, 0x00004);
}

void ccic_disable_clock(struct pxa910_camera_dev *pcdev)
{
	unsigned long flags;

	clk_set_rate(pcdev->rst_clk, 0x6800);
	ccic_set_clock(pcdev, REG_CLKCTRL, 0x0);
	clk_set_rate(pcdev->gateclk, 0);
	ccic_set_clock(pcdev, REG_CTRL1, 0x0);
	/* disable MIPI */
	spin_lock_irqsave(&reg_lock, flags);
	__raw_writel((~0x06000000) & __raw_readl(APMU_CCIC_DBG), APMU_CCIC_DBG);
	spin_unlock_irqrestore(&reg_lock, flags);
}

static int ccic_enable_clk(struct pxa910_camera_dev *pcdev)
{
	/*TODO need to do below reset for stress test */
	ccic_disable_clock(pcdev);

	if (BUS_IS_MIPI(pcdev->bus_type))
		ccic_set_clock_mipi(pcdev);
	else
		ccic_set_clock_parallel(pcdev);

	return 0;
}

static int ccic_config_image(struct pxa910_camera_dev *pcdev)
{
	int ret = 0;

	int imgsz;
	unsigned int temp;
	struct v4l2_pix_format *fmt = &pcdev->pix_format;
	int widthy = 0, widthuv = 0;
	dev_err(pcdev->soc_host.v4l2_dev.dev,
		KERN_ERR " %s %d bytesperline %d height %d\n", __func__,
		__LINE__, fmt->bytesperline,
		fmt->sizeimage / fmt->bytesperline);
	if (fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
		imgsz =
		    ((fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK) |
		    (((fmt->bytesperline)
		      * 4 / 3) & IMGSZ_H_MASK);
	} else if (fmt->pixelformat == V4L2_PIX_FMT_JPEG) {
		imgsz =
		    (((fmt->sizeimage /
		       fmt->bytesperline) << IMGSZ_V_SHIFT) & IMGSZ_V_MASK) |
		    (fmt->bytesperline & IMGSZ_H_MASK);

	} else {
		imgsz =
		    ((fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK) |
		    (fmt->bytesperline & IMGSZ_H_MASK);
	}
	/* YPITCH just drops the last two bits */
	/*ccic_reg_write_mask(cam, REG_IMGPITCH, fmt->bytesperline,
	   IMGP_YP_MASK);
	 */
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		widthy = fmt->width * 2;
		widthuv = fmt->width * 2;
		break;
	case V4L2_PIX_FMT_RGB565:
		widthy = fmt->width * 2;
		widthuv = 0;
		break;
	case V4L2_PIX_FMT_JPEG:
		widthy = fmt->bytesperline;
		widthuv = fmt->bytesperline;
		break;
	case V4L2_PIX_FMT_YUV422P:
		widthy = fmt->width;
		widthuv = fmt->width / 2;
		break;
	case V4L2_PIX_FMT_YUV420:
		widthy = fmt->width;
		widthuv = fmt->width / 2;
		break;
	default:
		break;
	}

	ccic_reg_write(pcdev, REG_IMGPITCH, widthuv << 16 | widthy);
	ccic_reg_write(pcdev, REG_IMGSIZE, imgsz);
	ccic_reg_write(pcdev, REG_IMGOFFSET, 0x0);
	/*
	 * Tell the controller about the image format we are using.
	 */
	switch (pcdev->pix_format.pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
				    C0_DF_YUV | C0_YUV_PLANAR |
				    C0_YUVE_YVYU, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_YUV420:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
				    C0_DF_YUV | C0_YUV_420PL |
				    C0_YUVE_YVYU, C0_DF_MASK);
		break;

	case V4L2_PIX_FMT_YUYV:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
				    C0_DF_YUV | C0_YUV_PACKED |
				    C0_YUVE_YUYV, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_UYVY:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
				    C0_DF_YUV | C0_YUV_PACKED |
				    C0_YUVE_UYVY, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_JPEG:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
				    C0_DF_YUV | C0_YUV_PACKED |
				    C0_YUVE_YUYV, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_RGB444:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
				    C0_DF_RGB | C0_RGBF_444 |
				    C0_RGB4_XRGB, C0_DF_MASK);
		break;

	case V4L2_PIX_FMT_RGB565:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
				    C0_DF_RGB | C0_RGBF_565 |
				    C0_RGB5_BGGR, C0_DF_MASK);
		break;

	default:
		dev_dbg(pcdev->soc_host.v4l2_dev.dev, "Unknown format %x\n",
			pcdev->pix_format.pixelformat);
		break;
	}
/*
 * Make sure it knows we want to use hsync/vsync.
 */
	ccic_reg_write_mask(pcdev, REG_CTRL0, C0_SIF_HVSYNC, C0_SIFM_MASK);
/*
 * This field controls the generation of EOF(internal signal)
 */
	temp = ccic_reg_read(pcdev, REG_CTRL0);
	temp |=  CO_EOF_VSYNC | C0_VEDGE_CTRL;
	ccic_reg_write(pcdev, REG_CTRL0, temp);

	return ret;
}

static void ccic_ctlr_irq_enable(struct pxa910_camera_dev *pcdev)
{
	/*
	 * Clear any pending interrupts, since we do not
	 * expect to have I/O active prior to enabling.
	 */
	ccic_reg_write(pcdev, REG_IRQSTAT, FRAMEIRQS);
	ccic_reg_set_bit(pcdev, REG_IRQMASK, FRAMEIRQS);
}

static void ccic_ctlr_irq_disable(struct pxa910_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_IRQMASK, FRAMEIRQS);
}

/*
* Make the controller start grabbing images.  Everything must
* be set up before doing this.
*/
static void ccic_ctlr_start(struct pxa910_camera_dev *pcdev)
{
	/* set_bit performs a read, so no other barrier should be
	   needed here */
	ccic_reg_set_bit(pcdev, REG_CTRL0, C0_ENABLE);
}

static void ccic_ctlr_stop(struct pxa910_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_CTRL0, C0_ENABLE);
}

void ccic_ctlr_init(struct pxa910_camera_dev *pcdev)
{
	/*
	 * Make sure it's not powered down.
	 */
	ccic_reg_clear_bit(pcdev, REG_CTRL1, C1_PWRDWN);
	/*
	 * Turn off the enable bit.  It sure should be off anyway,
	 * but it's good to be sure.
	 */
	ccic_reg_clear_bit(pcdev, REG_CTRL0, C0_ENABLE);
	/*
	 * Mask all interrupts.
	 */
	ccic_reg_write(pcdev, REG_IRQMASK, 0);
	/*
	 * Clock the sensor appropriately. Controller clock should
	 * be 48MHz, sensor "typical" value is half that.
	 */
}

/*
* Stop the controller, and don't return
* until we're really sure that no further DMA is going on.
*/
static void ccic_ctlr_stop_dma(struct pxa910_camera_dev *pcdev)
{
	/*
	 * Theory: stop the camera controller (whether it is operating
	 * or not).  Delay briefly just in case we race with the SOF
	 * interrupt, then wait until no DMA is active.
	 */
	ccic_ctlr_stop(pcdev);
	/*CSI2/DPHY need to be cleared, or no EOF will be received */
	ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x0);

	/* 166 = 1/6, jpeg fps is 8, that is why we set timeout as 200 */
	ccic_ctlr_irq_disable(pcdev);
	mdelay(200);
}

/*
* Power up and down.
*/
void ccic_ctlr_power_up(struct pxa910_camera_dev *pcdev)
{
	/*
	 * Part one of the sensor dance: turn the global
	 * GPIO signal on.
	 */
	ccic_reg_clear_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

static void ccic_ctlr_power_down(struct pxa910_camera_dev *pcdev)
{
	ccic_reg_set_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

/*
* Get everything ready, and start grabbing frames.
*/
static void ccic_config_phy(struct pxa910_camera_dev *pcdev)
{
	if (BUS_IS_MIPI(pcdev->bus_type)) {
		/* TODO DPHY clock tunning */
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0a00);
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0a06);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x33);
		ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x43);
	} else {
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x0);
	}
}

static void dma_attach_buf(struct pxa910_camera_dev *pcdev)
{
	struct pxa910_buffer *buf = NULL;
	struct videobuf_buffer *vb = NULL;
	int frame = 0;
	unsigned long flags = 0;
	spin_lock_irqsave(&pcdev->list_lock, flags);

	while (!list_empty(&pcdev->capture) && (frame < 2)) {

		buf = list_entry(pcdev->capture.next,
				 struct pxa910_buffer, vb.queue);

		if (NULL == buf)
			BUG_ON(1);
		if (NULL == (void *)(buf->yuv_p.y))
			BUG_ON(1);
		ccic_reg_write(pcdev, REG_Y0BAR + (frame << 2), buf->yuv_p.y);
		ccic_reg_write(pcdev, REG_U0BAR + (frame << 2), buf->yuv_p.u);
		ccic_reg_write(pcdev, REG_V0BAR + (frame << 2), buf->yuv_p.v);
		frame++;
		vb = &buf->vb;
		if (NULL == vb)
			BUG_ON(1);
		list_move_tail(&vb->queue, &pcdev->sb_dma);
		vb->state = VIDEOBUF_QUEUED;
		dev_err(pcdev->soc_host.v4l2_dev.dev, "%s %d\n", __func__, frame);
	}
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
}

/**
* pxa910_camera_start_capture - start video capturing
* @pcdev: camera device*
*/
static void pxa910_camera_start_capture(struct pxa910_camera_dev
					*pcdev)
{
	dev_dbg(pcdev->soc_host.v4l2_dev.dev, "%s\n", __func__);
	ccic_config_phy(pcdev);
	ccic_ctlr_irq_enable(pcdev);
	ccic_ctlr_start(pcdev);
}

static void pxa910_camera_stop_capture(struct pxa910_camera_dev
				       *pcdev)
{
	ccic_ctlr_stop_dma(pcdev);
	dev_dbg(pcdev->soc_host.v4l2_dev.dev, "%s\n", __func__);
}

/*
*  Videobuf operations
*/
static int pxa910_videobuf_setup(struct videobuf_queue *vq,
				 unsigned int *count, unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						     icd->
						     current_fmt->host_fmt);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;

	if (bytes_per_line < 0)
		return bytes_per_line;

	dev_dbg(icd->dev.parent, "count=%d, size=%d\n", *count, *size);

	*size = bytes_per_line * icd->user_height;

	if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		*size = pcdev->pix_format.sizeimage;
		bytes_per_line = pcdev->pix_format.bytesperline;
	}
	/*
	   if (0 == *count)
	   *count = 32;
	   if (*size * *count > vid_limit * 1024 * 1024)
	   *count = (vid_limit * 1024 * 1024) / *size;
	 */
	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct pxa910_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct videobuf_buffer *vb = &buf->vb;
	struct videobuf_dma_contig_memory *mem = vb->priv;

	BUG_ON(in_interrupt());

	dev_dbg(icd->dev.parent, "%s (vb=0x%p) 0x%08lx %d\n",
		__func__, vb, vb->baddr, vb->bsize);

	/*
	 * This waits until this buffer is out of danger, i.e., until it is no
	 * longer in STATE_QUEUED or STATE_ACTIVE
	 */
	/* videobuf_waiton(vb, 0, 0); */
	/* videobuf_dma_contig_free(vq, vb); */
	/* relapce videobuf_dma_contig_free */

	if (vb->memory != V4L2_MEMORY_USERPTR)
		return;
	if (!mem)
		return;
	/* handle user space pointer case */
	if (vb->baddr) {
		/*videobuf_dma_contig_user_put(mem); */
		mem->is_userptr = 0;
		mem->dma_handle = 0;
		mem->size = 0;
	}

	vb->state = VIDEOBUF_NEEDS_INIT;
}

extern struct page *va_to_page(unsigned long user_addr);
extern unsigned long va_to_pa(unsigned long user_addr, unsigned int size);

static int pxa910_videobuf_prepare(struct videobuf_queue *vq,
				   struct videobuf_buffer *vb,
				   enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;
	struct pxa910_buffer *buf = container_of(vb, struct pxa910_buffer, vb);
	int ret;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						     icd->
						     current_fmt->host_fmt);

	if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_JPEG)
		bytes_per_line = pcdev->pix_format.bytesperline;

	if (bytes_per_line < 0)
		return bytes_per_line;

	if (vb->baddr & 4095) {
		dev_err(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
			vb, vb->baddr, vb->bsize);
		dev_err(dev, "buffer base addr is not 4096 bytes aligned\n");
		BUG_ON(1);
	}

	if (vb->size & 31) {
		dev_err(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
			vb, vb->baddr, vb->bsize);
		dev_err(dev, "buffer size is not 32 bytes aligned\n");
		BUG_ON(1);
	}

	/* TODO use videobuf_to_dma_contig */
	/* Fixed Me.
	 * videobuf_to_dma_contig failed with
	 * the first frame.
	 */
	buf->dma_handles = va_to_pa(vb->baddr, vb->size);
	if (!buf->dma_handles) {
		dev_err(dev, "faied to get phy addr %s %d\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -ENOMEM;
	}

	buf->page = va_to_page(vb->baddr);
	if (!buf->page) {
		dev_err(dev, "fail to get page %s %d info\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}

	if (PageHighMem(buf->page)) {
		dev_err(dev, "camera: HIGHMEM is not supported %s %d info\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}

	buf->buffer = page_address(buf->page);
	if (!buf->buffer) {
		dev_err(dev, "camera: fail to get buffer info %s %d\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}
	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d dma_addr =  0x%p\n",
		__func__, vb, vb->baddr, vb->bsize, (void *)buf->dma_handles);

	/* Added list head initialization on alloc */
	WARN_ON(!list_empty(&vb->queue));

	/*
	 * This can be useful if you want to see if we actually fill
	 * the buffer with something
	 */
	/*
	   memset((void *)vb->baddr, 0xaa, vb->bsize);
	 */
	BUG_ON(NULL == icd->current_fmt);

	/*
	 * I think, in buf_prepare you only have to protect global data,
	 * the actual buffer is yours
	 */

	if (buf->code != icd->current_fmt->code ||
	    vb->width != icd->user_width ||
	    vb->height != icd->user_height || vb->field != field) {
		buf->code = icd->current_fmt->code;
		vb->width = icd->user_width;
		vb->height = icd->user_height;
		vb->field = field;
		vb->state = VIDEOBUF_NEEDS_INIT;
	}
	if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_JPEG)
		vb->size = pcdev->pix_format.sizeimage;
	else
		vb->size = bytes_per_line * vb->height;
	if (0 != vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}

	/* specific for TD */

	if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_YUV422P) {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = buf->yuv_p.y + vb->width * vb->height;
		buf->yuv_p.v = buf->yuv_p.u + vb->width * vb->height / 2;
	} else if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_YUV420) {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = buf->yuv_p.y + vb->width * vb->height;
		buf->yuv_p.v = buf->yuv_p.u + vb->width * vb->height / 4;
	} else {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = pcdev->rubbish_buf_phy;
		buf->yuv_p.v = pcdev->rubbish_buf_phy;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;

fail:
	free_buffer(vq, buf);
out:
	return ret;
}

/* Called under spinlock_irqsave(&pcdev->lock, ...) */
static void pxa910_videobuf_queue(struct videobuf_queue *vq,
				  struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;

	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d dma_addr =  0x%p\n",
		__func__, vb, vb->baddr, vb->bsize,
		(void *)videobuf_to_dma_contig(vb));

	list_add_tail(&vb->queue, &pcdev->capture);

	vb->state = VIDEOBUF_ACTIVE;
	if (vq->streaming == 1) {
		if (pcdev->state == 0) {
			dma_attach_buf(pcdev);
			pxa910_camera_start_capture(pcdev);
			pcdev->state = 1;
		}
	}
}

static void pxa910_videobuf_release(struct videobuf_queue *vq,
				    struct videobuf_buffer *vb)
{
	struct pxa910_buffer *buf = container_of(vb, struct pxa910_buffer, vb);
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;
	struct device *dev = icd->dev.parent;
	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);
	/* fix me */
	vb->state = VIDEOBUF_DONE;
	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		dev_dbg(dev, "%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		dev_dbg(dev, "%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		dev_dbg(dev, "%s (prepared)\n", __func__);
		break;
	default:
		dev_dbg(dev, "%s (unknown)\n", __func__);
		break;
	}
	if (vq->streaming == 0) {
		if (pcdev->state == 1) {
			pxa910_camera_stop_capture(pcdev);
			pcdev->state = 0;
		}
		INIT_LIST_HEAD(&pcdev->capture);
		INIT_LIST_HEAD(&pcdev->sb_dma);
	}

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops pxa910_videobuf_ops = {
	.buf_setup = pxa910_videobuf_setup,
	.buf_prepare = pxa910_videobuf_prepare,
	.buf_queue = pxa910_videobuf_queue,
	.buf_release = pxa910_videobuf_release,
};

static void pxa910_camera_init_videobuf(struct videobuf_queue *q,
					struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;
	int ret = 0;

	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	/*
	 * We must pass NULL as dev pointer, then all pci_* dma operations
	 * transform to normal dma_* ones.
	 */
	videobuf_queue_dma_contig_init(q, &pxa910_videobuf_ops, NULL,
				       &pcdev->list_lock,
				       V4L2_BUF_TYPE_VIDEO_CAPTURE,
				       V4L2_FIELD_NONE,
				       sizeof(struct pxa910_buffer), icd);

	ret = v4l2_subdev_call(sd, core, load_fw);
	if (ret < 0)
		BUG_ON(1);
}

static void pxa910_camera_activate(struct pxa910_camera_dev *pcdev)
{
	ccic_enable_clk(pcdev);
}

static void pxa910_camera_deactivate(struct pxa910_camera_dev
				     *pcdev)
{
	pxa910_camera_stop_capture(pcdev);
	ccic_disable_clock(pcdev);
}

static inline void ccic_udpate_buf2dma(struct pxa910_camera_dev *pcdev, short frame)
{
	struct pxa910_buffer *buf = NULL;
	struct videobuf_buffer *vb = NULL;
	unsigned long flags = 0;
	dma_addr_t dma_base_reg = 0;
	struct pxa910_buffer *newbuf = NULL;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;

	spin_lock_irqsave(&pcdev->list_lock, flags);

	dma_base_reg = ccic_reg_read(pcdev, REG_Y0BAR + (frame << 2));

	if (pcdev->rubbish_buf_phy == dma_base_reg)
		goto update_buf2dma;
	else {

		list_for_each_entry(buf, &pcdev->sb_dma, vb.queue) {

			if (dma_base_reg == buf->yuv_p.y) {
				vb = &buf->vb;
				if (NULL == vb)
					BUG_ON(1);
				if (pcdev->pix_format.pixelformat ==
						V4L2_PIX_FMT_JPEG) {
					dma_map_page(dev, buf->page, 0,
							64, DMA_FROM_DEVICE);

					if (!(buf->buffer[0] == 0xff
								&& buf->buffer[1] == 0xd8)) {
						dev_err(pcdev->soc_host.
								v4l2_dev.dev,
								"DROP JPEG 0x%04x\n",
								(short)(buf->
									buffer[0]));
						goto out;
					}
				}
				/* delete it from captuer */
				list_del_init(&vb->queue);
				vb->state = VIDEOBUF_DONE;
				dma_map_page(dev, buf->page, 0,
						vb->bsize, DMA_FROM_DEVICE);
				wake_up(&vb->done);

				break;
			}

		}
update_buf2dma:

		/* update the DMA base address */

		if (list_empty(&pcdev->capture)) {
			dev_dbg(pcdev->soc_host.v4l2_dev.dev,
					"%s %d rubbish\n", __func__, __LINE__);
			ccic_reg_write(pcdev, REG_Y0BAR + (frame << 2),
					pcdev->rubbish_buf_phy);
			ccic_reg_write(pcdev, REG_U0BAR + (frame << 2),
					pcdev->rubbish_buf_phy);
			ccic_reg_write(pcdev, REG_V0BAR + (frame << 2),
					pcdev->rubbish_buf_phy);

		} else {

			newbuf =
				list_entry(pcdev->capture.next,
						struct pxa910_buffer, vb.queue);

			if (NULL == newbuf)
				BUG_ON(1);
			if (NULL == (void *)(newbuf->yuv_p.y))
				BUG_ON(1);

			ccic_reg_write(pcdev,
					REG_Y0BAR + (frame << 2),
					newbuf->yuv_p.y);
			ccic_reg_write(pcdev,
					REG_U0BAR + (frame << 2),
					newbuf->yuv_p.u);
			ccic_reg_write(pcdev,
					REG_V0BAR + (frame << 2),
					newbuf->yuv_p.v);
			vb = &newbuf->vb;
			if (NULL == vb)
				BUG_ON(1);
			list_move_tail(&vb->queue, &pcdev->sb_dma);
			vb->state = VIDEOBUF_QUEUED;
		}

	}
out:
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
}

/* #define IRQ_DEBUG */
#ifdef IRQ_DEBUG
static inline void ccic_irq_debug(struct pxa910_camera_dev *pcdev)
{
	int line_num = ccic_reg_read(pcdev, REG_LNNUM);
	dev_err(pcdev->soc_host.v4l2_dev.dev, " line_num = %d\n", line_num);
}
#endif

static irqreturn_t pxa910_camera_irq(int irq, void *data)
{
	struct pxa910_camera_dev *pcdev = data;
	/* do something here */

	unsigned int irqs;
	short frame = 0x0f;
	irqs = ccic_reg_read(pcdev, REG_IRQSTAT);
	ccic_reg_write(pcdev, REG_IRQSTAT, irqs);

#ifdef IRQ_DEBUG
	dev_err(pcdev->soc_host.v4l2_dev.dev, " irqs = 0x%08x\n", irqs);
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
		ccic_irq_debug(pcdev);
#endif
		ccic_udpate_buf2dma(pcdev, frame);
	}

	return IRQ_HANDLED;
}

/*
* The following two functions absolutely depend on the fact, that
* there can be only one camera on PXA quick capture interface
* Called with .video_lock held
*/
static int pxa910_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;

	if (pcdev->icd)
		return -EBUSY;

#ifdef CONFIG_PM
	camera_pin_restore();
#endif
	set_power_constraint();
	wake_lock(&idle_lock);

	pcdev->icd = icd;
	pxa910_camera_activate(pcdev);

	return 0;
}

/* Called with .video_lock held */
static void pxa910_camera_remove_device(struct soc_camera_device
					*icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	pxa910_camera_deactivate(pcdev);
	pcdev->icd = NULL;

	wake_unlock(&idle_lock);
	unset_power_constraint();
#ifdef CONFIG_PM
	camera_pin_power_opt();
#endif
}

static int pxa910_camera_set_bus_param(struct soc_camera_device
				       *icd, __u32 pixfmt)
{
	struct device *dev = icd->dev.parent;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;

	int ret = 0;
	unsigned long common_flags = 0;

	common_flags = icd->ops->query_bus_param(icd);
	if (SOCAM_MIPI & common_flags)
		pcdev->bus_type = BUS_MIPI;
	else
		pcdev->bus_type = BUS_PARALLEL;

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}
	return 0;
}

static int pxa910_camera_set_fmt(struct soc_camera_device *icd,
				 struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;
	struct device *dev = icd->dev.parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_mbus_framefmt mf;
	int ret = 0;

	struct v4l2_pix_format *pix = &f->fmt.pix;
	memcpy(&(pcdev->pix_format), pix, sizeof(struct v4l2_pix_format));
	dev_err(dev, "S_FMT %c%c%c%c, %ux%u\n",
		pixfmtstr(pix->pixelformat), pix->width, pix->height);
	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_err(dev, "Format %c%c%c%c not found\n",
			pixfmtstr(pix->pixelformat));
		return -EINVAL;
	}

	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = pix->field;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}
	if (mf.code != xlate->code) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	icd->sense = NULL;

	pix->width = mf.width;
	pix->height = mf.height;
	pix->field = mf.field;
	pix->colorspace = mf.colorspace;
	icd->current_fmt = xlate;

	ret = ccic_config_image(pcdev);

	return ret;
}

static int pxa910_camera_try_fmt(struct soc_camera_device *icd,
				 struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_err(dev, "Format %c%c%c%c not found\n",
			pixfmtstr(pix->pixelformat));
		return -EINVAL;
	}

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
							xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;
	if(pix->pixelformat == V4L2_PIX_FMT_JPEG)
		pix->bytesperline = 2048;
	else
		pix->sizeimage = pix->height * pix->bytesperline;

	/* limit to sensor capabilities */
	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code 	= xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height = mf.height;
	pix->colorspace = mf.colorspace;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field	= V4L2_FIELD_NONE;
		break;
	default:
		dev_err(icd->dev.parent, "Field type %d unsupported.\n",
			mf.field);
		return -EINVAL;
	}

	return ret;

}

static int pxa910_camera_reqbufs(struct soc_camera_file *icf,
				 struct v4l2_requestbuffers *p)
{
	int i;
	for (i = 0; i < p->count; i++) {
		struct pxa910_buffer *buf = container_of(icf->vb_vidq.bufs[i],
							 struct pxa910_buffer,
							 vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}
	return 0;
}

static unsigned int pxa910_camera_poll(struct file *file, poll_table * pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct pxa910_buffer *buf;

	buf =
	    list_entry(icf->vb_vidq.stream.next,
		       struct pxa910_buffer, vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE || buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int pxa910_camera_querycap(struct soc_camera_host *ici,
				  struct v4l2_capability *cap)
{
	struct v4l2_dbg_chip_ident id;
	struct pxa910_camera_dev *pcdev = ici->priv;
	struct soc_camera_device *icd = pcdev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;

	int ret = 0;
	cap->version = PXA_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	ret = v4l2_subdev_call(sd, core, g_chip_ident, &id);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}

	strcpy(cap->card, "TD/TTC");

	if (id.ident == V4L2_IDENT_OV5642)
		strcpy(cap->driver, "ov5642");
	else
		strcpy(cap->driver, "unknow sensor");
	return 0;
}

static int pxa910_camera_suspend(struct soc_camera_device *icd,
				 pm_message_t state)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;
	int ret = 0;

	if ((pcdev->icd) && (pcdev->icd->ops->suspend))
		ret = pcdev->icd->ops->suspend(pcdev->icd, state);

	return ret;
}

static int pxa910_camera_resume(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa910_camera_dev *pcdev = ici->priv;
	int ret = 0;

	if ((pcdev->icd) && (pcdev->icd->ops->resume))
		ret = pcdev->icd->ops->resume(pcdev->icd);

	return ret;
}
static int pxa910_camera_set_parm(struct soc_camera_device *icd,
                                 struct v4l2_streamparm *para)
{
       return 0;
}

static const struct soc_mbus_pixelfmt ccic_formats[] = {
	{
	 .fourcc = V4L2_PIX_FMT_YUV422P,
	 .name = "YUV422PLANAR",
	 .bits_per_sample = 8,
	 .packing = SOC_MBUS_PACKING_2X8_PADLO,
	 .order = SOC_MBUS_ORDER_LE,
	 },
	{
	 .fourcc = V4L2_PIX_FMT_YUV420,
	 .name = "YUV420PLANAR",
	 .bits_per_sample = 12,
	 .packing = SOC_MBUS_PACKING_NONE,
	 .order = SOC_MBUS_ORDER_LE,
	 },
	{
	 .fourcc = V4L2_PIX_FMT_UYVY,
	 .name = "YUV422PACKED",
	 .bits_per_sample = 8,
	 .packing = SOC_MBUS_PACKING_2X8_PADLO,
	 .order = SOC_MBUS_ORDER_LE,
	 },

};

static int pxa910_camera_get_formats(struct soc_camera_device *icd,
				     unsigned int idx,
				     struct soc_camera_format_xlate
				     *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	int formats = 0, ret;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;
	int i = 0;
	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_err(dev, "Invalid format code #%u: %d\n", idx, code);
		return 0;
	}

	switch (code) {
		/* refer to mbus_fmt struct */
	case V4L2_MBUS_FMT_YUYV8_2X8_BE:
		/* TODO: add support for YUV420 and YUV422P */
		formats = ARRAY_SIZE(ccic_formats);

		if (xlate) {
			for (i = 0; i < ARRAY_SIZE(ccic_formats); i++) {
				xlate->host_fmt = &ccic_formats[i];
				xlate->code = code;
				xlate++;
			}
		}

		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		if (xlate)
			dev_err(dev, "Providing format %s\n", fmt->name);
		break;
	default:
		/* camera controller can not support
		   this format, which might supported by the sensor
		 */
		dev_err(dev, "Not support fmt %s\n", fmt->name);
		return 0;
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code	= code;
		xlate++;
	}

	return formats;
}

static void pxa910_camera_put_formats(struct soc_camera_device *icd)
{
	kfree(icd->host_priv);
	icd->host_priv = NULL;
}

static int pxa910_camera_enum_fsizes(struct soc_camera_device *icd,
					struct v4l2_frmsizeenum *fsizes)
{
	int ret;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	__u32 pixfmt = fsizes->pixel_format;
	struct v4l2_frmsizeenum *fsize_mbus = fsizes;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate)
		return -EINVAL;

	/* map xlate-code to pixel_format, sensor only handle xlate-code*/
	fsize_mbus->pixel_format = xlate->code;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fsizes, fsize_mbus);
	if (ret < 0)
		return ret;

	fsizes->pixel_format = pixfmt;

	return 0;
}

static struct soc_camera_host_ops pxa_soc_camera_host_ops = {
	.owner = THIS_MODULE,
	.add = pxa910_camera_add_device,
	.remove = pxa910_camera_remove_device,
	.suspend = pxa910_camera_suspend,
	.resume = pxa910_camera_resume,
	.set_fmt = pxa910_camera_set_fmt,
	.try_fmt = pxa910_camera_try_fmt,
	.init_videobuf = pxa910_camera_init_videobuf,
	.reqbufs = pxa910_camera_reqbufs,
	.poll = pxa910_camera_poll,
	.querycap = pxa910_camera_querycap,
	.set_bus_param = pxa910_camera_set_bus_param,
	.set_parm = pxa910_camera_set_parm,
	.get_formats = pxa910_camera_get_formats,
	.put_formats = pxa910_camera_put_formats,
	.enum_fsizes = pxa910_camera_enum_fsizes,
};

/* Power manament */
static void ccic_sleep_early_suspend(struct early_suspend *h)
{
	struct pxa910_camera_dev *pcdev = container_of(h, \
                        struct pxa910_camera_dev, ccic_early_suspend);
	clk_disable(pcdev->lcd_clk);
}

static void ccic_normal_late_resume(struct early_suspend *h)
{
	struct pxa910_camera_dev *pcdev = container_of(h, \
                        struct pxa910_camera_dev, ccic_early_suspend);
	clk_enable(pcdev->lcd_clk);
}

static int __devinit pxa910_camera_probe(struct platform_device
					 *pdev)
{
	struct pxa910_camera_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	int irq;
	int err = 0;
	int i = 0;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq < 0) {
		err = -ENODEV;
		goto exit;
	}
	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	/* allocate rubbish buffer */
	pcdev->rubbish_buf_virt =
	    (void *)__get_free_pages(GFP_KERNEL, get_order(dma_buf_size));
	if (!pcdev->rubbish_buf_virt) {
		dev_err(&pdev->dev, "Can't get memory for rubbish buffer\n");
		err = -ENOMEM;
		goto exit_kfree;
	} else {
		pcdev->rubbish_buf_phy = __pa(pcdev->rubbish_buf_virt);
	}
	pcdev->rst_clk = clk_get(&pdev->dev, "CCICRSTCLK");
	if (IS_ERR(pcdev->rst_clk)) {
		dev_err(&pdev->dev, "Could not get rstclk\n");
		err = PTR_ERR(pcdev->rst_clk);
		goto exit_clk;
	}

	pcdev->gateclk = clk_get(&pdev->dev, "CCICGATECLK");
	if (IS_ERR(pcdev->gateclk)) {
		dev_err(&pdev->dev, "Could not get gateclk\n");
		err = PTR_ERR(pcdev->gateclk);
		goto exit_clk;
	}

	pcdev->lcd_clk = clk_get(NULL, "LCDCLK");
	if (IS_ERR(pcdev->lcd_clk)) {
		dev_err(&pdev->dev, "unable to get LCDCLK");
		goto exit_clk;
	}
	clk_enable(pcdev->lcd_clk);

	pcdev->res = res;

	pcdev->pdata = pdev->dev.platform_data;

	INIT_LIST_HEAD(&pcdev->capture);
	INIT_LIST_HEAD(&pcdev->sb_dma);

	spin_lock_init(&pcdev->list_lock);

	/*
	 * Request the regions.
	 */
	if (!request_mem_region(res->start, resource_size(res),
				PXA910_CAM_DRV_NAME)) {
		err = -EBUSY;
		dev_err(&pdev->dev, "request_mem_region resource failed\n");
		goto exit_release;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "ioremap resource failed\n");
		goto exit_iounmap;
	}
	pcdev->irq = irq;
	pcdev->base = base;
	/* request irq */
	err = request_irq(pcdev->irq, pxa910_camera_irq, 0,
			  PXA910_CAM_DRV_NAME, pcdev);
	if (err) {
		dev_err(&pdev->dev, "Camera interrupt register failed\n");
		goto exit_free_irq;
	}

	/* setup dma with rubbish_buf_phy firstly */
	for (i = 0; i < 3; i++) {
		ccic_reg_write(pcdev, REG_Y0BAR + (i << 2),
			       pcdev->rubbish_buf_phy);
		ccic_reg_write(pcdev, REG_U0BAR + (i << 2),
			       pcdev->rubbish_buf_phy);
		ccic_reg_write(pcdev, REG_V0BAR + (i << 2),
			       pcdev->rubbish_buf_phy);
	}

	pxa910_camera_activate(pcdev);
	/*
	 * Initialize the controller and leave it powered up.  It will
	 * stay that way until the sensor driver shows up.
	 */
	ccic_ctlr_init(pcdev);
	ccic_ctlr_power_up(pcdev);
#ifdef CONFIG_PM
	camera_pin_power_opt();
#endif

	pcdev->ccic_early_suspend.level =
	    EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	    pcdev->ccic_early_suspend.suspend = ccic_sleep_early_suspend;
	pcdev->ccic_early_suspend.resume = ccic_normal_late_resume;
	register_early_suspend(&pcdev->ccic_early_suspend);

	pcdev->soc_host.drv_name = PXA910_CAM_DRV_NAME;
	pcdev->soc_host.ops = &pxa_soc_camera_host_ops;
	pcdev->soc_host.priv = pcdev;
	pcdev->soc_host.v4l2_dev.dev = &pdev->dev;
	pcdev->soc_host.nr = pdev->id;
	err = soc_camera_host_register(&pcdev->soc_host);
	if (err)
		goto exit_free_irq;

	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
	ccic_ctlr_power_down(pcdev);
exit_iounmap:
	iounmap(base);
exit_release:
	release_mem_region(res->start, resource_size(res));
exit_clk:
	clk_put(pcdev->rst_clk);
	clk_put(pcdev->gateclk);
	clk_put(pcdev->lcd_clk);
exit_kfree:
	/* free rubbish buffer */
	if (pcdev->rubbish_buf_virt)
		free_pages((unsigned long)pcdev->rubbish_buf_virt,
			   get_order(dma_buf_size));
	kfree(pcdev);
exit:
	return err;
}

static int pxa910_camera_remove(struct platform_device
				*pdev)
{

	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct pxa910_camera_dev *pcdev = container_of(soc_host,
						       struct
						       pxa910_camera_dev,
						       soc_host);
	struct resource *res;

	clk_put(pcdev->rst_clk);
	clk_put(pcdev->lcd_clk);
	clk_put(pcdev->gateclk);
	free_irq(pcdev->irq, pcdev);

	soc_camera_host_unregister(soc_host);

	iounmap(pcdev->base);

	res = pcdev->res;
	release_mem_region(res->start, resource_size(res));

	kfree(pcdev);

	/* free rubbish buffer */
	if (pcdev->rubbish_buf_virt)
		free_pages((unsigned long)pcdev->rubbish_buf_virt,
			   get_order(dma_buf_size));

	dev_info(&pdev->dev, "PXA Camera driver unloaded\n");

	return 0;
}

static struct platform_driver pxa910_camera_driver = {
	.driver = {
		   .name = PXA910_CAM_DRV_NAME,
		   },
	.probe = pxa910_camera_probe,
	.remove = pxa910_camera_remove,
};

static int __init pxa910_camera_init(void)
{
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

MODULE_DESCRIPTION("PXA910 SoC Camera Host driver");
MODULE_AUTHOR("Kassey Lee <ygli@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" PXA910_CAM_DRV_NAME);
