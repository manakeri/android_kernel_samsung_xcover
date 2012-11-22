/*----------------------------------------------------------

* V4L2 Driver for PXA95x camera host
* SCI -- Simple Camera Interface
* CSI -- CSI-2 controller, refer to MIPI CSI-2 protocol
*
* Based on linux/drivers/media/video/pxa_camera.c
*
* Copyright (C) 2010, Marvell International Ltd.
*              Qing Xu <qingx@marvell.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.

----------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/clk.h>

#include <mach/dma.h>
#include <mach/camera.h>
#include <mach/pxa3xx-regs.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-contig.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-chip-ident.h>

MODULE_AUTHOR("Qing Xu <qingx@marvell.com>");
MODULE_DESCRIPTION("Marvell PXA955 Simple Capture Controller Driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("Video");

/* CSI register base: 0x50020000*/
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

/* CSxCR0 bits */
#define CSI_CONT_CSI_EN            (0x1u<<0)  /* CSI controller Enable/Disable*/
#define CSI_CONT_NOL(n)            ((n)<<2)   /* Number of Active Lanes (2 bits)*/
#define CSI_CONT_VC0_CFG(n)        ((n)<<5)   /* Chan 0 Addr Conf (2 bits)*/
#define CSI_CONT_VC1_CFG(n)        ((n)<<7)   /* Chan 1 Addr Conf (2 bits)*/
#define CSI_CONT_VC2_CFG(n)        ((n)<<9)   /* Chan 2 Addr Conf (2 bits)*/
#define CSI_CONT_VC3_CFG(n)        ((n)<<11)  /* Chan 3 Addr Conf (2 bits)*/

/* CSxTIM0 bits*/
#define CSI_CONT_CLTERMEN(n)       ((n)<<0)   /* Time to wait before enabling clock HS termination (8 bits)*/
#define CSI_CONT_CLSETTLE(n)       ((n)<<8)   /* Time to wait before HS clock is valid (8 bits)*/
#define CSI_CONT_CLMISS(n)         ((n)<<16)  /* Time to detect that the clock has stopped toggling (8 bits)*/
#define CSI_CONT_HSTERMEN(n)       ((n)<<24)  /* Time to wait before enabling data HS termination (8 bits)*/

/* CSxTIM1 bits */
#define CSI_CONT_HS_Rx_TO(n)       ((n)<<0)   /* Time to wait before declaring an error on a packet reception. This counter counts on the escape mode clock which is 52 MHz (16 bits)*/
#define CSI_CONT_HSTSETTLE(n)      ((n)<<16)  /* Timeout at RX end to neglect transition effects (8 bits)*/

/* CSxPHYCAL bits */
#define CSI_CONT_MIPI_BG_VREF_EN   (0x1u<<0)  /* See DPHY specifications for details*/
#define CSI_CONT_MIPI_RCOMP_CLKSEL (0x1u<<1)  /* Used to control the source of the PHY calibration clock (External/Internal).*/
#define CSI_CONT_MIPI_RCOMP_LOAD   (0x1u<<2)  /* Used to enable loading the MIPI_REN bypass value*/
#define CSI_CONT_MIPI_REN_BYPASS(n) ((n)<<3)  /* An 8-bit value to bypass the internal MIPI_REN value.*/
#define CSI_CONT_MIPI_RESET        (0x1u<<10) /* Mipi Reset Bit for pxa955 instead of 8th bit of REN_BYPASS*/
#define CSI_CONT_MIPI_RCOMP_CLK_EXT (0x1u<<31) /* Used tp provide a SW generated extternal clock to the PHY calibration block.*/

/* CSGCR bits */
#define CSI_CLK_DIV(n)             ((n)<<0)  /* Generated Clock Divisor. GCLK = 1/(DIV+2). 12 bits*/
#define CSI_CLK_GCLK_EN            (0x1u<<16)/* Generated Clock Enable*/

/* CSxINEN bits */
#define CSI_CONT_CSI_EN_INT        (0x1u<<0)  /* ENABLE Interrupt Enable*/
#define CSI_CONT_SoF_INT_EN        (0x1u<<1)  /* Start of Frame Interrupt Enable*/
#define CSI_CONT_EoF_INT_EN        (0x1u<<2)  /* End of Frame Interrupt Enable*/
#define CSI_CONT_OFLOW_INT_EN      (0x1u<<3)  /* Overflow Interrupt Enable*/
#define CSI_CONT_PHY_ERR_EN        (0x1u<<4)  /* D-PHY-related Error Interrupt Enable*/
#define CSI_CONT_GEN_PACK_INT_EN   (0x1u<<5)  /* Generic Packet Interrupt Enable*/
#define CSI_CONT_TIMEOUT_EN        (0x1u<<8)  /* Timeout Interrupt Enable*/
#define CSI_CONT_PROT_EN           (0x1u<<12) /* Protection Interupt Enable*/

#define ACSR_ALUF_MASK 0x00600000L
#define ACSR_ALUF_OFFSET 21

/* sci register, sci0 base: 0x50000000*/
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

#define PXA955_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5)
#define PXA955_CAM_DRV_NAME "pxa95x-camera"

#define CCIC_0 0	/* for 2 camera controller*/
#define CCIC_1 1
#define CCIC_MAX 2

#define JPEG_COMPRESS_RATIO_HIGH 10
#define CHANNEL_NUM 3 		/*YUV*/
#define MAX_DMA_BUFS 4
#define MIN_DMA_BUFS 2
#define MAX_DMA_SIZE (1280 * 720 * 2)
#define JPEG_BUF_SIZE (1024 * 1024)

static unsigned int vid_mem_limit = 16;	/* Video memory limit, in Mb */
static unsigned int skip_frame = 0;

typedef enum
{
	CAM_PCLK_104=0,
	CAM_PCLK_156,
	CAM_PCLK_208,
	CAM_PCLK_78,
}CAM_PCLK_t;

struct csi_phy_config {
	unsigned int clk_termen;
	unsigned int clk_settle;
	unsigned int hs_termen;
	unsigned int hs_settle;
	unsigned int hs_rx_to;
};

/* descriptor needed for the camera controller DMA engine */
struct pxa_cam_dma {
	dma_addr_t	sg_dma;
	struct pxa_dma_desc	*sg_cpu;
	size_t	sg_size;
	int	sglen;
};

struct pxa_buf_node {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;
	enum v4l2_mbus_pixelcode	code;
	struct page *page;
	void *ptr_vaddr;/* vritual addr of user ptr buffer*/
	void *mmp_vaddr;/* vritual addr of mmap buffer*/
	struct pxa_cam_dma dma_desc[CHANNEL_NUM];
};

struct pxa955_cam_dev {
	struct soc_camera_host	soc_host;
	struct soc_camera_device *icd;
	struct list_head dev_list;	/* link to other devices */

	struct clk *sci1_clk;
	struct clk *sci2_clk;
	struct pxa95x_csi_dev *csidev;

	unsigned int		irq;
	void __iomem		*regs;
	wait_queue_head_t iowait;	/* waiting on frame data */
	bool streamon;
	bool streamoff;

	struct resource		*res;
	unsigned long		platform_flags;

	/* DMA buffers */
	struct list_head dma_buf_list;	/*dma buffer list, the list member is buf_node*/
	unsigned int dma_buf_size;		/* allocated size */
	unsigned int channels;
	unsigned int channel_size[CHANNEL_NUM];

	/* streaming buffers */
	spinlock_t spin_lock;  /* Access to device */
	struct videobuf_queue *videoq;
	struct pxa95x_cam_pdata *pdata;
};

/* refer to videobuf-dma-contig.c*/
struct videobuf_dma_contig_memory {
	u32 magic;
	void *vaddr;
	dma_addr_t dma_handle;
	unsigned long size;
	int is_userptr;
};

#define MAGIC_DC_MEM 0x0733ac61
#define MAGIC_CHECK(is, should)					    \
	if (unlikely((is) != (should)))	{				    \
		pr_err("magic mismatch: %x expected %x\n", (is), (should)); \
		BUG();							    \
	}

/* Len should 8 bytes align, bit[2:0] should be 0 */
#define SINGLE_DESC_TRANS_MAX   (1 << 24)

static inline void csi_reg_write(struct pxa95x_csi_dev *csi, unsigned int reg,
		unsigned int val)
{
	__raw_writel(val, csi->regs + reg);
}

static inline unsigned int csi_reg_read(struct pxa95x_csi_dev *csi,
		unsigned int reg)
{
	return __raw_readl(csi->regs + reg);
}

void csi_reg_dump(struct pxa95x_csi_dev *csi)
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

void csi_cken(struct pxa95x_csi_dev *csi, int flag)
{
	if (flag) {
		/* The order of enabling matters! AXI must be the 1st one */
		clk_enable(csi->axi_clk);
		clk_enable(csi->csi_tx_esc);
	} else {
		clk_disable(csi->csi_tx_esc);
		clk_disable(csi->axi_clk);
	};
}

void csi_clkdiv(struct pxa95x_csi_dev *csi)
{
	unsigned int val = 0;
	unsigned int clk = 0;

	/* using default sys video clock */
	clk = (ACSR & ACSR_ALUF_MASK) >> ACSR_ALUF_OFFSET;

	val = csi_reg_read(csi, REG_CSGCR);
	val &= 0xfffff000;/*bit 11:0 clk divider*/

	switch(clk)
	{
		case CAM_PCLK_104:
			/* divide clock by 4. 26Mhz.*/
			val = CSI_CLK_DIV(0x3) | CSI_CLK_GCLK_EN;
			break;

		case CAM_PCLK_156:
			/* divide clock by 6. 26Mhz.*/
			val = CSI_CLK_DIV(0x5) | CSI_CLK_GCLK_EN;
			break;

		case CAM_PCLK_208:
			/* divide clock by 8. 26Mhz.*/
			val = CSI_CLK_DIV(0x3) | CSI_CLK_GCLK_EN;/* in test result, the devider should be 3, no 7*/
			break;

		default:
			/* divide clock by 8. 26Mhz.*/
			val = CSI_CLK_DIV(0x7) | CSI_CLK_GCLK_EN;
			break;
	}
	csi_reg_write(csi, REG_CSGCR, val);
}

void csi_lane(struct pxa95x_csi_dev *csi, unsigned int lane)
{
	unsigned int val = 0;
	switch (lane) {
		case 1:
		/* 1 lane */
		val = CSI_CONT_NOL(0x00) |
			  CSI_CONT_VC0_CFG(0x0) |
			  CSI_CONT_VC1_CFG(0x1) |
			  CSI_CONT_VC2_CFG(0x2) |
			  CSI_CONT_VC3_CFG(0x3);
		break;

		case 2:
		/*
		* packet data with 00 goto channel 0, packet data with 01 goto
		* channel 1, channel 2 and channel 3 are reserved for future use
		*/
		val = CSI_CONT_NOL(0x01) |	/* 2 lanes */
			  CSI_CONT_VC0_CFG(0x0) |
			  CSI_CONT_VC1_CFG(0x1);
		break;
	}

	csi_reg_write(csi, REG_CSxCR0, val);
}

void csi_dphy(struct pxa95x_csi_dev *csi, struct csi_phy_config timing)
{
	unsigned int val = 0;
	unsigned int *calibration_p;

	/*
	* default: csi controller channel 0 data route to camera interface 0,
	* channel 1 data route to camera interface 1.
	*/
	val = 0x01;
	csi_reg_write(csi, REG_CSSCR, val);

	val = CSI_CONT_CLTERMEN(0x00) |
		  CSI_CONT_CLSETTLE(0x0C) |
		  CSI_CONT_CLMISS(0x00)   |
		  CSI_CONT_HSTERMEN(0x04);
	csi_reg_write(csi, REG_CSxTIM0, val);

	/* set time out before declarating an error to maximum */
#if defined(CONFIG_VIDEO_OV7690_BRIDGE)
	val = CSI_CONT_HS_Rx_TO(0xffff) |
		  /* CAM_CSI_CONT_HSTSETTLE(0x0e); *//* for ov3640 MIPI timming */
		  CSI_CONT_HSTSETTLE(0x40);/* for ov5642 MIPI timing, 0x40 is proper for OV7690 bridged via OV5642*/
#else
	val = CSI_CONT_HS_Rx_TO(0xffff) |
		  /* CAM_CSI_CONT_HSTSETTLE(0x0e); *//* for ov3640 MIPI timming */
		  CSI_CONT_HSTSETTLE(0x48); /* for ov5642 MIPI timing, both work on MG1-JIL and PV2*/
#endif
	csi_reg_write(csi, REG_CSxTIM1, val);

    calibration_p = ioremap_nocache(0x42404078, 4);
    if(calibration_p != NULL){
            val = *calibration_p | (5<<8);
            val &= ~(1<<9);
            *calibration_p = val;
            iounmap(calibration_p);
    }
	val = CSI_CONT_MIPI_RESET;
	val |= CSI_CONT_MIPI_REN_BYPASS(0x0) |
		CSI_CONT_MIPI_BG_VREF_EN;
	csi_reg_write(csi, REG_CSxPHYCAL, val);

}

void csi_enable(struct pxa95x_csi_dev *csi, int idx)
{
	unsigned int val = 0;

	/*
	* we assume sensor send all its data with packet 00, we have config
	* packet 00 goto channel 0 in csi_config()
	*/
	switch(idx)
	{
		case CCIC_1:/* channel 0 data goto camera interface 1*/
			/* MG2 has a different defination of
			 * CSSCR[SSEL0,SSEL1] with MG1 */
			if (cpu_is_pxa955_Cx() || cpu_is_pxa955_Dx())
				val = 0x10;
			else
				val = 0x01;
			csi_reg_write(csi, REG_CSSCR, val);
			break;

		case CCIC_0:/* channel 1 data goto camera interface 0*/
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
		csi_reg_write(csi, REG_CSxDTINTLV, val);
	}
}

void csi_disable(struct pxa95x_csi_dev *csi)
{
	unsigned int val = csi_reg_read(csi, REG_CSxCR0);

	val &= ~CSxCR0_CSIEN;
	csi_reg_write(csi, REG_CSxCR0, val);
}

static inline void sci_reg_write(struct pxa955_cam_dev *pcdev, unsigned int reg,
		unsigned int val)
{
	__raw_writel(val, pcdev->regs + reg);
}

static inline unsigned int sci_reg_read(struct pxa955_cam_dev *pcdev,
		unsigned int reg)
{
	return __raw_readl(pcdev->regs + reg);
}


static inline void sci_reg_write_mask(struct pxa955_cam_dev *pcdev, unsigned int reg,
		unsigned int val, unsigned int mask)
{
	unsigned int v = sci_reg_read(pcdev, reg);

	v = (v & ~mask) | (val & mask);
	sci_reg_write(pcdev, reg, v);
}

static inline void sci_reg_clear_bit(struct pxa955_cam_dev *pcdev,
		unsigned int reg, unsigned int val)
{
	sci_reg_write_mask(pcdev, reg, 0, val);
}

static inline void sci_reg_set_bit(struct pxa955_cam_dev *pcdev,
		unsigned int reg, unsigned int val)
{
	sci_reg_write_mask(pcdev, reg, val, val);
}

#ifdef DEBUG
static void sci_dump_registers(struct pxa955_cam_dev *pcdev)
{
	printk(KERN_ERR "SCICR0 0x%x\n", sci_reg_read(pcdev, REG_SCICR0));
	printk(KERN_ERR "SCICR1 0x%x\n", sci_reg_read(pcdev, REG_SCICR1));
	printk(KERN_ERR "SCISR 0x%x\n", sci_reg_read(pcdev, REG_SCISR));
	printk(KERN_ERR "SCIMASK 0x%x\n", sci_reg_read(pcdev, REG_SCIMASK));
	printk(KERN_ERR "SCIFIFO 0x%x\n", sci_reg_read(pcdev, REG_SCIFIFO));
	printk(KERN_ERR "SCIFIFOSR 0x%x\n", sci_reg_read(pcdev, REG_SCIFIFOSR));

	printk(KERN_ERR "SCIDADDR0 0x%x\n", sci_reg_read(pcdev, REG_SCIDADDR0));
	printk(KERN_ERR "SCISADDR0 0x%x\n", sci_reg_read(pcdev, REG_SCISADDR0));
	printk(KERN_ERR "SCITADDR0 0x%x\n", sci_reg_read(pcdev, REG_SCITADDR0));
	printk(KERN_ERR "SCIDCMD0 0x%x\n", sci_reg_read(pcdev, REG_SCIDCMD0));

	printk(KERN_ERR "SCIDADDR1 0x%x\n", sci_reg_read(pcdev, REG_SCIDADDR1));
	printk(KERN_ERR "SCISADDR1 0x%x\n", sci_reg_read(pcdev, REG_SCISADDR1));
	printk(KERN_ERR "SCITADDR1 0x%x\n", sci_reg_read(pcdev, REG_SCITADDR1));
	printk(KERN_ERR "SCIDCMD1 0x%x\n", sci_reg_read(pcdev, REG_SCIDCMD1));

	printk(KERN_ERR "SCIDADDR2 0x%x\n", sci_reg_read(pcdev, REG_SCIDADDR2));
	printk(KERN_ERR "SCISADDR2 0x%x\n", sci_reg_read(pcdev, REG_SCISADDR2));
	printk(KERN_ERR "SCITADDR2 0x%x\n", sci_reg_read(pcdev, REG_SCITADDR2));
	printk(KERN_ERR "SCIDCMD2 0x%x\n", sci_reg_read(pcdev, REG_SCIDCMD2));

	printk(KERN_ERR "SCIDBR0 0x%x\n", sci_reg_read(pcdev, REG_SCIDBR0));
	printk(KERN_ERR "SCIDCSR0 0x%x\n", sci_reg_read(pcdev, REG_SCIDCSR0));

	printk(KERN_ERR "SCIDBR1 0x%x\n", sci_reg_read(pcdev, REG_SCIDBR1));
	printk(KERN_ERR "SCIDCSR1 0x%x\n", sci_reg_read(pcdev, REG_SCIDCSR1));

	printk(KERN_ERR "SCIDBR2 0x%x\n", sci_reg_read(pcdev, REG_SCIDBR2));
	printk(KERN_ERR "SCIDCSR2 0x%x\n", sci_reg_read(pcdev, REG_SCIDCSR2));
}

static void dma_dump_desc(struct pxa955_cam_dev *pcdev)
{
	int i, k;
	struct pxa_buf_node *buf_node;
	printk(KERN_ERR "cam: dump_dma_desc ************+\n");

	printk(KERN_ERR "dma_buf_list head 0x%x\n",(unsigned int)&pcdev->dma_buf_list);
	list_for_each_entry(buf_node, &pcdev->dma_buf_list, vb.queue)  {
			printk(KERN_ERR "buf_node 0x%x\n",(unsigned int)buf_node);

		for (i = 0; i < pcdev->channels; i++){
			printk(KERN_ERR "chnnl %d, chnnl_size %d, sglen %d, sgdma 0x%x, sgsze %d\n",
				i,pcdev->channel_size[i],buf_node->dma_desc[i].sglen,buf_node->dma_desc[i].sg_dma,buf_node->dma_desc[i].sg_size);
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

static void dma_dump_buf_list(struct pxa955_cam_dev *pcdev)
{
	struct pxa_buf_node *buf_node;
	dma_addr_t dma_handles;

	printk(KERN_ERR "cam: dump_dma_buf_list ************+\n");
	list_for_each_entry(buf_node, &pcdev->dma_buf_list, vb.queue) {
		dma_handles = videobuf_to_dma_contig(&buf_node->vb);
		printk(KERN_ERR "cam: buf_node 0x%x, pa 0x%x\n",
			(unsigned int)buf_node, dma_handles);
	}
	printk(KERN_ERR "cam: dump_dma_buf_list ************-\n\n");
}
#endif

/* only handle in irq context*/
static void dma_fetch_frame(struct pxa955_cam_dev *pcdev)
{
	struct pxa_buf_node *buf_node = NULL;
	int node_num = 0;
	dma_addr_t dma_handles;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;
	u32 fmt = pcdev->icd->current_fmt->host_fmt->fourcc;

	list_for_each_entry(buf_node, &pcdev->dma_buf_list, vb.queue) {
		if (buf_node->vb.state == VIDEOBUF_QUEUED)
			node_num++;
	}
	if (node_num > 1) {
		/*
		* get the first node of dma_list, it must have been filled by dma, and
		* remove it from dma-buf-list.
		*/
		buf_node = list_entry(pcdev->dma_buf_list.next,
			struct pxa_buf_node, vb.queue);
		dma_handles = videobuf_to_dma_contig(&buf_node->vb);
		if (buf_node->vb.memory == V4L2_MEMORY_USERPTR) {
			/*
			* as mmap allocate uncache buffer, not necessary to invalid cache
			* for userptr, we are not sure buffer property, to make sure user
			* could get the real data from DDR, invalid cache.
			*/
			dma_map_page(dev,
					buf_node->page,
					0,
					buf_node->vb.bsize,
					DMA_FROM_DEVICE);
			if (fmt == V4L2_PIX_FMT_JPEG) {
				if ((((char *)buf_node->ptr_vaddr)[0] != 0xff)
				|| (((char *)buf_node->ptr_vaddr)[1] != 0xd8))
					printk(KERN_ERR "cam: JPEG error!\n");
			}

		} else if (buf_node->vb.memory == V4L2_MEMORY_MMAP) {
			if (fmt == V4L2_PIX_FMT_JPEG) {
				if ((((char *)buf_node->mmp_vaddr)[0] != 0xff)
				|| (((char *)buf_node->mmp_vaddr)[1] != 0xd8))
					printk(KERN_ERR "cam: JPEG error!\n");
			}
		}

		buf_node->vb.state = VIDEOBUF_DONE;
		wake_up(&buf_node->vb.done);
		list_del_init(&buf_node->vb.queue);
	} else {
		/*if there is only one left in dma_list, drop it!*/
		printk(KERN_DEBUG "cam: drop a frame!\n");
	}

}

static void dma_append_desc(struct pxa955_cam_dev* pcdev,
								struct pxa_buf_node* pre,
								struct pxa_buf_node* next)
{
	int i = 0;
	struct pxa_cam_dma *pre_dma = NULL, *next_dma = NULL;

	for (i = 0; i < pcdev->channels; i++) {
		pre_dma = &pre->dma_desc[i];
		next_dma = &next->dma_desc[i];
		pre_dma->sg_cpu[pre_dma->sglen-1].ddadr = (u32)next_dma->sg_dma;
	}
	printk(KERN_DEBUG "cam: append new dma 0x%x to 0x%x\n",
		next_dma->sg_dma, pre_dma->sg_dma);
}

static void dma_attach_bufs(struct pxa955_cam_dev *pcdev)
{
	struct pxa_buf_node *buf_node = NULL;
	struct pxa_buf_node *tail_node = NULL;
	unsigned int regval;
	bool dma_branched = false;

	list_for_each_entry(buf_node, &pcdev->dma_buf_list, vb.queue) {

		if (buf_node->vb.state == VIDEOBUF_QUEUED)
			continue;

		/*
		* we got a new en-queue buf which is not in HW dma chain, append it to
		* the tail of HW dma chain, and loop the tail to itself.
		*/
		if (buf_node->vb.state == VIDEOBUF_ACTIVE) {

			tail_node = list_entry(buf_node->vb.queue.prev,
						struct pxa_buf_node, vb.queue);

			/*
			* NOTE!!! only one desc for one frame buffer, if not in this way,
			* need change here, as phy addr might not located between the
			* begin and end, phy addr might not continuous between different
			* desc of one frame buffer.
			*/
			regval = sci_reg_read(pcdev, REG_SCITADDR0);
			if (((regval >= videobuf_to_dma_contig(&tail_node->vb))
				&& (regval < (videobuf_to_dma_contig(&tail_node->vb)
				+ pcdev->channel_size[0])))
				&& (dma_branched == false)) {
				/*
				* if we find DMA is looping in the last buf, and there is new
				* coming buf, (DMA target address shows that DMA is working in
				* the tail buffer) we SHOULD set DMA branch reg, force DMA move
				* to the new buf descriptor in the next frame, so we can pick up
				* this buf when next irq comes.
				*/
				dma_branched = true;
				sci_reg_write(pcdev, REG_SCIDBR0, (buf_node->dma_desc[0].sg_dma | SCIDBR_EN));
				if(pcdev->channels == 3) {
					sci_reg_write(pcdev, REG_SCIDBR1, (buf_node->dma_desc[1].sg_dma | SCIDBR_EN));
					sci_reg_write(pcdev, REG_SCIDBR2, (buf_node->dma_desc[2].sg_dma | SCIDBR_EN));
				}
			} else {
				/*
				* if it is not the last buf which DMA looping in, just append
				* desc to the tail of the DMA chain.
				*/
				dma_append_desc(pcdev, tail_node, buf_node);
			}
			dma_append_desc(pcdev, buf_node, buf_node);
			buf_node->vb.state = VIDEOBUF_QUEUED;
		}
	}

}

static int dma_alloc_desc(struct pxa_buf_node *buf_node,
					struct pxa955_cam_dev *pcdev)
{
	int i;
	unsigned int 	len = 0, len_tmp = 0;
	pxa_dma_desc 	*dma_desc_tmp;
	unsigned long 	dma_desc_phy_tmp;
	unsigned long 	srcphyaddr, dstphyaddr;
	struct pxa_cam_dma *desc;
	struct videobuf_buffer *vb = &buf_node->vb;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;
	srcphyaddr = 0;	/* TBD */

	dstphyaddr = videobuf_to_dma_contig(vb);

	for (i = 0; i < pcdev->channels; i++) {
		printk(KERN_DEBUG "cam: index %d, channels %d\n",vb->i, i);
		desc = &buf_node->dma_desc[i];
		len = pcdev->channel_size[i];

		desc->sglen = (len + SINGLE_DESC_TRANS_MAX - 1) / SINGLE_DESC_TRANS_MAX;
		desc->sg_size = (desc->sglen) * sizeof(struct pxa_dma_desc);

		if (desc->sg_cpu == NULL){
			desc->sg_cpu = dma_alloc_coherent(dev, desc->sg_size,
					     &desc->sg_dma, GFP_KERNEL);
		}
		printk(KERN_DEBUG "cam: sglen %d, size %d, sg_cpu 0x%x\n",
			desc->sglen, desc->sg_size, (unsigned int)desc->sg_cpu);
		if (!desc->sg_cpu){
			printk(KERN_ERR "cam: dma_alloc_coherent failed at chnnl %d!\n", i);
			goto err;
		}

		dma_desc_tmp = desc->sg_cpu;
		dma_desc_phy_tmp = desc->sg_dma;

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
	for (i = 0; i < pcdev->channels; i++) {
		desc = &buf_node->dma_desc[i];
		if (desc->sg_cpu) {
			dma_free_coherent(dev, desc->sg_size,
				    desc->sg_cpu, desc->sg_dma);
			desc->sg_cpu = 0;
		}
	}
	return -ENOMEM;
}

static void dma_free_desc(struct pxa_buf_node *buf_node,
							struct pxa955_cam_dev *pcdev)
{
	int i;
	struct pxa_cam_dma *desc;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;

	for (i = 0; i < pcdev->channels; i++) {
		desc = &buf_node->dma_desc[i];
		if (desc->sg_cpu){
			dma_free_coherent(dev, desc->sg_size,
				    desc->sg_cpu, desc->sg_dma);
			desc->sg_cpu = 0;
		}
	}
}

static void dma_free_bufs(struct pxa_buf_node *buf,
				struct pxa955_cam_dev *pcdev)
{
	struct videobuf_buffer *vb = &buf->vb;
	struct videobuf_dma_contig_memory *mem = vb->priv;

	if (buf->ptr_vaddr) {
		iounmap(buf->ptr_vaddr);
		buf->ptr_vaddr = 0;
	}
	/*
	* TODO:
	* This waits until this buffer is out of danger, i.e., until it is no
	* longer in STATE_QUEUED or STATE_ACTIVE
	*/
	/*videobuf_waiton(vb, 0, 0);*/

	dma_free_desc(buf, pcdev);

	/*
	* TODO: as user will call stream-off when he hasn't en-queue all
	* request-buffers, if directlty call videobuf_dma_contig_free here,
	* it will release non-initizlized buffers, it causes kernel panic. So,
	* we implement our own free operation as follow to avoid kernel panic.
	* (refer to videobuf-dma-contig.c)
	*/
	/*videobuf_dma_contig_free(vq, &buf->vb);*/

	/*
	* mmapped memory can't be freed here, otherwise mmapped region
	* would be released, while still needed. In this case, the memory
	* release should happen inside videobuf_vm_close().
	* So, it should free memory only if the memory were allocated for
	* read() operation.
	*/
	if (vb->memory != V4L2_MEMORY_USERPTR)
		return;

	if (!mem)
		return;

	MAGIC_CHECK(mem->magic, MAGIC_DC_MEM);

	/* handle user space pointer case */
	if (vb->baddr) {
		/*videobuf_dma_contig_user_put(mem);*/
		mem->is_userptr = 0;
		mem->dma_handle = 0;
		mem->size = 0;
	}

	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static void dma_chain_init(struct pxa955_cam_dev *pcdev)
{
	int i = 0;
	struct pxa_cam_dma *dma_cur = NULL, *dma_pre = NULL;
	struct pxa_buf_node *buf_cur = NULL, *buf_pre = NULL;
	struct pxa_buf_node *buf_node;

	/*chain the buffers in the dma_buf_list list*/
	list_for_each_entry(buf_node, &pcdev->dma_buf_list, vb.queue) {
		if (buf_node->vb.state == VIDEOBUF_ACTIVE) {
			buf_cur = buf_node;

			/*head of the dma_buf_list, this is the first dma desc*/
			if (buf_node->vb.queue.prev == &pcdev->dma_buf_list) {
				for (i = 0; i < pcdev->channels; i++) {
					dma_cur = &buf_cur->dma_desc[i];
					if (buf_pre)
						dma_pre = &buf_pre->dma_desc[i];
					sci_reg_write(pcdev, REG_SCIDADDR0 + i*0x10, dma_cur->sg_dma);
				}
				if (skip_frame) {
					buf_cur->vb.state = VIDEOBUF_QUEUED;
					dma_append_desc(pcdev, buf_cur, buf_cur);
					break;
				}
			} else {
				/* link to prev descriptor */
				dma_append_desc(pcdev, buf_pre, buf_cur);
			}

			/* find the tail, loop back to the tail itself */
			if (&pcdev->dma_buf_list == buf_node->vb.queue.next) {
				dma_append_desc(pcdev, buf_cur, buf_cur);
			}
			buf_pre = buf_cur;
			buf_node->vb.state = VIDEOBUF_QUEUED;
		}
	}

}

static int sci_cken(struct pxa955_cam_dev *pcdev, int flag)
{
	if (flag) {
		/* The order of enabling matters! AXI must be the 1st one */
		clk_enable(pcdev->sci1_clk);
		clk_enable(pcdev->sci2_clk);
	} else {
		clk_disable(pcdev->sci2_clk);
		clk_disable(pcdev->sci1_clk);
	};

	return 0;
}

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
	((x) >> 24) & 0xff
static void sci_s_fmt(struct pxa955_cam_dev *pcdev,
				struct v4l2_pix_format *fmt)
{
	unsigned int size = fmt->width*fmt->height;
	printk(KERN_NOTICE "cam: set fmt as %c%c%c%c, %ux%u\n",
		pixfmtstr(fmt->pixelformat), fmt->width, fmt->height);

	switch (fmt->pixelformat) {

	case V4L2_PIX_FMT_RGB565:
		pcdev->channels = 1;
		pcdev->channel_size[0] = size*2;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_RGB565) | SCICR1_FMT_OUT(FMT_RGB565));
	    break;

	case V4L2_PIX_FMT_JPEG:
		pcdev->channels = 1;
		/* use size get from sensor */
		/* pcdev->channel_size[0] = fmt->sizeimage;*/

		pcdev->channel_size[0] = JPEG_BUF_SIZE;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_JPEG) | SCICR1_FMT_OUT(FMT_JPEG));
	    break;

	case V4L2_PIX_FMT_YUV422P:
		pcdev->channels = 3;
		pcdev->channel_size[0] = size;
		pcdev->channel_size[1] = pcdev->channel_size[2] = size/2;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV422));
		break;

	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		pcdev->channels = 1;
		pcdev->channel_size[0] = size*2;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV422PACKET));
		break;

	case V4L2_PIX_FMT_YUV420:
		/* only accept YUV422 as input */
		pcdev->channels = 3;
		pcdev->channel_size[0] = size;
		pcdev->channel_size[1] = pcdev->channel_size[2] = size/4;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | SCICR1_FMT_OUT(FMT_YUV420));
		break;

	default:
		printk(KERN_ERR "cam: error can not support fmt!\n");
		break;
	}

}

static void sci_irq_enable(struct pxa955_cam_dev *pcdev, unsigned int val)
{
	sci_reg_write(pcdev, REG_SCISR, sci_reg_read(pcdev, REG_SCISR));
	sci_reg_clear_bit(pcdev, REG_SCIMASK, val);
}

static void sci_irq_disable(struct pxa955_cam_dev *pcdev, unsigned int val)
{
	sci_reg_set_bit(pcdev, REG_SCIMASK, val);
}

/*
 * Make the controller start grabbing images.  Everything must
 * be set up before doing this.
 */
static void sci_enable(struct pxa955_cam_dev *pcdev)
{
	int i = 0;
	unsigned int val = 0;

	/* start_fifo */
	for (i = 0; i < pcdev->channels; i++) {
		val = SCIFIFO_F0_EN << i;
		sci_reg_set_bit(pcdev, REG_SCIFIFO, val);
	}

	/* start_dma */
	for (i = 0; i < pcdev->channels; i++)
		sci_reg_set_bit(pcdev, REG_SCIDCSR0 + i*0x10, SCIDCSR_DMA_RUN);

	/* start sci */
	sci_reg_set_bit(pcdev, REG_SCICR0, SCICR0_CAP_EN | SCICR0_CI_EN);
}

static void sci_disable(struct pxa955_cam_dev *pcdev)
{
	int i = 0;
	unsigned int val = 0;

	/* stop_fifo */
	for (i = 0; i < pcdev->channels; i++) {
		val = SCIFIFO_F0_EN << i;
		sci_reg_clear_bit(pcdev, REG_SCIFIFO, val);
	}

	/* stop_dma */
	for (i = 0; i < pcdev->channels; i++) {
		sci_reg_clear_bit(pcdev, REG_SCIDCSR0 + i*0x10, SCIDCSR_DMA_RUN);
		sci_reg_clear_bit(pcdev, REG_SCIDBR0 + i*0x10, SCIDBR_EN);
	}
	/* stop sci */
	sci_reg_clear_bit(pcdev, REG_SCICR0, SCICR0_CAP_EN | SCICR0_CI_EN);
}

void sci_init(struct pxa955_cam_dev *pcdev)
{
	/*
	* Turn off the enable bit.  It sure should be off anyway,
	* but it's good to be sure.
	*/
	sci_reg_clear_bit(pcdev, REG_SCICR0, SCICR0_CI_EN);

	/* Mask all interrupts.*/
	sci_reg_write(pcdev, REG_SCIMASK, ~0);
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

static int pxa955_videobuf_setup(struct videobuf_queue *vq, unsigned int *count,
			      unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	if (icd->current_fmt->host_fmt->fourcc != V4L2_PIX_FMT_JPEG) {
		dev_dbg(icd->dev.parent, "count=%d, size=%d\n", *count, *size);

		*size = bytes_per_line * icd->user_height;
		if (0 == *count)
			*count = 32;
		if (*size * *count > vid_mem_limit * 1024 * 1024)
			*count = (vid_mem_limit * 1024 * 1024) / *size;
	} else {
		*size = JPEG_BUF_SIZE;
		if (0 == *count)
			*count = 32;
		if (*size * *count > vid_mem_limit * 1024 * 1024)
			*count = (vid_mem_limit * 1024 * 1024) / *size;
	}
	return 0;
}

static int pxa955_videobuf_prepare(struct videobuf_queue *vq,
		struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct pxa_buf_node *buf =
		container_of(vb, struct pxa_buf_node, vb);
	struct pxa_cam_dma *desc;
	unsigned int vaddr;
	int ret;
	size_t new_size;
	dma_addr_t dma_handles;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;
	new_size = bytes_per_line * icd->user_height;
	if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		new_size = JPEG_BUF_SIZE;
	}

	if (buf->code	!= icd->current_fmt->code ||
	    vb->width	!= icd->user_width ||
	    vb->height	!= icd->user_height ||
	    vb->field	!= field) {
		buf->code	= icd->current_fmt->code;
		vb->width	= icd->user_width;
		vb->height	= icd->user_height;
		vb->field	= field;
		if (vb->state != VIDEOBUF_NEEDS_INIT) {
			dma_free_bufs(buf, pcdev);
		}
	}

	if (vb->baddr && (vb->bsize < new_size)) {
		/* User provided buffer, but it is too small */
		printk(KERN_ERR
			"cam: buf in use %d is smaller than required size %d!\n",
			vb->bsize, new_size);
		return -ENOMEM;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {

		if (vb->memory == V4L2_MEMORY_USERPTR) {
			vaddr = PAGE_ALIGN(vb->baddr);
			if (vaddr != vb->baddr) {
				printk(KERN_ERR "cam: the memory is not page align!\n");
				return -EPERM;
			}

			/* get page info for dma invalid cache operation*/
			buf->page = va_to_page(vaddr);
			if (!buf->page) {
				printk(KERN_ERR "cam: fail to get page info!\n");
				return -EFAULT;
			}
		}
		/*
		* The total size of video-buffers that will be allocated / mapped.
		* size that we calculated in videobuf_setup gets assigned to
		* vb->bsize, and now we use the same calculation to get vb->size.
		*/
		vb->size = new_size;

		/* This actually (allocates and) maps buffers */
		ret = videobuf_iolock(vq, vb, NULL);

		/*only for jpeg case, as we need to verify the jpeg file header*/
		if (icd->current_fmt->host_fmt->fourcc
					== V4L2_PIX_FMT_JPEG) {

			if (vb->memory == V4L2_MEMORY_USERPTR) {
				/*
				* dma_handles only be valid after actually
				* map/allocated in videobuf_iolock
				*/
				dma_handles = videobuf_to_dma_contig(vb);
				/* map the PA to kernel space, for user-pointer method buf*/
				buf->ptr_vaddr = ioremap(dma_handles,
									PAGE_ALIGN(new_size));
				if (!buf->ptr_vaddr) {
					printk("cam: failed get vaddr of the user buffer!\n");
					return -EFAULT;
				}
			} else if (vb->memory == V4L2_MEMORY_MMAP) {
				buf->mmp_vaddr = videobuf_queue_to_vaddr(vq, vb);
				if (!buf->mmp_vaddr) {
					printk("cam: failed get vaddr of the mmap buffer!\n");
					return -EFAULT;
				}
			}
		}

		desc = &buf->dma_desc[0];
		if (desc->sg_cpu == NULL) {
			dma_alloc_desc(buf, pcdev);
		}

		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;
}

static void pxa955_videobuf_queue(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct csi_phy_config timing;

	dev_dbg(icd->dev.parent, "%s (vb=0x%p) 0x%08lx %d\n",
		__func__, vb, vb->baddr, vb->bsize);

	list_add_tail(&vb->queue, &pcdev->dma_buf_list);
	vb->state = VIDEOBUF_ACTIVE;

	if (vq->streaming == 1) {

		/* at first time, streamon set to true, means stream-on*/
		if (!pcdev->streamon) {

			if (list_empty(&pcdev->dma_buf_list)) {
				printk(KERN_ERR "cam: internal buffers are not ready!\n");
				return;
			}
			pcdev->streamon = true;
			pcdev->streamoff = false;

			dma_chain_init(pcdev);

			sci_irq_enable(pcdev, IRQ_EOFX|IRQ_OFO);
			csi_dphy(pcdev->csidev, timing);
			/* configure enable which camera interface controller*/
			csi_enable(pcdev->csidev, CCIC_0);
			sci_enable(pcdev);
		}
	}
}

static void pxa955_videobuf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct pxa_buf_node *buf = container_of(vb, struct pxa_buf_node, vb);

#ifdef DEBUG
	struct device *dev = icd->dev.parent;
	dev_err(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

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
#endif

	if ((vb->state == VIDEOBUF_ACTIVE
		|| vb->state == VIDEOBUF_QUEUED) &&
	    !list_empty(&vb->queue)) {
		vb->state = VIDEOBUF_ERROR;

		list_del_init(&vb->queue);
	}

	if (vq->streaming == 0) {
		if (!pcdev->streamoff) {
			INIT_LIST_HEAD(&pcdev->dma_buf_list);
			pcdev->streamon = false;
			pcdev->streamoff = true;

			csi_disable(pcdev->csidev);
			sci_irq_disable(pcdev, IRQ_EOFX|IRQ_OFO);
			sci_disable(pcdev);
		}
	}

	dma_free_bufs(buf, pcdev);

}

static struct videobuf_queue_ops pxa955_videobuf_ops = {
	.buf_setup      = pxa955_videobuf_setup,
	.buf_prepare    = pxa955_videobuf_prepare,
	.buf_queue      = pxa955_videobuf_queue,
	.buf_release    = pxa955_videobuf_release,
};

static int pxa955_cam_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;

	if (pcdev->icd)
		return -EBUSY;

	pcdev->icd = icd;

	csi_cken(pcdev->csidev, 1);
	sci_cken(pcdev, 1);
	sci_init(pcdev);
	csi_clkdiv(pcdev->csidev);

	dev_info(icd->dev.parent, "pxa955 camera driver attached to camera %d\n",
		 icd->devnum);

	return 0;
}

static void pxa955_cam_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct videobuf_queue *vq = pcdev->videoq;
	int i;

	BUG_ON(icd != pcdev->icd);
	sci_cken(pcdev, 0);
	csi_cken(pcdev->csidev, 0);

	if (pcdev->videoq) {
		/*
		* FIXME: it seems that no one calls videobuf_mmap_free in
		* soc-camera, so let's free the buffer node ourselves in
		* buf_release, to avoid memory leak. (pls refer to videobuf-core.c)
		* For mmap method, the buf node will be release
		* in videobuf_vm_close when application call munmap.
		*/
		for (i = 0; i < VIDEO_MAX_FRAME; i++)
			if (vq->bufs[i] && vq->bufs[i]->map)
				dev_err(icd->dev.parent, "can not free the mapped buffer\n");

		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == vq->bufs[i])
				continue;

			dev_info(icd->dev.parent, "release buffer node of %d\n", i);
			kfree(vq->bufs[i]);
			vq->bufs[i] = NULL;
		}
	}

	pcdev->icd = NULL;
	dev_info(icd->dev.parent, "pxa955 Camera driver detached from camera %d\n",
		 icd->devnum);
}

static const struct soc_mbus_pixelfmt pxa955_camera_formats[] = {
	{
			.fourcc 	= V4L2_PIX_FMT_YUV422P,
			.name		= "YUV422P",
			.bits_per_sample	= 8,
			.packing		= SOC_MBUS_PACKING_2X8_PADLO,
			.order			= SOC_MBUS_ORDER_LE,
	},{
		.fourcc			= V4L2_PIX_FMT_YUV420,
		.name			= "YUV420P",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_2X8_PADLO,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

/* pxa955_cam_get_formats provide all fmts that camera controller support*/
static int pxa955_cam_get_formats(struct soc_camera_device *icd, unsigned int idx,
				  struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	int formats = 0, ret, i;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;

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
	/* refer to mbus_fmt struct*/
	case V4L2_MBUS_FMT_YUYV8_2X8_BE:
		formats = ARRAY_SIZE(pxa955_camera_formats);

		if (xlate) {
			for (i = 0; i < ARRAY_SIZE(pxa955_camera_formats); i++) {
				xlate->host_fmt = &pxa955_camera_formats[i];
				xlate->code	= code;
				xlate++;
				dev_err(dev, "Providing format %s\n",
					pxa955_camera_formats[i].name);
			}
			dev_err(dev, "Providing format %s\n", fmt->name);
		}
		break;

	case V4L2_MBUS_FMT_YVYU8_2X8_BE:
	case V4L2_MBUS_FMT_YVYU8_2X8_LE:
	case V4L2_MBUS_FMT_YUYV8_2X8_LE:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
	case V4L2_MBUS_FMT_JPEG_1X8:
		if (xlate)
			dev_err(dev, "Providing format %s\n", fmt->name);
		break;
	default:
		/* camera controller can not support this format, which might supported by the sensor*/
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

static void pxa955_cam_put_formats(struct soc_camera_device *icd)
{
	kfree(icd->host_priv);
	icd->host_priv = NULL;
}

static int pxa955_cam_try_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(icd->dev.parent, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
							xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;
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

static int pxa955_cam_set_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct device *dev = icd->dev.parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	__u32 pixfmt = pix->pixelformat;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(dev, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);

	if (mf.code != xlate->code)
		return -EINVAL;

	icd->sense = NULL;
	pix->width		= mf.width;
	pix->height		= mf.height;
	pix->field		= mf.field;
	pix->colorspace		= mf.colorspace;
	icd->current_fmt	= xlate;

	sci_disable(pcdev);
	sci_s_fmt(pcdev, pix);
	v4l2_subdev_call(sd, sensor, g_skip_top_lines, &skip_frame);

	return ret;
}

static void pxa955_cam_init_videobuf(struct videobuf_queue *q,
			      struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;

	pcdev->videoq = q;
	/*
	* We must pass NULL as dev pointer, then all pci_* dma operations
	* transform to normal dma_* ones.
	*/
	videobuf_queue_dma_contig_init(q, &pxa955_videobuf_ops, icd->dev.parent,
				&pcdev->spin_lock,
				V4L2_BUF_TYPE_VIDEO_CAPTURE,
				V4L2_FIELD_NONE,
				sizeof(struct pxa_buf_node), icd);
}

static int pxa955_cam_reqbufs(struct soc_camera_file *icf,
			      struct v4l2_requestbuffers *p)
{
	int i;
	if (p->count < MIN_DMA_BUFS) {
		printk(KERN_ERR "cam: need %d buffers at least!\n", MIN_DMA_BUFS);
		return -EINVAL;
	}

	/*
	*This is for locking debugging only. I removed spinlocks and now I
	* check whether .prepare is ever called on a linked buffer, or whether
	* a dma IRQ can occur for an in-work or unlinked buffer. Until now
	* it hadn't triggered
	*/
	for (i = 0; i < p->count; i++) {
		struct pxa_buf_node *buf = container_of(icf->vb_vidq.bufs[i],
						      struct pxa_buf_node, vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}
	return 0;
}

static unsigned int pxa955_cam_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	return videobuf_poll_stream(file, &icf->vb_vidq, pt);
}

static int pxa955_cam_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	struct v4l2_dbg_chip_ident id;
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct soc_camera_device *icd = pcdev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int ret = 0;

	cap->version = PXA955_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	strcpy(cap->card, "MG1");
	strcpy(cap->driver, "N/A");

	ret = v4l2_subdev_call(sd, core, g_chip_ident, &id);
	if (ret < 0) {
		printk(KERN_ERR "cam: failed to get sensor's name!\n");
		return ret;
	}
	switch (id.ident) {
		case V4L2_IDENT_OV5642:
			strcpy(cap->driver, "ov5642");
			break;
		case V4L2_IDENT_OV7690:
			strcpy(cap->driver, "ov7690");
			break;
	}
	return 0;
}

static int pxa955_cam_set_bus_param(struct soc_camera_device *icd, __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	unsigned long ctrller_flags, sensor_flags, common_flags;
	int ret;
	int lane = 1;

	/* Configure this flag according to controller ability: support 3 lanes */
	ctrller_flags = SOCAM_MIPI | SOCAM_MIPI_1LANE \
			| SOCAM_MIPI_2LANE | SOCAM_MIPI_3LANE,
	sensor_flags = icd->ops->query_bus_param(icd);

	common_flags = soc_camera_bus_param_compatible(sensor_flags, ctrller_flags);
	if (!common_flags) {
		return -EINVAL;
	}

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

	if (common_flags & SOCAM_MIPI_1LANE) {
		lane = 1;
	} else if (common_flags & SOCAM_MIPI_2LANE) {
		lane = 2;
	}
	csi_lane(pcdev->csidev, lane);
	return 0;
}

static int pxa955_cam_set_param(struct soc_camera_device *icd,
				  struct v4l2_streamparm *parm)
{
	return 0;
}

static int pxa955_cam_enum_fsizes(struct soc_camera_device *icd,
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

static struct soc_camera_host_ops pxa955_soc_cam_host_ops = {
	.owner		= THIS_MODULE,
	.add		= pxa955_cam_add_device,
	.remove		= pxa955_cam_remove_device,
	.get_formats	= pxa955_cam_get_formats,
	.put_formats	= pxa955_cam_put_formats,
	.set_fmt	= pxa955_cam_set_fmt,
	.try_fmt	= pxa955_cam_try_fmt,
	.init_videobuf	= pxa955_cam_init_videobuf,
	.reqbufs	= pxa955_cam_reqbufs,
	.poll		= pxa955_cam_poll,
	.querycap	= pxa955_cam_querycap,
	.set_bus_param	= pxa955_cam_set_bus_param,
	.set_parm = pxa955_cam_set_param,
	.enum_fsizes = pxa955_cam_enum_fsizes,
};

static irqreturn_t csi_irq(int irq, void *data)
{
	struct pxa955_cam_dev *pcdev = data;
	struct pxa95x_csi_dev *pcsi = pcdev->csidev;
	unsigned int irqs;

	spin_lock(&pcsi->dev_lock);
	irqs = csi_reg_read(pcsi, REG_CSxINST);
	csi_reg_write(pcsi, REG_CSxINST, irqs);
	spin_unlock(&pcsi->dev_lock);
	return IRQ_HANDLED;
}

static irqreturn_t cam_irq(int irq, void *data)
{
	struct pxa955_cam_dev *pcdev = data;
	unsigned int irqs = 0;

	spin_lock(&pcdev->spin_lock);
	irqs = sci_reg_read(pcdev, REG_SCISR);
	sci_reg_write(pcdev, REG_SCISR, irqs);		/*clear irqs here*/
	if (irqs & IRQ_OFO) {

		printk(KERN_ERR "cam: ccic over flow error!\n");
		csi_disable(pcdev->csidev);
		sci_disable(pcdev);
		csi_enable(pcdev->csidev, CCIC_0);
		sci_enable(pcdev);
		spin_unlock(&pcdev->spin_lock);
		return IRQ_HANDLED;
	}

	if (irqs & IRQ_EOFX) {
		if (skip_frame == 0) {
			dma_fetch_frame(pcdev);
			dma_attach_bufs(pcdev);
		} else {
			printk(KERN_NOTICE "cam: skip frame %d\n", skip_frame);
			skip_frame--;
		}
	}

	spin_unlock(&pcdev->spin_lock);
	return IRQ_HANDLED;
}

static int pxa955_camera_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err = -ENOMEM;
	struct pxa955_cam_dev *pcdev;

	pcdev = kzalloc(sizeof(struct pxa955_cam_dev), GFP_KERNEL);
	if (pcdev == NULL)
		goto exit;
	memset(pcdev, 0, sizeof(struct pxa955_cam_dev));

	pcdev->csidev = kzalloc(sizeof(struct pxa95x_csi_dev), GFP_KERNEL);
	if (pcdev->csidev == NULL)
		goto exit;
	memset(pcdev->csidev, 0, sizeof(struct pxa95x_csi_dev));

	spin_lock_init(&pcdev->spin_lock);
	init_waitqueue_head(&pcdev->iowait);
	INIT_LIST_HEAD(&pcdev->dev_list);
	INIT_LIST_HEAD(&pcdev->dma_buf_list);

	pcdev->pdata = pdev->dev.platform_data;
	if (pcdev->pdata == NULL) {
		printk(KERN_ERR "cam: camera no platform data defined\n");
		return -ENODEV;
	}

	err = -EIO;

	/* init csi controller which is combined with this camera controller*/
	pcdev->csidev->regs =
		ioremap(pcdev->pdata->csidev->reg_start, SZ_4K);
	if (!pcdev->csidev->regs) {
		printk(KERN_ERR "cam: unable to ioremap pxa95x-camera csi regs\n");
		goto exit_free;
	}
	err = request_irq(pcdev->pdata->csidev->irq_num,
		csi_irq, 0, PXA955_CAM_DRV_NAME, pcdev);
	if (err) {
		printk(KERN_ERR "cam: unable to create csi ist\n");
		goto exit_iounmap;
	}

	spin_lock_init(&pcdev->csidev->dev_lock);
	pcdev->csidev->csi_tx_esc = clk_get(NULL, "CSI_TX_ESC");
	if (!pcdev->csidev->csi_tx_esc) {
		printk(KERN_ERR "cam: unable to get CSI_TX_ESC\n");
		goto exit_iounmap;
	};

	pcdev->csidev->axi_clk = clk_get(NULL, "AXICLK");
	if (!pcdev->csidev->axi_clk) {
		printk(KERN_ERR "cam: unable to get AXICLK\n");
		goto exit_iounmap;
	};

	/* init camera controller resource*/
	pcdev->irq = platform_get_irq(pdev, 0);
	if (pcdev->irq < 0) {
		printk(KERN_ERR "cam: camera no irq\n");
		return -ENXIO;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "cam: no IO memory resource defined\n");
		return -ENODEV;
	}

	pcdev->regs = ioremap(res->start, SZ_4K);
	if (!pcdev->regs) {
		printk(KERN_ERR "cam: unable to ioremap pxa95x-camera regs\n");
		goto exit_free;
	}
	err = request_irq(pcdev->irq, cam_irq,
		0, PXA955_CAM_DRV_NAME, pcdev);
	if (err) {
		printk(KERN_ERR "cam: unable to create ist\n");
		goto exit_iounmap;
	}

	pcdev->sci1_clk = clk_get(NULL, "SCI1CLK");
	if (!pcdev->sci1_clk) {
		printk(KERN_ERR "cam: unable to get sci clk\n");
		goto exit_free_irq;
	}
	pcdev->sci2_clk = clk_get(NULL, "SCI2CLK");
	if (!pcdev->sci2_clk) {
		printk(KERN_ERR "cam: unable to get sci clk\n");
		goto exit_free_irq;
	}

	pcdev->soc_host.drv_name	= PXA955_CAM_DRV_NAME;
	pcdev->soc_host.ops		= &pxa955_soc_cam_host_ops;
	pcdev->soc_host.priv		= pcdev;
	pcdev->soc_host.v4l2_dev.dev	= &pdev->dev;
	pcdev->soc_host.nr		= pdev->id;

	err = soc_camera_host_register(&pcdev->soc_host);
	if (err)
		goto exit_free_irq;

	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
exit_iounmap:
	iounmap(pcdev->csidev->regs);
	iounmap(pcdev->regs);
	clk_put(pcdev->sci1_clk);
	clk_put(pcdev->sci2_clk);
	clk_put(pcdev->sci1_clk);
	clk_put(pcdev->sci2_clk);
exit_free:
	kfree(pcdev->csidev);
	kfree(pcdev);
exit:
	return err;
}


static int pxa955_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct pxa955_cam_dev *pcdev = container_of(soc_host,
					struct pxa955_cam_dev, soc_host);

	if (pcdev == NULL) {
		printk(KERN_WARNING "cam: remove on unknown pdev %p\n", pdev);
		return -EIO;
	}

	csi_disable(pcdev->csidev);

	sci_disable(pcdev);
	sci_irq_disable(pcdev, IRQ_EOFX|IRQ_OFO);
	free_irq(pcdev->irq, pcdev);
	return 0;
}

static struct platform_driver pxa955_camera_driver = {
	.driver = {
		.name = PXA955_CAM_DRV_NAME
	},
	.probe 		= pxa955_camera_probe,
	.remove 	= pxa955_camera_remove,

};

static int __devinit pxa955_camera_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&pxa955_camera_driver);
	return ret;
}

static void __exit pxa955_camera_exit(void)
{
	platform_driver_unregister(&pxa955_camera_driver);
}

module_init(pxa955_camera_init);
module_exit(pxa955_camera_exit);
