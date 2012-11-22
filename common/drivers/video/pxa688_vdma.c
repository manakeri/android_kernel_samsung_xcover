/*
 * linux/drivers/video/pxa168fb.c -- Marvell PXA168 LCD Controller
 *
 *  Copyright (C) 2008 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2009-02-16  adapted from original version for PXA168
 *		Kevin Liu <kliu5@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_PXA688_VDMA

#include "pxa168fb.h"
#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/pxa168fb.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <mach/gpio.h>

#include <asm/mach-types.h>
#include <mach/regs-apmu.h>
#include <mach/mfp-mmp2.h>
#include <mach/regs-mpmu.h>
#include <asm/mach-types.h>

#include <plat/imm.h>

#define pitch_select(id)	(id ? ((id & 1)? LCD_TVD_PITCH_YC : PN2_LCD_DMA_PITCH_YC) : LCD_SPU_DMA_PITCH_YC)
#define vh_select(id)		(id ? ((id & 1)? LCD_TVD_HPXL_VLN : PN2_LCD_DMA_HPXL_VLN) : LCD_SPU_DMA_HPXL_VLN)
#define vdma_ctrl(id)		(id ? VDMA_CTRL_2 : VDMA_CTRL_1)

u32 lcd_pitch_read(struct pxa168fb_info *fbi)
{
	u32 reg = (u32)fbi->reg_base + pitch_select(fbi->id);

	return (__raw_readl(reg) & 0xffff);
}

u32 lcd_height_read(struct pxa168fb_info *fbi)
{
	u32 reg = (u32)fbi->reg_base + vh_select(fbi->id);

	return (__raw_readl(reg) & 0xfff0000) >> 16;
}

u32 lcd_width_read(struct pxa168fb_info *fbi)
{
	u32 reg = (u32)fbi->reg_base + vh_select(fbi->id);

	return (__raw_readl(reg) & 0xfff);
}

u32 vdma_ctrl_read(struct pxa168fb_info *fbi)
{
	u32 reg = (u32)fbi->reg_base + vdma_ctrl(fbi->id);

	return __raw_readl(reg);
}

void vdma_ctrl_write(struct pxa168fb_info *fbi, int value)
{
	u32 reg = (u32)fbi->reg_base + vdma_ctrl(fbi->id);

	__raw_writel(value, reg);
}

u32 vdma_paddr;
unsigned int vdma_lines;
unsigned int sram_freebytes;

void pxa688_vdma_clkset(int en)
{
	if (en)
		writel(readl(APMU_LCD2_CLK_RES_CTRL) | 0x11b,
				APMU_LCD2_CLK_RES_CTRL);
}

u32 pxa688fb_vdma_squ_malloc(void)
{/* malloc sram buffer for squ - 64 lines by default */
	static unsigned int immid = -1;
	void *vsqu;
	u32 psqu = 0;
	immid = imm_register_kernel("mmp2 dma");
	sram_freebytes = imm_get_freespace(0, immid);
	printk(KERN_INFO"%s: sram_freebytes = %d\n", __func__, sram_freebytes);
	vsqu = imm_malloc(sram_freebytes, IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM, immid);
	if(vsqu != NULL) {
		psqu = imm_get_physical(vsqu, immid);
		return psqu;
	} else {
		printk("%s: sram malloc failed!\n", __func__);
		return 0;
	}

}
void pxa688fb_vdma_release(struct pxa168fb_info *fbi)
{
	unsigned reg, isr, current_time, irq_mask;

	isr = readl(fbi->reg_base + SPU_IRQ_ISR);
	if (fbi->id == 0)
		irq_mask = DMA_FRAME_IRQ0_MASK | DMA_FRAME_IRQ1_MASK;
	else if (fbi->id == 1)
		irq_mask = TV_DMA_FRAME_IRQ0_MASK | TV_DMA_FRAME_IRQ1_MASK;
	else
		irq_mask = PN2_DMA_FRAME_IRQ0_MASK | PN2_DMA_FRAME_IRQ1_MASK;
	irq_status_clear(fbi,  irq_mask);
	current_time = jiffies;
	while((readl(fbi->reg_base + SPU_IRQ_ISR) &	irq_mask) == 0) {
		if (jiffies_to_msecs(jiffies - current_time) > EOF_TIMEOUT) {
			printk(KERN_ERR"EOF not detected !");
			break;
		}
	}
	reg = vdma_ctrl_read(fbi);
	reg &= ~0xF;
	vdma_ctrl_write(fbi, reg);
	reg = readl(fbi->reg_base + ((fbi->id == 0) ? LCD_SQULN1_CTRL :
				((fbi->id == 1) ? LCD_SQULN2_CTRL : LCD_PN2_SQULN1_CTRL)));
	reg &= (~0x1);
	writel(reg, fbi->reg_base + ((fbi->id == 0) ? LCD_SQULN1_CTRL :
				((fbi->id == 1) ? LCD_SQULN2_CTRL : LCD_PN2_SQULN1_CTRL)));

}
static int __get_vdma_rot_ctrl(int vmode, int angle, int yuv_format)
{
	int rotation, flag = 0, reg = 0;

	if (angle == 1)
		return 0;
	switch (angle) {
		case 90:
			rotation = 1;
			break;
		case 270:
			rotation = 0;
			break;
		case 180:
			rotation = 2;
			break;
		default:
			rotation = 0;
			return rotation;
	}
	switch(vmode)
	{
		case FB_VMODE_RGB565:
			reg = 4 << 2;
			reg |= rotation << 11;
			break;
		case FB_VMODE_YUV422PACKED:
		case FB_VMODE_YUV422PACKED_SWAPUV:
		case FB_VMODE_YUV422PACKED_SWAPYUorV:
			flag = 1;
		case FB_VMODE_YUV422PACKED_IRE_90_270:
			reg = 1 << 5;
			reg |= 1 << 7;
			reg |= 3 << 2;
			reg |= rotation << 11;
			break;
		case FB_VMODE_RGB888PACK:
		case FB_VMODE_RGB888UNPACK:
		default:
			reg = 6 << 2;
			reg |= rotation << 11;
			reg |= 0 << 7;
			break;

	}
	if ( vmode == FB_VMODE_YUV422PACKED_IRE_90_270 || flag == 1) {
		if (rotation == 2)
			reg |= 2 << 21;
		else
			reg |= 1 << 21;
		if (vmode == FB_VMODE_YUV422PACKED)
			yuv_format = 1;
		if (vmode == FB_VMODE_YUV422PACKED_SWAPUV)
			yuv_format = 2;
		if (vmode == FB_VMODE_YUV422PACKED_SWAPYUorV)
			yuv_format = 4;
		switch (yuv_format)
		{
			case 1:/*YUV_FORMAT_UYVY*/
				reg |= 1 << 9;
				break;
			case 2:/*YUV_FORMAT_VYUY*/
				reg |= 0 << 9;
				break;
			case 3:/*YUV_FORAMT_YVYU*/
				reg |= 3 << 9;
				break;
			case 4:/*YUV_FORMAT_YUYV*/
				reg |= 2 << 9;
				break;
		}
	}
	return reg;
}

static int __get_vdma_src_sz(struct pxa168fb_info *fbi, int width, int height)
{
	int res = lcd_pitch_read(fbi) * height;

	if (fbi->surface.videoMode == FB_VMODE_YUV422PACKED_IRE_90_270)
		return res >> 1;
	else
		return res;
}
static int __get_vdma_sa(struct pxa168fb_info *fbi, int width, int height, int bpp)
{
	int addr = 0;

	if (fbi->surface.videoMode == FB_VMODE_YUV422PACKED_IRE_90_270)
		bpp = 2;
	switch (fbi->id) {
		case 0:
		addr = readl(fbi->reg_base + LCD_SPU_DMA_START_ADDR_Y0);
		break;
		case 1:
		addr = readl(fbi->reg_base + LCD_TVD_START_ADDR_Y0);
		break;
		case 2:
		addr = readl(fbi->reg_base + PN2_LCD_DMA_START_ADDR_Y0);
		break;
	}
	switch (fbi->surface.viewPortInfo.rotation)
	{
		case 90:
		break;
		case 180:
		addr += width * height * bpp - 1;
		break;
		case 270:
		addr += height * (width - 1) * bpp;
		break;
	}
	return addr;

}
static int __get_vdma_sz(struct pxa168fb_info *fbi, int width, int height, int bpp)
{
	if (fbi->surface.videoMode == FB_VMODE_YUV422PACKED_IRE_90_270)
		bpp = 2;
	switch (fbi->surface.viewPortInfo.rotation)
	{
		case 1:
			return width * bpp | height << 16;
		case 90:
			return width << 16 | height;
		case 180:
			return width | height << 16;
		case 270:
			return width << 16 | height;
	}
	return 0;

}
static int __get_vdma_pitch(struct pxa168fb_info *fbi, int width, int height, int bpp)
{
	int src_bpp = bpp;

	if (fbi->surface.videoMode == FB_VMODE_YUV422PACKED_IRE_90_270)
		src_bpp = 2;
	switch (fbi->surface.viewPortInfo.rotation)
	{
		case 1:
			return width * src_bpp | (width * bpp << 16);
		case 90:
			return (width * bpp) << 16 | (height * src_bpp);
		case 180:
			return width * src_bpp | (width * bpp << 16);
		case 270:
			return (width * bpp) << 16 | (height * src_bpp);
	}
	return 0;

}

static int __get_vdma_ctrl(struct pxa168fb_info *fbi, int line)
{
	if (fbi->surface.viewPortInfo.rotation == 1)
		return (line << 8) | 0xa1;
	else
		return (line << 8) | 0xa5;
}
int pxa688fb_vdma_get_linenum(struct pxa168fb_info *fbi)
{
	int mulfactor, lines, lines_exp;
	int angle = fbi->surface.viewPortInfo.rotation;
	int pitch = lcd_pitch_read(fbi);
	int height = lcd_height_read(fbi);

	if (angle == 1)
	{
		mulfactor = 2;
	} else {
		mulfactor = 16;
	}
	lines = (sram_freebytes / pitch)&(~(mulfactor-1));

	if (lines < 2)  return 0; /* at least 2 lines*/
	if (lines > 64) lines = 64;

	for (lines_exp = 0; lines_exp < lines; lines_exp += mulfactor)
	{
		if (height%(lines-lines_exp) == 0)
		{
			break;
		}
	}
	if (lines_exp >= lines)
	{
		return 32;
	}
	lines -= lines_exp;
	return lines;
}
void pxa688fb_vdma_set(struct pxa168fb_info *fbi, u32 psqu, unsigned int lines, unsigned int layer)
{
	unsigned int reg, width, height, bpp;

	width = lcd_width_read(fbi);
	height = lcd_height_read(fbi);
	bpp = lcd_pitch_read(fbi) / width;
	if (!psqu || !fbi->surface.viewPortInfo.rotation) {
		pxa688fb_vdma_release(fbi);
	} else {
		reg = readl(fbi->reg_base + LCD_PN2_SQULN2_CTRL);
		switch (fbi->id) {
			case 0:
				reg |= 1 << 24;
				break;
			case 1:
				reg |= 1 << 25;
				break;
			case 2:
				reg |= 1 << 26;
				break;
			default:
				break;
		}
		writel(reg, fbi->reg_base + LCD_PN2_SQULN2_CTRL);
	}
	reg = (u32)psqu | ((lines/2-1)<<1 | 0x1);
	/*reg = (u32)psqu | ((lines/2-1)<<1);*/
	switch(fbi->id) {
		case 0:
			writel(reg, fbi->reg_base + LCD_SQULN1_CTRL);
			break;
		case 1:
			writel(reg, fbi->reg_base + LCD_SQULN2_CTRL);
			break;
		case 2:
			writel(reg, fbi->reg_base + LCD_PN2_SQULN1_CTRL);
			break;
	}
	writel((u32)psqu, fbi->reg_base + ((fbi->id == 0) ? VDMA_DA_1 : VDMA_DA_2));
	/*set VDMA_SRC_SZ*/
	reg = __get_vdma_src_sz(fbi, width, height);
	writel(reg, fbi->reg_base + ((fbi->id == 0) ? VDMA_SRC_SZ_1 : VDMA_SRC_SZ_2));
	/*set VDMA_SA*/
	reg = __get_vdma_sa(fbi, width, height, bpp);
	writel(reg, fbi->reg_base + ((fbi->id == 0) ? VDMA_SA_1 : VDMA_SA_2));
	/*set VDMA_SZ*/
	reg = __get_vdma_sz(fbi, width, height, bpp);
	writel(reg, fbi->reg_base + ((fbi->id == 0) ? VDMA_SZ_1 : VDMA_SZ_2));
	/*set VDMA_PITCH*/
	reg = __get_vdma_pitch(fbi, width, height, bpp);
	writel(reg, fbi->reg_base + ((fbi->id == 0) ? VDMA_PITCH_1 : VDMA_PITCH_2));
	/*set VDMA_ROT_CTRL*/
	reg = __get_vdma_rot_ctrl(fbi->surface.videoMode,
			fbi->surface.viewPortInfo.rotation,
			fbi->surface.viewPortInfo.yuv_format);
	writel(reg, fbi->reg_base + ((fbi->id == 0) ? VDMA_ROT_CTRL_1 : VDMA_ROT_CTRL_2));
	if (fbi->surface.videoMode == FB_VMODE_YUV422PACKED_SWAPYUorV &&
			fbi->surface.viewPortInfo.rotation == 180)
	{
		reg = dma_ctrl_read(fbi, 0);
		reg &= ~(0x1 << 2);
		dma_ctrl_write(fbi, 0, reg);
	}
	/*set VDMA_CTRL*/
	reg = __get_vdma_ctrl(fbi, lines);
	vdma_ctrl_write(fbi, reg);

}

#endif
