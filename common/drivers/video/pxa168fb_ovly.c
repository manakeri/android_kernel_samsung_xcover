/*
 * linux/drivers/video/pxa168fb_ovly.c -- Marvell PXA168 LCD Controller
 *
 * Copyright (C) Marvell Semiconductor Company.  All rights reserved.
 *
 * 2009-03-19   adapted from original version for PXA168
 *		Green Wan <gwan@marvell.com>
 *		Kevin Liu <kliu5@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

/*
 * 1. Adapted from:  linux/drivers/video/skeletonfb.c
 * 2. Merged from: linux/drivers/video/dovefb.c (Lennert Buytenhek)
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/console.h>
#include <linux/timer.h>

#define OVLY_TASKLET

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/pxa168fb.h>

#include "pxa168fb.h"

#ifdef OVLY_DVFM_CONSTRAINT
static int dvfm_dev_idx;
#include <mach/dvfm.h>
#endif

#define RESET_BUF	0x1
#define FREE_ENTRY	0x2

static int pxa168fb_set_par(struct fb_info *fi);
static void set_graphics_start(struct fb_info *fi, int xoffset, int yoffset);
static void set_dma_control0(struct pxa168fb_info *fbi);
static int wait_for_vsync(struct pxa168fb_info *fbi);
static int pxa168fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fi);
static void clearFilterBuf(u8 *bufList[][3], int iFlag);
static void collectFreeBuf(struct pxa168fb_info *fbi,
	u8 *filterList[][3], u8 **freeList);

/* buffer management:
 *    filterBufList: list return to upper layer which indicates buff is free
 *    freelist: list indicates buff is free
 *    waitlist: wait queue which indicates "using" buffer, will be writen in
 *              DMA register
 *    current: buffer on showing
 * Operation:
 *    flip: if !waitlist[0] || !waitlist[1] enqueue to waiting list;
 *          else enqueue the  waitlist[0] to freelist, new buf to waitlist
 *    get freelist: return freelist
 *    eof intr: enqueue current to freelist; dequeue waitlist[0] to current;
 *    buffers are protected spin_lock_irq disable/enable
 *    suspend: when lcd is suspend, move all buffers as "switched",
 *             but don't really set hw register.
 */

static unsigned int max_fb_size = 0;
static unsigned int fb_size_from_cmd = 0;

/* Compatibility mode global switch .....
 *
 * This is a secret switch for user space programs that may want to
 * select color spaces and set overlay position through the nonstd
 * element of fb_var_screeninfo This is the way legacy PXA display
 * drivers were used. The mode reverts back to normal mode on driver release.
 *
 * To turn on compatibility with older PXA, set the MSB of nonstd to 0xAA.
 */

static unsigned int COMPAT_MODE;

static struct _sViewPortInfo gViewPortInfo = {
	.srcWidth = 640,	/* video source size */
	.srcHeight = 480,
	.zoomXSize = 640,	/* size after zooming */
	.zoomYSize = 480,
};

static struct _sViewPortOffset gViewPortOffset = {
	.xOffset = 0,	/* position on screen */
	.yOffset = 0
};

static struct fbi_info ovly_info;

#ifdef FB_PM_DEBUG
static unsigned int g_regs[1024];
static unsigned int g_regs1[1024];
static unsigned int pxa168fb_rw_all_regs(struct pxa168fb_info *fbi,
		unsigned int *regs, int is_read)
{
	u32 i;
	u32 reg;

	for (i = 0xC0; i <= 0x01C4; i += 4) {
		if (is_read) {
			reg = readl(fbi->reg_base + i);
			regs[i] = reg;
		} else {
			writel(regs[i], fbi->reg_base + i);
		}
	}

	return 0;
}
#endif

#if 0
static struct fb_videomode *
find_best_mode(struct pxa168fb_info *fbi, struct fb_var_screeninfo *var)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_videomode *best_mode;
	int i;

	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
	best_mode = NULL;
	for (i = 0; i < mi->num_modes; i++) {
		struct fb_videomode *m = mi->modes + i;

		/*
		 * Check whether this mode is suitable.
		 */
		if (var->xres > m->xres)
			continue;
		if (var->yres > m->yres)
			continue;

		/*
		 * Check whether this mode is more suitable than
		 * the best mode so far.
		 */
		if (best_mode != NULL &&
		    (best_mode->xres < m->xres ||
		     best_mode->yres < m->yres ||
		     best_mode->pixclock > m->pixclock))
			continue;

		best_mode = m;
	}

	return best_mode;
}
#endif

static int determine_best_pix_fmt(struct fb_var_screeninfo *var)
{
	unsigned char pxa_format;

	/* compatibility switch: if var->nonstd MSB is 0xAA then skip to
	 * using the nonstd variable to select the color space.
	 */
	if(COMPAT_MODE != 0x2625) {

		/*
		 * Pseudocolor mode?
		 */
		if (var->bits_per_pixel == 8)
			return PIX_FMT_PSEUDOCOLOR;
		/*
		 * Check for YUV422PACK.
		 */
		if (var->bits_per_pixel == 16 && var->red.length == 16 &&
		    var->green.length == 16 && var->blue.length == 16) {
			if (var->red.offset >= var->blue.offset) {
				if (var->red.offset == 4)
					return PIX_FMT_YUV422PACK;
				else
					return PIX_FMT_YUYV422PACK;
			} else
				return PIX_FMT_YVU422PACK;
		}
		/*
		 * Check for YUV422PLANAR.
		 */
		if (var->bits_per_pixel == 16 && var->red.length == 8 &&
		    var->green.length == 4 && var->blue.length == 4) {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMT_YUV422PLANAR;
			else
				return PIX_FMT_YVU422PLANAR;
		}

		/*
		 * Check for YUV420PLANAR.
		 */
		if (var->bits_per_pixel == 12 && var->red.length == 8 &&
		    var->green.length == 2 && var->blue.length == 2) {
			if (var->red.offset >= var->blue.offset)
				return PIX_FMT_YUV420PLANAR;
			else
				return PIX_FMT_YVU420PLANAR;
		}
		/*
		 * Check for 565/1555.
		 */
		if (var->bits_per_pixel == 16 && var->red.length <= 5 &&
		    var->green.length <= 6 && var->blue.length <= 5) {
			if (var->transp.length == 0) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB565;
				else
					return PIX_FMT_BGR565;
			}

			if (var->transp.length == 1 && var->green.length <= 5) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB1555;
				else
					return PIX_FMT_BGR1555;
			}

			/* fall through */
		}

		/*
		 * Check for 888/A888.
		 */
		if (var->bits_per_pixel <= 32 && var->red.length <= 8 &&
		    var->green.length <= 8 && var->blue.length <= 8) {
			if (var->bits_per_pixel == 24 && var->transp.length == 0) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB888PACK;
				else
					return PIX_FMT_BGR888PACK;
			}

			if (var->bits_per_pixel == 32 && var->transp.offset == 24) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGBA888;
				else
					return PIX_FMT_BGRA888;
			} else {
				if (var->transp.length == 8) {
					if (var->red.offset >= var->blue.offset)
						return PIX_FMT_RGB888UNPACK;
					else
						return PIX_FMT_BGR888UNPACK;
				} else
					return PIX_FMT_YUV422PACK_IRE_90_270;

			}
			/* fall through */
		}
	} else {

		pxa_format = (var->nonstd >> 20) & 0xf;

		switch (pxa_format) {
		case 0:
			return PIX_FMT_RGB565;
			break;
		case 3:
			return PIX_FMT_YUV422PLANAR;
			break;
		case 4:
			return PIX_FMT_YUV420PLANAR;
			break;
		case 5:
			return PIX_FMT_RGB1555;
			break;
		case 6:
			return PIX_FMT_RGB888PACK;
			break;
		case 7:
			return PIX_FMT_RGB888UNPACK;
			break;
		case 8:
			return PIX_FMT_RGBA888;
			break;
		case 9:
			return PIX_FMT_YUV422PACK;
			break;

		default:
			return -EINVAL;
		}
	}



	return -EINVAL;
}

static int set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt)
{
	switch (pix_fmt) {
	case PIX_FMT_RGB565:
		var->bits_per_pixel = 16;
		var->red.offset = 11;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_BGR565:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 11;   var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_RGB1555:
		var->bits_per_pixel = 16;
		var->red.offset = 10;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 5 << 20;
		break;
	case PIX_FMT_BGR1555:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 10;   var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 5 << 20;
		break;
	case PIX_FMT_RGB888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 6 << 20;
		break;
	case PIX_FMT_BGR888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 6 << 20;
		break;
	case PIX_FMT_RGB888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 7 << 20;
		break;
	case PIX_FMT_BGR888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 7 << 20;
		break;
	case PIX_FMT_RGBA888:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 8 << 20;
		break;
	case PIX_FMT_BGRA888:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 8 << 20;
		break;
	case PIX_FMT_YUYV422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 8;     var->red.length = 16;
		var->green.offset = 4;   var->green.length = 16;
		var->blue.offset = 0;   var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 9 << 20;
		break;
	case PIX_FMT_YVU422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 16;
		var->green.offset = 8;   var->green.length = 16;
		var->blue.offset = 12;   var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 9 << 20;
		break;
	case PIX_FMT_YUV422PLANAR:
		var->bits_per_pixel = 16;
		var->red.offset = 8;	 var->red.length = 8;
		var->green.offset = 4;   var->green.length = 4;
		var->blue.offset = 0;   var->blue.length = 4;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 3 << 20;
		break;
	case PIX_FMT_YVU422PLANAR:
		var->bits_per_pixel = 16;
		var->red.offset = 0;	 var->red.length = 8;
		var->green.offset = 8;   var->green.length = 4;
		var->blue.offset = 12;   var->blue.length = 4;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 3 << 20;
		break;
	case PIX_FMT_YUV420PLANAR:
		var->bits_per_pixel = 12;
		var->red.offset = 4;	 var->red.length = 8;
		var->green.offset = 2;   var->green.length = 2;
		var->blue.offset = 0;   var->blue.length = 2;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 4 << 20;
		break;
	case PIX_FMT_YVU420PLANAR:
		var->bits_per_pixel = 12;
		var->red.offset = 0;	 var->red.length = 8;
		var->green.offset = 8;   var->green.length = 2;
		var->blue.offset = 10;   var->blue.length = 2;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 4 << 20;
		break;

	case PIX_FMT_PSEUDOCOLOR:
		var->bits_per_pixel = 8;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 0;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		break;
	case PIX_FMT_YUV422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 4;     var->red.length = 16;
		var->green.offset = 12;   var->green.length = 16;
		var->blue.offset = 0;    var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 9 << 20;
		break;
	case PIX_FMT_YUV422PACK_IRE_90_270:	/* YUV422 Packed will be YUV444 Packed after IRE 90 and 270 degree rotation*/
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 7 << 20;
		break;
	default:
		return  -EINVAL;
	}
	return 0;
}

static int convert_pix_fmt(u32 vmode)
{
/*	printk(KERN_INFO "vmode=%d\n", vmode); */
	switch(vmode) {
	case FB_VMODE_YUV422PACKED:
		return PIX_FMT_YUV422PACK;
	case FB_VMODE_YUV422PACKED_SWAPUV:
		return PIX_FMT_YVU422PACK;
	case FB_VMODE_YUV422PLANAR:
		return PIX_FMT_YUV422PLANAR;
	case FB_VMODE_YUV422PLANAR_SWAPUV:
		return PIX_FMT_YVU422PLANAR;
	case FB_VMODE_YUV420PLANAR:
		return PIX_FMT_YUV420PLANAR;
	case FB_VMODE_YUV420PLANAR_SWAPUV:
		return PIX_FMT_YVU420PLANAR;
	case FB_VMODE_YUV422PACKED_SWAPYUorV:
		return PIX_FMT_YUYV422PACK;
	case FB_VMODE_YUV422PACKED_IRE_90_270:
		return PIX_FMT_YUV422PACK_IRE_90_270;
	case FB_VMODE_RGB565:
		return PIX_FMT_RGB565;
	case FB_VMODE_BGR565:
		return PIX_FMT_BGR565;
	case FB_VMODE_RGB1555:
		return PIX_FMT_RGB1555;
	case FB_VMODE_BGR1555:
		return PIX_FMT_BGR1555;
	case FB_VMODE_RGB888PACK:
		return PIX_FMT_RGB888PACK;
	case FB_VMODE_BGR888PACK:
		return PIX_FMT_BGR888PACK;
	case FB_VMODE_RGBA888:
		return PIX_FMT_RGBA888;
	case FB_VMODE_BGRA888:
		return PIX_FMT_BGRA888;
	case FB_VMODE_RGB888UNPACK:
	case FB_VMODE_BGR888UNPACK:
	case FB_VMODE_YUV422PLANAR_SWAPYUorV:
	case FB_VMODE_YUV420PLANAR_SWAPYUorV:
	default:
		return -1;
	}
}

static void pxa168_sync_colorkey_structures(struct pxa168fb_info *fbi, int direction)
{
	struct _sColorKeyNAlpha *colorkey = &fbi->ckey_alpha;
	struct pxa168_fb_chroma *chroma = &fbi->chroma;
	unsigned int temp;

	dev_dbg(fbi->fb_info->dev, "ENTER %s\n", __FUNCTION__);
	if (direction == FB_SYNC_COLORKEY_TO_CHROMA) {
		chroma->mode        = colorkey->mode;
		chroma->y_alpha     = (colorkey->Y_ColorAlpha) & 0xff;
		chroma->y           = (colorkey->Y_ColorAlpha >> 8) & 0xff;
		chroma->y1          = (colorkey->Y_ColorAlpha >> 16) & 0xff;
		chroma->y2          = (colorkey->Y_ColorAlpha >> 24) & 0xff;

		chroma->u_alpha     = (colorkey->U_ColorAlpha) & 0xff;
		chroma->u           = (colorkey->U_ColorAlpha >> 8) & 0xff;
		chroma->u1          = (colorkey->U_ColorAlpha >> 16) & 0xff;
		chroma->u2          = (colorkey->U_ColorAlpha >> 24) & 0xff;

		chroma->v_alpha     = (colorkey->V_ColorAlpha) & 0xff;
		chroma->v           = (colorkey->V_ColorAlpha >> 8) & 0xff;
		chroma->v1          = (colorkey->V_ColorAlpha >> 16) & 0xff;
		chroma->v2          = (colorkey->V_ColorAlpha >> 24) & 0xff;
	}


	if (direction == FB_SYNC_CHROMA_TO_COLORKEY) {

		colorkey->mode = chroma->mode;
		temp = chroma->y_alpha;
		temp |= chroma->y << 8;
		temp |= chroma->y1 << 16;
		temp |= chroma->y2 << 24;
		colorkey->Y_ColorAlpha = temp;

		temp = chroma->u_alpha;
		temp |= chroma->u << 8;
		temp |= chroma->u1 << 16;
		temp |= chroma->u2 << 24;
		colorkey->U_ColorAlpha = temp;

		temp = chroma->v_alpha;
		temp |= chroma->v << 8;
		temp |= chroma->v1 << 16;
		temp |= chroma->v2 << 24;
		colorkey->V_ColorAlpha = temp;

	}
}

static u32 pxa168fb_ovly_set_colorkeyalpha(struct pxa168fb_info *fbi)
{
	struct _sColorKeyNAlpha *color_a = &fbi->ckey_alpha;
	unsigned int rb, x, layer, dma0, shift, r, b;
	struct pxa168fb_mach_info *mi;
	struct lcd_regs *regs;

	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
again:
	mi = fbi->dev->platform_data;
	regs = get_regs(fbi);
	dma0 = dma_ctrl_read(fbi, 0);
	shift = fbi->id ? 20 : 18;
	rb = layer = 0;
	r = color_a->Y_ColorAlpha;
	b = color_a->V_ColorAlpha;

	/* reset to 0x0 to disable color key. */
	x = dma_ctrl_read(fbi, 1) & ~(CFG_COLOR_KEY_MASK |
			CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK);

	/* switch to color key mode */
	switch (color_a->mode) {
	case FB_DISABLE_COLORKEY_MODE:
		/* do nothing */
		break;
	case FB_ENABLE_Y_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x1);
		break;
	case FB_ENABLE_U_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x2);
		break;
	case FB_ENABLE_V_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x4);
		pr_info("V colorkey not supported, Chroma key instead\n");
		break;
	case FB_ENABLE_RGB_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x3);
		rb = 1;
		break;
	case FB_ENABLE_R_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x1);
		rb = 1;
		break;
	case FB_ENABLE_G_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x6);
		pr_info("G colorkey not supported, Luma key instead\n");
		break;
	case FB_ENABLE_B_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x7);
		rb = 1;
		break;
	default:
		printk(KERN_INFO "unknown mode");
		return -1;
	}


	/* switch to alpha path selection */
	switch (color_a->alphapath) {
	case FB_VID_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x0);
		layer = CFG_CKEY_DMA;
		if (rb)
			rb = ((dma0 & CFG_DMA_SWAPRB_MASK)>> 4) ^
				(mi->panel_rbswap);
		break;
	case FB_GRA_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x1);
		layer = CFG_CKEY_GRA;
		if(rb)
			rb = ((dma0 & CFG_GRA_SWAPRB_MASK)>> 12) ^
				(mi->panel_rbswap);
		break;
	case FB_CONFIG_ALPHA:
		x |= CFG_ALPHA_MODE(0x2);
		rb = 0;
		break;
	default:
		printk(KERN_INFO "unknown alpha path");
		return -1;
	}

	/* check whether DMA turn on RB swap for this pixelformat. */
	if (rb) {
		if (color_a->mode == FB_ENABLE_R_COLORKEY_MODE) {
			x &= ~CFG_COLOR_KEY_MODE(0x1);
			x |= CFG_COLOR_KEY_MODE(0x7);
		}

		if (color_a->mode == FB_ENABLE_B_COLORKEY_MODE) {
			x &= ~CFG_COLOR_KEY_MODE(0x7);
			x |= CFG_COLOR_KEY_MODE(0x1);
		}

		/* exchange r b fields. */
		r = color_a->V_ColorAlpha;
		b = color_a->Y_ColorAlpha;

		/* only alpha_Y take effect, switch back from V */
		if (color_a->mode == FB_ENABLE_RGB_COLORKEY_MODE) {
			r &= 0xffffff00;
			r |= (color_a->Y_ColorAlpha & 0xff);
		}
	}

	/* configure alpha */
	x |= CFG_ALPHA((color_a->config & 0xff));
	dma_ctrl_write(fbi, 1, x);
	writel(r, &regs->v_colorkey_y);
	writel(color_a->U_ColorAlpha, &regs->v_colorkey_u);
	writel(b, &regs->v_colorkey_v);

	if (fbi->id != 2) {
		/* enable DMA colorkey on graphics/video layer in panel/TV path */
		x = readl(fbi->reg_base + LCD_TV_CTRL1);
		x &= ~(3<<shift); x |= layer<<shift;
		writel(x, fbi->reg_base + LCD_TV_CTRL1);
	}

	if (FB_MODE_DUP) {
		fbi = ovly_info.fbi[fb_dual];
		goto again;

#if 0
		if (debug_ck) {
			printk("\n\n   fb1 dma1 0x%x\n\n", dma_ctrl_read(fbi, 1));
			printk(" fb1 rb 0x%x colorkey @ 0x%p y: 0x%x; u: 0x%x; v: 0x%x\n\n",
				rb, &regs->v_colorkey_y, readl(&regs->v_colorkey_y),
				readl(&regs->v_colorkey_u), readl(&regs->v_colorkey_v));
		}
#endif
	}

	return 0;
}

static int check_surface_addr(struct fb_info *fi,
			struct _sVideoBufferAddr *new_addr)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	int changed = 0;

	dev_dbg(fi->dev, "Enter %s\n", __func__);

	/* Check buffer address */
	if (new_addr && new_addr->startAddr[0] &&
	    fbi->new_addr[0] != (unsigned long)new_addr->startAddr[0]) {
		fbi->new_addr[0] = (unsigned long)new_addr->startAddr[0];
		fbi->new_addr[1] = (unsigned long)new_addr->startAddr[1];
		fbi->new_addr[2] = (unsigned long)new_addr->startAddr[2];
		changed = 1;
	}

	return changed;
}

static int check_surface(struct fb_info *fi,
			FBVideoMode new_mode,
			struct _sViewPortInfo *new_info,
			struct _sViewPortOffset *new_offset,
			struct _sVideoBufferAddr *new_addr)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &fi->var;
	int changed = 0;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);

	/*
	 * check mode
	 */
	if (new_mode >= 0 && fbi->surface.videoMode != new_mode) {
		fbi->surface.videoMode = new_mode;
		fbi->pix_fmt = convert_pix_fmt(new_mode);
		set_pix_fmt(var, fbi->pix_fmt);
		changed = 1;
	}
	/*
	 * check view port settings.
	 */
	if (new_info &&
	    (fbi->surface.viewPortInfo.srcWidth != new_info->srcWidth ||
		 fbi->surface.viewPortInfo.srcHeight != new_info->srcHeight ||
		 fbi->surface.viewPortInfo.zoomXSize != new_info->zoomXSize ||
	     fbi->surface.viewPortInfo.zoomYSize != new_info->zoomYSize ||
		fbi->surface.viewPortInfo.yPitch != new_info->yPitch ||
		fbi->surface.viewPortInfo.uPitch != new_info->uPitch ||
		fbi->surface.viewPortInfo.vPitch != new_info->vPitch ||
		fbi->surface.viewPortInfo.rotation != new_info->rotation ||
		fbi->surface.viewPortInfo.yuv_format != new_info->yuv_format)) {
		if (!(new_addr && new_addr->startAddr[0])) {
			if (mi->mmap && (((new_info->srcWidth *
			new_info->srcHeight * var->bits_per_pixel / 8) * 2)
			> fbi->fb_size)) {
				pr_err("%s: requested memory buffer size %d"
					"exceed the max limit %d!\n", __func__,
				(new_info->srcWidth * new_info->srcHeight
				 * var->bits_per_pixel / 4), fbi->fb_size);
				return changed;
			}
		}
		var->xres_virtual = new_info->srcWidth;
		var->yres_virtual = new_info->srcHeight * 2;
		var->xres = new_info->srcWidth;
		var->yres = new_info->srcHeight;
		fbi->surface.viewPortInfo = *new_info;
		changed = 1;
	}

	/*
	 * Check offset
	 */
	if (new_offset &&
	    (fbi->surface.viewPortOffset.xOffset != new_offset->xOffset ||
	    fbi->surface.viewPortOffset.yOffset != new_offset->yOffset)) {
		fbi->surface.viewPortOffset.xOffset = new_offset->xOffset;
		fbi->surface.viewPortOffset.yOffset = new_offset->yOffset;
		changed = 1;
	}
	/*
	 * Check buffer address
	 */
	if (new_addr && new_addr->startAddr[0] &&
	    fbi->new_addr[0] != (unsigned long)new_addr->startAddr[0]) {
		/*check overlay buffer address and pitch alignment*/
		if (((unsigned long)new_addr->startAddr[0] & 63) &&
			(fbi->surface.viewPortInfo.yPitch & 7) &&
			(fbi->surface.viewPortInfo.uPitch & 7)) {
			printk(KERN_WARNING "Ovly: the memory base 0x%08lx is"
			" not 64 bytes aligned, pitch is not 8 bytes aligned,"
			" video playback maybe wrong!\n",
			(unsigned long)new_addr->startAddr[0]);
		}

		fbi->new_addr[0] = (unsigned long)new_addr->startAddr[0];
		fbi->new_addr[1] = (unsigned long)new_addr->startAddr[1];
		fbi->new_addr[2] = (unsigned long)new_addr->startAddr[2];
		changed = 1;
	}

	return changed;
}

static int pxa168fb_update_buff(struct fb_info *fi,
	struct _sOvlySurface *surface, int addr)
{
	if (addr) {
		/* update buffer address only if changed */
		if (check_surface_addr(fi, &surface->videoBufferAddr))
			set_graphics_start(fi, 0, 0);
		else
			return -EINVAL;
	} else if (check_surface(fi, surface->videoMode,
					&surface->viewPortInfo,
					&surface->viewPortOffset,
					&surface->videoBufferAddr))
		/* update other parameters other than buf addr */
		return pxa168fb_set_par(fi);

	return 0;
}

static void pxa168fb_clear_framebuffer(struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;

	memset(fbi->fb_start, 0, fbi->fb_size);
}

static int check_modex_active(int id, int on)
{
	int active = on;

	/* mode2, base off, dual on */
	if ((fb_mode == 2) && (id == fb_base))
		active = 0;

	/* mode3, dual off, base on */
	if ((fb_mode == 3) && (id == fb_dual))
		active = 0;

	pr_debug("%s fb_mode %d fbi[%d] on %d active %d\n",
		__func__, fb_mode, id, on, active);
	return active;
}

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
static void debug_identify_called_ioctl(struct fb_info *fi, int cmd, unsigned long arg)
{
	switch (cmd) {
	case FB_IOCTL_CLEAR_FRAMEBUFFER:
		dev_dbg(fi->dev," FB_IOCTL_CLEAR_FRAMEBUFFER\n");
		break;
	case FB_IOCTL_WAIT_VSYNC:
		dev_dbg(fi->dev," FB_IOCTL_WAIT_VSYNC\n");
		break;
	case FB_IOCTL_GET_VIEWPORT_INFO:
		dev_dbg(fi->dev," FB_IOCTL_GET_VIEWPORT_INFO with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SET_VIEWPORT_INFO:
		dev_dbg(fi->dev," FB_IOCTL_SET_VIEWPORT_INFO with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SET_VIDEO_MODE:
		dev_dbg(fi->dev," FB_IOCTL_SET_VIDEO_MODE with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_GET_VIDEO_MODE:
		dev_dbg(fi->dev," FB_IOCTL_GET_VIDEO_MODE with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_FLIP_VID_BUFFER:
		dev_dbg(fi->dev," FB_IOCTL_FLIP_VID_BUFFER with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_GET_FREELIST:
		dev_dbg(fi->dev," FB_IOCTL_GET_FREELIST with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_GET_BUFF_ADDR:
		dev_dbg(fi->dev," FB_IOCTL_GET_BUFF_ADDR with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SET_VID_OFFSET:
		dev_dbg(fi->dev," FB_IOCTL_SET_VID_OFFSET with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_GET_VID_OFFSET:
		dev_dbg(fi->dev," FB_IOCTL_GET_VID_OFFSET with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SET_MEMORY_TOGGLE:
		dev_dbg(fi->dev," FB_IOCTL_SET_MEMORY_TOGGLE with arg = %08x\n",(unsigned int) arg);
		break;
	case FB_IOCTL_SET_COLORKEYnALPHA:
		dev_dbg(fi->dev," FB_IOCTL_SET_COLORKEYnALPHA with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_GET_CHROMAKEYS:
		dev_dbg(fi->dev," FB_IOCTL_GET_CHROMAKEYS with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_CHROMAKEYS:
		dev_dbg(fi->dev," FB_IOCTL_PUT_CHROMAKEYS with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_GET_COLORKEYnALPHA:
		dev_dbg(fi->dev," FB_IOCTL_GET_COLORKEYnALPHA with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SWITCH_VID_OVLY:
		dev_dbg(fi->dev," FB_IOCTL_SWITCH_VID_OVLY with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SWITCH_GRA_OVLY:
		dev_dbg(fi->dev," FB_IOCTL_SWITCH_GRA_OVLY with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SWAP_VIDEO_RED_BLUE:
		dev_dbg(fi->dev," FB_IOCTL_PUT_SWAP_VIDEO_RED_BLUE with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SWAP_VIDEO_U_V:
		dev_dbg(fi->dev," FB_IOCTL_PUT_SWAP_VIDEO_U_V with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_SWAP_VIDEO_Y_UV:
		dev_dbg(fi->dev," FB_IOCTL_PUT_SWAP_VIDEO_Y_UV with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_VIDEO_ALPHABLEND:
		dev_dbg(fi->dev," FB_IOCTL_PUT_VIDEO_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_GLOBAL_ALPHABLEND:
		dev_dbg(fi->dev," FB_IOCTL_PUT_GLOBAL_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_GRAPHIC_ALPHABLEND:
		dev_dbg(fi->dev," FB_IOCTL_PUT_GRAPHIC_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
		break;

	}
}
#endif

static int rotation_unsupport_format(struct _sViewPortInfo viewPortInfo,
					FBVideoMode videoMode)
{
	/*if VDMA is not used, viewPortInfo.rotation should be 0*/
	if ((viewPortInfo.rotation == 0) || (viewPortInfo.rotation == 1))
		return 0;

	if (viewPortInfo.srcHeight == 1080) {
		pr_err("Error:1080P rotation is not supported!\n");
		return 1;
	}
	if (viewPortInfo.srcHeight == 720 && viewPortInfo.rotation == 180) {
		pr_err("Error:720p rotation 180 is not supported!\n");
		return 1;
	}
	switch (videoMode) {
	case FB_VMODE_YUV422PLANAR:
	case FB_VMODE_YUV422PLANAR_SWAPUV:
	case FB_VMODE_YUV422PLANAR_SWAPYUorV:
	case FB_VMODE_YUV420PLANAR:
	case FB_VMODE_YUV420PLANAR_SWAPUV:
	case FB_VMODE_YUV420PLANAR_SWAPYUorV:
		pr_err("Error:Planar is not supported!\n");
		return 1;
	default:
		break;
	}
	return 0;
}

static void ovlysurface_clear_pitch(struct _sOvlySurface *surface)
{
	surface->viewPortInfo.yPitch = 0;
	surface->viewPortInfo.uPitch = 0;
	surface->viewPortInfo.vPitch = 0;
}

static u8 *buf_dequeue(u8 **ppBufList)
{
	int i;
	u8 *pBuf;

	if (!ppBufList) {
		printk(KERN_ALERT "%s: invalid list\n", __func__);
		return NULL;
	}

	pBuf = ppBufList[0];

	for (i = 1; i < MAX_QUEUE_NUM; i++) {
		if (!ppBufList[i]) {
			ppBufList[i-1] = 0;
			break;
		}
		ppBufList[i-1] = ppBufList[i];
	}

	if (i >= MAX_QUEUE_NUM)
		printk(KERN_ALERT "%s: buffer overflow\n",  __func__);

	return pBuf;
}

static int buf_enqueue(u8 **ppBufList, u8 *pBuf)
{
	int i = 0 ;
	struct _sOvlySurface *srf0 = (struct _sOvlySurface *)(pBuf);
	struct _sOvlySurface *srf1 = 0;

	/* Error checking */
	if (!srf0)
		return -1;

	for (; i < MAX_QUEUE_NUM; i++) {
		srf1 = (struct _sOvlySurface *)ppBufList[i];
		if (!srf1) {
			ppBufList[i] = pBuf;
			return 0;
		}
	}

	if (i >= MAX_QUEUE_NUM)
		printk(KERN_INFO "buffer overflow\n");

	return -3;
}

static void buf_clear(u8 **ppBufList, int iFlag)
{
	int i = 0;

	/* Check null pointer. */
	if (!ppBufList)
		return;

	/* free */
	if (FREE_ENTRY & iFlag) {
		for (i = 0; i < MAX_QUEUE_NUM; i++) {
			if (ppBufList && ppBufList[i])
				kfree(ppBufList[i]);
		}
	}

	if (RESET_BUF & iFlag)
		memset(ppBufList, 0, MAX_QUEUE_NUM * sizeof(u8 *));
}

#ifdef OVLY_TASKLET
static void pxa168fb_ovly_task(unsigned long data)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)data;
#else
static void pxa168fb_ovly_work(struct work_struct *w)
{
	struct pxa168fb_info *fbi = container_of(w,
		struct pxa168fb_info, buf_work);
#endif
	unsigned long flags;

	if (fbi->debug)
		printk(KERN_DEBUG"%s fbi %d buf_retired %p\n",
			__func__, fbi->id, fbi->buf_retired);

	if (fbi->buf_retired) {
		/* enqueue current to freelist */
		spin_lock_irqsave(&fbi->buf_lock, flags);
		buf_enqueue(fbi->buf_freelist, fbi->buf_retired);
		fbi->buf_retired = 0;
		ovly_info.wait_peer = 0;
		spin_unlock_irqrestore(&fbi->buf_lock, flags);
	}
}

static u8 *buf_gethead(u8 **ppBufList)
{
	u8 *pBuf;

	if (!ppBufList)
		return NULL;

	pBuf = ppBufList[0];

	return pBuf;
}

static void buf_endframe(void *p)
{
	struct fb_info *fi = (struct fb_info *)p;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct _sOvlySurface *pOvlySurface;
	int ret;
	u8 *buf = buf_gethead(fbi->buf_waitlist);

	if (buf && fbi->buf_retired) {
		ovly_info.retire_err++;
	} else {
		buf = buf_dequeue(fbi->buf_waitlist);
		if (buf) {
			pOvlySurface = (struct _sOvlySurface *)buf;

			/* Update new surface settings */
			ret = pxa168fb_update_buff(fi, pOvlySurface, 0);

			if (!ret) {
				fbi->buf_retired = fbi->buf_current;
				fbi->buf_current = buf;
				if (ovly_info.flag && FB_MODE_DUP)
					ovly_info.wait_peer = 1;
				else {
#ifdef OVLY_TASKLET
					tasklet_schedule(&fbi->ovly_task);
#else
					if (fbi->system_work)
						schedule_work(&fbi->buf_work);
					else
						queue_work(fbi->work_q,
							&fbi->fb_info->queue);
#endif
				}
			} else {
				/* enqueue the repeated buffer to freelist */
				buf_enqueue(fbi->buf_freelist, buf);
				pr_info("Detect a same surface flipped in, "
					"may flicker.\n");
			}
		}
	}
}

static int pxa168fb_ovly_ioctl(struct fb_info *fi, unsigned int cmd,
			unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int vmode = 0;
	int gfx_on = 1;
	int vid_on = 1;
	int val = 0, mask = 0;
	unsigned char param;
	int blendval = 0;
	int res,tmp;
	int ret = 0;
	unsigned long flags;

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
	debug_identify_called_ioctl(fi, cmd, arg);
#endif

	switch (cmd) {
	case FB_IOCTL_CLEAR_FRAMEBUFFER:
		if (!mi->mmap)
			return -EINVAL;
		pxa168fb_clear_framebuffer(fi);
		return 0;
		break;
	case FB_IOCTL_WAIT_VSYNC:
		wait_for_vsync(fbi);
		break;
	case FB_IOCTL_GET_VIEWPORT_INFO:/*if rotate 90/270, w/h swap*/
		mutex_lock(&fbi->access_ok);
		if (fbi->surface.viewPortInfo.rotation == 90 ||
			fbi->surface.viewPortInfo.rotation == 270) {
			tmp = fbi->surface.viewPortInfo.srcWidth;
			fbi->surface.viewPortInfo.srcWidth =
			fbi->surface.viewPortInfo.srcHeight;
			fbi->surface.viewPortInfo.srcHeight = tmp;
			fbi->surface.viewPortInfo.rotation = 360 -
			fbi->surface.viewPortInfo.rotation;
		}
		res = copy_to_user(argp, &fbi->surface.viewPortInfo,
			sizeof(struct _sViewPortInfo)) ? -EFAULT : 0;
		if (fbi->surface.viewPortInfo.rotation == 90 ||
			fbi->surface.viewPortInfo.rotation == 270) {
			tmp = fbi->surface.viewPortInfo.srcWidth;
			fbi->surface.viewPortInfo.srcWidth =
			fbi->surface.viewPortInfo.srcHeight;
			fbi->surface.viewPortInfo.srcHeight = tmp;
			fbi->surface.viewPortInfo.rotation = 360 -
			fbi->surface.viewPortInfo.rotation;
		}
		mutex_unlock(&fbi->access_ok);
		return res;
	case FB_IOCTL_SET_VIEWPORT_INFO:/*if rotate 90/270, w/h swap*/
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gViewPortInfo, argp,
				sizeof(gViewPortInfo))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		if (rotation_unsupport_format(gViewPortInfo, -1)) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		gViewPortInfo.rotation = (360 - gViewPortInfo.rotation) % 360;
		if (gViewPortInfo.rotation == 90 ||
			gViewPortInfo.rotation == 270) {
			tmp = gViewPortInfo.srcWidth;
			gViewPortInfo.srcWidth =
			gViewPortInfo.srcHeight;
			gViewPortInfo.srcHeight = tmp;
		}

		if (check_surface(fi, -1, &gViewPortInfo, 0, 0))
			pxa168fb_set_par(fi);

		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_SET_VIDEO_MODE:
		/*
		 * Get data from user space.
		 */
		if (copy_from_user(&vmode, argp, sizeof(vmode)))
			return -EFAULT;

		if (check_surface(fi, vmode, 0, 0, 0))
			pxa168fb_set_par(fi);
		break;
	case FB_IOCTL_GET_VIDEO_MODE:
		return copy_to_user(argp, &fbi->surface.videoMode,
			sizeof(u32)) ? -EFAULT : 0;
	case FB_IOCTL_FLIP_VID_BUFFER:
	{
		struct _sOvlySurface *surface = 0;
		u8 *start_addr[3], *input_data;
		u32 length;
		int tmp;
		mutex_lock(&fbi->access_ok);
		surface = kmalloc(sizeof(struct _sOvlySurface),
				GFP_KERNEL);
		if (surface == NULL){
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		/* Get user-mode data. */
		if (copy_from_user(surface, argp,
					sizeof(struct _sOvlySurface))) {
			kfree(surface);
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		if (rotation_unsupport_format(surface->viewPortInfo,
			surface->videoMode)) {
			kfree(surface);
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		if (surface->viewPortInfo.rotation == 90 ||
			surface->viewPortInfo.rotation == 270) {
			surface->viewPortInfo.rotation = 360 -
			surface->viewPortInfo.rotation;
			tmp = surface->viewPortInfo.srcWidth;
			surface->viewPortInfo.srcWidth =
			surface->viewPortInfo.srcHeight;
			surface->viewPortInfo.srcHeight = tmp;
			switch (surface->videoMode) {
			case FB_VMODE_YUV422PACKED:
				surface->videoMode = FB_VMODE_YUV422PACKED_IRE_90_270;
				surface->viewPortInfo.yuv_format = 1;
				ovlysurface_clear_pitch(surface);
				break;
			case FB_VMODE_YUV422PACKED_SWAPUV:
				surface->videoMode = FB_VMODE_YUV422PACKED_IRE_90_270;
				surface->viewPortInfo.yuv_format = 2;
				ovlysurface_clear_pitch(surface);
				break;
			case FB_VMODE_YUV422PACKED_SWAPYUorV:
				surface->videoMode = FB_VMODE_YUV422PACKED_IRE_90_270;
				surface->viewPortInfo.yuv_format = 4;
				ovlysurface_clear_pitch(surface);
				break;
			default:
				surface->viewPortInfo.yuv_format = 0;
			}
		}
		length = surface->videoBufferAddr.length;
		start_addr[0] = surface->videoBufferAddr.startAddr[0];
		input_data = surface->videoBufferAddr.inputData;

		if (fbi->debug)
			printk(KERN_DEBUG"flip surface %p buf %p\n",
					surface, start_addr[0]);
		/*
		 * Has DMA addr?
		 */
		if (start_addr[0] && (!input_data)) {
			spin_lock_irqsave(&fbi->buf_lock, flags);
			/*if there's less than 2 frames in waitlist,
			 *enqueue this frame to waitlist*/
			if ((!fbi->buf_waitlist[0]) ||
				(!fbi->buf_waitlist[1])) {
				buf_enqueue(fbi->buf_waitlist, (u8 *)surface);
			} else {
				/*if there is two frame in waitlist, dequeue
				 *the older frame and enqueue it to freelist,
				 *then enqueue this frame to waitlist*/
				buf_enqueue(fbi->buf_freelist,
						buf_dequeue(fbi->buf_waitlist));
				buf_enqueue(fbi->buf_waitlist, (u8 *)surface);
			}
			spin_unlock_irqrestore(&fbi->buf_lock, flags);
		} else {
			if (!mi->mmap) {
				pr_err("fbi %d(line %d): input err, mmap is not"
					" supported\n", fbi->id, __LINE__);
				kfree(surface);
				mutex_unlock(&fbi->access_ok);
				return -EINVAL;
			}

			/* update buffer */
			pxa168fb_update_buff(fi, surface, 1);

			/* copy buffer */
			if (input_data) {
				wait_for_vsync(fbi);
				/* if support hw DMA, replace this. */
				if (copy_from_user(fbi->fb_start,
							input_data, length)){
					kfree(surface);
					mutex_unlock(&fbi->access_ok);
					return -EFAULT;
				}
				kfree(surface);
				mutex_unlock(&fbi->access_ok);
				return 0;
			}

			/*
			 * if it has its own physical address,
			 * switch to this memory. don't support YUV planar format
			 * with split YUV buffers. but below code seems have no
			 * chancee to execute. - FIXME
			 */
			if (start_addr[0]) {
				if (fbi->mem_status)
					free_pages(
							(unsigned long)fbi->fb_start,
							get_order(fbi->fb_size));
				else
					dma_free_writecombine(fbi->dev,
							fbi->fb_size,
							fbi->fb_start,
							fbi->fb_start_dma);

				fbi->fb_start = __va(start_addr[0]);
				fbi->fb_size = length;
				fbi->fb_start_dma =
					(dma_addr_t)__pa(fbi->fb_start);
				fbi->mem_status = 1;
				fi->fix.smem_start = fbi->fb_start_dma;
				fi->fix.smem_len = fbi->fb_size;
				fi->screen_base = fbi->fb_start;
				fi->screen_size = fbi->fb_size;
			}

			kfree(surface);
		}

		mutex_unlock(&fbi->access_ok);
		return 0;
	}
	case FB_IOCTL_GET_FREELIST:
	{
		if (fbi->debug)
			printk(KERN_DEBUG"get freelist\n");

		if (!ovly_info.flag && FB_MODE_DUP)
			wait_for_vsync(ovly_info.fbi[fb_dual]);

		spin_lock_irqsave(&fbi->buf_lock, flags);
		/* when lcd is suspend, move all buffers as "switched"*/
		if (!(gfx_info.fbi[fbi->id]->active))
			buf_endframe(fi);

		/* Collect expired frame to list */
		collectFreeBuf(fbi, fbi->filterBufList, fbi->buf_freelist);
		if (copy_to_user(argp, fbi->filterBufList,
					3*MAX_QUEUE_NUM*sizeof(u8 *))) {
			spin_unlock_irqrestore(&fbi->buf_lock, flags);
			return -EFAULT;
		}
		clearFilterBuf(fbi->filterBufList, RESET_BUF);
		spin_unlock_irqrestore(&fbi->buf_lock, flags);
		if (fbi->debug)
			printk(KERN_DEBUG"get freelist end\n");
		return 0;
	}
	case FB_IOCTL_GET_BUFF_ADDR:
	{
		return copy_to_user(argp, &fbi->surface.videoBufferAddr,
			sizeof(struct _sVideoBufferAddr)) ? -EFAULT : 0;
	}
	case FB_IOCTL_SET_VID_OFFSET:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gViewPortOffset, argp,
				sizeof(gViewPortOffset))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		if (check_surface(fi, -1, 0, &gViewPortOffset, 0))
			pxa168fb_set_par(fi);
		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_GET_VID_OFFSET:
		return copy_to_user(argp, &fbi->surface.viewPortOffset,
			sizeof(struct _sViewPortOffset)) ? -EFAULT : 0;

	case FB_IOCTL_SET_SURFACE:
	{
		mutex_lock(&fbi->access_ok);
		/* Get user-mode data. */
		if (copy_from_user(&fbi->surface_bak, argp,
					sizeof(struct _sOvlySurface))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		fbi->surface_set = 1;

		mutex_unlock(&fbi->access_ok);
		return 0;
	}
	case FB_IOCTL_GET_SURFACE:
	{
		mutex_lock(&fbi->access_ok);
		if (fbi->surface_set) {
			ret = copy_to_user(argp, &fbi->surface_bak,
			sizeof(struct _sOvlySurface));
		} else {
		    ret = copy_to_user(argp, &fbi->surface,
			sizeof(struct _sOvlySurface));
		}

		ret = (ret ? -EFAULT : 0);
		mutex_unlock(&fbi->access_ok);
		return ret;
	}

	case FB_IOCTL_SET_MEMORY_TOGGLE:
		break;

	case FB_IOCTL_SET_COLORKEYnALPHA:
		if (copy_from_user(&fbi->ckey_alpha, argp,
		    sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;

		pxa168fb_ovly_set_colorkeyalpha(fbi);
		break;
	case FB_IOCTL_GET_CHROMAKEYS:
		pxa168_sync_colorkey_structures(fbi, FB_SYNC_COLORKEY_TO_CHROMA);
		return copy_to_user(argp, &fbi->chroma, sizeof(struct pxa168_fb_chroma)) ? -EFAULT : 0;
	case FB_IOCTL_PUT_CHROMAKEYS:
		if (copy_from_user(&fbi->chroma, argp, sizeof(struct pxa168_fb_chroma)))
			return -EFAULT;
		pxa168_sync_colorkey_structures(fbi, FB_SYNC_CHROMA_TO_COLORKEY);
		pxa168fb_ovly_set_colorkeyalpha(fbi);
		return copy_to_user(argp, &fbi->chroma, sizeof(struct pxa168_fb_chroma)) ? -EFAULT : 0;

	case FB_IOCTL_GET_COLORKEYnALPHA:
		if (copy_to_user(argp, &fbi->ckey_alpha,
		    sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;
		break;
	case FB_IOCTL_SWITCH_VID_OVLY:
		if (copy_from_user(&vid_on, argp, sizeof(int)))
			return -EFAULT;
		spin_lock_irqsave(&fbi->var_lock, flags);
		mask = CFG_DMA_ENA_MASK;
again:
		val = CFG_DMA_ENA(check_modex_active(fbi->id, vid_on & 1));
#ifdef CONFIG_PXA688_VDMA
		if (vid_on == 0 && mi->vdma_enable == 1 &&
			fbi->surface.viewPortInfo.rotation != 0)
			pxa688fb_vdma_release(fbi);
#endif
		/* enable DMA only when fbi->new_addr not NULL */
		if (!val || fbi->new_addr[0])
			dma_ctrl_set(fbi, 0, mask, val);

		fbi->active = val ? 1 : 0;
		pr_debug("FB_IOCTL_SWITCH_VID_OVLY fbi %d active %d\n",
			fbi->id, fbi->active);

		if (FB_MODE_DUP) {
			fbi = ovly_info.fbi[fb_dual];
			goto again;
		}
		if (fb_mode && (fbi->id == fb_dual))
			fbi = ovly_info.fbi[fb_base];

		spin_unlock_irqrestore(&fbi->var_lock, flags);
		break;
	case FB_IOCTL_SWITCH_GRA_OVLY:
		if (copy_from_user(&gfx_on, argp, sizeof(int))){
			return -EFAULT;
		}
		mask = CFG_GRA_ENA_MASK;
		val = CFG_GRA_ENA(check_modex_active(fbi->id, gfx_on & 1));
		dma_ctrl_set(fbi, 0, mask, val);

		if (FB_MODE_DUP) {
			val = CFG_DMA_ENA(check_modex_active(fb_dual, gfx_on & 1));
			dma_ctrl_set(ovly_info.fbi[fb_dual], 0, mask, val);
		}

		break;
	case FB_IOCTL_SWAP_VIDEO_RED_BLUE:
		param = (arg & 0x1);
		dma_ctrl_set(fbi, 0, CFG_DMA_SWAPRB_MASK, CFG_DMA_SWAPRB(param));
		if (FB_MODE_DUP)
			dma_ctrl_set(ovly_info.fbi[fb_dual], 0, CFG_DMA_SWAPRB_MASK,
				CFG_DMA_SWAPRB(param));
		return 0;
		break;

	case FB_IOCTL_SWAP_VIDEO_U_V:
		param = (arg & 0x1);
		dma_ctrl_set(fbi, 0, CFG_DMA_SWAPUV_MASK, CFG_DMA_SWAPUV(param));
		if (FB_MODE_DUP)
			dma_ctrl_set(ovly_info.fbi[fb_dual], 0, CFG_DMA_SWAPUV_MASK,
				CFG_DMA_SWAPUV(param));
		return 0;
		break;

	case FB_IOCTL_SWAP_VIDEO_Y_UV:
		param = (arg & 0x1);
		dma_ctrl_set(fbi, 0, CFG_DMA_SWAPYU_MASK, CFG_DMA_SWAPYU(param));
		if (FB_MODE_DUP)
			dma_ctrl_set(ovly_info.fbi[fb_dual], 0, CFG_DMA_SWAPYU_MASK,
				CFG_DMA_SWAPYU(param));
	        return 0;
		break;

	case FB_IOCTL_PUT_VIDEO_ALPHABLEND:
		/*
		 *  This puts the blending control to the Video layer.
		 */
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(0) | CFG_ALPHA(0xff);
		dma_ctrl_set(fbi, 1, mask, val);
		if (FB_MODE_DUP)
			dma_ctrl_set(ovly_info.fbi[fb_dual], 1, mask, val);
		return 0;
		break;

	case FB_IOCTL_PUT_GLOBAL_ALPHABLEND:
		/*
		 *  The userspace application can specify a byte value for the amount of global blend
		 *  between the video layer and the graphic layer.
		 *
		 *  The alpha blending is per the formula below:
		 *  P = (V[P] * blendval/255) + (G[P] * (1 - blendval/255))
		 *
		 *     where: P = Pixel value, V = Video Layer, and G = Graphic Layer
		 */
		blendval = (arg & 0xff);
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(2) | CFG_ALPHA(blendval);
		dma_ctrl_set(fbi, 1, mask, val);
		if (FB_MODE_DUP)
			dma_ctrl_set(ovly_info.fbi[fb_dual], 1, mask, val);
		return 0;
		break;

	case FB_IOCTL_PUT_GRAPHIC_ALPHABLEND:
		/*
		 *  This puts the blending back to the default mode of allowing the
		 *  graphic layer to do pixel level blending.
		 */
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(1) | CFG_ALPHA(0x0);
		dma_ctrl_set(fbi, 1, mask, val);
		if (FB_MODE_DUP)
			dma_ctrl_set(ovly_info.fbi[fb_dual], 1, mask, val);
		return 0;
		break;

	default:
		break;
	}

	return 0;
}

static void collectFreeBuf(struct pxa168fb_info *fbi,
		u8 *filterList[][3], u8 **freeList)
{
	int i = 0, j = 0;
	struct _sOvlySurface *srf = 0;
	u8 *ptr;

	if (!filterList || !freeList)
		return;

	if (fbi->debug)
		printk(KERN_DEBUG"%s\n", __func__);
	for (i = 0, j = 0; i < MAX_QUEUE_NUM; i++) {

		ptr = freeList[i];

		/* Check freeList's content. */
		if (ptr) {
			for (; j < MAX_QUEUE_NUM; j++) {
				if (!(filterList[j][0])) {
					/* get surface's ptr. */
					srf = (struct _sOvlySurface *)ptr;

					/* save ptrs which need to be freed.*/
					filterList[j][0] =
					 srf->videoBufferAddr.startAddr[0];
					filterList[j][1] =
					 srf->videoBufferAddr.startAddr[1];
					filterList[j][2] =
					 srf->videoBufferAddr.startAddr[2];
					if (fbi->debug)
						printk(KERN_DEBUG"%s: buffer \
					%p will be returned\n", __func__,
					srf->videoBufferAddr.startAddr[0]);
					break;
				}
			}

			if (j >= MAX_QUEUE_NUM)
				break;

			kfree(freeList[i]);
			freeList[i] = 0;
		} else {
			/* till content is null. */
			break;
		}
	}
}

static void clearFilterBuf(u8 *ppBufList[][3], int iFlag)
{
	/* Check null pointer. */
	if (!ppBufList)
		return;
	if (RESET_BUF & iFlag)
		memset(ppBufList, 0, 3 * MAX_QUEUE_NUM * sizeof(u8 *));
}

static int pxa168fb_open(struct fb_info *fi, int user)
{
	struct pxa168fb_mach_info *mi;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	unsigned long flags;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);

	if (fb_mode && (fbi->id == fb_dual)) {
		printk("fb_mode %x, operation to TV path is not allowed.\n"
			"Please switch to mode0 if needed.\n", fb_mode);
		return -EAGAIN;
	}

#ifdef OVLY_DVFM_CONSTRAINT
	dvfm_disable_lowpower(dvfm_dev_idx);
#endif

	atomic_inc(&fbi->op_count);
	mi = fbi->dev->platform_data;
	fbi->new_addr[0] = 0;
	fbi->new_addr[1] = 0;
	fbi->new_addr[2] = 0;
	fi->fix.smem_start = fbi->fb_start_dma;
	fi->screen_base = fbi->fb_start;
	fbi->surface.videoMode = -1;
	fbi->surface.viewPortInfo.srcWidth = var->xres;
	fbi->surface.viewPortInfo.srcHeight = var->yres;
	if (mi->mmap) {
		fbi->active = 1;
		if (FB_MODE_DUP)
			ovly_info.fbi[fb_dual]->active = 1;
	}
	set_pix_fmt(var, fbi->pix_fmt);

	if (mutex_is_locked(&fbi->access_ok))
		mutex_unlock(&fbi->access_ok);
	/* clear buffer list. */
	spin_lock_irqsave(&fbi->buf_lock, flags);
	clearFilterBuf(fbi->filterBufList, RESET_BUF);
	buf_clear(fbi->buf_freelist, RESET_BUF|FREE_ENTRY);
	buf_clear(fbi->buf_waitlist, RESET_BUF|FREE_ENTRY);
	fbi->buf_current = 0;
	fbi->buf_retired = 0;
	fbi->surface_set = 0;
	ovly_info.wait_peer = 0;
	spin_unlock_irqrestore(&fbi->buf_lock, flags);
	return 0;
}

static int pxa168fb_release(struct fb_info *fi, int user)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
#ifdef CONFIG_PXA688_VDMA
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
#endif
	struct fb_var_screeninfo *var = &fi->var;
	u32 val, x;
	unsigned long flags;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
	fbi->surface.viewPortInfo.rotation	= 0;
	/* clear buffer list. */
	spin_lock_irqsave(&fbi->buf_lock, flags);
	clearFilterBuf(fbi->filterBufList, RESET_BUF);
	buf_clear(fbi->buf_freelist, RESET_BUF|FREE_ENTRY);
	buf_clear(fbi->buf_waitlist, RESET_BUF|FREE_ENTRY);
	if (fbi->buf_current)
		kfree(fbi->buf_current);
	fbi->buf_current = 0;
	if (fbi->buf_retired)
		kfree(fbi->buf_retired);
	fbi->buf_retired = 0;
	ovly_info.wait_peer = 0;
	spin_unlock_irqrestore(&fbi->buf_lock, flags);
	/* make sure graphics layer is enabled */
	x = (dma_ctrl_read(fbi, 0) & CFG_GRA_ENA_MASK);
	val = check_modex_active(fbi->id, 1);
	if (!x)
		dma_ctrl_set(fbi, 0, CFG_GRA_ENA_MASK, CFG_GRA_ENA(val));

	/* Compatibility with older PXA behavior. Force Video DMA engine off at RELEASE.*/

	if (atomic_dec_and_test(&fbi->op_count)) {
#ifdef CONFIG_PXA688_VDMA
		if (mi->vdma_enable) {
			val = vdma_ctrl_read(fbi) & 1;
			if (val)
				pxa688fb_vdma_release(fbi);
		}
#endif
		dma_ctrl_set(fbi, 0, CFG_DMA_ENA_MASK, 0);
		if (FB_MODE_DUP) {
			dma_ctrl_set(ovly_info.fbi[fb_dual], 0, CFG_DMA_ENA_MASK, 0);
		}
	}
	fbi->new_addr[0] = 0; fbi->new_addr[1] = 0; fbi->new_addr[2] = 0;
	fbi->active = 0;

	/* Turn off compatibility mode */

	var->nonstd &= ~0xff000000;
	COMPAT_MODE = 0;

	if (FB_MODE_DUP) {
		fbi = ovly_info.fbi[fb_dual];

		/* make sure graphics layer is enabled */
		x = (dma_ctrl_read(fbi, 0) & CFG_GRA_ENA_MASK);
		val = check_modex_active(fbi->id, 1);
		if (!x)
			dma_ctrl_set(fbi, 0, CFG_GRA_ENA_MASK, CFG_GRA_ENA(val));

		fbi->new_addr[0] = 0;
		fbi->new_addr[1] = 0;
		fbi->new_addr[2] = 0;
		fbi->active = 0;
	}

#ifdef OVLY_DVFM_CONSTRAINT
	dvfm_enable_lowpower(dvfm_dev_idx);
#endif

	return 0;
}

void pxa168fb_ovly_dual(int enable)
{
	struct pxa168fb_info *fbi_base = ovly_info.fbi[fb_base];
	struct pxa168fb_info *fbi_dual = ovly_info.fbi[fb_dual];

	pr_debug("%s enable %d fb_dual->active %d\n",
		__func__, enable, fbi_dual->active);
	if (enable) {
		pxa168fb_set_par(fbi_base->fb_info);
		fbi_dual->active = fbi_base->active;
	} else {
		/* disable video layer DMA */
		dma_ctrl_set(fbi_dual, 0, CFG_DMA_ENA_MASK, 0);
		fbi_dual->active = 0;

		/* free buffer if wait_peer flag is set */
		if (ovly_info.wait_peer) {
			ovly_info.wait_peer = 0;
#ifdef OVLY_TASKLET
			tasklet_schedule(&fbi_base->ovly_task);
#else
			if (fbi_base->system_work)
				schedule_work(&fbi_base->buf_work);
			else
				queue_work(fbi_base->work_q,
					&fbi_base->fb_info->queue);
#endif
		}
	}
}

static void set_mode(struct pxa168fb_info *fbi, struct fb_var_screeninfo *var,
		     struct fb_videomode *mode, int pix_fmt, int ystretch)
{
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
	set_pix_fmt(var, pix_fmt);

	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = max(var->xres, var->xres_virtual);
	if (ystretch)
		var->yres_virtual = var->yres*2;
	else
		var->yres_virtual = max(var->yres, var->yres_virtual);

	var->grayscale = 0;
	var->accel_flags = FB_ACCEL_NONE;
	var->pixclock = mode->pixclock;
	var->left_margin = mode->left_margin;
	var->right_margin = mode->right_margin;
	var->upper_margin = mode->upper_margin;
	var->lower_margin = mode->lower_margin;
	var->hsync_len = mode->hsync_len;
	var->vsync_len = mode->vsync_len;
	var->sync = mode->sync;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->rotate = FB_ROTATE_UR;
}

static int pxa168fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int pix_fmt;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
	if (var->bits_per_pixel == 8) {
		printk("bits per pixel too small\n");
		return -EINVAL;
	}

	/* compatibility mode: if the MSB of var->nonstd is 0xAA then
	 * set xres_virtual and yres_virtual to xres and yres.
	 */

	if((var->nonstd >> 24) == 0xAA)
		COMPAT_MODE = 0x2625;

	if((var->nonstd >> 24) == 0x55)
		COMPAT_MODE = 0x0;

	/*
	 * Basic geometry sanity checks.
	 */

	if (var->xoffset + var->xres > var->xres_virtual) {
		printk("ERROR: xoffset + xres is greater than xres_virtual\n");
		return -EINVAL;
	}
	if (var->yoffset + var->yres > var->yres_virtual) {
		printk("ERROR: yoffset + yres is greater than yres_virtual\n");
		return -EINVAL;
	}

	if (var->xres + var->right_margin +
	    var->hsync_len + var->left_margin > 2048)
		return -EINVAL;
	if (var->yres + var->lower_margin +
	    var->vsync_len + var->upper_margin > 2048)
		return -EINVAL;

	/*
	 * Check size of framebuffer.
	 */
	if (mi->mmap && (var->xres_virtual * var->yres_virtual *
	    (var->bits_per_pixel >> 3) > max_fb_size))
		return -EINVAL;

	/*
	 * Select most suitable hardware pixel format.
	 */
	pix_fmt = determine_best_pix_fmt(var);
	dev_dbg(fi->dev, "%s determine_best_pix_fmt returned: %d\n", __FUNCTION__, pix_fmt);
	if (pix_fmt < 0)
		return pix_fmt;

	return 0;
}

static u32 dma_ctrl0_update(int active, struct pxa168fb_mach_info *mi, u32 x, u32 pix_fmt)
{
	if (active)
		x |= 1;
	else
		x &= ~1;

	/*
	 * clear video layer's field
	 */
	x &= 0xef0fff01;

	/*
	 * If we are in a pseudo-color mode, we need to enable
	 * palette lookup.
	 */
	if (pix_fmt == PIX_FMT_PSEUDOCOLOR)
		x |= 0x10000000;

	x |= 1 << 6;	/* horizontal smooth filter */
	/*
	 * Configure hardware pixel format.
	 */
	x |= ((pix_fmt & ~0x1000) >> 1) << 20;

	/*
	 * color format in memory:
	 * PXA168/PXA910:
	 * PIX_FMT_YUV422PACK: UYVY(CbY0CrY1)
	 * PIX_FMT_YUV422PLANAR: YUV
	 * PIX_FMT_YUV420PLANAR: YUV
	 */
	if (((pix_fmt & ~0x1000) >> 1) == 5) {
		x |= 0x00000002;
		x |= (mi->panel_rbswap) << 4;
		if ((pix_fmt & 0x1000))
			x |= 1 << 2;	//Y and U/V is swapped
		else
			x |= (pix_fmt & 1) << 3;
	} else if (pix_fmt >= 12) {	/* PIX_FMT_YUV422PACK_IRE_90_270 is here */
		x |= 0x00000002;
		x |= (pix_fmt & 1) << 3;
		x |= (mi->panel_rbswap) << 4;
	} else {
		x |= (((pix_fmt & 1) ^ 1) ^ (mi->panel_rbswap)) << 4;
	}

	return x;
}

static void set_dma_control0(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi;
	u32 x = 0, x_bk = 0, active = 0;

	dev_dbg(fbi->fb_info->dev, "FB1: Enter %s\n", __FUNCTION__);
again:
	/*
	 * Get reg's current value
	 */
	x_bk = x = dma_ctrl_read(fbi, 0);
	mi = fbi->dev->platform_data;

	if (fbi->new_addr[0] || mi->mmap)
		active = check_modex_active(fbi->id, fbi->active);

	pr_debug("set dma ctrl0: fbi %d addr %lu active %x fbi->active %d\n",
		fbi->id, fbi->new_addr[0], active, fbi->active);
	/* update dma_ctrl0 according to pix_fmt */
	x = dma_ctrl0_update(active, mi, x, fbi->pix_fmt);

	/* clear reserved bits if not panel path */
	if (fbi->id)
		x &= ~CFG_ARBFAST_ENA(1);

	if (x_bk != x) {
		dma_ctrl_write(fbi, 0, x);
	}

	if (FB_MODE_DUP) {
		ovly_info.fbi[fb_dual]->pix_fmt = fbi->pix_fmt;
		fbi = ovly_info.fbi[fb_dual];
		mi = fbi->dev->platform_data;
		goto again;
	}
}

static void set_dma_control1(struct pxa168fb_info *fbi, int sync)
{
	u32 x = 0, x_bk = 0;

	dev_dbg(fbi->fb_info->dev, "FB1: Enter %s\n", __FUNCTION__);

again:
	/*
	 * Get current settings.
	 */
	x_bk = x = dma_ctrl_read(fbi, 1);

	/*
	 * We trigger DMA on the falling edge of vsync if vsync is
	 * active low, or on the rising edge if vsync is active high.
	 */
	if (!(sync & FB_SYNC_VERT_HIGH_ACT))
		x |= 0x08000000;


	if (x_bk != x) {
		dma_ctrl_write(fbi, 1, x);
	}

	if (FB_MODE_DUP) {
		fbi = ovly_info.fbi[fb_dual];
		goto again;
	}
}

static void clear_vid_irq(struct pxa168fb_info *fbi)
{
	int isr = readl(fbi->reg_base + SPU_IRQ_ISR);

	if (isr & display_done_imask(fbi->id)) {
		irq_status_clear(fbi, display_done_imask(fbi->id));
		pr_err("fbi %d irq miss, clear isr %x\n", fbi->id, isr);
	}
}

static int wait_for_vsync(struct pxa168fb_info *fbi)
{
	if (fbi) {
		atomic_set(&fbi->w_intr, 0);
		if (FB_MODE_DUP)
			atomic_set(&gfx_info.fbi[fb_dual]->w_intr, 0);

again:
		wait_event_interruptible_timeout(fbi->w_intr_wq,
			atomic_read(&fbi->w_intr), HZ/20);

		/* handle timeout case, to w/a irq miss */
		if (atomic_read(&fbi->w_intr) == 0)
			clear_vid_irq(fbi);

		if (FB_MODE_DUP) {
			fbi = gfx_info.fbi[fb_dual];
			goto again;
		}
	}
	return 0;
}

static int vid_irq_enabled(struct pxa168fb_info *fbi)
{
	if (irqs_disabled() || !fbi->active)
		return 0;

	/* check whether path clock is disabled */
	if (readl(fbi->reg_base + clk_div(fbi->id)) & (SCLK_DISABLE))
		return 0;

	/* in modex dma may not be enabled */
	if (!(dma_ctrl_read(fbi, 0) & CFG_DMA_ENA_MASK))
		return 0;

	return readl(fbi->reg_base + SPU_IRQ_ENA) & display_done_imask(fbi->id);
}

static void set_yuv_start(struct pxa168fb_info *fbi, unsigned long addr)
{
	struct lcd_regs *regs = get_regs(fbi);
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	unsigned long addr_y0, addr_u0, addr_v0;

	/* caculate yuv offset */
	addr_y0 = addr;

	if ((fbi->pix_fmt >= 12) && (fbi->pix_fmt <= 15))
		addr += var->xres * var->yres;
	addr_u0 = addr;

	if ((fbi->pix_fmt >> 1) == 6)
		addr += var->xres * var->yres/2;
	else if ((fbi->pix_fmt>>1) == 7)
		addr += var->xres * var->yres/4;
	addr_v0 = addr;

	/* update registers */
	writel(addr_y0, &regs->v_y0);
	writel(addr_u0, &regs->v_u0);
	writel(addr_v0, &regs->v_v0);
}

static void set_graphics_start(struct fb_info *fi, int xoffset, int yoffset)
{
	struct pxa168fb_info *fbi = fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &fi->var;
	struct lcd_regs *regs;
	int pixel_offset;
	unsigned long addr = 0;
	static int debugcount = 0;

	if(debugcount < 10)
		debugcount++;

	if (debugcount < 9)
		dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);

again:
	regs = get_regs(fbi);

	if (!(fbi->new_addr[0])) {
		if (!mi->mmap) {
			pr_debug("fbi %d(line %d): input err, mmap is not"
				" supported\n", fbi->id, __LINE__);
			return;
		}
		pixel_offset = (yoffset * var->xres_virtual) + xoffset;
		/* Set this at VSync interrupt time */
		addr = fbi->fb_start_dma + ((pixel_offset * var->bits_per_pixel) >> 3);
		set_yuv_start(fbi, addr);

		/* return until the address take effect after vsync occurs */
		if (vid_irq_enabled(fbi))
			wait_for_vsync(fbi);
	} else {
		if (fbi->debug)
			printk(KERN_DEBUG"%s: buffer updated to %x\n",
				 __func__, (int)fbi->new_addr[0]);

		writel(fbi->new_addr[0], &regs->v_y0);
		if (fbi->pix_fmt >= 12 && fbi->pix_fmt <= 15) {
			writel(fbi->new_addr[1], &regs->v_u0);
			writel(fbi->new_addr[2], &regs->v_v0);
		}
	}

	if (FB_MODE_DUP) {
		ovly_info.fbi[fb_dual]->fb_start_dma = fbi->fb_start_dma;
		ovly_info.fbi[fb_dual]->pix_fmt = fbi->pix_fmt;
		ovly_info.fbi[fb_dual]->new_addr[0] = fbi->new_addr[0];
		ovly_info.fbi[fb_dual]->new_addr[1] = fbi->new_addr[1];
		ovly_info.fbi[fb_dual]->new_addr[2] = fbi->new_addr[2];
		fbi = ovly_info.fbi[fb_dual];
		goto again;
	}
}

/* if video layer is full screen without alpha blending then turn off graphics dma to save bandwidth */
static void pxa168fb_graphics_off(struct pxa168fb_info *fbi)
{
	u32 dma1, gfx_bottom, v_size_dst, screen_active;
	struct lcd_regs *regs;

	dev_dbg(fbi->fb_info->dev, "%s\n", __func__);
again:
	regs = get_regs(fbi);

	dma1 = dma_ctrl_read(fbi, 1) & (CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK);
	gfx_bottom = (dma1 == (CFG_ALPHA_MODE(2) | CFG_ALPHA(0xff))) ? 1 : 0;
	v_size_dst = readl(&regs->v_size_z);
	screen_active = readl(&regs->screen_active);

	dev_dbg(fbi->fb_info->dev, "v_size_dst %d screen_active %d dma1 0x%x\n",
			v_size_dst, screen_active, dma1);

	/* graphics layer off if overlay on upper layer and full screen */
	if ((v_size_dst == screen_active) && gfx_bottom)
		dma_ctrl_set(fbi, 0, CFG_GRA_ENA_MASK, 0);

	if (FB_MODE_DUP) {
		fbi = ovly_info.fbi[fb_dual];
		goto again;
	}
}

static void set_yuv_pitch(struct pxa168fb_info *fbi, struct _sOvlySurface *surface, struct fb_var_screeninfo *var)
{
	struct lcd_regs *regs = get_regs(fbi);
	int xzoom = surface->viewPortInfo.zoomXSize, yzoom = surface->viewPortInfo.zoomYSize;
	int xpos = surface->viewPortOffset.xOffset, ypos = surface->viewPortOffset.yOffset;
	u32 x;

	if (((fbi->pix_fmt & ~0x1000) >> 1) < 6)
		writel((surface->viewPortInfo.yPitch) ?
			(surface->viewPortInfo.yPitch) :
			var->xres * var->bits_per_pixel >> 3,
			&regs->v_pitch_yc);
	else {
		writel(surface->viewPortInfo.yPitch ?
			surface->viewPortInfo.yPitch : var->xres,
			&regs->v_pitch_yc);
		writel((surface->viewPortInfo.uPitch && surface->viewPortInfo.vPitch) ?
			((surface->viewPortInfo.vPitch << 16) |
			 (surface->viewPortInfo.uPitch)) :
			((var->xres >> 1) << 16 | (var->xres >> 1)),
			&regs->v_pitch_uv);
	}

	dev_dbg(fbi->fb_info->dev, "Executing Standard Mode\n");
	writel((var->yres << 16) | var->xres, &regs->v_size);

	if (fb_mode && (fbi->id == fb_dual)) {
		dev_dbg(fbi->fb_info->dev, "%s line %d adjust ?? xpos %d ypos %d"
			"xzoom %d yzoom %d\nxres %d yres %d xres_z %d yres_z %d\n",
			__func__, __LINE__, xpos, ypos, xzoom, yzoom,
			gfx_info.xres, gfx_info.yres, gfx_info.xres_z, gfx_info.yres_z);
		yzoom = yzoom * gfx_info.yres_z / gfx_info.yres;
		xzoom = xzoom * gfx_info.xres_z / gfx_info.xres;
		ypos = ypos * gfx_info.yres_z / gfx_info.yres;
		xpos = xpos * gfx_info.xres_z / gfx_info.xres;
		dev_dbg(fbi->fb_info->dev, "\t to xpos %d ypos %d yzoom %d"
			" xzoom %d\n", xpos, ypos, yzoom, xzoom);
	}
	writel((yzoom << 16) | xzoom, &regs->v_size_z);

	/* enable two-level zoom down if the ratio exceed 64/bpp */
	if (xzoom && var->bits_per_pixel) {
		x = readl(fbi->reg_base + LCD_AFA_ALL2ONE);
		if (var->xres * var->bits_per_pixel > 64 * xzoom)
			x |= 1 << 20;
		else
			x &= ~(1 << 20);
		writel(x, fbi->reg_base + LCD_AFA_ALL2ONE);
	}

	if(COMPAT_MODE != 0x2625) {
		/* update video position offset */
		dev_dbg(fbi->fb_info->dev, "Setting Surface offsets...\n");

		writel(CFG_DMA_OVSA_VLN(ypos)| xpos, &regs->v_start);
	}
	else {
		dev_dbg(fbi->fb_info->dev, "Executing 2625 compatibility mode\n");
		xpos = (var->nonstd & 0x3ff);
		ypos = (var->nonstd >> 10) & 0x3ff;
		writel(CFG_DMA_OVSA_VLN(ypos) | xpos, &regs->v_start);
		writel((var->height << 16) | var->width, &regs->v_size_z);
	}
}

static int pxa168fb_set_par(struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	struct _sOvlySurface *surface = &fbi->surface;
	int pix_fmt;
#ifdef CONFIG_PXA688_VDMA
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	u32 val;
#endif

	dev_dbg(fi->dev,"FB1: Enter %s\n", __FUNCTION__);
	/*
	 * Determine which pixel format we're going to use.
	 */
	pix_fmt = determine_best_pix_fmt(&fi->var);
	dev_dbg(fi->dev,"determine_best_pix_fmt returned: %d\n", pix_fmt);
	if (pix_fmt < 0)
		return pix_fmt;
	fbi->pix_fmt = pix_fmt;

	dev_dbg(fi->dev, "pix_fmt=%d\n", pix_fmt);
	dev_dbg(fi->dev,"BPP = %d\n", var->bits_per_pixel);
	/*
	 * Set additional mode info.
	 */
	if (pix_fmt == PIX_FMT_PSEUDOCOLOR)
		fi->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		fi->fix.visual = FB_VISUAL_TRUECOLOR;
	fi->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;

	/* when lcd is suspend, read or write lcd controller's
	* register is not effective, so just return*/
	if (!(gfx_info.fbi[fbi->id]->active)) {
		printk(KERN_DEBUG"LCD is not active, don't touch hardware\n");
		return 0;
	}

	/*
	 * Configure graphics DMA parameters.
	 */
	set_graphics_start(fi, fi->var.xoffset, fi->var.yoffset);
	set_yuv_pitch(fbi, surface, &fbi->fb_info->var);

	/*
	 * Configure global panel parameters.
	 */
	set_dma_control0(fbi);
	set_dma_control1(fbi, fi->var.sync);

	if (FB_MODE_DUP) {
		ovly_info.fbi[fb_dual]->pix_fmt = fbi->pix_fmt;
		ovly_info.fbi[fb_dual]->fb_info->var.bits_per_pixel = fbi->fb_info->var.bits_per_pixel;
		ovly_info.fbi[fb_dual]->fb_info->var.nonstd = fbi->fb_info->var.nonstd;
		ovly_info.fbi[fb_dual]->fb_info->var.xres = fbi->fb_info->var.xres;
		ovly_info.fbi[fb_dual]->fb_info->var.yres = fbi->fb_info->var.yres;
		set_yuv_pitch(ovly_info.fbi[fb_dual], surface, &ovly_info.fbi[fb_dual]->fb_info->var);
	}

#ifdef CONFIG_PXA688_VDMA
	if (mi->vdma_enable && fbi->surface.viewPortInfo.rotation) {
		vdma_lines = pxa688fb_vdma_get_linenum(fbi);
		pxa688fb_vdma_set(fbi, vdma_paddr, vdma_lines, 1);
	} else if (mi->vdma_enable) {
		val = vdma_ctrl_read(fbi) & 1;
		if (val)
			pxa688fb_vdma_release(fbi);
	}
#endif

	pxa168fb_graphics_off(fbi);
	return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	return ((chan & 0xffff) >> (16 - bf->length)) << bf->offset;
}

static u32 to_rgb(u16 red, u16 green, u16 blue)
{
	red >>= 8;
	green >>= 8;
	blue >>= 8;

	return (red << 16) | (green << 8) | blue;
}

static int
pxa168fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		 unsigned int blue, unsigned int trans, struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;
	u32 val;

	if (fi->fix.visual == FB_VISUAL_TRUECOLOR && regno < 16) {
		val =  chan_to_field(red,   &fi->var.red);
		val |= chan_to_field(green, &fi->var.green);
		val |= chan_to_field(blue , &fi->var.blue);
		fbi->pseudo_palette[regno] = val;
	}

	if (fi->fix.visual == FB_VISUAL_PSEUDOCOLOR && regno < 256) {
		val = to_rgb(red, green, blue);
		writel(val, fbi->reg_base + LCD_SPU_SRAM_WRDAT);	/* FIXME */
		writel(0x8300 | regno, fbi->reg_base + LCD_SPU_SRAM_CTRL);
	}

	return 0;
}

static int pxa168fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	if (!mi->mmap)
		return -EINVAL;

	fbi->active = 1;
	set_graphics_start(fi, var->xoffset, var->yoffset);
	return 0;
}

static int pxa168fb_fb_sync(struct fb_info *info)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)info->par;

	return wait_for_vsync(fbi);
}

/*
 *  pxa168fb_handle_irq(two lcd controllers)
 */
irqreturn_t pxa168fb_ovly_isr(int id)
{
	struct pxa168fb_info *fbi = ovly_info.fbi[id];
	struct fb_info *fi;

	if (!fbi)
		return IRQ_HANDLED;
	fi = fbi->fb_info;
	if (fbi->debug)
		printk(KERN_DEBUG "%s fbi %d\n", __func__, fbi->id);

	if (fb_mode && (id == fb_dual)) {
		/* mirror mode, tv path irq, check wait_peer set or not */
		if (ovly_info.wait_peer) {
			/* trigger irq to free panel path buf */
			fbi = ovly_info.fbi[fb_base];
#ifdef OVLY_TASKLET
			tasklet_schedule(&fbi->ovly_task);
#else
			if (fbi->system_work)
				schedule_work(&fbi->buf_work);
			else
				queue_work(fbi->work_q,
					&fbi->fb_info->queue);
#endif
		}
		/* no buf need to be handled for tv path, wakeup only */
		goto wakeup;
	}

	if (atomic_read(&fbi->op_count)) {
		/* do buffer switch for video flip */
		buf_endframe(fi);

wakeup:
		/* wake up queue. */
		atomic_set(&fbi->w_intr, 1);
		wake_up(&fbi->w_intr_wq);
	}

	return IRQ_HANDLED;
}

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);

	return sprintf(buf, "fbi %d debug %d active %d, retire_err %d,"
		" wait peer: %d surface_set %d\n", fbi->id, fbi->debug,
		fbi->active, ovly_info.retire_err, ovly_info.flag,
		fbi->surface_set);
}

static ssize_t debug_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	int	tmp;

	sscanf(buf, "%d", &tmp);
	if ((tmp == 1) || (tmp == 0))
		fbi->debug = tmp;
	else if (tmp == 9)
		ovly_info.flag ^= 1;

	return size;
}

static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show, debug_store);

static struct fb_ops pxa168fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= pxa168fb_open,
	.fb_release	= pxa168fb_release,

	.fb_check_var	= pxa168fb_check_var,
	.fb_set_par	= pxa168fb_set_par,
	.fb_setcolreg	= pxa168fb_setcolreg,
	.fb_pan_display	= pxa168fb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_sync	= pxa168fb_fb_sync,
	.fb_ioctl	= pxa168fb_ovly_ioctl,
};

static int __init get_ovly_size(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	max_fb_size = n;
	fb_size_from_cmd = 1;
	return 1;
}
__setup("ovly_size=", get_ovly_size);

static int __devinit pxa168fb_probe(struct platform_device *pdev)
{
	struct pxa168fb_mach_info *mi;
	struct fb_info *fi;
	struct pxa168fb_info *fbi;
	struct lcd_regs *regs;
	struct resource *res;
	int ret;

	mi = pdev->dev.platform_data;
	if (mi == NULL)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -EINVAL;

	fi = framebuffer_alloc(sizeof(struct pxa168fb_info), &pdev->dev);
	if (fi == NULL){
		printk("%s: no enough memory!\n", __func__);
		return -ENOMEM;
	}

	fbi = fi->par;
	fbi->id = pdev->id;
	ovly_info.fbi[fbi->id] = fbi;
	ovly_info.flag = 1;

	platform_set_drvdata(pdev, fbi);
	fbi->fb_info = fi;
	fbi->dev = &pdev->dev;
	fbi->id = pdev->id;
	fbi->fb_info->dev = &pdev->dev;
	fbi->debug = 0;
	init_waitqueue_head(&fbi->w_intr_wq);
	mutex_init(&fbi->access_ok);
	spin_lock_init(&fbi->buf_lock);
	spin_lock_init(&fbi->var_lock);

#ifdef OVLY_TASKLET
	tasklet_init(&fbi->ovly_task, pxa168fb_ovly_task, (unsigned long)fbi);
#else
	fbi->system_work = 1;
	if (fbi->system_work)
		INIT_WORK(&fbi->buf_work, pxa168fb_ovly_work);
	else {
		fbi->work_q = create_singlethread_workqueue("pxa168fb-ovly");
		INIT_WORK(&fi->queue, pxa168fb_ovly_work);
	}
#endif

	/* FIXME - video layer has no specific clk. it depend on graphic layer clk.
	 * fbi->clk = clk_get(&pdev->dev, NULL);
	 */

	/*
	 * Initialise static fb parameters.
	 */
	fi->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
		    FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	fi->node = -1;
	strcpy(fi->fix.id, mi->id);
	fi->fix.type = FB_TYPE_PACKED_PIXELS;
	fi->fix.type_aux = 0;
	fi->fix.xpanstep = 1;
	fi->fix.ypanstep = 1;
	fi->fix.ywrapstep = 0;
	fi->fix.mmio_start = res->start;
	fi->fix.mmio_len = res->end - res->start + 1;
	fi->fix.accel = FB_ACCEL_NONE;
	fi->fbops = &pxa168fb_ops;
	fi->pseudo_palette = fbi->pseudo_palette;

	/*
	 * Map LCD controller registers.
	 */
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		printk("%s: no enough memory!\n", __func__);
		ret = -ENOMEM;
		goto failed;
	}

	fbi->mem_status = -1;
	if (mi->mmap) {
		fbi->mem_status = 0;
		/*
		 * Allocate framebuffer memory.
		 */
		if (!fb_size_from_cmd) {
			if (mi->max_fb_size)
				max_fb_size = mi->max_fb_size;
			else
				max_fb_size = DEFAULT_FB_SIZE;
		}
		max_fb_size = PAGE_ALIGN(max_fb_size);
		fbi->fb_size = max_fb_size;

		/*
		 * FIXME, It may fail to alloc DMA buffer from dma_alloc_xxx
		 */
		fbi->fb_start = dma_alloc_writecombine(fbi->dev, max_fb_size,
							&fbi->fb_start_dma,
							GFP_KERNEL);
		if (!fbi->fb_start || !fbi->fb_start_dma) {
			fbi->new_addr[0] = 0;
			fbi->new_addr[1] = 0;
			fbi->new_addr[2] = 0;
			fbi->mem_status = 1;
			fbi->fb_start = (void *)__get_free_pages
						(GFP_DMA | GFP_KERNEL,
						get_order(fbi->fb_size));
			fbi->fb_start_dma = (dma_addr_t)__virt_to_phys
							(fbi->fb_start);
		}

		if (fbi->fb_start == NULL) {
			pr_err("%s: no enough memory!\n", __func__);
			ret = -ENOMEM;
			goto failed;
		}
		printk(KERN_INFO"-------FBoverlay DMA buffer phy addr : %x\n",
			(unsigned int)fbi->fb_start_dma);

		memset(fbi->fb_start, 0, fbi->fb_size);
		fi->fix.smem_start = fbi->fb_start_dma;
		fi->fix.smem_len = fbi->fb_size;
		fi->screen_base = fbi->fb_start;
		fi->screen_size = fbi->fb_size;
	}

#ifdef FB_PM_DEBUG
	pxa168fb_rw_all_regs(fbi, g_regs, 1);
#endif

	fbi->surface.viewPortInfo.rotation = 0;
#ifdef CONFIG_PXA688_VDMA
	if (mi->vdma_enable && vdma_paddr == 0) {
		vdma_paddr = pxa688fb_vdma_squ_malloc();
		pxa688_vdma_clkset(1);
	}
#endif

	/*
	 * Fill in sane defaults.
	 */
	set_mode(fbi, &fi->var, mi->modes, mi->pix_fmt, 1);
	pxa168fb_set_par(fi);

	/*
	 * Configure default register values.
	 */
	regs = get_regs(fbi);
	writel(0, &regs->v_y1);
	writel(0, &regs->v_u1);
	writel(0, &regs->v_v1);
	writel(0, &regs->v_start);

	/* Set this frame off by default */
	dma_ctrl_set(fbi, 0, CFG_DMA_ENA_MASK, 0);

	/*
	 * Allocate color map.
	 */
	if (fb_alloc_cmap(&fi->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed;
	}

	/*
	 * Get IRQ number.
	 */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL)
		return -EINVAL;


	/*
	 * Enable Video interrupt
	 */
	irq_mask_set(fbi, display_done_imask(fbi->id),
			 display_done_imask(fbi->id));

	/*
	 * Register framebuffer.
	 */
	ret = register_framebuffer(fi);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa168fb: %d\n", ret);
		ret = -ENXIO;
		goto failed;
	}

	printk(KERN_INFO "pxa168fb_ovly: frame buffer device was loaded"
		" to /dev/fb%d <%s>.\n", fi->node, fi->fix.id);

#ifdef OVLY_DVFM_CONSTRAINT
	dvfm_register("overlay1", &dvfm_dev_idx);
#endif

	ret = device_create_file(&pdev->dev, &dev_attr_debug);
	if (ret < 0) {
		pr_err("device attr create fail: %d\n", ret);
		goto failed;
	}

	return 0;

failed:
	platform_set_drvdata(pdev, NULL);
	fb_dealloc_cmap(&fi->cmap);
	if (fbi->fb_start != NULL) {
		if (fbi->mem_status)
			free_pages((unsigned long)fbi->fb_start,
				get_order(max_fb_size));
		else
			dma_free_writecombine(fbi->dev, max_fb_size,
				fbi->fb_start, fbi->fb_start_dma);
	}
	if (fbi->reg_base != NULL)
		iounmap(fbi->reg_base);
	kfree(fbi);
	return ret;
}

#ifdef CONFIG_PM
#if 0
static int pxa168fb_vid_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);
	struct fb_info *fi = fbi->fb_info;

#ifdef FB_PM_DEBUG
	pxa168fb_rw_all_regs(fbi, g_regs1, 1);
#endif

	if (mesg.event & PM_EVENT_SLEEP)
		fb_set_suspend(fi, 1);
	pdev->dev.power.power_state = mesg;

#ifdef FB_PM_DEBUG
	pxa168fb_rw_all_regs(fbi, g_regs, 0);
#endif
	printk(KERN_INFO "pxa168fb_ovly.%d suspended, state = %d.\n", fbi->id, mesg.event);

	return 0;
}

static int pxa168fb_vid_resume(struct platform_device *pdev)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);
	struct fb_info *fi = fbi->fb_info;

	fb_set_suspend(fi, 0);

#ifdef FB_PM_DEBUG
	{
		u32 i;
		u32 reg;

		for (i = 0xC0; i <= 0x01C4; i += 4) {
			reg = readl(fbi->reg_base + i);
			if (reg != g_regs1[i])
				printk("Register 0x%08x: 0x%08x - 0x%08x.\n",
						i, g_regs1[i], reg);
		}
	}
#endif

	printk(KERN_INFO "pxa168fb_ovly.%d resumed.\n", fbi->id);

	return 0;
}
#endif
#endif


static struct platform_driver pxa168fb_driver = {
	.probe		= pxa168fb_probe,
/*	.remove		= pxa168fb_remove,		*/
#ifdef CONFIG_PM
//	.suspend	= pxa168fb_vid_suspend,
//	.resume		= pxa168fb_vid_resume,
#endif
	.driver		= {
		.name	= "pxa168fb_ovly",
		.owner	= THIS_MODULE,
	},
};

static int __devinit pxa168fb_init(void)
{
	return platform_driver_register(&pxa168fb_driver);
}

/*module_init(pxa168fb_init);*/
late_initcall(pxa168fb_init);

MODULE_DESCRIPTION("Framebuffer driver for PXA168");
MODULE_LICENSE("GPL");
