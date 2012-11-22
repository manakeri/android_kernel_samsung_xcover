/*
 * linux/drivers/video/pxa168fb.c -- Marvell PXA168 LCD Controller
 *
 *  Copyright (C) 2008 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2009-02-16  adapted from original version for PXA168
 *		Green Wan <gwan@marvell.com>
 *              Jun Nie <njun@marvell.com>
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
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/pxa168fb.h>
#include "pxa168fb.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

static int vsync_check;
static int irq_count;
static int vsync_count;
static int dispd_count;
static int f0_count;
static int f1_count;
static int vf0_count;
static int vf1_count;
static struct timer_list vsync_timer;
static struct wake_lock idle_lock;

#define DEFAULT_REFRESH		60	/* Hz */

/* Compatibility mode global switch .....
 *
 * This is a secret switch for user space programs that may want to
 * select color spaces and set resolutions the same as legacy PXA display
 * drivers. The switch is set and unset by setting a specific value in the
 * var_screeninfo.nonstd variable.
 *
 * To turn on compatibility with older PXA, set the MSB of nonstd to 0xAA.
 * To turn off compatibility with older PXA, set the MSB of nonstd to 0x55.
 */

static unsigned int COMPAT_MODE;
static unsigned int max_fb_size = 0;
static unsigned int fb_size_from_cmd = 0;

/* LCD mode switch
 * mode0: panel + 1080p TV path, seperately
 *
 * clone mode:
 * mode1: panel path + panel path graphics/video layer data resized to TV path
 * mode2: mode1 with panel path disalbed
 * mode3: mode1 with TV path disalbed
 *
 * fb_share mode:
 * TV path graphics layer share same frame buffer with panel path
 */
int fb_mode = 0;
int fb_share = 0;
struct fbi_info gfx_info;

struct lcd_regs *get_regs(struct pxa168fb_info *fbi)
{
	struct lcd_regs *regs = (struct lcd_regs *)((unsigned)fbi->reg_base);

	if (fbi->id == 0)
		regs = (struct lcd_regs *)((unsigned)fbi->reg_base + 0xc0);
	if (fbi->id == 2)
		regs = (struct lcd_regs *)((unsigned)fbi->reg_base + 0x200);

	return regs;
}
u32 dma_ctrl_read(struct pxa168fb_info *fbi, int ctrl1)
{
	u32 reg = (u32)fbi->reg_base + dma_ctrl(ctrl1, fbi->id);

	return __raw_readl(reg);
}

void dma_ctrl_write(struct pxa168fb_info *fbi, int ctrl1, u32 value)
{
	u32 reg = (u32)fbi->reg_base + dma_ctrl(ctrl1, fbi->id);

	__raw_writel(value, reg);
}

void dma_ctrl_set(struct pxa168fb_info *fbi, int ctrl1, u32 mask, u32 value)
{
	u32 reg = (u32)fbi->reg_base + dma_ctrl(ctrl1, fbi->id);
	u32 tmp1, tmp2;

	tmp1 = tmp2 = __raw_readl(reg); tmp2 &= ~mask; tmp2 |= value;
	if (tmp1 != tmp2) __raw_writel(tmp2, reg);
}

void irq_mask_set(struct pxa168fb_info *fbi, u32 mask, u32 val)
{
	u32 temp = readl(fbi->reg_base + SPU_IRQ_ENA);

	temp &= ~mask; temp |= val;
	writel(temp, fbi->reg_base + SPU_IRQ_ENA);
}

void irq_status_clear(struct pxa168fb_info *fbi, u32 mask)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	writel(mi->isr_clear_mask & (~mask), fbi->reg_base + SPU_IRQ_ISR);
}

int sclk_div_get(struct pxa168fb_info *fbi)
{
	u32 val = readl(fbi->reg_base + clk_div(fbi->id));

	pr_debug("%s SCLK 0x%x\n", __func__, val);
	return val & 0xff;
}

void sclk_div_set(struct pxa168fb_info *fbi, int divider)
{
	u32 val = readl(fbi->reg_base + clk_div(fbi->id));

	val &= ~0xff; val |= divider;
	writel(val, fbi->reg_base + clk_div(fbi->id));

	pr_debug("%s divider %x SCLK 0x%x\n", __func__, divider,
		readl(fbi->reg_base + clk_div(fbi->id)));
}

int pxa168fb_spi_send(struct pxa168fb_info *fbi, void *value,
		int count, unsigned int spi_gpio_cs)
{
	u32 x, spi_byte_len;
	u8 *cmd = (u8 *)value;
	int i, err, isr, iopad;
	unsigned int timeout = 0;

	if (spi_gpio_cs != -1) {
		err = gpio_request(spi_gpio_cs, "LCD_SPI_CS");
		if (err) {
			pr_err("failed to request GPIO for LCD CS\n");
			return -1;
		}
		gpio_direction_output(spi_gpio_cs, 1);
	}
	/* get spi data size */
	spi_byte_len = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
	spi_byte_len = (spi_byte_len >> 8) & 0xff;
	/* It should be (spi_byte_len + 7) >> 3, but spi controller
	 * request set one less than bit length */
	spi_byte_len = (spi_byte_len + 8) >> 3;
	/* spi command provided by platform should be 1, 2, or 4 byte aligned */
	if(spi_byte_len == 3)
		spi_byte_len = 4;

	iopad = readl(fbi->reg_base + SPU_IOPAD_CONTROL);
	for (i = 0; i < count; i++) {
		if ((iopad & CFG_IOPADMODE_MASK) != PIN_MODE_DUMB_18_SPI)
			writel(PIN_MODE_DUMB_18_SPI, fbi->reg_base + SPU_IOPAD_CONTROL);
		if (spi_gpio_cs != -1)
			gpio_set_value(spi_gpio_cs, 0);

		irq_status_clear(fbi,  SPI_IRQ_MASK);
		switch (spi_byte_len){
		case 1:
			writel(*cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		case 2:
			writel(*(u16*)cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		case 4:
			writel(*(u32*)cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		default:
			pr_err("Wrong spi bit length\n");
		}
		cmd += spi_byte_len;
		x = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
		x |= 0x1;
		writel(x, fbi->reg_base + LCD_SPU_SPI_CTRL);
		isr = readl(fbi->reg_base + SPU_IRQ_ISR);
		timeout = 0;
		while(!(isr & SPI_IRQ_ENA_MASK)) {
			udelay(100);
			isr = readl(fbi->reg_base + SPU_IRQ_ISR);
			if (timeout++ > 100) {
				printk("SPI IRQ may miss, just skip and send "
					"the following command, count %d!\n", i);
				break;
			}
		}
		irq_status_clear(fbi,  SPI_IRQ_MASK);
		x = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
		x &= ~0x1;
		writel(x, fbi->reg_base + LCD_SPU_SPI_CTRL);
		if (spi_gpio_cs != -1)
			gpio_set_value(spi_gpio_cs, 1);
		if ((iopad & CFG_IOPADMODE_MASK) != PIN_MODE_DUMB_18_SPI)
			writel(iopad, fbi->reg_base + SPU_IOPAD_CONTROL);
	}
	if (spi_gpio_cs != -1)
		gpio_free(spi_gpio_cs);
	return 0;
}

static int pxa168fb_power(struct pxa168fb_info *fbi,
		struct pxa168fb_mach_info *mi, int on)
{
	int ret = 0;
	printk(KERN_DEBUG "fbi->active %d on %d\n", fbi->active, on);
	if ((mi->spi_ctrl != -1) && (mi->spi_ctrl & CFG_SPI_ENA_MASK))
		writel(mi->spi_ctrl, fbi->reg_base + LCD_SPU_SPI_CTRL);

	if ((mi->pxa168fb_lcd_power) && ((fbi->active && !on ) || (!fbi->active && on)))
		ret = mi->pxa168fb_lcd_power(fbi, mi->spi_gpio_cs, mi->spi_gpio_reset, on);

	if ((mi->spi_ctrl != -1) && (mi->spi_ctrl & CFG_SPI_ENA_MASK))
		writel(mi->spi_ctrl & ~CFG_SPI_ENA_MASK,
			fbi->reg_base + LCD_SPU_SPI_CTRL);
	return ret;
}

/*************************************************************************/

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
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB888UNPACK;
				else
					return PIX_FMT_BGR888UNPACK;
			}
			/* fall through */
		}
	} else {

		pxa_format = (var->nonstd >> 20) & 0xf;

		switch (pxa_format) {
		case 0:
			return PIX_FMT_RGB565;
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

static void set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt)
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
	case PIX_FMT_YUV422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 4;     var->red.length = 16;
		var->green.offset = 12;   var->green.length = 16;
		var->blue.offset = 0;    var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
		var->nonstd |= 9 << 20;
		break;
	case PIX_FMT_PSEUDOCOLOR:
		var->bits_per_pixel = 8;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 0;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		break;
	}
}

static void set_mode(struct pxa168fb_info *fbi, struct fb_var_screeninfo *var,
		     const struct fb_videomode *mode, int pix_fmt, int ystretch)
{
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
	set_pix_fmt(var, pix_fmt);

	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = max(var->xres, var->xres_virtual);
	if (ystretch && !fb_share)
		var->yres_virtual = var->yres *2;
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

static int pxa168fb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;

	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);
	if (var->bits_per_pixel == 8) {
		pr_debug("%s var->bits_per_pixel == 8\n", __func__);
		return -EINVAL;
	}

	/* compatibility mode: if the MSB of var->nonstd is 0xAA then
	 * set xres_virtual and yres_virtual to xres and yres. */
	if((var->nonstd >> 24) == 0xAA)
		COMPAT_MODE = 0x2625;

	if((var->nonstd >> 24) == 0x55)
		COMPAT_MODE = 0x0;

	/* Basic geometry sanity checks */
	if (var->xoffset + var->xres > var->xres_virtual) {
		pr_debug("%s var->xoffset(%d) + var->xres(%d) >"
			" var->xres_virtual(%d)\n", __func__,
			var->xoffset, var->xres, var->xres_virtual);
		return -EINVAL;
	}
	if (var->yoffset + var->yres > var->yres_virtual) {
		pr_debug("%s var->yoffset(%d) + var->yres(%d) >"
			" var->yres_virtual(%d)\n", __func__,
			var->yoffset, var->yres, var->yres_virtual);
		return -EINVAL;
	}
	if (var->xres + var->right_margin +
	    var->hsync_len + var->left_margin > 3500) {
		pr_debug("%s var->xres(%d) + var->right_margin(%d) + "
			"var->hsync_len(%d) + var->left_margin(%d) "
			"> 3500\n", __func__, var->xres, var->right_margin,
			var->hsync_len, var->left_margin);
		return -EINVAL;
	}
	if (var->yres + var->lower_margin +
	    var->vsync_len + var->upper_margin > 2500) {
		pr_debug("%s var->yres(%d) + var->lower_margin(%d) + "
			"var->vsync_len(%d) + var->upper_margin(%d) "
			"> 3500\n", __func__, var->yres, var->lower_margin,
			var->vsync_len, var->upper_margin);
		return -EINVAL;
	}

	/* Check size of framebuffer */
	if (var->xres_virtual * var->yres_virtual *
	    (var->bits_per_pixel >> 3) > fbi->fb_size) {
		pr_debug("%s var->xres_virtual(%d) * var->yres_virtual(%d) "
			"* (var->bits_per_pixel(%d) >> 3) > max_fb_size(%d)\n",
			__func__, var->xres_virtual, var->yres_virtual,
			var->bits_per_pixel, fbi->fb_size);
		return -EINVAL;
	}

	return 0;
}

/*
 * The hardware clock divider has an integer and a fractional
 * stage:
 *
 *	clk2 = clk_in / integer_divider
 *	clk_out = clk2 * (1 - (fractional_divider >> 12))
 *
 * Calculate integer and fractional divider for given clk_in
 * and clk_out.
 */
static void set_clock_divider(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	int divider_int;
	int needed_pixclk;
	u64 div_result;
	u32 x = 0;

	/* check whether divider is fixed by platform */
	if (mi->sclk_div){
		writel(mi->sclk_div, fbi->reg_base + clk_div(fbi->id));
		if (!var->pixclock) {
			divider_int = mi->sclk_div & CLK_INT_DIV_MASK;
			x = clk_get_rate(fbi->clk) / divider_int / 1000;
			var->pixclock = 1000000000 / x;
			pr_debug("%s pixclock %d x %d divider_int %d\n",
				__func__, var->pixclock, x, divider_int);
		}
		return;
	}

	/* Notice: The field pixclock is used by linux fb
	 * is in pixel second. E.g. struct fb_videomode &
	 * struct fb_var_screeninfo
	 */

	/* Check input values */
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
	if (!var->pixclock) {
		pr_err("Input refresh or pixclock is wrong.\n");
		return;
	}

	/* Using PLL/AXI clock. */
	x = 0x40000000;

	/* Calc divider according to refresh rate */
	div_result = 1000000000000ll;
	do_div(div_result, var->pixclock);
	needed_pixclk = (u32)div_result;

	divider_int = clk_get_rate(fbi->clk) / needed_pixclk;

	/* check whether divisor is too small. */
	if (divider_int < 2) {
		pr_warning("Warning: clock source is too slow."
				 "Try smaller resolution\n");
		divider_int = 2;
	}

	/* Set setting to reg */
	x |= divider_int;
	if (fbi->id != 1)
		writel(x, fbi->reg_base + clk_div(fbi->id));
}

static u32 dma_ctrl0_update(int active, struct pxa168fb_mach_info *mi, u32 x, u32 pix_fmt)
{
	if (active)
		x |= 0x00000100;
	else
		x &= ~0x00000100;

	pr_debug("\t\t%s active %d value 0x%x, active bit %x\n",
		__func__, active, x, x&(0x00000100));

	/* If we are in a pseudo-color mode, we need to enable
	 * palette lookup  */
	if (pix_fmt == PIX_FMT_PSEUDOCOLOR)
		x |= 0x10000000;

	/* Configure hardware pixel format */
	x &= ~(0xF << 16);
	x |= (pix_fmt >> 1) << 16;

	/* Check YUV422PACK */
	x &= ~((1 << 9) | (1 << 11) | (1 << 10) | (1 << 12));
	if (((pix_fmt >> 1) == 5) || (pix_fmt & 0x1000)) {
		x |= 1 << 9;
		x |= (mi->panel_rbswap) << 12;
		if (pix_fmt == 11)
			x |= 1 << 11;
		if (pix_fmt & 0x1000)
			x |= 1 << 10;
	} else {
		/* Check red and blue pixel swap.
		 * 1. source data swap. BGR[M:L] rather than RGB[M:L] is stored in memeory as source format.
		 * 2. panel output data swap
		 */
		x |= (((pix_fmt & 1) ^ 1) ^ (mi->panel_rbswap)) << 12;
	}
	/* enable horizontal smooth filter for both graphic and video layers */
	x |= CFG_GRA_HSMOOTH(1) | CFG_DMA_HSMOOTH(1);

	return x;
}

static int check_modex_active(int id, int on)
{
	int active = on;

	if (fb_mode == 3) {
		if (id == fb_base)
			active = 1;
		if (id == fb_dual)
			active = 0;
	} else if ((id == fb_base) && (fb_mode == 2)) {
		active = 0;
	}

	return active;
}

static void set_dma_control0(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi;
	u32 x = 0, active, pix_fmt = fbi->pix_fmt;

	dev_dbg(fbi->fb_info->dev,"Enter %s\n", __FUNCTION__);
again:
	/* Set bit to enable graphics DMA */
	if (fbi->id != 1)
		dma_ctrl_set(fbi, 0, CFG_ARBFAST_ENA(1), CFG_ARBFAST_ENA(1));

	mi = fbi->dev->platform_data;
	x = dma_ctrl_read(fbi, 0);

	active = check_modex_active(fbi->id, fbi->active);

	if (fb_mode && (fbi->id == fb_dual))
		pix_fmt = gfx_info.fbi[fb_base]->pix_fmt;
	x = dma_ctrl0_update(active, mi, x, pix_fmt);
	dma_ctrl_write(fbi, 0, x);

	if (FB_MODE_DUP) {
		gfx_info.fbi[fb_dual]->pix_fmt = fbi->pix_fmt;
		fbi = gfx_info.fbi[fb_dual];
		mi = fbi->dev->platform_data;
		goto again;
	}
}

static void set_dma_control1(struct pxa168fb_info *fbi, int sync)
{
	u32 x;

	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
again:
	/* Configure default bits: vsync triggers DMA, gated clock
	 * enable, power save enable, configure alpha registers to
	 * display 100% graphics, and set pixel command.
	 */
	x = dma_ctrl_read(fbi, 1);

	/* We trigger DMA on the falling edge of vsync if vsync is
	 * active low, or on the rising edge if vsync is active high.
	 */
	if (!(sync & FB_SYNC_VERT_HIGH_ACT))
		x |= 0x08000000;
	else
		x &= ~0x08000000;
	dma_ctrl_write(fbi, 1, x);

	if (FB_MODE_DUP) {
		fbi = gfx_info.fbi[fb_dual];
		goto again;
	}
}

static void clear_gfx_irq(struct pxa168fb_info *fbi)
{
	int isr = readl(fbi->reg_base + SPU_IRQ_ISR);

	if ((isr & gf0_imask(fbi->id)) || (isr & gf1_imask(fbi->id))) {
		/* wake up queue. */
		if (isr & gf0_imask(fbi->id))
			irq_status_clear(fbi, gf0_imask(fbi->id));
		if (isr & gf1_imask(fbi->id))
			irq_status_clear(fbi, gf1_imask(fbi->id));
		printk(KERN_ERR"fbi %d irq miss, clear isr %x\n", fbi->id, isr);
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
			clear_gfx_irq(fbi);

		if (FB_MODE_DUP) {
			fbi = gfx_info.fbi[fb_dual];
			goto again;
		}
	}

	return 0;
}

static int gfx_irq_enabled(struct pxa168fb_info *fbi)
{
	if (irqs_disabled() || !fbi->active)
		return 0;

	/* check whether path clock is disabled */
	if (readl(fbi->reg_base + clk_div(fbi->id)) & (SCLK_DISABLE))
		return 0;

	/* in modex dma may not be enabled */
	if (!(dma_ctrl_read(fbi, 0) & CFG_GRA_ENA_MASK))
		return 0;

	return (readl(fbi->reg_base + SPU_IRQ_ENA) & (gf0_imask(fbi->id) | gf1_imask(fbi->id)));
}

static void set_graphics_start(struct fb_info *info,
	int xoffset, int yoffset, int wait_vsync)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *var = &info->var;
	int pixel_offset;
	unsigned long addr;
	static int debugcount = 0;
	struct lcd_regs *regs;

	if (debugcount < 10)
		debugcount++;

	if (debugcount < 9)
		dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

	pixel_offset = (yoffset * var->xres_virtual) + xoffset;
	addr = fbi->fb_start_dma + (pixel_offset * (var->bits_per_pixel >> 3));

	if (fb_mode && (fbi->id == fb_dual)) {
		printk(KERN_DEBUG "fb_mode %d fb%d gfx address align with fb%d,"
			" change not allowed!\n", fb_mode, fb_dual, fb_base);
		return;
	}

again:
	regs = get_regs(fbi);
	writel(addr, &regs->g_0);

	if (FB_MODE_DUP) {
		fbi = gfx_info.fbi[fb_dual];
		mi = fbi->dev->platform_data;
		goto again;
	}

	/* return until the address take effect after vsync occurs */
	if (wait_vsync && fbi->wait_vsync && gfx_irq_enabled(fbi))
		wait_for_vsync(fbi);
}

static void set_dumb_panel_control(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	u32 x;

	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

	/* Preserve enable flag */
	x = readl(fbi->reg_base + intf_ctrl(fbi->id)) & 0x00000001;
	x |= (fbi->is_blanked ? 0x7 : mi->dumb_mode) << 28;
	if (fbi->id == 1) {
		/* enable AXI urgent flag */
		x |= 0xff << 16;
	} else {
		x |= mi->gpio_output_data << 20;
		x |= mi->gpio_output_mask << 12;
	}
	x |= mi->panel_rgb_reverse_lanes ? 0x00000080 : 0;
	x |= mi->invert_composite_blank ? 0x00000040 : 0;
	x |= (info->var.sync & FB_SYNC_COMP_HIGH_ACT) ? 0x00000020 : 0;
	x |= mi->invert_pix_val_ena ? 0x00000010 : 0;
	x |= (info->var.sync & FB_SYNC_VERT_HIGH_ACT) ? 0x00000008 : 0;
	x |= (info->var.sync & FB_SYNC_HOR_HIGH_ACT) ? 0x00000004 : 0;
	x |= mi->invert_pixclock ? 0x00000002 : 0;
	writel(x, fbi->reg_base + intf_ctrl(fbi->id));	/* FIXME */
}

static void set_dumb_screen_dimensions(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_var_screeninfo *v = &info->var;
	struct lcd_regs *regs = get_regs(fbi);
	struct dsi_info *di = mi->dsi;
	int x, y, vec = 10;

	if (mi->phy_type & (DSI2DPI | DSI))
		vec = ((di->lanes <= 2) ? 1 : 2) * 10 * di->bpp / 8 / di->lanes;

	dev_dbg(info->dev, "Enter %s vec %d\n", __func__, vec);
	x = v->xres + v->right_margin + v->hsync_len + v->left_margin;
	x = x * vec / 10;
	y = v->yres + v->lower_margin + v->vsync_len + v->upper_margin;

	writel((y << 16) | x, &regs->screen_size);
}

static void pxa168fb_clear_framebuffer(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;

	memset(fbi->fb_start, 0, fbi->fb_size);
}

static void set_screen(struct pxa168fb_info *fbi, struct pxa168fb_mach_info *mi)
{
	struct fb_var_screeninfo *var;
	struct lcd_regs *regs;
	struct dsi_info *di;
	u32 x, h_porch, vsync_ctrl, vec = 10;
	u32 xres, yres, xres_z, yres_z, xres_virtual, bits_per_pixel;

again:
	var = &fbi->fb_info->var;
	regs = get_regs(fbi);
	di = mi->dsi;
	xres = var->xres; yres = var->yres;
	xres_z = var->xres; yres_z = var->yres;
	xres_virtual = var->xres_virtual;
	bits_per_pixel = var->bits_per_pixel;

	if (mi->phy_type & (DSI2DPI | DSI))
		vec = ((di->lanes <= 2) ? 1 : 2) * 10 * di->bpp / 8 / di->lanes;

	if ((fb_mode) && (fbi->id == fb_dual)) {
		xres = gfx_info.xres;
		yres = gfx_info.yres;
		bits_per_pixel = gfx_info.bpp;
		xres_virtual = max(gfx_info.xres, gfx_info.xres_virtual);
	}

	dev_dbg(fbi->fb_info->dev, "fb_mode %d fbi[%d]: xres %d xres_z %d"
		" yres %d yres_z %d xres_virtual %d bits_per_pixel %d\n",
		fb_mode, fbi->id, xres, xres_z, yres, yres_z,
		xres_virtual, bits_per_pixel);

	dev_dbg(fbi->fb_info->dev, "%s reg base %p regs->screen_active 0x%p\n",
			__func__, regs, &regs->screen_active);

	/* resolution, active */
	writel((var->yres << 16) | var->xres, &regs->screen_active);

	/* pitch, pixels per line */
	x = readl(&regs->g_pitch);
	x = (x & ~0xFFFF) | ((xres_virtual * bits_per_pixel) >> 3);
	writel(x, &regs->g_pitch);

	/* resolution, src size */
	writel((yres << 16) | xres, &regs->g_size);
	/* resolution, dst size */
	writel((yres_z << 16) | xres_z, &regs->g_size_z);

		/* h porch, left/right margin */
	if (mi->phy_type & (DSI2DPI | DSI)) {
		h_porch = (var->xres + var->right_margin) * vec / 10 - var->xres;
		h_porch = (var->left_margin * vec / 10) << 16 | h_porch;
	} else
		h_porch = (var->left_margin) << 16 | var->right_margin;
	writel(h_porch, &regs->screen_h_porch);

	/* v porch, upper/lower margin */
	writel((var->upper_margin << 16) | var->lower_margin,
			&regs->screen_v_porch);

	/* vsync ctrl */
	if (mi->phy_type & (DSI2DPI | DSI))
		vsync_ctrl = 0x01330133;
	else {
		if ((fbi->id == 0) || (fbi->id == 2))
			vsync_ctrl = ((var->width + var->left_margin) << 16)
				| (var->width + var->left_margin);
		else
			vsync_ctrl = ((var->xres + var->right_margin) << 16)
				| (var->xres + var->right_margin);

	}
	writel(vsync_ctrl, &regs->vsync_ctrl);	/* FIXME */

	/* blank color */
	writel(0x00000000, &regs->blank_color);

	if (FB_MODE_DUP) {
		fbi = gfx_info.fbi[fb_dual];
		mi = fbi->dev->platform_data;
		goto again;
	}
}

static void pxa168fb_set_regs(struct fb_info *info, int wait_vsync)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	u32 x;

	pr_debug("%s fbi %d\n", __func__, fbi->id);
	/* Calculate clock divisor. */
	set_clock_divider(fbi);

	/* Configure dma ctrl regs. */
	set_dma_control1(fbi, info->var.sync);

	/* Configure global panel parameters. */
	set_screen(fbi, mi);

	/* Configure dumb panel ctrl regs & timings */
	set_dumb_panel_control(info);
	set_dumb_screen_dimensions(info);

	x = readl(fbi->reg_base + intf_ctrl(fbi->id));
	if ((x & 1) == 0)
		writel(x | 1, fbi->reg_base + intf_ctrl(fbi->id));

	if (FB_MODE_DUP) {
		x = readl(fbi->reg_base + intf_ctrl(fb_dual));
		if ((x & 1) == 0)
			writel(x | 1, fbi->reg_base + intf_ctrl(fb_dual));
		dma_ctrl_set(gfx_info.fbi[fb_dual], 0, CFG_GRA_ENA_MASK, 0);
	}

	set_graphics_start(info, info->var.xoffset,
		info->var.yoffset, wait_vsync);
	set_dma_control0(fbi);
}

static int pxa168fb_set_par(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct pxa168fb_mach_info *mi;
	struct dsi_info *di;
	int pix_fmt;

	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

	mi = fbi->dev->platform_data;
	di = mi->dsi;

	/* Determine which pixel format we're going to use */
	pix_fmt = determine_best_pix_fmt(&info->var);
	if (pix_fmt < 0)
		return pix_fmt;
	fbi->pix_fmt = pix_fmt;
	dev_dbg(info->dev, "Pixel Format = %d\n", pix_fmt);

	/* Set additional mode info */
	if (pix_fmt == PIX_FMT_PSEUDOCOLOR)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	/* convet var to video mode; del for HDMI resolutions select via app
	mode = fb_find_best_mode(var, &info->modelist);
	set_mode(fbi, var, mode, pix_fmt, 1); */
	set_pix_fmt(var, pix_fmt);
	if (!var->xres_virtual)
		var->xres_virtual = var->xres;
	if (!var->yres_virtual)
		var->yres_virtual = var->yres * 2;
	var->grayscale = 0;
	var->accel_flags = FB_ACCEL_NONE;
	var->rotate = FB_ROTATE_UR;

	dev_dbg(info->dev, "xres=%d yres=%d\n", var->xres, var->yres);

	if (fbi->id == fb_base) {
		gfx_info.xres = var->xres; gfx_info.yres = var->yres;
		gfx_info.bpp = var->bits_per_pixel;
		gfx_info.xres_virtual = var->xres_virtual;
		pr_debug("%s fb_base xres %d yres %d bpp %d x_virtual %d\n",
			__func__, gfx_info.xres, gfx_info.yres, gfx_info.bpp,
			gfx_info.xres_virtual);
	}

	if (fbi->id == fb_dual) {
		gfx_info.xres_z = var->xres; gfx_info.yres_z = var->yres;
		pr_debug("%s fb_dual xres %d yres %d\n",
			__func__, gfx_info.xres_z, gfx_info.yres_z);
	}

	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;

	if (gfx_irq_enabled(fbi)) {
		fbi->info = info;
		wait_for_vsync(fbi);
	} else
		pxa168fb_set_regs(info, 1);
	return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	return ((chan & 0xffff) >> (16 - bf->length)) << bf->offset;
}

static u32 to_rgb(u16 red, u16 green, u16 blue)
{
	red >>= 8; green >>= 8; blue >>= 8;

	return (red << 16) | (green << 8) | blue;
}

static int
pxa168fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		 unsigned int blue, unsigned int trans, struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	u32 val;

	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR && regno < 16) {
		val =  chan_to_field(red,   &info->var.red);
		val |= chan_to_field(green, &info->var.green);
		val |= chan_to_field(blue , &info->var.blue);
		fbi->pseudo_palette[regno] = val;
	}

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR && regno < 256) {
		val = to_rgb(red, green, blue);
		writel(val, fbi->reg_base + LCD_SPU_SRAM_WRDAT);
		writel(0x8300 | regno, fbi->reg_base + LCD_SPU_SRAM_CTRL);
	}

	return 0;
}

static int pxa168fb_active(struct pxa168fb_info *fbi, int active)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	int clk = 0;

	dev_dbg(fbi->fb_info->dev, "Enter %s fbi[%d] active %d\n",
			__FUNCTION__, fbi->id, active);
	if (!active && fbi->active){
		atomic_set(&fbi->w_intr, 1);
		wake_up(&fbi->w_intr_wq);

		/* disable external panel power */
		if(pxa168fb_power(fbi, mi, 0)) {
			printk(KERN_ERR"%s %d pxa168fb_power control failed!\n", __func__,__LINE__);
		}
		fbi->active = 0;

		/* disable path clock */
		clk = readl(fbi->reg_base + clk_div(fbi->id)) | SCLK_DISABLE;
		writel(clk, fbi->reg_base + clk_div(fbi->id));
	}

	if (active && !fbi->active) {
		/* enable path clock */
		clk = readl(fbi->reg_base + clk_div(fbi->id)) & (~SCLK_DISABLE);
		writel(clk, fbi->reg_base + clk_div(fbi->id));

		/* enable external panel power */
		if(pxa168fb_power(fbi, mi, 1)) {
			printk(KERN_ERR"%s %d pxa168fb_power control failed!\n", __func__,__LINE__);
		}
		/* initialize external phy if needed */
		if (mi->phy_init && mi->phy_init(fbi)) {
			pr_err("%s fbi %d phy error\n", __func__, fbi->id);
			return -EIO;
		}

		fbi->active = 1;
		set_dma_control0(fbi);
	}
	return 0;
}

static int pxa168fb_blank(int blank, struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
#ifdef CONFIG_PM
	switch (blank) {
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			/* de-activate the device */
			pxa168fb_active(fbi, 0);

			/* sync dual path behavior */
			if (fb_mode)
				pxa168fb_active(gfx_info.fbi[fb_dual], 0);

			/* allow system enter low power modes */
			wake_unlock(&idle_lock);
			break;

		case FB_BLANK_UNBLANK:
			/* activate the device */
			pxa168fb_active(fbi, 1);

			/* sync dual path behavior */
			if (FB_MODE_DUP)
				pxa168fb_active(gfx_info.fbi[fb_dual], 1);

			/* avoid system enter low power modes */
			wake_lock(&idle_lock);
			break;
		default:
			break;
	}
	return 0;
#else
	fbi->is_blanked = (blank == FB_BLANK_UNBLANK) ? 0 : 1;
	set_dumb_panel_control(info);

	return 0;
#endif
}

static int pxa168fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);
	set_graphics_start(info, var->xoffset, var->yoffset, 1);
	return 0;
}

extern irqreturn_t pxa168fb_ovly_isr(int id);
extern irqreturn_t pxa168_ovly_isr(int id);
static irqreturn_t pxa168fb_handle_irq(int irq, void *dev_id)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)dev_id;
	u32 isr = readl(fbi->reg_base + SPU_IRQ_ISR);
	u32 imask = readl(fbi->reg_base + SPU_IRQ_ENA);
	u32 id, gfx, vid, err, sts;

	do {
		gfx = isr & gfx_imasks;
		vid = isr & vid_imasks;
		err = isr & error_imasks;
		/*printk(KERN_DEBUG "isr %x, gfx %x vid %x imask %x\n",
			isr, gfx, vid, imask); */
		irq_status_clear(gfx_info.fbi[0], gfx | vid | err);

		/* video layer */
		if (vid) {
			for (id = 0; id < 2; id++) {
				sts = imask & vid & display_done_imask(id);
				if (sts)
#ifdef CONFIG_PXA168_V4L2_OVERLAY
					pxa168_ovly_isr(id);
#else
					pxa168fb_ovly_isr(id);
#endif
			}
		}

		/* graphics layer */
		if (gfx) {
			for (id = 0; id < 2; id++) {
				fbi = gfx_info.fbi[id];
				if (!fbi)
					break;
				sts = imask & gfx & (gf0_imask(id) |
						     gf1_imask(id));
				if (sts) {
					/* update registers if needed */
					if (fbi->info) {
						pxa168fb_set_regs(fbi->info, 0);
						fbi->info = NULL;
					}

					/* wake up queue. */
					atomic_set(&fbi->w_intr, 1);
					wake_up(&fbi->w_intr_wq);
				}
			}
		}

		/* LCD under run error detect */
		if (err)
			printk(KERN_ERR"LCD under run appear,ISR = %x\n", err);

		fbi = gfx_info.fbi[0];
		if (vsync_check && (isr & path_imasks(fbi->debug))) {
			id = fbi->debug;
			irq_count++;
			if (isr & gf0_imask(id))
				f0_count++;
			if (isr & gf1_imask(id))
				f1_count++;
			if (isr & display_done_imask(id))
				dispd_count++;
			if (isr & vsync_imask(id)) {
				vsync_count++;
				irq_status_clear(fbi, vsync_imask(id));
			}
			if (isr & vf0_imask(id)) {
				vf0_count++;
				irq_status_clear(fbi, vf0_imask(id));
			}
			if (isr & vf1_imask(id)) {
				vf1_count++;
				irq_status_clear(fbi, vf1_imask(id));
			}
		}
	} while ((isr = readl(gfx_info.fbi[0]->reg_base + SPU_IRQ_ISR)) &
			(vid_imasks | gfx_imasks));

	return IRQ_HANDLED;
}

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
static void debug_identify_called_ioctl(struct fb_info *info, int cmd, unsigned long arg)
{
	switch (cmd) {
	case FB_IOCTL_CLEAR_FRAMEBUFFER:
		dev_dbg(fi->dev," FB_IOCTL_CLEAR_FRAMEBUFFER\n");
		break;
	case FB_IOCTL_PUT_SWAP_GRAPHIC_RED_BLUE:
		dev_dbg(info->dev," FB_IOCTL_PUT_SWAP_GRAPHIC_RED_BLUE with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_SWAP_GRAPHIC_U_V:
		dev_dbg(info->dev," FB_IOCTL_PUT_SWAP_GRAPHIC_U_V with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_SWAP_GRAPHIC_Y_UV:
		dev_dbg(info->dev," FB_IOCTL_PUT_SWAP_GRAPHIC_Y_UV with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_VIDEO_ALPHABLEND:
		dev_dbg(info->dev," FB_IOCTL_PUT_VIDEO_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
		break;
	case FB_IOCTL_PUT_GLOBAL_ALPHABLEND:
		dev_dbg(info->dev," FB_IOCTL_PUT_GLOBAL_ALPHABLEND with arg = %08x\n",(unsigned int) arg);
		break;
	case FB_IOCTL_PUT_GRAPHIC_ALPHABLEND:
		dev_dbg(info->dev," FB_IOCTL_PUT_GRAPHIC_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
		break;

	}
}
#endif

static int pxa168_graphic_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int blendval;
	int val, mask;
	unsigned char param;
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
	debug_identify_called_ioctl(info, cmd, arg);
#endif
	dev_dbg(info->dev, "%s cmd 0x%x\n", __func__, cmd);

	switch (cmd) {

	case FB_IOCTL_CLEAR_FRAMEBUFFER:
		pxa168fb_clear_framebuffer(info);
		break;
	case FB_IOCTL_WAIT_VSYNC:
		wait_for_vsync(fbi);
		break;
	case FB_IOCTL_WAIT_VSYNC_ON:
		fbi->wait_vsync = 1;
		if (FB_MODE_DUP)
			gfx_info.fbi[fb_dual]->wait_vsync = 1;
		break;
	case FB_IOCTL_WAIT_VSYNC_OFF:
		fbi->wait_vsync = 0;
		if (FB_MODE_DUP)
			gfx_info.fbi[fb_dual]->wait_vsync = 0;
		break;
	case FB_IOCTL_PUT_VIDEO_ALPHABLEND:
		/* This puts the blending control to the Video layer */
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(0) | CFG_ALPHA(0xff);
		dma_ctrl_set(fbi, 1, mask, val);
		if (FB_MODE_DUP)
			dma_ctrl_set(gfx_info.fbi[fb_dual], 1, mask, val);
		break;

	case FB_IOCTL_PUT_GLOBAL_ALPHABLEND:
		/*  The userspace application can specify a byte value for the amount of global blend
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
			dma_ctrl_set(gfx_info.fbi[fb_dual], 1, mask, val);
		break;

	case FB_IOCTL_PUT_GRAPHIC_ALPHABLEND:
		/*  This puts the blending back to the default mode of allowing the
		 *  graphic layer to do pixel level blending.
		 */
		mask = CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK;
		val = CFG_ALPHA_MODE(1) | CFG_ALPHA(0x0);
		dma_ctrl_set(fbi, 1, mask, val);
		if (FB_MODE_DUP)
			dma_ctrl_set(gfx_info.fbi[fb_dual], 1, mask, val);
		break;

	case FB_IOCTL_SWAP_GRAPHIC_RED_BLUE:
		param = (arg & 0x1);
		mask = CFG_GRA_SWAPRB_MASK;
		val = CFG_GRA_SWAPRB(param);
		dma_ctrl_set(fbi, 0, mask, val);
		if (FB_MODE_DUP)
			dma_ctrl_set(gfx_info.fbi[fb_dual], 1, mask, val);
		break;

	case FB_IOCTL_SWAP_GRAPHIC_U_V:
		param = (arg & 0x1);
		mask = CFG_GRA_SWAPUV_MASK;
		val = CFG_GRA_SWAPUV(param);
		dma_ctrl_set(fbi, 0, mask, val);
		if (FB_MODE_DUP)
			dma_ctrl_set(gfx_info.fbi[fb_dual], 1, mask, val);
		break;

	case FB_IOCTL_SWAP_GRAPHIC_Y_UV:
		param = (arg & 0x1);
		mask = CFG_GRA_SWAPYU_MASK;
		val = CFG_GRA_SWAPYU(param);
		dma_ctrl_set(fbi, 0, mask, val);
		break;
	default:
		if (mi->ioctl)
			mi->ioctl(info, cmd, arg);
		else
			pr_warning("%s: unknown IOCTL 0x%x\n", __func__, cmd);
		break;

	}
	return 0;
}

static int pxa168fb_release(struct fb_info *info, int user)
{
	struct fb_var_screeninfo *var = &info->var;

	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

	/* Turn off compatibility mode */
	var->nonstd &= ~0xff000000;
	COMPAT_MODE = 0;

	return 0;
}

static struct fb_ops pxa168fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= pxa168fb_check_var,
	.fb_release	= pxa168fb_release,
	.fb_set_par	= pxa168fb_set_par,
	.fb_setcolreg	= pxa168fb_setcolreg,
	.fb_blank	= pxa168fb_blank,
	.fb_pan_display	= pxa168fb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_ioctl       = pxa168_graphic_ioctl,
};

static int pxa168fb_init_mode(struct fb_info *info,
			      struct pxa168fb_mach_info *mi)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int ret = 0;
	u32 total_w, total_h, refresh;
	u64 div_result;
	const struct fb_videomode *m;

	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

	/* Set default value */
	refresh = DEFAULT_REFRESH;

	/* If has bootargs, apply it first */
	if (fbi->dft_vmode.xres && fbi->dft_vmode.yres &&
	    fbi->dft_vmode.refresh) {
		/* set data according bootargs */
		var->xres = fbi->dft_vmode.xres;
		var->yres = fbi->dft_vmode.yres;
		refresh = fbi->dft_vmode.refresh;
	}

	/* try to find best video mode. */
	m = fb_find_best_mode(&info->var, &info->modelist);
	if (m)
		fb_videomode_to_var(&info->var, m);

	/* Init settings. */
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres * 2;

	if (!var->pixclock) {
		/* correct pixclock. */
		total_w = var->xres + var->left_margin + var->right_margin +
			var->hsync_len;
		total_h = var->yres + var->upper_margin + var->lower_margin +
			var->vsync_len;

		div_result = 1000000000000ll;
		do_div(div_result, total_w * total_h * refresh);
		var->pixclock = (u32)div_result;
	}
	return ret;
}

static void pxa168fb_set_default(struct pxa168fb_info *fbi,
		struct pxa168fb_mach_info *mi)
{
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	struct lcd_regs *regs = get_regs(fbi);
	u32 dma_ctrl1 = 0x2012ff81;
	u32 burst_length = (mi->burst_len == 16) ?
		CFG_CYC_BURST_LEN16 : CFG_CYC_BURST_LEN8;

	/* Configure default register values */
	writel(mi->io_pin_allocation_mode | burst_length,
			fbi->reg_base + SPU_IOPAD_CONTROL);
		/* enable 16 cycle burst length to get better formance */

	writel(0x00000000, &regs->blank_color);
	writel(0x00000000, &regs->g_1);
	writel(0x00000000, &regs->g_start);

	/* Configure default bits: vsync triggers DMA,
	 * power save enable, configure alpha registers to
	 * display 100% graphics, and set pixel command.
	 */
	if (fbi->id == 1) {
		if (mi->phy_type & (DSI2DPI | DSI))
			dma_ctrl1 = 0xa03eff00;
		else
			dma_ctrl1 = 0x203eff00;	/* FIXME */
	}

	/* configure dma trigger @ vsync rising or falling edge */
	if (!(var->sync & FB_SYNC_VERT_HIGH_ACT))
		dma_ctrl1 |= 0x08000000;
	else
		dma_ctrl1 &= ~0x08000000;

	dma_ctrl_write(fbi, 1, dma_ctrl1);
}

static int __init get_fb_size(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	max_fb_size = n;
	fb_size_from_cmd = 1;
	return 1;
}
__setup("fb_size=", get_fb_size);

static int __init get_fb_share(char *str)
{
	fb_share = 1;
	return 1;
}
__setup("fb_share", get_fb_share);

#ifdef CONFIG_PM

static int _pxa168fb_suspend(struct pxa168fb_info *fbi)
{
	struct fb_info *info = fbi->fb_info;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	/* notify others */
	fb_set_suspend(info, 1);

	/* stop graphics dma transaction */
	fbi->dma_ctrl0 = dma_ctrl_read(fbi, 0);
	dma_ctrl_set(fbi, 0, CFG_GRA_ENA_MASK, 0);

	/*If if video is playing, hide it in early suspend*/
	dma_ctrl_set(fbi, 0, CFG_DMA_ENA_MASK, 0);

	/*Before disable lcd clk, disable all lcd interrupts*/
	fbi->irq_mask = readl(fbi->reg_base + SPU_IRQ_ENA);
	irq_mask_set(fbi, 0xffffffff, 0);

	/* disable external panel power */
	if(pxa168fb_power(fbi, mi, 0)) {
		printk(KERN_ERR"%s %d pxa168fb_power control failed!\n", __func__,__LINE__);
	}
	fbi->active = 0;

	/* disable clock */
	clk_disable(fbi->clk);

	pr_debug("pxa168fb.%d suspended\n", fbi->id);
	return 0;
}

static int _pxa168fb_resume(struct pxa168fb_info *fbi)
{
	struct fb_info *info = fbi->fb_info;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

	/* enable clock */
	if (mi->sclk_src)
		clk_set_rate(fbi->clk, mi->sclk_src);
	clk_enable(fbi->clk);

	/* enable external panel power */
	if(pxa168fb_power(fbi, mi, 1)) {
		printk(KERN_ERR"%s %d pxa168fb_power control failed!\n", __func__,__LINE__);
	}
	/* register setting should retain so no need to set again.
	 * pxa168fb_set_par(info);
	 * pxa168fb_set_default(fbi, mi);
	 */

	/* initialize external phy if needed */
	if (mi->phy_init && mi->phy_init(fbi)) {
		pr_err("%s fbi %d phy error\n", __func__, fbi->id);
		return -EIO;
	}

	/*After enable lcd clk, restore lcd interrupts*/
	irq_mask_set(fbi, 0xffffffff, fbi->irq_mask);

	/* restore graphics dma after resume */
	dma_ctrl_write(fbi, 0, fbi->dma_ctrl0);
	fbi->active = 1;

	/* notify others */
	fb_set_suspend(info, 0);

	pr_debug("pxa168fb.%d resumed.\n", fbi->id);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void pxa168fb_early_suspend(struct early_suspend *h)
{
	struct pxa168fb_info *fbi = container_of(h, struct pxa168fb_info, early_suspend);

	_pxa168fb_suspend(fbi);
	wake_unlock(&idle_lock);

	return;
}
static void pxa168fb_late_resume(struct early_suspend *h)
{
	struct pxa168fb_info *fbi = container_of(h, struct pxa168fb_info, early_suspend);

	wake_lock(&idle_lock);
	_pxa168fb_resume(fbi);

	return;
}

#else

static int pxa168fb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);

	_pxa168fb_suspend(fbi);
	pdev->dev.power.power_state = mesg;
	wake_unlock(&idle_lock);

	return 0;
}

static int pxa168fb_resume(struct platform_device *pdev)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);

	wake_lock(&idle_lock);
	_pxa168fb_resume(fbi);

	return 0;
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

#endif /* CONFIG_PM */

/**********************************************************************************************************************/

static ssize_t lcd_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	struct fb_var_screeninfo *var = &fbi->fb_info->var;
	struct lcd_regs *regs = get_regs(fbi);

	printk("fbi %d base 0x%p\n", fbi->id, fbi->reg_base);
	printk(KERN_INFO"var\n");
	printk(KERN_INFO"\t xres              %4d yres              %4d\n",
			var->xres,		var->yres);
	printk(KERN_INFO"\t xres_virtual      %4d yres_virtual      %4d\n",
			var->xres_virtual,	var->yres_virtual);
	printk(KERN_INFO"\t xoffset           %4d yoffset           %4d\n",
			var->xoffset,		var->yoffset);
	printk(KERN_INFO"\t left_margin(hbp)  %4d right_margin(hfp) %4d\n",
			var->left_margin,	var->right_margin);
	printk(KERN_INFO"\t upper_margin(vbp) %4d lower_margin(vfp) %4d\n",
			var->upper_margin,	var->lower_margin);
	printk(KERN_INFO"\t hsync_len         %4d vsync_len         %4d\n",
			var->hsync_len,		var->vsync_len);
	printk(KERN_INFO"\t bits_per_pixel    %d\n", var->bits_per_pixel);
	printk(KERN_INFO"\t pixclock          %d\n", var->pixclock);
	printk(KERN_INFO"\t sync              0x%x\n", var->sync);
	printk(KERN_INFO"\t vmode             0x%x\n", var->vmode);
	printk(KERN_INFO"\t rotate            0x%x\n", var->rotate);
	printk(KERN_INFO"\n");

	printk("video layer\n");
	printk("\tv_y0        ( @%3x ) 0x%x\n", (int)(&regs->v_y0      ) & 0xfff, readl(&regs->v_y0       ) );
	printk("\tv_u0        ( @%3x ) 0x%x\n", (int)(&regs->v_u0      ) & 0xfff, readl(&regs->v_u0       ) );
	printk("\tv_v0        ( @%3x ) 0x%x\n", (int)(&regs->v_v0      ) & 0xfff, readl(&regs->v_v0       ) );
	printk("\tv_c0        ( @%3x ) 0x%x\n", (int)(&regs->v_c0      ) & 0xfff, readl(&regs->v_c0       ) );
	printk("\tv_y1        ( @%3x ) 0x%x\n", (int)(&regs->v_y1      ) & 0xfff, readl(&regs->v_y1       ) );
	printk("\tv_u1        ( @%3x ) 0x%x\n", (int)(&regs->v_u1      ) & 0xfff, readl(&regs->v_u1       ) );
	printk("\tv_v1        ( @%3x ) 0x%x\n", (int)(&regs->v_v1      ) & 0xfff, readl(&regs->v_v1       ) );
	printk("\tv_c1        ( @%3x ) 0x%x\n", (int)(&regs->v_c1      ) & 0xfff, readl(&regs->v_c1       ) );
	printk("\tv_pitch_yc  ( @%3x ) 0x%x\n", (int)(&regs->v_pitch_yc) & 0xfff, readl(&regs->v_pitch_yc ) );
	printk("\tv_pitch_uv  ( @%3x ) 0x%x\n", (int)(&regs->v_pitch_uv) & 0xfff, readl(&regs->v_pitch_uv ) );
	printk("\tv_start     ( @%3x ) 0x%x\n", (int)(&regs->v_start   ) & 0xfff, readl(&regs->v_start    ) );
	printk("\tv_size      ( @%3x ) 0x%x\n", (int)(&regs->v_size    ) & 0xfff, readl(&regs->v_size     ) );
	printk("\tv_size_z    ( @%3x ) 0x%x\n", (int)(&regs->v_size_z  ) & 0xfff, readl(&regs->v_size_z   ) );
	printk("\n");

	printk("graphic layer\n");
	printk("\tg_0         ( @%3x ) 0x%x\n", (int)(&regs->g_0      ) & 0xfff, readl(&regs->g_0     ) );
	printk("\tg_1         ( @%3x ) 0x%x\n", (int)(&regs->g_1      ) & 0xfff, readl(&regs->g_1     ) );
	printk("\tg_pitch     ( @%3x ) 0x%x\n", (int)(&regs->g_pitch  ) & 0xfff, readl(&regs->g_pitch ) );
	printk("\tg_start     ( @%3x ) 0x%x\n", (int)(&regs->g_start  ) & 0xfff, readl(&regs->g_start ) );
	printk("\tg_size      ( @%3x ) 0x%x\n", (int)(&regs->g_size   ) & 0xfff, readl(&regs->g_size  ) );
	printk("\tg_size_z    ( @%3x ) 0x%x\n", (int)(&regs->g_size_z ) & 0xfff, readl(&regs->g_size_z) );
	printk("\n");

	printk("hardware cursor\n");
	printk("\thc_start    ( @%3x ) 0x%x\n", (int)(&regs->hc_start ) & 0xfff,  readl(&regs->hc_start      ) );
	printk("\thc_size     ( @%3x ) 0x%x\n", (int)(&regs->hc_size  ) & 0xfff, readl(&regs->hc_size       ) );
	printk("\n");

	printk("screen info\n");
	printk("\tscreen_size     ( @%3x ) 0x%x\n", (int)(&regs->screen_size    ) & 0xfff, readl(&regs->screen_size     ) );
	printk("\tscreen_active   ( @%3x ) 0x%x\n", (int)(&regs->screen_active  ) & 0xfff, readl(&regs->screen_active   ) );
	printk("\tscreen_h_porch  ( @%3x ) 0x%x\n", (int)(&regs->screen_h_porch ) & 0xfff, readl(&regs->screen_h_porch  ) );
	printk("\tscreen_v_porch  ( @%3x ) 0x%x\n", (int)(&regs->screen_v_porch ) & 0xfff, readl(&regs->screen_v_porch  ) );
	printk("\n");

	printk("color\n");
	printk("\tblank_color     ( @%3x ) 0x%x\n", (int)(&regs->blank_color    ) & 0xfff, readl(&regs->blank_color     )  );
	printk("\thc_Alpha_color1 ( @%3x ) 0x%x\n", (int)(&regs->hc_Alpha_color1) & 0xfff, readl(&regs->hc_Alpha_color1 )  );
	printk("\thc_Alpha_color2 ( @%3x ) 0x%x\n", (int)(&regs->hc_Alpha_color2) & 0xfff, readl(&regs->hc_Alpha_color2 )  );
	printk("\tv_colorkey_y    ( @%3x ) 0x%x\n", (int)(&regs->v_colorkey_y   ) & 0xfff, readl(&regs->v_colorkey_y    )  );
	printk("\tv_colorkey_u    ( @%3x ) 0x%x\n", (int)(&regs->v_colorkey_u   ) & 0xfff, readl(&regs->v_colorkey_u    )  );
	printk("\tv_colorkey_v    ( @%3x ) 0x%x\n", (int)(&regs->v_colorkey_v   ) & 0xfff, readl(&regs->v_colorkey_v    )  );
	printk("\n");

	printk("control\n");
	printk("\tvsync_ctrl      ( @%3x ) 0x%x\n", (int)(&regs->vsync_ctrl      ) & 0xfff, readl(&regs->vsync_ctrl     ));

	printk("\tdma_ctrl0       ( @%3x ) 0x%x\n", (int)(dma_ctrl(0, fbi->id)) & 0xfff, dma_ctrl_read(fbi, 0));
	printk("\tdma_ctrl1       ( @%3x ) 0x%x\n", (int)(dma_ctrl(1, fbi->id)) & 0xfff, dma_ctrl_read(fbi, 1));

	printk("\tintf_ctrl       ( @%3x ) 0x%x\n", (int)(intf_ctrl(fbi->id)) & 0xfff, readl(fbi->reg_base + intf_ctrl(fbi->id)));

	printk("\tirq_enable      ( @%3x ) 0x%8x\n", (int)(SPU_IRQ_ENA) & 0xfff, readl(fbi->reg_base + SPU_IRQ_ENA));
	printk("\tirq_status      ( @%3x ) 0x%8x\n", (int)(SPU_IRQ_ISR) & 0xfff, readl(fbi->reg_base + SPU_IRQ_ISR));
	printk("\tclk_div         ( @%3x ) 0x%x\n", (int)(clk_div(fbi->id)) & 0xfff, readl(fbi->reg_base + clk_div(fbi->id)));

	printk("\n");

	return sprintf(buf, "fb %d: active %d, debug %d\n", fbi->id, fbi->active, fbi->debug);
}

static ssize_t lcd_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);
	if (value)
		fbi->debug = 1;
	else
		fbi->debug = 0;

	return size;
}

static DEVICE_ATTR(lcd, S_IRUGO | S_IWUSR, lcd_show, lcd_store);

static ssize_t vsync_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);

	return sprintf(buf, "fbi %d wait vsync: %d\n", fbi->id, fbi->wait_vsync);
}

static ssize_t vsync_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);

	if(sscanf(buf, "%d", &fbi->wait_vsync) != 1)
		printk(KERN_ERR"%s %d erro input of wait vsync flag\n",__func__,__LINE__);
	return size;
}

static DEVICE_ATTR(vsync, S_IRUGO | S_IWUSR, vsync_show, vsync_store);

/*************************************************************************/
static int pxa168fb_set_par(struct fb_info *info);
static int pxa168fb_mode_switch(int mode)
{
	struct pxa168fb_info *fbi_base = gfx_info.fbi[fb_base];
	struct pxa168fb_info *fbi = gfx_info.fbi[fb_dual];
	struct fb_info *info_base = fbi_base->fb_info;
	struct fb_info *info_dual = fbi->fb_info;

	if (fb_share) {
		printk("fb_share mode already, try clone mode w/o"
			" fb_share in cmdline\n");
		return -EAGAIN;
	}

	pr_debug("fbi_base: fb_start_dma 0x%p fb_start 0x%p fb_size %d\n",
		(int *)fbi_base->fb_start_dma, fbi_base->fb_start,
		fbi_base->fb_size);
	pr_debug("fbi     : fb_start_dma 0x%p fb_start 0x%p fb_size %d\n",
		(int *)fbi->fb_start_dma, fbi->fb_start, fbi->fb_size);

	switch (mode) {
	case 0:
		if (fb_mode) {
			/* turn off video layer */
			pxa168fb_ovly_dual(0);
			fb_mode = mode;
			fbi->fb_start_dma = fbi->fb_start_dma_bak;
			fbi->fb_start = fbi->fb_start_bak;
			pxa168fb_set_par(info_dual);
			pxa168fb_set_par(info_base);
		}
		break;
	case 1:
	case 2:
	case 3:
		if (fb_mode != mode) {
			fb_mode = mode;
			fbi->fb_start_dma = fbi_base->fb_start_dma;
			fbi->fb_start = fbi_base->fb_start;
			pxa168fb_set_par(info_base);
			/* turn on video layer */
			pxa168fb_ovly_dual(1);
		}
		break;
	default:
		break;
		;
	}

	return 0;
}
/*************************************************************************/

#define VSYNC_CHECK_TIME	(10 * HZ)
static void vsync_check_timer(unsigned long data)
{
	vsync_check = 0;
	del_timer(&vsync_timer);
	pr_info("fbi %d: irq_count %d\n", gfx_info.fbi[0]->debug, irq_count);
	pr_info("\tvsync_count %d\n",  vsync_count);
	pr_info("\tdispd_count %d\n",  dispd_count);
	pr_info("\tf0_count %d\n", f0_count);
	pr_info("\tf1_count %d\n", f1_count);
	pr_info("\tvf0_count %d\n", vf0_count);
	pr_info("\tvf1_count %d\n", vf1_count);
}

static struct proc_dir_entry *pxa168fb_proc;
static unsigned int proc_reg = 0;
static int pxa168fb_proc_write (struct file *file, const char *buffer,
                      unsigned long count, void *data)
{
	struct fb_info *info = gfx_info.fbi[0]->fb_info;
	struct pxa168fb_mach_info *mi = gfx_info.fbi[0]->dev->platform_data;
	struct dsi_info *di = mi->dsi;
	char kbuf[11], vol[11];
	int index, reg_val, i;

	if (count >= 12)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	if ('-' == kbuf[0]) {
		memcpy(vol, kbuf+1, count-1);
		proc_reg = (int) simple_strtoul(vol, NULL, 16);
		printk("reg @ 0x%x: 0x%x\n", proc_reg,
			__raw_readl(gfx_info.fbi[0]->reg_base + proc_reg));
		return count;
	} else if ('0' == kbuf[0] && 'x' == kbuf[1]) {
		/* set the register value */
		reg_val = (int)simple_strtoul(kbuf, NULL, 16);
		__raw_writel(reg_val, gfx_info.fbi[0]->reg_base + proc_reg);
		printk("set reg @ 0x%x: 0x%x\n", proc_reg,
			__raw_readl(gfx_info.fbi[0]->reg_base + proc_reg));
		return count;
	} else if ('l' == kbuf[0]) {
		for (i = 0; i < 0x300; i+=4) {
			if (!(i % 16) && i)
				printk("\n0x%3x: ", i);
			printk(" %8x", __raw_readl(gfx_info.fbi[0]->reg_base + i));
		}
		printk("\n");
		return count;
 	} else if ('d' == kbuf[0]) {
 		unsigned addr;
 		if (!di) {
 			printk("fb0 no dsi info\n");
 			return count;
 		}
 		addr = (unsigned)di->regs;
 		for (i = 0x0; i < 0x200; i+=4) {
 			if (!(i % 16))
 				printk("\n0x%3x: ", i);
 			printk(" %8x", __raw_readl(addr + i));
 		}
 		printk("\n");
 		return count;
	}

	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
	case 0:
	case 1:
	case 2:
	case 3:
		pxa168fb_mode_switch(index);
		break;

	case 4:
		memset(info->screen_base, 2, info->screen_size/2);
		memset(info->screen_base + info->screen_size/4, 8, info->screen_size/2);
		break;
	case 9:
		init_timer(&vsync_timer);
		vsync_timer.function = vsync_check_timer;
		vsync_check = 1;
		vsync_count = 0;
		dispd_count = 0;
		irq_count = 0;
		f0_count = f1_count = 0;
		vf0_count = vf1_count = 0;
		mod_timer(&vsync_timer, jiffies + 10*HZ);
		break;

	default:
		return -EINVAL;
	}

    return count;
}

static int pxa168fb_proc_read (char *buffer, char **buffer_location, off_t offset,
                            int buffer_length, int *zero, void *ptr)
{

    if(offset > 0)
        return 0;

    return sprintf(buffer, "fb_mode %d reg @ 0x%x: 0x%x\n", fb_mode,
	proc_reg, __raw_readl(gfx_info.fbi[0]->reg_base + proc_reg));
}

static int __devinit pxa168fb_probe(struct platform_device *pdev)
{
	struct pxa168fb_mach_info *mi;
	struct fb_info *info = 0;
	struct pxa168fb_info *fbi = 0;
	struct resource *res;
	struct clk *clk;
	int irq,irq_enable_mask, ret = 0;
	volatile unsigned int tmp;
	struct dsi_info *di;
	static int proc_inited;

	mi = pdev->dev.platform_data;
	if (mi == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	clk = clk_get(NULL, "LCDCLK");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get LCDCLK");
		return PTR_ERR(clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		return -ENOENT;
	}

	info = framebuffer_alloc(sizeof(struct pxa168fb_info), &pdev->dev);
	if ((info == NULL) || (!info->par))
		return -ENOMEM;

	/* Initialize private data */
	fbi = info->par;
	fbi->id = pdev->id;
	if (!fbi->id)
		memset(&gfx_info, 0, sizeof(gfx_info));

	di = mi->dsi;
	if (di) {
		printk("fb%d dsi %d di->lanes %d\n",
			fbi->id, di->id, di->lanes);
		if (di->id & 1)
			di->regs = (unsigned)ioremap_nocache(DSI1_REGS_PHYSICAL_BASE,
					sizeof(struct dsi_regs));
		else
			di->regs = (unsigned)ioremap_nocache(DSI2_REGS_PHYSICAL_BASE,
					sizeof(struct dsi_regs));
	} else
		printk("fb%d no dsi info\n", fbi->id);

	fbi->fb_info = info;
	platform_set_drvdata(pdev, fbi);
	fbi->clk = clk;
	fbi->dev = &pdev->dev;
	fbi->fb_info->dev = &pdev->dev;
	fbi->is_blanked = 0;
	fbi->debug = 0;
	fbi->active = mi->active;

	/* Initialize boot setting */
	fbi->dft_vmode.xres = mi->modes->xres;
	fbi->dft_vmode.yres = mi->modes->yres;
	fbi->dft_vmode.refresh = mi->modes->refresh;

	init_waitqueue_head(&fbi->w_intr_wq);

	/* Initialise static fb parameters */
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
			FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	strcpy(info->fix.id, mi->id);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 1;
	info->fix.ypanstep = 1;
	info->fix.ywrapstep = 0;
	info->fix.mmio_start = res->start;
	info->fix.mmio_len = res->end - res->start + 1;
	info->fix.accel = FB_ACCEL_NONE;
	info->fbops = &pxa168fb_ops;
	info->pseudo_palette = fbi->pseudo_palette;

	/* Map LCD controller registers */
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		ret = -ENOMEM;
		goto failed;
	}

	/* Allocate framebuffer memory */
	if (!fb_size_from_cmd) {
		if (mi->max_fb_size)
			max_fb_size = mi->max_fb_size;
		else
			max_fb_size = DEFAULT_FB_SIZE;
	}
	if (fb_share) {
		/* fb_share mode, allocate more memory as frame buffer */
		max_fb_size = max(max_fb_size,
			(mi->modes->xres * mi->modes->yres * 4 * 3));
	}

	max_fb_size = PAGE_ALIGN(max_fb_size);
	fbi->fb_size = max_fb_size;

	if(fb_share && (fbi->id == 1) && gfx_info.fbi[0]->fb_start) {
		fbi->fb_start = gfx_info.fbi[0]->fb_start;
		fbi->fb_start_dma = gfx_info.fbi[0]->fb_start_dma;
		printk("--share--FB DMA buffer phy addr : %x\n",
			(unsigned int)fbi->fb_start_dma);
	} else {
		fbi->fb_start = dma_alloc_writecombine(fbi->dev, max_fb_size,
				&fbi->fb_start_dma, GFP_KERNEL);

		if (!fbi->fb_start || !fbi->fb_start_dma) {
			fbi->fb_start = (void *)__get_free_pages(GFP_DMA |
				GFP_KERNEL, get_order(fbi->fb_size));
			fbi->fb_start_dma =
				(dma_addr_t)__virt_to_phys(fbi->fb_start);
		}

		if (fbi->fb_start == NULL) {
			printk("%s: no enough memory!\n", __func__);
			ret = -ENOMEM;
			goto failed;
		}
		printk("---------FB DMA buffer phy addr : %x\n",
			(unsigned int)fbi->fb_start_dma);

		memset(fbi->fb_start, 0, fbi->fb_size);
	}

	/* fill backup information for mode switch */
	fbi->fb_start_dma_bak = fbi->fb_start_dma;
	fbi->fb_start_bak = fbi->fb_start;

	info->fix.smem_start = fbi->fb_start_dma;
	info->fix.smem_len = fbi->fb_size;
	info->screen_base = fbi->fb_start;
	info->screen_size = fbi->fb_size;

	/* avoid system enter low power modes */
	wake_lock(&idle_lock);

	/* Set video mode according to platform data */
	set_mode(fbi, &info->var, mi->modes, mi->pix_fmt, 1);

	fb_videomode_to_modelist(mi->modes, mi->num_modes, &info->modelist);

	/* init video mode data */
	pxa168fb_init_mode(info, mi);

	/* enable controller clock */
	if (mi->sclk_src)
		clk_set_rate(fbi->clk, mi->sclk_src);
	clk_enable(fbi->clk);
	pr_info("fb%d: sclk_src %d clk_get_rate = %d\n", fbi->id,
		mi->sclk_src, (int)clk_get_rate(fbi->clk));

	/* Fill in sane defaults */
	pxa168fb_set_default(fbi, mi);	/* FIXME */
	pxa168fb_set_par(info);

	tmp = readl(fbi->reg_base + LCD_TOP_CTRL);
	tmp |= 0xfff0;		/* FIXME */
	writel(tmp, fbi->reg_base + LCD_TOP_CTRL);

	/* Allocate color map */
	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	/* Register irq handler */
	if (!fbi->id) {
		/* Clear the irq status before kernel startup */
		irq_status_clear(fbi, 0xFFFFFFFF);

		ret = request_irq(irq, pxa168fb_handle_irq, IRQF_DISABLED,
					 mi->id, fbi);
		if (ret < 0) {
			dev_err(&pdev->dev, "unable to request IRQ\n");
			ret = -ENXIO;
			goto failed_free_cmap;
		}
	}

	gfx_info.fbi[fbi->id] = fbi;
	/* Enable GFX interrupt */
	irq_enable_mask = gf0_imask(fbi->id) |
			  gf1_imask(fbi->id) |
			  error_irq_imasks(fbi->id);
	irq_mask_set(fbi, irq_enable_mask, irq_enable_mask);
	fbi->wait_vsync = 1;

	/* enable power supply */
	fbi->active = 0;
	if(pxa168fb_power(fbi, mi, 1)) {
			ret = -EINVAL;
			goto failed_free_irq;
		}
	fbi->active = 1;

	/* initialize external phy if needed */
	if (mi->phy_init && mi->phy_init(fbi)) {
		pr_err("%s fbi %d phy error\n", __func__, fbi->id);
		ret = -EIO;
		goto failed_free_irq;
	}

	/* Register framebuffer */
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa168-fb: %d\n", ret);
		ret = -ENXIO;
		goto failed_free_irq;
	}
	pr_info("pxa168fb: frame buffer device was loaded"
		" to /dev/fb%d <%s>.\n", info->node, info->fix.id);

#ifdef CONFIG_HAS_EARLYSUSPEND
	fbi->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	fbi->early_suspend.suspend = pxa168fb_early_suspend;
	fbi->early_suspend.resume = pxa168fb_late_resume;
        register_early_suspend(&fbi->early_suspend);
#endif

#ifdef CONFIG_PXA688_DSI
	if (mi->phy_type & (DSI2DPI | DSI)) {
		ret = device_create_file(&pdev->dev, &dev_attr_dsi);
		if (ret < 0) {
			printk("device attr create fail: %d\n", ret);
			goto failed_free_irq;
		}
	}
#endif

	ret = device_create_file(&pdev->dev, &dev_attr_lcd);
	if (ret < 0) {
		pr_err("device attr lcd create fail: %d\n", ret);
		goto failed_free_irq;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_vsync);
	if (ret < 0) {
		pr_err("device attr create fail: %d\n", ret);
		goto failed_free_irq;
	}

	if (!proc_inited) {
		proc_inited = 1;
		pxa168fb_proc = create_proc_entry ("pxa168fb" , 0666 , NULL);
		pxa168fb_proc->read_proc = pxa168fb_proc_read;
		pxa168fb_proc->write_proc = pxa168fb_proc_write;
	}

	return 0;

failed_free_irq:
	free_irq(irq, fbi);
failed_free_cmap:
	fb_dealloc_cmap(&info->cmap);
failed_free_clk:
	clk_disable(fbi->clk);
	clk_put(fbi->clk);
failed:
	pr_err("pxa168-fb: frame buffer device init failed\n");
	platform_set_drvdata(pdev, NULL);
	fb_dealloc_cmap(&info->cmap);

	if (fbi && fbi->reg_base) {
		iounmap(fbi->reg_base);
		kfree(fbi);
	}

	wake_unlock(&idle_lock);
	return ret;
}

static struct platform_driver pxa168fb_driver = {
	.driver		= {
		.name	= "pxa168-fb",
		.owner	= THIS_MODULE,
	},
	.probe		= pxa168fb_probe,
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= pxa168fb_suspend,
	.resume		= pxa168fb_resume,
#endif
#endif
};

static int __devinit pxa168fb_init(void)
{
	wake_lock_init(&idle_lock, WAKE_LOCK_IDLE, "pxa168fb_idle");
	return platform_driver_register(&pxa168fb_driver);
}
module_init(pxa168fb_init);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@marvell.com>");
MODULE_DESCRIPTION("Framebuffer driver for PXA168");
MODULE_LICENSE("GPL");
