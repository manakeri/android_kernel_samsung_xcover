/*
 *  linux/drivers/video/pxa95xfb_ovly.c
 */

#include "pxa95xfb.h"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
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

#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/pxa3xx-regs.h>

#include <mach/gpio.h>
#include <mach/pxa95xfb.h>

/* buffer management:
 *    freelist: list return to upper layer which indicates buff is free
 *    waitlist: wait queue which indicates "using" buffer, waitlist[0] is write in dma desc
 *    current: buffer on panel
 * Operation:
 *    flip: if !waitlist[0] set buf; enqueue buf to waitlist
 *    get freelist: return freelist
 *    eof intr: enqueue current to freelist; dequeue waitlist[0] to current; if !new waitlist[0] set buf;
 *    buffers are protected by mutex: ?? how to protect in intr?
 * suspend:
 *    when suspend & get freelist, simulate eof intr one time
 */

static int convert_pix_fmt(u32 vmode)
{
	pr_info("vmode=%d\n", vmode);
	switch (vmode) {
		case FB_VMODE_YUV422PACKED:
			return PIX_FMTIN_YUV422IL;
		case FB_VMODE_YUV422PLANAR:
			return PIX_FMTIN_YUV422;
		case FB_VMODE_YUV420PLANAR:
			return PIX_FMTIN_YUV420;
			/* TODO - add U/V and R/B SWAP format and YUV444 */
		case FB_VMODE_RGB565:
			return PIX_FMTIN_RGB_16;
		case FB_VMODE_RGB888PACK:
			return PIX_FMTIN_RGB_24_PACK;
		case FB_VMODE_RGBA888:
			return PIX_FMTIN_RGB_32;
		case FB_VMODE_RGB888UNPACK:
			return PIX_FMTIN_RGB_24;
		default:
			return -1;
	}
}

/* pxa95x lcd controller could only resize in step. Adjust offset in such case to keep picture in center */
static void adjust_offset_for_resize(struct _sViewPortInfo *info, struct _sViewPortOffset *offset)
{
	int wstep, hstep;

	if ((info->zoomXSize == info->srcWidth && info->zoomYSize == info->srcHeight)
		|| !info->zoomXSize || !info->zoomYSize)
		return;

	wstep = (info->zoomXSize > info->srcWidth) ? (info->srcWidth /8) : (info->srcWidth /32);
	hstep = (info->zoomYSize > info->srcHeight) ? (info->srcHeight /8) : (info->srcHeight /32);

	offset->xOffset += (info->zoomXSize % wstep) / 2;
	offset->yOffset += (info->zoomYSize % hstep) / 2;
	info->zoomXSize = (info->zoomXSize / wstep) * wstep;
	info->zoomYSize = (info->zoomYSize / hstep) * hstep;
}

static int check_surface(struct fb_info *fi,
		FBVideoMode new_mode,
		struct _sViewPortInfo *new_info,
		struct _sViewPortOffset *new_offset,
		struct _sVideoBufferAddr *new_addr)
{
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	int changed = 0;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);

	if (new_info && new_offset)
		adjust_offset_for_resize(new_info, new_offset);

	/*
	 * Check buffer address: most case we need not to set mixer but only to set dma descriptor
	 */
	if (new_addr && new_addr->startAddr &&
			fbi->user_addr != (unsigned long)new_addr->startAddr) {
		fbi->user_addr = (unsigned long)new_addr->startAddr;
		changed = 0;
	}


	/*
	 * check mode
	 */
	if (new_mode >= 0 && fbi->surface.videoMode != new_mode) {
		fbi->surface.videoMode = new_mode;
		fbi->pix_fmt = convert_pix_fmt(new_mode);
		lcdc_set_pix_fmt(var, fbi->pix_fmt);
		fbi->bpp = var->bits_per_pixel;
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
			 fbi->surface.viewPortInfo.ycPitch != new_info->ycPitch)) {
		fbi->surface.viewPortInfo.srcWidth = new_info->srcWidth;
		fbi->surface.viewPortInfo.srcHeight = new_info->srcHeight;
		fbi->surface.viewPortInfo.zoomXSize = new_info->zoomXSize;
		fbi->surface.viewPortInfo.zoomYSize = new_info->zoomYSize;
		fbi->surface.viewPortInfo.ycPitch =
			(new_info->ycPitch) ? new_info->ycPitch : new_info->srcWidth*fbi->bpp/8;
		pr_info("Ovly update: [%d %d] - [%d %d], ycpitch %d\n",
			new_info->srcWidth, new_info->srcHeight,
			new_info->zoomXSize, new_info->zoomYSize,
			fbi->surface.viewPortInfo.ycPitch);
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
		pr_info("Ovly update offset [%d %d]\n",
			new_offset->xOffset, new_offset->yOffset);
		changed = 1;
	}


	return changed;
}

static void __attribute__ ((unused)) buf_print(struct pxa95xfb_info *fbi)
{
	int i;
	printk("Curent buff: %x \n", fbi->buf_current);

	printk("buf_waitlist:");
	for (i = 0; i < MAX_QUEUE_NUM; i++) {
		if (fbi->buf_waitlist[i])
			printk("<%d: %x> ", i, fbi->buf_waitlist[i]);
	}
	printk("\n");

	printk("buf_freelist:");
	for (i = 0; i < MAX_QUEUE_NUM; i++) {
		if (fbi->buf_freelist[i])
			printk("<%d: %x> ", i, fbi->buf_freelist[i]);
	}
	printk("\n");
}

static u32 buf_dequeue(u32 *list)
{
	int i;
	u32 ret;

	if (!list){
		printk(KERN_ALERT "%s: invalid list\n", __func__);
		return -1;
	}

	ret = list[0];

	for (i = 1; i < MAX_QUEUE_NUM; i++) {
		if (!list[i]){
			list[i-1] = 0;
			break;
		}
		list[i-1] = list[i];
		/*printk(KERN_INFO "%s: move buff %x from list[%d] to list[%d]\n", __func__, list[i], i, i-1);*/
	}

	if (i >= MAX_QUEUE_NUM)
		printk(KERN_ALERT "%s: buffer overflow\n",  __func__);

	/*printk(KERN_INFO "%s: dequeue: %x\n", __func__, ret);*/
	return ret;
}

static int buf_enqueue(u32 *list, u32 buf)
{
	int i;

	if (!list){
		printk(KERN_ALERT "%s: invalid list\n", __func__);
		return -1;
	}

	for (i = 0; i < MAX_QUEUE_NUM; i++) {
		if (!list[i]) {
			list[i] = buf;
			/*printk(KERN_INFO "%s: add buff %x to list[%d]\n", __func__, buf, i);*/
			return 0;
		}

		if (list[i] == buf) {
			/* already in list, free this request. */
			printk(KERN_WARNING "%s: buff %x is same as list[%d]\n", __func__, buf, i);
			return 0;
		}
	}

	if (i >= MAX_QUEUE_NUM)
		printk(KERN_ALERT "%s: buffer overflow\n",  __func__);

	return -2;
}

static void buf_clear(u32 *list)
{
	/* Check null pointer. */
	if (list)
		memset(list, 0, MAX_QUEUE_NUM * sizeof(u8 *));
}

/* fake endframe when suspend or overlay off */
static void buf_fake_endframe(void * p)
{
	struct pxa95xfb_info *fbi = p;
	u32 t = buf_dequeue(fbi->buf_waitlist);
	if (!t) {
		pr_info("buffer empty even for fake endframe");
		return;
	}

	/*pr_info("%s: move %x to current, move %x to freelist\n",
		__func__, t, fbi->buf_current);*/
	/*enqueue current to freelist*/
	buf_enqueue(fbi->buf_freelist, fbi->buf_current);
	/*dequeue waitlist[0] to current*/
	fbi->buf_current = t;

}

static void buf_endframe(void * p)
{
	struct pxa95xfb_info *fbi = p;
	u32 t, f = lcdc_get_fr_addr(fbi);
	if (!f || f == fbi->buf_waitlist[0]) {
		t = buf_dequeue(fbi->buf_waitlist);
		if (t) {
			/*printk(KERN_INFO "%s: move %x to current, move %x to freelist\n",
				__func__, t, fbi->buf_current);*/
			/*enqueue current to freelist*/
			buf_enqueue(fbi->buf_freelist, fbi->buf_current);
			/*dequeue waitlist[0] to current*/
			fbi->buf_current = t;
		}
	}
	/*if new waitlist[0] set buf*/
	if(fbi->buf_waitlist[0]){
		/*printk(KERN_INFO "%s: flip buf %x on\n", __func__, fbi->buf_waitlist[0]);*/
		fbi->user_addr = fbi->buf_waitlist[0];
		lcdc_set_fr_addr(fbi, &fbi->fb_info->var);
	}
}

static int pxa95xfb_vid_open(struct fb_info *fi, int user)
{
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	unsigned long x;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
	fbi->open_count ++;
	fbi->user_addr = 0;
	fbi->surface.videoMode = -1;
	fbi->surface.viewPortInfo.srcWidth = var->xres;
	fbi->surface.viewPortInfo.ycPitch = var->xres * fbi->bpp / 8;
	fbi->surface.viewPortInfo.srcHeight = var->yres;

	if(mutex_is_locked(&fbi->access_ok))
		mutex_unlock(&fbi->access_ok);

	local_irq_save(x);
	/* clear buffer list. */
	buf_clear(fbi->buf_freelist);
	buf_clear(fbi->buf_waitlist);
	fbi->buf_current = 0;
	local_irq_restore(x);
	return 0;
}

static int pxa95xfb_vid_release(struct fb_info *fi, int user)
{
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	unsigned long x;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
	local_irq_save(x);
	/* clear buffer list. */
	buf_clear(fbi->buf_freelist);
	buf_clear(fbi->buf_waitlist);
	fbi->buf_current = 0;
	local_irq_restore(x);

	fbi->user_addr = 0;
	fbi->surface.videoMode = -1;
	fbi->surface.viewPortInfo.srcWidth = var->xres;
	fbi->surface.viewPortInfo.srcHeight = var->yres;
	fbi->surface.viewPortInfo.ycPitch = var->xres * fbi->bpp / 8;
	/* Turn off compatibility mode */
	var->nonstd &= ~0xff000000;
	fbi->open_count --;
	return 0;
}

static int overlay_dump_buffer = 0;

static int pxa95xfb_vid_ioctl(struct fb_info *fi, unsigned int cmd,
		unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pxa95xfb_info *fbi = (struct pxa95xfb_info *)fi->par;
	static struct _sOvlySurface gOvlySurface;
	int on;
	unsigned long x;

	switch (cmd) {
	case FB_IOCTL_WAIT_VSYNC:
		lcdc_wait_for_vsync(fbi);
		break;
	case FB_IOCTL_GET_VIEWPORT_INFO:
		return copy_to_user(argp, &gOvlySurface.viewPortInfo,
				sizeof(struct _sViewPortInfo)) ? -EFAULT : 0;
	case FB_IOCTL_SET_VIEWPORT_INFO:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gOvlySurface.viewPortInfo, argp,
				sizeof(gOvlySurface.viewPortInfo))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		if (gOvlySurface.viewPortInfo.zoomXSize*
			gOvlySurface.viewPortInfo.zoomYSize
			> 1280*720) {
			printk(KERN_NOTICE
				"SET_VIEWPORT_INFO: %d %d un-supported\n",
				gOvlySurface.viewPortInfo.zoomXSize,
				gOvlySurface.viewPortInfo.zoomYSize);
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_SET_VIDEO_MODE:
		/*Get data from user space.
		 *return error for not supported format
		 */
		if (copy_from_user(&gOvlySurface.videoMode,
				argp, sizeof(gOvlySurface.videoMode))
			|| convert_pix_fmt(gOvlySurface.videoMode) < 0)
			return -EFAULT;
		break;
	case FB_IOCTL_GET_VIDEO_MODE:
		return copy_to_user(argp, &gOvlySurface.videoMode,
				sizeof(u32)) ? -EFAULT : 0;
	case FB_IOCTL_SET_VID_OFFSET:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gOvlySurface.viewPortOffset,
			argp,
			sizeof(gOvlySurface.viewPortOffset))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_GET_VID_OFFSET:
		return copy_to_user(argp,
			&gOvlySurface.viewPortOffset,
			sizeof(struct _sViewPortOffset)) ? -EFAULT : 0;
	case FB_IOCTL_GET_SURFACE:
		return copy_to_user(argp, &gOvlySurface,
				sizeof(struct _sOvlySurface)) ? -EFAULT : 0;
	case FB_IOCTL_SET_SURFACE:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gOvlySurface, argp,
					sizeof(struct _sOvlySurface))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		if (gOvlySurface.viewPortInfo.zoomXSize *
			gOvlySurface.viewPortInfo.zoomYSize
			> 1280*720) {
			printk(KERN_NOTICE
				"SET_SURFACE: %d %d un-supported\n",
				gOvlySurface.viewPortInfo.zoomXSize,
				gOvlySurface.viewPortInfo.zoomYSize);
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}
		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_FLIP_VID_BUFFER:
	{
		struct _sOvlySurface surface;
		u8 *start_addr, *input_data;
		u32 t;

		/* Get user-mode data. */
		if (copy_from_user(&surface, argp,
					sizeof(struct _sOvlySurface)))
			return -EFAULT;

		start_addr = surface.videoBufferAddr.startAddr;
		input_data = surface.videoBufferAddr.inputData;

		if (fbi->on && !fbi->controller_on) {
			check_surface(fi, surface.videoMode,
						&surface.viewPortInfo,
						&surface.viewPortOffset, 0);
			WARN_ON(fbi->buf_waitlist[0]);
			local_irq_save(x);
			/*if !waitlist[0] enqueue buf to waitlist*/
			/*printk(KERN_INFO "%s: flip %x on\n",
			__func__, (u32)start_addr);*/
			fbi->user_addr = (u32)start_addr;
			lcdc_set_fr_addr(fbi, &fbi->fb_info->var);
			buf_enqueue(fbi->buf_waitlist, (u32)start_addr);
			local_irq_restore(x);
			converter_openclose(fbi, fbi->on);
			if(!pxa95xfbi[0]->suspend)
				lcdc_set_lcd_controller(fbi);
			fbi->controller_on = fbi->on;
			return 0;
		}


		/* Fix the first green frames when camera preview */
		if( !fbi->controller_on ) {
			buf_enqueue(fbi->buf_freelist, (u32)start_addr);
			return 0;
		}
		if (start_addr && !input_data) {
			if (overlay_dump_buffer)
				printk("flip %x\n", (u32)start_addr);
			/* user pointer way */
			if (check_surface(fi, surface.videoMode,
						&surface.viewPortInfo,
						&surface.viewPortOffset, 0)
					&& !pxa95xfbi[0]->suspend) {
				/* in this case, surface mode changed, need sync */
				lcdc_set_lcd_controller(fbi);
				local_irq_save(x);
				while ((t = buf_dequeue(&fbi->buf_waitlist[1])))
					buf_enqueue(fbi->buf_freelist, (u32)t);
				buf_enqueue(fbi->buf_waitlist, (u32)start_addr);
				local_irq_restore(x);
			} else {
				local_irq_save(x);
				/*if !waitlist[0] enqueue buf to waitlist*/
				if (!fbi->buf_waitlist[0]) {
					/*printk(KERN_INFO "%s: flip %x on\n",
					__func__, (u32)start_addr);*/
					fbi->user_addr = (u32)start_addr;
					lcdc_set_fr_addr(fbi, &fbi->fb_info->var);
				}
				buf_enqueue(fbi->buf_waitlist, (u32)start_addr);
				local_irq_restore(x);
			}
		} else if  (input_data) {
			/* copy buffer way*/
			lcdc_wait_for_vsync(fbi);
			if (check_surface(fi, surface.videoMode,
						&surface.viewPortInfo,
						&surface.viewPortOffset, 0)
					&& !pxa95xfbi[0]->suspend)
				lcdc_set_lcd_controller(fbi);
			/* if support hw DMA, replace this. */
			return copy_from_user(fbi->fb_start, input_data,
					surface.videoBufferAddr.length);
		}
		return 0;
	}
	case FB_IOCTL_GET_FREELIST:
		local_irq_save(x);
		/* safe check: when lcd is suspend,
		 * move all buffers as "switched"*/
		if (pxa95xfbi[0]->suspend || !fbi->on)
			buf_fake_endframe(fbi);
		if (copy_to_user(argp, fbi->buf_freelist,
					MAX_QUEUE_NUM*sizeof(u8 *))) {
			local_irq_restore(x);
			return -EFAULT;
		}
		buf_clear(fbi->buf_freelist);
		local_irq_restore(x);
		return 0;
	case FB_IOCTL_GET_BUFF_ADDR:
		return copy_to_user(argp, &fbi->surface.videoBufferAddr,
				sizeof(struct _sVideoBufferAddr)) ? -EFAULT : 0;
	case FB_IOCTL_SET_MEMORY_TOGGLE:
		break;
	case FB_IOCTL_SET_COLORKEYnALPHA:
		if (copy_from_user(&fbi->ckey_alpha, argp,
					sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;
		if (!pxa95xfbi[0]->suspend)
			lcdc_set_colorkeyalpha(fbi);
		break;
	case FB_IOCTL_GET_COLORKEYnALPHA:
		if (copy_to_user(argp, &fbi->ckey_alpha,
					sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;
		break;
	case FB_IOCTL_SWITCH_VID_OVLY:
		if (copy_from_user(&on, argp, sizeof(int)))
			return -EFAULT;
		if (on == fbi->on)
			printk(KERN_INFO "PXA95xfb ovly: video already: %s\n",
				fbi->on ? "on" : "off");
		else {
			fbi->on = on;
			printk(KERN_INFO "PXA95xfb ovly: video switch: %s\n",
				fbi->on ? "on" : "off");
			if (!fbi->on && fbi->controller_on) {
				if(!pxa95xfbi[0]->suspend)
					lcdc_set_lcd_controller(fbi);
				converter_openclose(fbi, on);
				fbi->controller_on = 0;
				fbi->user_addr = 0;
			}

		}
		break;
	case FB_IOCTL_SWITCH_GRA_OVLY:
		if (copy_from_user(&on, argp, sizeof(int)))
			return -EFAULT;
		if (on == pxa95xfbi[0]->on) {
			printk(KERN_INFO "PXA95xfb ovly: video already: %s\n",
				pxa95xfbi[0]->on ? "on" : "off");
		} else {
			pxa95xfbi[0]->on = on;
			printk(KERN_INFO "PXA95xfb ovly: graphic switch: %s\n",
				pxa95xfbi[0]->on ? "on" : "off");
			converter_openclose(pxa95xfbi[0], on);
			if(!pxa95xfbi[0]->suspend)
				lcdc_set_lcd_controller(pxa95xfbi[0]);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int pxa95xfb_vid_blank(int blank, struct fb_info *info)
{
	struct pxa95xfb_info *fbi = info->par;

	fbi->is_blanked = (blank == FB_BLANK_UNBLANK) ? 0 : 1;

	return 0;	/* TODO */
}

static struct fb_ops pxa95xfb_vid_ops = {
	.owner		= THIS_MODULE,
	.fb_open        = pxa95xfb_vid_open,
	.fb_release     = pxa95xfb_vid_release,
	.fb_blank		= pxa95xfb_vid_blank,
	.fb_ioctl       = pxa95xfb_vid_ioctl,
	.fb_set_par		= pxa95xfb_set_par,
	.fb_check_var	= pxa95xfb_check_var,
	.fb_setcolreg	= pxa95xfb_setcolreg,	/* TODO */
	.fb_pan_display	= pxa95xfb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int __devinit pxa95xfb_vid_probe(struct platform_device *pdev)
{
	struct pxa95xfb_mach_info *mi;
	struct fb_info *info = 0;
	struct pxa95xfb_info *fbi = 0;
	struct resource *res;
	struct clk *clk = NULL;
	int irq_conv, ret = 0, i;
	struct pxa95xfb_conv_info *conv;

	mi = pdev->dev.platform_data;
	if (mi == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}
	printk(KERN_INFO "vid probe: %s.%d: %s\n", pdev->name, pdev->id, mi->id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		return -ENOENT;
	}

	if (mi->converter == LCD_M2DSI0) {
		clk = clk_get(&pdev->dev, "PXA95x_DSI0CLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get DSI0 CLK");
			goto failed;
		}
	} else if (mi->converter == LCD_M2DSI1) {
		clk = clk_get(&pdev->dev, "PXA95x_DSI1CLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get DSI1 CLK");
			goto failed;
		}
	} else if (mi->converter == LCD_M2HDMI) {
		clk = clk_get(&pdev->dev, "PXA95x_iHDMICLK");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "unable to get internal HDMI CLK");
			goto failed;
		}
	}

	if(mi->converter != LCD_MIXER_DISABLE){
		irq_conv = platform_get_irq(pdev, mi->converter);
		if (irq_conv < 0) {
			dev_err(&pdev->dev, "no IRQ defined\n");
			ret = -ENOMEM;
			goto failed_free_clk;
		}
	}else{
		dev_err(&pdev->dev, "no converter defined\n");
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	info = framebuffer_alloc(sizeof(struct pxa95xfb_info), &pdev->dev);
	if (info == NULL){
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	/* Initialize private data */
	fbi = info->par;
	if(!fbi) {
		ret = -EINVAL;
		goto failed_free_clk;
	}
	fbi->fb_info = info;
	platform_set_drvdata(pdev, fbi);
	fbi->dev = &pdev->dev;
	fbi->id = pdev->id;
	fbi->on = 0;
	fbi->controller_on = 0;
	fbi->active = 0;
	fbi->open_count = fbi->on;/*if fbi on as default, open count +1 as default*/
	fbi->is_blanked = 0;
	fbi->suspend = 0;
	fbi->debug = 0;
	fbi->window = mi->window;
	fbi->zorder = mi->zorder;
	fbi->mixer_id = mi->mixer_id;
	fbi->converter = mi->converter;
	fbi->user_addr = 0;
	fbi->eof_intr_en = 1;
	fbi->vsync_en = 0;
	fbi->eof_handler = buf_endframe;

	mutex_init(&fbi->access_ok);
	init_waitqueue_head(&fbi->w_intr_wq);

	memset(&fbi->surface, 0, sizeof(fbi->surface));
	memset(&fbi->mode, 0, sizeof(struct fb_videomode));

	/* Map LCD controller registers.*/
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	/* Allocate framebuffer memory.*/
	if (mi->output == OUTPUT_HDMI)
		fbi->fb_size = PAGE_ALIGN(mi->modes[0].xres * mi->modes[0].yres * 2 + PAGE_SIZE);
	else
		fbi->fb_size = PAGE_ALIGN(mi->modes[0].xres * mi->modes[0].yres * 4 + PAGE_SIZE);
	fbi->fb_start = dma_alloc_writecombine(fbi->dev, fbi->fb_size + PAGE_SIZE,
			&fbi->fb_start_dma,
			GFP_KERNEL);
	if (fbi->fb_start == NULL) {
		ret = -ENOMEM;
		goto failed_free_clk;
	}
	memset(fbi->fb_start, 0x0, fbi->fb_size);
	fbi->fb_start = fbi->fb_start + PAGE_SIZE;
	fbi->fb_start_dma = fbi->fb_start_dma + PAGE_SIZE;

	/* Initialise static fb parameters.*/
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
	info->fbops = &pxa95xfb_vid_ops;
	info->pseudo_palette = fbi->pseudo_palette;
	info->fix.smem_start = fbi->fb_start_dma;
	info->fix.smem_len = fbi->fb_size;
	info->screen_base = fbi->fb_start;
	info->screen_size = fbi->fb_size;

	for(i = 0; i < mi->num_modes; i++)
		lcdc_correct_pixclock(&mi->modes[i]);
	fb_videomode_to_modelist(mi->modes, mi->num_modes, &info->modelist);
	/* init var: according to modes[0] */
	lcdc_set_pix_fmt(&info->var, mi->pix_fmt_in);
	lcdc_set_mode_to_var(fbi, &info->var, &mi->modes[0]);
	memcpy(&fbi->mode, &mi->modes[0], sizeof(struct fb_videomode));
	info->var.xoffset = info->var.yoffset = 0;

	pxa95xfbi[fbi->id+1] = fbi;

	/*init converter*/
	conv = &pxa95xfb_conv[fbi->converter -1];
	if(!conv->inited){
		conv->inited = 1;
		conv->output = mi->output;
		conv->clk = clk;
		conv->irq = irq_conv;
		conv->pix_fmt_out = mi->pix_fmt_out;
		conv->active = mi->active;
		conv->invert_pixclock = mi->invert_pixclock;
		conv->panel_rbswap = mi->panel_rbswap;
		conv->panel_type = mi->panel_type;
		conv->power = mi->panel_power;
		conv->reset = mi->reset;

		converter_init(fbi);
	}

	if(fbi->on)
		converter_openclose(fbi, 1);

	pxa95xfb_set_par(info);

	/* Allocate color map.*/
	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed_free_irq_conv;
	}

	/* Register framebuffer.*/
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa95x-fb: %d\n", ret);
		ret = -ENXIO;
		goto failed_free_cmap;
	}
	printk(KERN_INFO "pxa95xfb_ovly: frame buffer device %s was loaded"
			" to /dev/fb%d <%s>.\n", conv->output?"HDMI":"PANEL", info->node, info->fix.id);

	return 0;

failed_free_cmap:
	fb_dealloc_cmap(&info->cmap);
failed_free_irq_conv:
	free_irq(irq_conv, fbi);
failed_free_clk:
	clk_disable(clk);
	clk_put(clk);
failed:
	pr_err("pxa95xfb-ovly: frame buffer device init failed\n");
	platform_set_drvdata(pdev, NULL);

	if (fbi && fbi->reg_base) {
		iounmap(fbi->reg_base);
		kfree(fbi);
	}

	return ret;
}

static struct platform_driver pxa95xfb_vid_driver = {
	.driver		= {
		.name	= "pxa95xfb-ovly",
		.owner	= THIS_MODULE,
	},
	.probe		= pxa95xfb_vid_probe,
};

static int __devinit pxa95xfb_vid_init(void)
{
	return platform_driver_register(&pxa95xfb_vid_driver);
}
/* module_init(pxa95xfb_init); */
late_initcall(pxa95xfb_vid_init);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@marvell.com>");
MODULE_DESCRIPTION("Framebuffer driver for PXA95x");
MODULE_LICENSE("GPL");
