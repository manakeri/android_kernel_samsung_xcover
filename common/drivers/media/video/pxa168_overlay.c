/*
 * PXA168 v4l2 overlay driver
 *
 * adapted from pxa168_vout.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/videodev2.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/slab.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/videobuf-dma-contig.h>

#include <asm/processor.h>
#include <mach/dma.h>
#include <mach/pxa168fb.h>
#include "../../video/pxa168fb.h"

MODULE_AUTHOR("Jun Nie");
MODULE_DESCRIPTION("PXA168 Video out driver");
MODULE_LICENSE("GPL");

/* configuration macros */
#define VOUT_NAME		"pxa168_v4l2_ovly"

#define VID_MAX_WIDTH		1920	/* Largest width */
#define VID_MAX_HEIGHT		1080	/* Largest height */
#define VID_QVGA_WIDTH		320
#define VID_QVGA_HEIGHT		240
#define VID_MIN_WIDTH		16
#define VID_MIN_HEIGHT		16

#define MAX_BUF_SIZE (VID_MAX_WIDTH*VID_MAX_HEIGHT*4)

/* max control: hue, alpha, chroma key, contrast, saturation, gamma */
#define MAX_CID		5

#define COLOR_KEY_GFX_DST 0
#define COLOR_KEY_VID_SRC 1

/*
 * Maximum amount of memory to use for rendering buffers.
 * Default is enough to four (RGB24) DVI 720P buffers.
 */
#define MAX_ALLOWED_VIDBUFFERS            4

const static struct v4l2_fmtdesc pxa168_formats[] = {
	{
	 .description = "YUV420, planer",
	 .pixelformat = V4L2_PIX_FMT_YUV420,
	},
	{
	 .description = "YVU420, planer",
	 .pixelformat = V4L2_PIX_FMT_YVU420,
	},
	{
	 .description = "YUV422, planer",
	 .pixelformat = V4L2_PIX_FMT_YUV422P,
	},
	{
	 .description = "YUYV (YUV 4:2:2), packed",
	 .pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
	 .description = "UYVY (YUV 4:2:2), packed",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	},
	{
	 .description = "RGB565",
	 .pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
	 .description = "RGB555",
	 .pixelformat = V4L2_PIX_FMT_RGB555X,
	},
	{
	 .description = "RGB8888, unpacked",
	 .pixelformat = V4L2_PIX_FMT_RGB32,
	},
	{
	 .description = "BGR8888, unpacked",
	 .pixelformat = V4L2_PIX_FMT_BGR32,
	},
	{
	 .description = "RGB888, packed",
	 .pixelformat = V4L2_PIX_FMT_RGB24,
	},
	{
	 .description = "BGR888, packed",
	 .pixelformat = V4L2_PIX_FMT_BGR24,
	},
};
#define NUM_OUTPUT_FORMATS (ARRAY_SIZE(pxa168_formats))

struct pxa168_overlay {
	int id;
	struct device *dev;
	const char *name;
	void *reg_base;
	struct clk *clk;

	/* dynamic fields */
	int state;

	struct video_device *vdev;
	int opened;

	bool enabled;
	bool alpha_enabled;
	u32 paddr;
	u16 ypitch;
	u16 uvpitch;
	int y_size;
	int uv_size;
	int trans_enabled;
	int trans_key_type;
	int trans_key;
	bool mirror;

	u16 pos_x;
	u16 pos_y;
	u16 out_width;		/* if 0, out_width == width */
	u16 out_height;		/* if 0, out_height == height */
	u8 global_alpha;
	struct _sColorKeyNAlpha ckey_alpha;	/* fix me */
	u32 dma_ctl0;
	bool update;		/* if 1, update the overlay control info */

	spinlock_t vbq_lock;	/* spinlock for videobuf queues */

	/* non-NULL means streaming is in progress. */
	bool streaming;

	struct v4l2_pix_format pix;
	struct v4l2_rect crop;
	struct v4l2_window win;
	struct v4l2_framebuffer fbuf;	/* graphics layer info */

	/* Lock to protect the shared data structures in ioctl */
	struct mutex lock;

	/* V4L2 control structure for different control id */
	struct v4l2_control control[MAX_CID];
	int flicker_filter;
	/* V4L2 control structure for different control id */

	int ps, vr_ps, line_length, first_int, field_id;
	enum v4l2_memory memory;
	struct videobuf_buffer *cur_frm, *next_frm;
	struct list_head dma_queue;
	u32 cropped_offset;
	s32 tv_field1_offset;

	/* Buffer queue variabled */
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
};

struct pxa168_overlay *v4l2_ovly[3];

static struct videobuf_queue_ops video_vbq_ops;

/* Local Helper functions */
static void pxa168_ovly_cleanup_device(struct pxa168_overlay *ovly);
static int debug = 1;

module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

/* Return the default overlay cropping rectangle in crop given the image
 * size in pix and the video display size in fbuf.  The default
 * cropping rectangle is the largest rectangle no larger than the capture size
 * that will fit on the display.  The default cropping rectangle is centered in
 * the image.  All dimensions and offsets are rounded down to even numbers.
 */
void pxa168_ovly_default_crop(struct v4l2_pix_format *pix,
			      struct v4l2_framebuffer *fbuf,
			      struct v4l2_rect *crop)
{
	crop->width = (pix->width < VID_MAX_WIDTH) ?
	    pix->width : VID_MAX_WIDTH;
	crop->height = (pix->height < VID_MAX_HEIGHT) ?
	    pix->height : VID_MAX_HEIGHT;
	crop->width &= ~1;
	crop->height &= ~1;
	crop->left = ((pix->width - crop->width) >> 1) & ~1;
	crop->top = ((pix->height - crop->height) >> 1) & ~1;
}

/* Given a new render window in new_win, adjust the window to the
 * nearest supported configuration.  The adjusted window parameters are
 * returned in new_win.
 * Returns zero if succesful, or -EINVAL if the requested window is
 * impossible and cannot reasonably be adjusted.
 */
int pxa168_ovly_try_window(struct v4l2_framebuffer *fbuf,
			   struct v4l2_window *new_win)
{
	struct v4l2_rect try_win;

	/* make a working copy of the new_win rectangle */
	try_win = new_win->w;

	/* adjust the preview window so it fits on the display by clipping any
	 * offscreen areas
	 */
	if (try_win.left < 0) {
		try_win.width += try_win.left;
		try_win.left = 0;
	}
	if (try_win.top < 0) {
		try_win.height += try_win.top;
		try_win.top = 0;
	}
	try_win.width = (try_win.width < fbuf->fmt.width) ?
	    try_win.width : fbuf->fmt.width;
	try_win.height = (try_win.height < fbuf->fmt.height) ?
	    try_win.height : fbuf->fmt.height;
	if (try_win.left + try_win.width > fbuf->fmt.width)
		try_win.width = fbuf->fmt.width - try_win.left;
	if (try_win.top + try_win.height > fbuf->fmt.height)
		try_win.height = fbuf->fmt.height - try_win.top;
	try_win.width &= ~1;
	try_win.height &= ~1;

	if (try_win.width <= 0 || try_win.height <= 0)
		return -EINVAL;

	/* We now have a valid preview window, so go with it */
	new_win->w = try_win;
	new_win->field = V4L2_FIELD_ANY;
	return 0;
}

/* Given a new render window in new_win, adjust the window to the
 * nearest supported configuration.  The image cropping window in crop
 * will also be adjusted if necessary.  Preference is given to keeping the
 * the window as close to the requested configuration as possible.  If
 * successful, new_win, ovly->win, and crop are updated.
 * Returns zero if succesful, or -EINVAL if the requested preview window is
 * impossible and cannot reasonably be adjusted.
 */
int pxa168_ovly_new_window(struct v4l2_rect *crop,
			   struct v4l2_window *win,
			   struct v4l2_framebuffer *fbuf,
			   struct v4l2_window *new_win)
{
	int err;

	err = pxa168_ovly_try_window(fbuf, new_win);
	if (err)
		return err;

	/* update our preview window */
	win->w = new_win->w;
	win->field = new_win->field;
	win->chromakey = new_win->chromakey;

	/* adjust the cropping window to allow for resizing limitations */
	if (crop->height >= (win->w.height << 2)) {
		/* The maximum vertical downsizing ratio is 4:1 */
		crop->height = win->w.height << 2;
	}
	if (crop->width >= (win->w.width << 2)) {
		/* The maximum horizontal downsizing ratio is 4:1 */
		crop->width = win->w.width << 2;
	}
	return 0;
}

/* Given a new cropping rectangle in new_crop, adjust the cropping rectangle to
 * the nearest supported configuration.  The image render window in win will
 * also be adjusted if necessary.  The preview window is adjusted such that the
 * horizontal and vertical rescaling ratios stay constant.  If the render
 * window would fall outside the display boundaries, the cropping rectangle
 * will also be adjusted to maintain the rescaling ratios.  If successful, crop
 * and win are updated.
 * Returns zero if succesful, or -EINVAL if the requested cropping rectangle is
 * impossible and cannot reasonably be adjusted.
 */
int pxa168_ovly_new_crop(struct v4l2_pix_format *pix,
			 struct v4l2_rect *crop, struct v4l2_window *win,
			 struct v4l2_framebuffer *fbuf,
			 const struct v4l2_rect *new_crop)
{
	struct v4l2_rect try_crop;
	unsigned long vresize, hresize;

	/* make a working copy of the new_crop rectangle */
	try_crop = *new_crop;

	/* adjust the cropping rectangle so it fits in the image */
	if (try_crop.left < 0) {
		try_crop.width += try_crop.left;
		try_crop.left = 0;
	}
	if (try_crop.top < 0) {
		try_crop.height += try_crop.top;
		try_crop.top = 0;
	}
	try_crop.width = (try_crop.width < pix->width) ?
	    try_crop.width : pix->width;
	try_crop.height = (try_crop.height < pix->height) ?
	    try_crop.height : pix->height;
	if (try_crop.left + try_crop.width > pix->width)
		try_crop.width = pix->width - try_crop.left;
	if (try_crop.top + try_crop.height > pix->height)
		try_crop.height = pix->height - try_crop.top;
	try_crop.width &= ~1;
	try_crop.height &= ~1;
	if (try_crop.width <= 0 || try_crop.height <= 0)
		return -EINVAL;

	/* vertical resizing */
	vresize = (1024 * crop->height) / win->w.height;
	if (vresize > 4096)
		vresize = 4096;
	else if (vresize == 0)
		vresize = 1;
	win->w.height = ((1024 * try_crop.height) / vresize) & ~1;
	if (win->w.height == 0)
		win->w.height = 2;
	if (win->w.height + win->w.top > fbuf->fmt.height) {
		/* We made the preview window extend below the bottom of the
		 * display, so clip it to the display boundary and resize the
		 * cropping height to maintain the vertical resizing ratio.
		 */
		win->w.height = (fbuf->fmt.height - win->w.top) & ~1;
		if (try_crop.height == 0)
			try_crop.height = 2;
	}
	/* horizontal resizing */
	hresize = (1024 * crop->width) / win->w.width;
	if (hresize > 4096)
		hresize = 4096;
	else if (hresize == 0)
		hresize = 1;
	win->w.width = ((1024 * try_crop.width) / hresize) & ~1;
	if (win->w.width == 0)
		win->w.width = 2;
	if (win->w.width + win->w.left > fbuf->fmt.width) {
		/* We made the preview window extend past the right side of the
		 * display, so clip it to the display boundary and resize the
		 * cropping width to maintain the horizontal resizing ratio.
		 */
		win->w.width = (fbuf->fmt.width - win->w.left) & ~1;
		if (try_crop.width == 0)
			try_crop.width = 2;
	}

	/* Check for resizing constraints */
	if (try_crop.height >= (win->w.height << 2)) {
		/* The maximum vertical downsizing ratio is 4:1 */
		try_crop.height = win->w.height << 2;
	}
	if (try_crop.width >= (win->w.width << 2)) {
		/* The maximum horizontal downsizing ratio is 4:1 */
		try_crop.width = win->w.width << 2;
	}

	/* update our cropping rectangle and we're done */
	*crop = try_crop;
	return 0;
}

/* Given a new format in pix and fbuf,  crop and win
 * structures are initialized to default values. crop
 * is initialized to the largest window size that will fit on the display.  The
 * crop window is centered in the image. win is initialized to
 * the same size as crop and is centered on the display.
 * All sizes and offsets are constrained to be even numbers.
 */
void pxa168_ovly_new_format(struct v4l2_pix_format *pix,
			    struct v4l2_framebuffer *fbuf,
			    struct v4l2_rect *crop, struct v4l2_window *win)
{
	/* crop defines the preview source window in the image capture
	 * buffer
	 */
	pxa168_ovly_default_crop(pix, fbuf, crop);


	win->w.width = (crop->width < fbuf->fmt.width) ?
	    crop->width : fbuf->fmt.width;
	win->w.height = (crop->height < fbuf->fmt.height) ?
	    crop->height : fbuf->fmt.height;
	win->w.width &= ~1;
	win->w.height &= ~1;
	win->w.left = ((fbuf->fmt.width - win->w.width) >> 1) & ~1;
	win->w.top = ((fbuf->fmt.height - win->w.height) >> 1) & ~1;
}

/* Try format */
static int pxa168_ovly_try_format(struct v4l2_pix_format *pix)
{
	int ifmt, bpp = 0;

	pix->height = clamp(pix->height, (u32) VID_MIN_HEIGHT,
			    (u32) VID_MAX_HEIGHT);
	pix->width =
	    clamp(pix->width, (u32) VID_MIN_WIDTH, (u32) VID_MAX_WIDTH);

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == pxa168_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = pxa168_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_ANY;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 12;
		break;
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 16;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = 16;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = 24;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = 32;
		break;
	}
	pix->bytesperline = (pix->width * bpp) >> 3;
	pix->sizeimage = pix->bytesperline * pix->height;
	return 0;
}

/* Setup the overlay info */
int pxa168vid_setup_overlay(struct pxa168_overlay *ovly, int posx, int posy,
			    int outw, int outh, u32 addr)
{
	ovly->paddr = addr;

	if ((ovly->pos_x != posx) || (ovly->pos_y != posy)) {
		ovly->pos_x = posx;
		ovly->pos_y = posy;
		ovly->update = 1;
	}

	if ((ovly->out_width != outw) || (ovly->out_height != outh)) {
		ovly->out_width = outw;
		ovly->out_height = outh;
		ovly->update = 1;
	}

	ovly->global_alpha = ovly->win.global_alpha;

	v4l2_dbg(1, debug, ovly->vdev,
		 "%s:ovly.enable=%d ovly.addr=%x pix.width=%d\n pix.height=%d "
		 "crop.width=%d crop.height=%d\n"
		 "ovly.posx=%d ovly.posy=%d ovly.out_width = %d "
		 "ovly.out_height=%d\n ",
		 __func__, ovly->enabled,
		 ovly->paddr, ovly->pix.width, ovly->pix.height,
		 ovly->crop.height, ovly->crop.width,
		 ovly->pos_x, ovly->pos_y, ovly->out_width, ovly->out_height);
	return 0;
}

/* Initialize the overlay structure */
int pxa168vid_init(struct pxa168_overlay *ovly, u32 addr)
{
	int ret = 0;
	int posx, posy;
	int outw, outh;
	u32 reg;
	struct v4l2_window *win;
	struct v4l2_pix_format *pix = &ovly->pix;
	struct pxa168fb_mach_info *mi;
	mi = ovly->dev->platform_data;

	reg = readl(ovly->reg_base + LCD_SPU_DMA_CTRL0);
	ovly->dma_ctl0 = reg;
	reg &= 0xef0fffe1;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YVU420:
		reg |= CFG_DMA_SWAPUV(1);
	case V4L2_PIX_FMT_YUV420:
	default:
		reg |= CFG_DMAFORMAT(GMODE_YUV420PLANAR) |
			CFG_YUV2RGB_DMA(1) |
			(mi->panel_rbswap << 4);
		ovly->ypitch = pix->width;
		ovly->uvpitch = pix->width >> 1;
		ovly->y_size = pix->width * pix->height;
		ovly->uv_size = ovly->y_size >> 2;
		break;
	case V4L2_PIX_FMT_YUV422P:
		reg |= CFG_DMAFORMAT(GMODE_YUV422PLANAR) |
			CFG_YUV2RGB_DMA(1) |
			(mi->panel_rbswap << 4);
		ovly->ypitch = pix->width;
		ovly->uvpitch = pix->width >> 1;
		ovly->y_size = pix->width * pix->height;
		ovly->uv_size = ovly->y_size >> 1;
		break;
	case V4L2_PIX_FMT_UYVY:
		reg |= CFG_DMA_SWAPYU(1);
	case V4L2_PIX_FMT_YUYV:
		reg |= CFG_DMAFORMAT(GMODE_YUV422PACKED) |
			CFG_YUV2RGB_DMA(1) |
			(mi->panel_rbswap << 4);
		ovly->ypitch = pix->width << 1;
		ovly->uvpitch = 0;
		break;
	case V4L2_PIX_FMT_RGB555X:
		reg |= CFG_DMAFORMAT(VMODE_RGB1555);
	case V4L2_PIX_FMT_RGB565:
		reg |= (mi->panel_rbswap) ? (0) : (CFG_DMA_SWAPRB(1));
		ovly->ypitch = pix->width << 1;
		ovly->uvpitch = 0;
		break;
	case V4L2_PIX_FMT_BGR24:
		reg |= CFG_DMAFORMAT(VMODE_RGB888PACKED);
		reg |= (mi->panel_rbswap) ? CFG_DMA_SWAPRB(1) : (0);
		ovly->ypitch = pix->width * 3;
		ovly->uvpitch = 0;
		break;
	case V4L2_PIX_FMT_BGR32:
		reg |= CFG_DMA_SWAPRB(1);
		reg |= (mi->panel_rbswap) ? CFG_DMA_SWAPRB(1) : (0);
	case V4L2_PIX_FMT_RGB32:
		reg |= CFG_DMAFORMAT(VMODE_RGBA888);
		reg |= (mi->panel_rbswap) ? (0) : (CFG_DMA_SWAPRB(1));
		ovly->ypitch = pix->width << 2;
		ovly->uvpitch = 0;
		break;
	}
	if (reg != ovly->dma_ctl0) {
		ovly->dma_ctl0 = reg;
		ovly->update = 1;
	}

	win = &ovly->win;

	outw = win->w.width;
	outh = win->w.height;
	posx = win->w.left;
	posy = win->w.top;

	ret = pxa168vid_setup_overlay(ovly, posx, posy, outw, outh, addr);
	if (ret)
		goto err;

	return 0;
err:
	printk(KERN_WARNING VOUT_NAME "apply_changes failed\n");
	return ret;
}

int pxa168vid_apply_changes(struct pxa168_overlay *ovly)
{
	u32 bpp = ovly->pix.bytesperline / ovly->pix.width;
	u32 offset = ovly->crop.top * ovly->ypitch + ovly->crop.left * bpp;

	writel(ovly->paddr + offset,
		ovly->reg_base + LCD_SPU_DMA_START_ADDR_Y0);
	if (ovly->uvpitch) {
		offset = ovly->crop.top * ovly->uvpitch +
			 ovly->crop.left * bpp;

		writel(ovly->paddr + ovly->y_size + offset,
		       ovly->reg_base + LCD_SPU_DMA_START_ADDR_U0);
		writel(ovly->paddr + ovly->uv_size + ovly->y_size  + offset,
		       ovly->reg_base + LCD_SPU_DMA_START_ADDR_V0);
	}

	if (ovly->update) {
		writel((ovly->crop.height << 16) | ovly->crop.width,
		       ovly->reg_base + LCD_SPU_DMA_HPXL_VLN);
		writel(ovly->ypitch, ovly->reg_base + LCD_SPU_DMA_PITCH_YC);
		writel((ovly->uvpitch) << 16 | (ovly->uvpitch),
		       ovly->reg_base + LCD_SPU_DMA_PITCH_UV);

		writel(CFG_DMA_OVSA_VLN(ovly->pos_y) | ovly->pos_x,
		       ovly->reg_base + LCD_SPUT_DMA_OVSA_HPXL_VLN);

		writel((ovly->out_height << 16) | ovly->out_width,
			ovly->reg_base + LCD_SPU_DZM_HPXL_VLN);

		writel(ovly->dma_ctl0, ovly->reg_base + LCD_SPU_DMA_CTRL0);
		ovly->update = 0;
	}

	return 0;

}

int pxa168_ovly_set_colorkeyalpha(struct pxa168_overlay *ovly)
{
	unsigned int rb = 0;
	unsigned int dma0 = 0;
	unsigned int temp;
	unsigned int x;
	struct pxa168fb_mach_info *mi;
	struct _sColorKeyNAlpha *color_a = &ovly->ckey_alpha;

	mi = ovly->dev->platform_data;

	if (ovly->id <= 0)
		x = readl(ovly->reg_base +
			  LCD_SPU_DMA_CTRL1) & ~(CFG_COLOR_KEY_MASK |
						 CFG_ALPHA_MODE_MASK |
						 CFG_ALPHA_MASK);
	else
		x = readl(ovly->reg_base +
			  LCD_TV_CTRL1) & ~(CFG_COLOR_KEY_MASK |
					    CFG_ALPHA_MODE_MASK |
					    CFG_ALPHA_MASK);
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
		break;
	case FB_ENABLE_B_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x7);
		rb = 1;
		break;
	default:
		printk(KERN_INFO "unknown mode");
		return -1;
	}

	if (rb) {
		if (ovly->id <= 0)
			dma0 = readl(ovly->reg_base + LCD_SPU_DMA_CTRL0);
		else
			dma0 = readl(ovly->reg_base + LCD_TV_CTRL0);
	}
	/* switch to alpha path selection and decide whether to do RB swap */
	switch (color_a->alphapath) {
	case FB_VID_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x0);
		if (rb)
			rb = ((dma0 & CFG_DMA_SWAPRB_MASK) >> 4) ^
					(mi->panel_rbswap);
		break;
	case FB_GRA_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x1);
		if (rb)
			rb = ((dma0 & CFG_GRA_SWAPRB_MASK) >> 12) ^
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
		temp = color_a->Y_ColorAlpha;
		color_a->Y_ColorAlpha = color_a->V_ColorAlpha;
		color_a->V_ColorAlpha = temp;

		/* only alpha_Y take effect, switch back from V */
		if (color_a->mode == FB_ENABLE_RGB_COLORKEY_MODE) {
			color_a->Y_ColorAlpha &= 0xffffff00;
			temp = color_a->V_ColorAlpha & 0xff;
			color_a->Y_ColorAlpha |= temp;
		}
	}

	/* configure alpha */
	x |= CFG_ALPHA((color_a->config & 0xff));
	if (ovly->id <= 0) {
		writel(x, ovly->reg_base + LCD_SPU_DMA_CTRL1);
		writel(0xc0000, ovly->reg_base + LCD_TV_CTRL1);
		writel(color_a->Y_ColorAlpha,
		       ovly->reg_base + LCD_SPU_COLORKEY_Y);
		writel(color_a->U_ColorAlpha,
		       ovly->reg_base + LCD_SPU_COLORKEY_U);
		writel(color_a->V_ColorAlpha,
		       ovly->reg_base + LCD_SPU_COLORKEY_V);
	} else {
		writel(x, ovly->reg_base + LCD_TV_CTRL1);
		writel(color_a->Y_ColorAlpha,
		       ovly->reg_base + LCD_TV_COLORKEY_Y);
		writel(color_a->U_ColorAlpha,
		       ovly->reg_base + LCD_TV_COLORKEY_U);
		writel(color_a->V_ColorAlpha,
		       ovly->reg_base + LCD_TV_COLORKEY_V);
	}
	return 0;
}

/* Video buffer call backs */

/* Buffer setup function is called by videobuf layer when REQBUF ioctl is
 * called. This is used to alloc buffers and return size and count of
 * buffers allocated in V4L2_MEMORY_MMAP case. After the call to this
 * buffer, videobuf layer will setup buffer queue depending on the size
 * and count of buffers.
 */
static int pxa168_ovly_buffer_setup(struct videobuf_queue *q,
				    unsigned int *count, unsigned int *size)
{
	struct pxa168_overlay *ovly = q->priv_data;

	if (!ovly)
		return -EINVAL;

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != q->type)
		return -EINVAL;

	/* *size = PAGE_ALIGN(ovly->pix.width * ovly->pix.height * 2); */
	*size = PAGE_ALIGN(ovly->pix.sizeimage);


	v4l2_dbg(1, debug, ovly->vdev, "buffer_setup, count=%d, size=%d\n",
		 *count, *size);

	return 0;
}

/* This function will be called when VIDIOC_QBUF ioctl is called.
 * It prepare buffers before give out for the display. This function
 * user space virtual address into physical address if userptr memory
 * exchange mechanism is used.
 */
static int pxa168_ovly_buffer_prepare(struct videobuf_queue *q,
				      struct videobuf_buffer *vb,
				      enum v4l2_field field)
{
	struct pxa168_overlay *ovly = q->priv_data;
	int ret = 0;

	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = ovly->pix.width;
		vb->height = ovly->pix.height;
		vb->size = ovly->pix.sizeimage;
		vb->field = field;
	}

	ret = videobuf_iolock(q, vb, NULL);
	if (ret < 0)
		goto prep_buf;

	vb->state = VIDEOBUF_PREPARED;

prep_buf:
	return ret;
}

/* Buffer queue funtion will be called from the videobuf layer when _QBUF
 * ioctl is called. It is used to enqueue buffer, which is ready to be
 * displayed. */
static void pxa168_ovly_buffer_queue(struct videobuf_queue *q,
				     struct videobuf_buffer *vb)
{
	struct pxa168_overlay *ovly = q->priv_data;

	/* Driver is also maintainig a queue. So enqueue buffer in the driver
	 * queue */
	list_add_tail(&vb->queue, &ovly->dma_queue);

	vb->state = VIDEOBUF_QUEUED;
}

/* Buffer release function is called from videobuf layer to release buffer
 * which are already allocated */
static void pxa168_ovly_buffer_release(struct videobuf_queue *q,
				       struct videobuf_buffer *vb)
{
	vb->state = VIDEOBUF_NEEDS_INIT;

	videobuf_dma_contig_free(q, vb);

	return;
}

/*
 *  File operations
 */
static int pxa168_ovly_release(struct file *file)
{

	struct pxa168_overlay *ovly = file->private_data;
	struct videobuf_queue *q;
	unsigned int ret;

	v4l2_dbg(1, debug, ovly->vdev, "Entering %s\n", __func__);

	if (!ovly)
		return 0;
	q = &ovly->vbq;

	/* Turn off the pipeline */
	ret = pxa168vid_apply_changes(ovly);
	if (ret)
		printk(KERN_ERR VOUT_NAME "Unable to apply changes\n");

	videobuf_mmap_free(q);

	/* Even if apply changes fails we should continue
	   freeing allocated memeory */
	if (ovly->streaming) {
		/* u32 mask = 0; */
		/* stop dma */
		/* pxa168_dispc_unregister_isr(pxa168_ovly_isr, ovly, mask); */
		ovly->streaming = 0;

		videobuf_streamoff(q);
		videobuf_queue_cancel(q);

	}

	ovly->opened -= 1;
	file->private_data = NULL;

	videobuf_mmap_free(q);
	v4l2_dbg(1, debug, ovly->vdev, "Exiting %s\n", __func__);
	return ret;
}

static int pxa168_ovly_open(struct file *file)
{
	struct pxa168_overlay *ovly = NULL;
	struct videobuf_queue *q;

	ovly = video_drvdata(file);
	v4l2_dbg(1, debug, ovly->vdev, "Entering %s\n", __func__);

	if (ovly == NULL)
		return -ENODEV;

	/* for now, we only support single open */
	if (ovly->opened)
		return -EBUSY;

	ovly->opened += 1;

	file->private_data = ovly;
	ovly->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	q = &ovly->vbq;
	video_vbq_ops.buf_setup = pxa168_ovly_buffer_setup;
	video_vbq_ops.buf_prepare = pxa168_ovly_buffer_prepare;
	video_vbq_ops.buf_release = pxa168_ovly_buffer_release;
	video_vbq_ops.buf_queue = pxa168_ovly_buffer_queue;
	spin_lock_init(&ovly->vbq_lock);

	videobuf_queue_dma_contig_init(q, &video_vbq_ops, NULL, &ovly->vbq_lock,
				       ovly->type, V4L2_FIELD_NONE, sizeof
				       (struct videobuf_buffer), ovly);
	v4l2_dbg(1, debug, ovly->vdev, "Exiting %s\n", __func__);
	return 0;
}

static unsigned int pxa168_poll(struct file *file,
				struct poll_table_struct *wait)
{
	struct pxa168_overlay *ovly = file->private_data;
	struct videobuf_queue *q = &ovly->vbq;

	v4l2_dbg(1, debug, ovly->vdev, "%s\n", __func__);

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != ovly->type)
		return POLLERR;

	return videobuf_poll_stream(file, q, wait);
}

static int video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pxa168_overlay *ovly = file->private_data;

	return videobuf_mmap_mapper(&ovly->vbq, vma);
}

/* V4L2 ioctls */
static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct pxa168_overlay *ovly = fh;

	strlcpy(cap->driver, VOUT_NAME, sizeof(cap->driver));
	strlcpy(cap->card, ovly->vdev->name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *fh,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = pxa168_formats[index].flags;
	strlcpy(fmt->description, pxa168_formats[index].description,
		sizeof(fmt->description));
	fmt->pixelformat = pxa168_formats[index].pixelformat;
	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct pxa168_overlay *ovly = fh;

	f->fmt.pix = ovly->pix;
	return 0;

}

static int vidioc_try_fmt_vid_out(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct pxa168_overlay *ovly = fh;

	if (ovly->streaming)
		return -EBUSY;

	pxa168_ovly_try_format(&f->fmt.pix);
	return 0;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct pxa168_overlay *ovly = fh;
	int ret;

	if (ovly->streaming)
		return -EBUSY;

	mutex_lock(&ovly->lock);

	/* change to samller size is OK */

	pxa168_ovly_try_format(&f->fmt.pix);

	/* save the new output format */
	ovly->pix = f->fmt.pix;

	/* set default crop and win */
	pxa168_ovly_new_format(&ovly->pix, &ovly->fbuf, &ovly->crop,
			       &ovly->win);

	/* Save the changes in the overlay strcuture and set controller */
	ret = pxa168vid_init(ovly, 0);
	if (ret) {
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");
		mutex_unlock(&ovly->lock);
		return -EINVAL;
	}
	mutex_unlock(&ovly->lock);
	return 0;
}

static int vidioc_try_fmt_vid_overlay(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	int err = -EINVAL;
	struct pxa168_overlay *ovly = fh;
	struct v4l2_window *win = &f->fmt.win;

	err = pxa168_ovly_try_window(&ovly->fbuf, win);

	if (err)
		return err;

	win->global_alpha = 255;

	return 0;
}

static int vidioc_s_fmt_vid_overlay(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct pxa168_overlay *ovly = fh;
	int err = -EINVAL;
	struct v4l2_window *win = &f->fmt.win;

	mutex_lock(&ovly->lock);

	err = pxa168_ovly_new_window(&ovly->crop, &ovly->win, &ovly->fbuf, win);
	if (err) {
		mutex_unlock(&ovly->lock);
		return err;
	}

	if (ovly->trans_enabled) {
		ovly->win.chromakey = f->fmt.win.chromakey;
		ovly->trans_key = ovly->win.chromakey;
		ovly->ckey_alpha.Y_ColorAlpha = ovly->win.chromakey;
		ovly->ckey_alpha.U_ColorAlpha = ovly->win.chromakey;
		ovly->ckey_alpha.V_ColorAlpha = ovly->win.chromakey;
	} else {
		ovly->win.chromakey = 0;
		ovly->trans_key = ovly->win.chromakey;
		ovly->ckey_alpha.Y_ColorAlpha = 0;
		ovly->ckey_alpha.U_ColorAlpha = 0;
		ovly->ckey_alpha.V_ColorAlpha = 0;
	}
	if (ovly->alpha_enabled) {
		ovly->win.global_alpha = f->fmt.win.global_alpha;
		/*Fix me */
		ovly->ckey_alpha.mode = FB_ENABLE_RGB_COLORKEY_MODE;
		ovly->ckey_alpha.alphapath = FB_GRA_PATH_ALPHA;
		ovly->ckey_alpha.config = ovly->win.global_alpha;
	} else {
		ovly->win.global_alpha = 255;
		/*Fix me */
		ovly->ckey_alpha.mode = FB_DISABLE_COLORKEY_MODE;
		ovly->ckey_alpha.alphapath = FB_CONFIG_ALPHA;
		ovly->ckey_alpha.config = ovly->win.global_alpha;

	}

	pxa168_ovly_set_colorkeyalpha(ovly);

	v4l2_dbg(1, debug, ovly->vdev, "chromakey %d, global_alpha %d\n",
		 ovly->win.chromakey, ovly->win.global_alpha);

	mutex_unlock(&ovly->lock);
	return 0;
}

static int vidioc_enum_fmt_vid_overlay(struct file *file, void *fh,
				       struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = pxa168_formats[index].flags;
	strlcpy(fmt->description, pxa168_formats[index].description,
		sizeof(fmt->description));
	fmt->pixelformat = pxa168_formats[index].pixelformat;
	return 0;
}

static int vidioc_g_fmt_vid_overlay(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct pxa168_overlay *ovly = fh;
	struct v4l2_window *win = &f->fmt.win;
	u32 key_value = 0;

	win->w = ovly->win.w;
	win->field = ovly->win.field;
	win->global_alpha = ovly->win.global_alpha;

	key_value = ovly->trans_key;
	/* win->chromakey = key_value; */
	win->chromakey = ovly->win.chromakey;
	return 0;
}

static int vidioc_cropcap(struct file *file, void *fh,
			  struct v4l2_cropcap *cropcap)
{
	struct pxa168_overlay *ovly = fh;
	enum v4l2_buf_type type = cropcap->type;
	struct v4l2_pix_format *pix = &ovly->pix;

	cropcap->type = type;
	if (type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* Width and height are always even */
	cropcap->bounds.width = pix->width & ~1;
	cropcap->bounds.height = pix->height & ~1;

	pxa168_ovly_default_crop(&ovly->pix, &ovly->fbuf, &cropcap->defrect);
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct pxa168_overlay *ovly = fh;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	crop->c = ovly->crop;
	return 0;
}

static int vidioc_s_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct pxa168_overlay *ovly = fh;
	int err = -EINVAL;

	if (ovly->streaming)
		return -EBUSY;

	mutex_lock(&ovly->lock);

	if (crop->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		err = pxa168_ovly_new_crop(&ovly->pix, &ovly->crop, &ovly->win,
					   &ovly->fbuf, &crop->c);
		if (!err)
			ovly->update = 1;
		mutex_unlock(&ovly->lock);
		return err;
	} else {
		mutex_unlock(&ovly->lock);
		return -EINVAL;
	}
}

static int vidioc_queryctrl(struct file *file, void *fh,
			    struct v4l2_queryctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
	case V4L2_CID_HUE:
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_GAMMA:

		v4l2_ctrl_query_fill(ctrl, 0, 0xFFFFFF, 1, 0);
		break;
	default:
		ctrl->name[0] = '\0';
		return -EINVAL;
	}
	return 0;
}

static int vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	/* struct pxa168_overlay *ovly = fh; */

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		{
			return 0;
		}

	default:
		return -EINVAL;
	}
}

static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct pxa168_overlay *ovly = fh;

	switch (a->id) {
	case V4L2_CID_VFLIP:
		{
			unsigned int color = a->value;

			mutex_lock(&ovly->lock);

			ovly->control[1].value = color;
			mutex_unlock(&ovly->lock);
			return 0;
		}

	default:
		return -EINVAL;
	}

}

static int vidioc_reqbufs(struct file *file, void *fh,
			  struct v4l2_requestbuffers *req)
{
	struct pxa168_overlay *ovly = fh;
	struct videobuf_queue *q = &ovly->vbq;

	/* if memory is not mmp or userptr
	   return error */
	if ((V4L2_MEMORY_MMAP != req->memory) &&
	    (V4L2_MEMORY_USERPTR != req->memory))
		return -EINVAL;

	if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) || (req->count < 0))
		return -EINVAL;

	INIT_LIST_HEAD(&ovly->dma_queue);

	return videobuf_reqbufs(q, req);

}

static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct pxa168_overlay *ovly = fh;

	return videobuf_querybuf(&ovly->vbq, b);
}

static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *buffer)
{
	struct pxa168_overlay *ovly = fh;
	struct videobuf_queue *q = &ovly->vbq;

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != buffer->type)
		return -EINVAL;

	if (V4L2_MEMORY_USERPTR == buffer->memory) {
		if ((buffer->length < ovly->pix.sizeimage) ||
		    (0 == buffer->m.userptr)) {
			return -EINVAL;
		}
		/* v4l2_dbg(1, debug, ovly->vdev, "qbuf id %d addr %p\n",
		 * buffer->index, buffer->m.userptr); */
	}

	v4l2_dbg(1, debug, ovly->vdev, "qbuf id %d\n", buffer->index);

	return videobuf_qbuf(q, buffer);
}

static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct pxa168_overlay *ovly = fh;
	struct videobuf_queue *q = &ovly->vbq;
	int ret = 0;

	if (!ovly->streaming)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK)
		/* Call videobuf_dqbuf for non blocking mode */
		ret = videobuf_dqbuf(q, (struct v4l2_buffer *) b, 1);
	else
		/* Call videobuf_dqbuf for  blocking mode */
		ret = videobuf_dqbuf(q, (struct v4l2_buffer *) b, 0);

	v4l2_dbg(1, debug, ovly->vdev, "dqbuf ret %d id %d\n", ret, b->index);

	return ret;
}

static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct pxa168_overlay *ovly = fh;
	struct videobuf_queue *q = &ovly->vbq;
	u32 addr = 0;
	int ret = 0;
	u32 reg;
	/* u32 mask = 0; */

	v4l2_dbg(1, debug, ovly->vdev, "enter stream on!\n");
	mutex_lock(&ovly->lock);

	if (ovly->streaming) {
		mutex_unlock(&ovly->lock);
		return -EBUSY;
	}

	ret = videobuf_streamon(q);
	if (ret < 0) {
		mutex_unlock(&ovly->lock);
		return ret;
	}

	if (list_empty(&ovly->dma_queue)) {
		mutex_unlock(&ovly->lock);
		return -EIO;
	}
	/* Get the next frame from the buffer queue */
	ovly->next_frm = ovly->cur_frm = list_entry(ovly->dma_queue.next,
						    struct videobuf_buffer,
						    queue);
	/* Remove buffer from the buffer queue */
	list_del(&ovly->cur_frm->queue);
	/* Mark state of the current frame to active */
	ovly->cur_frm->state = VIDEOBUF_ACTIVE;
	/* Initialize field_id and started member */
	ovly->field_id = 0;
	/* set flag here. Next QBUF will start DMA */
	ovly->streaming = 1;

	ovly->first_int = 1;

	addr = (unsigned long) videobuf_to_dma_contig(ovly->next_frm)
	    + ovly->cropped_offset;

	/* pxa168_dispc_register_isr(pxa168_ovly_isr, ovly, mask); */

	ovly->enabled = 1;
	ovly->paddr = addr;

	/* First save the configuration in ovelray structure */
	ret = pxa168vid_init(ovly, addr);
	if (ret)
		printk(KERN_ERR VOUT_NAME "failed to set overlay ovly\n");
	/* Enable the pipeline and set the Go bit */
	ret = pxa168vid_apply_changes(ovly);
	if (ret)
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");

	writel(readl(ovly->reg_base + SPU_IRQ_ENA) |
	       DUMB_FRAMEDONE_ENA_MASK, ovly->reg_base + SPU_IRQ_ENA);
	reg = readl(ovly->reg_base + LCD_SPU_DMA_CTRL0) | CFG_DMA_ENA_MASK;
	writel(reg, ovly->reg_base + LCD_SPU_DMA_CTRL0);

	mutex_unlock(&ovly->lock);

	v4l2_dbg(1, debug, ovly->vdev, "stream on!\n");
	return ret;
}

static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct pxa168_overlay *ovly = fh;
	int ret = 0;
	u32 temp;
	/* u32 mask = 0; */

	if (!ovly->streaming)
		return -EINVAL;

	ovly->streaming = 0;
	/* stop dma */

	/* pxa168_dispc_unregister_isr(pxa168_ovly_isr, ovly, mask); */

	/* Turn off the pipeline */
	ret = pxa168vid_apply_changes(ovly);
	if (ret) {
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");
		return ret;
	}
	videobuf_streamoff(&ovly->vbq);
	videobuf_queue_cancel(&ovly->vbq);

	temp = readl(ovly->reg_base + LCD_SPU_DMA_CTRL0);
	temp &= ~(CFG_DMA_ENA_MASK);
	writel(temp, ovly->reg_base + LCD_SPU_DMA_CTRL0);
	v4l2_dbg(1, debug, ovly->vdev, "stream off!\n");
	return 0;
}

static int vidioc_s_fbuf(struct file *file, void *fh,
			 struct v4l2_framebuffer *a)
{
	struct pxa168_overlay *ovly = fh;
	int key_type = COLOR_KEY_GFX_DST;
	int enable = 0;

	/* Doesn't support the Destination color key
	   and alpha blending together ?????????????????????????? */
	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY) &&
	    (a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA))
		return -EINVAL;

	/*if ((a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY)) {
	   ovly->fbuf.flags |= V4L2_FBUF_FLAG_SRC_CHROMAKEY;
	   key_type =  COLOR_KEY_VID_SRC;
	   } else
	   ovly->fbuf.flags &= ~V4L2_FBUF_FLAG_SRC_CHROMAKEY;
	 */

	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY)) {
		ovly->fbuf.flags |= V4L2_FBUF_FLAG_CHROMAKEY;
		key_type = COLOR_KEY_GFX_DST;
	} else
		ovly->fbuf.flags &= ~V4L2_FBUF_FLAG_CHROMAKEY;

	if (a->flags & V4L2_FBUF_FLAG_CHROMAKEY)
		enable = 1;
	else
		enable = 0;
	ovly->trans_enabled = enable;
	ovly->trans_key_type = key_type;

	if (a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA) {
		ovly->fbuf.flags |= V4L2_FBUF_FLAG_LOCAL_ALPHA;
		enable = 1;
	} else {
		ovly->fbuf.flags &= ~V4L2_FBUF_FLAG_LOCAL_ALPHA;
		enable = 0;
	}

	if (a->flags & V4L2_FBUF_FLAG_GLOBAL_ALPHA) {
		ovly->fbuf.flags |= V4L2_FBUF_FLAG_GLOBAL_ALPHA;
		enable = 1;
	} else {
		ovly->fbuf.flags &= ~V4L2_FBUF_FLAG_GLOBAL_ALPHA;
		enable = 0;
	}

	/* set par */
	ovly->alpha_enabled = enable;

	return 0;
}

static int vidioc_g_fbuf(struct file *file, void *fh,
			 struct v4l2_framebuffer *a)
{
	struct pxa168_overlay *ovly = fh;

	a->flags = 0x0;

	a->capability = V4L2_FBUF_CAP_GLOBAL_ALPHA |
	    V4L2_FBUF_CAP_LOCAL_ALPHA | V4L2_FBUF_CAP_CHROMAKEY;

	/*if (ovly->trans_key_type == COLOR_KEY_VID_SRC)
	   a->flags |= V4L2_FBUF_FLAG_SRC_CHROMAKEY;
	   if (ovly->trans_key_type == COLOR_KEY_GFX_DST)
	   a->flags = V4L2_FBUF_FLAG_CHROMAKEY;

	   if (ovly->alpha_enabled) */
	a->flags = ovly->fbuf.flags;

	return 0;
}

static const struct v4l2_ioctl_ops vout_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out = vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out = vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out = vidioc_s_fmt_vid_out,
	.vidioc_queryctrl = vidioc_queryctrl,
	.vidioc_g_ctrl = vidioc_g_ctrl,
	.vidioc_s_fbuf = vidioc_s_fbuf,
	.vidioc_g_fbuf = vidioc_g_fbuf,
	.vidioc_s_ctrl = vidioc_s_ctrl,
	.vidioc_try_fmt_vid_overlay = vidioc_try_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay = vidioc_s_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_overlay = vidioc_enum_fmt_vid_overlay,
	.vidioc_g_fmt_vid_overlay = vidioc_g_fmt_vid_overlay,
	.vidioc_cropcap = vidioc_cropcap,
	.vidioc_g_crop = vidioc_g_crop,
	.vidioc_s_crop = vidioc_s_crop,
	.vidioc_reqbufs = vidioc_reqbufs,
	.vidioc_querybuf = vidioc_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vidioc_dqbuf,
	.vidioc_streamon = vidioc_streamon,
	.vidioc_streamoff = vidioc_streamoff,
};

static const struct v4l2_file_operations pxa168_ovly_fops = {
	.owner = THIS_MODULE,
	.ioctl = video_ioctl2,
	.poll = pxa168_poll,
	.mmap = video_mmap,
	.open = pxa168_ovly_open,
	.release = pxa168_ovly_release,
};

irqreturn_t pxa168_ovly_isr(int id)
{
	struct timeval timevalue;
	struct pxa168_overlay *ovly = v4l2_ovly[id];
	u32 ret = 0;

	if (!ovly)
		return IRQ_NONE;

	if (!ovly->streaming)
		return IRQ_NONE;

	spin_lock(&ovly->vbq_lock);
	do_gettimeofday(&timevalue);

	if (!ovly->first_int && (ovly->cur_frm != ovly->next_frm)) {
		ovly->cur_frm->ts = timevalue;
		ovly->cur_frm->state = VIDEOBUF_DONE;
		wake_up_interruptible(&ovly->cur_frm->done);
		ovly->cur_frm = ovly->next_frm;
	}
	ovly->first_int = 0;
	if (list_empty(&ovly->dma_queue))
		goto irq;

	ovly->next_frm = list_entry(ovly->dma_queue.next,
				    struct videobuf_buffer, queue);
	list_del(&ovly->next_frm->queue);

	ovly->next_frm->state = VIDEOBUF_ACTIVE;

	ovly->paddr = videobuf_to_dma_contig(ovly->next_frm);

	/* First save the configuration in ovelray structure */
	ret = pxa168vid_init(ovly, ovly->paddr);

	if (ret)
		printk(KERN_ERR VOUT_NAME "failed to set overlay ovly\n");

	ret = pxa168vid_apply_changes(ovly);
	if (ret)
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");

irq:
	spin_unlock(&ovly->vbq_lock);

	v4l2_dbg(1, debug, ovly->vdev, "buf %d is active!\n",
		 ovly->next_frm->i);

	return IRQ_HANDLED;
}

/* Init functions used during driver intitalization */
/* Initial setup of video_data */
static int __init pxa168_ovly_setup_video_data(struct pxa168_overlay *ovly)
{
	struct v4l2_pix_format *pix;
	struct video_device *vdev;
	struct v4l2_control *control;
	unsigned int x;

	/* set the default pix */
	pix = &ovly->pix;

	/* Set the default picture of QVGA  */
	pix->width = VID_QVGA_WIDTH;
	pix->height = VID_QVGA_HEIGHT;

	/* Default pixel format is RGB 5-6-5 */
	pix->pixelformat = V4L2_PIX_FMT_YUV420;
	pix->field = V4L2_FIELD_ANY;
	pix->bytesperline = (pix->width * 3) >> 1;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_JPEG;

	/* We rely on base layer driver zoomed size to get the LCD size */
	x = readl(ovly->reg_base + LCD_SPU_GZM_HPXL_VLN);
	ovly->fbuf.fmt.height = x >> 16;
	ovly->fbuf.fmt.width = x & 0xffff;

	/* Set the data structures for the overlay parameters */
	ovly->win.global_alpha = 255;
	ovly->fbuf.flags = 0;
	ovly->fbuf.capability = V4L2_FBUF_CAP_GLOBAL_ALPHA;
	ovly->win.chromakey = 0;
	INIT_LIST_HEAD(&ovly->dma_queue);

	pxa168_ovly_new_format(pix, &ovly->fbuf, &ovly->crop, &ovly->win);

	/*Initialize the control variables for
	   and background color. */
	control = ovly->control;
	control[0].value = 0;
	ovly->control[2].value = 0;

	control[1].id = V4L2_CID_VFLIP;
	control[1].value = 0;

	/* initialize the video_device struct */
	vdev = ovly->vdev = video_device_alloc();

	if (!vdev) {
		printk(KERN_ERR VOUT_NAME ": could not allocate"
		       " video device struct\n");
		return -ENOMEM;
	}
	vdev->release = video_device_release;
	vdev->ioctl_ops = &vout_ioctl_ops;

	strlcpy(vdev->name, VOUT_NAME, sizeof(vdev->name));
	vdev->vfl_type = VFL_TYPE_GRABBER;

	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vdev->fops = &pxa168_ovly_fops;
	mutex_init(&ovly->lock);

	vdev->minor = -1;
	return 0;
}

/* Create video out devices and alloc buffer when boot up if needed */
static int __init pxa168_ovly_create_video_devices(struct pxa168_overlay *ovly)
{
	int ret = 0;
	struct video_device *vdev;

	/* Setup the default configuration for the video devices
	 */
	if (pxa168_ovly_setup_video_data(ovly) != 0) {
		ret = -ENOMEM;
		goto error1;
	}

	vdev = ovly->vdev;
	video_set_drvdata(vdev, ovly);

	/* Register the Video device with V4L2
	 */
	if (video_register_device(vdev, VFL_TYPE_GRABBER, 1) < 0) {
		printk(KERN_ERR VOUT_NAME ": could not register "
		       "Video for Linux device\n");
		vdev->minor = -1;
		ret = -ENODEV;
		goto error2;
	}

	/* Configure the overlay structure */
	ret = pxa168vid_init(ovly, 0);
	if (ret)
		printk(KERN_ERR VOUT_NAME "failed to set overlay ovly\n");

	/* Enable the pipeline and set the Go bit */
	ret = pxa168vid_apply_changes(ovly);
	if (ret) {
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");
		goto error2;
	} else
		goto success;
error2:
	video_device_release(vdev);
error1:
	kfree(ovly);
	return ret;

success:
	printk(KERN_INFO VOUT_NAME ": registered and initialized "
	       "video device %d [v4l2]\n", vdev->minor);

	return -ENODEV;
}

static void pxa168_ovly_cleanup_device(struct pxa168_overlay *ovly)
{
	struct video_device *vdev;

	if (!ovly)
		return;
	vdev = ovly->vdev;

	if (vdev) {
		if (vdev->minor == -1) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vdev);
		} else {
			/*
			 * The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device(vdev);
		}
	}

	kfree(ovly);
}

/* Driver functions */
static int pxa168_ovly_remove(struct platform_device *pdev)
{
	struct pxa168_overlay *ovly = platform_get_drvdata(pdev);

	pxa168_ovly_cleanup_device(ovly);

	/* put_device(ovly); */
	kfree(ovly);
	return 0;
}

static int __init pxa168_ovly_probe(struct platform_device *pdev)
{
	int ret = 0;
	int temp;
	struct pxa168_overlay *ovly = NULL;
	struct pxa168fb_mach_info *mi;

	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -EINVAL;

	mi = pdev->dev.platform_data;
	if (mi == NULL)
		return -EINVAL;

	ovly = kzalloc(sizeof(struct pxa168_overlay), GFP_KERNEL);
	if (ovly == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	/* get LCD clock information. */
	ovly->clk = clk_get(&pdev->dev, "LCDCLK");

	ovly->id = pdev->id;
	ovly->dev = &pdev->dev;

	/*
	 * Map LCD controller registers.
	 */
	ovly->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (ovly->reg_base == NULL) {
		printk("pxa168fb_ovly: no enough memory 2!\n");
		ret = -ENOMEM;
		goto error0;
	}

	/*
	 * Configure default register values.
	 */
	writel(0x00000000, ovly->reg_base + LCD_SPU_DMA_START_ADDR_Y1);
	writel(0x00000000, ovly->reg_base + LCD_SPU_DMA_START_ADDR_U1);
	writel(0x00000000, ovly->reg_base + LCD_SPU_DMA_START_ADDR_V1);
	writel(0x00000000, ovly->reg_base + LCD_SPUT_DMA_OVSA_HPXL_VLN);
	/* Set this frame off by default */
	temp = readl(ovly->reg_base + LCD_SPU_DMA_CTRL0);
	temp &= ~(CFG_DMA_ENA_MASK);
	writel(temp, ovly->reg_base + LCD_SPU_DMA_CTRL0);

	/*
	 * Get IRQ number.
	 */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL)
		return -EINVAL;
	ovly->name = "ovly";

	v4l2_ovly[pdev->id] = ovly;

	/* register v4l2 interface */
	ret = pxa168_ovly_create_video_devices(ovly);
	if (ret)
		goto error0;

	v4l2_dbg(1, debug, ovly->vdev, "v4l2_ovly probed\n");
	return 0;

error0:
	return ret;
}

static const struct platform_device_id mmpv4l2_id_table[] = {
	{"pxa168-v4l2_ovly", 0},
	{"pxa910-v4l2_ovly", 1},
};

static struct platform_driver pxa168_ovly_driver = {
	.driver = {
		   .name = VOUT_NAME,
		   },
	.probe = pxa168_ovly_probe,
	.remove = pxa168_ovly_remove,
	.id_table = mmpv4l2_id_table,
};

static int __init pxa168_ovly_init(void)
{

	if (platform_driver_register(&pxa168_ovly_driver) != 0) {
		printk(KERN_ERR VOUT_NAME
		       ": could not register PXA168 V4L2 driver\n");
		return -EINVAL;
	}
	return 0;
}

static void pxa168_ovly_cleanup(void)
{
	platform_driver_unregister(&pxa168_ovly_driver);
}

late_initcall(pxa168_ovly_init);
module_exit(pxa168_ovly_cleanup);
