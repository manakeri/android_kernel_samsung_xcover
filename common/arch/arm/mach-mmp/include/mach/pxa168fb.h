/*
 * linux/arch/arm/mach-mmp/include/mach/pxa168fb.h
 *
 *  Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_PXA168FB_H
#define __ASM_MACH_PXA168FB_H
/* ---------------------------------------------- */
/*              Header Files                      */
/* ---------------------------------------------- */
#include <linux/fb.h>
#include <linux/earlysuspend.h>
/* ---------------------------------------------- */
/*              IOCTL Definition                  */
/* ---------------------------------------------- */
#define FB_IOC_MAGIC                        'm'
#define FB_IOCTL_CONFIG_CURSOR              _IO(FB_IOC_MAGIC, 0)
#define FB_IOCTL_DUMP_REGS                  _IO(FB_IOC_MAGIC, 1)
#define FB_IOCTL_CLEAR_IRQ                  _IO(FB_IOC_MAGIC, 2)

/*
 * There are many video mode supported.
 */
#define FB_IOCTL_SET_VIDEO_MODE             _IO(FB_IOC_MAGIC, 3)
#define FB_IOCTL_GET_VIDEO_MODE             _IO(FB_IOC_MAGIC, 4)
/* Request a new video buffer from driver. User program needs to free
 * this memory.
 */
#define FB_IOCTL_CREATE_VID_BUFFER          _IO(FB_IOC_MAGIC, 5)

/* Configure viewport in driver. */
#define FB_IOCTL_SET_VIEWPORT_INFO          _IO(FB_IOC_MAGIC, 6)
#define FB_IOCTL_GET_VIEWPORT_INFO          _IO(FB_IOC_MAGIC, 7)

/* Flip the video buffer from user mode. Vide buffer can be separated into:
 * a. Current-used buffer - user program put any data into it. It will be
 *    displayed immediately.
 * b. Requested from driver but not current-used - user programe can put any
 *    data into it. It will be displayed after calling
 *    FB_IOCTL_FLIP_VID_BUFFER.
 *    User program should free this memory when they don't use it any more.
 * c. User program alloated - user program can allocated a contiguos DMA
 *    buffer to store its video data. And flip it to driver. Notices that
 *    this momory should be free by user programs. Driver won't take care of
 *    this.
 */
#define FB_IOCTL_FLIP_VID_BUFFER            _IO(FB_IOC_MAGIC, 8)

/* Get the current buffer information. User program could use it to display
 * anything directly. If developer wants to allocate multiple video layers,
 * try to use FB_IOCTL_CREATE_VID_BUFFER  to request a brand new video
 * buffer.
 */
#define FB_IOCTL_GET_BUFF_ADDR              _IO(FB_IOC_MAGIC, 9)

/* Get/Set offset position of screen */
#define FB_IOCTL_SET_VID_OFFSET             _IO(FB_IOC_MAGIC, 10)
#define FB_IOCTL_GET_VID_OFFSET             _IO(FB_IOC_MAGIC, 11)

/* Turn on the memory toggle function to improve the frame rate while playing
 * movie.
 */
#define FB_IOCTL_SET_MEMORY_TOGGLE          _IO(FB_IOC_MAGIC, 12)

/* Turn on the memory toggle function to improve the frame rate while playing
 * movie.
 */
#define FB_IOCTL_SET_COLORKEYnALPHA         _IO(FB_IOC_MAGIC, 13)
#define FB_IOCTL_GET_COLORKEYnALPHA         _IO(FB_IOC_MAGIC, 14)
#define FB_IOCTL_SWITCH_GRA_OVLY            _IO(FB_IOC_MAGIC, 15)
#define FB_IOCTL_SWITCH_VID_OVLY            _IO(FB_IOC_MAGIC, 16)

/* For VPro integration */
#define FB_IOCTL_GET_FREELIST               _IO(FB_IOC_MAGIC, 17)

/* Wait for vsync happen. */
#define FB_IOCTL_WAIT_VSYNC                 _IO(FB_IOC_MAGIC, 18)

/* clear framebuffer: Makes resolution or color space changes look nicer */
#define FB_IOCTL_CLEAR_FRAMEBUFFER              _IO(FB_IOC_MAGIC, 19)

/* Wait for vsync each time pan display */
#define FB_IOCTL_WAIT_VSYNC_ON              _IO(FB_IOC_MAGIC, 20)
#define FB_IOCTL_WAIT_VSYNC_OFF             _IO(FB_IOC_MAGIC, 21)

/* Get and set the display surface */
#define FB_IOCTL_GET_SURFACE			_IO(FB_IOC_MAGIC, 22)
#define FB_IOCTL_SET_SURFACE			_IO(FB_IOC_MAGIC, 23)

/* Global alpha blend controls - Maintaining compatibility with existing
   user programs. */
#define FB_IOCTL_PUT_VIDEO_ALPHABLEND            0xeb
#define FB_IOCTL_PUT_GLOBAL_ALPHABLEND           0xe1
#define FB_IOCTL_PUT_GRAPHIC_ALPHABLEND          0xe2

/* color swapping */
#define FB_IOCTL_SWAP_GRAPHIC_RED_BLUE       0xe3
#define FB_IOCTL_SWAP_GRAPHIC_U_V            0xe4
#define FB_IOCTL_SWAP_GRAPHIC_Y_UV           0xe5
#define FB_IOCTL_SWAP_VIDEO_RED_BLUE         0xe6
#define FB_IOCTL_SWAP_VIDEO_U_V              0xe7
#define FB_IOCTL_SWAP_VIDEO_Y_UV             0xe8

/* colorkey compatibility */
#define FB_IOCTL_GET_CHROMAKEYS                  0xe9
#define FB_IOCTL_PUT_CHROMAKEYS                  0xea

/* cmu operation */
#define FB_IOCTL_CMU_SWITCH		0xf1
#define FB_IOCTL_CMU_WRITE		0xf2
#define FB_IOCTL_CMU_READ		0xf3
#define FB_IOCTL_CMU_SET_ROUTE  0xf4
#define FB_IOCTL_CMU_SET_PIP	0xf5
#define FB_IOCTL_CMU_GET_RES   0xf6
#define FB_IOCTL_CMU_SET_LETTER_BOX 0xf7

#define FB_VMODE_RGB565			0x100
#define FB_VMODE_BGR565                 0x101
#define FB_VMODE_RGB1555		0x102
#define FB_VMODE_BGR1555                0x103
#define FB_VMODE_RGB888PACK		0x104
#define FB_VMODE_BGR888PACK		0x105
#define FB_VMODE_RGB888UNPACK		0x106
#define FB_VMODE_BGR888UNPACK		0x107
#define FB_VMODE_RGBA888		0x108
#define FB_VMODE_BGRA888		0x109

#define FB_VMODE_YUV422PACKED               0x0
#define FB_VMODE_YUV422PACKED_SWAPUV        0x1
#define FB_VMODE_YUV422PACKED_SWAPYUorV     0x2
#define FB_VMODE_YUV422PLANAR               0x3
#define FB_VMODE_YUV422PLANAR_SWAPUV        0x4
#define FB_VMODE_YUV422PLANAR_SWAPYUorV     0x5
#define FB_VMODE_YUV420PLANAR               0x6
#define FB_VMODE_YUV420PLANAR_SWAPUV        0x7
#define FB_VMODE_YUV420PLANAR_SWAPYUorV     0x8
#define FB_VMODE_YUV422PACKED_IRE_90_270    0x9

#define FB_HWCMODE_1BITMODE                 0x0
#define FB_HWCMODE_2BITMODE                 0x1

#define FB_DISABLE_COLORKEY_MODE            0x0
#define FB_ENABLE_Y_COLORKEY_MODE           0x1
#define FB_ENABLE_U_COLORKEY_MODE           0x2
#define FB_ENABLE_V_COLORKEY_MODE           0x4
#define FB_ENABLE_RGB_COLORKEY_MODE         0x3
#define FB_ENABLE_R_COLORKEY_MODE           0x5
#define FB_ENABLE_G_COLORKEY_MODE           0x6
#define FB_ENABLE_B_COLORKEY_MODE           0x7

#define FB_VID_PATH_ALPHA		0x0
#define FB_GRA_PATH_ALPHA		0x1
#define FB_CONFIG_ALPHA			0x2

#define FB_SYNC_COLORKEY_TO_CHROMA          1
#define FB_SYNC_CHROMA_TO_COLORKEY          2

#define PXA168FB_FB_NUM           2
/* overlay max buffer number */
#define MAX_QUEUE_NUM   30

/* ---------------------------------------------- */
/*              Data Structure                    */
/* ---------------------------------------------- */
/*
 * The follow structures are used to pass data from
 * user space into the kernel for the creation of
 * overlay surfaces and setting the video mode.
 */

#define FBVideoMode int

struct _sViewPortInfo {
        unsigned short srcWidth;        /* video source size */
        unsigned short srcHeight;
        unsigned short zoomXSize;       /* size after zooming */
        unsigned short zoomYSize;
        unsigned short yPitch;
        unsigned short uPitch;
	unsigned short vPitch;
		unsigned int rotation;
		unsigned int yuv_format;
};

struct _sViewPortOffset {
        unsigned short xOffset;         /* position on screen */
        unsigned short yOffset;
};

struct _sVideoBufferAddr {
        unsigned char   frameID;        /* which frame wants */
        unsigned char *startAddr[3];    /* new buffer (PA). three addr for YUV planar */
        unsigned char *inputData;       /* input buf address (VA) */
        unsigned int length;            /* input data's length */
};

/* The pxa168_fb_chroma structure is here for legacy compatibility with former
 * PXA display driver usage.
 */

struct pxa168_fb_chroma {
        u_char     mode;
        u_char     y_alpha;
        u_char     y;
        u_char     y1;
        u_char     y2;
        u_char     u_alpha;
        u_char     u;
        u_char     u1;
        u_char     u2;
        u_char     v_alpha;
        u_char     v;
        u_char     v1;
        u_char     v2;
};


struct _sColorKeyNAlpha {
	unsigned int mode;
	unsigned int alphapath;
	unsigned int config;
	unsigned int Y_ColorAlpha;
	unsigned int U_ColorAlpha;
	unsigned int V_ColorAlpha;
};

struct _sOvlySurface {
        FBVideoMode videoMode;
        struct _sViewPortInfo viewPortInfo;
        struct _sViewPortOffset viewPortOffset;
        struct _sVideoBufferAddr videoBufferAddr;
	unsigned int user_data[4];
};

struct _sCursorConfig {
        unsigned char   enable;         /* enable cursor or not */
        unsigned char   mode;           /* 1bit or 2bit mode */
        unsigned int color1;            /* foreground color */
        unsigned int color2;            /* background color */
        unsigned short xoffset;
        unsigned short yoffset;
        unsigned short  width;
        unsigned short height;
        unsigned char *pBuffer;         /* cursor data */
};

struct _pxa168fb_cmu_config {
	unsigned int addr;
	unsigned int data;
};

struct _pxa168fb_cmu_pip {
	unsigned int left;
	unsigned int right;
	unsigned int top;
	unsigned int bottom;
};

/* Dumb interface */
#define PIN_MODE_DUMB_24		0
#define PIN_MODE_DUMB_18_SPI		1
#define PIN_MODE_DUMB_18_GPIO		2
#define PIN_MODE_DUMB_16_SPI		3
#define PIN_MODE_DUMB_16_GPIO		4
#define PIN_MODE_DUMB_12_SPI_GPIO	5
#define PIN_MODE_SMART_18_SPI		6
#define PIN_MODE_SMART_16_SPI		7
#define PIN_MODE_SMART_8_SPI_GPIO	8
#define PIN_MODE_DUMB_18_SMART_8	9
#define PIN_MODE_DUMB_16_SMART_8_SPI	10
#define PIN_MODE_DUMB_16_SMART_8_GPIO	11
#define PIN_MODE_DUMB_16_DUMB_16	12

/* Dumb interface pin allocation */
#define DUMB_MODE_RGB565		0
#define DUMB_MODE_RGB565_UPPER		1
#define DUMB_MODE_RGB666		2
#define DUMB_MODE_RGB666_UPPER		3
#define DUMB_MODE_RGB444		4
#define DUMB_MODE_RGB444_UPPER		5
#define DUMB_MODE_RGB888		6

/* default fb buffer size WVGA-32bits */
#define DEFAULT_FB_SIZE	(800 * 480 * 4)

/*
 * Buffer pixel format
 * bit0 is for rb swap.
 * bit12 is for Y UorV swap
 */
#define PIX_FMT_RGB565		0
#define PIX_FMT_BGR565		1
#define PIX_FMT_RGB1555		2
#define PIX_FMT_BGR1555		3
#define PIX_FMT_RGB888PACK	4
#define PIX_FMT_BGR888PACK	5
#define PIX_FMT_RGB888UNPACK	6
#define PIX_FMT_BGR888UNPACK	7
#define PIX_FMT_RGBA888		8
#define PIX_FMT_BGRA888		9
#define PIX_FMT_YUV422PACK	10
#define PIX_FMT_YVU422PACK	11
#define PIX_FMT_YUV422PLANAR	12
#define PIX_FMT_YVU422PLANAR	13
#define PIX_FMT_YUV420PLANAR	14
#define PIX_FMT_YVU420PLANAR	15
#define PIX_FMT_PSEUDOCOLOR	20
#define PIX_FMT_YUYV422PACK	(0x1000|PIX_FMT_YUV422PACK)
#define PIX_FMT_YUV422PACK_IRE_90_270	(0x1000|PIX_FMT_RGB888UNPACK)

/*
 * panel interface
 */
#define DPI		0
#define DSI2DPI		1
#define DSI		2

#ifdef __KERNEL__
#include <linux/interrupt.h>

/*
 * PXA LCD controller private state.
 */
struct pxa168fb_info {
	struct device		*dev;
	struct clk		*clk;
	int                     id;
	void			*reg_base;
	void			*dsi1_reg_base;
	void			*dsi2_reg_base;
	unsigned long		new_addr[3];	/* three addr for YUV planar */
	unsigned char		*filterBufList[MAX_QUEUE_NUM][3];
	unsigned char		*buf_freelist[MAX_QUEUE_NUM];
	unsigned char		*buf_waitlist[MAX_QUEUE_NUM];
	unsigned char		*buf_current;
	unsigned char		*buf_retired;
	dma_addr_t		fb_start_dma;
	void			*fb_start;
	int			fb_size;
	dma_addr_t		fb_start_dma_bak;
	void			*fb_start_bak;
	int			fb_size_bak;
	atomic_t		op_count;
	atomic_t		w_intr;
	wait_queue_head_t	w_intr_wq;
	struct mutex		access_ok;
	spinlock_t		buf_lock;
	struct _sOvlySurface    surface;
	struct _sOvlySurface    surface_bak;
	struct _sColorKeyNAlpha ckey_alpha;
	struct fb_videomode	dft_vmode;
	struct fb_videomode     out_vmode;
	int                     fixed_output;
	unsigned char		*hwc_buf;
	unsigned int		pseudo_palette[16];
	char			*mode_option;
	struct fb_info          *fb_info;
	struct fb_info          *info;
	int                     io_pin_allocation;
	int			pix_fmt;
	unsigned		is_blanked:1,
				surface_set:1,
				debug:1,
				active:1;

	/* lock for variables, e.g. active */
	spinlock_t		var_lock;
	/* mem_status:
	* 0: DMA mem is from DMA region.
	* 1: DMA mem is from normal region.
	* -1: no frame buffer.
	*/
	int			mem_status;
	unsigned                wait_vsync;
	struct pxa168_fb_chroma chroma;

	/* suspend/resume related */
	struct early_suspend    early_suspend;
	int                     dma_ctrl0;
	unsigned		irq_mask;
#ifdef OVLY_TASKLET
	/* tasklet */
	unsigned		tasklet;
	struct tasklet_struct	ovly_task;
#else
	/* set 1 to use system thread instead of single thread */
	unsigned		system_work;
	/* system thread to handle work, priority 20 */
	struct work_struct	buf_work;
	/* single thread workqueue for higher priority */
	struct workqueue_struct *work_q;
#endif
};

struct dsi_phy {
	unsigned int hs_prep_constant;    /* Unit: ns. */
	unsigned int hs_prep_ui;
	unsigned int hs_zero_constant;
	unsigned int hs_zero_ui;
	unsigned int hs_trail_constant;
	unsigned int hs_trail_ui;
	unsigned int hs_exit_constant;
	unsigned int hs_exit_ui;
	unsigned int ck_zero_constant;
	unsigned int ck_zero_ui;
	unsigned int ck_trail_constant;
	unsigned int ck_trail_ui;
	unsigned int req_ready;
};

struct dsi_info {
	unsigned	id;
	unsigned	regs;
	unsigned	lanes;
	unsigned	bpp;
	unsigned	rgb_mode;
	unsigned	burst_mode;
	unsigned	lpm_line_en;
	unsigned	lpm_frame_en;
	unsigned	last_line_turn;
	unsigned	hex_slot_en;
	unsigned	all_slot_en;
	unsigned	hbp_en;
	unsigned	hact_en;
	unsigned	hfp_en;
	unsigned	hex_en;
	unsigned	hlp_en;
	unsigned	hsa_en;
	unsigned	hse_en;
	unsigned	eotp_en;
	struct dsi_phy  *phy;
};
/*
 * CMU information
 */
struct cmu_calibration {
	int left;
	int right;
	int top;
	int bottom;
};
struct cmu_res {
	int width;
	int height;
};

/*
 * PXA fb machine information
 */
struct pxa168fb_mach_info {
	char	id[16];
	unsigned int	sclk_src;
	unsigned int	sclk_div;

	int		num_modes;
	struct fb_videomode *modes;
	unsigned int max_fb_size;

	/*
	 * Pix_fmt
	 */
	unsigned	pix_fmt;

	/*
	 * Burst length
	 */
	unsigned	burst_len;
	/*
	 *ISR clear mask
	 */
	unsigned	isr_clear_mask;
	/*
	 * I/O pin allocation.
	 */
	unsigned int	io_pin_allocation_mode;

	/*
	 * Dumb panel -- assignment of R/G/B component info to the 24
	 * available external data lanes.
	 */
	unsigned	dumb_mode:4;
	unsigned	panel_rgb_reverse_lanes:1;

	/*
	 * Dumb panel -- GPIO output data.
	 */
	unsigned	gpio_output_mask:8;
	unsigned	gpio_output_data:8;

	/*
	 * Dumb panel -- configurable output signal polarity.
	 */
	unsigned	invert_composite_blank:1;
	unsigned	invert_pix_val_ena:1;
	unsigned	invert_pixclock:1;
	unsigned	invert_vsync:1;
	unsigned	invert_hsync:1;
	unsigned	panel_rbswap:1;
	unsigned	active:1;
	unsigned	enable_lcd:1;
	unsigned	mmap:1;
	/*
	 * SPI control
	 */
	unsigned int	spi_ctrl;
	unsigned int	spi_gpio_cs;
	unsigned int	spi_gpio_reset;

	/*
	 * panel interface
	*/
	unsigned int	phy_type;
	int		(*phy_init)(struct pxa168fb_info *);

	/*
	 * vdma option
	 */
	unsigned int vdma_enable;

	/*
	 * power on/off function.
	 */
	int (*pxa168fb_lcd_power)(struct pxa168fb_info *, unsigned int, unsigned int, int);

	/*
	 * dsi to dpi setting function
	 */
	int (*dsi2dpi_set)(struct pxa168fb_info *);
	int (*xcvr_reset)(struct pxa168fb_info *);
	/*
	 * dsi setting function
	 */
	void (*dsi_set)(struct pxa168fb_info *);
	struct dsi_info *dsi;
	/*
	 * special ioctls
	 */
	int (*ioctl)(struct fb_info *info, unsigned int cmd, unsigned long arg);
	/*CMU platform calibration*/
	struct cmu_calibration cmu_cal[3];
	struct cmu_calibration cmu_cal_letter_box[3];
};

struct fbi_info {
	struct pxa168fb_info *fbi[3];
	int	xres;
	int	yres;
	int	xres_z;
	int	yres_z;
	int	bpp;
	int	xres_virtual;
	/* flag for overlay buffer management:
	 *	0: wait_for_vsync at GET_FREELIST;
	 *	1: wait_peer @ pxa168fb_ovly_isr */
	int	flag;
	/* wait_peer is used only for mirror mode to update overlay buffer
	*	1: The first path's display done come,
	*	 update register and set wait_peer = 1
	*	0: The second path's display done come,
	*	 return buffer and clear wait_peer */
	int	wait_peer;
	int	retire_err;
};

#define fb_base		0
#define fb_dual		1
#define FB_MODE_DUP	((fbi->id == fb_base) && fb_mode && gfx_info.fbi[fb_dual])

/* DSI burst mode */
#define DSI_BURST_MODE_SYNC_PULSE			0x0
#define DSI_BURST_MODE_SYNC_EVENT			0x1
#define DSI_BURST_MODE_BURST				0x2

/* DSI input data RGB mode */
#define DSI_LCD_INPUT_DATA_RGB_MODE_565			0
#define DSI_LCD_INPUT_DATA_RGB_MODE_666PACKET		1
#define DSI_LCD_INPUT_DATA_RGB_MODE_666UNPACKET		2
#define DSI_LCD_INPUT_DATA_RGB_MODE_888			3

/*VDMA*/
#define EOF_TIMEOUT 20

extern int fb_mode;
extern struct device_attribute dev_attr_dsi;
extern struct fbi_info gfx_info;
extern struct lcd_regs *get_regs(struct pxa168fb_info *fbi);
extern struct cmu_calibration cmu_cal[3];
extern struct cmu_calibration cmu_cal_letter_box[3];
extern struct cmu_res res;

extern u32 dma_ctrl_read(struct pxa168fb_info *fbi, int ctrl1);
extern void dma_ctrl_write(struct pxa168fb_info *fbi, int ctrl1, u32 value);
extern void dma_ctrl_set(struct pxa168fb_info *fbi, int ctrl1, u32 mask, u32 value);
extern void irq_mask_set(struct pxa168fb_info *fbi, u32 mask, u32 val);
extern void irq_status_clear(struct pxa168fb_info *fbi, u32 mask);
extern int sclk_div_get(struct pxa168fb_info *fbi);
extern void sclk_div_set(struct pxa168fb_info *fbi, int divider);

extern int pxa168fb_spi_send(struct pxa168fb_info *fbi, void *cmd, int count, unsigned int spi_gpio_cs);
extern int pxa688_cmu_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
extern void pxa168fb_ovly_dual(int enable);

/* dsi related */
extern void pxa168fb_dsi_send(struct pxa168fb_info *fbi, void *value);
extern void dsi_cclk_set(struct pxa168fb_info *fbi, int en);
extern void dsi_set_dphy(struct pxa168fb_info *fbi);
extern void dsi_reset(struct pxa168fb_info *fbi, int hold);
extern void dsi_set_controller(struct pxa168fb_info *fbi);

/* VDMA related */
extern u32 vdma_paddr;
extern unsigned int vdma_lines;
extern unsigned int sram_freebytes;
extern void pxa688_vdma_clkset(int en);
extern u32 pxa688fb_vdma_squ_malloc(void);
extern void pxa688fb_vdma_set(struct pxa168fb_info *fbi, u32 psqu, unsigned int lines, unsigned int layer);
extern void pxa688fb_vdma_release(struct pxa168fb_info *fbi);
extern int pxa688fb_vdma_get_linenum(struct pxa168fb_info *fbi);
extern u32 vdma_ctrl_read(struct pxa168fb_info *fbi);
extern void vdma_ctrl_write(struct pxa168fb_info *fbi, int value);

#endif /* __KERNEL__ */
#endif /* __ASM_MACH_PXA168FB_H */
