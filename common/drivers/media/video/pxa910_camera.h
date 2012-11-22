/*
 * Register definitions for the m88alp01 camera interface.  Offsets in bytes
 * as given in the spec.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.
 *
 * Written by Jonathan Corbet, corbet@lwn.net.
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#define REG_Y0BAR	0x00
#define REG_Y1BAR	0x04
#define REG_Y2BAR	0x08
#define REG_U0BAR	0x0c
#define REG_U1BAR	0x10
#define REG_U2BAR	0x14
#define REG_V0BAR	0x18
#define REG_V1BAR	0x1C
#define REG_V2BAR	0x20

/* for MIPI enable */
#define REG_CSI2_CTRL0	0x100
#define REG_CSI2_DPHY0	0x120
#define REG_CSI2_DPHY1  0x124
#define REG_CSI2_DPHY2  0x128
#define REG_CSI2_DPHY3  0x12c
#define REG_CSI2_DPHY4  0x130
#define REG_CSI2_DPHY5  0x134
#define REG_CSI2_DPHY6  0x138

/* ... */

#define REG_IMGPITCH	0x24	/* Image pitch register */
#define   IMGP_YP_SHFT	  2		/* Y pitch params */
#define   IMGP_YP_MASK	  0x00003ffc	/* Y pitch field */
#define	  IMGP_UVP_SHFT	  18		/* UV pitch (planar) */
#define   IMGP_UVP_MASK   0x3ffc0000
#define REG_IRQSTATRAW	0x28	/* RAW IRQ Status */
#define   IRQ_EOF0	  0x00000001	/* End of frame 0 */
#define   IRQ_EOF1	  0x00000002	/* End of frame 1 */
#define   IRQ_EOF2	  0x00000004	/* End of frame 2 */
#define   IRQ_SOF0	  0x00000008	/* Start of frame 0 */
#define   IRQ_SOF1	  0x00000010	/* Start of frame 1 */
#define   IRQ_SOF2	  0x00000020	/* Start of frame 2 */
#define   IRQ_OVERFLOW	  0x00000040	/* FIFO overflow */
#define   IRQ_TWSIW       0x00000100    /* TWSI (smbus) write */
#define   IRQ_TWSIR       0x00000200    /* TWSI read */
#define   IRQ_TWSIE       0x00000400    /* TWSI error */
#define   TWSIIRQS (IRQ_TWSIW|IRQ_TWSIR|IRQ_TWSIE)
#define   FRAMEIRQS (IRQ_EOF0|IRQ_EOF1|IRQ_EOF2 | IRQ_OVERFLOW)
#define   ALLIRQS (TWSIIRQS|FRAMEIRQS|IRQ_OVERFLOW)
#define REG_IRQMASK	0x2c	/* IRQ mask - same bits as IRQSTAT */
#define REG_IRQSTAT	0x30	/* IRQ status / clear */

#define REG_IMGSIZE	0x34	/* Image size */
#define  IMGSZ_V_MASK	  0x1fff0000
#define  IMGSZ_V_SHIFT	  16
#define	 IMGSZ_H_MASK	  0x00003fff
#define REG_IMGOFFSET	0x38	/* IMage offset */

#define REG_CTRL0	0x3c	/* Control 0 */
#define   C0_ENABLE	  0x00000001	/* Makes the whole thing go */
#define   CO_EOF_VSYNC	  (1<<22)	/*generate eof by VSYNC */
#define   C0_VEDGE_CTRL	  (1 << 23)
/* Mask for all the format bits */
#define   C0_DF_MASK	  0x00fffffc    /* Bits 2-23 */

/* RGB ordering */
#define   C0_RGB4_RGBX	  0x00000000
#define	  C0_RGB4_XRGB	  0x00000004
#define	  C0_RGB4_BGRX	  0x00000008
#define   C0_RGB4_XBGR	  0x0000000c
#define   C0_RGB5_RGGB	  0x00000000
#define	  C0_RGB5_GRBG	  0x00000004
#define	  C0_RGB5_GBRG	  0x00000008
#define   C0_RGB5_BGGR	  0x0000000c

/* Spec has two fields for DIN and DOUT, but they must match, so
   combine them here. */
#define   C0_DF_YUV	  0x00000000    /* Data is YUV	    */
#define   C0_DF_RGB	  0x000000a0	/* ... RGB		    */
#define   C0_DF_BAYER     0x00000140	/* ... Bayer                */
/* 8-8-8 must be missing from the below - ask */
#define   C0_RGBF_565	  0x00000000
#define   C0_RGBF_444	  0x00000800
#define   C0_RGB_BGR	  0x00001000	/* Blue comes first */
#define   C0_YUV_PLANAR	  0x00000000	/* YUV 422 planar format */
#define   C0_YUV_PACKED	  0x00008000	/* YUV 422 packed	*/
#define   C0_YUV_420PL	  0x0000a000	/* YUV 420 planar	*/
/* Think that 420 packed must be 111 - ask */
#define	  C0_YUVE_YUYV	  0x00000000	/* Y1CbY0Cr  */
#define	  C0_YUVE_YVYU	  0x00010000	/* Y1CrY0Cb  */
#define	  C0_YUVE_VYUY	  0x00020000	/* CrY1CbY0  */
#define	  C0_YUVE_UYVY	  0x00030000	/* CbY1CrY0  */
#define   C0_YUVE_XYUV	  0x00000000    /* 420: .YUV */
#define	  C0_YUVE_XYVU	  0x00010000	/* 420: .YVU */
#define	  C0_YUVE_XUVY	  0x00020000	/* 420: .UVY */
#define	  C0_YUVE_XVUY	  0x00030000	/* 420: .VUY */
/* Bayer bits 18,19 if needed */
#define   C0_HPOL_LOW	  0x01000000	/* HSYNC polarity active low */
#define   C0_VPOL_LOW	  0x02000000	/* VSYNC polarity active low */
#define   C0_VCLK_LOW	  0x04000000	/* VCLK on falling edge */
#define   C0_DOWNSCALE	  0x08000000	/* Enable downscaler */
#define	  C0_SIFM_MASK	  0xc0000000	/* SIF mode bits */
#define   C0_SIF_HVSYNC	  0x00000000	/* Use H/VSYNC */
#define   CO_SOF_NOSYNC	  0x40000000	/* Use inband active signaling */


#define REG_CTRL1	0x40	/* Control 1 */
#define   C1_444ALPHA	  0x00f00000	/* Alpha field in RGB444 */
#define   C1_ALPHA_SHFT	  20
#define   C1_DMAB32	  0x00000000	/* 32-byte DMA burst */
#define   C1_DMAB16	  0x02000000	/* 16-byte DMA burst */
#define	  C1_DMAB64	  0x04000000	/* 64-byte DMA burst */
#define	  C1_DMAB_MASK	  0x06000000
#define   C1_TWOBUFS	  0x08000000	/* Use only two DMA buffers */
#define   C1_PWRDWN	  0x10000000	/* Power down */

#define REG_LNNUM	0x60	/* lines num DMA filled */
#define REG_CLKCTRL	0x88	/* Clock control */
#define   CLK_DIV_MASK	  0x0000ffff	/* Upper bits RW "reserved" */


/*
 * Useful stuff that probably belongs somewhere global.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define CIF_WIDTH	352
#define CIF_HEIGHT	288

#define SENSOR_LOW 0
#define SENSOR_HIGH 1
#define SENSOR_MAX 2
#if defined(CONFIG_PXA910_CAMERA)
struct ccic_camera;

void ccic_set_clock(unsigned int reg, unsigned int val);
unsigned int ccic_get_clock(unsigned int reg);
void ccic_set_clock_mipi(void);
void ccic_set_clock_parallel(void);
void ccic_disable_mclk(void);
void ccic_enable_mclk(struct ccic_camera *cam);
int ccic_sensor_attach(struct i2c_client *client);
void ccic_disable_clock(void);
#endif
