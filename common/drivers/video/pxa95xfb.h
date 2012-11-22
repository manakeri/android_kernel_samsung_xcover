/*
 * linux/arch/arm/mach-pxa/include/mach/regs-lcd-pxa95x.h
 *
 *  Copyright (C) 2009-2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_REGS_LCD_PXA95x_H
#define __ASM_ARCH_REGS_LCD_PXA95x_H

#include <linux/types.h>

/*   registers address */
#define LCD_CTL			0x0
#define LCD_CTL_STS		0x0004
#define LCD_CTL_INT_STS		0x0100

#define LCD_FETCH_STS0		0x0204

#define LCD_NXT_DESC_ADDR0      0x0208
#define LCD_NXT_DESC_ADDR1      0x0248
#define LCD_NXT_DESC_ADDR2      0x0288
#define LCD_NXT_DESC_ADDR3      0x02C8
#define LCD_NXT_DESC_ADDR4      0x0308
#define LCD_NXT_DESC_ADDR5      0x0348
#define LCD_NXT_DESC_ADDR6      0x0388

#define LCD_FETCH_CTL0          0x0200
#define LCD_FETCH_CTL1          0x0240
#define LCD_FETCH_CTL2          0x0280
#define LCD_FETCH_CTL3          0x02C0
#define LCD_FETCH_CTL4          0x0300
#define LCD_FETCH_CTL5          0x0340
#define LCD_FETCH_CTL6          0x0380

#define LCD_FR_ADDR0			0x0210

#define LCD_WIN0_CTL            0x0600
#define LCD_WIN1_CTL            0x0640
#define LCD_WIN2_CTL            0x0680
#define LCD_WIN3_CTL            0x06C0
#define LCD_WIN4_CTL            0x07C0
#define LCD_WIN0_CFG            0x0604
#define LCD_WIN1_CFG            0x0644
#define LCD_WIN2_CFG            0x0684
#define LCD_WIN3_CFG            0x06C4
#define LCD_WIN4_CFG            0x07C4

#define LCD_WIN0_SCALE_RW	0x0618
#define LCD_WIN1_SCALE_RW       0x0658
#define LCD_WIN2_SCALE_RW       0x0698
#define LCD_WIN3_SCALE_RW       0x06D8
#define LCD_WIN4_SCALE_RW       0x07D8

#define LCD_WIN0_SCALE_PTR	0x0614
#define LCD_WIN1_SCALE_PTR      0x0654
#define LCD_WIN2_SCALE_PTR      0x0694
#define LCD_WIN3_SCALE_PTR      0x06D4
#define LCD_WIN4_SCALE_PTR      0x07D4

#define LCD_WIN0_CROP0		0x060C
#define LCD_WIN1_CROP0		0x064C
#define LCD_WIN2_CROP0		0x068C
#define LCD_WIN3_CROP0		0x06CC
#define LCD_WIN4_CROP0		0x07CC
#define LCD_WIN0_CROP1		0x0610
#define LCD_WIN1_CROP1		0x0650
#define LCD_WIN2_CROP1		0x0690
#define LCD_WIN3_CROP1		0x06D0
#define LCD_WIN4_CROP1		0x07D0

#define LCD_MIXER0_CTL0         0x0800
#define LCD_MIXER0_CTL1         0x0804
#define LCD_MIXER0_CTL2     	0x0844
#define LCD_MIXER0_STS			0x082C
#define LCD_MIXER0_STS1			0x0848
#define LCD_MIXER0_BP_CFG0      0x0808
#define LCD_MIXER0_BP_CFG1      0x080C
#define LCD_MIXER0_OL1_CFG0	0x0810
#define LCD_MIXER0_OL1_CFG1     0x0814
#define LCD_MIXER0_OL2_CFG0     0x0818
#define LCD_MIXER0_OL2_CFG1     0x081C


#define LCD_MIXER1_CTL0         0x0900
#define LCD_MIXER1_CTL1         0x0904
#define LCD_MIXER1_CTL2     	0x0944
#define LCD_MIXER1_STS			0x092C
#define LCD_MIXER1_STS1			0x0948
#define LCD_MIXER1_BP_CFG0      0x0908
#define LCD_MIXER1_BP_CFG1      0x090C
#define LCD_MIXER1_OL1_CFG0     0x0910
#define LCD_MIXER1_OL1_CFG1     0x0914

#define LCD_MIXER2_CTL0         0x0A00
#define LCD_MIXER2_CTL1         0x0A04
#define LCD_MIXER2_CTL2		0x0A44
#define LCD_MIXER2_STS		0x0A2C
#define LCD_MIXER2_STS1		0x0A48
#define LCD_MIXER2_BP_CFG0      0x0A08
#define LCD_MIXER2_BP_CFG1      0x0A0C
#define LCD_MIXER2_OL1_CFG0     0x0A10
#define LCD_MIXER2_OL1_CFG1     0x0A14

#define LCD_CH_ERR_ADDR         0x05A0
#define LCD_CH0_ALPHA       	0x05C0
#define LCD_CH1_ALPHA       	0x05C4
#define LCD_CH2_ALPHA           0x05C8
#define LCD_CH3_ALPHA           0x05CC
#define LCD_CH4_ALPHA           0x05D0

#define LCD_CH0_CLR_MATCH      	0x05E0
#define LCD_CH1_CLR_MATCH      	0x05E4
#define LCD_CH2_CLR_MATCH      	0x05E8
#define LCD_CH3_CLR_MATCH      	0x05EC
#define LCD_CH4_CLR_MATCH      	0x05F0

#define LCD_CONV0_CTL    	0x1000
#define LCD_CONV0_INT_STS	0x1028
#define LCD_CONV1_INT_STS       0x2028

#define LCD_MIXER0_TIM0		0x1004
#define LCD_MIXER0_TIM1  	0x1008

#define LCD_MIXERx_G_CTL        0x1014
#define LCD_MIXERx_G_DAT_RED    0x1018
#define LCD_MIXERx_G_DAT_GREEN  0x101C
#define LCD_MIXERx_G_DAT_BLUE   0x1020

#define LCD_FETCH_INT_STS0	0x0580
#define LCD_FETCH_INT_STS1      0x0584
#define LCD_GWIN_INT_STS        0x010c
#define LCD_GMIX_INT_STS        0x0110
#define LCD_WIN0_INT_STS        0x061c
#define LCD_MIXER0_INT_STS      0x0840
#define LCD_MIXER1_INT_STS      0x0940

/*  bits definition */
/* LCD_MIXERx_G_CTL bits */
#define LCD_MIXERx_G_CTL_ADD_PTR_R(n)   ((n)<<0)
#define LCD_MIXERx_G_CTL_ADD_PTR_G(n)   ((n)<<4)
#define LCD_MIXERx_G_CTL_ADD_PTR_B(n)   ((n)<<8)
#define LCD_MIXERx_G_CTL_Q4_C(n)        ((n)<<16)
#define LCD_MIXERx_G_CTL_Q3_C(n)        ((n)<<18)
#define LCD_MIXERx_G_CTL_Q2_C(n)        ((n)<<20)
#define LCD_MIXERx_G_CTL_Q1_C(n)        ((n)<<22)
#define LCD_MIXERx_G_CTL_G_VAL(n)       ((n)<<24)

#define LCD_CHx_ALPHA_CH_ALPHA(n)       ((n & 0xff)<<0)
#define LCD_CHx_ALPHA_CLR_KEY_EN(n)     ((n & 0x3)<<29)
#define LCD_CHx_ALPHA_W_ALPHA_EN        (1<<31)

/* ACCR1_REG dsi bits */
#define ACCR1_REG_DSI_BIT1_OFFSET   13
#define ACCR1_REG_DSI_BIT2_OFFSET   16
#define ACCR1_REG_DSI_BIT1(n)	    ((n)<<ACCR1_REG_DSI_BIT1_OFFSET)
#define ACCR1_REG_DSI_BIT2(n)	    ((n)<<ACCR1_REG_DSI_BIT2_OFFSET)
#define ACCR1_REG_DSI_BIT_MASK      (0x7)


/* LCD_CTL bits */
#define LCD_CTL_PCLK_DIV(n)	    ((n))
#define LCD_CTL_GFETCH_INT_EN   (0x1u<<15)
#define LCD_CTL_GWIN_INT_EN	(0x1u<<16)
#define LCD_CTL_GWIN_INT_EN	(0x1u<<16)
#define LCD_CTL_GMIX_INT_EN  	(0x1u<<17)
#define LCD_CTL_CLK_SSP_EN      (0x1u<<21)
#define LCD_CTL_IDLE_CG_EN      (0x1u<<22)
#define LCD_CTL_AXI32_EN    	(0x1u<<23)
#define LCD_CTL_LCD_QD_INT_EN	(0x1u<<27)
#define LCD_CTL_LCD_DIS_INT_EN	(0x1u<<28)
#define LCD_CTL_LCD_EN_INT_EN	(0x1u<<29)
#define LCD_CTL_LCD_QD      	(0x1u<<30)
#define LCD_CTL_LCD_EN      	(0x1u<<31)
#define LCD_CTL_LCD_PCLK_EN_BO  (0x1u<<10)

/* LCD_CTL_STS bits */
#define LCD_CTL_STS_PCLK_DIV_STS (0x1ffu << 0)
#define LCD_CTL_STS_LCD_QD_STS   (0x1u << 27)
#define LCD_CTL_STS_LCD_EN_STS   (0x1u << 29)

/* LCD_CTL_INT_STS bits */
#define LCD_CTL_INT_STS_LCD_QD_INT_STS  (0x1u<<0)
#define LCD_CTL_INT_STS_LCD_DIS_INT_STS (0x1u<<1)
#define LCD_CTL_INT_STS_LCD_EN_INT_STS  (0x1u<<2)
#define LCD_CTL_INT_STS_GFETCH_INT_STS  (0x1u<<8)
#define LCD_CTL_INT_STS_GWIN_INT_STS    (0x1u<<9)
#define LCD_CTL_INT_STS_GMIX_INT_STS    (0x1u<<10)

/* LCD_FETCH_CTLx bits */
#define LCD_FETCH_CTLx_CHAN_EN            (0x1u<<0)
#define LCD_FETCH_CTLx_SRC_FOR(n)         ((n)<<2)
#define LCD_FETCH_CTLx_END_FR_INT_EN      (0x1u<<6)
#define LCD_FETCH_CTLx_START_FR_INT_EN    (0x1u<<7)
#define LCD_FETCH_CTLx_BUS_ERR_INT_EN     (0x1u<<8)
#define LCD_FETCH_CTLx_SP_MODE            (0x1u<<10)
#define LCD_FETCH_CTLx_SP_MODIFIED_FRAME  (0x1u<<11)

/* LCD_FETCH_STSx bits */
#define LCD_FETCH_STSx_CHAN_STS           (0x1u<<0)
#define LCD_FETCH_STSx_SRC_FOR_STS        (0x7u<<2)

/* LCD_NXT_DESC_ADDRx */
#define LCD_NXT_DESC_ADDRx_NXT_DESC_ADDR(n)	    ((n)<<4)

/* LCD_FR_IDx */
#define LCD_FR_IDx_FR_ID(n) ((n))

/* LCD_FR_ADDRx */
#define LCD_FR_ADDRx_FR_ADDR(n) ((n)<<4)

/* LCD_FETCH_INT_STS0 bits */
#define LCD_FETCH_INT_STS0_START_FR0_INT_STS (0x1u<<0)
#define LCD_FETCH_INT_STS0_START_FR1_INT_STS (0x1u<<1)
#define LCD_FETCH_INT_STS0_START_FR2_INT_STS (0x1u<<2)
#define LCD_FETCH_INT_STS0_START_FR3_INT_STS (0x1u<<3)
#define LCD_FETCH_INT_STS0_START_FR4_INT_STS (0x1u<<4)
#define LCD_FETCH_INT_STS0_START_FR5_INT_STS (0x1u<<5)
#define LCD_FETCH_INT_STS0_START_FR6_INT_STS (0x1u<<6)
#define LCD_FETCH_INT_STS0_START_FR7_INT_STS (0x1u<<7)
#define LCD_FETCH_INT_STS0_END_FR0_INT_STS   (0x1u<<16)
#define LCD_FETCH_INT_STS0_END_FR1_INT_STS   (0x1u<<17)
#define LCD_FETCH_INT_STS0_END_FR2_INT_STS   (0x1u<<18)
#define LCD_FETCH_INT_STS0_END_FR3_INT_STS   (0x1u<<19)
#define LCD_FETCH_INT_STS0_END_FR4_INT_STS   (0x1u<<20)
#define LCD_FETCH_INT_STS0_END_FR5_INT_STS   (0x1u<<21)
#define LCD_FETCH_INT_STS0_END_FR6_INT_STS   (0x1u<<22)
#define LCD_FETCH_INT_STS0_END_FR7_INT_STS   (0x1u<<23)
#define LCD_FETCH_INT_STS0_OVERFLOW_INT_STS  (0x1u<<31)

#define LCD_FETCH_INT_STS0_START_FRx(x)		(0x1u<<x)
#define LCD_FETCH_INT_STS0_END_FRx(x)		(0x1u<<(x+16))
#define LCD_FETCH_INT_STS1_BUS_ERRx(x)		(0x1u<<x)
#define LCD_FETCH_INT_STS1_BR_STSx(x)		(0x1u<<(x+16))


/* LCD_WINx_CTL bits */
#define LCD_WINx_CTL_WIN_EN_STS      (0x1u<<0)
#define LCD_WINx_CTL_WIN_URUN_INT_EN (0x1u<<1)
#define LCD_WINx_CTL_WIN_EOF_INT_EN  (0x1u<<2)
#define LCD_WINx_CTL_WIN_SOF_INT_EN  (0x1u<<3)
#define LCD_WINx_CTL_WIN_XRES(n)     ((n)<<8)
#define LCD_WINx_CTL_WIN_YRES(n)     ((n)<<20)

/* LCD_WINx_CFG bits */
#define LCD_WINx_CFG_WIN_XSCALE(n)	((n)<<19)
#define LCD_WINx_CFG_WIN_YSCALE(n)      ((n)<<13)

/* LCD_WINx_CROP bits */
#define LCD_WINx_CROP0_LC(n)		((n)<<0)
#define LCD_WINx_CROP0_TR(n)		((n)<<23)
#define LCD_WINx_CROP1_BR(n)		((n)<<0)
#define LCD_WINx_CROP1_RC(n)		((n)<<23)

/* LCD_MIXERx_CTL0 bits */
#define LCD_MIXERx_CTL0_MIX_EN           (0x1u<<0)
#define LCD_MIXERx_CTL0_BP_COLOR_USE     (0x1u<<1)
#define LCD_MIXERx_CTL0_BP_ID_MASK       (0x1fu<<2)
#define LCD_MIXERx_CTL0_BP_ID(n)         ((n) <<2)
#define LCD_MIXERx_CTL0_OL1_COLOR_USE    (0x1u<<7)
#define LCD_MIXERx_CTL0_OL1_ID_MASK      (0x1fu<<8)
#define LCD_MIXERx_CTL0_OL1_ID(n)        ((n) <<8)
#define LCD_MIXERx_CTL0_OL2_COLOR_USE    (0x1u<<13)
#define LCD_MIXERx_CTL0_OL2_ID_MASK      (0x1fu<<14)
#define LCD_MIXERx_CTL0_OL2_ID(n)        ((n) <<14)
#define LCD_MIXERx_CTL0_OL3_COLOR_USE    (0x1u<<19)
#define LCD_MIXERx_CTL0_OL3_ID_MASK      (0x1fu<<20)
#define LCD_MIXERx_CTL0_OL3_ID(n)        ((n) <<20)
#define LCD_MIXERx_CTL0_CRSR_ID_MASK     (0x1fu<<26)
#define LCD_MIXERx_CTL0_CRSR_ID(n)       ((n) <<26)
#define LCD_MIXERx_CTL0_DISP_UPDATE      (0x1u<<31)
#define LCD_MIXERx_CTL0_OLx_ID(ol, n)         ((n) << (2 + ol * 6))
#define LCD_MIXERx_CTL0_OLx_ID_MASK(ol)       (0x1fu<<(2 + ol * 6))
#define LCD_MIXERx_CTL0_ALL_OL_MASK      (~((1<<31) | (1<<25) | (1<<0)))
#define LCD_MIXERx_CTL0_ALL_OL_DISABLE      ((0x1f<<2) | (0x1f<<8) | (0x1f<<14) | (0x1f<<20) | (0x1f<<26))


/* LCD_MIXERx_CTL1 bits */
#define LCD_MIXERx_CTL1_DISP_EN_INT_EN     (0x1u<<0)
#define LCD_MIXERx_CTL1_DISP_UPDATE_INT_EN (0x1u<<1)
#define LCD_MIXERx_CTL1_BLINK_INT_EN       (0x1u<<2)
#define LCD_MIXERx_CTL1_DISP_XRES(n)       ((n) <<5)
#define LCD_MIXERx_CTL1_DISP_YRES(n)       ((n) <<17)

/* LCD_MIXERx_CTL2 bits */
#define LCD_MIXERx_CTL2_CONV_ID(n)       ((n) <<26)

/* LCD_MIXERx_CFG0 bits */
#define LCD_MIXERx_CFG0_XPOS(n)       ((n) <<0)
#define LCD_MIXERx_CFG0_YPOS(n)       ((n) <<12)

/* LCD_MIXERx_CFG1 bits */
#define LCD_MIXERx_CFG1_XRES(n)       ((n) <<0)
#define LCD_MIXERx_CFG1_YRES(n)       ((n) <<12)

/* LCD_MIXERx_STS0 bits */
#define LCD_MIXERx_STS0_MIX_EN_STS    (0x1u<<0)

/* LCD_CONVx_CTL bits */
#define LCD_CONVx_CTL_DISP_URUN_INT_EN (0x1u<<0)
#define LCD_CONVx_CTL_DISP_EOF_INT_EN  (0x1u<<1)
#define LCD_CONVx_CTL_DISP_SOF_INT_EN  (0x1u<<2)
#define LCD_CONVx_CTL_SMART_INT_EN     (0x1u<<3)
#define LCD_CONVx_CTL_SP_RD_INT_EN     (0x1u<<4)
#define LCD_CONVx_CTL_SP_UNDRUN_INT_EN (0x1u<<5)
#define LCD_CONVx_CTL_SP_BREAK_INT_EN  (0x1u<<7)
#define LCD_CONVx_CTL_FLOW_DIS         (0x1u<<8)
#define LCD_CONVx_CTL_BREAK            (0x1u<<9)
#define LCD_CONVx_CTL_CONV_EN          (0x1u<<10)
#define LCD_CONVx_CTL_TV_FOR           (0x1u<<15)
#define LCD_CONVx_CTL_OP_FOR(n)        ((n) <<16)
#define LCD_CONVx_CTL_END_BIT          (0x1u<<19)
#define LCD_CONVx_CTL_SYNC_CNT(n)      ((n) <<20)
#define LCD_CONVx_CTL_GPIO0_CTL        (0x1u<<28)
#define LCD_CONVx_CTL_GPIO1_CTL        (0x1u<<29)
#define LCD_CONVx_CTL_DISP_TYPE(n)     ((n) <<30)

/* LCD_MIXERx_TIM0 bits */
#define LCD_MIXERx_TIM0_PCP           (0x1u<<0)
#define LCD_MIXERx_TIM0_OEP           (0x1u<<1)
#define LCD_MIXERx_TIM0_VSP           (0x1u<<2)
#define LCD_MIXERx_TIM0_HSP           (0x1u<<3)
#define LCD_MIXERx_TIM0_VSYNC_OUT     (0x1u<<4)
#define LCD_MIXERx_TIM0_MODE          (0x1u<<5)
#define LCD_MIXERx_TIM0_HSW(n)        ((n) <<8)
#define LCD_MIXERx_TIM0_ELW(n)        ((n) <<16)
#define LCD_MIXERx_TIM0_BLW(n)        ((n) <<24)

/* LCD_MIXERx_TIM1 bits */
#define LCD_MIXERx_TIM1_VSW(n)        ((n)<<8)
#define LCD_MIXERx_TIM1_EFW(n)        ((n)<<16)
#define LCD_MIXERx_TIM1_BFW(n)        ((n)<<24)

/* LCD_MIXERx_TIM2 bits */
#define LCD_MIXERx_TIM2_GSUP(n)       ((n)<<0)
#define LCD_MIXERx_TIM2_SPW(n)        ((n)<<8)
#define LCD_MIXERx_TIM2_SPSH(n)       ((n)<<13)
#define LCD_MIXERx_TIM2_GSW(n)        ((n)<<24)

/* LCD_MIXERx_TIM3 bits */
#define LCD_MIXERx_TIM3_REVDEL(n)     ((n) <<0)
#define LCD_MIXERx_TIM3_GSP           (0x1u<<8)
#define LCD_MIXERx_TIM3_SPP           (0x1u<<9)
#define LCD_MIXERx_TIM3_PSP           (0x1u<<10)
#define LCD_MIXERx_TIM3_PSH(n)        ((n) <<24)

/* LCD_MIXERx_TIM4 bits */
#define LCD_MIXERx_TIM4_TSH(n)        ((n)<<0)

/*LCD DSI Dx register offsets*/
#define LCD_DSI_DxSCR0_OFFSET		0x0000
#define LCD_DSI_DxCONV_GEN_NULL_BLANK_OFFSET	0x0034
#define LCD_DSI_DxCONV_DSI_RD1_OFFSET		0x003c
#define LCD_DSI_DxCONV_FIFO_THRESH_OFFSET	0x0040
#define LCD_DSI_DxCONV_FIFO_THRESH_INT_OFFSET	0x0044
#define LCD_DSI_DxSCR1_OFFSET          	0x0100
#define LCD_DSI_DxSSR_OFFSET		0x0104
#define LCD_DSI_DxTRIG_OFFSET          	0x0108
#define LCD_DSI_DxINEN_OFFSET		0x010C
#define LCD_DSI_DxINST1_OFFSET         	0x0110
#define LCD_DSI_DxTEINTCNT_OFFSET	0x0114
#define LCD_DSI_Dx_G_CTL_OFFSET		0x0014
#define LCD_DSI_Dx_G_DAT_RED_OFFSET	0x0018
#define LCD_DSI_Dx_G_DAT_GREEN_OFFSET  	0x001C
#define LCD_DSI_Dx_G_DAT_BLUE_OFFSET   	0x0020
#define LCD_DSI_DxPRSR_OFFSET          	0x0024
#define LCD_DSI_DxINST0_OFFSET		0x0028
#define LCD_DSI_DxADAT_OFFSET		0x0128
#define LCD_DSI_DxCFIF_OFFSET		0x002C
#define LCD_DSI_DxTIM0_OFFSET		0x0130
#define LCD_DSI_DxTIM1_OFFSET		0x0134
#define LCD_DSI_DxPHY_TIM0_OFFSET	0x013C
#define LCD_DSI_DxPHY_TIM1_OFFSET	0x0140
#define LCD_DSI_DxPHY_TIM2_OFFSET	0x0144
#define LCD_DSI_DxPHY_CAL_OFFSET	0x0148
#define LCD_DSI_DxVSYNC_JITT_TIM0	0x014C
#define LCD_DSI_DxHSYNC_JITT_TIM0	0x0150
#define LCD_DSI_DxVSYNC_JITT_TIM1	0x0154
#define LCD_DSI_DxHSYNC_JITT_TIM1	0x0158
#define LCD_DSI_DxVSYNC_JITT_TIM2	0x015C
#define LCD_DSI_DxHSYNC_JITT_TIM2	0x0160
#define LCD_DSI_DxVSYNC_JITT_TIM3       0x0164
#define LCD_DSI_DxHSYNC_JITT_TIM3       0x0168

/* LCD_DSI_DxSCR0 bits */
#define LCD_DSI_DxSCR0_DISP_URUN_INT_EN (0x1u<<0)
#define LCD_DSI_DxSCR0_DISP_EOF_INT_EN  (0x1u<<1)
#define LCD_DSI_DxSCR0_DISP_SOF_INT_EN  (0x1u<<2)
#define LCD_DSI_DxSCR0_SMART_INT_EN     (0x1u<<3)
#define LCD_DSI_DxSCR0_SP_RD_INT_EN     (0x1u<<4)
#define LCD_DSI_DxSCR0_SP_UNDRUN_INT_EN (0x1u<<5)
#define LCD_DSI_DxSCR0_SP_BREAK_INT_EN  (0x1u<<7)
#define LCD_DSI_DxSCR0_FLOW_DIS         (0x1u<<8)
#define LCD_DSI_DxSCR0_BREAK            (0x1u<<9)
#define LCD_DSI_DxSCR0_CONV_EN          (0x1u<<10)
#define LCD_DSI_DxSCR0_PRIM_VC          (0x1u<<11)
#define LCD_DSI_DxSCR0_SYNC_CNT(n)      ((n) <<20)

/* LCD_DSI_DxSCR1 bits */
#define LCD_DSI_DxSCR1_DSI_EN     (0x1u<<0)
#define LCD_DSI_DxSCR1_NOL_MASK   (0x3u<<2)
#define LCD_DSI_DxSCR1_NOL(n)     ((n) <<2)
#define LCD_DSI_DxSCR1_PT         (0x1u<<4)
#define LCD_DSI_DxSCR1_VC0(n)     ((n) <<6)
#define LCD_DSI_DxSCR1_VC1(n)     ((n) <<8)
#define LCD_DSI_DxSCR1_VC_EN      (0x1u<<10)
#define LCD_DSI_DxSCR1_VSYNC_MODE (0x1u<<12)
#define LCD_DSI_DxSCR1_POWER_MASK (0x119u<<23)
#define LCD_DSI_DxSCR1_RIHS(n)    ((n) <<23)
#define LCD_DSI_DxSCR1_TRIG       (0x1u<<24)
#define LCD_DSI_DxSCR1_ULPS(n)    ((n) <<26)
#define LCD_DSI_DxSCR1_LPDT(n)    ((n) <<27)
#define LCD_DSI_DxSCR1_BLANK_NULL_FRMT(n)    ((n) <<29)
#define LCD_DSI_DxSCR1_BLANK_NULL_FRMT_MASK    (3 <<29)
#define LCD_DSI_DxSCR1_BTA(n)     ((n) <<31)

/* LCD_DSI_DxSSR bits */
#define LCD_DSI_DxSSR_DSI_EN_STAT (0x1u<<0)
#define LCD_DSI_DxSSR_L0_PM       (0x1u<<24)
#define LCD_DSI_DxSSR_L1_PM       (0x1u<<25)
#define LCD_DSI_DxSSR_L2_PM       (0x1u<<26)
#define LCD_DSI_DxSSR_L3_PM       (0x1u<<27)
#define LCD_DSI_DxSSR_STOP_ST     (0x1u<<28)
#define LCD_DSI_DxSSR_STOP_ST_ALL (0x1u<<29)
#define LCD_DSI_DxSSR_DIR         (0x1u<<31)

/* LCD_DSI_DxINEN bits */
#define LCD_DSI_DxINEN_LP_CONT_EN  (0x1u << 24)
#define LCD_DSI_DxINEN_PORT_ERR_EN (0x1u << 16)
#define LCD_DSI_DxINEN_TIMEOUT_EN  (0x1u << 12)
#define LCD_DSI_DxINEN_ACK_ERR_EN  (0x1u << 8)
#define LCD_DSI_DxINEN_PHY_ERR_EN  (0x1u << 4)

/* LCD_DSI_DxCONV_FIFO_THRESH bits */
#define LCD_DSI_DxCONV_FIFO_THRESH_PIXEL_FIFO_SIZE(n)  ((n) << 21)
#define LCD_DSI_DxCONV_FIFO_THRESH_PIXEL_FIFO_THRESH(n) (n)

/* LCD_DSI_DxCONV_FIFO_THRESH_INT bits */
#define LCD_DSI_DxCONV_FIFO_THRESH_INT_THRESH_INT_EN  (0x1u << 31)
#define LCD_DSI_DxCONV_FIFO_THRESH_INT_THRESH_INT_ST  (0x1u << 15)

/* LCD_DSI_DxCONV_GEN_NULL_BLANK bits*/
#define DxCONV_GEN_NULL_BLANK_NULL_BLANK_DATA(n)   ((n) <<0)
#define DxCONV_GEN_NULL_BLANK_GEN_PXL_FORMAT(n)    ((n) <<26)
#define DxCONV_GEN_NULL_BLANK_GEN_PXL_FORMAT2(n)   ((n) <<28)
#define DxCONV_GEN_NULL_BLANK_GEN_PXL_FORMAT3(n)   ((n) <<30)
#define DxCONV_GEN_NULL_BLANK_NULL_BLANK_DATA_MASK (0xffffffu <<0)

/* LCD_DSI_DxTIM0 bits */
#define LCD_DSI_DxTIM0_HS_TX_TO(n)   ((n)<<0)
#define LCD_DSI_DxTIM0_LP_RX_TO(n)   ((n)<<16)

/* LCD_DSI_DxTIM1 bits */
#define LCD_DSI_DxTIM1_BTA_TO(n)    ((n)<<0)
#define LCD_DSI_DxTIM1_G_TIMER(n)   ((n)<<16)

/* LCD_DSI_DxPHY_TIM0 bits */
#define LCD_DSI_DxPHY_TIM0_HSTRAIL(n)   ((n)<<0)
#define LCD_DSI_DxPHY_TIM0_HSZERO(n)    ((n)<<8)
#define LCD_DSI_DxPHY_TIM0_HSPREP(n)    ((n)<<16)
#define LCD_DSI_DxPHY_TIM0_LPX(n)       ((n)<<24)

/* LCD_DSI_DxPHY_TIM1 bits */
#define LCD_DSI_DxPHY_TIM1_CLTRAIL(n)   ((n)<<0)
#define LCD_DSI_DxPHY_TIM1_CLZERO(n)    ((n)<<8)
#define LCD_DSI_DxPHY_TIM1_TAGO(n)      ((n)<<16)
#define LCD_DSI_DxPHY_TIM1_HSEXIT(n)    ((n)<<24)

/* LCD_DSI_DxPHY_TIM2 bits */
#define LCD_DSI_DxPHY_TIM2_WAKEUP(n)   ((n)<<0)
#define LCD_DSI_DxPHY_TIM2_TAGET(n)    ((n)<<16)
#define LCD_DSI_DxPHY_TIM2_REQ_RDY_DELAY(n) ((n)<<24)

/* LCD_CONVx_G_CTL bits */
#define LCD_CONVx_G_CTL_ADD_PTR_R(n)   ((n)<<0)
#define LCD_CONVx_G_CTL_ADD_PTR_G(n)   ((n)<<4)
#define LCD_CONVx_G_CTL_ADD_PTR_B(n)   ((n)<<8)
#define LCD_CONVx_G_CTL_Q4_C(n)        ((n)<<16)
#define LCD_CONVx_G_CTL_Q3_C(n)        ((n)<<18)
#define LCD_CONVx_G_CTL_Q2_C(n)        ((n)<<20)
#define LCD_CONVx_G_CTL_Q1_C(n)        ((n)<<22)
#define LCD_CONVx_G_CTL_G_VAL(n)       ((n)<<24)

/* LCD_DSI_COMMAND bits */
#define LCD_DSI_COMMAND_DATA(n)  ((n)<<0)
#define LCD_DSI_COMMAND_CMD(n)   ((n)<<8)
#define LCD_DSI_COMMAND_LOOP(n)  ((n)<<13)
#define LCD_DSI_COMMAND_VC(n)    ((n) <<14)

/* LCD_DSI_DxINST0 bits */
#define LCD_DSI_DxINST0_SP_GEN_INT_STS    (0x1u<<6)
#define LCD_DSI_DxINST0_BREAK_INT_STS     (0x1u<<7)

/* LCD_DSI_DxINST1 bits */
#define LCD_DSI_DxINST1_BTA_TO_ERR    (0x1u<<14)
#define LCD_DSI_DxINST1_TRIG_ST       (0x1u<<19)

typedef struct DisplayControllerFrameDescriptor {
	u32	LCD_NXT_DESC_ADDRx;   /* Next Descriptor Address Register */
	u32	LCD_FR_ADDRx;
	u32	LCD_FR_IDx;           /* Frame ID Register */
	u32	LCD_CH_CMDx;
} DisplayControllerFrameDescriptor;


/* ---------------------------------------------- */
/*             structures for HDMI converter      */
/* ---------------------------------------------- */
/*HDMI register offsets*/
#define HDMI_CONV_CTL			0x0000
#define HDMI_MIXER_TIM0			0x0004
#define HDMI_MIXER_TIM1			0x0008
#define HDMI_MIXER_TIM2			0x000c
#define HDMI_MIXER_TIM3			0x0010
#define HDMI_MIXER_TIM4			0x0030
#define HDMI_MIXER_G_CTL		0x0014
#define HDMI_MIXER_G_DAT_RED		0x0018
#define HDMI_MIXER_G_DAT_GREEN		0x001c
#define HDMI_MIXER_G_DAT_BLUE		0x0020
#define HDMI_CONV_INT_STS		0x0028
#define HDMI_CLK_DIV			0x0044
#define HDMI_CLRSPC_COEFF0		0x0048
#define HDMI_CLRSPC_COEFF1		0x004c
#define HDMI_CLRSPC_COEFF2		0x0050
#define HDMI_CLRSPC_COEFF3		0x0054
#define HDMI_CLRSPC_COEFF4		0x0058

/* Specific registers for Nevo HDMI converter */
/* HDMI_CONVx_CTL bits */
#define HDMI_CONVx_CTL_DISP_URUN_INT_EN (0x1u<<0)
#define HDMI_CONVx_CTL_DISP_EOF_INT_EN  (0x1u<<1)
#define HDMI_CONVx_CTL_DISP_SOF_INT_EN  (0x1u<<2)
#define HDMI_CONVx_CTL_HDMI_INT_EN      (0x1u<<4)
#define HDMI_CONVx_CTL_CEC_FIFO_INT_EN  (0x1u<<5)
#define HDMI_CONVx_CTL_CEC_INT_EN       (0x1u<<6)
#define HDMI_CONVx_CTL_HDMI_YUV_EN      (0x1u<<7)
#define HDMI_CONVx_CTL_INTERLACER_VSYNC_INT_EN  (0x1u<<8)
#define HDMI_CONVx_CTL_WRITE_INTERLACER_REG_EN  (0x1u<<9)
#define HDMI_CONVx_CTL_OP_FOR(n)        ((n) << 16)
#define HDMI_CONVx_CTL_DISP_TYPE(n)     ((n) << 30)

/*HDMI_MIXERx_TIM0 bits */
#define HDMI_MIXERx_TIM0_PCP           (0x1u<<0)
#define HDMI_MIXERx_TIM0_OEP           (0x1u<<1)
#define HDMI_MIXERx_TIM0_VSP           (0x1u<<2)
#define HDMI_MIXERx_TIM0_HSP           (0x1u<<3)
#define HDMI_MIXERx_TIM0_VDEP          (0x1u<<4)
#define HDMI_MIXERx_TIM0_VSYNC_DEL     (0x1u<<6)
#define HDMI_MIXERx_TIM0_HDEP          (0x1u<<7)

/*HDMI_MIXERx_TIM2 bits */
#define HDMI_MIXERx_TIM2_BLW(n)        ((n) << 0)
#define HDMI_MIXERx_TIM2_ELW(n)        ((n) << 16)

/*HDMI_MIXERx_TIM3 bits */
#define HDMI_MIXERx_TIM3_HSW(n)        ((n) << 0)
#define HDMI_MIXERx_TIM3_BFW(n)        ((n) << 16)

/*HDMI_MIXERx_TIM4 bits */
#define HDMI_MIXERx_TIM4_EFW(n)        ((n) << 0)
#define HDMI_MIXERx_TIM4_VSW(n)        ((n) << 16)

/*HDMI_CONVx_INT_STS bits */
#define HDMI_CONVx_INT_STS_DISP_URUN_INT_STS     (0x1u<<0)
#define HDMI_CONVx_INT_STS_DISP_EOF_INT_STS      (0x1u<<1)
#define HDMI_CONVx_INT_STS_DISP_SOF_INT_STS      (0x1u<<2)
#define HDMI_CONVx_INT_STS_DISP_HDMI_INT_STS     (0x1u<<4)
#define HDMI_CONVx_INT_STS_DISP_CEC_FIFO_INT_STS (0x1u<<5)
#define HDMI_CONVx_INT_STS_DISP_CEC_INT_STS      (0x1u<<7)
#define HDMI_CONVx_INT_STS_INTERLACER_VSYNC_STS  (0x1u<<8)
#define HDMI_CONVx_INT_STS_DISP_URUN_INT_STS     (0x1u<<0)

/*HDMI_CLK_DIV bits */
#define HDMI_CLK_DIV_PCLK_DIV(n)       ((n) << 0)
#define HDMI_CLK_DIV_CEC_REFCLK_DIV(n) ((n) << 8)
#define HDMI_CLK_DIV_PR_CLK_DIV(n)     ((n) << 16)

#define HDMI_PLL_DIV_VALUE 5

#endif /* __ASM_ARCH_REGS_LCD_PXA95x_H */
