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

#ifdef CONFIG_PXA688_DSI

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

/* dsi phy timing */
static struct dsi_phy phy = {
	.hs_prep_constant	= 60,    /* Unit: ns. */
	.hs_prep_ui		= 5,
	.hs_zero_constant	= 85,
	.hs_zero_ui		= 5,
	.hs_trail_constant	= 0,
	.hs_trail_ui		= 64,
	.hs_exit_constant	= 100,
	.hs_exit_ui		= 0,
	.ck_zero_constant	= 300,
	.ck_zero_ui		= 0,
	.ck_trail_constant	= 60,
	.ck_trail_ui		= 0,
	.req_ready		= 0x3c,
};

#define dsi_ex_pixel_cnt		0
#define dsi_hex_en			0
/* (Unit: Mhz) */
#define dsi_hsclk			(clk_get_rate(fbi->clk)/1000000)
#define dsi_lpclk			3

#define to_dsi_bcnt(timing, bpp)	(((timing) * (bpp)) >> 3)

static unsigned int dsi_lane[5] = {0, 0x1, 0x3, 0x7, 0xf};

void pxa168fb_dsi_send(struct pxa168fb_info *fbi, void *value)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_regs *dsi = (struct dsi_regs *)mi->dsi->regs;
	u8 *dsi_cmd_tmp = (u8 *)value;
	int count = (int)(*dsi_cmd_tmp);
	u8 *dsi_cmd = (u8*)&dsi_cmd_tmp[1];
	unsigned int firstTimeFlag = 1, firstPacket = 0;
	volatile int loop = 0, tmp = 0, bc =0, dsiAddr=0, wAddr=0;
	volatile unsigned int reg;

	pr_debug("count is %d\r\n",count);

	/* write all packet bytes to packet data buffer */
	for (loop = 0; loop < count; loop++) {
		tmp |= ((int)dsi_cmd[loop]) << (bc * 8);
		bc ++;

		pr_debug("bc is %d\r\n",bc);
		if(bc == 4) {
			/* XM: save 1st packet */
			if(firstTimeFlag) {
				firstTimeFlag = 0;
				firstPacket = tmp;
			}

			writel(tmp, &dsi->dat0);
			wAddr = 0xC0000000 | (dsiAddr << 16);
			writel(wAddr, &dsi->cmd3);

			/* while (readl(&dsi->cmd3) & 0x80000000)
				msleep(1); */
			pr_debug("total count is %d, wAddr is 0x%08x,"
				" data is 0x%08x\r\n", count, wAddr, tmp);
			tmp = 0; bc = 0; dsiAddr += 4;
		}
	}

	/* handle last none 4Byte align data */
	if (bc) {
		writel(tmp, &dsi->dat0);
		wAddr = 0xC0000000 | (dsiAddr << 16);
		writel(wAddr, &dsi->cmd3);

		/* while (readl(&dsi->cmd3) & 0x80000000)
			msleep(1); */
		pr_debug("last one total count is %d, wAddr is 0x%08x,"
			" data is 0x%08x\r\n", count, wAddr, tmp);
		tmp = 0;
	}

	/* send out the packet */
	tmp = 0xC8000000 | count; writel(tmp, &dsi->cmd0);
	pr_debug("write count is 0x%08x\r\n",tmp);

	/* XM: workaround for race condition before A2 stepping */
	if (cpu_is_mmp2_a0() || cpu_is_mmp2_a1() ||
		cpu_is_mmp2_z0() || cpu_is_mmp2_z1()) {
		unsigned int count = 0;

		for(count = 0; count < 10; count++) {
			/* reg@0xa8 is a register for debugging purpose; it is
			 * a reflection of the dsi logic and phy interface */
			reg = readl(fbi->dsi1_reg_base + 0xa8);

			if(firstPacket != reg) {
				/* adjust clk phase used for dsi buffer index so
				 * that dsi sends the right packet. The data are
				 * still retained in DSI buffer after sending */
				pr_debug("dsi LP racing condition w/a\r\n");
				count = 0;

				/* Adjust clk phase */
				sclk_div_set(fbi, 2); mdelay(4);
				sclk_div_set(fbi, 6);

				/* Reset dsi module */
				reg = readl(&dsi->ctrl0) | (0x1 << 31);
				writel(reg, &dsi->ctrl0); msleep(1);
				reg &= ~ (0x1 << 31);
				writel(reg, &dsi->ctrl0); msleep(1);

				/* send out the packet again */
				writel(tmp, &dsi->cmd0);
			} else
				break;
		}
	}

	/* wait for completion */
	while (readl(&dsi->cmd0) & 0x80000000)
		msleep(1);
}
#if 0 /* original version */
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_regs *dsi = (struct dsi_regs *)mi->dsi->regs;
	int loop = 0, tmp = 0;
	u8 *dsi_cmd = (u8 *)value;
	int count = (int)(*dsi_cmd);
	if (count != 0x4) {
		/* set up packet header for long packet */
		*(dsi_cmd + 1) = 0x29;
		*(dsi_cmd + 2) = (u8)(count - 6);
		*(dsi_cmd + 3) = 0;
	}
	/* write all packet bytes to packet data buffer */
	for (loop = 0; loop < count; loop++) {
		tmp |= ((int)*(dsi_cmd + loop + 1)) << ((loop % 4) * 8);
		if (!((loop + 1) % 4)) {
			writel(tmp, &dsi->dat0);
			writel(0xc0000000 | ((loop - 3) << 16), &dsi->cmd3);
			while (readl(&dsi->cmd3) & 0x80000000)
				msleep(1);
			tmp = 0;
		}
	}
	if (loop % 4) {
		writel(tmp, &dsi->dat0);
		writel(0xc0000000 | (4 * (loop / 4) << 16), &dsi->cmd3);
		while (readl(&dsi->cmd3) & 0x80000000)
			msleep(1);
		tmp = 0;
	}
	/* send out the packet */
	if (count == 0x4)
		tmp = 0xc0000000 | count;
	else
		tmp = 0x80000000 | (count - 6);
	writel(tmp, &dsi->cmd0);
	while (readl(&dsi->cmd0) & 0x80000000)
		msleep(1);
}
#endif

void dsi_cclk_set(struct pxa168fb_info *fbi, int en)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_regs *dsi = (struct dsi_regs *)mi->dsi->regs;

	if (en)
		writel(0x1, &dsi->phy_ctrl1);
	else
		writel(0x0, &dsi->phy_ctrl1);
	mdelay(100);
}

void dsi_set_dphy(struct pxa168fb_info *fbi)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_regs *dsi = (struct dsi_regs *)mi->dsi->regs;
	u32 ui, lpx_clk, lpx_time, ta_get, ta_go, wakeup, reg;
	u32 hs_prep, hs_zero, hs_trail, hs_exit, ck_zero, ck_trail, ck_exit;

	ui = 1000/dsi_hsclk + 1;
	lpx_clk = (DSI_ESC_CLK / dsi_lpclk / 2) - 1;
	lpx_time = (lpx_clk + 1) * DSI_ESC_CLK_T;
	/* Below is for NT35451 */
	ta_get = ((lpx_time * 10 + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;
	ta_go  = ((lpx_time * 4 + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;
	wakeup = 0xfff0;

	hs_prep = phy.hs_prep_constant + phy.hs_prep_ui * ui;
	hs_prep = ((hs_prep + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;

	/* Our hardware added 3-byte clk automatically.
	 * 3-byte 3 * 8 * ui.
	 */
	hs_zero = phy.hs_zero_constant + phy.hs_zero_ui * ui;
	if(hs_zero > (24 * ui))
		hs_zero -= (24 * ui);
	else
		hs_zero = DSI_ESC_CLK_T;

	if(hs_zero > (DSI_ESC_CLK_T * 2))
		hs_zero = ((hs_zero + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;
	else
		hs_zero = 1;

	hs_trail = phy.hs_trail_constant + phy.hs_trail_ui * ui;
	hs_trail = ((hs_trail + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;

	hs_exit = phy.hs_exit_constant + phy.hs_exit_ui * ui;
	hs_exit = ((hs_exit + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;

	ck_zero = phy.ck_zero_constant + phy.ck_zero_ui * ui;
	ck_zero = ((ck_zero + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;

	ck_trail = phy.ck_trail_constant + phy.ck_trail_ui * ui;
	ck_trail = ((ck_trail + (DSI_ESC_CLK_T >> 1)) / DSI_ESC_CLK_T) - 1;

	ck_exit = hs_exit;

	/* bandgap ref enable */
	reg = readl(&dsi->phy_rcomp0);
	reg |= (1<<9);
	writel(reg, &dsi->phy_rcomp0);

	/* timing_0 */
	reg = (hs_exit << DSI_PHY_TIME_0_CFG_CSR_TIME_HS_EXIT_SHIFT)
		| (hs_trail << DSI_PHY_TIME_0_CFG_CSR_TIME_HS_TRAIL_SHIFT)
		| (hs_zero << DSI_PHY_TIME_0_CDG_CSR_TIME_HS_ZERO_SHIFT)
		| (hs_prep);
	writel(reg, &dsi->phy_timing0);

	reg = (ta_get << DSI_PHY_TIME_1_CFG_CSR_TIME_TA_GET_SHIFT)
		| (ta_go << DSI_PHY_TIME_1_CFG_CSR_TIME_TA_GO_SHIFT)
		| wakeup;
	writel(reg, &dsi->phy_timing1);

	reg = (ck_exit << DSI_PHY_TIME_2_CFG_CSR_TIME_CK_EXIT_SHIFT)
		| (ck_trail << DSI_PHY_TIME_2_CFG_CSR_TIME_CK_TRAIL_SHIFT)
		| (ck_zero << DSI_PHY_TIME_2_CFG_CSR_TIME_CK_ZERO_SHIFT)
		| lpx_clk;
	writel(reg, &dsi->phy_timing2);


	reg = (lpx_clk << DSI_PHY_TIME_3_CFG_CSR_TIME_LPX_SHIFT) | \
	      phy.req_ready;
	writel(reg, &dsi->phy_timing3);

	/* calculated timing on brownstone:
	 * DSI_PHY_TIME_0 0x06080204
	 * DSI_PHY_TIME_1 0x6d2bfff0
	 * DSI_PHY_TIME_2 0x603130a
	 * DSI_PHY_TIME_3 0xa3c
	 */
}

void dsi_reset(struct pxa168fb_info *fbi, int hold)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_regs *dsi = (struct dsi_regs *)mi->dsi->regs;
	volatile unsigned int reg;

	printk(KERN_DEBUG "%s\n", __func__);
	writel(0x0, &dsi->ctrl0);
	reg = readl(&dsi->ctrl0);
	reg |= DSI_CTRL_0_CFG_SOFT_RST | DSI_CTRL_0_CFG_SOFT_RST_REG;

	if (!hold) {
		writel(reg, &dsi->ctrl0);
		reg &= ~(DSI_CTRL_0_CFG_SOFT_RST | DSI_CTRL_0_CFG_SOFT_RST_REG);
		mdelay(1);
	}
	writel(reg, &dsi->ctrl0);
}

void dsi_set_controller(struct pxa168fb_info *fbi)
{
	struct fb_var_screeninfo *var = &(fbi->fb_info->var);
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_regs *dsi = (struct dsi_regs *)mi->dsi->regs;
	struct dsi_lcd_regs *dsi_lcd = &dsi->lcd1;
	struct dsi_info *di = mi->dsi;
	unsigned hsync_b, hbp_b, hact_b, hex_b, hfp_b, httl_b;
	unsigned hsync, hbp, hact, hfp, httl, h_total, v_total;
	unsigned hsa_wc, hbp_wc, hact_wc, hex_wc, hfp_wc, hlp_wc;
	int bpp = di->bpp, hss_bcnt = 4, hse_bct = 4, lgp_over_head = 6, reg;

	if (di->id & 2)
		dsi_lcd = &dsi->lcd2;
	printk("%s dsi %d lanes %d burst_mode %d bpp %d\n",
		__func__, di->id, di->lanes, di->burst_mode, bpp);

	h_total = var->xres + var->left_margin + var->right_margin + var->hsync_len;
	v_total = var->yres + var->upper_margin + var->lower_margin + var->vsync_len;

	hact_b = to_dsi_bcnt(var->xres, bpp);
	hfp_b = to_dsi_bcnt(var->right_margin, bpp);
	hbp_b = to_dsi_bcnt(var->left_margin, bpp);
	hsync_b = to_dsi_bcnt(var->hsync_len, bpp);
	hex_b = to_dsi_bcnt(dsi_ex_pixel_cnt, bpp);
	httl_b = hact_b + hsync_b + hfp_b + hbp_b + hex_b;

	hact = hact_b / di->lanes;
	hfp = hfp_b / di->lanes;
	hbp = hbp_b / di->lanes;
	hsync = hsync_b / di->lanes;
	httl = hact + hfp + hbp + hsync;

	/* word count in the unit of byte */
	hsa_wc = (di->burst_mode == DSI_BURST_MODE_SYNC_PULSE) ? \
		(hsync_b - hss_bcnt - lgp_over_head) : 0;

	/* Hse is with backporch */
	hbp_wc = (di->burst_mode == DSI_BURST_MODE_SYNC_PULSE) ? \
		(hbp_b - hse_bct - lgp_over_head) \
		: (hsync_b + hbp_b - hss_bcnt - lgp_over_head);

	hfp_wc = ((di->burst_mode == DSI_BURST_MODE_BURST) && (dsi_hex_en == 0)) ? \
		(hfp_b + hex_b - lgp_over_head - lgp_over_head) : \
		(hfp_b - lgp_over_head - lgp_over_head);

	hact_wc =  ((var->xres) * bpp) >> 3;

	/* disable Hex currently */
	hex_wc = 0;

	/*  There is no hlp with active data segment.  */
	hlp_wc = (di->burst_mode == DSI_BURST_MODE_SYNC_PULSE) ? \
		(httl_b - hsync_b - hse_bct - lgp_over_head) : \
		(httl_b - hss_bcnt - lgp_over_head);

	/* FIXME - need to double check the (*3) is bytes_per_pixel from input data or output to panel */
	/* dsi_lane_enable - Set according to specified DSI lane count */
	writel(dsi_lane[di->lanes] << DSI_PHY_CTRL_2_CFG_CSR_LANE_EN_SHIFT, &dsi->phy_ctrl2);
	writel(dsi_lane[di->lanes] << DSI_CPU_CMD_1_CFG_TXLP_LPDT_SHIFT, &dsi->cmd1);
	if (mi->phy_type == DSI)
		mi->dsi_set(fbi);

	/* SET UP LCD1 TIMING REGISTERS FOR DSI BUS */
	/* NOTE: Some register values were obtained by trial and error */
	writel((hact << 16) | httl, &dsi_lcd->timing0);
	writel((hsync << 16) | hbp, &dsi_lcd->timing1);
	/*
	 * For now the active size is set really low (we'll use 10) to allow
	 * the hardware to attain V Sync. Once the DSI bus is up and running,
	 * the final value will be put in place for the active size (this is
	 * done below). In a later stepping of the processor this workaround
	 * will not be required.
	 */
	if (cpu_is_mmp2_z0() || cpu_is_mmp2_z1() || cpu_is_mmp3_z0())
		writel(((0xa) << 16) | (v_total), &dsi_lcd->timing2);
	else
		writel(((var->yres)<<16) | (v_total), &dsi_lcd->timing2);

	writel(((var->vsync_len) << 16) | (var->upper_margin), &dsi_lcd->timing3);

	/* SET UP LCD1 WORD COUNT REGISTERS FOR DSI BUS */
	/* Set up for word(byte) count register 0 */
	writel((hbp_wc << 16) | hsa_wc, &dsi_lcd->wc0);
	writel((hfp_wc << 16) | hact_wc, &dsi_lcd->wc1);
	writel((hex_wc << 16) | hlp_wc, &dsi_lcd->wc2);
	/* calculated value on brownstone:
	 * WC0: 0x1a0000
	 * WC1: 0x1500f00
	 * WC2: 0x1076 */

	/* Configure LCD control register 1 FOR DSI BUS */
	reg = ((di->rgb_mode << DSI_LCD2_CTRL_1_CFG_L1_RGB_TYPE_SHIFT)
		| (di->burst_mode << DSI_LCD1_CTRL_1_CFG_L1_BURST_MODE_SHIFT)
		| (di->lpm_line_en ? DSI_LCD1_CTRL_1_CFG_L1_LPM_LINE_EN : 0)
		| (di->lpm_frame_en ? DSI_LCD1_CTRL_1_CFG_L1_LPM_FRAME_EN : 0)
		| (di->last_line_turn ? DSI_LCD1_CTRL_1_CFG_L1_LAST_LINE_TURN :0)
		| (di->hex_slot_en ? 0 :0)   //disable Hex slot;
		| (di->all_slot_en ? 0 :0)   //disable all slots;
		| (di->hbp_en ? DSI_LCD1_CTRL_1_CFG_L1_HBP_PKT_EN :0)
		| (di->hact_en ? DSI_LCD1_CTRL_1_CFG_L1_HACT_PKT_EN :0)
		| (di->hfp_en ? DSI_LCD1_CTRL_1_CFG_L1_HFP_PKT_EN : 0)
		| (di->hex_en ? 0 : 0)      // Hex packet is disabled
		| (di->hlp_en ? DSI_LCD1_CTRL_1_CFG_L1_HLP_PKT_EN : 0));

	reg |= (di->burst_mode == DSI_BURST_MODE_SYNC_PULSE) ? \
		(((di->hsa_en) ? DSI_LCD1_CTRL_1_CFG_L1_HSA_PKT_EN :0)
		| (DSI_LCD1_CTRL_1_CFG_L1_HSE_PKT_EN ))  // Hse is always eabled;
		:
		(((di->hsa_en) ? 0 :0)   // Hsa packet is disabled;
		| ((di->hse_en) ? 0 :0));     // Hse packet is disabled;

	reg |=  DSI_LCD1_CTRL_1_CFG_L1_VSYNC_RST_EN;
	writel(reg, &dsi_lcd->ctrl1);

	/*Start the transfer of LCD data over the DSI bus*/
	/* DSI_CTRL_1 */
	reg = readl(&dsi->ctrl1);
	reg &= ~(DSI_CTRL_1_CFG_LCD2_VCH_NO_MASK | DSI_CTRL_1_CFG_LCD1_VCH_NO_MASK);
	reg |= 0x1 << ((di->id & 1) ? DSI_CTRL_1_CFG_LCD2_VCH_NO_SHIFT : DSI_CTRL_1_CFG_LCD1_VCH_NO_SHIFT);

	reg &= ~(DSI_CTRL_1_CFG_EOTP);
	if (di->eotp_en)
		reg |= DSI_CTRL_1_CFG_EOTP;	/* EOTP */

	writel(reg, &dsi->ctrl1);

	/* DSI_CTRL_0 */
	reg = DSI_CTRL_0_CFG_LCD1_SLV | DSI_CTRL_0_CFG_LCD1_TX_EN | DSI_CTRL_0_CFG_LCD1_EN;
	if (di->id & 2)
		reg = reg << 1;
	writel(reg, &dsi->ctrl0);
	mdelay(100);

	/* FIXME - second part of the workaround */
	if (cpu_is_mmp2_z0() || cpu_is_mmp2_z1() || cpu_is_mmp3_z0())
		writel(((var->yres - 2)<<16) | (v_total), &dsi_lcd->timing2);
	else
		writel(((var->yres)<<16) | (v_total), &dsi_lcd->timing2);
}

ssize_t dsi_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct pxa168fb_info *fbi = dev_get_drvdata(dev);
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct dsi_regs *dsi;

	if (!mi || !mi->dsi) {
		printk("no dsi interface available\n");
		goto out;
	}
	dsi = (struct dsi_regs *)mi->dsi->regs;

	printk("dsi regs base 0x%p\n", dsi);
	printk("\tctrl0       (@%3x):\t0x%x\n", (int)(&dsi->ctrl0         )&0xfff, readl(&dsi->ctrl0      ) );
	printk("\tctrl1       (@%3x):\t0x%x\n", (int)(&dsi->ctrl1         )&0xfff, readl(&dsi->ctrl1      ) );
	printk("\tirq_status  (@%3x):\t0x%x\n", (int)(&dsi->irq_status    )&0xfff, readl(&dsi->irq_status ) );
	printk("\tirq_mask    (@%3x):\t0x%x\n", (int)(&dsi->irq_mask      )&0xfff, readl(&dsi->irq_mask   ) );
	printk("\tcmd0        (@%3x):\t0x%x\n", (int)(&dsi->cmd0          )&0xfff, readl(&dsi->cmd0       ) );
	printk("\tcmd1        (@%3x):\t0x%x\n", (int)(&dsi->cmd1          )&0xfff, readl(&dsi->cmd1       ) );
	printk("\tcmd2        (@%3x):\t0x%x\n", (int)(&dsi->cmd2          )&0xfff, readl(&dsi->cmd2       ) );
	printk("\tcmd3        (@%3x):\t0x%x\n", (int)(&dsi->cmd3          )&0xfff, readl(&dsi->cmd3       ) );
	printk("\tdat0        (@%3x):\t0x%x\n", (int)(&dsi->dat0          )&0xfff, readl(&dsi->dat0       ) );
	printk("\tsmt_cmd     (@%3x):\t0x%x\n", (int)(&dsi->smt_cmd       )&0xfff, readl(&dsi->smt_cmd    ) );
	printk("\tsmt_ctrl0   (@%3x):\t0x%x\n", (int)(&dsi->smt_ctrl0     )&0xfff, readl(&dsi->smt_ctrl0  ) );
	printk("\tsmt_ctrl1   (@%3x):\t0x%x\n", (int)(&dsi->smt_ctrl1     )&0xfff, readl(&dsi->smt_ctrl1  ) );
	printk("\trx0_status  (@%3x):\t0x%x\n", (int)(&dsi->rx0_status    )&0xfff, readl(&dsi->rx0_status ) );
	printk("\trx0_header  (@%3x):\t0x%x\n", (int)(&dsi->rx0_header    )&0xfff, readl(&dsi->rx0_header ) );
	printk("\trx1_status  (@%3x):\t0x%x\n", (int)(&dsi->rx1_status    )&0xfff, readl(&dsi->rx1_status ) );
	printk("\trx1_header  (@%3x):\t0x%x\n", (int)(&dsi->rx1_header    )&0xfff, readl(&dsi->rx1_header ) );
	printk("\trx_ctrl     (@%3x):\t0x%x\n", (int)(&dsi->rx_ctrl       )&0xfff, readl(&dsi->rx_ctrl    ) );
	printk("\trx_ctrl1    (@%3x):\t0x%x\n", (int)(&dsi->rx_ctrl1      )&0xfff, readl(&dsi->rx_ctrl1   ) );
	printk("\trx2_status  (@%3x):\t0x%x\n", (int)(&dsi->rx2_status    )&0xfff, readl(&dsi->rx2_status ) );
	printk("\trx2_header  (@%3x):\t0x%x\n", (int)(&dsi->rx2_header    )&0xfff, readl(&dsi->rx2_header ) );
	printk("\tphy_ctrl1   (@%3x):\t0x%x\n", (int)(&dsi->phy_ctrl1     )&0xfff, readl(&dsi->phy_ctrl1  ) );
	printk("\tphy_ctrl2   (@%3x):\t0x%x\n", (int)(&dsi->phy_ctrl2     )&0xfff, readl(&dsi->phy_ctrl2  ) );
	printk("\tphy_ctrl3   (@%3x):\t0x%x\n", (int)(&dsi->phy_ctrl3     )&0xfff, readl(&dsi->phy_ctrl3  ) );
	printk("\tphy_status0 (@%3x):\t0x%x\n", (int)(&dsi->phy_status0   )&0xfff, readl(&dsi->phy_status0) );
	printk("\tphy_rcomp0  (@%3x):\t0x%x\n", (int)(&dsi->phy_rcomp0    )&0xfff, readl(&dsi->phy_rcomp0 ) );
	printk("\tphy_timing0 (@%3x):\t0x%x\n", (int)(&dsi->phy_timing0   )&0xfff, readl(&dsi->phy_timing0) );
	printk("\tphy_timing1 (@%3x):\t0x%x\n", (int)(&dsi->phy_timing1   )&0xfff, readl(&dsi->phy_timing1) );
	printk("\tphy_timing2 (@%3x):\t0x%x\n", (int)(&dsi->phy_timing2   )&0xfff, readl(&dsi->phy_timing2) );
	printk("\tphy_timing3 (@%3x):\t0x%x\n", (int)(&dsi->phy_timing3   )&0xfff, readl(&dsi->phy_timing3) );
	printk("\tphy_timing4 (@%3x):\t0x%x\n", (int)(&dsi->phy_timing4   )&0xfff, readl(&dsi->phy_timing4) );
	printk("\tphy_timing5 (@%3x):\t0x%x\n", (int)(&dsi->phy_timing5   )&0xfff, readl(&dsi->phy_timing5) );
	printk("\tmem_ctrl    (@%3x):\t0x%x\n", (int)(&dsi->mem_ctrl      )&0xfff, readl(&dsi->mem_ctrl   ) );
	printk("\ttx_timer    (@%3x):\t0x%x\n", (int)(&dsi->tx_timer      )&0xfff, readl(&dsi->tx_timer   ) );
	printk("\trx_timer    (@%3x):\t0x%x\n", (int)(&dsi->rx_timer      )&0xfff, readl(&dsi->rx_timer   ) );
	printk("\tturn_timer  (@%3x):\t0x%x\n", (int)(&dsi->turn_timer    )&0xfff, readl(&dsi->turn_timer ) );

	printk("\nlcd1 regs\n");
	printk("\tctrl0       (@%3x):\t0x%x\n", (int)(&dsi->lcd1.ctrl0    )&0xfff, readl(&dsi->lcd1.ctrl0    ) );
	printk("\tctrl1       (@%3x):\t0x%x\n", (int)(&dsi->lcd1.ctrl1    )&0xfff, readl(&dsi->lcd1.ctrl1    ) );
	printk("\ttiming0     (@%3x):\t0x%x\n", (int)(&dsi->lcd1.timing0  )&0xfff, readl(&dsi->lcd1.timing0  ) );
	printk("\ttiming1     (@%3x):\t0x%x\n", (int)(&dsi->lcd1.timing1  )&0xfff, readl(&dsi->lcd1.timing1  ) );
	printk("\ttiming2     (@%3x):\t0x%x\n", (int)(&dsi->lcd1.timing2  )&0xfff, readl(&dsi->lcd1.timing2  ) );
	printk("\ttiming3     (@%3x):\t0x%x\n", (int)(&dsi->lcd1.timing3  )&0xfff, readl(&dsi->lcd1.timing3  ) );
	printk("\twc0         (@%3x):\t0x%x\n", (int)(&dsi->lcd1.wc0      )&0xfff, readl(&dsi->lcd1.wc0      ) );
	printk("\twc1         (@%3x):\t0x%x\n", (int)(&dsi->lcd1.wc1      )&0xfff, readl(&dsi->lcd1.wc1      ) );
	printk("\twc2         (@%3x):\t0x%x\n", (int)(&dsi->lcd1.wc2      )&0xfff, readl(&dsi->lcd1.wc2      ) );
	printk("\tslot_cnt0   (@%3x):\t0x%x\n", (int)(&dsi->lcd1.slot_cnt0)&0xfff, readl(&dsi->lcd1.slot_cnt0) );
	printk("\tslot_cnt1   (@%3x):\t0x%x\n", (int)(&dsi->lcd1.slot_cnt1)&0xfff, readl(&dsi->lcd1.slot_cnt1) );

	printk("\nlcd2 regs\n");
	printk("\tctrl0       (@%3x):\t0x%x\n", (int)(&dsi->lcd2.ctrl0    )&0xfff, readl(&dsi->lcd2.ctrl0    ) );
	printk("\tctrl1       (@%3x):\t0x%x\n", (int)(&dsi->lcd2.ctrl1    )&0xfff, readl(&dsi->lcd2.ctrl1    ) );
	printk("\ttiming0     (@%3x):\t0x%x\n", (int)(&dsi->lcd2.timing0  )&0xfff, readl(&dsi->lcd2.timing0  ) );
	printk("\ttiming1     (@%3x):\t0x%x\n", (int)(&dsi->lcd2.timing1  )&0xfff, readl(&dsi->lcd2.timing1  ) );
	printk("\ttiming2     (@%3x):\t0x%x\n", (int)(&dsi->lcd2.timing2  )&0xfff, readl(&dsi->lcd2.timing2  ) );
	printk("\ttiming3     (@%3x):\t0x%x\n", (int)(&dsi->lcd2.timing3  )&0xfff, readl(&dsi->lcd2.timing3  ) );
	printk("\twc0         (@%3x):\t0x%x\n", (int)(&dsi->lcd2.wc0      )&0xfff, readl(&dsi->lcd2.wc0      ) );
	printk("\twc1         (@%3x):\t0x%x\n", (int)(&dsi->lcd2.wc1      )&0xfff, readl(&dsi->lcd2.wc1      ) );
	printk("\twc2         (@%3x):\t0x%x\n", (int)(&dsi->lcd2.wc2      )&0xfff, readl(&dsi->lcd2.wc2      ) );
	printk("\tslot_cnt0   (@%3x):\t0x%x\n", (int)(&dsi->lcd2.slot_cnt0)&0xfff, readl(&dsi->lcd2.slot_cnt0) );
	printk("\tslot_cnt1   (@%3x):\t0x%x\n\n", (int)(&dsi->lcd2.slot_cnt1)&0xfff, readl(&dsi->lcd2.slot_cnt1) );

out:
	return sprintf(buf, "%d\n", fbi->id);
}
ssize_t dsi_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	printk("%s\n", __func__);

	return size;
}
DEVICE_ATTR(dsi, S_IRUGO | S_IWUSR, dsi_show, dsi_store);

#endif
