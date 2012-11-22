 /*
  * linux/driver/media/video/pxa970_hdmitx.c
  *
  *  Copyright (C) 2011 Marvell International Ltd.
  *  AUTHOR:  O. Baron
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>

#include <mach/pxa970_hdmitx.h>

/* Assign outside functions and definitions */

/*If set, internal logic is used to detect video format.
 * Else the programmed values are used.*/
static volatile struct hdtx_registers *hdtx_regs;

static inline void set_int_frm_sel(int set_bit)
{
	if (set_bit)
		hdtx_regs->video_ctrl |= HDMITX_VIDEO_CTRL_INT_FRM_SEL;
	else
		hdtx_regs->video_ctrl &= ~HDMITX_VIDEO_CTRL_INT_FRM_SEL;

	return;
}

static inline int get_int_frm_sel(void)
{
	return ((hdtx_regs->video_ctrl & HDMITX_VIDEO_CTRL_INT_FRM_SEL)
		== HDMITX_VIDEO_CTRL_INT_FRM_SEL);
}

/*If set, input video is in YC.*/
static inline void set_in_yc(int set_bit)
{
	if (set_bit)
		hdtx_regs->video_ctrl |= HDMITX_VIDEO_CTRL_IN_YC;
	else
		hdtx_regs->video_ctrl &= ~HDMITX_VIDEO_CTRL_IN_YC;

	return;
}

static inline int get_in_yc(void)
{
	return ((hdtx_regs->video_ctrl & HDMITX_VIDEO_CTRL_IN_YC)
		== HDMITX_VIDEO_CTRL_IN_YC);
}

static void get_hw_computed_timing_params(
		struct hdtx_timing_para *para)
{
	/* Assemble the timing parameters */
	para->htot =
		(u16)(hdtx_regs->htot_sts_l | (hdtx_regs->htot_sts_h << 8))
		+ 1;

	para->hblank =
		(u16)(hdtx_regs->hblank_sts_l | (hdtx_regs->hblank_sts_h << 8))
		+ 1;

	para->vtot =
		(u16)(hdtx_regs->vtot_sts_l | (hdtx_regs->vtot_sts_h << 8))
		+ 1;

	para->vres =
		(u16)(hdtx_regs->vres_sts_l | (hdtx_regs->vres_sts_h << 8));

	para->vstart =
		(u16)(hdtx_regs->vstart_sts_l | (hdtx_regs->vstart_sts_h << 8))
		+ 1;

	return;
}

static void __attribute__ ((unused))
	set_sw_programmed_timing_params(u16 hblank, u16 vstart)
{
	hdtx_regs->hblank_l = (u8) (hblank & 0xFF);
	hdtx_regs->hblank_h = (u8) ((hblank >> 8) & 0xFF);

	hdtx_regs->vstart_l = (u8) (vstart & 0xFF);
	hdtx_regs->vstart_h = (u8) ((vstart >> 8) & 0xFF);

	return;
}

static inline void set_pixel_repetition(u8 pix_rep)
{
	hdtx_regs->hdmi_ctrl |= HDMITX_HDMI_CTRL_PIX_RPT(pix_rep);

	return;
}

static inline u8 get_pixel_repetition(void)
{
	u8 tmp_pix_rep = 0;

	tmp_pix_rep = (hdtx_regs->hdmi_ctrl & HDMITX_HDMI_CTRL_PIX_RPT_MASK)
		>> HDMITX_HDMI_CTRL_PIX_RPT_OFFSET;

	return tmp_pix_rep;
}

static u16 __attribute__ ((unused))
	calc_hblank(u16 htotal, u16 hres, u8 pixel_repetition_factor)
{
	u16 tmp_hblank = 0;

	tmp_hblank = htotal - hres;

	tmp_hblank = tmp_hblank * (pixel_repetition_factor + 1);

	tmp_hblank = tmp_hblank - 1;

	return tmp_hblank;
}

static u16 __attribute__ ((unused))
	calc_vstart(u16 vtotal, u16 vres, int interlaced)
{
	u16 tmp_vstart = 0;

	/* Calculate vstart according to inputs */
	tmp_vstart = vtotal - vres;

	/* If interlaced, divide by 2 */
	if (interlaced)
		tmp_vstart = tmp_vstart / 2;

	return tmp_vstart;
}

/* Return true if video frame format has been learnt by the hardware. */
static inline int is_video_frame_ready(void)
{
	return hdtx_regs->video_sts & HDMITX_VIDEO_STS_INIT_OVER;
}

/*This bit should be set for HDMI mode
 * only after init_over status is found to be set.*/
static inline void set_mode(int hdmi_mode)
{
	if (hdmi_mode)
		hdtx_regs->hdmi_ctrl |= HDMITX_HDMI_CTRL_HDMI_MODE;
	else
		hdtx_regs->hdmi_ctrl &= ~HDMITX_HDMI_CTRL_HDMI_MODE;
}

/*: If enabled then Audio/Video mute is enabled.*/
static void __attribute__ ((unused))
	mute_video_audio(int mute_audio, int mute_video)
{
	/* mute according to input */
	if (mute_audio)
		hdtx_regs->avmute_ctrl |= HDMITX_AVMUTE_CTRL_REG_AUDIO;
	else
		hdtx_regs->avmute_ctrl &= ~HDMITX_AVMUTE_CTRL_REG_AUDIO;

	/* mute according to input */
	if (mute_video)
		hdtx_regs->avmute_ctrl |= HDMITX_AVMUTE_CTRL_REG_VIDEO;
	else
		hdtx_regs->avmute_ctrl &= ~HDMITX_AVMUTE_CTRL_REG_VIDEO;
}

static void __attribute__ ((unused)) print_regs(void)
{
	u8 *iterator = (u8 *)hdtx_regs;
	u8 i, j;
	j = 0;

	for (i = 0; i * 8 < 0x400; i++) {
		if (i == 0)
			pr_info(" 0x0");
		else {
			if (i * 8 <= 0xF)
				pr_info(" ");
			pr_info("%d", i * 8);
		}
		pr_info(": ");

		for (j = 0; j < 8; j++) {
			if (*iterator > 0 && *iterator <= 0xF)
				pr_info(" ");
			pr_info("%d", *iterator++);
			pr_info(" ");
		}
		pr_info("\n");
	}
}

static void write_pkt0_byte0_30(u32 pkt_idx, u8 value, int byteNumber)
{
	u8 *iterator = (u8 *)hdtx_regs;

	switch (pkt_idx) {
	case 0:
		iterator += (0x60 + byteNumber);
		break;
	case 1:
		iterator += (0x80 + byteNumber);
		break;
	case 2:
		iterator += (0xA0 + byteNumber);
		break;
	case 3:
		iterator += (0xC0 + byteNumber);
		break;
	case 4:
		iterator += (0xE0 + byteNumber);
		break;
	case 5:
		iterator += (0x100 + byteNumber);
		break;
	default:
		pr_info("Unknown pkt type!\n\r");
	}

	*iterator = value;
}

static void send_packet(u32 pkt_idx, u8 *pDataBuf, u8 length)
{
	u8 i;
	u8 *pBase = (u8 *)hdtx_regs;
	u8 *host_pkt_ctrl0 = (u8 *) (pBase + 0x5E);
	u8 *host_pkt_ctrl1 = (u8 *) (pBase + 0x5F);

	for (i = 0; i < length; i++)
		write_pkt0_byte0_30(pkt_idx, pDataBuf[i], i);

	/* write_pkt0_byte0_30(pkt_idx,0x1,i); */

	/* transmit pkt_idx */
	*host_pkt_ctrl1 |= (0x1 << pkt_idx);
	*host_pkt_ctrl0 |= (0x1 << pkt_idx);
}

static u8 CheckSum(u8 *pDataBuf, u8 length)
{
	u8 chkSum = 0;
	u8 count;

	if (pDataBuf == NULL || length == 0)
		return 0;

	for (count = 0; count < length; count++)
		chkSum += pDataBuf[count];

	return 0x100 - chkSum;
}

static void send_info_frame(u8 format)
{
	u8 buf[14];
	u8 *pBase = (u8 *)hdtx_regs;
	u8 *write_tdata3_0 = (u8 *) (pBase + 0x58);
	u8 *write_tdata3_1 = (u8 *) (pBase + 0x59);
	u8 *write_tdata3_2 = (u8 *) (pBase + 0x5A);

	buf[0] = 0x82;
	buf[1] = 0x2;
	buf[2] = 0xd;
	buf[3] = 0x0;		/* checksum : 0xb3 */
	buf[4] = 0x10;		/* RGB, Active Format, no bar, no scan */
	buf[6] = 0x0;		/* No IT content, xvYCC601, no scaling */
	buf[7] = format;
	buf[8] = 0x0;		/* limited YCC range, graphics, no repeat */
	buf[9] = 0x0;
	buf[10] = 0x0;
	buf[11] = 0x0;
	buf[12] = 0x0;
	buf[13] = 0x0;

	/* packet header for AVI Packet in pkt1 */
	switch (format) {
	case 1:
	case 2:
	case 3:
		buf[5] = 0x98;	/* ITU-R 709, 4:3 */
		break;

	case 4:
	case 5:
	case 10:
	case 11:
	case 16:		/*1920x1080p - format 16 */
		buf[5] = 0xa8;	/* ITU-R 709, 16:9 */
		break;

	default:
		buf[5] = 0xa8;	/* ITU-R 709, 4:3 */
		break;
	}

	buf[3] = CheckSum(buf, sizeof(buf));
	send_packet(0, buf, sizeof(buf));

	/* Audio frame: not enabled yet
	buf[0] = 0x84;
	buf[1] = 0x1;
	buf[2] = 0xa;
	buf[3] = 0x0;
	buf[4] = 0x0;
	buf[5] = 0x0;
	buf[6] = 0x0;
	buf[7] = 0x0;
	buf[8] = 0x0;
	buf[9] = 0x0;
	buf[10] = 0x0;
	buf[11] = 0x0;
	buf[12] = 0x0;
	buf[13] = 0x0;

	buf[3] = CheckSum(buf, sizeof(buf));
	send_packet(0,buf, sizeof(buf));
	*/

	/* Data to be trasmitted on the Clock Channel 3 */
	*write_tdata3_0 = 0xe0;
	*write_tdata3_1 = 0x83;
	*write_tdata3_2 = 0xf;
}

static void hdtx_cfg(void)
{
	u8 *pBase = (u8 *)hdtx_regs;
	u8 *write_phy_fifo_ptrs = (u8 *)(pBase + 0x131);
	u8 *write_phy_fifo_soft_rst = (u8 *)(pBase + 0x130);

	pr_info("hdtx_cfg");

	/* HDMI_CTRL, enable HDMI mode */
	/* Should be set only after <initover>
	 * is set in video status register */
	hdtx_regs->hdmi_ctrl = 0x1;

	/* Deep color FIFO write pointer value upon soft reset
	 * using the Deep Color FIFO Soft Reset Register.*/
	hdtx_regs->dc_fifo_wr_ptr = 0x1;
	/* Deep color FIFO read pointer value upon soft reset
	 * using the Deep Color FIFO Soft Reset Register.*/
	hdtx_regs->dc_fifo_rd_ptr = 0x1A;
	/* Reset Deep Color Fifo */
	hdtx_regs->dc_fifo_sft_rst = 0x1;
	/* Release reset */
	hdtx_regs->dc_fifo_sft_rst = 0x0;

	/* Read/Write Pointers */
	*write_phy_fifo_ptrs = 0x08;
	/* Reset Phy FIFO */
	*write_phy_fifo_soft_rst = 0x1;
	/* Release */
	*write_phy_fifo_soft_rst = 0x0;
}

static void __attribute__ ((unused)) hdtx_audio_cfg(void)
{
	volatile struct ubits_registers *ubits =
		(struct ubits_registers *)
			((u32)hdtx_regs + HDMITX_UBITS_OFFSET);

	pr_info("hdtx_audio_cfg\n");

	hdtx_regs->acr_n0 = 0x0;
	hdtx_regs->acr_n1 = 0x18;
	hdtx_regs->acr_n2 = 0x0;

	hdtx_regs->acr_cts0 = 0x0;
	hdtx_regs->acr_cts1 = 0x0;
	hdtx_regs->acr_cts2 = 0x0;

	hdtx_regs->acr_ctrl = 0x1;

	hdtx_regs->mem_size_l = 0xFF;
	hdtx_regs->mem_size_h = 0x1;

	hdtx_regs->i2s_dlen = 0x10;

	hdtx_regs->aud_ctrl = 0x1F;

	hdtx_regs->gcp_cfg0 = 0x1;
	hdtx_regs->gcp_cfg1 = 0x43;

	hdtx_regs->i2s_dbg_lft0 = 0x12;
	hdtx_regs->i2s_dbg_lft1 = 0x34;
	hdtx_regs->i2s_dbg_lft2 = 0x56;
	hdtx_regs->i2s_dbg_lft3 = 0x78;

	hdtx_regs->i2s_dbg_rit0 = 0x12;
	hdtx_regs->i2s_dbg_rit1 = 0x34;
	hdtx_regs->i2s_dbg_rit2 = 0x56;
	hdtx_regs->i2s_dbg_rit3 = 0x78;

	hdtx_regs->chsts_0 = 0x12;
	hdtx_regs->chsts_1 = 0x34;
	hdtx_regs->chsts_2 = 0x56;
	hdtx_regs->chsts_3 = 0x78;
	hdtx_regs->chsts_4 = 0x9a;

	hdtx_regs->chsts_0 = 0x12;
	hdtx_regs->chsts_1 = 0x34;
	hdtx_regs->chsts_2 = 0x56;
	hdtx_regs->chsts_3 = 0x78;
	hdtx_regs->chsts_4 = 0x9a;

	ubits->ubits_0 = 0x1;
	ubits->ubits_1 = 0x2;
	ubits->ubits_2 = 0x3;
	ubits->ubits_3 = 0x4;
	ubits->ubits_4 = 0x5;
	ubits->ubits_5 = 0x6;
	ubits->ubits_6 = 0x7;
	ubits->ubits_7 = 0x8;
	ubits->ubits_8 = 0x9;
	ubits->ubits_9 = 0x10;
	ubits->ubits_10 = 0x11;
	ubits->ubits_11 = 0x12;
	ubits->ubits_12 = 0x13;
	ubits->ubits_13 = 0x14;

	/* ubits->hbr_pkt = 0x01;
	hdtx_regs->aud_ctrl = 0xc3; */

}

static void __attribute__ ((unused))
	print_timings(struct hdtx_timing_para *timing)
{
	pr_info("hblank:%08X\n", timing->hblank);
	pr_info("htot:%08X\n", timing->htot);
	pr_info("vres:%08X\n", timing->vres);
	pr_info("vstart:%08X\n", timing->vstart);
	pr_info("vtot:%08X\n", timing->vtot);
}

static int init(int hdmi_format)
{
	int to;
	struct hdtx_timing_para timing;

	set_int_frm_sel(1);

	send_info_frame(hdmi_format);

	/* Wait until init_over is set */
	to = 30;
	while (is_video_frame_ready() == 0 && --to)
		;

	if (!to) {
		get_hw_computed_timing_params(&timing);
		/* print_timings(&timing);*/
		/* print_regs(); */
		return 0;
	}
	pr_info("HDMI tx header frame has been sent.\n");

	hdtx_cfg();
	/* hdtx_audio_cfg(); */
	to = 30;
	while (is_video_frame_ready() == 0 && --to)
		;

	if (!to) {
		get_hw_computed_timing_params(&timing);
		/* print_timings(&timing);*/
		/* print_regs(); */
		return 0;
	}
	pr_info("HDMI tx regs set ok.\n");

	/*
	get_hw_computed_timing_params(&timing);
	print_hdmi_timings(&timing);
	print_regs();
	*/

	set_mode(1);

	return 0;
}

static int hdmitx_remove(struct platform_device *pdev)
{
	if (hdtx_regs)
		iounmap(hdtx_regs);

	return 0;
}

static int hdmitx_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct hdtx_plat_data *data = pdev->dev.platform_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		goto failed;
	}

	hdtx_regs = ioremap_nocache(res->start, res->end - res->start);
	if (!hdtx_regs) {
		dev_err(&pdev->dev, "remap registers failed\n");
		goto failed;
	}

	if (data->boot_en)
		init(data->format);

	return 0;
failed:
	hdmitx_remove(pdev);
	platform_set_drvdata(pdev, NULL);
	return -ENOMEM;
}


static struct platform_driver hdmitx_driver = {
	.probe		= hdmitx_probe,
	.remove		= hdmitx_remove,
	.driver = {
		.name	= "pxa970-ihdmi",
		.owner	= THIS_MODULE,
	},
};

static int __init hdmitx_init(void)
{
	int ret = 0;
	/*2 --480p, 17 -- 576p, refer to global_hdmi_frame[] */
	/*ret = pxa970_hdmitx_init(4); */
	ret = platform_driver_register(&hdmitx_driver);
	return ret;
}

late_initcall(hdmitx_init);

MODULE_AUTHOR("Hezi Shahmoon");
MODULE_DESCRIPTION("Internal HDMI driver for PXA970");
MODULE_LICENSE("GPL");
