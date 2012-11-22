/*******************************************************************
 *    Copyright 2009 Marvell Corporation
 *
 *    DESCRIPTION:
 *
 *    AUTHOR:  O. Baron
 *
 *    Date Created:  11/05/2010 5:28PM
 *
 *    FILENAME: hdmitx.h
 *
 *    HISTORY:
 *
 *
 *
 *******************************************************************/

#ifndef HDMITX_H
#define HDMITX_H

/* DEFENITIONS */
#define HDMITX_UBITS_OFFSET 0x120

struct hdtx_registers {
	u8 acr_n0;		/* 0  */
	u8 acr_n1;
	u8 acr_n2;
	u8 reserved0;
	u8 acr_cts0;
	u8 acr_cts1;
	u8 acr_cts2;
	u8 acr_ctrl;
	u8 acr_sts0;		/* 8  */
	u8 acr_sts1;
	u8 acr_sts2;
	u8 aud_ctrl;
	u8 i2s_ctrl;		/* c  */
	u8 i2s_dlen;
	u8 reserved1;
	u8 reserved2;
	u8 i2s_dbg_lft0;	/* 10  */
	u8 i2s_dbg_lft1;
	u8 i2s_dbg_lft2;
	u8 i2s_dbg_lft3;
	u8 i2s_dbg_rit0;	/* 14  */
	u8 i2s_dbg_rit1;
	u8 i2s_dbg_rit2;
	u8 i2s_dbg_rit3;
	u8 chsts_0;		/* 18  */
	u8 chsts_1;
	u8 chsts_2;
	u8 chsts_3;
	u8 chsts_4;		/* 1c  */
	u8 fifo_ctrl;
	u8 mem_size_l;
	u8 mem_size_h;
	u8 gcp_cfg0;		/* 20  */
	u8 gcp_cfg1;
	u8 aud_sts;
	u8 reserved6;
	u8 reserved7;		/* 24  */
	u8 reserved8;
	u8 hblank_l;
	u8 hblank_h;
	u8 reserved9;		/* 28  */
	u8 reserved10;
	u8 reserved11;
	u8 reserved12;
	u8 vstart_l;		/* 2c  */
	u8 vstart_h;
	u8 htot_sts_l;
	u8 htot_sts_h;
	u8 hblank_sts_l;	/* 30  */
	u8 hblank_sts_h;
	u8 vtot_sts_l;
	u8 vtot_sts_h;
	u8 vres_sts_l;		/* 34  */
	u8 vres_sts_h;
	u8 vstart_sts_l;
	u8 vstart_sts_h;
	u8 video_sts;		/* 38  */
	u8 video_ctrl;
	u8 hdmi_ctrl;
	u8 reserved13;
	u8 reserved14;		/* 3c  */
	u8 reserved15;
	u8 reserved16;
	u8 reserved17;
	u8 reserved18;		/* 40  */
	u8 reserved19;
	u8 reserved20;
	u8 reserved21;
	u8 reserved22;		/* 44  */
	u8 reserved23;
	u8 pp_hw;
	u8 dc_fifo_sft_rst;
	u8 dc_fifo_wr_ptr;	/* 48  */
	u8 dc_fifo_rd_ptr;
	u8 dc_pp_ctrl;
	u8 reserved24;
	u8 tdata0_0;		/* 4c  */
	u8 tdata0_1;
	u8 tdata0_2;
	u8 reserved25;
	u8 tdata1_0;		/* 50  */
	u8 tdata1_1;
	u8 tdata1_2;
	u8 reserved26;
	u8 tdata2_0;		/* 54  */
	u8 tdata2_1;
	u8 tdata2_2;
	u8 reserved27;
	u8 tdata3_0;		/* 58  */
	u8 tdata3_1;
	u8 tdata3_2;
	u8 tdata_sel;
	u8 swap_ctrl;		/* 5c  */
	u8 avmute_ctrl;
	u8 hst_pkt_ctrl0;
	u8 hst_pkt_ctrl1;
	/* TBD */
};

struct ubits_registers {
	u8 ubits_0;		/* 120  */
	u8 ubits_1;
	u8 ubits_2;
	u8 ubits_3;
	u8 ubits_4;		/* 124  */
	u8 ubits_5;
	u8 ubits_6;
	u8 ubits_7;
	u8 ubits_8;		/* 128  */
	u8 ubits_9;
	u8 ubits_10;
	u8 ubits_11;
	u8 ubits_12;		/* 12C  */
	u8 ubits_13;
	u8 hbr_pkt;
};

/* video_ctrl bits */
#define HDMITX_VIDEO_CTRL_IN_YC         (0x1u<<0)
#define HDMITX_VIDEO_CTRL_FLD_POL       (0x1u<<1)
#define HDMITX_VIDEO_CTRL_DEBUG_CTRL    (0x1u<<2)
#define HDMITX_VIDEO_CTRL_ACR_PRI_SEL_I (0x1u<<3)
#define HDMITX_VIDEO_CTRL_INT_FRM_SEL   (0x1u<<6)

/* hdmi_ctrl bits */
#define HDMITX_HDMI_CTRL_HDMI_MODE     (0x1u<<0)
#define HDMITX_HDMI_CTRL_LAYOUT        (0x1u<<1)
#define HDMITX_HDMI_CTRL_BCH_ROT       (0x1u<<2)
#define HDMITX_HDMI_CTRL_PIX_RPT(n)    ((n) << 3)
#define HDMITX_HDMI_CTRL_PIX_RPT_OFFSET (0x1u<<3)
#define HDMITX_HDMI_CTRL_PIX_RPT_MASK  (0xfu<<3)

/* video_sts bits */
#define HDMITX_VIDEO_STS_INIT_OVER (0x1u<<0)

/* avmute_ctrl bits */
#define HDMITX_AVMUTE_CTRL_REG_AUDIO (0x1u<<0)
#define HDMITX_AVMUTE_CTRL_REG_VIDEO (0x1u<<1)

struct hdtx_timing_para {
	u16 htot;
	u16 hblank;
	u16 vtot;
	u16 vres;
	u16 vstart;
};

struct hdtx_plat_data {
	int boot_en;
	int format;
};

void __init pxa970_set_ihdmi_info(void* info);

#endif
