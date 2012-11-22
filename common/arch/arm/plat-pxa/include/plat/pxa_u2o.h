/*
 * linux/include/asm-arm/arch-pxa/pxa_u2o.h
 *
 * This supports machine-specific differences in how the PXA
 * USB 2.0 Device Controller (U2O) is wired.
 *
 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __ASM_ARCH_PXA_U2O_H
#define __ASM_ARCH_PXA_U2O_H

#include <plat/pxausb_common.h>
/* PHY registers */
#define U2PPLL		(0x000)       /* U2O PHY PLL Control */
#define U2PTX		(0x004)       /* U2O PHY TX Control */
#define U2PRX		(0x008)       /* U2O PHY RX Control */
#define U2IVREF		(0x00C)       /* U2O PHY IVREF Control */
#define U2PT0		(0x010)       /* U2O PHY Test 0 Control */
#define U2PT1		(0x014)       /* U2O PHY Test 1 Control */
#define U2PT2		(0x018)       /* U2O PHY Test 2 Control */
#define U2PT3		(0x01C)       /* U2O PHY Test 3 Control */
#define U2PT4		(0x020)       /* U2O PHY Test 4 Control */
#define U2PT5		(0x024)       /* U2O PHY Test 5 Control */
#define U2PID		(0x028)       /* U2O PHY ID Register */
#define U2PRS		(0x02C)       /* U2O PHY Reserve Register */
#define U2PMN		(0x030)       /* U2O PHY Monitor Register */
#define U2OCG		(0x108)       /* U2O Clock Gate Register */

#define U2P480		(0x42404078)         /* U2O PHY 480Mhz Control */

/* U2PPLL */
#define U2PPLL_CTRL_REG		        (0x000)
#define U2PPLL_TSTB			(1<<31)
#define U2PPLL_PLLVDD18_SHIFT		(29)
#define U2PPLL_PLLVDD18_MASK		(3<<29)
#define U2PPLL_PLLVDD12_SHIFT		(27)
#define U2PPLL_PLLVDD12_MASK		(3<<27)
#define U2PPLL_PLLCALI12_SHIFT		(25)
#define U2PPLL_PLLCALI12_MASK		(3<<25)
#define U2PPLL_CLK_BLK_EN		(1<<24)
#define U2PPLL_READY			(1<<23)
#define U2PPLL_KVCO_EXT			(1<<22)
#define U2PPLL_VCOCAL_START		(1<<21)
#define U2PPLL_BGP_VSEL_SHIFT		(19)
#define U2PPLL_BGP_VSEL_MASK		(3<<19)
#define U2PPLL_LOCKDET_ISEL		(1<<18)
#define U2PPLL_KVCO_SHIFT		(15)
#define U2PPLL_KVCO_MASK		(7<<15)
#define U2PPLL_ICP_SHIFT		(12)
#define U2PPLL_ICP_MASK			(7<<12)
#define U2PPLL_FBDIV_SHIFT		(4)
#define U2PPLL_FBDIV_MASK		(0xff<<4)
#define U2PPLL_REFDIV_SHIFT		(0)
#define U2PPLL_REFDIV_MASK		(0xf<<4)

/* U2PTX */
#define U2PTX_CTRL_REG                  (0x004)
#define U2PTX_RCAL_START		(1<<12)

/* U2PRX */
#define U2PRX_EARLY_VOS_ON_EN			(1<<31)
#define U2PRX_RXDATA_BLOCK_EN			(1<<30)
#define U2PRX_RXDATA_BLOCK_LENGTH_SHIFT		(28)
#define U2PRX_RXDATA_BLOCK_LENGTH_MASK		(3<<28)
#define U2PRX_EDGE_DET_SEL_SHIFT		(26)
#define U2PRX_EDGE_DET_SEL_MASK			(3<<26)
#define U2PRX_EDGE_DET_EN			(1<<25)
#define U2PRX_S2TO3_DLY_SEL_SHIFT		(23)
#define U2PRX_S2TO3_DLY_SEL_MASK		(3<<23)
#define U2PRX_CDR_COEF_SEL			(1<<22)
#define U2PRX_CDR_FASTLOCK_EN			(1<<21)
#define U2PRX_PHASE_FREEZE_DLY			(1<<20)
#define U2PRX_REG_USQ_LENGTH			(1<<19)
#define U2PRX_REG_ACQ_LENGTH_SHIFT		(17)
#define U2PRX_REG_ACQ_LENGTH_MASK		(3<<17)
#define U2PRX_REG_SQ_LENGTH_SHIFT		(15)
#define U2PRX_REG_SQ_LENGTH_MASK		(3<<15)
#define U2PRX_DLL_SEL_SHIFT			(13)
#define U2PRX_DLL_SEL_MASK			(3<<13)
#define U2PRX_CAP_SEL_SHIFT			(10)
#define U2PRX_CAP_SEL_MASK			(7<<10)
#define U2PRX_DISCON_THRESH_SHIFT		(8)
#define U2PRX_DISCON_THRESH_MASk		(3<<8)
#define U2PRX_SQ_THRESH_SHIFT			(4)
#define U2PRX_SQ_THRESH_MASK			(0xf<<4)
#define U2PRX_LPF_COEF_SHIFT			(2)
#define U2PRX_LPF_COEF_MASK			(3<<2)
#define U2PRX_INTPI_SHIFT			(0)
#define U2PRX_INTPI_MASK			(3<<0)

/* U2IVREF */
#define U2IVREF_SAMPLER_CTRL           		(1<<31)
#define U2IVREF_RXVDD18_SHIFT          		(29)
#define U2IVREF_RXVDD18_MASK           		(3<<29)
#define U2IVREF_SQ_CM_SEL              		(1<<10)
#define U2IVREF_BG_VSEL_SHIFT          		(8)
#define U2IVREF_BG_VSEL_MASK           		(3<<8)
#define U2IVREF_RXVDD12_SHIFT          		(6)
#define U2IVREF_RXVDD12_MASK           		(3<<6)
#define U2IVREF_FSDRV_EN_SHIFT         		(2)
#define U2IVREF_FSDRV_EN_MASK          		(0xf<<2)
#define U2IVREF_REG_IMP_CAL_DLY_SHIFT  		(0)
#define U2IVREF_REG_IMP_CAL_DLY_MASK   		(3<<0)

/* U2PT0 */
#define U2PT0_REG_ARC_DPDM_MODE			(1<<28)

/* usb otg controller register base */
#define PXA168_U2O_REGBASE 	(0xd4208000)
#define PXA168_U2O_PHYBASE	(0xd4207000)

#define PXA935_U2O_REGBASE	(0x55502000)
#define PXA935_U2O_PHYBASE	(0x5550a000)

#define PXA168_U2H_REGBASE      (0xd4209000)
#define PXA168_U2H_PHYBASE      (0xd4206000)

#define MMP3_HSIC1_REGBASE	(0xf0001000)
#define MMP3_HSIC1_PHYBASE	(0xf0001800)

#define MMP3_HSIC2_REGBASE	(0xf0002000)
#define MMP3_HSIC2_PHYBASE	(0xf0002800)

#define MMP3_FSIC_REGBASE	(0xf0003000)
#define MMP3_FSIC_PHYBASE	(0xf0003800)

#define USB_REG_RANGE		(0x1ff)
#define USB_PHY_RANGE		(0xff)

/* registers */
#define U2x_CAPREGS_OFFSET       0x100

#define U2xUSBCMD				(0x140)       /* U2O Command */
#define U2xUSBCMD_RST				(1<<1)      /* Reset */
#define U2xUSBCMD_RS				(1<<0)      /* Run/Stop */

#define U2xUSBSTS				(0x144)       /* U2O Status */
#define U2xUSBINTR				(0x148)       /* U2O Interrupt Enable */

#define U2xPORTSC				(0x184)       /* U2O Port Status */
#define U2xPORTSC_PP                            (1<<12)   		  /* Port Power */
#define U2xPORTSC_PTS_MASK                      (3<<30)   		  /* Parallel Transceiver Select */

#define U2xUSBINTR				(0x148)       /* U2O Interrupt Enable */
#define U2xUSBMODE				(0x1A8)       /* U2O Device Mode */
#define U2xUSBMODE_CM_MASK                      (3<<0)   		  /* U2O Controller Mode */

#define U2xOTGSC				(0x1A4)       /* U2O On-The-Go Status and Control */

/* OTG interrupt enable bit masks */
#define  U2xOTGSC_DPIE                         (0x40000000)   /* Data-line pulsing IE */
#define  U2xOTGSC_1MSIE                        (0x20000000)   /* 1 Millisecond timer IE */
#define  U2xOTGSC_BSEIE                        (0x10000000)   /* B-session end IE */
#define  U2xOTGSC_BSVIE                        (0x08000000)   /* B-session valid IE */
#define  U2xOTGSC_ASVIE                        (0x04000000)   /* A-session valid IE */
#define  U2xOTGSC_AVVIE                        (0x02000000)   /* A-V-bus valid IE */
#define  U2xOTGSC_IDIE                         (0x01000000)   /* OTG ID IE */
#define  U2xOTGSC_IE_MASK   		       (0x7F000000)

/* OTG interrupt status bit masks */
#define  U2xOTGSC_IS_MASK   (0x007F0000)
#define  U2xOTGSC_DPIS                         (0x00400000)   /* Data-line pulsing IS */
#define  U2xOTGSC_1MSIS                        (0x00200000)   /* 1 Millisecond timer IS */
#define  U2xOTGSC_BSEIS                        (0x00100000)   /* B-session end IS */
#define  U2xOTGSC_BSVIS                        (0x00080000)   /* B-session valid IS */
#define  U2xOTGSC_ASVIS                        (0x00040000)   /* A-session valid IS */
#define  U2xOTGSC_AVVIS                        (0x00020000)   /* A-Vbus valid IS */
#define  U2xOTGSC_IDIS                         (0x00010000)   /* OTG ID IS */

/* OTG status bit masks */
#define  U2xOTGSC_DPS                          (0x00004000)   /* Data-line pulsing */
#define  U2xOTGSC_1MST                         (0x00002000)   /* 1 Milliseconf timer toggle */
#define  U2xOTGSC_BSE                          (0x00001000)   /* B-session end */
#define  U2xOTGSC_BSV                          (0x00000800)   /* B-session valid */
#define  U2xOTGSC_ASV                          (0x00000400)   /* A-session valid */
#define  U2xOTGSC_AVV                          (0x00000200)   /* A-Vbus Valid */
#define  U2xOTGSC_ID                           (0x00000100)   /* OTG ID */

/* OTG control bit masks */
#define  U2xOTGSC_CTL_BITS                     (0x2F)
#define  U2xOTGSC_HABA                         (0x00000080)   /* hardware assisted B-Dis to A-Con */
#define  U2xOTGSC_HADP                         (0x00000040)   /* hardware assisted data pulse bits*/
#define  U2xOTGSC_IDPU                         (0x00000020)   /* ID pull-up enable */
#define  U2xOTGSC_DP                           (0x00000010)   /* Data-pulsing */
#define  U2xOTGSC_OT                           (0x00000008)   /* OTG termination */
#define  U2xOTGSC_HAAR                         (0x00000004)   /* Auto reset bit */
#define  U2xOTGSC_VC                           (0x00000002)   /* Vbus charge */
#define  U2xOTGSC_VD                           (0x00000001)   /* Vbus discharge */

#define UTMI_REVISION		0x0
#define UTMI_CTRL		0x4
#define UTMI_PLL		0x8
#define UTMI_TX			0xc
#define UTMI_RX			0x10
#define UTMI_IVREF		0x14
#define UTMI_T0			0x18
#define UTMI_T1			0x1c
#define UTMI_T2			0x20
#define UTMI_T3			0x24
#define UTMI_T4			0x28
#define UTMI_T5			0x2c
#define UTMI_RESERVE		0x30
#define UTMI_USB_INT		0x34
#define UTMI_DBG_CTL		0x38
#define UTMI_OTG_ADDON		0x3c

/* For UTMICTRL Register */
#define UTMI_CTRL_USB_CLK_EN                    (1<<31)
/* pxa168 */
#define UTMI_CTRL_SUSPEND_SET1                  (1<<30)
#define UTMI_CTRL_SUSPEND_SET2                  (1<<29)
#define UTMI_CTRL_RXBUF_PDWN                    (1<<24)
#define UTMI_CTRL_TXBUF_PDWN                    (1<<11)

#define UTMI_CTRL_INPKT_DELAY_SHIFT             30
#define UTMI_CTRL_INPKT_DELAY_SOF_SHIFT 	28
#define UTMI_CTRL_PU_REF_SHIFT			20
#define UTMI_CTRL_ARC_PULLDN_SHIFT              12
#define UTMI_CTRL_PLL_PWR_UP_SHIFT              1
#define UTMI_CTRL_PWR_UP_SHIFT                  0
/* For UTMI_PLL Register */
#define UTMI_PLL_CLK_BLK_EN_SHIFT               24
#define UTMI_PLL_FBDIV_SHIFT                    4
#define UTMI_PLL_REFDIV_SHIFT                   0
#define UTMI_PLL_FBDIV_MASK                     0x00000FF0
#define UTMI_PLL_REFDIV_MASK                    0x0000000F
#define UTMI_PLL_ICP_MASK                       0x00007000
#define UTMI_PLL_KVCO_MASK                      0x00031000
#define UTMI_PLL_PLLCALI12_SHIFT		29
#define UTMI_PLL_PLLCALI12_MASK			(0x3<<29)
#define UTMI_PLL_PLLVDD18_SHIFT			27
#define UTMI_PLL_PLLVDD18_MASK			(0x3<<27)
#define UTMI_PLL_PLLVDD12_SHIFT			25
#define UTMI_PLL_PLLVDD12_MASK			(0x3<<25)
#define UTMI_PLL_KVCO_SHIFT			15
#define UTMI_PLL_ICP_SHIFT			12
/* For UTMI_TX Register */
#define UTMI_TX_REG_EXT_FS_RCAL_SHIFT		27
#define UTMI_TX_REG_EXT_FS_RCAL_MASK		(0xf<<27)
#define UTMI_TX_REG_EXT_FS_RCAL_EN_MASK		26
#define UTMI_TX_REG_EXT_FS_RCAL_EN		(0x1<<26)
#define UTMI_TX_LOW_VDD_EN_SHIFT                11
#define UTMI_TX_IMPCAL_VTH_SHIFT                14
#define UTMI_TX_IMPCAL_VTH_MASK                 (0x7<<14)
#define UTMI_TX_CK60_PHSEL_SHIFT                17
#define UTMI_TX_CK60_PHSEL_MASK                 (0xf<<17)
#define UTMI_TX_TXVDD12_SHIFT                   22
#define UTMI_TX_TXVDD12_MASK                    (0x3<<22)
#define UTMI_TX_AMP_SHIFT			0
#define UTMI_TX_AMP_MASK			(0x7<<0)
/* For UTMI_RX Register */
#define UTMI_RX_SQ_THRESH_SHIFT                 4
#define UTMI_RX_SQ_THRESH_MASK                  (0xf<<4)
#define UTMI_REG_SQ_LENGTH_SHIFT                15
#define UTMI_REG_SQ_LENGTH_MASK                 (0x3<<15)

#define REG_RCAL_START                          0x00001000
#define VCOCAL_START                            0x00200000
#define KVCO_EXT                                0x00400000
#define PLL_READY                               0x00800000
#define CLK_BLK_EN                              0x01000000

#define UTMI_OTG_ADDON_OTG_ON			(1<<0)

/* For MMP3 USB Phy */
#define USB2_PLL_REG0		0x4
#define USB2_PLL_REG1		0x8
#define USB2_TX_REG0		0x10
#define USB2_TX_REG1		0x14
#define USB2_TX_REG2		0x18
#define USB2_RX_REG0		0x20
#define USB2_RX_REG1		0x24
#define USB2_RX_REG2		0x28
#define USB2_ANA_REG0		0x30
#define USB2_ANA_REG1		0x34
#define USB2_ANA_REG2		0x38
#define USB2_DIG_REG0		0x3C
#define USB2_DIG_REG1		0x40
#define USB2_DIG_REG2		0x44
#define USB2_DIG_REG3		0x48
#define USB2_TEST_REG0		0x4C
#define USB2_TEST_REG1		0x50
#define USB2_TEST_REG2		0x54
#define USB2_CHARGER_REG0	0x58
#define USB2_OTG_REG0		0x5C
#define USB2_PHY_MON0		0x60
#define USB2_RESETVE_REG0	0x64
#define USB2_ICID_REG0		0x78
#define USB2_ICID_REG1		0x7C

#define USB2_PLL_FBDIV_SHIFT_MMP3		0
#define USB2_PLL_REFDIV_SHIFT_MMP3		8
#define USB2_PLL_VDD12_SHIFT_MMP3		12
#define USB2_PLL_VDD18_SHIFT_MMP3		14
#define USB2_PLL_FBDIV_MASK_MMP3		0x00FF
#define USB2_PLL_REFDIV_MASK_MMP3		0x0F00

#define USB2_PLL_CAL12_SHIFT_MMP3		0
#define USB2_PLL_VCOCAL_START_SHIFT_MMP3	2
#define USB2_PLL_KVCO_SHIFT_MMP3		4
#define USB2_PLL_ICP_SHIFT_MMP3			8
#define USB2_PLL_LOCK_BYPASS_SHIFT_MMP3		12
#define USB2_PLL_PU_PLL_SHIFT_MMP3		13
#define USB2_PLL_PU_PLL_MASK			0x00002000
#define USB2_PLL_CALI12_MASK_MMP3		(0x3)
#define USB2_PLL_KVCO_MASK_MMP3			(0x7<<4)
#define USB2_PLL_ICP_MASK_MMP3			(0x7<<8)
#define USB2_PLL_READY_MASK_MMP3		(0x1<<15)

#define USB2_TX_CK60_PHSEL_SHIFT_MMP3		0
#define USB2_TX_AMP_SHIFT_MMP3			4
#define USB2_TX_VDD12_SHIFT_MMP3		8
#define USB2_TX_RCAL_START_SHIFT_MMP3		13
#define USB2_TX_IMPCAL_VTH_MASK_MMP3		(0x7<<8)
#define USB2_TX_CK60_PHSEL_MASK_MMP3		(0xf)
#define USB2_TX_AMP_MASK_MMP3			(0x7<<4)
#define USB2_TX_VDD12_MASK_MMP3			(0x3<<8)

#define USB2_TX_IMPCAL_VTH_SHIFT_MMP3		8
#define USB2_TX_DRV_SLEWRATE_SHIFT		10

#define USB2_RX_SQ_THRESH_SHIFT_MMP3		4
#define USB2_RX_SQ_LENGTH_SHIFT_MMP3		10
#define USB2_RX_SQ_THRESH_MASK_MMP3		(0xf<<4)
#define USB2_RX_SQ_LENGTH_MASK_MMP3		(0x3<<10)

#define USB2_ANA_PU_ANA_SHIFT_MMP3		14

#define USB2_OTG_PU_OTG_SHIFT_MMP3		3

/* fsic registers */
#define FSIC_MISC			0x4
#define FSIC_INT			0x28
#define FSIC_CTRL			0x30

/* hsic registers */
#define HSIC_CTRL			0x8
#define HSIC_CTRL_HSIC_ENABLE		(1<<7)
#define HSIC_CTRL_PLL_BYPASS		(1<<4)

#define HSIC_INT			0x14
#define HSIC_INT_READY_INT_EN		(1<<10)
#define HSIC_INT_CONNECT_INT_EN		(1<<9)
#define HSIC_INT_CORE_INT_EN		(1<<8)
#define HSIC_INT_HS_READY		(1<<2)
#define HSIC_INT_CONNECT		(1<<1)
#define HSIC_INT_CORE			(1<<0)

#define HSIC_USB_CTRL			0x28
#define HSIC_USB_CTRL_CLKEN		1
#define	HSIC_USB_CLK_PHY		0x0
#define HSIC_USB_CLK_PMU		0x1

#define res_size(res)   ((res)->end - (res)->start + 1)

struct device;
struct otg_transceiver;

struct pxa_usb_plat_info {
	int (*phy_init) (unsigned base);
	int (*phy_deinit) (unsigned base);
	int (*plat_init) (struct device *dev);
	int (*vbus_set) (int);
	int (*vbus_status) (unsigned base);	/* do we see host? */
	int (*vbus_detect) (void *func, int enable);
	int (*usbid_detect) (struct otg_transceiver *otg);
	int (*set_power) (int);
	int clk_gating;
	int rely_on_vbus;
	int in_single;
	int out_single;
	int is_otg;
	unsigned int regbase;
	unsigned int phybase;
	void * (*init_pmic_ops) (void);
};

/*following defined are used by userspace */
/* required for registration for uevent*/
#define USB_DRV_NAME			"pxa-u2o"
/* required for sysfs interface */
#define USB_SYSFS_PATH			"/sys/devices/platform/pxa-u2o/"
#define USB_SYSFS_CONNECTED		USB_SYSFS_PATH "connected"
#define USB_SYSFS_MODE			USB_SYSFS_PATH "mode"
#define USB_SYSFS_COMPOSITE		USB_SYSFS_PATH "composite"

enum gadget_attributes{
    PXA_GADGET_CABLE_DISCONNECTED = 0,
	PXA_GADGET_CABLE_CONNECTED,
	PXA_GADGET_MODE_DEVICE,
	PXA_GADGET_MODE_HOST,
};

extern struct otg_xceiv_ops *init_pxa9xx_otg_xceiv_ops(void);
extern struct otg_ctrl_ops *init_pxa9xx_otg_ctrl_ops(void);

extern int otg_is_client(void);
extern int otg_is_host(void);


extern unsigned int u2o_get(unsigned int base, unsigned int offset);
extern void u2o_set(unsigned int base, unsigned int offset, unsigned int value);
extern void u2o_clear(unsigned int base, unsigned int offset, unsigned int value);
extern void u2o_write(unsigned int base, unsigned int offset, unsigned int value);

extern int rely_on_vbus;
extern int connected;
#endif /* __ASM_ARCH_PXA_U2O_H */

