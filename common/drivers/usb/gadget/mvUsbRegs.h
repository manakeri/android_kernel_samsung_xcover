/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell 
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File in accordance with the terms and conditions of the General 
Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
available along with the File in the license.txt file or by writing to the Free 
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
DISCLAIMED.  The GPL License provides additional details about this warranty 
disclaimer.
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File under the following licensing terms. 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer. 

    *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution. 

    *   Neither the name of Marvell nor the names of its contributors may be 
        used to endorse or promote products derived from this software without 
        specific prior written permission. 
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef __INCmvUsbRegsh
#define __INCmvUsbRegsh

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//#include "mvCtrlEnvSpec.h"

#define	USB_REG_BASE(dev)	(&__REG_3(0x55502000))
#define	USB_PHY_REG_BASE(dev)	(&__REG_3(0x5550A000))
#define MV_USB_MAX_ADDR_DECODE_WIN  4

/*******************************************/
/* USB ARC Core Registers                  */
/*******************************************/
#if 1
#define MV_USB_CORE_ID_REG(dev)                 (0x00)
#define MV_USB_CORE_GENERAL_REG(dev)            (0x04)
#define MV_USB_CORE_HOST_REG(dev)               (0x08)
#define MV_USB_CORE_DEVICE_REG(dev)             (0x0C)
#define MV_USB_CORE_TX_BUF_REG(dev)             (0x10)
#define MV_USB_CORE_RX_BUF_REG(dev)             (0x14)
//#define MV_USB_CORE_TTTX_BUF_REG(dev)           (USB_REG_BASE(dev) + 0x18)
//#define MV_USB_CORE_TTRX_BUF_REG(dev)           (USB_REG_BASE(dev) + 0x1C)

#define MV_USB_CORE_GPTIMER0_LD_REG(dev)        (0x80)
#define MV_USB_CORE_GPTIMER0_CTRL_REG(dev)      (0x84)
#define MV_USB_CORE_GPTIMER1_LD_REG(dev)        (0x88)
#define MV_USB_CORE_GPTIMER1_CTRL_REG(dev)      (0x8C)
#define MV_USB_CORE_SUBS_CFG_REG(dev)           (0x90)

#define MV_USB_CORE_CAP_LENGTH_REG(dev)         (0x100)
#define MV_USB_CORE_CAP_HCS_PARAMS_REG(dev)     (0x104)
#define MV_USB_CORE_CAP_HCC_PARAMS_REG(dev)     (0x108)

#define MV_USB_CORE_CAP_DCI_VERSION_REG(dev)    (0x120)
#define MV_USB_CORE_CAP_DCC_PARAMS_REG(dev)     (0x124)

#define MV_USB_CORE_CMD_REG(dev)                (0x140)

#define MV_USB_CORE_CMD_RUN_BIT             0
#define MV_USB_CORE_CMD_RUN_MASK            (1 << MV_USB_CORE_CMD_RUN_BIT)

#define MV_USB_CORE_CMD_RESET_BIT           1
#define MV_USB_CORE_CMD_RESET_MASK          (1 << MV_USB_CORE_CMD_RESET_BIT)

#define MV_USB_CORE_STATUS_REG(dev)             (0x144)
#define MV_USB_CORE_INTR_REG(dev)               (0x148)
#define MV_USB_CORE_FRAME_INDEX_REG(dev)        (0x14C)

#define MV_USB_CORE_PERIODIC_LIST_BASE_REG(dev) (0x154)
#define MV_USB_CORE_DEV_ADDR_REG(dev)           (0x154)

#define MV_USB_CORE_ASYNC_LIST_ADDR_REG(dev)    (0x158)
#define MV_USB_CORE_ENDPOINT_LIST_ADDR_REG(dev) (0x158)

#define MV_USB_CORE_TT_CTRL_REG(dev)            (0x15C)
#define MV_USB_CORE_BURST_SIZE_REG(dev)         (0x160)
#define MV_USB_CORE_TX_FILL_TUNING_REG(dev)     (0x164)
//#define MV_USB_CORE_TX_TT_FILL_TUNING_REG(dev)  (USB_REG_BASE(dev) + 0x168)
#define MV_USB_CORE_EP_NAK_REG(dev)             (0x178)
#define MV_USB_CORE_EP_NAKEN_REG(dev)           (0x17C)
#define MV_USB_CORE_CONFIG_FLAG_REG(dev)        (0x180)
#define MV_USB_CORE_PORTSC_REG(dev)             (0x184)
#define MV_USB_CORE_OTGSC_REG(dev)              (0x1A4)

#define MV_USB_CORE_MODE_REG(dev)               (0x1A8)

#define MV_USB_CORE_MODE_OFFSET                 0
#define MV_USB_CORE_MODE_MASK                   (3 << MV_USB_CORE_MODE_OFFSET)
#define MV_USB_CORE_MODE_HOST                   (3 << MV_USB_CORE_MODE_OFFSET)
#define MV_USB_CORE_MODE_DEVICE                 (2 << MV_USB_CORE_MODE_OFFSET)

/* Bit[2] (ES) - don't care */

#define MV_USB_CORE_SETUP_LOCK_DISABLE_BIT      3
#define MV_USB_CORE_SETUP_LOCK_DISABLE_MASK     (1 << MV_USB_CORE_SETUP_LOCK_DISABLE_BIT)

#define MV_USB_CORE_STREAM_DISABLE_BIT          4
#define MV_USB_CORE_STREAM_DISABLE_MASK         (1 << MV_USB_CORE_STREAM_DISABLE_BIT)


#define MV_USB_CORE_ENDPT_SETUP_STAT_REG(dev)    (0x1AC)
#define MV_USB_CORE_ENDPT_PRIME_REG(dev)         (0x1B0)
#define MV_USB_CORE_ENDPT_FLUSH_REG(dev)         (0x1B4)
#define MV_USB_CORE_ENDPT_STATUS_REG(dev)        (0x1B8)
#define MV_USB_CORE_ENDPT_COMPLETE_REG(dev)      (0x1BC)
#define MV_USB_CORE_ENDPT_CTRL_REG(dev, ep)      (0x1C0 + (ep*4))

/*******************************************/
/* USB Bridge Registers                    */
/*******************************************/
/*#define MV_USB_BRIDGE_CTRL_REG(dev)              (USB_REG_BASE(dev) + 0x300)
#define MV_USB_BRIDGE_CORE_BYTE_SWAP_OFFSET 4
#define MV_USB_BRIDGE_CORE_BYTE_SWAP_MASK   (1 << MV_USB_BRIDGE_CORE_BYTE_SWAP_OFFSET)
#define MV_USB_BRIDGE_CORE_BYTE_SWAP_EN     (0 << MV_USB_BRIDGE_CORE_BYTE_SWAP_OFFSET)

#define MV_USB_BRIDGE_INTR_CAUSE_REG(dev)        (USB_REG_BASE(dev) + 0x310)
#define MV_USB_BRIDGE_INTR_MASK_REG(dev)         (USB_REG_BASE(dev) + 0x314)
*/
#define MV_USB_MUX_CTRL_REG(dev)                 (0x300)
#define MV_USB_INT_STATUS_REG(dev)               (0x304)
#define MV_USB_INT_ENABLE_REG(dev)               (0x308)

/* BITs in Interrupt Enable and Status registers */
#define MV_USB_INT_WKEN_BIT        0
#define MV_USB_INT_WKEN_MASK       (1<<MV_USB_INT_WKEN_BIT)

//#define MV_USB_BRIDGE_ERROR_ADDR_REG(dev)        (USB_REG_BASE(dev) + 0x31C)

#define MV_USB_WIN_CTRL_REG(dev, win)        (0x320 + ((win)<<4)) 
#define MV_USB_WIN_BASE_REG(dev, win)        (0x324 + ((win)<<4))

/* BITs in Windows 0-3 Control and Base Registers */
#if 0
#define MV_USB_WIN_ENABLE_BIT               0
#define MV_USB_WIN_ENABLE_MASK              (1 << MV_USB_WIN_ENABLE_BIT)

#define MV_USB_WIN_BURST_WR_LIMIT_BIT       1
#define MV_USB_WIN_BURST_WR_LIMIT_MASK      (1 << MV_USB_WIN_BURST_WR_LIMIT_BIT)
#define MV_USB_WIN_BURST_WR_NO_LIMIT        (0 << MV_USB_WIN_BURST_WR_LIMIT_BIT)
#define MV_USB_WIN_BURST_WR_32BIT_LIMIT     (1 << MV_USB_WIN_BURST_WR_LIMIT_BIT)

#define MV_USB_WIN_TARGET_OFFSET            4
#define MV_USB_WIN_TARGET_MASK              (0xF << MV_USB_WIN_TARGET_OFFSET)

#define MV_USB_WIN_ATTR_OFFSET              8
#define MV_USB_WIN_ATTR_MASK                (0xFF << MV_USB_WIN_ATTR_OFFSET)

#define MV_USB_WIN_SIZE_OFFSET              16
#define MV_USB_WIN_SIZE_MASK                (0xFFFF << MV_USB_WIN_SIZE_OFFSET)

#define MV_USB_WIN_BASE_OFFSET              16
#define MV_USB_WIN_BASE_MASK                (0xFFFF << MV_USB_WIN_BASE_OFFSET)


#define MV_USB_BRIDGE_IPG_REG(dev)          (USB_REG_BASE(dev) + 0x360)
#endif

/*******************************************/
/* Interrupt Controller Registers          */
/*******************************************/
/*#define USB_CAUSE_REG(dev)                  (USB_REG_BASE(dev) + 0x310)
#define USB_MASK_REG(dev)                   (USB_REG_BASE(dev) + 0x314)
#define USB_ERROR_ADDR_REG(dev)             (USB_REG_BASE(dev) + 0x31c)
*/

/*******************************************/
/* USB PHY Registers                       */
/*******************************************/


#if 0
#define MV_USB_PHY_POWER_CTRL_REG(dev)          (__REG(0x42400000) + 0x4078)

#define U2PPLL					(__REG(0x5550a000))
#define U2PPLL_ICP_MASK		0x00007000
#define U2PPLL_ICP_SHIFT	12
#define U2PPLL_KVCO_MASK	0x00038000
#define U2PPLL_KVCO_SHIFT	15

#define U2PTX					(__REG(0x5550a004))
#define U2PRX					(__REG(0x5550a008))
#define U2PIVREF				(__REG(0x5550a00C))
#define U2PT0					(__REG(0x5550a010))
#define U2PT1					(__REG(0x5550a014))
#define U2PT2					(__REG(0x5550a018))
#define U2PT3					(__REG(0x5550a01C))
#define U2PT4					(__REG(0x5550a020))
#define U2PT5					(__REG(0x5550a024))
#define U2PID					(__REG(0x5550a028))
#define U2PRS					(__REG(0x5550a02C))
#define U2PMN					(__REG(0x5550a030))
#define U2P480					(__REG(0x42404078))


#define MV_USB_PHY_PLL_CTRL_REG		        (0x000)
#define MV_USB_PHY_PLL_TSTB			(1<<31)
#define MV_USB_PHY_PLL_PLLVDD18_SHIFT		(29)
#define MV_USB_PHY_PLL_PLLVDD18_MASK		(3<<29)
#define MV_USB_PHY_PLL_PLLVDD12_SHIFT		(27)
#define MV_USB_PHY_PLL_PLLVDD12_MASK		(3<<27)
#define MV_USB_PHY_PLL_PLLCALI12_SHIFT		(25)
#define MV_USB_PHY_PLL_PLLCALI12_MASK		(3<<25)
#define MV_USB_PHY_PLL_CLK_BLK_EN		(1<<24)
#define MV_USB_PHY_PLL_READY			(1<<23)
#define MV_USB_PHY_PLL_KVCO_EXT			(1<<22)
#define MV_USB_PHY_PLL_VCOCAL_START		(1<<21)
#define MV_USB_PHY_PLL_BGP_VSEL_SHIFT		(19)
#define MV_USB_PHY_PLL_BGP_VSEL_MASK		(3<<19)
#define MV_USB_PHY_PLL_LOCKDET_ISEL		(1<<18)
#define MV_USB_PHY_PLL_KVCO_SHIFT		(15)
#define MV_USB_PHY_PLL_KVCO_MASK		(7<<15)
#define MV_USB_PHY_PLL_ICP_SHIFT		(12)
#define MV_USB_PHY_PLL_ICP_MASK			(7<<12)
#define MV_USB_PHY_PLL_FBDIV_SHIFT		(4)
#define MV_USB_PHY_PLL_FBDIV_MASK		(0xff<<4)
#define MV_USB_PHY_PLL_REFDIV_SHIFT		(0)
#define MV_USB_PHY_PLL_REFDIV_MASK		(0xf<<4)


#define MV_USB_PHY_TX_CTRL_REG                  (0x004)
#define MV_USB_PHY_TX_RCAL_START		(1<<12)

#define MV_USB_PHY_RX_CTRL_REG                  (0x008)
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

#define MV_USB_PHY_IVREF_CTRL_REG               (0x00C)
#define MV_USB_PHY_IVREF_SAMPLER_CTRL           (1<<31)
#define MV_USB_PHY_IVREF_RXVDD18_SHIFT          (29)
#define MV_USB_PHY_IVREF_RXVDD18_MASK           (3<<29)
#define MV_USB_PHY_IVREF_SQ_CM_SEL              (1<<10)
#define MV_USB_PHY_IVREF_BG_VSEL_SHIFT          (8)
#define MV_USB_PHY_IVREF_BG_VSEL_MASK           (3<<8)
#define MV_USB_PHY_IVREF_RXVDD12_SHIFT          (6)
#define MV_USB_PHY_IVREF_RXVDD12_MASK           (3<<6)
#define MV_USB_PHY_IVREF_FSDRV_EN_SHIFT         (2)
#define MV_USB_PHY_IVREF_FSDRV_EN_MASK          (0xf<<2)
#define MV_USB_PHY_IVREF_REG_IMP_CAL_DLY_SHIFT  (0)
#define MV_USB_PHY_IVREF_REG_IMP_CAL_DLY_MASK   (3<<0)

#define MV_USB_PHY_TEST_CTRL0_REG               (0x010)

#define MV_USB_PHY_TEST_CTRL0_DPDM_MODE		(1<<28)

#define MV_USB_OTG_CLK_GATING_REG               (0x108)
#endif
#else
	/********************/
#define MV_USB_CORE_ID_REG(dev)                 (USB_REG_BASE(dev) + 0x00)
#define MV_USB_CORE_GENERAL_REG(dev)            (USB_REG_BASE(dev) + 0x04)
#define MV_USB_CORE_HOST_REG(dev)               (USB_REG_BASE(dev) + 0x08)
#define MV_USB_CORE_DEVICE_REG(dev)             (USB_REG_BASE(dev) + 0x0C)
#define MV_USB_CORE_TX_BUF_REG(dev)             (USB_REG_BASE(dev) + 0x10)
#define MV_USB_CORE_RX_BUF_REG(dev)             (USB_REG_BASE(dev) + 0x14)
//#define MV_USB_CORE_TTTX_BUF_REG(dev)           (USB_REG_BASE(dev) + 0x18)
//#define MV_USB_CORE_TTRX_BUF_REG(dev)           (USB_REG_BASE(dev) + 0x1C)

#define MV_USB_CORE_GPTIMER0_LD_REG(dev)        (USB_REG_BASE(dev) + 0x80)
#define MV_USB_CORE_GPTIMER0_CTRL_REG(dev)      (USB_REG_BASE(dev) + 0x84)
#define MV_USB_CORE_GPTIMER1_LD_REG(dev)        (USB_REG_BASE(dev) + 0x88)
#define MV_USB_CORE_GPTIMER1_CTRL_REG(dev)      (USB_REG_BASE(dev) + 0x8C)
#define MV_USB_CORE_SUBS_CFG_REG(dev)           (USB_REG_BASE(dev) + 0x90)

#define MV_USB_CORE_CAP_LENGTH_REG(dev)         (USB_REG_BASE(dev) + 0x100)
#define MV_USB_CORE_CAP_HCS_PARAMS_REG(dev)     (USB_REG_BASE(dev) + 0x104)
#define MV_USB_CORE_CAP_HCC_PARAMS_REG(dev)     (USB_REG_BASE(dev) + 0x108)

#define MV_USB_CORE_CAP_DCI_VERSION_REG(dev)    (USB_REG_BASE(dev) + 0x120)
#define MV_USB_CORE_CAP_DCC_PARAMS_REG(dev)     (USB_REG_BASE(dev) + 0x124)

#define MV_USB_CORE_CMD_REG(dev)                (USB_REG_BASE(dev) + 0x140)

#define MV_USB_CORE_CMD_RUN_BIT             0
#define MV_USB_CORE_CMD_RUN_MASK            (1 << MV_USB_CORE_CMD_RUN_BIT)

#define MV_USB_CORE_CMD_RESET_BIT           1
#define MV_USB_CORE_CMD_RESET_MASK          (1 << MV_USB_CORE_CMD_RESET_BIT)

#define MV_USB_CORE_STATUS_REG(dev)             (USB_REG_BASE(dev) + 0x144)
#define MV_USB_CORE_INTR_REG(dev)               (USB_REG_BASE(dev) + 0x148)
#define MV_USB_CORE_FRAME_INDEX_REG(dev)        (USB_REG_BASE(dev) + 0x14C)

#define MV_USB_CORE_PERIODIC_LIST_BASE_REG(dev) (USB_REG_BASE(dev) + 0x154)
#define MV_USB_CORE_DEV_ADDR_REG(dev)           (USB_REG_BASE(dev) + 0x154)

#define MV_USB_CORE_ASYNC_LIST_ADDR_REG(dev)    (USB_REG_BASE(dev) + 0x158)
#define MV_USB_CORE_ENDPOINT_LIST_ADDR_REG(dev) (USB_REG_BASE(dev) + 0x158)

#define MV_USB_CORE_TT_CTRL_REG(dev)            (USB_REG_BASE(dev) + 0x15C)
#define MV_USB_CORE_BURST_SIZE_REG(dev)         (USB_REG_BASE(dev) + 0x160)
#define MV_USB_CORE_TX_FILL_TUNING_REG(dev)     (USB_REG_BASE(dev) + 0x164)
//#define MV_USB_CORE_TX_TT_FILL_TUNING_REG(dev)  (USB_REG_BASE(dev) + 0x168)
#define MV_USB_CORE_EP_NAK_REG(dev)             (USB_REG_BASE(dev) + 0x178)
#define MV_USB_CORE_EP_NAKEN_REG(dev)           (USB_REG_BASE(dev) + 0x17C)
#define MV_USB_CORE_CONFIG_FLAG_REG(dev)        (USB_REG_BASE(dev) + 0x180)
#define MV_USB_CORE_PORTSC_REG(dev)             (USB_REG_BASE(dev) + 0x184)
#define MV_USB_CORE_OTGSC_REG(dev)              (USB_REG_BASE(dev) + 0x1A4)

#define MV_USB_CORE_MODE_REG(dev)               (USB_REG_BASE(dev) + 0x1A8)

#define MV_USB_CORE_MODE_OFFSET                 0
#define MV_USB_CORE_MODE_MASK                   (3 << MV_USB_CORE_MODE_OFFSET)
#define MV_USB_CORE_MODE_HOST                   (3 << MV_USB_CORE_MODE_OFFSET)
#define MV_USB_CORE_MODE_DEVICE                 (2 << MV_USB_CORE_MODE_OFFSET)

/* Bit[2] (ES) - don't care */

#define MV_USB_CORE_SETUP_LOCK_DISABLE_BIT      3
#define MV_USB_CORE_SETUP_LOCK_DISABLE_MASK     (1 << MV_USB_CORE_SETUP_LOCK_DISABLE_BIT)

#define MV_USB_CORE_STREAM_DISABLE_BIT          4
#define MV_USB_CORE_STREAM_DISABLE_MASK         (1 << MV_USB_CORE_STREAM_DISABLE_BIT)


#define MV_USB_CORE_ENDPT_SETUP_STAT_REG(dev)    (USB_REG_BASE(dev) + 0x1AC)
#define MV_USB_CORE_ENDPT_PRIME_REG(dev)         (USB_REG_BASE(dev) + 0x1B0)
#define MV_USB_CORE_ENDPT_FLUSH_REG(dev)         (USB_REG_BASE(dev) + 0x1B4)
#define MV_USB_CORE_ENDPT_STATUS_REG(dev)        (USB_REG_BASE(dev) + 0x1B8)
#define MV_USB_CORE_ENDPT_COMPLETE_REG(dev)      (USB_REG_BASE(dev) + 0x1BC)
#define MV_USB_CORE_ENDPT_CTRL_REG(dev, ep)      (USB_REG_BASE(dev) + 0x1C0 + (ep*4))

/*******************************************/
/* USB Bridge Registers                    */
/*******************************************/
/*#define MV_USB_BRIDGE_CTRL_REG(dev)              (USB_REG_BASE(dev) + 0x300)
#define MV_USB_BRIDGE_CORE_BYTE_SWAP_OFFSET 4
#define MV_USB_BRIDGE_CORE_BYTE_SWAP_MASK   (1 << MV_USB_BRIDGE_CORE_BYTE_SWAP_OFFSET)
#define MV_USB_BRIDGE_CORE_BYTE_SWAP_EN     (0 << MV_USB_BRIDGE_CORE_BYTE_SWAP_OFFSET)

#define MV_USB_BRIDGE_INTR_CAUSE_REG(dev)        (USB_REG_BASE(dev) + 0x310)
#define MV_USB_BRIDGE_INTR_MASK_REG(dev)         (USB_REG_BASE(dev) + 0x314)
*/
#define MV_USB_MUX_CTRL_REG(dev)                 (USB_REG_BASE(dev) + 0x300)
#define MV_USB_INT_STATUS_REG(dev)               (USB_REG_BASE(dev) + 0x304)
#define MV_USB_INT_ENABLE_REG(dev)               (USB_REG_BASE(dev) + 0x308)

/* BITs in Interrupt Enable and Status registers */
#define MV_USB_INT_WKEN_BIT        0
#define MV_USB_INT_WKEN_MASK       (1<<MV_USB_INT_WKEN_BIT)

//#define MV_USB_BRIDGE_ERROR_ADDR_REG(dev)        (USB_REG_BASE(dev) + 0x31C)

#define MV_USB_WIN_CTRL_REG(dev, win)        (USB_REG_BASE(dev) + 0x320 + ((win)<<4)) 
#define MV_USB_WIN_BASE_REG(dev, win)        (USB_REG_BASE(dev) + 0x324 + ((win)<<4))

/* BITs in Windows 0-3 Control and Base Registers */
#if 0
#define MV_USB_WIN_ENABLE_BIT               0
#define MV_USB_WIN_ENABLE_MASK              (1 << MV_USB_WIN_ENABLE_BIT)

#define MV_USB_WIN_BURST_WR_LIMIT_BIT       1
#define MV_USB_WIN_BURST_WR_LIMIT_MASK      (1 << MV_USB_WIN_BURST_WR_LIMIT_BIT)
#define MV_USB_WIN_BURST_WR_NO_LIMIT        (0 << MV_USB_WIN_BURST_WR_LIMIT_BIT)
#define MV_USB_WIN_BURST_WR_32BIT_LIMIT     (1 << MV_USB_WIN_BURST_WR_LIMIT_BIT)

#define MV_USB_WIN_TARGET_OFFSET            4
#define MV_USB_WIN_TARGET_MASK              (0xF << MV_USB_WIN_TARGET_OFFSET)

#define MV_USB_WIN_ATTR_OFFSET              8
#define MV_USB_WIN_ATTR_MASK                (0xFF << MV_USB_WIN_ATTR_OFFSET)

#define MV_USB_WIN_SIZE_OFFSET              16
#define MV_USB_WIN_SIZE_MASK                (0xFFFF << MV_USB_WIN_SIZE_OFFSET)

#define MV_USB_WIN_BASE_OFFSET              16
#define MV_USB_WIN_BASE_MASK                (0xFFFF << MV_USB_WIN_BASE_OFFSET)


#define MV_USB_BRIDGE_IPG_REG(dev)          (USB_REG_BASE(dev) + 0x360)
#endif

/*******************************************/
/* Interrupt Controller Registers          */
/*******************************************/
/*#define USB_CAUSE_REG(dev)                  (USB_REG_BASE(dev) + 0x310)
#define USB_MASK_REG(dev)                   (USB_REG_BASE(dev) + 0x314)
#define USB_ERROR_ADDR_REG(dev)             (USB_REG_BASE(dev) + 0x31c)
*/

/*******************************************/
/* USB PHY Registers                       */
/*******************************************/

#define MV_USB_PHY_POWER_CTRL_REG(dev)          (__REG(0x42400000) + 0x4078)

#define MV_USB_PHY_PLL_CTRL_REG(dev)            (USB_PHY_REG_BASE(dev) + 0x000)

/*#define MV_USB_PHY_POWER_UP_BIT                 0
#define MV_USB_PHY_POWER_UP_MASK                (1<<MV_USB_PHY_POWER_UP_BIT)

#define MV_USB_PHY_PLL_POWER_UP_BIT             1
#define MV_USB_PHY_PLL_POWER_UP_MASK            (1<<MV_USB_PHY_PLL_POWER_UP_BIT)
*/
#endif

#include  "mvOs.h"
#if 0
typedef unsigned int	MV_U32;
typedef unsigned char	MV_U8;
#endif
typedef unsigned long	MV_ULONG;
typedef unsigned char	MV_BOOL;

#if 0
#define MV_USB_PHY_TX_CTRL_REG(dev)             (USB_PHY_REG_BASE(dev) + 0x004) 
#define MV_USB_PHY_RX_CTRL_REG(dev)             (USB_PHY_REG_BASE(dev) + 0x008) 
#define MV_USB_PHY_IVREF_CTRL_REG(dev)          (USB_PHY_REG_BASE(dev) + 0x00C) 
#define MV_USB_PHY_TEST_GROUP_CTRL_REG_0(dev)   (USB_PHY_REG_BASE(dev) + 0x010) 
#define MV_USB_PHY_TEST_GROUP_CTRL_REG_1(dev)   (USB_PHY_REG_BASE(dev) + 0x014) 
#define MV_USB_PHY_TEST_GROUP_CTRL_REG_2(dev)   (USB_PHY_REG_BASE(dev) + 0x018) 
#define MV_USB_PHY_TEST_GROUP_CTRL_REG_3(dev)   (USB_PHY_REG_BASE(dev) + 0x01C) 
#define MV_USB_PHY_TEST_GROUP_CTRL_REG_4(dev)   (USB_PHY_REG_BASE(dev) + 0x020) 
#define MV_USB_PHY_TEST_GROUP_CTRL_REG_5(dev)   (USB_PHY_REG_BASE(dev) + 0x024) 
#define MV_USB_PHY_ID_REG(dev)                  (USB_PHY_REG_BASE(dev) + 0x028) 
#define MV_USB_PHY_RESV_REG(dev)                (USB_PHY_REG_BASE(dev) + 0x02C) 
#define MV_USB_PHY_MONITOR_REG(dev)             (USB_PHY_REG_BASE(dev) + 0x030) 
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCmvUsbRegsh */

