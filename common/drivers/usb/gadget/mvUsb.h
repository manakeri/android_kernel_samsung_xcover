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

#ifndef __INCmvUsbh
#define __INCmvUsbh

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef CONFIG_USB_COMPOSITE
#include <plat/pxausb_comp.h>
#endif
/* includes */
#include "mvUsbRegs.h"
//#include "mvCtrlEnvLib.h"
//#include "mvCtrlEnvAddrDec.h"
#include <linux/mfd/88pm860x.h>

struct mv_usb_dev;

struct mv_usb_ep 
{
    struct usb_ep       ep;
    struct mv_usb_dev*  usb_dev;
    struct list_head    req_list;
    spinlock_t          req_lock;
    unsigned            num : 8,
			processing : 1,
#ifdef CONFIG_USB_COMPOSITE
			assigned : 1,
#endif
    			ack_sent : 1,
			ack_recv : 1,
                        is_enabled : 1,
                        is_in : 1;
};

struct mv_usb_request
{
	struct usb_request req;
	uint_32 alloc_buffer;
	void *align_buffer;
	unsigned align_length;
};

struct mv_usb_dev 
{
    /* each pci device provides one gadget, several endpoints */
    struct usb_gadget           gadget;
    spinlock_t                  lock;
    struct usb_gadget_driver    *driver;
    struct mv_usb_ep            ep[2*ARC_USB_MAX_ENDPOINTS];
    unsigned                    enabled : 1,
                                protocol_stall : 1,
                                got_irq : 1;
    u16                         chiprev;
    struct device               *dev; 
    void*                       mv_usb_handle;
    int                         dev_no;
    u8                          vbus_gpp_no;
#if defined(CONFIG_PLAT_PXA)
    struct pxa_usb_plat_info 	*info;
    struct clk			*clk;
    unsigned int __iomem	regbase;
    unsigned int __iomem	phybase;
    struct clk			*island_clk;
    struct clk			*bus_clk;
    struct pxa3xx_comp		cdev;
    struct work_struct		work;
    struct timer_list		timer;
#ifdef CONFIG_USB_OTG
    struct otg_transceiver	*transceiver;
#endif
    int				in_single;
    int				out_single;
#endif
	void ( *set_charger_type)(enum enum_charger_type type);
};

/* Functions */
//MV_STATUS   mvUsbInit(int dev, MV_BOOL isHost);
//static MV_U32      mvUsbGetCapRegAddr(int devNo);
void        mvUsbDevResetComplete(int devNo);

#ifdef MV_USB_VOLTAGE_FIX
MV_U8       mvUsbGppInit(int dev);
int         mvUsbBackVoltageUpdate(int dev, MV_U8 gppNo);
#endif /* MV_USB_VOLTAGE_FIX */

/* Power management routines */
void        mvUsbPowerDown(int port);
void        mvUsbPowerUp(int port);

/*
MV_STATUS   mvUsbWinInit(int dev);
MV_STATUS   mvUsbWinSet(int dev, MV_U32 winNum, MV_DEC_WIN *pAddrWin);
MV_STATUS   mvUsbWinGet(int dev, MV_U32 winNum, MV_DEC_WIN *pAddrWin);
*/

void        mvUsbAddrDecShow(void);
void        mvUsbRegs(int dev);
void        mvUsbCoreRegs(int dev, int isHost);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCmvUsbh */

