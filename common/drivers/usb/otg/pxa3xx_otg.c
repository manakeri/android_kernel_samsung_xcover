/*
 * linux/drivers/usb/gadget/pxa_otg.c
 * PXA3xx usb otg controller
 *
 * Copyright (C) 2005 Intel Corporation
 * Copyright (C) 2007 Marvell International Ltd
 *
 * 2007.5.16	brown,mark	added support for otg carkit mode
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/usb/otg.h>

#include <asm/uaccess.h>
#include <asm/mach-types.h>

#include <mach/cputype.h>
#include <plat/vbus.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>
#include <asm/io.h>

#define DRIVER_DESC "USB OTG on Monahans"

#ifdef CONFIG_CPU_PXA310
extern struct otg_xceiv_ops *init_pxa310_otg_xceiv_ops(void);
extern struct otg_ctrl_ops *init_pxa310_otg_ctrl_ops(void);
#endif

#ifdef CONFIG_CPU_PXA300
extern struct otg_xceiv_ops *init_pxa300_otg_xceiv_ops(void);
extern struct otg_ctrl_ops *init_pxa300_otg_ctrl_ops(void);
#endif

void  pxa_otg_vbus_handle(int status);
static void pxa3xx_update_otg(struct pxa_otg *pxa_otg);
static void otg_a_idle(struct pxa_otg *pxa_otg);
static void otg_b_idle(struct pxa_otg *pxa_otg);
void pxa_otg_reset(void);

static char *state_string[] = {
	"undefined",
	"b_idle",
	"b_srp_init",
	"b_peripheral",
	"b_wait_acon",
	"b_host",
	"a_idle",
	"a_wait_vrise",
	"a_wait_bcon",
	"a_host",
	"a_suspend",
	"a_peripheral",
	"a_wait_vfall",
	"a_vbus_err"
};

inline void  pxa3xx_otg_require_bus(int status)
{
	pxa_otg_vbus_handle(status);
}
/******************************************************************
 * timer help functions
 ******************************************************************
 */
#define TIMER_NUM	5
static struct timer_list timer_array[TIMER_NUM];
static unsigned timer_state;

/* register a timer
 * parameters
 * pxa_otg - OTG handle
 * interval - delay of timer
 * timer_callback - callback function when timer expired
 */
int otg_register_timer(struct pxa_otg *pxa_otg, \
		       int interval, \
		       void (*timer_callback)(unsigned long))
{
	unsigned count, temp;
	int ret;
	struct timer_list *timer;

	/* find a free timer */
	ret = 0;
	count = TIMER_NUM;
	temp = timer_state;
	while ((temp & 0x1) && count) {
		temp = (temp >> 1);
		count-- ;
		ret++ ;
	}

	if (ret >= TIMER_NUM) {
		pr_err("No free timer\n");
		return -1;
	} else {
		timer = &timer_array[ret];
		timer_state |= (1 << ret) ;
	}

	init_timer(timer);
	timer->data = (unsigned long)pxa_otg;
	timer->function = timer_callback;
	timer->expires = jiffies + interval;
	add_timer(timer);

	return ret;
}

int otg_cancel_timer(int timer_id)
{
	if ((timer_id < 0) || (timer_id >= TIMER_NUM)
	    || !((1 << timer_id) & timer_state))
		return OTG_INVALID_PARAMETER;

	del_timer(&timer_array[timer_id]);
	timer_state &= ~(1 << timer_id);

	return 0;
}

/* Timer expired, call this functio to indicate the timer free
 */
int otg_timer_expired(int timer_id)
{
	if ((timer_id < 0) || (timer_id >= TIMER_NUM)
	    || !((1 << timer_id) & timer_state))
		return OTG_INVALID_PARAMETER;

	timer_state &= ~(1 << timer_id);

	return 0;
}

int otg_timer_reset(void)
{
	int i;

	for (i = 0; i < TIMER_NUM; i++)
		otg_cancel_timer(i);
	return 0;
}

/******************************************************************
 * OTG platform independent functions
 ******************************************************************
 */

/*
 *  Set or clear field a_set_b_hnp_en
 *  mode - the desired mode for USB host port2
 */
int otg_set_b_hnp(struct pxa_otg *pxa_otg, int set)
{
	if (set)
		pxa_otg->otg_ctrl->a_set_b_hnp_en = 1;
	else
		pxa_otg->otg_ctrl->a_set_b_hnp_en = 0;

	return 0;
}

/* called when timer a_aidl_bdis_tmr expires
 */
static void otg_timer_aidle_bdis(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	pxa_otg->otg_ctrl->a_aidl_bdis_timeout = 1;
	pxa3xx_update_otg(pxa_otg);
	otg_timer_expired(pxa_otg->otg_ctrl->a_aidl_bdis_tmr);
	pxa_otg->otg_ctrl->a_aidl_bdis_tmr = -1;
}

/* called when timer a_wait_bcon_tmr expires
 */
static void otg_timer_await_bcon(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	pxa_otg->otg_ctrl->a_wait_bcon_timeout = 1;
	pxa3xx_update_otg(pxa_otg);
	otg_timer_expired(pxa_otg->otg_ctrl->a_wait_bcon_tmr);
	pxa_otg->otg_ctrl->a_wait_bcon_tmr = -1;
	pr_info("B Device No Response!\n");
}

/* called when timer a_wait_vfall_tmr expires
 */
static void otg_timer_await_vfall(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	pxa_otg->otg_ctrl->a_wait_vfall_timeout = 1;
	pxa3xx_update_otg(pxa_otg);
	otg_timer_expired(pxa_otg->otg_ctrl->a_wait_vfall_tmr);
	pxa_otg->otg_ctrl->a_wait_vfall_tmr = -1;
	pr_info("VBUS always high!\n");
}

/* called when timer a_bidl_adis_tmr expires
 */
static void otg_timer_bidle_adis(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	pxa_otg->otg_ctrl->a_bidl_adis_timeout = 1;
	pxa3xx_update_otg(pxa_otg);
	otg_timer_expired(pxa_otg->otg_ctrl->a_bidl_adis_tmr);
	pxa_otg->otg_ctrl->a_bidl_adis_tmr = -1;
}

/* called when timer b_se0_srp_tmr expires
 */
static void otg_timer_se0_srp(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	otg_timer_expired(pxa_otg->otg_ctrl->b_se0_srp_tmr);
	pxa_otg->otg_ctrl->b_se0_srp_tmr = -1;
	pxa3xx_update_otg(pxa_otg);
}

/* called when timer b_srp_fail_tmr expires
 */
static void otg_timer_srp_fail(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	otg_timer_expired(pxa_otg->otg_ctrl->b_srp_fail_tmr);
	pxa_otg->otg_ctrl->b_srp_fail_tmr = -1;

	otg_timer_expired(pxa_otg->otg_ctrl->b_se0_srp_tmr);
	pxa_otg->otg_ctrl->b_se0_srp_tmr = -1;

	if (pxa_otg->otg_ctrl->b_bus_req == 1) {
		pr_info("A Device didn't response after sending Session!\n");
		pxa_otg->otg_ctrl->b_bus_req = 0;
	} else
		pr_info("A Device didn't response because the VBUS is low!\n");
}

/* called when timer b_ase0_brst_tmr expires */
static void otg_timer_base0_brst(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	pr_debug(" otg state %s\n",
		 state_string[pxa_otg->otg.state]);

	pxa_otg->otg_ctrl->b_ase0_brst_timeout = 1;
	pxa3xx_update_otg(pxa_otg);

	otg_timer_expired(pxa_otg->otg_ctrl->b_ase0_brst_tmr);
	pxa_otg->otg_ctrl->b_ase0_brst_tmr = -1;
	pr_info("As a client, A Device didn't response!\n");
}

static void otg_timer_drv_rsm(unsigned long data)
{
	struct pxa_otg *pxa_otg = (struct pxa_otg *)data;

	pxa_otg->xceiv_ops->otgx_set_mode(USB_INT_OTG_HOST_DP);
	usb_bus_start_enum(pxa_otg->otg.host, pxa_otg->otg.host->otg_port);
	pxa_otg->otg_ctrl->b_bus_resume = 1;
	pxa3xx_update_otg(pxa_otg);
	otg_timer_expired(pxa_otg->otg_ctrl->a_drv_rsm_tmr);
	pxa_otg->otg_ctrl->a_drv_rsm_tmr = -1;
}
/* called when initialize fields in handle
 */
static void otg_init_handle(struct pxa_otg *pxa_otg)
{
	/* Initialize internal variables*/
	pxa_otg->otg_ctrl->a_set_b_hnp_en = 0;
	pxa_otg->otg.gadget->a_hnp_support = 0;
	pxa_otg->otg.gadget->a_alt_hnp_support = 0;

	/* Initialize OTG inputs*/
	pxa_otg->otg_ctrl->a_bus_drop = 0;
	pxa_otg->otg_ctrl->a_bus_req = 0;
	pxa_otg->otg_ctrl->a_bus_suspend = 0;
	pxa_otg->otg_ctrl->a_vbus_vld = 0;
	pxa_otg->otg_ctrl->a_conn = 0;
	pxa_otg->otg_ctrl->b_bus_req = 0;
	pxa_otg->otg_ctrl->b_sess_vld = 0;
	pxa_otg->otg_ctrl->b_bus_suspend = 0;
	pxa_otg->otg_ctrl->b_bus_resume = 0;
	pxa_otg->otg_ctrl->b_conn = 0;
	pxa_otg->otg_ctrl->a_suspend_req = 0;

	/* Initialize timer identifiers */
	pxa_otg->otg_ctrl->a_wait_vrise_tmr = -1;
	pxa_otg->otg_ctrl->a_wait_vfall_tmr = -1;
	pxa_otg->otg_ctrl->a_wait_bcon_tmr = -1;
	pxa_otg->otg_ctrl->a_aidl_bdis_tmr = -1;
	pxa_otg->otg_ctrl->b_ase0_brst_tmr = -1;
	pxa_otg->otg_ctrl->b_se0_srp_tmr = -1;
	pxa_otg->otg_ctrl->b_srp_fail_tmr = -1;
	pxa_otg->otg_ctrl->a_srp_rspns_tmr = -1;
	pxa_otg->otg_ctrl->a_bidl_adis_tmr = -1;
	pxa_otg->otg_ctrl->a_drv_rsm_tmr = -1;

	otg_timer_reset();
}

/* called from irq handlers */
static void otg_a_idle(struct pxa_otg *pxa_otg)
{
	if (pxa_otg->otg.state == OTG_STATE_A_IDLE)
		return;
	otg_init_handle(pxa_otg);
	pxa_otg->otg.default_a = OTG_A_DEVICE;
	pxa_otg->otg.state = OTG_STATE_A_IDLE;

	/* disable the device controller */
	usb_gadget_vbus_disconnect(pxa_otg->otg.gadget);
	usb_gadget_disconnect(pxa_otg->otg.gadget);

	pr_debug("  --> %s\n", state_string[pxa_otg->otg.state]);
	pr_info("mini A plug in,"
	       " please require vbus or send SRP from B device!\n");

}

/* called from irq handlers */
static void otg_b_idle(struct pxa_otg *pxa_otg)
{
	if (pxa_otg->otg.state == OTG_STATE_B_IDLE)
		return;

// VBUS #ifdef CONFIG_PXA_VBUS //dh0318.lee remove unnecessary switch on USB driver for enabling USB VBUS
	/* Disable usb pump */
	pxa_set_vbus(VBUS_LOW, 0);
//#endif

	otg_init_handle(pxa_otg);
	pxa_otg->otg.default_a = OTG_B_DEVICE;
	pxa_otg->otg.state = OTG_STATE_B_IDLE;

	pr_debug("  --> %s\n", state_string[pxa_otg->otg.state]);

}

static void otg_a_wait_vfail(struct pxa_otg *pxa_otg)
{
// VBUS #ifdef CONFIG_PXA_VBUS
	/* Disable usb pump */
	pxa_set_vbus(VBUS_LOW, 0);
// #endif
	otg_cancel_timer(pxa_otg->otg_ctrl->a_wait_bcon_tmr);
	pxa_otg->otg_ctrl->a_wait_bcon_tmr = -1;
	usb_gadget_disconnect(pxa_otg->otg.gadget);
	usb_gadget_vbus_disconnect(pxa_otg->otg.gadget);
	pxa_otg->otg_ctrl->a_vbus_vld = 0;
}

static int otg_deinit(struct pxa_otg * pxa_otg)
{
	pxa_otg->ctrl_ops->otgc_deinit();
	return 0;
}

int otg_interrupt_handle(struct pxa_otg *pxa_otg)
{
	u16 interrupt_type;
	struct otg_xceiv_ops *xceiv_ops = pxa_otg->xceiv_ops;

	interrupt_type = pxa_otg->ctrl_ops->otgc_interrupt_handle(pxa_otg);

	pr_debug("\tinterrupt type:%x otg state %s\n",
		 interrupt_type, state_string[pxa_otg->otg.state]);


	/* mini-A is plugged in */
	if (interrupt_type & OTG_INT_IDF) {
		xceiv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
		pr_debug(" OTGID low, Mini-A plug in\n");
		otg_a_idle(pxa_otg);
		pxa_otg->otg_ctrl->a_bus_req = 1;
		pxa_otg->otg_ctrl->a_bus_drop = 0;
		pxa3xx_update_otg(pxa_otg);
	}

	/* mini-A is plugged out */
	if (interrupt_type & OTG_INT_IDR) {
/*	    && !pxa_otg->otg_ctrl->b_sess_vld) {
 *	    FIXME without it, maybe block the OPT */
		pr_debug(" OTGID high, Mini-A plug out\n");
		if (OTG_A_DEVICE == pxa_otg->otg.default_a) {
			if (xceiv_ops->otgx_check_vbus())
				xceiv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			else
				xceiv_ops->otgx_set_mode(USB_OTG_LP);
		}
		otg_b_idle(pxa_otg);
	}

	if (interrupt_type & OTG_INT_RSV)
		pxa_otg_vbus_handle(USBOTG_SRP_DETECT);

	if (interrupt_type & OTG_INT_RVV)
		pxa_otg_vbus_handle(USBOTG_VBUS_VALID);

	if (interrupt_type & OTG_INT_FVV)
		pxa_otg_vbus_handle(0);

	if (interrupt_type & OTG_INT_LS) {
		xceiv_ops->otgx_drive_resume();
		if (pxa_otg->otg_ctrl->b_se0_srp_tmr == -1) {
			pxa_otg->otg_ctrl->a_drv_rsm_tmr =
				otg_register_timer(pxa_otg, T_A_DRV_RSM,
						   otg_timer_drv_rsm);
		}
	}

	if (interrupt_type & OTG_INT_LP_DIS) {
		pxa_otg->otg_ctrl->b_conn = 0;
		pxa3xx_update_otg(pxa_otg);
		pxa_otg->udev = NULL;
	}

	return 0;
}

static int pxa_otg_start_srp(struct otg_transceiver *otg);
int otg_state_machine(struct pxa_otg *pxa_otg, int old_state)
{
	int timer_id;

	switch (old_state) {
	case OTG_STATE_UNDEFINED:
		break;
	case OTG_STATE_B_IDLE:
		/* If state is idle send SRP, tell host to wake up and
		 * power VBUS */
		if (pxa_otg->otg_ctrl->b_bus_req &&
		    !(pxa_otg->otg_ctrl->b_sess_vld)) {
			if (pxa_otg->otg_ctrl->b_se0_srp_tmr == -1) {
				pxa_otg->otg_ctrl->b_se0_srp_tmr =
					otg_register_timer(pxa_otg,
							   T_B_SE0_SRP,
							   otg_timer_se0_srp);
			}
			pxa_otg->otg.state = OTG_STATE_B_SRP_INIT;
			pxa_otg_start_srp(&(pxa_otg->otg));
			pxa_otg->otg.state = OTG_STATE_B_IDLE;
			if (pxa_otg->otg_ctrl->b_srp_fail_tmr == -1) {
				pxa_otg->otg_ctrl->b_srp_fail_tmr =
					otg_register_timer(pxa_otg,
							   T_B_SRP_FAIL,
							   otg_timer_srp_fail);
			}
		}

		if (pxa_otg->otg_ctrl->b_sess_vld) {
			pxa_otg->otg.state = OTG_STATE_B_PERIPHERAL;
			otg_cancel_timer(pxa_otg->otg_ctrl->b_se0_srp_tmr);
			pxa_otg->otg_ctrl->b_se0_srp_tmr = -1;
			otg_cancel_timer(pxa_otg->otg_ctrl->b_srp_fail_tmr);
			pxa_otg->otg_ctrl->b_srp_fail_tmr = -1;
		}
		break;
	case OTG_STATE_B_SRP_INIT:
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!(pxa_otg->otg_ctrl->b_sess_vld)) {
			pxa_otg->otg.state = OTG_STATE_B_IDLE;
			if (pxa_otg->otg_ctrl->b_srp_fail_tmr == -1) {
				pxa_otg->otg_ctrl->b_srp_fail_tmr =
					otg_register_timer(pxa_otg,
							   T_B_SRP_FAIL,
							   otg_timer_srp_fail);
			}
		}

		if ((pxa_otg->otg_ctrl->a_bus_suspend) &&
		    (pxa_otg->xceiv_ops->otgx_check_b_hnp())) {
			if (pxa_otg->otg_ctrl->b_bus_req) {
				pxa_otg->otg.state = OTG_STATE_B_WAIT_ACON;
				/* Start timer Tb_ase0_brst_tmr */
				if (pxa_otg->otg_ctrl->b_ase0_brst_tmr == -1) {
					pxa_otg->otg_ctrl->b_ase0_brst_tmr =
						otg_register_timer(pxa_otg,
							T_B_ASE0_BRST,
							otg_timer_base0_brst);
				}
			}
		}

		break;
	case OTG_STATE_B_WAIT_ACON:
		if (!(pxa_otg->otg_ctrl->b_sess_vld))
			pxa_otg->otg.state = OTG_STATE_B_IDLE;

		if (pxa_otg->otg_ctrl->b_ase0_brst_timeout)
			pxa_otg->otg.state = OTG_STATE_B_PERIPHERAL;

		if (pxa_otg->otg_ctrl->a_conn) {
			otg_cancel_timer(pxa_otg->otg_ctrl->b_ase0_brst_tmr);
			pxa_otg->otg_ctrl->b_ase0_brst_tmr = -1;
			pxa_otg->otg.state = OTG_STATE_B_HOST;
		}

		break;
	case OTG_STATE_B_HOST:
		if (!(pxa_otg->otg_ctrl->a_conn) && pxa_otg->otg.gadget)
			pxa_otg->otg.state = OTG_STATE_B_PERIPHERAL;

		if (!(pxa_otg->otg_ctrl->b_sess_vld))
			pxa_otg->otg.state = OTG_STATE_B_IDLE;
		break;
	case OTG_STATE_A_IDLE:
		if (!(pxa_otg->otg_ctrl->a_bus_drop) &&
		    (pxa_otg->otg_ctrl->a_bus_req ||
		     pxa_otg->otg_ctrl->a_srp_det))
			pxa_otg->otg.state = OTG_STATE_A_WAIT_VRISE;
		break;
	case OTG_STATE_A_WAIT_VRISE:
		if (pxa_otg->otg_ctrl->a_vbus_vld)
			pxa_otg->otg.state = OTG_STATE_A_WAIT_BCON;
		break;
	case OTG_STATE_A_WAIT_BCON:
		if (!(pxa_otg->otg_ctrl->a_vbus_vld)) {
			otg_cancel_timer(pxa_otg->otg_ctrl->a_wait_bcon_tmr);
			pxa_otg->otg_ctrl->a_wait_bcon_tmr = -1;
			pxa_otg->otg.state = OTG_STATE_A_VBUS_ERR;
		}

		if (pxa_otg->otg_ctrl->b_conn) {
			otg_cancel_timer(pxa_otg->otg_ctrl->a_wait_bcon_tmr);
			pxa_otg->otg_ctrl->a_wait_bcon_tmr = -1;
			pxa_otg->otg.state = OTG_STATE_A_HOST;
		}

		if (pxa_otg->otg_ctrl->a_wait_bcon_timeout) {
			pxa_otg->otg.state = OTG_STATE_A_WAIT_VFALL;
			pxa_otg->otg_ctrl->a_bus_req = 0;
		}
		break;
	case OTG_STATE_A_HOST:
		if (!(pxa_otg->otg_ctrl->a_vbus_vld))
			pxa_otg->otg.state = OTG_STATE_A_VBUS_ERR;

		if (pxa_otg->otg_ctrl->a_suspend_req) {
			pxa_otg->otg.state = OTG_STATE_A_SUSPEND;
			if ((pxa_otg->otg_ctrl->a_set_b_hnp_en)
			    && (pxa_otg->otg_ctrl->a_aidl_bdis_tmr == -1)) {
				timer_id = otg_register_timer(pxa_otg,
						T_A_AIDL_BDIS,
						otg_timer_aidle_bdis);
				if (timer_id < 0)
					break;

				pxa_otg->otg_ctrl->a_aidl_bdis_tmr = timer_id;
			}
		}

		if (!(pxa_otg->otg_ctrl->b_conn))
			pxa_otg->otg.state = OTG_STATE_A_WAIT_BCON;

		break;
	case OTG_STATE_A_SUSPEND:
		if (!(pxa_otg->otg_ctrl->a_vbus_vld))
			pxa_otg->otg.state = OTG_STATE_A_VBUS_ERR;

		else if (pxa_otg->otg_ctrl->b_bus_resume ||
			 pxa_otg->otg_ctrl->a_bus_req) {
			pxa_otg->otg.state = OTG_STATE_A_HOST;
			pxa_otg->xceiv_ops->otgx_set_mode(USB_INT_OTG_HOST_DP);
			usb_bus_start_enum(pxa_otg->otg.host,
					   pxa_otg->otg.host->otg_port);
		} else if (pxa_otg->otg_ctrl->a_aidl_bdis_timeout)
			pxa_otg->otg.state = OTG_STATE_A_WAIT_VFALL;

		else if (!(pxa_otg->otg_ctrl->b_conn)) {
			if (!pxa_otg->otg_ctrl->a_set_b_hnp_en)
				pxa_otg->otg.state = OTG_STATE_A_WAIT_BCON;
			else if (pxa_otg->otg_ctrl->a_bidl_adis_tmr == -1) {
				pxa_otg->otg.state = OTG_STATE_A_PERIPHERAL;
				timer_id = otg_register_timer(pxa_otg, \
					T_A_BIDL_ADIS, otg_timer_bidle_adis);
				if (timer_id < 0)
					break;
				pxa_otg->otg_ctrl->a_bidl_adis_tmr = timer_id;
			}
			otg_cancel_timer(pxa_otg->otg_ctrl->a_aidl_bdis_tmr);
			pxa_otg->otg_ctrl->a_aidl_bdis_tmr = -1;
		}

		break;
	case OTG_STATE_A_PERIPHERAL:
		if (!(pxa_otg->otg_ctrl->a_vbus_vld))
			pxa_otg->otg.state = OTG_STATE_A_VBUS_ERR;
#if 0
		if ((pxa_otg->otg_ctrl->b_bus_suspend))
			pxa_otg->otg.state = OTG_STATE_A_WAIT_BCON;
#endif
		if ((pxa_otg->otg_ctrl->b_conn)) {
			otg_cancel_timer(pxa_otg->otg_ctrl->a_bidl_adis_tmr);
			pxa_otg->otg_ctrl->a_bidl_adis_tmr = -1;
		}

		if (pxa_otg->otg_ctrl->a_bidl_adis_timeout)
			pxa_otg->otg.state = OTG_STATE_A_WAIT_BCON;

		break;
	case OTG_STATE_A_WAIT_VFALL:
		if (!(pxa_otg->otg_ctrl->b_conn) &&
			!pxa_otg->xceiv_ops->otgx_check_vbus())
			otg_cancel_timer(pxa_otg->otg_ctrl->a_wait_vfall_tmr);
			pxa_otg->otg_ctrl->a_wait_vfall_tmr = -1;
			pxa_otg->otg.state = OTG_STATE_A_IDLE;

		if (pxa_otg->otg_ctrl->a_bus_req)
			pxa_otg->otg.state = OTG_STATE_A_WAIT_VRISE;

		if (pxa_otg->otg_ctrl->a_wait_vfall_timeout) {
			pxa_otg->otg.state = OTG_STATE_A_IDLE;
		}
		break;
	case OTG_STATE_A_VBUS_ERR:
		if (pxa_otg->otg_ctrl->a_clr_err) {
			pxa_otg->otg_ctrl->a_clr_err = 0;
			pxa_otg->otg.state = OTG_STATE_A_WAIT_VFALL;
			if (!(pxa_otg->otg_ctrl->b_conn))
				pxa_otg->otg.state = OTG_STATE_A_IDLE;

			if (pxa_otg->otg_ctrl->a_bus_req)
				pxa_otg->otg.state = OTG_STATE_A_WAIT_VRISE;
		}
		break;
	default:
		break;
	}

	return pxa_otg->otg.state;
}

/******************************************************************
 * OTG interface functions
 ******************************************************************
 */
static struct pxa_otg *the_transceiver;

/*  When USB cable attached, call this function*/
void  pxa_otg_vbus_handle(int status)
{
	struct pxa_otg *pxa_otg;
	struct pxa_otgc *otg_ctrl;

	if (the_transceiver) {
		pxa_otg = the_transceiver;
		otg_ctrl = pxa_otg->otg_ctrl;
		pr_debug(" old otg state %s,  require %d\n",
			 state_string[pxa_otg->otg.state],  status);
		otg_ctrl->b_sess_vld = (status & USBOTG_VBUS_VALID) ? 1 : 0;
		otg_ctrl->a_vbus_vld = (status & USBOTG_VBUS_VALID) ? 1 : 0;
		if (pxa_otg->otg.state == OTG_STATE_A_IDLE)
			otg_ctrl->a_srp_det = (status & USBOTG_SRP_DETECT) ?
				1 : 0;

		pxa3xx_update_otg(pxa_otg);

		pr_debug(" current state %s\n",
			 state_string[pxa_otg->otg.state]);

	} else {
		printk(KERN_ERR "%s failed to get transceiver\n", __func__);
	}
}

/* Called from pxa3xx udc interrupt handler to deal with otg interrupt*/
static int pxa_otg_interrupt(struct otg_transceiver *otg)
{
	int old_default;
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver)
		return -ENODEV;

	old_default = pxa_otg->otg.default_a;
	otg_interrupt_handle(pxa_otg);

	/* This eradicate unnecessary suspend & reset interrupt */
	if (!old_default && pxa_otg->otg.default_a)
		pxa_otg->udev = NULL;
	else if (old_default && !pxa_otg->otg.default_a)
		pxa_otg->udev = NULL;

	return 0;
}

static int pxa_otginit(struct otg_transceiver *otg)
{
	enum otg_function function;
        struct pxa_otg *pxa_otg = container_of(otg, \
						struct pxa_otg, otg);

	pxa_otg->ctrl_ops->otgc_init(pxa_otg->info);
	/* Read the OTGID pin to detect whether there is a mini-A plug
	 * is alreadly inserted in receptacle.*/
	function = pxa_otg->xceiv_ops->otgx_detect_default_func();
	pr_info("otg default %s\n", function?"a device":"b device");

	otg_init_handle(pxa_otg);
	pxa_otg->ctrl_ops->otgc_interrupt_init();

//VBUS #ifdef CONFIG_PXA_VBUS
	pxa_prepare_vbus(1);
//#endif

	pxa_otg->otg.state = OTG_STATE_UNDEFINED;
	if (OTG_A_DEVICE == function){
		otg_a_idle(pxa_otg);
// VBUS #ifdef CONFIG_PXA_VBUS
		pxa_set_usbdev(USB_A_DEVICE);
//#endif
		otg_interrupt_handle(pxa_otg);
	}
	else if (OTG_B_DEVICE == function)
		otg_b_idle(pxa_otg);

	return 0;
}

static int pxa_otg_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver)
		return -ENODEV;

	pxa_otg->otg.host = host;

	return 0;
}

static int pxa_otg_set_peripheral(struct otg_transceiver *otg,
				  struct usb_gadget *gadget)
{
	unsigned long flags;
	int ret;
	struct pxa_otgc *otg_ctrl;
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver)
		return -ENODEV;

	if (gadget == NULL) {
		pxa_otg->ctrl_ops->otgc_init_gadget();
		pxa_otg->otg.gadget = NULL;
		otg_deinit(pxa_otg);
	} else {
		if (pxa_otg->otg.gadget)
			return 0;
		pxa_otg->otg.gadget = gadget;
		otg_ctrl = pxa_otg->otg_ctrl;
		local_irq_save(flags);
		ret = otg_init(otg);
		if (ret) {
			pr_err("%s: failed to call otg_init:%d\n",
			       __FUNCTION__, ret);
			local_irq_restore(flags);
			return -EFAULT;
		}
		local_irq_restore(flags);
	}
	return 0;
}

static int pxa_otg_set_power(struct otg_transceiver *otg, unsigned mA)
{
	return 0;
}

static int pxa_otg_start_srp(struct otg_transceiver *otg)
{
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver)
		return -ENODEV;

	pxa_otg->xceiv_ops->otgx_dp_session();
	pxa_otg->xceiv_ops->otgx_vbus_session(pxa_otg);

	return 0;
}

static int pxa_otg_start_hnp(struct otg_transceiver *otg)
{
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver)
		return -ENODEV;

	pr_debug("is called");
	/*otg_suspend_device*/
	pxa_otg->otg_ctrl->a_suspend_req = 1;
	pxa3xx_update_otg(pxa_otg);

	return 0;
}

static int pxa_otg_connect(struct otg_transceiver *otg, struct usb_device *udev)
{
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver)
		return -ENODEV;

	pr_debug("is called");
	/*otg_respond_connect*/
	if (!pxa_otg->otg_ctrl->a_conn && !pxa_otg->otg_ctrl->b_conn) {
		pxa_otg->otg_ctrl->a_conn = 1;
		pxa_otg->otg_ctrl->b_conn = 1;
		pxa3xx_update_otg(pxa_otg);
	}

	pr_debug("otg state:%s\n", state_string[pxa_otg->otg.state]);
	if (udev)
		pxa_otg->udev = udev;
	return 0;
}

static int pxa_otg_disconnect(struct otg_transceiver *otg)
{
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver || !the_transceiver)
		return -ENODEV;

	if (pxa_otg->otg.state == OTG_STATE_A_SUSPEND)
		return 0;

	pr_debug("%s is called, state:%s\n", __func__,
		 state_string[pxa_otg->otg.state]);
	/*otg_respond_disconnect*/

	if (pxa_otg->otg_ctrl->a_conn || pxa_otg->otg_ctrl->b_conn) {
		pxa_otg->otg_ctrl->a_conn = 0;
		pxa_otg->otg_ctrl->b_conn = 0;
		pxa3xx_update_otg(pxa_otg);
	}

	pxa_otg->udev = NULL;

	pr_debug("otg state:%s\n", state_string[pxa_otg->otg.state]);
	return 0;
}

static int pxa_otg_host_suspend(struct otg_transceiver *otg)
{
	struct pxa_otg *pxa_otg = container_of(otg, \
					       struct pxa_otg, otg);

	if (!otg || pxa_otg != the_transceiver)
		return -ENODEV;

	pr_debug("%s otg state before:%s\n", __func__,
		 state_string[pxa_otg->otg.state]);

	/*otg_respond_suspend*/
	pxa_otg->otg_ctrl->b_bus_suspend = 1;
	pxa_otg->otg_ctrl->a_bus_suspend = 1;
	pxa3xx_update_otg(pxa_otg);


	pr_debug("otg state after:%s\n", state_string[pxa_otg->otg.state]);
	return 0;
}

static void pxa3xx_update_otg(struct pxa_otg *pxa_otg)
{
	int state = pxa_otg->otg.state;
	int timer_id;
	struct otg_xceiv_ops *xceiv_ops = pxa_otg->xceiv_ops;

	if (pxa_otg->otg.host && pxa_otg->otg.host->b_hnp_enable)
		otg_set_b_hnp(pxa_otg, 1);

	otg_state_machine(pxa_otg, state);

	/* change some inputs' state*/
	pxa_otg->otg_ctrl->b_bus_suspend = 0;
	pxa_otg->otg_ctrl->a_bus_suspend = 0;
	pxa_otg->otg_ctrl->a_srp_det = 0;

	pxa_otg->otg_ctrl->b_ase0_brst_timeout = 0;
	pxa_otg->otg_ctrl->a_aidl_bdis_timeout = 0;
	pxa_otg->otg_ctrl->a_bidl_adis_timeout = 0;
	pxa_otg->otg_ctrl->a_wait_bcon_timeout = 0;

	if ((state != pxa_otg->otg.state)) {
		switch (pxa_otg->otg.state) {
		case OTG_STATE_B_IDLE:
			if (pxa_otg->otg.gadget) {
				usb_gadget_disconnect(pxa_otg->otg.gadget);
				usb_gadget_vbus_disconnect(pxa_otg->otg.gadget);
			}
			if (pxa_otg->otg_ctrl->b_sess_vld)
				xceiv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			else
				xceiv_ops->otgx_set_mode(USB_OTG_LP);
			pxa_otg->otg_ctrl->b_bus_req = 0;
			if (pxa_otg->otg.host)
				pxa_otg->otg.host->is_b_host = 0;
			break;
		case OTG_STATE_B_SRP_INIT:
			break;
		case OTG_STATE_B_PERIPHERAL:
			if (pxa_otg->otg.host)
				pxa_otg->otg.host->is_b_host = 0;
			pxa_otg->otg_ctrl->b_bus_req = 1;
			xceiv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			if (pxa_otg->otg.gadget) {
				usb_gadget_connect(pxa_otg->otg.gadget);
				usb_gadget_vbus_connect(pxa_otg->otg.gadget);
			}
			break;
		case OTG_STATE_B_WAIT_ACON:
			if (pxa_otg->otg.host)
				pxa_otg->otg.host->is_b_host = 1;
			if (pxa_otg->otg.gadget) {
				usb_gadget_disconnect(pxa_otg->otg.gadget);
				usb_gadget_vbus_disconnect(pxa_otg->otg.gadget);
			}
			xceiv_ops->otgx_set_mode(USB_INT_OTG_HOST_DP);
			break;
		case OTG_STATE_B_HOST:
			if (pxa_otg->otg.host){
				usb_bus_start_enum(pxa_otg->otg.host, pxa_otg->otg.host->otg_port);
				pxa_otg->otg.host->is_b_host = 1;
			}
			break;
		case OTG_STATE_A_IDLE:
			if (pxa_otg->otg.gadget) {
				pxa_otg->otg.gadget->is_a_peripheral = 0;
				usb_gadget_disconnect(pxa_otg->otg.gadget);
				usb_gadget_vbus_disconnect(pxa_otg->otg.gadget);
			}
			otg_init_handle(pxa_otg);
			xceiv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			break;
		case OTG_STATE_A_WAIT_VRISE:
// VBUS #ifdef CONFIG_PXA_VBUS
			/* Enable USB PUMP */
			pxa_set_vbus(VBUS_HIGH, 0);
// #endif
			if (xceiv_ops->otgx_check_vbus())
				pxa_otg_vbus_handle(USBOTG_VBUS_VALID);
			break;
		case OTG_STATE_A_WAIT_BCON:
			if (pxa_otg->otg.gadget) {
				pxa_otg->otg.gadget->is_a_peripheral = 0;
				usb_gadget_disconnect(pxa_otg->otg.gadget);
				/* remove reduntant vbus disconnect,
				 * already called in A_IDLE mode
				usb_gadget_vbus_disconnect(pxa_otg->otg.gadget);
				 */
			}
			otg_set_b_hnp(pxa_otg, 0);
			if (state != OTG_STATE_A_HOST)
				xceiv_ops->otgx_set_mode(USB_INT_OTG_HOST_DP);
			if (pxa_otg->otg_ctrl->a_wait_bcon_tmr == -1) {
				timer_id = otg_register_timer(pxa_otg,
					T_A_WAIT_BCON * 10000, otg_timer_await_bcon);
				if (timer_id < 0)
					break; /* return OTG_TIMER_FAILED; */

				pxa_otg->otg_ctrl->a_wait_bcon_tmr = timer_id;
			}
			break;
		case OTG_STATE_A_HOST:
			break;
		case OTG_STATE_A_SUSPEND:
			xceiv_ops->otgx_start_autoresume();
			break;
		case OTG_STATE_A_PERIPHERAL:
			xceiv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			if (pxa_otg->otg.gadget) {
				pxa_otg->otg.gadget->is_a_peripheral = 1;
				usb_gadget_vbus_connect(pxa_otg->otg.gadget);
				usb_gadget_connect(pxa_otg->otg.gadget);
			}
			break;
		case OTG_STATE_A_WAIT_VFALL:
			if (pxa_otg->otg.gadget)
				pxa_otg->otg.gadget->is_a_peripheral = 0;
			xceiv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			xceiv_ops->otgx_set_mode(USB_OTG_LP);
			otg_a_wait_vfail(pxa_otg);
			if (pxa_otg->otg_ctrl->a_wait_vfall_tmr == -1) {
				timer_id = otg_register_timer(pxa_otg,
					T_SSEND_LKG, otg_timer_await_vfall);
				if (timer_id < 0)
					break; /* return OTG_TIMER_FAILED; */

				pxa_otg->otg_ctrl->a_wait_vfall_tmr = timer_id;
			}
			break;
		case OTG_STATE_A_VBUS_ERR:
// VBUS #ifdef CONFIG_PXA_VBUS
			pxa_set_vbus(VBUS_LOW, 0);
// #endif
			pr_info("VBUS ERR, please clear this err!\n");
			break;
		default:
			break;
		}
	}
	pr_debug("%s:from %s to %s\n", __func__, state_string[state],
		 state_string[pxa_otg->otg.state]);
}

#ifdef CONFIG_PROC_FS
static const char *proc_node_name = "driver/otg";

static int otg_proc_read(char *page, char **start, off_t off, int count,
			 int *eof, void *_dev)
{
	struct pxa_otg	*pxa_otg = _dev;
	char			*next = page;
	unsigned size = count;
	unsigned long flags;
	int t;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n");
	size -= t;
	next += t;

	t = scnprintf(next, size, "otg state:%s bus required:%d\n",
		      state_string[pxa_otg->otg.state],
		      ((pxa_otg->otg.default_a == OTG_B_DEVICE) ?
		       pxa_otg->otg_ctrl->b_bus_req
		       : pxa_otg->otg_ctrl->a_bus_req));

	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

extern void dump_ulpi_regs(void);
extern int usb_suspend_both(struct usb_device *udev, pm_message_t msg);
extern int usb_resume_both(struct usb_device *udev);
static int otg_proc_write(struct file *filp, const char *buffer,
			  unsigned long count, void *data)
{
	char kbuf[8];
	int index;
	struct pxa_otg	*pxa_otg = data;
	struct otg_transceiver otg = pxa_otg->otg;
	static int force_a_device;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
	case 2:
		/* force to configure as A device */
		printk("otg is forced to set as A device\n");
		otg_a_idle(pxa_otg);
		force_a_device = 1;
		/* fall through */
	case 1:
		if (rely_on_vbus && !force_a_device)
			/* otg may not inited here */
			otg_init(&otg);

		if (pxa_otg->otg.default_a == OTG_A_DEVICE) {
			pxa_otg->otg_ctrl->a_bus_req = 1;
			pxa_otg->otg_ctrl->a_bus_drop = 0;
		} else
			pxa_otg->otg_ctrl->b_bus_req = 1;
		pxa3xx_update_otg(pxa_otg);

		/* After complete use usb device. Suspend the device connected */
		break;

	case 3:
		if (pxa_otg->otg.default_a == OTG_A_DEVICE) {
// VBUS #ifdef CONFIG_PXA_VBUS
			pxa_prepare_vbus(0);
// #endif
			pxa_otg->otg_ctrl->a_bus_drop = 1;
			pxa_otg->otg_ctrl->a_bus_req = 0;
			if (pxa_otg->otg_ctrl->a_vbus_vld) {
				pxa_otg->otg.state = OTG_STATE_A_WAIT_VFALL;
				otg_a_wait_vfail(pxa_otg);
#ifdef CONFIG_USB_PXA_U2O
				//u2o_clear(base, U2xPORTSC, ~U2xPORTSC_PP);
#endif
			}
			pxa_otg->otg_ctrl->a_bus_drop = 0;
			mdelay(200);
			if (!pxa_otg->xceiv_ops->otgx_check_vbus()) {
				pxa_otg->otg_ctrl->a_vbus_vld = 0;
				otg_a_idle(pxa_otg);
				pxa_otg->xceiv_ops->otgx_set_mode(USB_OTG_LP);
			}
// VBUS #ifdef CONFIG_PXA_VBUS
			pxa_prepare_vbus(1);
// #endif
		} else {
			pxa_otg->otg_ctrl->b_bus_req = 0;
			pxa3xx_update_otg(pxa_otg);
		}
		/* force A device, USBID won't change in this case */
		if (force_a_device) {
			/* detect actual USBID level and reset state machine */
			otg_init(&otg);
			force_a_device = 0;
		}
		break;
	case 4:
		if (pxa_otg->otg.state == OTG_STATE_A_VBUS_ERR) {
			pxa_otg->otg_ctrl->a_clr_err = 1;
			pxa3xx_update_otg(pxa_otg);
		}
		break;

	case 5:
// VBUS #ifdef CONFIG_PXA_VBUS
		pxa_set_vbus(VBUS_HIGH, 0);
// #endif
		break;
	case 6:
// VBUS #ifdef CONFIG_PXA_VBUS
		pxa_set_vbus(VBUS_LOW, 0);
// #endif
		break;
	default:
		return -EINVAL;
	}
	return count;
}

#define create_proc_files() \
	do {	struct proc_dir_entry *ent; \
		ent = create_proc_entry(proc_node_name, 0, NULL); \
		if (ent) { \
			ent->data = pxa_otg; \
			ent->read_proc = otg_proc_read; \
			ent->write_proc = otg_proc_write; \
		} \
	} while (0);
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)
#else
#define create_proc_files() do { } while (0)
#define remove_proc_files() do { } while (0)
#endif

static int pxa_otg_probe(struct platform_device *pdev)
{
	struct pxa_otg *pxa_otg;
	int status;

	if (the_transceiver)
		return 0;

	pxa_otg = kzalloc(sizeof(struct pxa_otg), GFP_KERNEL);
	if (!pxa_otg) {
		pr_err("%s: failed to allocate memory!\n",
		       __FUNCTION__);
		return -ENOMEM;
	}
	/* Initialize filed of otg */
	pxa_otg->otg.dev = &pdev->dev;
	pxa_otg->otg.label = "pxa3xx-otg";
	pxa_otg->otg.init = pxa_otginit;
	pxa_otg->otg.set_host = pxa_otg_set_host;
	pxa_otg->otg.set_peripheral = pxa_otg_set_peripheral;
	pxa_otg->otg.set_power = pxa_otg_set_power;
	pxa_otg->otg.start_srp = pxa_otg_start_srp;
	pxa_otg->otg.start_hnp = pxa_otg_start_hnp;
	pxa_otg->otg.disconnect = pxa_otg_disconnect;
	pxa_otg->otg.connect = pxa_otg_connect;
	pxa_otg->otg.host_suspend = pxa_otg_host_suspend;
	pxa_otg->otg.state = OTG_STATE_UNDEFINED;
	pxa_otg->otg.otg_interrupt = pxa_otg_interrupt;
	pxa_otg->udev = NULL;

#ifdef CONFIG_USB_PXA_U2O
	pxa_otg->xceiv_ops = init_pxa9xx_otg_xceiv_ops();
	pxa_otg->ctrl_ops = init_pxa9xx_otg_ctrl_ops();
#endif

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	if (cpu_is_pxa310() || cpu_is_pxa970()) {
		pxa_otg->xceiv_ops = init_pxa310_otg_xceiv_ops();
		pxa_otg->ctrl_ops = init_pxa310_otg_ctrl_ops();
	} else
#endif
	{
#if defined(CONFIG_USB_OTG_PXA3xx_UDC)
		pxa_otg->xceiv_ops = init_pxa300_otg_xceiv_ops();
		pxa_otg->ctrl_ops = init_pxa300_otg_ctrl_ops();
#endif
	}

	if (!pxa_otg->xceiv_ops || !pxa_otg->ctrl_ops) {
		pr_err("%s: failed to set ops for ctrl_ops or xceiv_ops!\n",
		       __FUNCTION__);
		kfree(pxa_otg);
		return -EINVAL;
	}
	/* Allocate & initialize field of otg_ctrl */
	pxa_otg->otg_ctrl = kzalloc(sizeof(struct pxa_otgc), GFP_KERNEL);
	if (!pxa_otg->otg_ctrl) {
		pr_err("%s: failed to allocate memory for otg_ctrl!\n",
		       __FUNCTION__);
		kfree(pxa_otg);
		return -ENOMEM;
	}

	the_transceiver = pxa_otg;

	pxa_otg->info = pdev->dev.platform_data;

	if (pxa_otg->info) {
		struct pxa_usb_plat_info *info;
		struct resource	    *res;

		info = (struct pxa_usb_plat_info *)pxa_otg->info;
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "u2o");
		if(!res){
			printk(KERN_ERR "res is NULL\n");
			return -ENOMEM;
		}
		else{
			info->regbase = (unsigned int) ioremap_nocache(res->start, res_size(res));
		}
		if (!info->regbase) {
			printk(KERN_ERR "Cannot get regbase 0x%x\n",
					info->regbase);
			return -ENOMEM;
		}
		printk("u2otg regbase 0x%x res->start %x pxa_otg->info %p\n",
				info->regbase, res->start, pxa_otg->info);

	}

	status = otg_set_transceiver(&pxa_otg->otg);
	if (status < 0)
		dev_err(&pdev->dev, "can't register transceiver, %d\n",
			status);

	platform_set_drvdata(pdev, pxa_otg);

	create_proc_files();
	return 0;
}

static int pxa_otg_remove(struct platform_device *pdev)
{
	struct pxa_otg *pxa_otg = platform_get_drvdata(pdev);

	remove_proc_files();

	pxa_otg->ctrl_ops->otgc_deinit();
	kfree(pxa_otg->otg_ctrl);
	kfree(pxa_otg);
	the_transceiver = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int pxa_otg_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int pxa_otg_resume(struct platform_device *dev)
{
	return 0;
}
#endif

static struct platform_driver pxa_otg_driver = {
	.driver         = {
		.name   = "pxa-otg",
	},
	.probe		= pxa_otg_probe,
	.remove		= pxa_otg_remove,
#ifdef CONFIG_PM
	.suspend        = pxa_otg_suspend,
	.resume		= pxa_otg_resume,
#endif
};

static int __init pxa_otg_init(void)
{
	return platform_driver_register(&pxa_otg_driver);
}

static void __exit pxa_otg_exit(void)
{
	if (the_transceiver)
		otg_set_transceiver(NULL);
	platform_driver_unregister(&pxa_otg_driver);
}

module_init(pxa_otg_init);
module_exit(pxa_otg_exit);
MODULE_LICENSE("GPL");

